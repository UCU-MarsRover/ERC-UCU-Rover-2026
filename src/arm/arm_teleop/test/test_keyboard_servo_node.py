"""Gripper sync/gating/limits, safe-pose gripper reset, and panel-align memory.

Internal methods (_on_joint_state, _publish_gripper, _handle_safe_pose,
_check_panel_visibility, ...) are called directly rather than round-tripped
through real pub/sub or a running Gazebo/controller_manager, since the
behavior under test lives entirely in this node's own state machine —
feeding it through the real /joint_states topic would also reintroduce a
genuine flakiness trap found while verifying this by hand: against a busy
topic (e.g. a live sim publishing at 100Hz), rclpy.spin_once can starve
this node's own publish timer indefinitely and report a false failure.
"""
import contextlib
import json
import pathlib
import time

import pytest
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import Constraints
from rclpy.duration import Duration
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState, Joy

from arm_teleop.keyboard_servo_node import (
    GRIPPER_JOINT_NAME,
    HOME_POSE_JOINTS,
    GamepadInputLoop,
    KeyboardInputLoop,
    ServoController,
    ecodes,
)


@pytest.fixture
def controller():
    node = ServoController()
    yield node
    node.destroy_node()


def make_joint_state(position, name=GRIPPER_JOINT_NAME):
    msg = JointState()
    msg.name = [name]
    msg.position = [position]
    return msg


def make_arm_joint_state(values):
    """A /joint_states message covering all 6 HOME_POSE_JOINTS."""
    msg = JointState()
    msg.name = list(HOME_POSE_JOINTS)
    msg.position = list(values)
    return msg


def make_joy(axes=None, buttons=None):
    msg = Joy()
    # index: 0/1 left stick, 2/3 right stick, 4/5 L2/R2; wide enough to
    # cover the panel buttons (11/12) without every caller specifying them.
    msg.axes = axes if axes is not None else [0.0, 0.0, 0.0, 0.0, 1.0, 1.0]
    msg.buttons = buttons if buttons is not None else [0] * 13
    return msg


def publish_ticks(controller, n=5, dt=0.05):
    """Call _publish_gripper() n times with a fixed, simulated tick interval.

    The very first call after a velocity is set never moves anything (no
    prior tick to diff a dt against — see _publish_gripper), so this backdates
    _last_gripper_tick_time by exactly ``dt`` before every later call instead
    of relying on the real (near-zero, runner-speed-dependent) wall-clock gap
    between back-to-back calls in a tight loop — otherwise this is flaky:
    locally the gap may happen to accumulate enough dt to reach the clamp,
    but on a faster/slower CI runner it may not.
    """
    for _ in range(n):
        if controller._last_gripper_tick_time is not None:
            controller._last_gripper_tick_time = time.monotonic() - dt
        controller._publish_gripper()


class _FakeFuture:
    """Stand-in for an rclpy Future that resolves synchronously."""

    def __init__(self, result):
        self._result = result

    def result(self):
        return self._result

    def add_done_callback(self, cb):
        cb(self)


# ── gripper sync (restart while partially/fully open) ──────────────────────

def test_gripper_starts_unsynced(controller):
    assert controller._gripper_state_received is False
    assert controller._gripper_position == 0.0


def test_gripper_syncs_from_first_joint_state(controller):
    controller._on_joint_state(make_joint_state(0.008))
    assert controller._gripper_state_received is True
    assert controller._gripper_position == pytest.approx(0.008)


def test_gripper_ignores_state_after_first_sync(controller):
    controller._on_joint_state(make_joint_state(0.008))
    controller._gripper_position = 0.003  # simulate an in-progress move
    controller._on_joint_state(make_joint_state(0.011))
    assert controller._gripper_position == pytest.approx(0.003)


def test_gripper_state_without_gripper_joint_is_ignored(controller):
    controller._on_joint_state(make_joint_state(1.0, name='some_other_joint'))
    assert controller._gripper_state_received is False


# ── commands withheld before sync (the restart-mid-open scenario) ──────────

def test_gripper_commands_withheld_before_sync(controller, monkeypatch):
    right_seen = []
    left_seen = []
    monkeypatch.setattr(controller._gripper_right_pub, 'publish', right_seen.append)
    monkeypatch.setattr(controller._gripper_left_pub, 'publish', left_seen.append)

    controller.set_gripper_velocity(-0.006)
    publish_ticks(controller)

    assert right_seen == []
    assert left_seen == []
    assert controller._gripper_position == 0.0  # never moved: no sync yet


def test_gripper_publishes_once_synced(controller, monkeypatch):
    right_seen = []
    monkeypatch.setattr(controller._gripper_right_pub, 'publish', right_seen.append)

    controller._on_joint_state(make_joint_state(0.008))
    controller._publish_gripper()

    assert right_seen


def test_gripper_first_tick_after_sync_holds_instead_of_jumping(controller):
    controller._on_joint_state(make_joint_state(0.008))
    controller.set_gripper_velocity(-0.006)
    controller._publish_gripper()
    # No prior tick to diff a dt against yet, so this tick only records the
    # timestamp — it does not guess a dt and move (that would risk a jump
    # from the synced value on a slow first tick).
    assert controller._gripper_position == pytest.approx(0.008)


def test_gripper_moves_gradually_from_synced_value_not_from_zero(controller):
    controller._on_joint_state(make_joint_state(0.008))
    controller.set_gripper_velocity(-0.006)
    controller._publish_gripper()  # establishes _last_gripper_tick_time
    controller._last_gripper_tick_time = time.monotonic() - 0.1
    controller._publish_gripper()  # dt ~= 0.1s
    assert controller._gripper_position == pytest.approx(0.008 - 0.006 * 0.1, abs=1e-3)


# ── limits ──────────────────────────────────────────────────────────────

def test_gripper_position_clamps_to_stroke_bounds(controller):
    controller._on_joint_state(make_joint_state(0.006))
    controller.set_gripper_velocity(-10.0)  # absurdly fast close
    publish_ticks(controller)
    assert controller._gripper_position == 0.0

    controller.set_gripper_velocity(10.0)  # absurdly fast open
    publish_ticks(controller)
    assert controller._gripper_position == pytest.approx(controller._gripper_stroke)


def test_gripper_stroke_is_configurable(controller):
    controller._gripper_stroke = 0.02  # simulate a different bringup's parameter
    controller._on_joint_state(make_joint_state(0.0))
    controller.set_gripper_velocity(10.0)
    publish_ticks(controller)
    assert controller._gripper_position == pytest.approx(0.02)


def test_gripper_left_command_is_negated_right(controller, monkeypatch):
    right_seen = []
    left_seen = []
    monkeypatch.setattr(controller._gripper_right_pub, 'publish', right_seen.append)
    monkeypatch.setattr(controller._gripper_left_pub, 'publish', left_seen.append)

    controller._on_joint_state(make_joint_state(0.006))
    controller._publish_gripper()

    assert right_seen and left_seen
    assert list(left_seen[0].data) == [-right_seen[0].data[0]]


# ── keyboard: gripper key combination -> velocity ───────────────────────

def test_keyboard_open_key_sets_positive_velocity(controller):
    loop = KeyboardInputLoop(controller)
    loop._gripper_pressed.add(ecodes.KEY_B)
    loop._recompute_gripper_velocity()
    assert controller.gripper_vel == pytest.approx(controller.gripper_speed)


def test_keyboard_close_key_sets_negative_velocity(controller):
    loop = KeyboardInputLoop(controller)
    loop._gripper_pressed.add(ecodes.KEY_V)
    loop._recompute_gripper_velocity()
    assert controller.gripper_vel == pytest.approx(-controller.gripper_speed)


def test_keyboard_both_gripper_keys_cancel_out(controller):
    loop = KeyboardInputLoop(controller)
    loop._gripper_pressed.add(ecodes.KEY_B)
    loop._gripper_pressed.add(ecodes.KEY_V)
    loop._recompute_gripper_velocity()
    assert controller.gripper_vel == 0.0


def test_stop_zeroes_gripper_velocity(controller):
    controller.set_gripper_velocity(0.7)
    controller.stop()
    assert controller.gripper_vel == 0.0


def test_keyboard_safe_pose_clears_gripper_state(controller, monkeypatch):
    loop = KeyboardInputLoop(controller)
    monkeypatch.setattr(controller, 'move_to_safe_pose', lambda: True)
    monkeypatch.setattr(controller, 'start_servo', lambda: True)
    controller.set_gripper_velocity(0.5)
    loop._gripper_pressed.add(30)  # arbitrary key code standing in for 'b'/'v'

    loop._handle_safe_pose()

    assert loop._gripper_pressed == set()
    assert controller.gripper_vel == 0.0


# ── panel visibility / remembered-position state ────────────────────────

def test_is_panel_visible_false_initially(controller):
    assert controller.is_panel_visible() is False


def test_is_panel_visible_true_after_pose(controller):
    controller._on_panel_pose(PoseStamped())
    assert controller.is_panel_visible() is True


def test_is_panel_visible_false_once_stale(controller):
    controller._on_panel_pose(PoseStamped())
    controller._last_panel_visible_time = (
        controller.get_clock().now()
        - Duration(seconds=controller._panel_visible_max_age_sec + 1.0)
    )
    assert controller.is_panel_visible() is False


def test_has_remembered_panel_position_reflects_state(controller):
    assert controller.has_remembered_panel_position is False
    controller._panel_align_succeeded_once = True
    assert controller.has_remembered_panel_position is True


# ── align_to_panel(): now just one path — always calls panel_align_node,
# which itself owns the remembered-target/live-align decision (see
# test_panel_align_node.py for that side) ──────────────────────────────

def test_align_fails_when_service_unavailable(controller, monkeypatch):
    monkeypatch.setattr(controller._panel_align_client, 'wait_for_service', lambda timeout_sec=0: False)
    assert controller.align_to_panel() is False
    assert controller.has_remembered_panel_position is False


def test_align_calls_service_and_sets_flag_on_success(controller, monkeypatch):
    monkeypatch.setattr(controller._panel_align_client, 'wait_for_service', lambda timeout_sec=0: True)

    class _Result:
        success = True
        message = 'aligned'

    called = []
    monkeypatch.setattr(
        controller._panel_align_client, 'call_async',
        lambda req: called.append(req) or _FakeFuture(_Result()),
    )

    assert controller.align_to_panel() is True
    assert len(called) == 1
    assert controller.has_remembered_panel_position is True


def test_align_failure_does_not_set_flag(controller, monkeypatch):
    monkeypatch.setattr(controller._panel_align_client, 'wait_for_service', lambda timeout_sec=0: True)

    class _Result:
        success = False
        message = 'planning failed'

    monkeypatch.setattr(
        controller._panel_align_client, 'call_async', lambda req: _FakeFuture(_Result())
    )

    assert controller.align_to_panel() is False
    assert controller.has_remembered_panel_position is False


def test_align_timeout_returns_false_without_hanging(controller, monkeypatch):
    """A future that never resolves must not block past panel_align_timeout."""
    monkeypatch.setattr(controller, '_panel_align_timeout', 0.05)
    monkeypatch.setattr(controller._panel_align_client, 'wait_for_service', lambda timeout_sec=0: True)

    class _NeverResolvingFuture:
        def add_done_callback(self, cb):
            pass  # never calls cb — simulates a hung/lost response

    monkeypatch.setattr(
        controller._panel_align_client, 'call_async', lambda req: _NeverResolvingFuture()
    )

    assert controller.align_to_panel() is False


# ── keyboard: panel prompt / align handlers ──────────────────────────────

def test_keyboard_check_panel_visibility_prompts_on_rising_edge(controller, monkeypatch):
    loop = KeyboardInputLoop(controller)
    monkeypatch.setattr(controller, 'is_panel_visible', lambda: True)
    monkeypatch.setattr(controller, 'stop', lambda: None)
    loop._check_panel_visibility()
    assert loop._panel_prompt_pending is True
    assert loop._panel_was_visible is True


def test_keyboard_handle_panel_align_resumes_servo_regardless_of_outcome(controller, monkeypatch):
    loop = KeyboardInputLoop(controller)
    monkeypatch.setattr(controller, 'align_to_panel', lambda: False)
    started = []
    monkeypatch.setattr(controller, 'start_servo', lambda: started.append(True))
    loop._handle_panel_align()
    assert started == [True]


# ── safe-pose gating (keyboard) ─────────────────────────────────────────

def test_keyboard_starts_locked_out(controller):
    loop = KeyboardInputLoop(controller)
    assert loop._servo_started is False


def test_keyboard_safe_pose_starts_servo_only_on_success(controller, monkeypatch):
    loop = KeyboardInputLoop(controller)
    monkeypatch.setattr(controller, 'move_to_safe_pose', lambda: True)
    started = []
    monkeypatch.setattr(controller, 'start_servo', lambda: started.append(True) or True)
    loop._handle_safe_pose()
    assert loop._servo_started is True
    assert started == [True]


def test_keyboard_safe_pose_does_not_start_servo_on_failure(controller, monkeypatch):
    loop = KeyboardInputLoop(controller)
    monkeypatch.setattr(controller, 'move_to_safe_pose', lambda: False)
    started = []
    monkeypatch.setattr(controller, 'start_servo', lambda: started.append(True))
    loop._handle_safe_pose()
    assert loop._servo_started is False
    assert started == []


# ── gamepad: panel align button ─────────────────────────────────────────

def test_gamepad_panel_align_button_replays_remembered_position(controller, monkeypatch):
    loop = GamepadInputLoop(controller)
    monkeypatch.setattr(controller, 'move_to_safe_pose', lambda: True)
    monkeypatch.setattr(controller, 'start_servo', lambda: True)
    loop._handle_safe_pose()  # unlocks teleop

    controller._panel_align_succeeded_once = True
    aligned = []
    monkeypatch.setattr(loop, '_handle_panel_align', lambda: aligned.append(True))

    loop._on_joy(make_joy())  # establishes a button baseline
    buttons = [0] * 13
    buttons[loop.BUTTON_PANEL_ALIGN] = 1
    loop._on_joy(make_joy(buttons=buttons))

    assert aligned == [True]


# ── safe-pose gating + joy timeout (gamepad) ────────────────────────────

def test_gamepad_starts_teleop_locked(controller):
    loop = GamepadInputLoop(controller)
    assert loop._teleop_locked is True


def test_gamepad_stick_input_ignored_while_locked(controller):
    loop = GamepadInputLoop(controller)
    controller.vx = 999.0  # sentinel: only stop() would clear this
    loop._on_joy(make_joy(axes=[0.0, 0.0, 0.0, 1.0, 1.0, 1.0]))  # right stick forward
    assert controller.vx == 0.0


def test_gamepad_safe_pose_unlocks_teleop_on_success(controller, monkeypatch):
    loop = GamepadInputLoop(controller)
    monkeypatch.setattr(controller, 'move_to_safe_pose', lambda: True)
    monkeypatch.setattr(controller, 'start_servo', lambda: True)
    loop._handle_safe_pose()
    assert loop._teleop_locked is False


def test_gamepad_safe_pose_stays_locked_on_failure(controller, monkeypatch):
    loop = GamepadInputLoop(controller)
    monkeypatch.setattr(controller, 'move_to_safe_pose', lambda: False)
    loop._handle_safe_pose()
    assert loop._teleop_locked is True


# ── review-flagged: mode state must not change ahead of a successful move ──

def test_gamepad_sampling_mode_not_committed_on_home_failure(controller, monkeypatch):
    loop = GamepadInputLoop(controller)
    monkeypatch.setattr(controller, 'move_to_safe_pose', lambda **kwargs: False)
    loop._handle_safe_pose('sampling')
    assert loop._sampling_mode is False
    assert controller._sampling_mode is False


def test_gamepad_drill_mode_not_committed_on_home_failure(controller, monkeypatch):
    loop = GamepadInputLoop(controller)
    monkeypatch.setattr(controller, 'move_to_safe_pose', lambda **kwargs: False)
    loop._handle_safe_pose('drill')
    assert loop._drill_mode is False
    assert controller._drill_mode is False


def test_gamepad_sampling_mode_committed_on_home_success(controller, monkeypatch):
    loop = GamepadInputLoop(controller)
    monkeypatch.setattr(controller, 'move_to_safe_pose', lambda **kwargs: True)
    monkeypatch.setattr(controller, 'start_servo', lambda: True)
    loop._handle_safe_pose('sampling')
    assert loop._sampling_mode is True
    assert controller._sampling_mode is True


def test_gamepad_drill_mode_committed_on_home_success(controller, monkeypatch):
    loop = GamepadInputLoop(controller)
    monkeypatch.setattr(controller, 'move_to_safe_pose', lambda **kwargs: True)
    monkeypatch.setattr(controller, 'start_servo', lambda: True)
    loop._handle_safe_pose('drill')
    assert loop._drill_mode is True
    assert controller._drill_mode is True


def test_gamepad_joy_timeout_stops_the_arm(controller):
    loop = GamepadInputLoop(controller)
    controller.vx = 999.0
    loop._last_joy_time = controller.get_clock().now() - Duration(seconds=1.0)
    loop._check_joy_timeout()
    assert controller.vx == 0.0


def test_gamepad_no_timeout_when_joy_recently_seen(controller):
    loop = GamepadInputLoop(controller)
    controller.vx = 999.0
    loop._last_joy_time = controller.get_clock().now()
    loop._check_joy_timeout()
    assert controller.vx == 999.0  # untouched: no timeout yet


# ── move_to_safe_pose() must be collision-aware (review: not raw JTC) ──
# _FakeFuture is the one defined near the top of this file.

class _NeverResolvingFuture:
    """Simulates a goal handle whose result callback never arrives."""

    def add_done_callback(self, cb):
        pass


class _FakeGoalHandle:
    def __init__(self, accepted=True):
        self.accepted = accepted
        self.cancel_calls = 0

    def get_result_async(self):
        return _NeverResolvingFuture()

    def cancel_goal_async(self):
        self.cancel_calls += 1
        return _FakeFuture(None)


def test_move_to_safe_pose_uses_collision_aware_planning(controller, monkeypatch):
    monkeypatch.setattr(controller, 'stop_servo', lambda: True)
    monkeypatch.setattr(controller, 'use_trajectory_controller', lambda: True)
    # arm_motion_lock() now calls a real ROS service (arm_motion_lock_server)
    # — no such server in this unit test, so bypass it like every other
    # cross-node call in this file.
    monkeypatch.setattr(
        'arm_teleop.keyboard_servo_node.arm_motion_lock',
        lambda *a, **kw: contextlib.nullcontext())
    calls = []

    def _fake_execute(constraints):
        calls.append(constraints)
        return True, ''
    monkeypatch.setattr(controller, '_execute_move_group_constraints', _fake_execute)

    target = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]
    assert controller.move_to_safe_pose(positions=target, name='test') is True
    assert len(calls) == 1
    got = {jc.joint_name: jc.position for jc in calls[0].joint_constraints}
    assert got == dict(zip(HOME_POSE_JOINTS, target))
    # The raw FollowJointTrajectory path this used to take is gone entirely.
    assert not hasattr(controller, '_traj_client')


def test_execute_move_group_constraints_cancels_goal_on_timeout(controller, monkeypatch):
    controller.set_parameters([Parameter('safe_pose_timeout', value=0.05)])
    gh = _FakeGoalHandle(accepted=True)
    monkeypatch.setattr(controller._move_group_client, 'send_goal_async', lambda goal: _FakeFuture(gh))

    ok, error = controller._execute_move_group_constraints(Constraints())

    assert ok is False
    assert 'timed out' in error
    # Critical: leaving this goal running server-side after reporting
    # failure would let the arm keep moving under it once the caller
    # (believing the move failed) switches controllers/restarts Servo.
    assert gh.cancel_calls == 1


# ── run_planned_activity() must keep the arm still for the full 5s ─────

def test_set_velocity_forced_to_zero_during_activity_delay(controller):
    controller._activity_delay_active = True
    controller.set_velocity(vx=1.0, vy=1.0, vz=1.0, wx=1.0, wy=1.0, wz=1.0,
                             view_vx=1.0, view_vy=1.0, view_vz=1.0)
    assert (controller.vx, controller.vy, controller.vz) == (0.0, 0.0, 0.0)
    assert (controller.wx, controller.wy, controller.wz) == (0.0, 0.0, 0.0)
    assert (controller.view_vx, controller.view_vy, controller.view_vz) == (0.0, 0.0, 0.0)


def test_set_gripper_velocity_forced_to_zero_during_activity_delay(controller):
    controller._activity_delay_active = True
    controller.set_gripper_velocity(1.0)
    assert controller.gripper_vel == 0.0


def test_run_planned_activity_holds_input_suppressed_for_the_whole_sleep(controller, monkeypatch):
    monkeypatch.setattr(controller, '_signal_activity_indicator', lambda active: None)
    seen_during_sleep = []

    def _fake_sleep(seconds):
        seen_during_sleep.append(controller._activity_delay_active)

    monkeypatch.setattr(
        'arm_teleop.keyboard_servo_node.time.sleep', _fake_sleep)

    controller.run_planned_activity(lambda: True, 'test')

    assert seen_during_sleep == [True]  # suppressed for the entire wait
    assert controller._activity_delay_active is False  # cleared once action() runs


# ── review: a taught pose must never carry a NaN/inf value onward ──────

def test_load_home_pose_from_json_rejects_non_finite_values(monkeypatch):
    from arm_teleop.keyboard_servo_node import HOME_POSE_JOINTS, _load_home_pose_from_json
    bad_pose = {name: 0.0 for name in HOME_POSE_JOINTS}
    bad_pose[HOME_POSE_JOINTS[0]] = float('inf')  # e.g. a corrupted/malformed taught entry
    contents = json.dumps({'home': bad_pose})

    monkeypatch.setattr(pathlib.Path, 'is_file', lambda self: True)
    monkeypatch.setattr(pathlib.Path, 'read_text', lambda self: contents)

    assert _load_home_pose_from_json('home') is None


def test_load_home_pose_from_json_accepts_finite_values(monkeypatch):
    from arm_teleop.keyboard_servo_node import HOME_POSE_JOINTS, _load_home_pose_from_json
    good_pose = {name: 0.1 for name in HOME_POSE_JOINTS}
    contents = json.dumps({'home': good_pose})

    monkeypatch.setattr(pathlib.Path, 'is_file', lambda self: True)
    monkeypatch.setattr(pathlib.Path, 'read_text', lambda self: contents)

    assert _load_home_pose_from_json('home') == [0.1] * len(HOME_POSE_JOINTS)
