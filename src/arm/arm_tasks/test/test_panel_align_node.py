"""panel_align_node.py lifecycle/safety tests — the dangerous cases flagged
in review: hung execution + cancellation, cross-process motion exclusion,
and collision-checked remembered-position replay.

Internal methods are called/monkeypatched directly rather than round-tripped
through real action servers, mirroring test_keyboard_servo_node.py's own
rationale for the same choice.
"""
import threading

import rclpy
import pytest
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import Constraints, MoveItErrorCodes, RobotTrajectory
from rclpy.parameter import Parameter
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from arm_teleop.arm_motion_lock import ArmMotionBusy
from arm_tasks.panel_align_node import JOINT_LIMITS, PANEL_JOINT_LIMIT_PATH_CONSTRAINTS, PanelAlignNode


@pytest.fixture(scope='module', autouse=True)
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def node():
    n = PanelAlignNode()
    yield n
    n.destroy_node()


class _FakeFuture:
    """Stand-in for an rclpy Future that resolves synchronously."""

    def __init__(self, result=None):
        self._result = result

    def result(self):
        return self._result

    def add_done_callback(self, cb):
        cb(self)


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


def _valid_joint_positions():
    """Mid-range for every joint — well clear of any limit."""
    return {name: (lo + hi) / 2 for name, (lo, hi) in JOINT_LIMITS.items()}


def _fake_plan_result(joint_positions: dict):
    class _Result:
        error_code = MoveItErrorCodes(val=MoveItErrorCodes.SUCCESS)
        planned_trajectory = type('T', (), {'joint_trajectory': JointTrajectory(
            joint_names=list(joint_positions.keys()),
            points=[JointTrajectoryPoint(positions=list(joint_positions.values()))],
        )})()
    return _Result()


# ── Issue 1: hung execution must be cancelled, not left running ─────────

def test_execute_cancels_goal_and_waits_on_timeout(node, monkeypatch):
    node.set_parameters([Parameter('execution_timeout_sec', value=0.05)])
    monkeypatch.setattr(node._execute_client, 'wait_for_server', lambda timeout_sec=0: True)
    gh = _FakeGoalHandle(accepted=True)
    monkeypatch.setattr(node._execute_client, 'send_goal_async', lambda goal: _FakeFuture(gh))

    ok, reason = node._execute(RobotTrajectory(joint_trajectory=JointTrajectory()))

    assert ok is False
    assert 'cancelled' in reason
    assert gh.cancel_calls == 1


def test_execute_does_not_cancel_a_goal_that_was_never_accepted(node, monkeypatch):
    node.set_parameters([Parameter('execution_timeout_sec', value=0.05)])
    monkeypatch.setattr(node._execute_client, 'wait_for_server', lambda timeout_sec=0: True)
    gh = _FakeGoalHandle(accepted=False)
    monkeypatch.setattr(node._execute_client, 'send_goal_async', lambda goal: _FakeFuture(gh))

    ok, reason = node._execute(RobotTrajectory(joint_trajectory=JointTrajectory()))

    assert ok is False
    assert gh.cancel_calls == 0  # nothing to cancel — rejected immediately


# ── _request_plan() bounds OMPL's own search space, not just post-plan ──

def test_request_plan_bounds_ompl_to_panel_joint_limits(node, monkeypatch):
    monkeypatch.setattr(node._move_action_client, 'wait_for_server', lambda timeout_sec=0: True)
    sent_goals = []

    class _FakeResultWrapper:
        result = _fake_plan_result(_valid_joint_positions())

    def _fake_send_goal_async(goal):
        sent_goals.append(goal)
        gh = _FakeGoalHandle(accepted=True)
        gh.get_result_async = lambda: _FakeFuture(_FakeResultWrapper())
        return _FakeFuture(gh)

    monkeypatch.setattr(node._move_action_client, 'send_goal_async', _fake_send_goal_async)

    result = node._request_plan(Constraints())

    assert result is not None
    assert len(sent_goals) == 1
    # Without this, OMPL samples the URDF's full +-2pi range and only
    # gets rejected after the fact by _check_joint_margins — this is the
    # actual fix for that wasted plan-then-reject cycle.
    assert sent_goals[0].request.path_constraints == PANEL_JOINT_LIMIT_PATH_CONSTRAINTS


# ── Issue 2: cross-process exclusion surfaces as a clean failure ────────

def test_align_to_panel_fails_cleanly_when_arm_motion_busy(node, monkeypatch):
    def _raise_busy(*a, **kw):
        raise ArmMotionBusy("arm motion lock held by 'gs-laptop/keyboard_servo_node/123'")
    monkeypatch.setattr('arm_tasks.panel_align_node.arm_motion_lock', _raise_busy)

    assert node.align_to_panel() is False
    assert 'keyboard_servo_node' in node._last_status_message
    # The in-process lock must be released even when the cross-process
    # one raises, or a SECOND call would wrongly report "already in
    # progress" instead of the real busy reason.
    assert node._align_running.acquire(blocking=False)
    node._align_running.release()


# ── align_to_panel() dispatch: remembered target always wins ────────────

def test_dispatches_to_live_align_when_nothing_remembered(node, monkeypatch):
    called = []
    monkeypatch.setattr('arm_tasks.panel_align_node.arm_motion_lock', lambda *a, **kw: _NullContext())
    monkeypatch.setattr(node, '_align_live_locked', lambda: called.append('live') or True)
    monkeypatch.setattr(node, '_align_to_remembered_locked', lambda: called.append('remembered') or True)

    assert node.align_to_panel() is True
    assert called == ['live']


def test_dispatches_to_remembered_replay_once_learned(node, monkeypatch):
    node._remembered_target_joints = _valid_joint_positions()
    monkeypatch.setattr('arm_tasks.panel_align_node.arm_motion_lock', lambda *a, **kw: _NullContext())
    called = []
    monkeypatch.setattr(node, '_align_live_locked', lambda: called.append('live') or True)
    monkeypatch.setattr(node, '_align_to_remembered_locked', lambda: called.append('remembered') or True)

    assert node.align_to_panel() is True
    assert called == ['remembered']


# ── Issue 3: remembered replay is collision-checked, not a raw move ─────

def test_remembered_replay_applies_collision_object_and_uses_joint_constraints(node, monkeypatch):
    target = _valid_joint_positions()
    node._remembered_target_joints = target
    node._remembered_panel_collision_pose = PoseStamped()

    monkeypatch.setattr(node, '_call_stop_servo', lambda: True)
    monkeypatch.setattr(node, '_use_trajectory_controller', lambda: True)
    monkeypatch.setattr(node, '_current_live_panel_collision_pose', lambda: None)  # panel not visible now

    collision_calls = []
    monkeypatch.setattr(
        node, '_apply_panel_collision_object', lambda pose: collision_calls.append(pose) or True)

    plan_calls = []

    def _fake_request_plan(goal_constraints):
        plan_calls.append(goal_constraints)
        return _fake_plan_result(target)
    monkeypatch.setattr(node, '_request_plan', _fake_request_plan)
    monkeypatch.setattr(node, '_execute', lambda traj: (True, ''))

    assert node._align_to_remembered_locked() is True
    # Collision-checked: the panel's CollisionObject was inserted before
    # planning, using the CACHED pose since live detection wasn't available —
    # this is the actual fix for "replay bypasses collision checking".
    assert collision_calls == [node._remembered_panel_collision_pose]
    # Joint-space, not Cartesian: exact final pose guaranteed regardless
    # of the path OMPL finds to get there.
    assert len(plan_calls) == 1
    assert plan_calls[0].joint_constraints
    assert not plan_calls[0].position_constraints
    assert not plan_calls[0].orientation_constraints
    got = {jc.joint_name: jc.position for jc in plan_calls[0].joint_constraints}
    assert got == target


def test_remembered_replay_prefers_live_panel_pose_when_available(node, monkeypatch):
    node._remembered_target_joints = _valid_joint_positions()
    node._remembered_panel_collision_pose = PoseStamped()
    live_pose = PoseStamped()

    monkeypatch.setattr(node, '_call_stop_servo', lambda: True)
    monkeypatch.setattr(node, '_use_trajectory_controller', lambda: True)
    monkeypatch.setattr(node, '_current_live_panel_collision_pose', lambda: live_pose)

    collision_calls = []
    monkeypatch.setattr(
        node, '_apply_panel_collision_object', lambda pose: collision_calls.append(pose) or True)
    monkeypatch.setattr(node, '_request_plan', lambda gc: _fake_plan_result(node._remembered_target_joints))
    monkeypatch.setattr(node, '_execute', lambda traj: (True, ''))

    assert node._align_to_remembered_locked() is True
    assert collision_calls == [live_pose]  # live pose preferred over the cached one


def test_remembered_replay_fails_on_joint_limit_margin(node, monkeypatch):
    # Every joint pinned exactly at its lower limit — 0% margin.
    tight = {name: lo for name, (lo, hi) in JOINT_LIMITS.items()}
    node._remembered_target_joints = tight
    node._remembered_panel_collision_pose = PoseStamped()

    monkeypatch.setattr(node, '_call_stop_servo', lambda: True)
    monkeypatch.setattr(node, '_use_trajectory_controller', lambda: True)
    monkeypatch.setattr(node, '_current_live_panel_collision_pose', lambda: None)
    monkeypatch.setattr(node, '_apply_panel_collision_object', lambda pose: True)
    monkeypatch.setattr(node, '_request_plan', lambda gc: _fake_plan_result(tight))
    executed = []
    monkeypatch.setattr(node, '_execute', lambda traj: executed.append(True) or (True, ''))

    assert node._align_to_remembered_locked() is False
    assert executed == []  # never even tried to move — rejected before execution


# ── orient_gripper_to_remembered() ('m'): tighter tolerance, no replan ──

def _fake_tf(x, y, z):
    class _T:
        transform = type('X', (), {'translation': type('V', (), {'x': x, 'y': y, 'z': z})()})()
    return _T()


def test_orient_gripper_fails_without_remembered_orientation(node, monkeypatch):
    node._remembered_target_orientation = None
    stop_calls = []
    monkeypatch.setattr(node, '_call_stop_servo', lambda: stop_calls.append(True) or True)

    assert node._orient_gripper_to_remembered_locked() is False
    assert stop_calls == []  # bailed out before touching the arm at all


def test_orient_gripper_uses_ik_solution_for_current_position(node, monkeypatch):
    node._remembered_target_orientation = (0.0, 0.0, 0.0, 1.0)
    monkeypatch.setattr(node._tf_buffer, 'lookup_transform', lambda *a, **kw: _fake_tf(1.0, 2.0, 3.0))
    monkeypatch.setattr(node, '_call_stop_servo', lambda: True)
    monkeypatch.setattr(node, '_use_trajectory_controller', lambda: True)

    ik_calls = []
    solution = _valid_joint_positions()

    def _fake_compute_ik(target_pose):
        ik_calls.append(target_pose)
        return solution
    monkeypatch.setattr(node, '_compute_ik_within_panel_limits', _fake_compute_ik)

    plan_calls = []
    extra_constraint_calls = []

    def _fake_request_plan(goal_constraints, extra_position_constraints=None):
        plan_calls.append(goal_constraints)
        extra_constraint_calls.append(extra_position_constraints)
        return _fake_plan_result(solution)
    monkeypatch.setattr(node, '_request_plan', _fake_request_plan)
    monkeypatch.setattr(node, '_execute', lambda traj: (True, ''))

    assert node._orient_gripper_to_remembered_locked() is True
    # IK ran on the CURRENT tip position (from TF) + the remembered
    # orientation — not a fresh Cartesian pose goal (see
    # _compute_ik_within_panel_limits' own docstring for why).
    assert len(ik_calls) == 1
    p = ik_calls[0].pose.position
    assert (p.x, p.y, p.z) == (1.0, 2.0, 3.0)
    # Planned as an exact joint-space goal to the IK solution, guaranteed
    # inside JOINT_LIMITS by _compute_ik_within_panel_limits itself.
    assert len(plan_calls) == 1
    assert not plan_calls[0].position_constraints
    assert not plan_calls[0].orientation_constraints
    got = {jc.joint_name: jc.position for jc in plan_calls[0].joint_constraints}
    assert got == solution
    # review-flagged: the nearest IK branch alone only bounds the
    # endpoint — this path_constraint is what keeps the WHOLE motion
    # near the current position, not just where it ends up.
    assert len(extra_constraint_calls) == 1
    (stay_near,) = extra_constraint_calls[0]
    assert stay_near.link_name == 'arm_tcp_link'
    region_pose = stay_near.constraint_region.primitive_poses[0].position
    assert (region_pose.x, region_pose.y, region_pose.z) == (1.0, 2.0, 3.0)
    expected_radius = node.get_parameter('orient_gripper_path_position_radius').value
    assert stay_near.constraint_region.primitives[0].dimensions[0] == expected_radius


def test_orient_gripper_applies_live_panel_collision_object_when_available(node, monkeypatch):
    node._remembered_target_orientation = (0.0, 0.0, 0.0, 1.0)
    node._remembered_panel_collision_pose = PoseStamped()
    live_pose = PoseStamped()
    monkeypatch.setattr(node._tf_buffer, 'lookup_transform', lambda *a, **kw: _fake_tf(1.0, 2.0, 3.0))
    monkeypatch.setattr(node, '_call_stop_servo', lambda: True)
    monkeypatch.setattr(node, '_use_trajectory_controller', lambda: True)
    monkeypatch.setattr(node, '_current_live_panel_collision_pose', lambda: live_pose)
    monkeypatch.setattr(node, '_compute_ik_within_panel_limits', lambda pose: _valid_joint_positions())

    collision_calls = []
    monkeypatch.setattr(
        node, '_apply_panel_collision_object', lambda pose: collision_calls.append(pose) or True)
    monkeypatch.setattr(node, '_request_plan', lambda gc, **kw: _fake_plan_result(_valid_joint_positions()))
    monkeypatch.setattr(node, '_execute', lambda traj: (True, ''))

    assert node._orient_gripper_to_remembered_locked() is True
    assert collision_calls == [live_pose]  # live pose preferred over the cached one


def test_orient_gripper_falls_back_to_remembered_collision_pose(node, monkeypatch):
    node._remembered_target_orientation = (0.0, 0.0, 0.0, 1.0)
    node._remembered_panel_collision_pose = PoseStamped()
    monkeypatch.setattr(node._tf_buffer, 'lookup_transform', lambda *a, **kw: _fake_tf(1.0, 2.0, 3.0))
    monkeypatch.setattr(node, '_call_stop_servo', lambda: True)
    monkeypatch.setattr(node, '_use_trajectory_controller', lambda: True)
    monkeypatch.setattr(node, '_current_live_panel_collision_pose', lambda: None)  # panel not visible now
    monkeypatch.setattr(node, '_compute_ik_within_panel_limits', lambda pose: _valid_joint_positions())

    collision_calls = []
    monkeypatch.setattr(
        node, '_apply_panel_collision_object', lambda pose: collision_calls.append(pose) or True)
    monkeypatch.setattr(node, '_request_plan', lambda gc, **kw: _fake_plan_result(_valid_joint_positions()))
    monkeypatch.setattr(node, '_execute', lambda traj: (True, ''))

    assert node._orient_gripper_to_remembered_locked() is True
    assert collision_calls == [node._remembered_panel_collision_pose]


def test_orient_gripper_fails_cleanly_when_collision_object_apply_fails(node, monkeypatch):
    node._remembered_target_orientation = (0.0, 0.0, 0.0, 1.0)
    node._remembered_panel_collision_pose = PoseStamped()
    monkeypatch.setattr(node._tf_buffer, 'lookup_transform', lambda *a, **kw: _fake_tf(1.0, 2.0, 3.0))
    monkeypatch.setattr(node, '_call_stop_servo', lambda: True)
    monkeypatch.setattr(node, '_use_trajectory_controller', lambda: True)
    monkeypatch.setattr(node, '_current_live_panel_collision_pose', lambda: None)
    monkeypatch.setattr(node, '_apply_panel_collision_object', lambda pose: False)
    ik_calls = []
    monkeypatch.setattr(
        node, '_compute_ik_within_panel_limits',
        lambda pose: ik_calls.append(True) or _valid_joint_positions())
    planned = []
    monkeypatch.setattr(node, '_request_plan', lambda gc, **kw: planned.append(True) or _fake_plan_result({}))

    assert node._orient_gripper_to_remembered_locked() is False
    assert ik_calls == []  # never even tried IK — rejected before that
    assert planned == []  # never even tried to plan — rejected before that


def test_orient_gripper_fails_cleanly_on_plan_failure(node, monkeypatch):
    node._remembered_target_orientation = (0.0, 0.0, 0.0, 1.0)
    monkeypatch.setattr(node._tf_buffer, 'lookup_transform', lambda *a, **kw: _fake_tf(0.0, 0.0, 0.0))
    monkeypatch.setattr(node, '_call_stop_servo', lambda: True)
    monkeypatch.setattr(node, '_use_trajectory_controller', lambda: True)
    monkeypatch.setattr(node, '_compute_ik_within_panel_limits', lambda pose: _valid_joint_positions())
    monkeypatch.setattr(node, '_request_plan', lambda gc, **kw: None)
    executed = []
    monkeypatch.setattr(node, '_execute', lambda traj: executed.append(True) or (True, ''))

    assert node._orient_gripper_to_remembered_locked() is False
    assert executed == []


def test_orient_gripper_fails_cleanly_when_no_ik_solution(node, monkeypatch):
    node._remembered_target_orientation = (0.0, 0.0, 0.0, 1.0)
    monkeypatch.setattr(node._tf_buffer, 'lookup_transform', lambda *a, **kw: _fake_tf(0.0, 0.0, 0.0))
    monkeypatch.setattr(node, '_call_stop_servo', lambda: True)
    monkeypatch.setattr(node, '_use_trajectory_controller', lambda: True)
    monkeypatch.setattr(node, '_compute_ik_within_panel_limits', lambda pose: None)
    planned = []
    monkeypatch.setattr(node, '_request_plan', lambda gc, **kw: planned.append(True) or _fake_plan_result({}))

    assert node._orient_gripper_to_remembered_locked() is False
    assert planned == []  # never even tried to plan — no IK solution to plan toward


# ── _compute_ik_within_panel_limits(): retries until a solution fits ────

def _fake_ik_result(joint_positions: dict, error_val=MoveItErrorCodes.SUCCESS):
    class _Result:
        error_code = MoveItErrorCodes(val=error_val)
        solution = type('S', (), {'joint_state': type('J', (), {
            'name': list(joint_positions.keys()), 'position': list(joint_positions.values()),
        })()})()
    return _Result()


def test_compute_ik_within_panel_limits_returns_first_valid_solution(node, monkeypatch):
    monkeypatch.setattr(node._compute_ik_client, 'wait_for_service', lambda timeout_sec=0: True)
    valid = _valid_joint_positions()
    monkeypatch.setattr(
        node._compute_ik_client, 'call_async', lambda req: _FakeFuture(_fake_ik_result(valid)))

    result = node._compute_ik_within_panel_limits(PoseStamped())
    assert result == valid


def test_compute_ik_within_panel_limits_retries_past_out_of_range_solutions(node, monkeypatch):
    # Runs every attempt now (collects all valid solutions, then picks
    # the closest to current state — see the "prefers closest" test
    # below), so this needs exactly as many canned responses as attempts.
    node.set_parameters([Parameter('live_align_ik_attempts', value=3)])
    monkeypatch.setattr(node._compute_ik_client, 'wait_for_service', lambda timeout_sec=0: True)
    out_of_range = {name: hi + 1.0 for name, (lo, hi) in JOINT_LIMITS.items()}
    valid = _valid_joint_positions()
    responses = [_fake_ik_result(out_of_range), _fake_ik_result(out_of_range), _fake_ik_result(valid)]
    calls = []

    def _call_async(req):
        calls.append(req)
        return _FakeFuture(responses[len(calls) - 1])
    monkeypatch.setattr(node._compute_ik_client, 'call_async', _call_async)

    result = node._compute_ik_within_panel_limits(PoseStamped())
    assert result == valid
    assert len(calls) == 3


def test_compute_ik_within_panel_limits_gives_up_after_attempts_exhausted(node, monkeypatch):
    node.set_parameters([Parameter('live_align_ik_attempts', value=3)])
    monkeypatch.setattr(node._compute_ik_client, 'wait_for_service', lambda timeout_sec=0: True)
    out_of_range = {name: hi + 1.0 for name, (lo, hi) in JOINT_LIMITS.items()}
    calls = []

    def _call_async(req):
        calls.append(req)
        return _FakeFuture(_fake_ik_result(out_of_range))
    monkeypatch.setattr(node._compute_ik_client, 'call_async', _call_async)

    assert node._compute_ik_within_panel_limits(PoseStamped()) is None
    assert len(calls) == 3


# ── review: malformed/partial/non-finite IK responses must never pass ───
# (all([]) is vacuously True in Python — an empty or incomplete solution
# dict must be rejected explicitly, not just "not fail to iterate").

def test_compute_ik_within_panel_limits_rejects_solution_missing_a_joint(node, monkeypatch):
    node.set_parameters([Parameter('live_align_ik_attempts', value=1)])
    monkeypatch.setattr(node._compute_ik_client, 'wait_for_service', lambda timeout_sec=0: True)
    incomplete = _valid_joint_positions()
    del incomplete['arm_wrist_2_end_effector_joint']
    monkeypatch.setattr(
        node._compute_ik_client, 'call_async', lambda req: _FakeFuture(_fake_ik_result(incomplete)))

    assert node._compute_ik_within_panel_limits(PoseStamped()) is None


def test_compute_ik_within_panel_limits_rejects_empty_solution(node, monkeypatch):
    node.set_parameters([Parameter('live_align_ik_attempts', value=1)])
    monkeypatch.setattr(node._compute_ik_client, 'wait_for_service', lambda timeout_sec=0: True)
    monkeypatch.setattr(
        node._compute_ik_client, 'call_async', lambda req: _FakeFuture(_fake_ik_result({})))

    assert node._compute_ik_within_panel_limits(PoseStamped()) is None


def test_compute_ik_within_panel_limits_rejects_mismatched_name_position_arrays(node, monkeypatch):
    node.set_parameters([Parameter('live_align_ik_attempts', value=1)])
    monkeypatch.setattr(node._compute_ik_client, 'wait_for_service', lambda timeout_sec=0: True)
    names = list(JOINT_LIMITS.keys())
    positions = [0.0] * (len(names) - 1)  # one short — malformed response

    def _fake_mismatched(req):
        class _Result:
            error_code = MoveItErrorCodes(val=MoveItErrorCodes.SUCCESS)
            solution = type('S', (), {'joint_state': type('J', (), {
                'name': names, 'position': positions,
            })()})()
        return _FakeFuture(_Result())
    monkeypatch.setattr(node._compute_ik_client, 'call_async', _fake_mismatched)

    assert node._compute_ik_within_panel_limits(PoseStamped()) is None


def test_compute_ik_within_panel_limits_rejects_non_finite_values(node, monkeypatch):
    node.set_parameters([Parameter('live_align_ik_attempts', value=1)])
    monkeypatch.setattr(node._compute_ik_client, 'wait_for_service', lambda timeout_sec=0: True)
    bad = _valid_joint_positions()
    bad['arm_wrist_2_end_effector_joint'] = float('nan')
    monkeypatch.setattr(
        node._compute_ik_client, 'call_async', lambda req: _FakeFuture(_fake_ik_result(bad)))

    assert node._compute_ik_within_panel_limits(PoseStamped()) is None


def test_compute_ik_within_panel_limits_rejects_non_positive_attempts(node, monkeypatch):
    node.set_parameters([Parameter('live_align_ik_attempts', value=0)])
    monkeypatch.setattr(node._compute_ik_client, 'wait_for_service', lambda timeout_sec=0: True)
    calls = []
    monkeypatch.setattr(
        node._compute_ik_client, 'call_async',
        lambda req: calls.append(req) or _FakeFuture(_fake_ik_result(_valid_joint_positions())))

    assert node._compute_ik_within_panel_limits(PoseStamped()) is None
    assert calls == []  # never even tried to call /compute_ik


# ── review: a late callback from a timed-out attempt must not corrupt a
# later attempt's state (result/done were being captured by NAME, not by
# value, so a stale callback could overwrite a newer attempt's outcome) ──

class _DelayedFuture:
    """Stores the callback instead of firing it immediately, so a test
    can simulate it arriving AFTER the caller already stopped waiting."""

    def __init__(self, result):
        self._result = result
        self._cb = None

    def result(self):
        return self._result

    def add_done_callback(self, cb):
        self._cb = cb

    def fire(self):
        self._cb(self)


def test_compute_ik_within_panel_limits_late_callback_does_not_corrupt_next_attempt(node, monkeypatch):
    node.set_parameters([Parameter('live_align_ik_attempts', value=2)])
    monkeypatch.setattr(node._compute_ik_client, 'wait_for_service', lambda timeout_sec=0: True)
    out_of_range = {name: hi + 1.0 for name, (lo, hi) in JOINT_LIMITS.items()}
    valid = _valid_joint_positions()

    futures = []

    def _call_async(req):
        if not futures:
            # First attempt: a future that never fires on its own — its
            # callback only runs when the test calls .fire() explicitly,
            # simulating a response that arrives late.
            fut = _DelayedFuture(_fake_ik_result(out_of_range))
            futures.append(fut)
            return fut
        # Second attempt: fires immediately, like every other test's
        # _FakeFuture — a normal, on-time response.
        futures.append(None)
        return _FakeFuture(_fake_ik_result(valid))
    monkeypatch.setattr(node._compute_ik_client, 'call_async', _call_async)

    # Only the FIRST attempt's own done.wait() should time out (its
    # future never fires); the second's Event is already set by the time
    # its own wait() runs, so faking it to succeed is just documenting
    # that — not doing the real work.
    wait_calls = []

    def _fake_wait(self, timeout=None):
        wait_calls.append(self)
        return len(wait_calls) != 1
    monkeypatch.setattr(threading.Event, 'wait', _fake_wait)

    result = node._compute_ik_within_panel_limits(PoseStamped())
    assert result == valid
    assert len(futures) == 2

    # Fire attempt 1's callback late, well after the method already
    # returned attempt 2's result — must not raise (its result/done were
    # isolated per-attempt via default-arg capture, so this writes into
    # a dict/event nothing reads anymore instead of attempt 2's).
    futures[0].fire()


def test_compute_ik_within_panel_limits_prefers_solution_closest_to_current_state(node, monkeypatch):
    node.set_parameters([Parameter('live_align_ik_attempts', value=2)])
    monkeypatch.setattr(node._compute_ik_client, 'wait_for_service', lambda timeout_sec=0: True)
    current = _valid_joint_positions()
    node._latest_joint_positions = dict(current)

    near = dict(current)
    near['arm_wrist_2_end_effector_joint'] += 0.05
    far = dict(current)
    far['arm_wrist_2_end_effector_joint'] -= 1.0  # a different, more distant branch

    # The FAR solution is returned first (as if that's the branch KDL's
    # seed happened to converge on); the NEAR one only shows up on a
    # later retry. The method must still prefer 'near' — this is what
    # makes 'm' actually reorient in place instead of accepting whatever
    # valid branch happened to come back first.
    responses = [_fake_ik_result(far), _fake_ik_result(near)]
    calls = []

    def _call_async(req):
        calls.append(req)
        return _FakeFuture(responses[len(calls) - 1])
    monkeypatch.setattr(node._compute_ik_client, 'call_async', _call_async)

    result = node._compute_ik_within_panel_limits(PoseStamped())
    assert result == near


class _NullContext:
    def __enter__(self):
        return self

    def __exit__(self, *exc):
        return False
