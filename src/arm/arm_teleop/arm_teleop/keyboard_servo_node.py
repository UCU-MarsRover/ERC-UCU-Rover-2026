#!/usr/bin/env python3
"""
Keyboard control node for MoveIt Servo.

Reads keyboard input and sends Cartesian velocity commands to MoveIt Servo.
Designed to be modular — core logic is in ServoController class,
so switching to gamepad (joy package) requires only replacing the input source.

Controls:
    EEF translation — absolute (in arm_mount_link):
        w / s  — +X / -X
        a / d  — +Y / -Y
        q / e  — +Z / -Z

    EEF translation — view-relative (in arm_camera_link, rigid with the
    gripper; axes are REP-103 so +X is where the camera and gripper point):
        Up / Down    — forward / back
        Left / Right — left / right
        t / g        — up / down

    Both translation sets are summed, so they can be held together.

    EEF rotation (about arm_tcp_link). Names are from the operator's point of
    view, i.e. the camera frame, which is what the TCP axes actually work out
    to: TCP +X is the camera's left-right axis (pitch), TCP +Y its vertical
    axis (yaw), TCP +Z its line of sight (roll):
        i / k  — pitch (+/- wx)
        u / o  — yaw   (+/- wy)
        j / l  — roll  (+/- wz)

    Gripper (commanded directly on gripper_right/left_controller, bypassing Servo):
        b / v  — open / close

    r      — move to home + start servo
    p      — align to detected panel (see panel_align_node); the first
             successful align each session is remembered, so later presses
             replay that exact position instead of re-planning
    f      — level tool (collision-checked; locks pitch/yaw after — 'r'
             unlocks)
    m      — rotate the gripper in place to face the remembered panel
             direction, current position kept (needs 'p' once first)
    ESC/x  — exit

    r/p/f/m (and their gamepad equivalents, see below) first light the
    activity indicator and hold for 5s with the arm untouched before
    actually moving — ERC 2026 Rules, Appendix 3, REQ-OPS-080/090/100 (see
    ServoController.run_planned_activity()).

Gamepad controls (via ros2 joy joy_node, e.g. Stadia controller).
All gamepad translation is view-relative (arm_camera_link); rotation is
about arm_tcp_link, same as the keyboard's I/K/U/O/J/L:
    Left stick  left/right — EEF left / right (camera)
    Left stick  up/down    — EEF forward / back (camera)
    Right stick up/down    — EEF up / down (camera)
    Right stick left/right — yaw (TCP)
    R1 + right stick up/down    — pitch (TCP)
    R1 + right stick left/right — roll (TCP)
    A                — move to home + start servo
    Button 7         — align to detected panel (see keyboard 'p' above)
    Button 8         — reorient gripper only (see keyboard 'm' above)
    X                — exit

Usage (stack in one terminal, input in another):
        ros2 launch arm_bringup arm.launch.py use_fake_hardware:=false
        ros2 run arm_teleop keyboard_servo_node
        # optional: pin a device — ros2 run ... --ros-args -p keyboard_device_path:=/dev/input/event19

    Gazebo sim (sim clock + faster speeds, see arm_sim/config/keyboard_servo_sim.yaml):
        ros2 run arm_teleop keyboard_servo_node --ros-args \\
            --params-file $(ros2 pkg prefix arm_sim)/share/arm_sim/config/keyboard_servo_sim.yaml

    Gamepad only:
        ros2 launch arm_teleop gamepad.launch.py
"""

import sys
import os
import fcntl
import functools
import threading
import termios
import time
import json
import math
import select
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped, Quaternion, TwistStamped
from sensor_msgs.msg import JointState, Joy
from std_msgs.msg import Int8, Float64MultiArray, ColorRGBA, String
from std_srvs.srv import Trigger
from action_msgs.msg import GoalStatus
from controller_manager_msgs.srv import ListControllers, SwitchController
from builtin_interfaces.msg import Duration
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes
from tf2_ros import Buffer, TransformListener
from tf2_ros import TransformException
from indomitus_interfaces.msg import EndEffectorState
from indomitus_interfaces.srv import AcquireArmMotionLock, ReleaseArmMotionLock
import evdev
from evdev import ecodes
import socket

from arm_teleop.arm_motion_lock import ArmMotionBusy, arm_motion_lock


DEFAULT_LINEAR_SPEED  = 0.6
DEFAULT_ANGULAR_SPEED = 1.8
DEFAULT_PUBLISH_RATE  = 100.0
# Q/E (±Z) folds the shoulder/elbow; without ω the TCP pitch walks.
# Hold attitude only — do not scale XYZ (that made teleop feel slow).
HOLD_ANGULAR_GAIN = 6.0
HOLD_ANGULAR_MAX = 0.8
HOLD_CMD_EPS = 1e-4
# _level_hold target (sampling + drill modes): world-down, roll unconstrained.
DOWN_AXIS = (0.0, 0.0, -1.0)

SAMPLING_POINT_AXIS = (0.0, 0.0, 1.0)
DRILL_POINT_AXIS = (0.0, -1.0, 0.0)
# jaw's own probing mode (see set_probing_mode) uses the same tool-forward
# axis as SAMPLING_POINT_AXIS — jaw's TCP +Z is its reach direction, same
# convention sampling's tool shares — kept as its own named constant so jaw
# and drill_sampling don't read as coupled if either axis ever needs to
# change independently.
PROBING_POINT_AXIS = (0.0, 0.0, 1.0)
# Untested on real hardware yet — B/Y (sampling_home/drill_home) and the
# level button log and do nothing while False.
SAMPLING_DRILL_MODES_ENABLED = True

LEVEL_TOOL_TARGET_POSE = [
    -0.06367867092618458,  # arm_mount_base_joint
    0.8861027135101929,    # arm_base_shoulder_joint
    -0.36962295375967424,  # arm_shoulder_forearm_joint
    0.26862548294026134,   # arm_forearm_wrist_1_joint
    2.125629420127624,     # arm_wrist_1_wrist_2_joint
    -1.0466698958262262,   # arm_wrist_2_end_effector_joint
]

DEFAULT_LINEAR_FRAME  = 'arm_mount_link'
DEFAULT_EE_FRAME      = 'arm_tcp_link'

DEFAULT_VIEW_FRAME    = 'arm_camera_link'
# Fallback if poses.json "home" cannot be loaded (matches SRDF group_state home).
DEFAULT_HOME_POSE     = [-1.552, 0.5057, 1.1731, 0.717, 0.0093, -1.536]
# 'auto' picks a USB/external keyboard (Keychron, etc.) over the laptop's
# built-in AT Translated Set 2 device — the usual failure mode in Docker.
DEFAULT_KEYBOARD_DEVICE_PATH = 'auto'
DEFAULT_GAMEPAD_SHIFT_BUTTON = 10
DEFAULT_SAFE_POSE_TIMEOUT = 60.0
# Scaling passed on every move_group goal in _execute_move_group_constraints
# (A/B/Y home moves, level_tool) — fraction of each joint's own max_velocity/
# max_acceleration (arm_moveit_config/config/joint_limits.yaml), which
# itself already has a conservative default_velocity/acceleration_scaling_
# factor of 0.1 ("for beginners"). 0.15 here felt slow/jerky in practice —
# low acceleration in particular reads as jerkiness, not just slowness,
# since the arm can't ramp torque smoothly. Raise further if still too slow,
# lower again if it overshoots/oscillates or feels unsafe near people.
DEFAULT_PLAN_EXECUTE_VELOCITY_SCALING = 1.0
DEFAULT_PLAN_EXECUTE_ACCELERATION_SCALING = 1.0

DEFAULT_GRIPPER_SPEED = 0.006   # m/s
DEFAULT_GRIPPER_STROKE = 0.012  # m — matches finger_stroke in arm_macro.xacro

GRIPPER_JOINT_NAME = 'arm_jaw_gripper_finger_right_joint'

DEFAULT_PANEL_POSE_TOPIC = '/panel_pose'
DEFAULT_PANEL_VISIBLE_MAX_AGE_SEC = 3.0
DEFAULT_PANEL_ALIGN_TIMEOUT = 120.0

# ERC 2026 Rules, Appendix 3, REQ-OPS-090 mandates this be >= 5.0s during
# actual competition — see run_planned_activity()'s own docstring. Exposed
# as the 'activity_indicator_pre_delay_sec' ROS param specifically so it can
# (and, at competition, MUST) be set back to ERC_REQUIRED_... via launch
# argument; the default below is 0.0 for bench testing only.
ERC_REQUIRED_ACTIVITY_INDICATOR_PRE_DELAY_SEC = 5.0
DEFAULT_ACTIVITY_INDICATOR_PRE_DELAY_SEC = 0.0
DEFAULT_ACTIVITY_INDICATOR_TOPIC = 'activity_indicator'
ACTIVITY_INDICATOR_COLOR_ACTIVE = (0.0, 0.0, 1.0, 1.0)  # blue, a=1 (lit)
ACTIVITY_INDICATOR_COLOR_IDLE = (0.0, 0.0, 0.0, 0.0)    # off

JTC_CONTROLLER_NAME = 'indomitus_arm_controller'
FORWARD_CONTROLLER_NAME = 'indomitus_arm_forward_position_controller'
MOVEIT_GROUP_NAME = 'indomitus_arm'

HOME_POSE_JOINTS = [
    'arm_mount_base_joint',
    'arm_base_shoulder_joint',
    'arm_shoulder_forearm_joint',
    'arm_forearm_wrist_1_joint',
    'arm_wrist_1_wrist_2_joint',
    'arm_wrist_2_end_effector_joint',
]
def _load_home_pose_from_json(pose_name='home'):
    """Return ``pose_name`` joint positions from poses.json, or None if unavailable."""
    candidates = [
        Path('/opt/ws/src/arm/arm_teleop/poses.json'),
        Path(__file__).resolve().parent.parent / 'poses.json',
    ]
    try:
        from ament_index_python.packages import get_package_share_directory
        share = Path(get_package_share_directory('arm_teleop')) / 'poses.json'
        candidates.insert(0, share)
    except Exception:
        pass

    for path in candidates:
        if not path.is_file():
            continue
        try:
            data = json.loads(path.read_text())
            pose = data.get(pose_name) or {}
            values = [float(pose[name]) for name in HOME_POSE_JOINTS]
        except (KeyError, TypeError, ValueError, OSError, json.JSONDecodeError):
            continue
        if not all(math.isfinite(v) for v in values):
            return None
        return values
    return None


def _quat_multiply(a: Quaternion, b: Quaternion) -> Quaternion:
    return Quaternion(
        x=a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
        y=a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
        z=a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
        w=a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
    )


def _quat_conj(q: Quaternion) -> Quaternion:
    return Quaternion(x=-q.x, y=-q.y, z=-q.z, w=q.w)


def _quat_rotvec(q: Quaternion):
    w = max(-1.0, min(1.0, q.w))
    x, y, z = q.x, q.y, q.z
    if w < 0.0:
        w, x, y, z = -w, -x, -y, -z
    half = math.acos(w)
    sine = math.sqrt(max(0.0, 1.0 - w * w))
    if sine < 1e-8:
        return (2.0 * x, 2.0 * y, 2.0 * z)
    scale = 2.0 * half / sine
    return (scale * x, scale * y, scale * z)


def _rotate_vector_by_quat(q, x: float, y: float, z: float):
    """Rotate a free vector by a geometry_msgs quaternion (x,y,z,w)."""
    qx, qy, qz, qw = q.x, q.y, q.z, q.w
    tx = 2.0 * (qy * z - qz * y)
    ty = 2.0 * (qz * x - qx * z)
    tz = 2.0 * (qx * y - qy * x)
    return (
        x + qw * tx + (qy * tz - qz * ty),
        y + qw * ty + (qz * tx - qx * tz),
        z + qw * tz + (qx * ty - qy * tx),
    )


def _list_keyboard_candidates():
    """Return evdev devices that look like QWERTY keyboards (path, name, score)."""
    required = {ecodes.KEY_R, ecodes.KEY_W, ecodes.KEY_A, ecodes.KEY_ESC}
    candidates = []
    for path in evdev.list_devices():
        try:
            device = evdev.InputDevice(path)
        except (FileNotFoundError, PermissionError, OSError):
            continue
        keys = set(device.capabilities().get(ecodes.EV_KEY, []))
        if not required.issubset(keys):
            continue
        name = device.name or ''
        phys = device.phys or ''
        name_l = name.lower()
        # Skip obvious non-keyboards that still expose a few KEY_* codes.
        if any(bad in name_l for bad in ('sleep', 'lid', 'power', 'video bus', 'hdmi', 'headphone')):
            continue
        score = 0
        if 'usb' in phys or phys.startswith('usb-'):
            score += 100
        if '/input0' in phys:
            score += 20  # main HID collection on multi-interface boards
        if 'keychron' in name_l or 'keyboard' in name_l:
            score += 10
        if 'at translated' in name_l or phys.startswith('isa'):
            score -= 50  # laptop PS/2 — usually wrong when an external KB is plugged in
        score += min(len(keys), 200) / 200.0
        candidates.append((score, path, name, phys))
    candidates.sort(reverse=True)
    return candidates


def _resolve_keyboard_device_path(requested: str) -> str | None:
    """Resolve ``auto`` / empty to the best keyboard path, else return ``requested``."""
    requested = (requested or '').strip()
    if requested and requested.lower() != 'auto':
        return requested
    candidates = _list_keyboard_candidates()
    if not candidates:
        return None
    return candidates[0][1]

# Must mirror moveit_servo::StatusCode (status_codes.h) exactly — a prior
# version of this table was off by one, mislabeling JOINT_BOUND as HALT_FOR_COLLISION.
SERVO_STATUS_INVALID                              = -1
SERVO_STATUS_OK                                    = 0
SERVO_STATUS_DECELERATE_FOR_APPROACHING_SINGULARITY = 1
SERVO_STATUS_HALT_FOR_SINGULARITY                  = 2
SERVO_STATUS_DECELERATE_FOR_COLLISION              = 3
SERVO_STATUS_HALT_FOR_COLLISION                    = 4
SERVO_STATUS_JOINT_BOUND                           = 5
SERVO_STATUS_DECELERATE_FOR_LEAVING_SINGULARITY    = 6

SERVO_STATUS_NAMES = {
    SERVO_STATUS_INVALID: 'INVALID',
    SERVO_STATUS_OK: 'NO_WARNING',
    SERVO_STATUS_DECELERATE_FOR_APPROACHING_SINGULARITY: 'DECELERATE_FOR_APPROACHING_SINGULARITY',
    SERVO_STATUS_HALT_FOR_SINGULARITY: 'HALT_FOR_SINGULARITY',
    SERVO_STATUS_DECELERATE_FOR_COLLISION: 'DECELERATE_FOR_COLLISION',
    SERVO_STATUS_HALT_FOR_COLLISION: 'HALT_FOR_COLLISION',
    SERVO_STATUS_JOINT_BOUND: 'JOINT_BOUND',
    SERVO_STATUS_DECELERATE_FOR_LEAVING_SINGULARITY: 'DECELERATE_FOR_LEAVING_SINGULARITY',
}


class ServoController(Node):
    """ROS2 node that turns Cartesian velocity commands into MoveIt Servo messages.

    Declares and reads ROS parameters for speed, publish rate, command frame,
    safe pose and keyboard device path, publishes ``TwistStamped`` messages on
    a timer, and exposes helper methods to start/stop Servo and to drive the
    arm to a predefined safe pose via a ``FollowJointTrajectory`` action.
    """

    def __init__(self):
        """Initialize the node, declare parameters, and set up pub/sub/clients.

        Declares all ROS parameters (with defaults), reads their resolved
        values into instance attributes, zeroes the internal velocity state,
        creates the twist publisher, the Servo start/stop service clients,
        the trajectory action client, the publish timer, and the servo
        status subscription.
        """

        super().__init__('keyboard_servo_node')

        self.declare_parameter('linear_speed',  DEFAULT_LINEAR_SPEED)
        self.declare_parameter('angular_speed', DEFAULT_ANGULAR_SPEED)
        self.declare_parameter('publish_rate',  DEFAULT_PUBLISH_RATE)
        self.declare_parameter('linear_frame',  DEFAULT_LINEAR_FRAME)
        # Deprecated alias for linear_frame (older launch/params files).
        self.declare_parameter('command_frame', DEFAULT_LINEAR_FRAME)
        self.declare_parameter('ee_frame',      DEFAULT_EE_FRAME)
        self.declare_parameter('view_frame',    DEFAULT_VIEW_FRAME)
        # A / R move to this joint vector (defaults to poses.json "home").
        self.declare_parameter('safe_pose',     DEFAULT_HOME_POSE)
        # Empty (default): auto-pick '{end_effector}_home' if poses.json has
        # it (e.g. 'jaw_home'), else fall back to 'home'. Set explicitly to
        # pin a name regardless of end_effector.
        self.declare_parameter('home_pose_name', '')
        self.declare_parameter('keyboard_device_path', DEFAULT_KEYBOARD_DEVICE_PATH)
        self.declare_parameter('gamepad_shift_button', DEFAULT_GAMEPAD_SHIFT_BUTTON)
        self.declare_parameter('safe_pose_timeout', DEFAULT_SAFE_POSE_TIMEOUT)
        self.declare_parameter('plan_execute_velocity_scaling', DEFAULT_PLAN_EXECUTE_VELOCITY_SCALING)
        self.declare_parameter('plan_execute_acceleration_scaling', DEFAULT_PLAN_EXECUTE_ACCELERATION_SCALING)
        self.declare_parameter('gripper_speed', DEFAULT_GRIPPER_SPEED)
        self.declare_parameter('gripper_stroke', DEFAULT_GRIPPER_STROKE)
        self.declare_parameter('end_effector', 'jaw')
        self.declare_parameter('panel_pose_topic', DEFAULT_PANEL_POSE_TOPIC)
        self.declare_parameter('panel_visible_max_age_sec', DEFAULT_PANEL_VISIBLE_MAX_AGE_SEC)
        self.declare_parameter('panel_align_timeout', DEFAULT_PANEL_ALIGN_TIMEOUT)
        self.declare_parameter('activity_indicator_topic', DEFAULT_ACTIVITY_INDICATOR_TOPIC)
        # ERC-mandated at competition (see the constant's own comment) —
        # defaults to 0.0 (no wait) for bench testing, set explicitly to
        # ERC_REQUIRED_ACTIVITY_INDICATOR_PRE_DELAY_SEC (5.0) for real runs.
        self.declare_parameter('activity_indicator_pre_delay_sec', DEFAULT_ACTIVITY_INDICATOR_PRE_DELAY_SEC)

        self._linear_speed  = self.get_parameter('linear_speed').value
        self._angular_speed = self.get_parameter('angular_speed').value
        self._publish_rate  = self.get_parameter('publish_rate').value
        linear_frame = self.get_parameter('linear_frame').value
        command_frame = self.get_parameter('command_frame').value
        if linear_frame != DEFAULT_LINEAR_FRAME:
            self._linear_frame = linear_frame
        elif command_frame != DEFAULT_LINEAR_FRAME:
            self._linear_frame = command_frame
        else:
            self._linear_frame = DEFAULT_LINEAR_FRAME
        self._ee_frame      = self.get_parameter('ee_frame').value
        self._view_frame    = self.get_parameter('view_frame').value
        # Read before home_pose_name resolution below, which derives its
        # auto-pick from it.
        self._end_effector = self.get_parameter('end_effector').value
        home_pose_name_param = self.get_parameter('home_pose_name').value
        if home_pose_name_param:
            self._home_pose_name = home_pose_name_param
        else:
            auto_home_pose_name = f'{self._end_effector}_home'
            self._home_pose_name = (
                auto_home_pose_name
                if _load_home_pose_from_json(auto_home_pose_name) is not None
                else 'home'
            )
        # Prefer poses.json home unless the caller overrode safe_pose explicitly.
        pose_from_param = list(self.get_parameter('safe_pose').value)
        pose_from_json = _load_home_pose_from_json(self._home_pose_name)
        if pose_from_param == list(DEFAULT_HOME_POSE) and pose_from_json is not None:
            self._safe_pose = pose_from_json
            pose_source = f'poses.json["{self._home_pose_name}"]'
        else:
            self._safe_pose = pose_from_param
            pose_source = 'safe_pose parameter'
        # Sampling/drill modes' own A/R targets; each falls back to the jaw
        # home pose until poses.json has a real entry.
        self._sampling_home_pose_name = 'sampling_home'
        self._sampling_home_pose = self._load_tool_home_pose(self._sampling_home_pose_name)
        self._drill_home_pose_name = 'drill_home'
        self._drill_home_pose = self._load_tool_home_pose(self._drill_home_pose_name)
        # drill_sampling's B (sampling_home -> sampling_to_drill -> drill_home)
        # and Y (container detour, target depends on which mode is currently
        # active) — see GamepadInputLoop._handle_safe_pose/_on_joy.
        self._sampling_to_drill_pose_name = 'sampling_to_drill'
        self._sampling_to_drill_pose = self._load_tool_home_pose(self._sampling_to_drill_pose_name)
        self._sampling_container_pose_name = 'sampling_container'
        self._sampling_container_pose = self._load_tool_home_pose(self._sampling_container_pose_name)
        self._drill_container_pose_name = 'drill_container'
        self._drill_container_pose = self._load_tool_home_pose(self._drill_container_pose_name)
        # jaw's own probing mode — same _level_hold mechanism as sampling
        # mode (point tool-forward axis down, continuously, no pitch/yaw
        # stick input routed to it at all — see PROBING_POINT_AXIS/_publish/
        # GamepadInputLoop's axis mapping), just for end_effector='jaw'
        # instead of drill_sampling. B enters it (see _on_joy), A exits.
        self._jaw_probing_home_pose_name = 'jaw_probing_home'
        self._jaw_probing_home_pose = self._load_tool_home_pose(self._jaw_probing_home_pose_name)
        # jaw's own Y — two-leg move via jaw_probing_home, then (only on
        # success) jaw_probing_container — see move_jaw_probing_to_container.
        self._jaw_probing_container_pose_name = 'jaw_probing_container'
        self._jaw_probing_container_pose = self._load_tool_home_pose(self._jaw_probing_container_pose_name)
        self._keyboard_device_path = self.get_parameter('keyboard_device_path').value
        self._gamepad_shift_button = int(self.get_parameter('gamepad_shift_button').value)
        self._safe_pose_timeout    = self.get_parameter('safe_pose_timeout').value
        self._plan_execute_velocity_scaling = self.get_parameter('plan_execute_velocity_scaling').value
        self._plan_execute_acceleration_scaling = self.get_parameter('plan_execute_acceleration_scaling').value
        self._gripper_speed        = self.get_parameter('gripper_speed').value
        self._gripper_stroke       = self.get_parameter('gripper_stroke').value
        self._panel_pose_topic          = self.get_parameter('panel_pose_topic').value
        self._panel_visible_max_age_sec = self.get_parameter('panel_visible_max_age_sec').value
        self._panel_align_timeout       = self.get_parameter('panel_align_timeout').value
        self._activity_indicator_topic  = self.get_parameter('activity_indicator_topic').value
        self._activity_indicator_pre_delay_sec = self.get_parameter('activity_indicator_pre_delay_sec').value

        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0
        self.wx = 0.0
        self.wy = 0.0
        self.wz = 0.0
        # View-relative translation, kept separate from vx/vy/vz because it is
        # expressed in view_frame and only resolved to linear_frame at publish
        # time — the transform changes as the arm moves.
        self.view_vx = 0.0
        self.view_vy = 0.0
        self.view_vz = 0.0
        self._hold_quat = None
        # Scales HOLD_ANGULAR_GAIN/MAX in _orientation_hold for boosted push (set_velocity's hold_boost).
        self._hold_boost = 1.0
        # Selects _level_hold over _orientation_hold in _publish(); set via
        # set_sampling_mode()/set_drill_mode()/set_probing_mode(). sampling/
        # drill are mutually exclusive with each other (see those); probing
        # is jaw's own equivalent and never coexists with either in practice
        # since end_effector is fixed for the whole session.
        self._sampling_mode = False
        self._drill_mode = False
        self._probing_mode = False
        self._pitch_yaw_locked = False
        self._activity_delay_active = False
        self._joint_positions = {}

        self.gripper_vel = 0.0
        # Guess (closed) until the first /joint_states reading syncs this —
        # see _on_joint_state. Avoids commanding a jump from a wrong assumed
        # position on startup if the gripper wasn't actually closed.
        self._gripper_position = 0.0
        self._gripper_state_received = False
        self._last_gripper_tick_time = None

        self._panel_align_succeeded_once = False
        self._last_panel_visible_time = None
        self._motion_lock = threading.Lock()

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # Servo subscribes BEST_EFFORT; default RELIABLE can drop twists.
        self._pub = self.create_publisher(
            TwistStamped, 'servo_node/delta_twist_cmds', qos_profile_sensor_data
        )
        self._gripper_right_pub = self.create_publisher(
            Float64MultiArray, 'gripper_right_controller/commands', 10
        )
        self._gripper_left_pub = self.create_publisher(
            Float64MultiArray, 'gripper_left_controller/commands', 10
        )
        self._start_client = self.create_client(Trigger, 'servo_node/start_servo')
        self._stop_client  = self.create_client(Trigger, 'servo_node/stop_servo')
        self._panel_align_client = self.create_client(Trigger, 'panel_align/align')
        self._orient_gripper_client = self.create_client(Trigger, 'panel_align/orient_gripper')
        self._switch_client = self.create_client(
            SwitchController, 'controller_manager/switch_controller'
        )
        self._list_controllers_client = self.create_client(
            ListControllers, 'controller_manager/list_controllers'
        )
        # Home/mode-engage moves plan through this (OMPL, collision-checked
        # against the live planning scene) instead of a raw FollowJointTrajectory
        # goal — see _move_to_joint_positions_locked()/level_tool().
        self._move_group_client = ActionClient(self, MoveGroup, 'move_action')
        self._acquire_lock_client = self.create_client(
            AcquireArmMotionLock, 'arm_motion_lock/acquire')
        self._release_lock_client = self.create_client(
            ReleaseArmMotionLock, 'arm_motion_lock/release')
        # Identifies THIS process to arm_motion_lock_server — hostname
        # covers the actual cross-host case (GS vs Jetson), pid separates
        # two runs on the same host (e.g. sim + a stray leftover process).
        self._motion_lock_holder_id = f'{socket.gethostname()}/keyboard_servo_node/{os.getpid()}'
        self._js_sub = self.create_subscription(
            JointState, 'joint_states', self._on_joint_state, 10
        )
        self._panel_pose_sub = self.create_subscription(
            PoseStamped, self._panel_pose_topic, self._on_panel_pose, 10
        )
        # See run_planned_activity's own docstring (REQ-OPS-080/090/100)
        # — publishes activity-indicator INTENT; nothing in this repo drives
        # a physical lamp off it yet.
        self._activity_indicator_pub = self.create_publisher(
            ColorRGBA, self._activity_indicator_topic, 10
        )
        self._timer = self.create_timer(1.0 / self._publish_rate, self._publish)

        self._servo_status = SERVO_STATUS_OK
        self._status_sub = self.create_subscription(
            Int8,
            'servo_node/status',
            self._on_servo_status,
            10
        )

        self.get_logger().info(
            f'ServoController ready — '
            f'linear_speed={self._linear_speed}, '
            f'angular_speed={self._angular_speed}, '
            f'linear_frame={self._linear_frame} (XYZ + Servo twist frame), '
            f'ee_frame={self._ee_frame} (roll/pitch/yaw input), '
            f'view_frame={self._view_frame} (arrow-key translation), '
            f'A/R home from {pose_source}: {[round(v, 4) for v in self._safe_pose]}'
        )
        self.get_logger().warn(
            'Servo runs with check_collisions=true (self/scene proximity '
            'thresholds 0.003/0.005 m, see servo.yaml) — teleop WILL decelerate '
            'near a modeled collision, watch for "Close to a collision, '
            'decelerating" in servo_node\'s own log. Singularity deceleration is '
            'still effectively disabled (lower_singularity_threshold=10000.0).'
        )

    @property
    def linear_speed(self) -> float:
        """Return the configured linear speed scale, in meters per second."""
        return self._linear_speed

    @property
    def angular_speed(self) -> float:
        """Return the configured angular speed scale, in radians per second."""
        return self._angular_speed

    @property
    def gripper_speed(self) -> float:
        """Return the configured gripper speed, in meters per second."""
        return self._gripper_speed

    @property
    def gripper_stroke(self) -> float:
        """Return the configured gripper stroke (fully-open finger position, in meters)."""
        return self._gripper_stroke

    @property
    def keyboard_device_path(self) -> str:
        """Return the filesystem path of the keyboard input device (evdev)."""
        return self._keyboard_device_path

    @property
    def gamepad_shift_button(self) -> int:
        """Return the Joy button index that shifts the right stick."""
        return self._gamepad_shift_button

    @property
    def sampling_home_pose(self):
        """Return the joint targets A/R drives to while sampling mode is armed."""
        return self._sampling_home_pose

    @property
    def sampling_home_pose_name(self) -> str:
        """Return the poses.json key sampling_home_pose came from (for logging)."""
        return self._sampling_home_pose_name

    @property
    def drill_home_pose(self):
        """Return the joint targets A/R drives to while drill mode is armed."""
        return self._drill_home_pose

    @property
    def drill_home_pose_name(self) -> str:
        """Return the poses.json key drill_home_pose came from (for logging)."""
        return self._drill_home_pose_name

    @property
    def sampling_to_drill_pose(self):
        """Return the joint targets B's first leg drives to (drill_sampling)."""
        return self._sampling_to_drill_pose

    @property
    def sampling_to_drill_pose_name(self) -> str:
        """Return the poses.json key sampling_to_drill_pose came from (for logging)."""
        return self._sampling_to_drill_pose_name

    @property
    def sampling_container_pose(self):
        """Return the joint targets Y drives to while sampling mode is armed."""
        return self._sampling_container_pose

    @property
    def sampling_container_pose_name(self) -> str:
        """Return the poses.json key sampling_container_pose came from (for logging)."""
        return self._sampling_container_pose_name

    @property
    def drill_container_pose(self):
        """Return the joint targets Y drives to while drill mode is armed."""
        return self._drill_container_pose

    @property
    def drill_container_pose_name(self) -> str:
        """Return the poses.json key drill_container_pose came from (for logging)."""
        return self._drill_container_pose_name

    @property
    def jaw_probing_home_pose(self):
        """Return the joint targets B drives to for jaw's own probing mode."""
        return self._jaw_probing_home_pose

    @property
    def jaw_probing_home_pose_name(self) -> str:
        """Return the poses.json key jaw_probing_home_pose came from (for logging)."""
        return self._jaw_probing_home_pose_name

    @property
    def jaw_probing_container_pose(self):
        """Return the joint targets Y's second leg drives to (jaw probing)."""
        return self._jaw_probing_container_pose

    @property
    def jaw_probing_container_pose_name(self) -> str:
        """Return the poses.json key jaw_probing_container_pose came from (for logging)."""
        return self._jaw_probing_container_pose_name

    @property
    def end_effector(self) -> str:
        """Return the 'end_effector' parameter (which tool is mounted)."""
        return self._end_effector

    def _load_tool_home_pose(self, pose_name: str):
        """Load poses.json[pose_name], falling back to the jaw safe pose with a warning."""
        pose = _load_home_pose_from_json(pose_name)
        if pose is not None:
            return pose
        self.get_logger().warn(
            f'No valid "{pose_name}" entry in poses.json — its mode\'s A/R '
            'will use the jaw home pose until one is added.'
        )
        return self._safe_pose

    @property
    def has_remembered_panel_position(self) -> bool:
        """True once panel_align/align has succeeded at least once this session.

        panel_align_node owns the actual remembered joint target (and
        replans a fresh, collision-checked path to it every call) — this
        is just the local echo of "does one exist", for prompt/gating
        purposes only.
        """
        return self._panel_align_succeeded_once

    def set_velocity(self, vx=0.0, vy=0.0, vz=0.0,
                     wx=0.0, wy=0.0, wz=0.0,
                     view_vx=0.0, view_vy=0.0, view_vz=0.0,
                     hold_boost=1.0):
        """Set the current Cartesian velocity command.

        Args:
            vx: Linear velocity along global (mount) X, m/s.
            vy: Linear velocity along global (mount) Y, m/s.
            vz: Linear velocity along global (mount) Z, m/s.
            wx: Angular velocity about global X (roll of EEF), rad/s.
            wy: Angular velocity about global Y (pitch of EEF), rad/s.
            wz: Angular velocity about global Z (yaw of EEF), rad/s.
            view_vx: Linear velocity forward/back in ``view_frame``, m/s.
            view_vy: Linear velocity left/right in ``view_frame``, m/s.
            view_vz: Linear velocity up/down in ``view_frame``, m/s.

        Notes:
            Linear velocities are in ``linear_frame`` (default
            ``arm_mount_link``). Angular velocities are specified in
            ``ee_frame`` (default ``arm_tcp_link``) and rotated into
            ``linear_frame`` before publish. Servo's
            ``robot_link_command_frame`` must match ``linear_frame``.

            The ``view_*`` components are an independent translation set in
            ``view_frame`` (default ``arm_camera_link``); they are rotated
            into ``linear_frame`` and *added* to vx/vy/vz, so pressing keys
            from both sets at once simply sums the two motions.

            The three trailing arguments (plus hold_boost) are keyword-
            friendly on purpose: ``gamepad_servo_node`` calls this with six
            positional values and must keep working unchanged.

            hold_boost: Multiplier applied to _orientation_hold's gain/cap
                for this cycle. Pass the same multiplier used to scale the
                push (e.g. a held push-boost button) so the wrist's
                resistance to being bent by reaction torque grows with the
                push strength instead of staying fixed while the push
                triples.
        """
        if self._activity_delay_active:
            vx = vy = vz = wx = wy = wz = view_vx = view_vy = view_vz = 0.0
        self.vx = vx
        self.vy = vy
        self.vz = vz
        # pitch/yaw stay locked out after level_tool() until move_to_safe_pose()
        # clears it (see _pitch_yaw_locked's own comment) — roll (wz) and
        # translation are unaffected.
        self.wx = 0.0 if self._pitch_yaw_locked else wx
        self.wy = 0.0 if self._pitch_yaw_locked else wy
        self.wz = wz
        self.view_vx = view_vx
        self.view_vy = view_vy
        self.view_vz = view_vz
        self._hold_boost = hold_boost

    def set_gripper_velocity(self, vel: float):
        """Set the current gripper velocity command, in meters per second.

        Positive opens (toward gripper_stroke), negative closes (toward 0,
        the touching/closed position set by finger_x_closed in the URDF).
        """
        self.gripper_vel = 0.0 if self._activity_delay_active else vel

    def set_gripper_target(self, position: float):
        """Jump the visualized gripper straight to ``position`` (meters).

        For one-shot commands (gamepad SAFE_OPEN/SAFE_CLOSE) instead of
        the velocity-integrated path set_gripper_velocity() drives.
        Clamped and picked up by the very next _publish_gripper() tick,
        same as any other change to _gripper_position — including that
        method's own "withheld until the first real /joint_states sync"
        guard, so a press before that arrives doesn't slam a stale
        default onto gripper_right/left_controller.
        """
        self._gripper_position = max(0.0, min(self._gripper_stroke, position))

    def _on_joint_state(self, msg: JointState):
        for name, pos in zip(msg.name, msg.position):
            self._joint_positions[name] = float(pos)
        # Runs only until the first message that names GRIPPER_JOINT_NAME —
        # after that, _gripper_position is our own commanded state and the
        # real joint may legitimately lag behind it while moving.
        if not self._gripper_state_received and GRIPPER_JOINT_NAME in msg.name:
            index = msg.name.index(GRIPPER_JOINT_NAME)
            self._gripper_position = msg.position[index]
            self._gripper_state_received = True

    def stop(self):
        """Zero out all velocity components, halting Cartesian and gripper motion.

        Equivalent to calling ``set_velocity()`` with no arguments plus
        ``set_gripper_velocity(0.0)``. The publish timer keeps running on
        exit (it's spun on its own thread until destroy_node()), so every
        exit/stop path — ESC, X, a lost keyboard device, a /joy timeout —
        must go through this to also stop the gripper, not just the arm.
        """
        self.set_velocity()
        self.set_gripper_velocity(0.0)
        self._hold_quat = None

    def set_sampling_mode(self, active: bool):
        """Arm/disarm sampling mode; drops _hold_quat and disarms drill mode (mutually exclusive)."""
        self._sampling_mode = active
        if active:
            self._drill_mode = False
        self._hold_quat = None

    def set_drill_mode(self, active: bool):
        """Arm/disarm drill mode; drops _hold_quat and disarms sampling mode (mutually exclusive)."""
        self._drill_mode = active
        if active:
            self._sampling_mode = False
        self._hold_quat = None

    def set_probing_mode(self, active: bool):
        """Arm/disarm jaw's probing mode; drops _hold_quat. Independent of
        sampling/drill (never coexists with them — different end_effector)."""
        self._probing_mode = active
        self._hold_quat = None

    def _controller_states(self) -> dict:
        """Return {controller_name: state} via list_controllers, or {} on failure.

        STRICT switching errors on a controller already in its requested
        state (already active / already inactive), so callers use this to
        drop no-op entries before asking to switch.
        """
        if not self._list_controllers_client.wait_for_service(timeout_sec=2.0):
            return {}
        done_event = threading.Event()
        states = {}

        def _cb(future):
            try:
                for c in future.result().controller:
                    states[c.name] = c.state
            except Exception as exc:
                self.get_logger().error(f'list_controllers exception: {exc!r}')
            finally:
                done_event.set()

        future = self._list_controllers_client.call_async(ListControllers.Request())
        future.add_done_callback(_cb)
        done_event.wait(timeout=3.0)
        return states

    def _switch_controllers(self, activate, deactivate) -> bool:
        """Activate/deactivate ros2_control controllers (JTC <-> forward).

        Args:
            activate: Controllers to activate.
            deactivate: Controllers to deactivate.

        Returns:
            True if the switch service reported success, or if every
            controller was already in its requested state.
        """
        if not self._switch_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error('controller_manager/switch_controller unavailable')
            return False

        states = self._controller_states()
        if states:
            activate = [c for c in activate if states.get(c) != 'active']
            deactivate = [c for c in deactivate if states.get(c) == 'active']
            if not activate and not deactivate:
                return True
        # If list_controllers itself failed, fall through with the original,
        # unfiltered lists rather than silently dropping deactivate targets.

        req = SwitchController.Request()
        req.activate_controllers = list(activate)
        req.deactivate_controllers = list(deactivate)
        req.strictness = SwitchController.Request.STRICT
        req.activate_asap = True
        req.timeout = Duration(sec=3, nanosec=0)

        done_event = threading.Event()
        outcome = {'ok': False}

        def _cb(future):
            try:
                res = future.result()
                outcome['ok'] = bool(res.ok)
                if not res.ok:
                    self.get_logger().error(
                        f'Controller switch failed (activate={activate}, '
                        f'deactivate={deactivate})'
                    )
            except Exception as exc:
                self.get_logger().error(f'Controller switch exception: {exc!r}')
            finally:
                done_event.set()

        future = self._switch_client.call_async(req)
        future.add_done_callback(_cb)
        if not done_event.wait(timeout=5.0):
            self.get_logger().error('Controller switch timed out')
            return False
        if outcome['ok']:
            self.get_logger().info(
                f'Controllers: activate={list(activate)} deactivate={list(deactivate)}'
            )
        return outcome['ok']

    def use_trajectory_controller(self) -> bool:
        """Claim joints with JTC for home / Plan&Execute / teach_poses."""
        return self._switch_controllers(
            activate=[JTC_CONTROLLER_NAME],
            deactivate=[FORWARD_CONTROLLER_NAME],
        )

    def use_streaming_controller(self) -> bool:
        """Claim joints with forward position controller for Servo teleop."""
        return self._switch_controllers(
            activate=[FORWARD_CONTROLLER_NAME],
            deactivate=[JTC_CONTROLLER_NAME],
        )

    def _signal_activity_indicator(self, active: bool) -> None:
        """Publish the activity-indicator colour (best-effort, never raises).

        ``active`` picks ACTIVITY_INDICATOR_COLOR_ACTIVE (blue) or
        ACTIVITY_INDICATOR_COLOR_IDLE (off) — see that pair's own comment
        for why blue and for the caveat that nothing yet turns this into
        light on real hardware.
        """
        r, g, b, a = ACTIVITY_INDICATOR_COLOR_ACTIVE if active else ACTIVITY_INDICATOR_COLOR_IDLE
        msg = ColorRGBA(r=r, g=g, b=b, a=a)
        self._activity_indicator_pub.publish(msg)

    def run_planned_activity(self, action, label: str):
        """Run ``action`` (a zero-arg callable) gated by the ERC-mandated
        activity-indicator warm-up — the single choke point 'r'/'p'/'f'/'m'
        and their gamepad equivalents all route their actual move/align/
        level/orient call through, so REQ-OPS-080/090/100 are satisfied
        exactly once instead of separately at each call site.

        The wait duration is the ``activity_indicator_pre_delay_sec``
        parameter, NOT a hardcoded constant — REQ-OPS-090 requires >= 5.0s
        at actual competition (see ERC_REQUIRED_ACTIVITY_INDICATOR_PRE_DELAY_SEC),
        but the parameter defaults to 0.0 (no wait at all) for bench
        testing. Set it explicitly back to 5.0 (or higher) via launch
        argument / --ros-args -p before any competition run — nothing
        enforces that automatically.

        Sequence:
          1. stop() plus _activity_delay_active=True — set_velocity() and
             set_gripper_velocity() force zero while this is set, so held
             stick/key input during the wait can't sneak a command through
             (review-flagged: the sleep alone did not guarantee this).
          2. Publish the indicator ON (blue).
          3. Sleep activity_indicator_pre_delay_sec — REQ-OPS-090 at
             competition; skipped entirely when it's 0.0.
          4. Clear _activity_delay_active, then call ``action()`` —
             REQ-OPS-100's "at least 5s after the command was issued" (at
             competition, where the parameter is actually >= 5.0) — and
             return whatever it returns.
          5. Publish the indicator OFF once ``action()`` returns, success or
             failure alike (``finally``) — REQ-OPS-080's "continue to emit
             ... until all rover activities are finished".

        Callers are already running this on their own background thread
        (spawned from ``_read_loop``/``_on_joy``'s button dispatch), so the
        sleep here does not stall keyboard/joy event processing regardless
        of how long it's configured to be.
        """
        delay = self._activity_indicator_pre_delay_sec
        self.get_logger().info(
            f'{label}: activity indicator on, holding {delay:.1f}s before moving...'
        )
        self.stop()
        self._activity_delay_active = True
        self._signal_activity_indicator(True)
        try:
            time.sleep(delay)  # no-op for delay <= 0.0 (bench-testing default)
            self._activity_delay_active = False
            return action()
        finally:
            self._activity_delay_active = False
            self._signal_activity_indicator(False)

    def stop_servo(self) -> bool:
        """Call the Servo ``stop_servo`` service and wait for confirmation.

        Waits up to 2 seconds for the service to become available and up to
        10 seconds for the asynchronous call to complete. After Servo stops,
        re-activates the trajectory controller so home / Execute can run.

        Returns:
            bool: True if Servo stopped AND the trajectory controller took
            over; False otherwise.
        """
        if not self._stop_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn('Servo stop service not available')
            return False

        done_event = threading.Event()

        def _cb(future):
            """Mark the stop-servo call as complete (service response callback)."""
            done_event.set()

        future = self._stop_client.call_async(Trigger.Request())
        future.add_done_callback(_cb)

        if not done_event.wait(timeout=10.0):
            self.get_logger().warn('Servo stop timed out')
            return False

        # Prefer JTC when teleop is idle so Plan&Execute / home work.
        return self.use_trajectory_controller()

    def move_to_safe_pose(self, positions=None, name=None):
        """Stop motion and drive the arm to the configured home pose.

        Halts current velocity commands, confirms Servo has stopped and
        the trajectory controller is active, then delegates the actual
        move to ``_move_to_joint_positions``.

        Args:
            positions: Joint targets to use instead of ``self._safe_pose``
                (same order as ``HOME_POSE_JOINTS``) — e.g. drill mode
                passes ``self._drill_home_pose`` here so A/R lands
                somewhere else than the jaw home pose. Defaults to
                ``self._safe_pose`` when omitted.
            name: Label for the log line only (defaults to
                ``self._home_pose_name``); has no effect on motion.

        Returns:
            bool: True if the controller reported the goal SUCCEEDED with
            error code SUCCESSFUL; False if Servo could not be confirmed
            stopped, the trajectory controller could not be activated, the
            action server was unavailable, the goal was rejected, the
            trajectory was aborted/canceled or finished with a controller
            error, or no result arrived within the timeout.
        """
        target_positions = list(self._safe_pose) if positions is None else list(positions)
        target_name = self._home_pose_name if name is None else name

        self.stop()

        if not self.stop_servo():
            self.get_logger().error(
                'Could not confirm Servo stopped — aborting home move.'
            )
            return False

        if not self.use_trajectory_controller():
            self.get_logger().error(
                'Could not activate trajectory controller — aborting home move.'
            )
            return False

        success = self._move_to_joint_positions(target_positions, f'home ({target_name})')
        if success:
            self._pitch_yaw_locked = False
        return success

    def move_sampling_to_drill(self) -> bool:
        """Two-leg move for drill_sampling's B: sampling_to_drill waypoint,
        then — only if that leg actually succeeds — drill_home.

        Same stop/stop_servo/trajectory-controller setup as
        ``move_to_safe_pose``, done once up front for both legs (JTC stays
        claimed the whole time, no need to re-switch controllers between
        them). Each leg is its own collision-checked ``_move_to_joint_positions``
        call/lock acquisition — if the first leg is rejected or aborts, the
        second is never attempted and the arm is left holding wherever the
        first leg's plan stopped, not silently warped to drill_home.

        Returns:
            bool: True only if BOTH legs completed successfully (see
            ``move_to_safe_pose``'s own return-value contract for the exact
            failure modes each leg can hit).
        """
        self.stop()

        if not self.stop_servo():
            self.get_logger().error(
                'Could not confirm Servo stopped — aborting sampling_to_drill sequence.'
            )
            return False

        if not self.use_trajectory_controller():
            self.get_logger().error(
                'Could not activate trajectory controller — aborting sampling_to_drill sequence.'
            )
            return False

        waypoint_ok = self._move_to_joint_positions(
            list(self._sampling_to_drill_pose), f'{self._sampling_to_drill_pose_name} (waypoint)'
        )
        if not waypoint_ok:
            self.get_logger().error(
                f'{self._sampling_to_drill_pose_name} failed — aborting before the drill_home leg.'
            )
            return False

        final_ok = self._move_to_joint_positions(
            list(self._drill_home_pose), f'home ({self._drill_home_pose_name})'
        )
        if final_ok:
            self._pitch_yaw_locked = False
        return final_ok

    def move_jaw_probing_to_container(self) -> bool:
        """Two-leg move for jaw's Y: jaw_probing_home waypoint, then — only
        if that leg actually succeeds — jaw_probing_container.

        Mirrors ``move_sampling_to_drill``'s own structure exactly (same
        stop/stop_servo/trajectory-controller setup done once for both legs,
        same abort-before-second-leg-on-failure behavior) — see its
        docstring for the full reasoning, it applies here unchanged.

        Returns:
            bool: True only if BOTH legs completed successfully.
        """
        self.stop()

        if not self.stop_servo():
            self.get_logger().error(
                'Could not confirm Servo stopped — aborting jaw_probing_container sequence.'
            )
            return False

        if not self.use_trajectory_controller():
            self.get_logger().error(
                'Could not activate trajectory controller — aborting jaw_probing_container sequence.'
            )
            return False

        waypoint_ok = self._move_to_joint_positions(
            list(self._jaw_probing_home_pose), f'{self._jaw_probing_home_pose_name} (waypoint)'
        )
        if not waypoint_ok:
            self.get_logger().error(
                f'{self._jaw_probing_home_pose_name} failed — aborting before the '
                f'{self._jaw_probing_container_pose_name} leg.'
            )
            return False

        final_ok = self._move_to_joint_positions(
            list(self._jaw_probing_container_pose), f'home ({self._jaw_probing_container_pose_name})'
        )
        if final_ok:
            self._pitch_yaw_locked = False
        return final_ok

    def _move_to_joint_positions(self, target_positions, label: str) -> bool:
        """Plan (OMPL, collision-checked) and execute a move from the
        current joint state to ``target_positions``, then wait.

        Used by ``move_to_safe_pose()`` for the home move, and by
        panel_align_node.py's own remembered-position replay for the same
        reason: a raw point-to-point move can't tell whether the
        straight-line path to the target is actually clear. Callers are
        responsible for ``stop()``/``stop_servo()``/switching to the
        trajectory controller first.

        Non-blocking-locked (``_motion_lock``) rather than queued: two 'r'
        presses racing (see the lock's declaration in ``__init__``) should
        fail one of them cleanly, not silently submit a second goal that
        preempts the first mid-motion. ``_motion_lock`` alone only
        protects against racing WITHIN this process though — also takes
        ``arm_motion_lock()`` (see that module) so a home move here can't
        race a live align running in the separate panel_align_node
        process either, for the same reason.

        Blocks the calling thread up to ``safe_pose_timeout`` wall-clock
        seconds (<= 0 waits forever).

        Returns:
            bool: True if the controller reported the goal SUCCEEDED with
            error code SUCCESSFUL; False if another motion was already in
            progress (this process or panel_align_node's), the action
            server was unavailable, the goal was rejected, the trajectory
            was aborted/canceled or finished with a controller error, or
            no result arrived within the timeout.
        """
        if not self._motion_lock.acquire(blocking=False):
            self.get_logger().error(
                f'Another arm motion is already in progress — aborting {label}.'
            )
            return False
        lease_sec = self._safe_pose_timeout + 15.0 if self._safe_pose_timeout > 0.0 else 600.0
        try:
            try:
                with arm_motion_lock(
                        self._acquire_lock_client, self._release_lock_client,
                        self._motion_lock_holder_id, lease_sec):
                    return self._move_to_joint_positions_locked(target_positions, label)
            except ArmMotionBusy as exc:
                self.get_logger().error(f'{exc} — aborting {label}.')
                return False
        finally:
            self._motion_lock.release()

    def _move_to_joint_positions_locked(self, target_positions, label: str) -> bool:
        joint_constraints = [
            JointConstraint(joint_name=n, position=p, tolerance_above=0.005, tolerance_below=0.005, weight=1.0)
            for n, p in zip(HOME_POSE_JOINTS, target_positions)
        ]
        self.get_logger().info(
            f'Moving to {label} (collision-checked plan): {[round(v, 4) for v in target_positions]}'
        )
        success, error = self._execute_move_group_constraints(
            Constraints(joint_constraints=joint_constraints)
        )
        if not success:
            self.get_logger().error(f'{label} move failed: {error}')
            return False
        self.get_logger().info(f'{label} reached!')
        return True

    def level_tool(self) -> bool:
        """Move to ``LEVEL_TOOL_TARGET_POSE``, collision-checked.

        A fixed joint-space target rather than a computed one — see
        ``LEVEL_TOOL_TARGET_POSE``'s own comment for how it was picked and
        verified. No live geometry computation, which is why it's
        predictable.

        On success this also sets ``_pitch_yaw_locked`` (see its own
        comment) so teleop can't tilt the tool back off vertical
        afterward — translation and roll keep working normally.
        ``move_to_safe_pose()`` clears it.

        Returns:
            bool: True if move_group reported the goal SUCCEEDED; False
            on any failure — see ``move_to_safe_pose``'s own return-value
            contract, same failure modes apply here (Servo not stopped,
            trajectory controller unavailable, action server unavailable,
            goal rejected, no collision-free plan found, or no result
            within the timeout).
        """
        self.stop()

        if not self.stop_servo():
            self.get_logger().error(
                'Could not confirm Servo stopped — aborting level move.'
            )
            return False

        if not self.use_trajectory_controller():
            self.get_logger().error(
                'Could not activate trajectory controller — aborting level move.'
            )
            return False

        if not self._move_group_client.wait_for_server(timeout_sec=3.0):
            self.get_logger().error('move_group action server not available')
            return False

        joint_constraints = [
            JointConstraint(
                joint_name=name, position=pos,
                tolerance_above=0.005, tolerance_below=0.005, weight=1.0,
            )
            for name, pos in zip(HOME_POSE_JOINTS, LEVEL_TOOL_TARGET_POSE)
        ]

        self.get_logger().info('Leveling tool (fixed target pose, collision-checked plan)...')
        success, error = self._execute_move_group_constraints(
            Constraints(joint_constraints=joint_constraints)
        )
        if not success:
            self.get_logger().error(f'Level move failed: {error}')
            return False
        self._pitch_yaw_locked = True
        self.get_logger().info('Tool leveled! Pitch/yaw locked — "r" unlocks.')
        return True

    def _execute_move_group_constraints(self, constraints: Constraints) -> tuple[bool, str]:
        """Plan and execute a single move_action goal for ``constraints``.

        Shared by ``move_to_safe_pose()`` and ``level_tool()``. Returns
        ``(success, error)`` — ``error`` is empty on success, otherwise a
        short description (goal rejected, no result within the timeout, or
        the MoveIt status/error code on failure).

        Speed comes from the ``plan_execute_velocity_scaling``/
        ``plan_execute_acceleration_scaling`` params (fraction of each
        joint's own max_velocity/max_acceleration in joint_limits.yaml) —
        raise them if A/B/Y home moves feel slow or jerky (low acceleration
        in particular reads as jerkiness), lower them again if a move
        overshoots/oscillates or feels unsafe near people.
        """
        goal = MoveGroup.Goal()
        goal.request.group_name = MOVEIT_GROUP_NAME
        goal.request.pipeline_id = 'ompl'
        goal.request.goal_constraints = [constraints]
        goal.request.num_planning_attempts = 5
        goal.request.allowed_planning_time = 10.0

        goal.request.max_velocity_scaling_factor = self._plan_execute_velocity_scaling
        goal.request.max_acceleration_scaling_factor = self._plan_execute_acceleration_scaling
        goal.planning_options.plan_only = False  # plan then execute in one goal
        goal.planning_options.replan = True
        goal.planning_options.replan_attempts = 5

        done_event = threading.Event()
        outcome = {'success': False, 'error': ''}
        goal_handle_box = {}

        def result_cb(future):
            """Record the move_group result's status and MoveIt error code."""
            try:
                wrapped = future.result()
                result = wrapped.result
                if (wrapped.status == GoalStatus.STATUS_SUCCEEDED
                        and result.error_code.val == MoveItErrorCodes.SUCCESS):
                    outcome['success'] = True
                else:
                    outcome['error'] = (
                        f'goal status {wrapped.status}, '
                        f'MoveIt error code {result.error_code.val}'
                    )
            except Exception as e:
                outcome['error'] = f'failed to read result: {e!r}'
            finally:
                done_event.set()

        def goal_response_cb(future):
            """Handle move_action's goal-acceptance response."""
            try:
                goal_handle = future.result()
            except Exception as e:
                goal_handle = None
                outcome['error'] = f'goal request failed: {e!r}'
            if not goal_handle or not goal_handle.accepted:
                outcome['error'] = outcome['error'] or 'goal rejected'
                done_event.set()
                return
            goal_handle_box['gh'] = goal_handle
            goal_handle.get_result_async().add_done_callback(result_cb)

        future = self._move_group_client.send_goal_async(goal)
        future.add_done_callback(goal_response_cb)

        timeout = self._safe_pose_timeout if self._safe_pose_timeout > 0.0 else None
        if not done_event.wait(timeout=timeout):
            self.get_logger().warn(
                f'No move_group result within {self._safe_pose_timeout:.1f}s — '
                'controller may be unresponsive '
                '(raise the safe_pose_timeout parameter if the sim is just slow).'
            )
            gh = goal_handle_box.get('gh')
            if gh is not None:
                cancel_done = threading.Event()
                gh.cancel_goal_async().add_done_callback(lambda _f: cancel_done.set())
                cancel_done.wait(timeout=5.0)
            return False, 'timed out waiting for a result'
        return outcome['success'], outcome['error']

    def _on_panel_pose(self, msg: PoseStamped):
        """Record the arrival time of a panel_pose message (see is_panel_visible).

        panel_pose_fuser_node only publishes this when its own 2+-marker
        and disagreement checks pass (see panel_perception) — deliberately
        NOT the cheaper panel_visible (1+ marker) topic, since a
        single-marker pose estimate was confirmed live to be unreliable
        enough to compute a physically unreachable align target. Reading
        the same topic panel_align_node itself acts on means "the operator
        sees the prompt" and "align would actually accept this pose" agree.
        """
        self._last_panel_visible_time = self.get_clock().now()

    def is_panel_visible(self) -> bool:
        """Return True if a panel_pose message has arrived recently.

        This is the same 2+-marker bar panel_align_node itself requires
        before it will plan a move — see _on_panel_pose.
        """
        if self._last_panel_visible_time is None:
            return False
        age = (self.get_clock().now() - self._last_panel_visible_time).nanoseconds / 1e9
        return age <= self._panel_visible_max_age_sec

    def align_to_panel(self) -> bool:
        """Ask panel_align_node to align to the panel.

        This node no longer owns the remembered target itself —
        panel_align_node does (see its own align_to_panel(): live
        CV+MoveIt align on the first successful call, then a fresh
        collision-checked MoveIt plan to that exact remembered joint
        target on every call after, rather than a raw point-to-point
        move — see PR review for why the earlier point-to-point replay
        wasn't safe). This method is now just the service call:
        panel_align_node itself decides whether it can act (remembered
        target, or live panel visibility) and reports why not otherwise.

        Returns:
            bool: True if the arm reached a panel-aligned pose; False on
            any failure — see the logged message for which.
        """
        self.stop()

        if not self._panel_align_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error('panel_align/align service not available')
            return False

        done_event = threading.Event()
        outcome = {'success': False, 'message': ''}

        def _cb(future):
            try:
                result = future.result()
                outcome['success'] = result.success
                outcome['message'] = result.message
            except Exception as e:
                outcome['message'] = f'panel align call failed: {e!r}'
            finally:
                done_event.set()

        future = self._panel_align_client.call_async(Trigger.Request())
        future.add_done_callback(_cb)

        if not done_event.wait(timeout=self._panel_align_timeout):
            self.get_logger().warn(
                f'panel_align/align timed out after {self._panel_align_timeout:.1f}s'
            )
            return False
        if not outcome['success']:
            self.get_logger().error(f'Panel align failed: {outcome["message"]}')
            return False
        self.get_logger().info(f'Panel align succeeded: {outcome["message"]}')
        self._panel_align_succeeded_once = True
        return True

    def orient_gripper_to_panel(self) -> bool:
        """Ask panel_align_node to reorient just the gripper toward the
        remembered panel direction, without moving the arm back to the
        remembered position. Mirrors align_to_panel()'s call pattern.
        """
        self.stop()

        if not self._orient_gripper_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error('panel_align/orient_gripper service not available')
            return False

        done_event = threading.Event()
        outcome = {'success': False, 'message': ''}

        def _cb(future):
            try:
                result = future.result()
                outcome['success'] = result.success
                outcome['message'] = result.message
            except Exception as e:
                outcome['message'] = f'orient gripper call failed: {e!r}'
            finally:
                done_event.set()

        future = self._orient_gripper_client.call_async(Trigger.Request())
        future.add_done_callback(_cb)

        if not done_event.wait(timeout=self._panel_align_timeout):
            self.get_logger().warn(
                f'panel_align/orient_gripper timed out after {self._panel_align_timeout:.1f}s'
            )
            return False
        if not outcome['success']:
            self.get_logger().error(f'Gripper orient failed: {outcome["message"]}')
            return False
        self.get_logger().info(f'Gripper orient succeeded: {outcome["message"]}')
        return True

    def start_servo(self) -> bool:
        """Switch to streaming controller, then start MoveIt Servo.

        Waits up to 2 seconds for the service to become available and up to
        20 seconds for the call to complete. Falls back to the trajectory
        controller on any failure, so the joints are never left claimed by
        the streaming controller with Servo not actually running.

        Returns:
            bool: True if Servo confirmed it started.
        """
        if not self.use_streaming_controller():
            self.get_logger().error(
                'Could not activate forward position controller — Servo not started.'
            )
            return False
        if not self._start_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error('Servo start service not available')
            self.use_trajectory_controller()
            return False

        done_event = threading.Event()
        outcome = {'ok': False}

        def _cb(future):
            try:
                result = future.result()
                outcome['ok'] = bool(result.success)
                if not result.success:
                    self.get_logger().warn(f'Servo start failed: {result.message}')
            except Exception as e:
                self.get_logger().error(f'Servo start error: {e}')
            finally:
                done_event.set()

        future = self._start_client.call_async(Trigger.Request())
        future.add_done_callback(_cb)

        if not done_event.wait(timeout=20.0):
            self.get_logger().error('Servo start timed out')
            self.use_trajectory_controller()
            return False

        if outcome['ok']:
            self.get_logger().info('Servo started successfully')
        else:
            self.use_trajectory_controller()
        return outcome['ok']

    def _on_servo_status(self, msg: Int8):
        """Handle incoming Servo status updates.

        Automatically restarts Servo only when the status transitions into
        SERVO_STATUS_HALT_FOR_SINGULARITY, since that halt is typically
        recoverable by re-issuing start_servo (Servo re-attempts the motion
        away from the singular configuration). Other halt states — e.g.
        SERVO_STATUS_HALT_FOR_COLLISION — are intentionally NOT
        auto-recovered: they represent conditions where continuing motion
        could be unsafe, so they are left for the operator to resolve
        manually (e.g. by moving the arm clear via keyboard input, or a
        workspace/collision object review) rather than being silently
        retried.
        """
        code = msg.data
        if code != self._servo_status:
            name = SERVO_STATUS_NAMES.get(code, f'UNKNOWN({code})')
            if code in (SERVO_STATUS_OK, SERVO_STATUS_DECELERATE_FOR_APPROACHING_SINGULARITY,
                        SERVO_STATUS_DECELERATE_FOR_LEAVING_SINGULARITY):
                self.get_logger().info(f'Servo status -> {name}')
            elif code == SERVO_STATUS_DECELERATE_FOR_COLLISION:
                # Scales velocity down, doesn't zero it — motion continues.
                self.get_logger().warn(f'Servo status -> {name} (decelerating, not stopped)')
            else:
                self.get_logger().warn(f'Servo status -> {name} (motion stopped by Servo)')
            if code == SERVO_STATUS_HALT_FOR_SINGULARITY:
                self.start_servo()
        self._servo_status = code

    def _linear_in_command_frame(self):
        """Sum mount-frame and view-frame translation, both in ``linear_frame``.

        The view-frame part (arrow keys) is resolved through TF on every
        publish rather than once at key-press, so "forward" tracks the
        camera as the arm moves.

        Returns:
            tuple[float, float, float]: (vx, vy, vz) in ``linear_frame``.
            If TF is unavailable only the mount-frame part is returned, so
            W/S/A/D/Q/E keep working when the view frame does not resolve.
        """
        if (self.view_vx == 0.0 and self.view_vy == 0.0
                and self.view_vz == 0.0):
            return self.vx, self.vy, self.vz
        if self._view_frame == self._linear_frame:
            return (self.vx + self.view_vx,
                    self.vy + self.view_vy,
                    self.vz + self.view_vz)
        try:
            transform = self._tf_buffer.lookup_transform(
                self._linear_frame,
                self._view_frame,
                rclpy.time.Time(),
            )
        except TransformException as exc:
            self.get_logger().warn(
                f'TF {self._linear_frame} <- {self._view_frame} unavailable '
                f'({exc}); ignoring view-relative translation',
                throttle_duration_sec=2.0,
            )
            return self.vx, self.vy, self.vz
        rx, ry, rz = _rotate_vector_by_quat(
            transform.transform.rotation,
            self.view_vx, self.view_vy, self.view_vz,
        )
        return self.vx + rx, self.vy + ry, self.vz + rz

    def _angular_in_command_frame(self):
        """Map EEF-frame angular velocity into ``linear_frame`` via TF.

        Returns:
            tuple[float, float, float]: (wx, wy, wz) in ``linear_frame``.
            If TF is unavailable, returns (0, 0, 0).
        """
        if self.wx == 0.0 and self.wy == 0.0 and self.wz == 0.0:
            return 0.0, 0.0, 0.0
        if self._ee_frame == self._linear_frame:
            return self.wx, self.wy, self.wz
        try:
            transform = self._tf_buffer.lookup_transform(
                self._linear_frame,
                self._ee_frame,
                rclpy.time.Time(),
            )
        except TransformException as exc:
            self.get_logger().warn(
                f'TF {self._linear_frame} <- {self._ee_frame} unavailable '
                f'({exc}); publishing zero angular command',
                throttle_duration_sec=2.0,
            )
            return 0.0, 0.0, 0.0
        return _rotate_vector_by_quat(
            transform.transform.rotation, self.wx, self.wy, self.wz
        )

    def _orientation_hold(self, wx, wy, wz):
        """Keep TCP attitude while translating (Q/E pitch, also WASD).

        Does not scale linear speed. I/K/U/O/J/L still command rotation.
        """
        # View-relative keys translate just as much as W/S/A/D/Q/E do, so the
        # attitude hold has to engage for them too — otherwise arrow-key moves
        # would be the one path where TCP orientation is left to walk freely.
        driving_lin = (
            abs(self.vx) > HOLD_CMD_EPS or abs(self.vy) > HOLD_CMD_EPS or
            abs(self.vz) > HOLD_CMD_EPS or
            abs(self.view_vx) > HOLD_CMD_EPS or
            abs(self.view_vy) > HOLD_CMD_EPS or
            abs(self.view_vz) > HOLD_CMD_EPS
        )
        driving_ang = (
            abs(self.wx) > HOLD_CMD_EPS or abs(self.wy) > HOLD_CMD_EPS or
            abs(self.wz) > HOLD_CMD_EPS
        )
        if not driving_lin:
            self._hold_quat = None
            return wx, wy, wz
        if driving_ang:
            self._hold_quat = None
            return wx, wy, wz
        try:
            transform = self._tf_buffer.lookup_transform(
                self._linear_frame, self._ee_frame, rclpy.time.Time()
            )
        except TransformException:
            return wx, wy, wz
        q = transform.transform.rotation
        quat = Quaternion(x=q.x, y=q.y, z=q.z, w=q.w)
        if self._hold_quat is None:
            self._hold_quat = quat
            return wx, wy, wz
        q_err = _quat_multiply(_quat_conj(quat), self._hold_quat)
        rx, ry, rz = _quat_rotvec(q_err)
        hx, hy, hz = _rotate_vector_by_quat(quat, rx, ry, rz)
        # Scaled by hold_boost (see set_velocity) so a boosted push doesn't
        # out-muscle a fixed-strength hold — the wrist's resistance to being
        # bent grows with the push instead of staying constant.
        gain = HOLD_ANGULAR_GAIN * self._hold_boost
        cap = HOLD_ANGULAR_MAX * self._hold_boost
        wx = max(-cap, min(cap, wx + gain * hx))
        wy = max(-cap, min(cap, wy + gain * hy))
        wz = max(-cap, min(cap, wz + gain * hz))
        return wx, wy, wz

    def _level_hold(self, wx, wy, wz, local_axis=SAMPLING_POINT_AXIS):
        """Keep local_axis (in ee_frame) pointed at fixed DOWN_AXIS; wz (roll) passes through untouched.

        Shared by sampling mode and drill mode — they differ only in which
        local_axis (SAMPLING_POINT_AXIS vs DRILL_POINT_AXIS) gets aligned.
        Separate from _orientation_hold: that one drops on any rotation input
        and targets a captured reference, but here roll is continuous and the
        target is a fixed constant. Correction uses exact cross-product
        rotation (not _orientation_hold's small-angle quaternion-drop trick,
        which spun wildly on large initial errors here).
        """
        try:
            transform = self._tf_buffer.lookup_transform(
                self._linear_frame, self._ee_frame, rclpy.time.Time()
            )
        except TransformException:
            return wx, wy, wz
        q = transform.transform.rotation
        # Current pointing axis, in linear_frame.
        zx, zy, zz = _rotate_vector_by_quat(q, *local_axis)
        tx, ty, tz = DOWN_AXIS
        dot = max(-1.0, min(1.0, zx * tx + zy * ty + zz * tz))
        angle = math.acos(dot)
        axis_x = zy * tz - zz * ty
        axis_y = zz * tx - zx * tz
        axis_len = math.hypot(axis_x, axis_y)
        if axis_len < 1e-8:
            # Already pointing down (or exactly opposite — no unique
            # shortest-rotation axis either way; leave uncorrected rather
            # than divide by ~0).
            return wx, wy, wz
        scale = angle / axis_len
        hx, hy = axis_x * scale, axis_y * scale
        gain = HOLD_ANGULAR_GAIN * self._hold_boost
        cap = HOLD_ANGULAR_MAX * self._hold_boost
        wx = max(-cap, min(cap, wx + gain * hx))
        wy = max(-cap, min(cap, wy + gain * hy))
        return wx, wy, wz

    def _publish(self):
        """Publish twist in ``linear_frame`` (mount).

        Mount XYZ as-is, view-frame XYZ and EEF ω rotated in via TF.

        Servo ``robot_link_command_frame`` must equal ``linear_frame``.
        Publishing Cartesian velocity in the TCP frame was observed to
        produce almost no joint motion for mount-aligned X; mount-frame
        twists move the EEF correctly. The view-relative keys therefore
        resolve to mount here instead of switching the published frame.
        """
        vx, vy, vz = self._linear_in_command_frame()
        wx, wy, wz = self._angular_in_command_frame()
        if self._sampling_mode:
            wx, wy, wz = self._level_hold(wx, wy, wz, SAMPLING_POINT_AXIS)
        elif self._drill_mode:
            wx, wy, wz = self._level_hold(wx, wy, wz, DRILL_POINT_AXIS)
        elif self._probing_mode:
            wx, wy, wz = self._level_hold(wx, wy, wz, PROBING_POINT_AXIS)
        else:
            wx, wy, wz = self._orientation_hold(wx, wy, wz)

        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._linear_frame
        msg.twist.linear.x  = vx
        msg.twist.linear.y  = vy
        msg.twist.linear.z  = vz
        msg.twist.angular.x = wx
        msg.twist.angular.y = wy
        msg.twist.angular.z = wz
        self._pub.publish(msg)

        self._publish_gripper()

    def _publish_gripper(self):
        """Integrate gripper position from ``gripper_vel`` and publish.

        Withheld entirely until the first /joint_states sync (see
        _gripper_position above) — otherwise this would command the
        default-assumed 0.0 (closed) from the very first tick, which on a
        restart while the real gripper is open would slam it shut before
        any real state ever arrived.

        Runs every publish tick (100Hz) for smooth motion. Right and left
        each have their own single-joint controller (see GRIPPER_JOINT_NAME
        above) and are published separately every time; left is always
        -right.
        """
        if not self._gripper_state_received:
            return

        now = time.monotonic()
        if self.gripper_vel != 0.0:
            if self._last_gripper_tick_time is not None:
                dt = now - self._last_gripper_tick_time
                self._gripper_position += self.gripper_vel * dt
            self._last_gripper_tick_time = now
        else:
            self._last_gripper_tick_time = None

        self._gripper_position = max(0.0, min(self._gripper_stroke, self._gripper_position))

        self._gripper_right_pub.publish(Float64MultiArray(data=[self._gripper_position]))
        self._gripper_left_pub.publish(Float64MultiArray(data=[-self._gripper_position]))


HELP = """
╔══════════════════════════════════════════════════╗
║  Keyboard Servo — EEF control                    ║
╠══════════════════════════════════════════════════╣
║  EEF translation (absolute, arm_mount_link):     ║
║    w / s  — +X / -X                              ║
║    a / d  — +Y / -Y                              ║
║    q / e  — +Z / -Z                              ║
║  EEF translation (view-relative, camera):        ║
║    Up/Dn  — forward / back                       ║
║    Lt/Rt  — left / right                         ║
║    t / g  — up / down                            ║
║  EEF rotation (about arm_tcp_link):              ║
║    i / k  — pitch (wx)                           ║
║    u / o  — yaw   (wy)                           ║
║    j / l  — roll  (wz)                           ║
║  Gripper:                                        ║
║    b / v  — open / close                         ║
║  Panel:                                          ║
║    p      — align to detected panel              ║
║    m      — reorient gripper only (p first)      ║
║  Other:                                          ║
║    r      — move to home + start servo           ║
║    f      — level tool (collision-checked; locks ║
║             pitch/yaw after — 'r' unlocks)       ║
║  p/r/f wait 5s (activity indicator) before moving║
║    ESC/x  — exit                                 ║
╚══════════════════════════════════════════════════╝
"""


class KeyboardInputLoop:
    """Reads raw keyboard events via evdev and drives a ``ServoController``.

    Maintains the set of currently pressed direction keys, recomputes the
    combined Cartesian velocity whenever the set changes, and handles the
    special "safe pose" and "exit" key bindings.
    """

    _DIRECTIONS = {
        ecodes.KEY_W: ( 1.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0),
        ecodes.KEY_S: (-1.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0),
        ecodes.KEY_A: ( 0.0,  1.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0),
        ecodes.KEY_D: ( 0.0, -1.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0),
        ecodes.KEY_Q: ( 0.0,  0.0,  1.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0),
        ecodes.KEY_E: ( 0.0,  0.0, -1.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0),
        ecodes.KEY_I: ( 0.0,  0.0,  0.0,  1.0,  0.0,  0.0,  0.0,  0.0,  0.0),  # pitch
        ecodes.KEY_K: ( 0.0,  0.0,  0.0, -1.0,  0.0,  0.0,  0.0,  0.0,  0.0),
        ecodes.KEY_U: ( 0.0,  0.0,  0.0,  0.0, -1.0,  0.0,  0.0,  0.0,  0.0),  # yaw
        ecodes.KEY_O: ( 0.0,  0.0,  0.0,  0.0,  1.0,  0.0,  0.0,  0.0,  0.0),
        ecodes.KEY_J: ( 0.0,  0.0,  0.0,  0.0,  0.0,  1.0,  0.0,  0.0,  0.0),  # roll
        ecodes.KEY_L: ( 0.0,  0.0,  0.0,  0.0,  0.0, -1.0,  0.0,  0.0,  0.0),
        ecodes.KEY_UP:    ( 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,  1.0,  0.0,  0.0),
        ecodes.KEY_DOWN:  ( 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0,  0.0,  0.0),
        ecodes.KEY_LEFT:  ( 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,  0.0,  1.0,  0.0),
        ecodes.KEY_RIGHT: ( 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,  0.0, -1.0,  0.0),
        ecodes.KEY_T:     ( 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,  0.0,  0.0,  1.0),
        ecodes.KEY_G:     ( 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,  0.0,  0.0, -1.0),
    }

    _PITCH_YAW_KEYS = {ecodes.KEY_I, ecodes.KEY_K, ecodes.KEY_U, ecodes.KEY_O}

    _GRIPPER_KEYS = {
        ecodes.KEY_B: 1.0,   # open
        ecodes.KEY_V: -1.0,  # close
    }

    _KEYSTATE_UP = 0
    _KEYSTATE_DOWN = 1
    _KEYSTATE_REPEAT = 2

    _PANEL_LOST_CONFIRM_SEC = 2.0

    def __init__(self, controller: 'ServoController'):
        """Store a reference to the controller and initialize input state.

        Args:
            controller: The ``ServoController`` node that will receive
                velocity commands derived from keyboard input.
        """
        self._controller = controller
        self._linear_speed = controller.linear_speed
        self._angular_speed = controller.angular_speed
        self._gripper_speed = controller.gripper_speed
        self._device_path = controller.keyboard_device_path
        self._lock = threading.Lock()
        self._pressed = set()
        self._gripper_pressed = set()
        self._exit_event = threading.Event()
        self._devices = []
        self._read_thread = None
        self._servo_started = False
        self._safe_pose_running = threading.Lock()
        self._level_running = threading.Lock()
        self._panel_align_running = threading.Lock()
        self._orient_gripper_running = threading.Lock()

        self._panel_was_visible = False
        self._panel_prompt_pending = False
        self._panel_lost_since = None
        self._panel_watch_timer = controller.create_timer(0.2, self._check_panel_visibility)

    def _open_device(self) -> bool:
        """Open evdev keyboard(s) for teleop.

        ``keyboard_device_path:=auto`` (default) opens every QWERTY-capable
        keyboard and merges events. Pinning only ``/dev/input/event3`` (laptop
        AT keyboard) while typing on a Keychron produced zero key events.

        Returns:
            bool: True if at least one device opened.
        """
        requested = (self._device_path or '').strip()
        if requested and requested.lower() != 'auto':
            paths = [requested]
        else:
            paths = [p for _s, p, _n, _ph in _list_keyboard_candidates()]

        if not paths:
            self._controller.get_logger().error(
                'No suitable keyboard found via evdev. Set keyboard_device_path '
                'to an explicit /dev/input/eventN.'
            )
            return False

        opened = []
        for path in paths:
            try:
                device = evdev.InputDevice(path)
                # Older python-evdev has no set_nonblocking(); use fcntl.
                flag = fcntl.fcntl(device.fd, fcntl.F_GETFL)
                fcntl.fcntl(device.fd, fcntl.F_SETFL, flag | os.O_NONBLOCK)
            except (FileNotFoundError, PermissionError, OSError) as e:
                self._controller.get_logger().warn(f'Skipping {path!r}: {e!r}')
                continue
            opened.append(device)
            self._controller.get_logger().info(f'Listening on {path} ({device.name})')

        if not opened:
            self._controller.get_logger().error(f'Could not open any of {paths!r}')
            return False

        self._devices = opened
        self._device_path = opened[0].path
        print(
            '\nKeyboard input:\n  '
            + '\n  '.join(f'{d.name} ({d.path})' for d in opened)
            + '\nPress r = home + start Servo, then WASD to move.\n'
        )
        return True

    def _recompute_velocity(self):
        """Recompute and apply the combined velocity from all pressed keys.

        Sums the per-axis direction vectors of every currently pressed key
        in ``self._pressed``, scales the result by the controller's linear
        and angular speed settings, and forwards it to
        ``ServoController.set_velocity``.
        """
        vx = vy = vz = wx = wy = wz = 0.0
        cvx = cvy = cvz = 0.0
        with self._lock:
            active = list(self._pressed)
        for code in active:
            d = self._DIRECTIONS.get(code)
            if d is None:
                continue
            vx += d[0]
            vy += d[1]
            vz += d[2]
            wx += d[3]
            wy += d[4]
            wz += d[5]
            cvx += d[6]
            cvy += d[7]
            cvz += d[8]
        self._controller.set_velocity(
            vx * self._linear_speed, vy * self._linear_speed, vz * self._linear_speed,
            wx * self._angular_speed, wy * self._angular_speed, wz * self._angular_speed,
            view_vx=cvx * self._linear_speed,
            view_vy=cvy * self._linear_speed,
            view_vz=cvz * self._linear_speed,
        )

    def _recompute_gripper_velocity(self):
        """Recompute and apply gripper velocity from currently pressed b/v."""
        with self._lock:
            active = list(self._gripper_pressed)
        vel = sum(self._GRIPPER_KEYS.get(c, 0.0) for c in active) * self._gripper_speed
        self._controller.set_gripper_velocity(vel)

    def _check_panel_visibility(self):
        """Poll panel visibility and arm the one-shot prompt on a rising edge.

        Runs on a ROS timer (not tied to key events) since the panel can
        appear in frame without the operator pressing anything. Only
        fires the prompt/gate on a *confirmed* False -> True transition —
        'p' stays usable at any time the panel is visible (or a position
        is already remembered) regardless of this flag (see _read_loop),
        so re-detecting an already-visible panel does nothing here.

        "Confirmed" (via _panel_lost_since/_PANEL_LOST_CONFIRM_SEC) means
        is_panel_visible() must have been continuously False for a real
        stretch of time, not just one poll tick — a single marker at a
        marginal angle realistically drops detection for a second or more
        while the operator is actively driving, and reacting to every one
        of those gaps as "the panel left and came back" would re-fire the
        prompt on every flicker.
        """
        now = self._controller.get_clock().now()
        # Jaw-only, same as the P/M key handlers above and GamepadInputLoop's
        # equivalent lockout — suppresses the prompt entirely for other tools.
        raw_visible = (self._controller.end_effector == 'jaw'
                       and self._controller.is_panel_visible())
        if raw_visible:
            self._panel_lost_since = None
            visible = True
        elif self._panel_was_visible:
            if self._panel_lost_since is None:
                self._panel_lost_since = now
            lost_sec = (now - self._panel_lost_since).nanoseconds / 1e9
            visible = lost_sec < self._PANEL_LOST_CONFIRM_SEC
        else:
            visible = False

        if visible and not self._panel_was_visible:
            self._panel_prompt_pending = True
            self._controller.stop()
            print('\n>>> Panel detected! Press P to align to it. <<<')
        self._panel_was_visible = visible

    def _handle_panel_align(self):
        """Run panel alignment and hand control back to the operator either way.

        Unlike _handle_safe_pose, Servo is restarted on failure too: most
        align_to_panel() failures (stale detection, no remembered
        position, planning rejected) never move the arm at all, and even
        the execution-failure path only happens after a real
        collision-checked plan or a deterministic replay — so there's no
        equivalent of move_to_safe_pose()'s "arm may be stopped
        mid-trajectory, don't hand back control blindly" risk. Stranding
        the operator with no teleop just because alignment didn't succeed
        would defeat the point of it being an assistive, not mandatory,
        action.

        Guarded by a non-blocking lock (mirrors _handle_safe_pose) so a
        second 'p' press while an align is already in flight is ignored
        instead of racing a second FollowJointTrajectory/align call
        against the first — confirmed live: nothing here previously
        stopped that, since KEY_P has no already-pressed dedup the way
        direction/gripper keys do.
        """
        if not self._panel_align_running.acquire(blocking=False):
            return
        try:
            print('Aligning to panel...')
            if self._controller.run_planned_activity(self._controller.align_to_panel, 'align_to_panel'):
                print('Panel align succeeded.')
            else:
                print('Panel align failed.')
            print('Resuming manual control...')
            self._controller.start_servo()
        finally:
            self._panel_align_running.release()

    def _handle_orient_gripper(self):
        """Rotate the gripper in place to face the remembered panel direction."""
        if not self._orient_gripper_running.acquire(blocking=False):
            return
        try:
            print('Orienting gripper toward panel...')
            if self._controller.run_planned_activity(
                    self._controller.orient_gripper_to_panel, 'orient_gripper_to_panel'):
                print('Gripper oriented.')
            else:
                print('Gripper orient failed.')
            print('Resuming manual control...')
            self._controller.start_servo()
        finally:
            self._orient_gripper_running.release()

    def _handle_safe_pose(self):
        """Clear pressed keys, stop motion, and move to the safe pose.

        Servo is only started if the safe-pose move actually succeeded;
        starting it after a rejected/aborted/timed-out move would let the
        operator resume Cartesian teleop from a pose that never reached the
        intended safe configuration.

        Intended to run in its own thread (spawned from ``_read_loop``) so
        that the blocking safe-pose and servo-start calls do not stall
        keyboard event processing.
        """
        if not self._safe_pose_running.acquire(blocking=False):
            return
        try:
            with self._lock:
                self._pressed.clear()
                self._gripper_pressed.clear()
            self._controller.stop()
            print('Moving to home...')
            if self._controller.run_planned_activity(self._controller.move_to_safe_pose, 'move_to_safe_pose'):
                if self._exit_event.is_set():
                    print('Exit requested during home move — Servo not started.')
                    return
                print('Starting servo...')
                if self._controller.start_servo():
                    self._servo_started = True
                else:
                    print('Servo failed to start — staying on trajectory controller.')
            else:
                print('Home move failed — Servo not started.')
        finally:
            self._safe_pose_running.release()

    def _handle_level(self):
        """Reorient the tool straight down via a collision-checked plan.

        Servo is restarted regardless of outcome, same reasoning as
        move_to_safe_pose's failure path being the ONE case that does
        not: a rejected/failed plan here never moved the arm from
        wherever it already safely was, so there's no "resumed
        mid-trajectory" risk to guard against.
        """
        if not self._level_running.acquire(blocking=False):
            return
        try:
            print('Leveling tool...')
            if self._controller.run_planned_activity(self._controller.level_tool, 'level_tool'):
                print('Tool leveled.')
            else:
                print('Level move failed.')
            print('Resuming manual control...')
            self._controller.start_servo()
        finally:
            self._level_running.release()

    def _read_loop(self):
        """Continuously read raw key events from all opened keyboards."""
        try:
            while not self._exit_event.is_set():
                if not self._devices:
                    break
                try:
                    ready, _, _ = select.select(
                        [dev.fd for dev in self._devices], [], [], 0.2
                    )
                except (ValueError, OSError) as e:
                    self._controller.get_logger().error(f'Keyboard select failed: {e!r}')
                    break
                if not ready:
                    continue
                fd_to_dev = {dev.fd: dev for dev in self._devices}
                for fd in ready:
                    device = fd_to_dev.get(fd)
                    if device is None:
                        continue
                    try:
                        for event in device.read():
                            if event.type != ecodes.EV_KEY:
                                continue
                            code, value = event.code, event.value

                            if code in (ecodes.KEY_ESC, ecodes.KEY_X) and value == self._KEYSTATE_DOWN:
                                self._exit_event.set()
                                return

                            if code == ecodes.KEY_R and value == self._KEYSTATE_DOWN:
                                threading.Thread(
                                    target=self._handle_safe_pose, daemon=True
                                ).start()
                                continue

                            if code == ecodes.KEY_F and value == self._KEYSTATE_DOWN:
                                if SAMPLING_DRILL_MODES_ENABLED:
                                    threading.Thread(
                                        target=self._handle_level, daemon=True
                                    ).start()
                                else:
                                    print('level_tool() is disabled (SAMPLING_DRILL_MODES_ENABLED=False) — ignored.')
                                continue

                            if code == ecodes.KEY_P and value == self._KEYSTATE_DOWN:
                                if not self._servo_started:
                                    continue
                                # Panel align/orient is jaw-only — only the
                                # jaw gripper physically interacts with the
                                # panel (see GamepadInputLoop's identical
                                # lockout for the reasoning).
                                if self._controller.end_effector != 'jaw':
                                    print(
                                        f"Panel align is locked out with end_effector="
                                        f"'{self._controller.end_effector}' — only the "
                                        'jaw gripper interacts with the panel.'
                                    )
                                    continue
                                self._panel_prompt_pending = False
                                if (self._controller.is_panel_visible()
                                        or self._controller.has_remembered_panel_position):
                                    threading.Thread(
                                        target=self._handle_panel_align, daemon=True
                                    ).start()
                                else:
                                    print('No panel currently in view and no panel position remembered yet.')
                                continue

                            if code == ecodes.KEY_M and value == self._KEYSTATE_DOWN:
                                if not self._servo_started:
                                    continue
                                if self._controller.end_effector != 'jaw':
                                    print(
                                        f"Gripper orient is locked out with end_effector="
                                        f"'{self._controller.end_effector}' — only the "
                                        'jaw gripper interacts with the panel.'
                                    )
                                    continue
                                if self._controller.has_remembered_panel_position:
                                    threading.Thread(
                                        target=self._handle_orient_gripper, daemon=True
                                    ).start()
                                else:
                                    print('No panel position remembered yet — align (p) first.')
                                continue

                            if code in self._GRIPPER_KEYS:
                                if not self._servo_started:
                                    continue
                                if self._panel_prompt_pending and value == self._KEYSTATE_DOWN:
                                    self._panel_prompt_pending = False
                                    print('Continuing manual control (panel align not triggered).')
                                if value == self._KEYSTATE_DOWN:
                                    with self._lock:
                                        already_pressed = code in self._gripper_pressed
                                        other = (ecodes.KEY_V if code == ecodes.KEY_B
                                                 else ecodes.KEY_B)
                                        self._gripper_pressed.discard(other)
                                        self._gripper_pressed.add(code)
                                    if not already_pressed:
                                        self._recompute_gripper_velocity()
                                        key_name = ecodes.KEY[code].removeprefix('KEY_').lower()
                                        print(f'{key_name} gripper_vel={self._controller.gripper_vel:.4f}')
                                elif value == self._KEYSTATE_UP:
                                    with self._lock:
                                        self._gripper_pressed.discard(code)
                                    self._recompute_gripper_velocity()
                                continue

                            if code not in self._DIRECTIONS:
                                continue

                            if not self._servo_started:
                                continue

                            if self._panel_prompt_pending and value == self._KEYSTATE_DOWN:
                                self._panel_prompt_pending = False
                                print('Continuing manual control (panel align not triggered).')

                            if value == self._KEYSTATE_DOWN:
                                with self._lock:
                                    already_pressed = code in self._pressed
                                    self._pressed.add(code)
                                if not already_pressed:
                                    self._recompute_velocity()
                                    if not (self._controller._pitch_yaw_locked
                                            and code in self._PITCH_YAW_KEYS):
                                        key_name = ecodes.KEY[code].removeprefix('KEY_').lower()
                                        # view_* included or the arrow keys would
                                        # report all-zero and look like a no-op.
                                        print(
                                            f'{key_name} vx={self._controller.vx:.2f} '
                                            f'vy={self._controller.vy:.2f} '
                                            f'vz={self._controller.vz:.2f} '
                                            f'wx={self._controller.wx:.2f} '
                                            f'wy={self._controller.wy:.2f} '
                                            f'wz={self._controller.wz:.2f} '
                                            f'| fwd={self._controller.view_vx:.2f} '
                                            f'left={self._controller.view_vy:.2f} '
                                            f'up={self._controller.view_vz:.2f}'
                                        )
                            elif value == self._KEYSTATE_UP:
                                with self._lock:
                                    self._pressed.discard(code)
                                self._recompute_velocity()
                    except BlockingIOError:
                        continue
                    except OSError as e:
                        self._controller.get_logger().warn(
                            f'Lost keyboard {device.path}: {e!r}'
                        )
                        self._devices = [d for d in self._devices if d.fd != fd]
                        if not self._devices:
                            raise
        except OSError as e:
            self._controller.get_logger().error(f'Keyboard read loop failed: {e!r}')
        finally:
            self._exit_event.set()

    def run(self):
        """Open the keyboard device and run the input loop until exit.

        Opens the evdev device (returning early if this fails), prints the
        help banner, disables terminal echo where possible, starts the
        event-reading loop in a daemon thread, and blocks until the exit
        event is set. On exit, stops the controller's motion and restores
        the original terminal settings.
        """
        if not self._open_device():
            return
        print(HELP, flush=True)
        self._controller.get_logger().info(
            'Keyboard teleop ready. Press r = home + Servo, then WASD. '
            'Do not start a second keyboard_servo_node.'
        )

        # ros2 launch often has no TTY; fileno()/tcgetattr would abort the node.
        old_term_settings = None
        stdin_fd = None
        try:
            stdin_fd = sys.stdin.fileno()
        except (AttributeError, ValueError, OSError):
            stdin_fd = None
        if stdin_fd is not None:
            try:
                old_term_settings = termios.tcgetattr(stdin_fd)
                new_term_settings = termios.tcgetattr(stdin_fd)
                new_term_settings[3] &= ~termios.ECHO
                termios.tcsetattr(stdin_fd, termios.TCSADRAIN, new_term_settings)
            except (termios.error, OSError):
                old_term_settings = None

        self._read_thread = threading.Thread(target=self._read_loop, daemon=True)
        self._read_thread.start()
        try:
            self._exit_event.wait()
        finally:
            print('\nExiting...')
            with self._safe_pose_running:
                pass
            self._controller.stop()
            if old_term_settings is not None:
                termios.tcflush(stdin_fd, termios.TCIFLUSH)
                termios.tcsetattr(stdin_fd, termios.TCSADRAIN, old_term_settings)


class EndEffectorClient:
    """Publishes one-shot commands to end_effector_controller/command,
    tracks latest end_effector_controller/state. See arm_peripherals/
    end_effector_can_node.py for the actual CAN link."""

    _VALID_COMMANDS = frozenset({
        'open', 'close', 'drill_up', 'drill_down',
        'stop_step', 'stop_drill', 'lock', 'unlock', 'read_ph',
    })

    def __init__(self, node):
        self._pub = node.create_publisher(String, 'end_effector_controller/command', 10)
        self._state_sub = node.create_subscription(
            EndEffectorState, 'end_effector_controller/state', self._on_state, 10)
        self._logger = node.get_logger()
        self.load_right_g = 0.0
        self.load_left_g = 0.0
        self.connected = False

    def send(self, command: str) -> None:
        assert command in self._VALID_COMMANDS, command
        self._pub.publish(String(data=command))

    def _on_state(self, msg) -> None:
        self.load_right_g = msg.load_right_g
        self.load_left_g = msg.load_left_g
        self.connected = msg.connected
        self._logger.info(
            f'End-effector load: right={self.load_right_g}g left={self.load_left_g}g',
            throttle_duration_sec=1.0,
        )


GAMEPAD_HELP = """
╔══════════════════════════════════════════════╗
║  Gamepad — EEF control (view-relative)       ║
╠══════════════════════════════════════════════╣
║  Left stick   ←→  — left / right  (camera)   ║
║               ↑↓  — forward / back (camera)  ║
║  Right stick  ↑↓  — up / down     (camera)   ║
║               ←→  — yaw   (TCP)              ║
║  R1 + right   ↑↓  — pitch (TCP)              ║
║               ←→  — roll  (TCP)              ║
║  9 (button)       — push boost (hold)        ║
║  11 (button)      — jaw: gripper OPEN        ║
║                     drill_sampling: claw     ║
║                     OPEN, or drill UP in     ║
║                     drill mode               ║
║  13 (button)      — jaw: gripper CLOSE       ║
║                     drill_sampling: claw     ║
║                     CLOSE, or drill DOWN in  ║
║                     drill mode               ║
║  12 (button)      — drill_sampling: LOCK     ║
║  14 (button)      — drill_sampling: UNLOCK   ║
║   (SAFE gripper / claw+drill+lock firmware,  ║
║   over CAN — B/Y below own sampling/drill    ║
║   mode entry)                                ║
║   drill: right ←→ fwd/back, right ↑↓         ║
║   left/right, left ↑↓ up/down, no roll       ║
║   sampling: right ↑↓/←→ inverted, left ←→    ║
║   roll                                       ║
║   (pitch/yaw locked in both modes)           ║
║   A/R -> mode-specific home pose             ║
║  6 (button)       — point tool straight down ║
║   (collision-checked; sampling/drill modes)  ║
║  A                — home + start servo       ║
║  B                — go to sampling_home      ║
║  Y                — go to drill_home         ║
║  Button 7          — align to panel          ║
║  Button 8          — reorient gripper only   ║
║  A/B/Y/7/8: 5s indicator wait before move    ║
║  X                — exit                     ║
╚══════════════════════════════════════════════╝
"""


class GamepadInputLoop:
    """Reads sensor_msgs/Joy messages and drives a ``ServoController``.

    Replaces the raw-keyboard evdev input of ``KeyboardInputLoop`` with a
    subscription to the ``joy`` package's ``/joy`` topic — as this module's
    docstring already promises, ``ServoController`` itself needs no changes.

    Launch via ``arm_teleop/launch/gamepad.launch.py``, which starts
    ``game_controller_node`` (not plain ``joy_node``): it maps raw HID
    reports through SDL's GameController DB into a fixed canonical index
    order (A=0, X=2, LEFTSHOULDER=9, RIGHTSHOULDER=10, DPAD_UP=11,
    DPAD_LEFT=13, ... — same on every machine/controller SDL recognizes),
    instead of joy_node's raw per-device layout, which was observed to
    shift index depending on kernel/Bluetooth stack even for the identical
    physical pad. The indices below match that canonical table directly.

    Trigger axes (4/5) still get a per-axis rest sample while sticks are
    centered (``_trigger_amount``) as a safety net — GameController's
    TRIGGERLEFT/TRIGGERRIGHT are specified to rest at 0.0, but this costs
    nothing if that already holds.
    """

    # Gamepad translation is view-relative (camera frame), not mount-frame —
    # the operator is looking through the camera, so "forward" should follow
    # it. Mount-frame XYZ stays available on the keyboard.
    AXIS_LEFT_X = 0     # view +Y / -Y  (left / right)
    AXIS_LEFT_Y = 1     # view +X / -X  (forward / back)
    AXIS_RIGHT_X = 2    # yaw (-wy)     — roll  (-wz) while R1 held
    AXIS_RIGHT_Y = 3    # view +Z / -Z  — pitch (+wx) while R1 held
    AXIS_L2 = 4         # unmapped; used only for trigger rest calibration
    AXIS_R2 = 5

    # 'A' — jaw/astrobio: move to home + start servo. drill_sampling: always
    # goes to sampling_home instead (also how you get back to it from
    # sampling_container after Y — see BUTTON_DRILL_HOME below).
    BUTTON_SAFE_POSE = 0
    # 'B' — astrobio: locked out (no drill/sampling tool mounted).
    # drill_sampling: goes to drill_home — via the sampling_to_drill
    # transition waypoint first if drill mode isn't already armed (see
    # ServoController.move_sampling_to_drill), straight there otherwise
    # (already in the drill zone, e.g. returning from drill_container — see
    # _on_joy). jaw: arms probing mode instead (jaw_probing_home +
    # continuous point-down hold, same _level_hold mechanism as sampling
    # mode — see PROBING_POINT_AXIS); plain A (BUTTON_SAFE_POSE) is jaw's
    # way back out. One-shot, not a toggle — there is no button that turns
    # drill_sampling's own sampling/drill mode back off short of restarting
    # with a different end_effector.
    BUTTON_SAMPLING_HOME = 1
    BUTTON_EXIT = 2        # 'X' — exit
    # 'Y' — astrobio: locked out. drill_sampling: container detour, target
    # depends on whichever mode is CURRENTLY armed (not fixed) —
    # sampling_container while sampling_mode is active, drill_container
    # while drill_mode is active (see _on_joy) — armed by A/B above, not by
    # this button itself; one-shot per press, does not change which mode is
    # armed. jaw: two-leg move instead — jaw_probing_home waypoint, then
    # (only on success) jaw_probing_container (see
    # ServoController.move_jaw_probing_to_container) — arms probing mode on
    # success, unlike drill_sampling's own Y.
    BUTTON_DRILL_HOME = 3
    BUTTON_LB = 4          # unmapped (settle check only)
    # LEFTSHOULDER/L1. Held to scale up commanded velocity — raises the
    # per-cycle position step Servo re-anchors from, which is what caps
    # static push force (not kp/kd).
    BUTTON_PUSH_BOOST = 9
    # Bounded by max_cmd_speed_rad_s (arm_macro.xacro): the hardware
    # interface rate-limits actual motion to that many rad/s regardless
    # of what Servo asks for, so beyond a certain multiplier this stops
    # helping and max_cmd_speed_rad_s becomes the real ceiling instead.
    PUSH_BOOST_MULTIPLIER = 3.0
    # Shift button has no class constant — parameterized (DEFAULT_GAMEPAD_SHIFT_BUTTON),
    # read via self._shift_button, in case a pad's SDL mapping is ever wrong.

    BUTTON_GRIPPER_OPEN = 11
    BUTTON_GRIPPER_CLOSE = 13
    # DPAD_DOWN / DPAD_RIGHT — drill_sampling tool's electric lock
    # (EndEffectorClient.send('lock'/'unlock')). No jaw/astrobio
    # equivalent, so these are no-ops unless end_effector == 'drill_sampling'.
    BUTTON_LOCK = 12
    BUTTON_UNLOCK = 14
    # On-demand collision-checked "point straight down now" (mirrors
    # KeyboardInputLoop's 'f') — see ServoController.level_tool() for why
    # this needs to be its own move_group plan, not just a harder nudge
    # from _level_hold. Index unverified — see the comment above.
    BUTTON_LEVEL = 6
    # NOT 12/14 — those are BUTTON_LOCK/BUTTON_UNLOCK above, gated on
    # end_effector=='drill_sampling'. panel_align/orient_gripper have no
    # such gate (any end_effector, whenever a panel is visible/remembered),
    # so sharing an index with LOCK/UNLOCK let one button press fire both
    # a drill command and an arm motion at once (review-flagged). L3/R3
    # (stick clicks) are otherwise unused.
    BUTTON_PANEL_ALIGN = 7      # 'p' equivalent — align to detected panel
    BUTTON_ORIENT_GRIPPER = 8   # 'm' equivalent — reorient gripper only

    _DEADZONE = 0.2
    _JOY_TIMEOUT_SEC = 0.2
    _WATCHDOG_PERIOD_SEC = 0.1
    # See KeyboardInputLoop's identically-named constant.
    _PANEL_LOST_CONFIRM_SEC = 2.0

    def __init__(self, controller: 'ServoController'):
        """Store a reference to the controller and subscribe to ``/joy``.

        Args:
            controller: The ``ServoController`` node that will receive
                velocity commands derived from gamepad input. Its own
                ``create_subscription`` is reused for the ``/joy`` topic
                so the callback runs on the node's existing executor —
                no separate thread is needed, unlike the keyboard's
                blocking evdev read loop.
        """
        self._controller = controller
        self._linear_speed = controller.linear_speed
        self._angular_speed = controller.angular_speed
        self._shift_button = controller.gamepad_shift_button
        # Mirrors ServoController's own _sampling_mode/_drill_mode/_probing_mode
        # — needed here too since they decide how the right stick maps to
        # wx/wy/wz/view_vz.
        self._sampling_mode = False
        self._drill_mode = False
        self._probing_mode = False
        self._exit_event = threading.Event()
        self._prev_buttons = None
        self._safe_pose_running = threading.Lock()
        self._panel_align_running = threading.Lock()
        self._safe_pose_active = False
        self._level_running = threading.Lock()
        self._level_active = False
        self._orient_gripper_running = threading.Lock()
        self._orient_gripper_active = False
        self._prev_cmd = (0.0,) * 6

        self._last_joy_time = None
        self._joy_silent = False
        self._teleop_locked = True

        self._joy_settling = False
        # Per-trigger rest samples (axis index -> float). None until the
        # first centered settle so we do not assume both are +1.0.
        self._trigger_rest = {}

        self._panel_was_visible = False
        self._panel_prompt_pending = False
        self._panel_lost_since = None

        self._sub = controller.create_subscription(Joy, 'joy', self._on_joy, 10)
        self._watchdog_timer = controller.create_timer(
            self._WATCHDOG_PERIOD_SEC, self._check_joy_timeout
        )

        self._ee_client = EndEffectorClient(controller)

    @classmethod
    def _deadzone(cls, value: float) -> float:
        """Zero out small stick values so resting drift doesn't creep the arm."""
        return 0.0 if abs(value) < cls._DEADZONE else value

    def _axis(self, axes, index: int) -> float:
        """Return ``axes[index]`` with deadzone applied, or 0.0 if out of range."""
        if index >= len(axes):
            return 0.0
        return self._deadzone(axes[index])

    def _calibrate_triggers(self, axes) -> None:
        """Record L2/R2 rest values from a centered Joy snapshot."""
        for index in (self.AXIS_L2, self.AXIS_R2):
            if index < len(axes):
                self._trigger_rest[index] = float(axes[index])
        self._controller.get_logger().info(
            f'Trigger rest L2={self._trigger_rest.get(self.AXIS_L2, float("nan")):.2f} '
            f'R2={self._trigger_rest.get(self.AXIS_R2, float("nan")):.2f}'
        )

    def _sticks_centered(self, axes) -> bool:
        """True when both sticks are inside the deadzone (triggers ignored)."""
        return all(
            self._axis(axes, i) == 0.0
            for i in (self.AXIS_LEFT_X, self.AXIS_LEFT_Y,
                      self.AXIS_RIGHT_X, self.AXIS_RIGHT_Y)
        )

    def _trigger_amount(self, axes, index: int) -> float:
        """Return how far a trigger (L2/R2) is pressed: 0.0 .. 1.0.

        Supports both common rest conventions on this Stadia over ``joy_node``:
        rest near +1 (press toward -1) and rest near 0 (press toward ±1).
        Before calibration, only the +1-rest formula is used, and a raw
        value near 0 is treated as released — otherwise an uncalibrated
        0-rest R2 looks like a constant half-press (wz ≈ 0.5 * angular).
        """
        if index >= len(axes):
            return 0.0
        raw = float(axes[index])
        rest = self._trigger_rest.get(index)

        if rest is None:
            # Uncalibrated: never treat raw≈0 as a half-press (0-rest R2
            # phantom). Full +1-rest presses still register via raw≤-0.5.
            if raw >= 0.5 or raw <= -0.5:
                amount = (1.0 - raw) / 2.0
            else:
                amount = 0.0
        elif rest > 0.5:
            # Classic: +1 released → -1 fully pressed.
            amount = (rest - raw) / (rest - (-1.0))
        else:
            # Rest near 0: any deflection toward ±1 is a press.
            amount = abs(raw - rest)

        if amount < 0.0:
            amount = 0.0
        elif amount > 1.0:
            amount = 1.0
        return 0.0 if amount < self._DEADZONE else amount

    def _button_pressed(self, buttons, index: int) -> bool:
        """Return True if ``buttons[index]`` is currently held down."""
        return index < len(buttons) and buttons[index] == 1

    def _button_rising_edge(self, buttons, index: int) -> bool:
        """Return True if ``buttons[index]`` was just pressed this message.

        Requires a real previous reading — see ``_prev_buttons`` in
        ``__init__`` for why ``None`` (no baseline yet) always reports
        no edge rather than comparing against an assumed all-zero state.
        """
        if self._prev_buttons is None:
            return False
        was_pressed = index < len(self._prev_buttons) and self._prev_buttons[index] == 1
        return self._button_pressed(buttons, index) and not was_pressed

    @staticmethod
    def _active_label(view_vx, view_vy, view_vz, wx, wy, wz, shift: bool) -> str:
        """Describe which physical control(s) are driving a nonzero command."""
        # wy (yaw) only ever comes from the plain right stick, wx/wz (pitch/
        # roll) only from the shifted one — so the axes identify the source.
        parts = []
        if view_vx or view_vy:
            parts.append('left stick')
        if view_vz or wy:
            parts.append('right stick')
        if wx or wz:
            parts.append('R1+right stick')
        return '+'.join(parts) if parts else ('R1' if shift else 'idle')

    def _check_joy_timeout(self):
        """Stop the arm if no ``/joy`` message has arrived recently.

        Runs on the node's own timer, independent of message arrival, so
        a disconnected controller or a dead ``joy_node`` can't
        leave the last commanded velocity republishing forever — Servo's
        own command timeout does not help here because ServoController
        keeps re-publishing that last twist every tick regardless of
        whether new input has arrived. This only zeroes the current
        command; it does not lock teleop out, so control resumes on its
        own the moment ``/joy`` messages start arriving again (e.g. a
        Bluetooth reconnect) — no separate re-arm step.
        """
        if self._last_joy_time is None:
            return
        elapsed = (self._controller.get_clock().now() - self._last_joy_time).nanoseconds / 1e9
        if elapsed > self._JOY_TIMEOUT_SEC:
            if not self._joy_silent:
                self._joy_silent = True
                self._joy_settling = True
                self._trigger_rest.clear()
                self._prev_buttons = None
                self._controller.get_logger().warn(
                    f'/joy timed out after {elapsed:.2f}s — stopping arm.'
                )
            self._controller.stop()

    def _log_raw_joy(self, axes, buttons, note: str = ''):
        """Log the full Joy message — an out-of-range index fails silently otherwise."""
        axes_str = ', '.join(f'{i}:{v:+.2f}' for i, v in enumerate(axes))
        buttons_str = ', '.join(f'{i}:{b}' for i, b in enumerate(buttons))
        self._controller.get_logger().info(
            f'/joy raw{note} — axes[{len(axes)}]: {{{axes_str}}}  '
            f'buttons[{len(buttons)}]: {{{buttons_str}}}'
        )

    def _on_joy(self, msg: Joy):
        """Translate one Joy snapshot into a velocity command and edge-triggered actions."""
        axes = msg.axes
        buttons = msg.buttons

        if self._joy_silent:
            self._joy_silent = False
            self._controller.get_logger().info('/joy resumed.')
        self._last_joy_time = self._controller.get_clock().now()

        if self._prev_buttons is None:
            # First message (also re-fires after a /joy dropout). Warn early
            # if gamepad_shift_button is out of range for this controller.
            self._log_raw_joy(axes, buttons, ' (first message)')
            if self._shift_button >= len(buttons):
                self._controller.get_logger().warn(
                    f'gamepad_shift_button={self._shift_button} but this '
                    f'/joy only reports {len(buttons)} button(s) '
                    f'(0..{len(buttons) - 1}) — that index can never be '
                    f'pressed. Pick a real index from the buttons[] list above.'
                )

        safe_pose_pressed = self._button_rising_edge(buttons, self.BUTTON_SAFE_POSE)
        sampling_home_pressed = self._button_rising_edge(buttons, self.BUTTON_SAMPLING_HOME)
        drill_home_pressed = self._button_rising_edge(buttons, self.BUTTON_DRILL_HOME)
        exit_pressed = self._button_rising_edge(buttons, self.BUTTON_EXIT)
        gripper_open_pressed = self._button_rising_edge(buttons, self.BUTTON_GRIPPER_OPEN)
        gripper_close_pressed = self._button_rising_edge(buttons, self.BUTTON_GRIPPER_CLOSE)
        lock_pressed = self._button_rising_edge(buttons, self.BUTTON_LOCK)
        unlock_pressed = self._button_rising_edge(buttons, self.BUTTON_UNLOCK)
        level_pressed = self._button_rising_edge(buttons, self.BUTTON_LEVEL)
        panel_align_pressed = self._button_rising_edge(buttons, self.BUTTON_PANEL_ALIGN)
        orient_gripper_pressed = self._button_rising_edge(buttons, self.BUTTON_ORIENT_GRIPPER)

        # Log the raw state on any button change, not just the mapped ones,
        # so an unmapped shift button still shows up.
        if self._prev_buttons is not None:
            width = max(len(buttons), len(self._prev_buttons))
            changed = [
                i for i in range(width)
                if self._button_pressed(buttons, i)
                != (i < len(self._prev_buttons) and self._prev_buttons[i] == 1)
            ]
            if changed:
                self._log_raw_joy(
                    axes, buttons,
                    f' (button(s) {changed} changed; shift configured as '
                    f'{self._shift_button})',
                )

        self._prev_buttons = list(buttons)

        if exit_pressed:
            self._exit_event.set()

        end_effector = self._controller.end_effector

        if gripper_open_pressed:
            if end_effector == 'drill_sampling':
                if self._drill_mode:
                    self._ee_client.send('drill_up')
                    self._controller.get_logger().info('Drill: UP sent.')
                else:
                    self._ee_client.send('open')
                    self._controller.get_logger().info('Claw: OPEN sent.')
            elif end_effector == 'astrobio':
                self._ee_client.send('open')
                self._controller.get_logger().info('Astrobio: SUCK ON sent.')
            else:
                self._ee_client.send('open')
                self._controller.set_gripper_target(self._controller.gripper_stroke)
                self._controller.get_logger().info('Gripper: SAFE_OPEN sent.')

        if gripper_close_pressed:
            if end_effector == 'drill_sampling':
                if self._drill_mode:
                    self._ee_client.send('drill_down')
                    self._controller.get_logger().info('Drill: DOWN sent.')
                else:
                    self._ee_client.send('close')
                    self._controller.get_logger().info('Claw: CLOSE sent.')
            elif end_effector == 'astrobio':
                self._ee_client.send('close')
                self._controller.get_logger().info('Astrobio: SUCK OFF sent.')
            else:
                self._ee_client.send('close')
                self._controller.set_gripper_target(0.0)
                self._controller.get_logger().info('Gripper: SAFE_CLOSE sent.')

        if lock_pressed and end_effector == 'drill_sampling':
            self._ee_client.send('lock')
            self._controller.get_logger().info('Claw/drill: LOCK sent.')
        elif lock_pressed and end_effector == 'astrobio':
            self._ee_client.send('read_ph')
            self._controller.get_logger().info('Astrobio: READ PH sent.')

        if unlock_pressed and end_effector == 'drill_sampling':
            self._ee_client.send('unlock')
            self._controller.get_logger().info('Claw/drill: UNLOCK sent.')

        # drill_sampling's own A/B/Y sequence (button labels below match the
        # physical buttons, not the old sampling/drill "home" naming):
        #   A -> sampling_home (always; also how you get back to it from
        #        sampling_container). For jaw, plain A/home also exits
        #        probing mode if it was armed (see _handle_safe_pose).
        #   B -> drill_home, routed by whichever mode is currently active:
        #        via the sampling_to_drill transition waypoint first if NOT
        #        already in drill mode (see
        #        ServoController.move_sampling_to_drill), or straight there
        #        if drill mode is already armed (e.g. returning from
        #        drill_container — no need to detour back through the
        #        sampling-side waypoint just to reach a pose you're already
        #        near). For jaw specifically, B instead arms probing mode
        #        (jaw_probing_home + continuous point-down hold, same
        #        _level_hold mechanism as sampling mode — see
        #        PROBING_POINT_AXIS) — jaw has no drill/sampling tool, so
        #        this button was otherwise unused for it.
        #   Y -> drill_sampling: container detour, routed by whichever mode
        #        is currently active: sampling_container while sampling_mode
        #        is armed, drill_container while drill_mode is armed. Does
        #        not change which mode is armed (see _handle_safe_pose's
        #        docstring). jaw: two-leg move instead — jaw_probing_home
        #        waypoint, then (only on success) jaw_probing_container (see
        #        ServoController.move_jaw_probing_to_container), arming
        #        probing mode on success — jaw's own equivalent of B's
        #        sampling_to_drill chain, just on Y instead since jaw
        #        already uses B to arm probing mode via jaw_probing_home
        #        alone.
        if safe_pose_pressed and end_effector == 'drill_sampling':
            self._controller.get_logger().info(
                'A pressed (drill_sampling) — going straight to sampling_home.'
            )
            threading.Thread(
                target=self._handle_safe_pose, args=('sampling',), daemon=True
            ).start()
        elif safe_pose_pressed:
            threading.Thread(target=self._handle_safe_pose, daemon=True).start()

        if sampling_home_pressed and not SAMPLING_DRILL_MODES_ENABLED:
            self._controller.get_logger().warn(
                'Sampling home is disabled (SAMPLING_DRILL_MODES_ENABLED=False) — ignored.'
            )
        elif sampling_home_pressed and end_effector == 'astrobio':
            self._controller.get_logger().warn(
                f"B (sampling_home) is locked out with end_effector='{end_effector}' "
                '— no drill/sampling tool mounted.'
            )
        elif sampling_home_pressed and end_effector == 'jaw':
            self._controller.get_logger().info(
                'B pressed (jaw) — going to jaw_probing_home (probing mode).'
            )
            threading.Thread(
                target=self._handle_safe_pose, args=('probing',), daemon=True
            ).start()
        elif sampling_home_pressed and end_effector == 'drill_sampling':
            if self._drill_mode:
                # Already in drill mode (e.g. sitting at drill_container via
                # Y) — go straight back to drill_home, no need to route
                # through the sampling-side transition waypoint again.
                self._controller.get_logger().info(
                    'B pressed (drill_sampling, already drill mode) — '
                    'going straight to drill_home.'
                )
                threading.Thread(
                    target=self._handle_safe_pose, args=('drill',), daemon=True
                ).start()
            else:
                self._controller.get_logger().info(
                    'B pressed (drill_sampling) — going to sampling_to_drill, then drill_home.'
                )
                threading.Thread(
                    target=self._handle_safe_pose, args=('sampling_to_drill',), daemon=True
                ).start()
        elif sampling_home_pressed:
            # Mode is NOT flipped here — only once the home move actually
            # succeeds (see _handle_safe_pose's target_mode handling).
            # Flipping it immediately would change axis mapping/collision
            # assumptions for a physical configuration the arm may never
            # reach (review-flagged: a rejected/failed move used to leave
            # software mode and physical pose out of sync).
            self._controller.get_logger().info(
                'B pressed — going straight to sampling_home.'
            )
            threading.Thread(
                target=self._handle_safe_pose, args=('sampling',), daemon=True
            ).start()

        if drill_home_pressed and not SAMPLING_DRILL_MODES_ENABLED:
            self._controller.get_logger().warn(
                'Drill home is disabled (SAMPLING_DRILL_MODES_ENABLED=False) — ignored.'
            )
        elif drill_home_pressed and end_effector == 'astrobio':
            self._controller.get_logger().warn(
                f"Y (drill_home) is locked out with end_effector='{end_effector}' "
                '— no drill/sampling tool mounted.'
            )
        elif drill_home_pressed and end_effector == 'jaw':
            self._controller.get_logger().info(
                'Y pressed (jaw) — going to jaw_probing_home, then jaw_probing_container.'
            )
            threading.Thread(
                target=self._handle_safe_pose, args=('jaw_probing_container',), daemon=True
            ).start()
        elif drill_home_pressed and end_effector == 'drill_sampling':
            if self._drill_mode:
                container_target = 'drill_container'
            elif self._sampling_mode:
                container_target = 'sampling_container'
            else:
                container_target = None
            if container_target is None:
                self._controller.get_logger().warn(
                    'Y pressed but neither sampling_home nor drill_home has '
                    'been reached yet — press A first.'
                )
            else:
                self._controller.get_logger().info(
                    f'Y pressed (drill_sampling) — going to {container_target}.'
                )
                threading.Thread(
                    target=self._handle_safe_pose, args=(container_target,), daemon=True
                ).start()
        elif drill_home_pressed:
            # See sampling_home_pressed's comment above — same deferred-
            # mode-switch reasoning applies here.
            self._controller.get_logger().info(
                'Y pressed — going straight to drill_home.'
            )
            threading.Thread(
                target=self._handle_safe_pose, args=('drill',), daemon=True
            ).start()

        if level_pressed and not SAMPLING_DRILL_MODES_ENABLED:
            self._controller.get_logger().warn(
                'level_tool() is disabled (SAMPLING_DRILL_MODES_ENABLED=False) — ignored.'
            )
        elif level_pressed:
            threading.Thread(target=self._handle_level, daemon=True).start()

        if self._teleop_locked or self._safe_pose_active or self._level_active or self._orient_gripper_active:
            self._controller.stop()
            return

        if self._joy_settling:
            centered = (
                self._sticks_centered(axes)
                and self._trigger_amount(axes, self.AXIS_L2) == 0.0
                and self._trigger_amount(axes, self.AXIS_R2) == 0.0
                and not self._button_pressed(buttons, self.BUTTON_PUSH_BOOST)
                and not self._button_pressed(buttons, self._shift_button)
            )
            self._controller.stop()
            if centered:
                self._calibrate_triggers(axes)
                self._joy_settling = False
                self._controller.get_logger().info('Sticks centered — resuming control.')
            return

        if not self._trigger_rest and self._sticks_centered(axes):
            self._calibrate_triggers(axes)

        # Held BUTTON_PUSH_BOOST scales up the commanded velocity — see its
        # comment for why that's what actually raises the arm's static
        # push force against resistance, not kp/kd/gravity-ff.
        boost = (self.PUSH_BOOST_MULTIPLIER
                 if self._button_pressed(buttons, self.BUTTON_PUSH_BOOST) else 1.0)
        linear_speed = self._linear_speed * boost
        angular_speed = self._angular_speed * boost

        now = self._controller.get_clock().now()
        # Panel detection/prompting is jaw-only — see panel_align_pressed's
        # own lockout below for why (only the jaw gripper interacts with
        # the panel). Suppressed entirely for other tools, or a
        # drill/astrobio operator would get an unhelpful "press button 7"
        # prompt (and an unwanted stop()) for a feature that just refuses.
        raw_panel_visible = end_effector == 'jaw' and self._controller.is_panel_visible()
        if raw_panel_visible:
            self._panel_lost_since = None
            panel_visible = True
        elif self._panel_was_visible:
            if self._panel_lost_since is None:
                self._panel_lost_since = now
            panel_visible = (now - self._panel_lost_since).nanoseconds / 1e9 < self._PANEL_LOST_CONFIRM_SEC
        else:
            panel_visible = False

        if panel_visible and not self._panel_was_visible:
            self._panel_prompt_pending = True
            self._controller.stop()
            print('\n>>> Panel detected! Press button 7 to align to it. <<<')
        self._panel_was_visible = panel_visible

        if panel_align_pressed and end_effector != 'jaw':
            self._controller.get_logger().warn(
                f"Panel align is locked out with end_effector='{end_effector}' "
                '— only the jaw gripper interacts with the panel.'
            )
        elif panel_align_pressed:
            self._panel_prompt_pending = False
            if panel_visible or self._controller.has_remembered_panel_position:
                threading.Thread(target=self._handle_panel_align, daemon=True).start()
            else:
                print('No panel currently in view and no panel position remembered yet.')

        if orient_gripper_pressed and end_effector != 'jaw':
            self._controller.get_logger().warn(
                f"Gripper orient is locked out with end_effector='{end_effector}' "
                '— only the jaw gripper interacts with the panel.'
            )
        elif orient_gripper_pressed:
            if self._controller.has_remembered_panel_position:
                threading.Thread(target=self._handle_orient_gripper, daemon=True).start()
            else:
                print('No panel position remembered yet — align (button 7) first.')

        if self._panel_prompt_pending:
            # Same "any real stick input dismisses it" rule as
            # KeyboardInputLoop — checked on just the 4 sticks, since the
            # gripper/panel buttons aren't "driving".
            any_axis_active = (
                self._axis(axes, self.AXIS_LEFT_X) != 0.0
                or self._axis(axes, self.AXIS_LEFT_Y) != 0.0
                or self._axis(axes, self.AXIS_RIGHT_X) != 0.0
                or self._axis(axes, self.AXIS_RIGHT_Y) != 0.0
            )
            if any_axis_active:
                self._panel_prompt_pending = False
                print('Continuing manual control (panel align not triggered).')
            else:
                self._controller.stop()
                return

        left_x = self._axis(axes, self.AXIS_LEFT_X)
        left_y = self._axis(axes, self.AXIS_LEFT_Y)
        view_vx = left_y * linear_speed

        right_x = self._axis(axes, self.AXIS_RIGHT_X)
        right_y = self._axis(axes, self.AXIS_RIGHT_Y)
        shift = self._button_pressed(buttons, self._shift_button)

        view_vz = 0.0
        wx = wy = wz = 0.0
        if self._sampling_mode:
            # Note wx/wy are never assigned in this branch at all (stay 0.0
            # from above): pitch/yaw has no stick input routed to it here,
            # _level_hold in _publish is the only thing that ever touches
            # them while this mode is active.
            view_vy = -right_y * linear_speed      # inverted
            view_vz = right_x * linear_speed       # inverted
            wz = -left_x * angular_speed           # roll — pitch/yaw locked level
        elif self._probing_mode:
            # Right stick axes swapped vs sampling mode above (by request):
            # vertical stick drives up/down (view_vz, inverted — also by
            # request), horizontal stick drives left/right (view_vy) — same
            # wx/wy-untouched note as sampling mode applies here too.
            view_vy = -right_x * linear_speed
            view_vz = -right_y * linear_speed
            wz = -left_x * angular_speed           # roll — pitch/yaw locked level
        elif self._drill_mode:
            # Right stick axes swapped vs the naive mapping on purpose (by
            # request): vertical stick drives forward/back, horizontal
            # stick drives left/right — opposite of every other mode here.
            view_vx = -right_y * linear_speed
            view_vy = -right_x * linear_speed
            view_vz = left_y * linear_speed
        elif shift:
            view_vy = left_x * linear_speed
            wx = right_y * angular_speed          # pitch
            wz = -right_x * angular_speed         # roll
        else:
            view_vy = left_x * linear_speed
            view_vz = -right_y * linear_speed      # stick up = view +Z
            wy = -right_x * angular_speed         # yaw

        # Mount-frame translation is unused here — every gamepad axis is
        # view-relative or a rotation.
        self._controller.set_velocity(
            0.0, 0.0, 0.0, wx, wy, wz,
            view_vx=view_vx, view_vy=view_vy, view_vz=view_vz,
            hold_boost=boost,
        )

        cmd = (view_vx, view_vy, view_vz, wx, wy, wz)
        if cmd != self._prev_cmd and any(c != 0.0 for c in cmd):
            label = self._active_label(view_vx, view_vy, view_vz, wx, wy, wz, shift)
            print(f'{label} fwd={view_vx:.2f} left={view_vy:.2f} up={view_vz:.2f} '
                  f'wx={wx:.2f} wy={wy:.2f} wz={wz:.2f}')
        self._prev_cmd = cmd

    def _handle_panel_align(self):
        """Run panel alignment and hand control back to the operator either way.

        Mirrors KeyboardInputLoop._handle_panel_align — see its docstring
        for why Servo is restarted on failure too, unlike _handle_safe_pose,
        and for why this is guarded by a non-blocking lock.
        """
        if not self._panel_align_running.acquire(blocking=False):
            return
        try:
            print('Aligning to panel...')
            if self._controller.run_planned_activity(self._controller.align_to_panel, 'align_to_panel'):
                print('Panel align succeeded.')
            else:
                print('Panel align failed.')
            print('Resuming manual control...')
            self._controller.start_servo()
        finally:
            self._panel_align_running.release()

    def _handle_orient_gripper(self):
        """Rotate the gripper in place to face the remembered panel direction
        (mirrors KeyboardInputLoop._handle_orient_gripper)."""
        if not self._orient_gripper_running.acquire(blocking=False):
            return
        self._orient_gripper_active = True
        try:
            print('Orienting gripper toward panel...')
            if self._controller.run_planned_activity(
                    self._controller.orient_gripper_to_panel, 'orient_gripper_to_panel'):
                print('Gripper oriented.')
            else:
                print('Gripper orient failed.')
            print('Resuming manual control...')
            self._controller.start_servo()
        finally:
            self._orient_gripper_active = False
            self._orient_gripper_running.release()

    def _handle_safe_pose(self, target_mode=None):
        """Stop motion and move to the safe pose (mirrors KeyboardInputLoop's 'r').

        Args:
            target_mode: ``None`` for plain A (jaw/astrobio home) — also
                the only path that disarms jaw's ``_probing_mode`` if it was
                armed, since jaw has no other "return to normal" button.
                drill_sampling's own A/B/Y sequence (see ``_on_joy``):
                ``'sampling'`` (A, sampling_home), ``'sampling_to_drill'``
                (B, two-leg move ending at drill_home — see
                ``ServoController.move_sampling_to_drill``),
                ``'sampling_container'``/``'drill_container'`` (Y, routed
                by whichever mode is currently active — side detour, does
                NOT change ``_sampling_mode``/``_drill_mode`` below).
                ``'drill'`` is kept only for callers that want to jump
                straight to drill_home without the sampling_to_drill leg.
                ``'probing'`` is jaw's own B — jaw_probing_home + continuous
                point-down hold (see ``PROBING_POINT_AXIS``), the jaw
                equivalent of ``'sampling'``. ``'jaw_probing_container'`` is
                jaw's own Y — two-leg move via jaw_probing_home then (only
                on success) jaw_probing_container, see
                ``ServoController.move_jaw_probing_to_container`` — the jaw
                equivalent of ``'sampling_to_drill'``.
                Only decides which pose(s) to target and which mode to
                commit AFTER a successful move — see below.

        Guarded by a non-blocking lock so a second button press while a
        move is already in progress is ignored instead of racing a
        redundant safe-pose goal against the first. Also sets
        ``_safe_pose_active`` for the duration so ``_on_joy`` ignores
        stick/trigger input while it's set — otherwise a stick held
        during the move would resume Cartesian motion the instant
        ``start_servo()`` re-enables Servo, before the operator has a
        chance to let go.

        This is also the only place ``_teleop_locked`` is cleared (see
        its declaration in ``__init__``).

        Sampling/drill mode itself is committed (on both this loop and
        the controller) ONLY after ``home_ok`` — review-flagged: setting
        it immediately on button press let the software mode (and its
        axis mapping / _level_hold target) change even when the move was
        rejected/failed, leaving the arm physically in its old
        configuration while teleop already assumed the new one.
        """
        if not self._safe_pose_running.acquire(blocking=False):
            return
        self._safe_pose_active = True
        try:
            self._controller.stop()
            print('Moving to home...')
            if target_mode == 'sampling':
                action = functools.partial(
                    self._controller.move_to_safe_pose,
                    positions=self._controller.sampling_home_pose,
                    name=self._controller.sampling_home_pose_name,
                )
            elif target_mode == 'drill':
                action = functools.partial(
                    self._controller.move_to_safe_pose,
                    positions=self._controller.drill_home_pose,
                    name=self._controller.drill_home_pose_name,
                )
            elif target_mode == 'sampling_to_drill':
                action = self._controller.move_sampling_to_drill
            elif target_mode == 'sampling_container':
                action = functools.partial(
                    self._controller.move_to_safe_pose,
                    positions=self._controller.sampling_container_pose,
                    name=self._controller.sampling_container_pose_name,
                )
            elif target_mode == 'drill_container':
                action = functools.partial(
                    self._controller.move_to_safe_pose,
                    positions=self._controller.drill_container_pose,
                    name=self._controller.drill_container_pose_name,
                )
            elif target_mode == 'probing':
                action = functools.partial(
                    self._controller.move_to_safe_pose,
                    positions=self._controller.jaw_probing_home_pose,
                    name=self._controller.jaw_probing_home_pose_name,
                )
            elif target_mode == 'jaw_probing_container':
                action = self._controller.move_jaw_probing_to_container
            else:
                action = self._controller.move_to_safe_pose
            home_ok = self._controller.run_planned_activity(action, 'move_to_safe_pose')
            if home_ok:
                if target_mode == 'sampling':
                    self._sampling_mode = True
                    self._drill_mode = False
                    self._controller.set_sampling_mode(True)
                elif target_mode in ('drill', 'sampling_to_drill'):
                    self._drill_mode = True
                    self._sampling_mode = False
                    self._controller.set_drill_mode(True)
                elif target_mode in ('probing', 'jaw_probing_container'):
                    self._probing_mode = True
                    self._controller.set_probing_mode(True)
                elif target_mode is None:
                    # Plain A/home (jaw/astrobio) — jaw's only way back from
                    # probing mode, since jaw has no dedicated "exit" button.
                    self._probing_mode = False
                    self._controller.set_probing_mode(False)
                # sampling_container/drill_container: side detour, current
                # mode is left exactly as it was (see docstring above).
                print('Starting servo...')
                if self._controller.start_servo():
                    self._teleop_locked = False
                    self._controller.get_logger().info('Teleop enabled.')
                else:
                    self._controller.get_logger().warn(
                        'Servo failed to start — staying on trajectory controller.'
                    )
            else:
                print('Home move failed — Servo not started.')
        finally:
            self._safe_pose_active = False
            self._safe_pose_running.release()

    def _handle_level(self):
        """Reorient the tool straight down via a collision-checked plan
        (mirrors KeyboardInputLoop's 'f' — see level_tool's own docstring
        for why this needs to be its own move_group plan).

        ``_level_active`` mirrors ``_safe_pose_active``'s own role: held
        for the duration so ``_on_joy`` ignores stick input mid-move,
        same reasoning as that flag's own declaration.
        """
        if not self._level_running.acquire(blocking=False):
            return
        self._level_active = True
        try:
            self._controller.stop()
            print('Leveling tool...')
            if self._controller.run_planned_activity(self._controller.level_tool, 'level_tool'):
                print('Tool leveled.')
            else:
                print('Level move failed.')
            print('Resuming manual control...')
            self._controller.start_servo()
        finally:
            self._level_active = False
            self._level_running.release()

    def run(self):
        """Print the help banner and block until the exit button is pressed."""
        print(GAMEPAD_HELP)
        try:
            self._exit_event.wait()
        finally:
            print('\nExiting...')
            self._controller.stop()


def _run_teleop(controller: 'ServoController', input_loop) -> None:
    """Spin ``controller`` in a background thread and run ``input_loop`` until exit.

    Shared by ``main`` (keyboard) and ``main_gamepad`` (gamepad): both
    input loops expose the same ``run()`` contract (block until an exit
    condition, leave the controller stopped), so the ROS lifecycle
    around them doesn't need to be duplicated per input source.
    """
    spin_thread = threading.Thread(target=rclpy.spin, args=(controller,), daemon=True)
    spin_thread.start()

    try:
        input_loop.run()
    except KeyboardInterrupt:
        pass
    finally:
        controller.stop()
        if not controller.stop_servo():
            controller.get_logger().warn(
                'Could not confirm Servo stopped before shutdown — '
                'it may still be active.'
            )
        controller.destroy_node()
        rclpy.shutdown()


def main():
    """Entry point: initialize ROS2, run the keyboard input loop, and clean up.

    See ``_run_teleop`` for the shared spin/cleanup lifecycle.
    """
    rclpy.init()
    controller = ServoController()
    _run_teleop(controller, KeyboardInputLoop(controller))


def main_gamepad():
    """Entry point: initialize ROS2, run the gamepad input loop, and clean up.

    Requires a running ``joy`` publisher — see ``arm_teleop/launch/gamepad.launch.py``.
    See ``_run_teleop`` for the shared spin/cleanup lifecycle.
    """
    rclpy.init()
    controller = ServoController()
    _run_teleop(controller, GamepadInputLoop(controller))


if __name__ == '__main__':
    main()