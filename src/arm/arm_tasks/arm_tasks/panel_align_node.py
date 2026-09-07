"""MoveIt-planned, collision-checked alignment to a detected panel.

Separate node from ``keyboard_servo_node.py`` on purpose: MoveIt planning
is a new, heavier dependency this codebase hasn't used before, and
shouldn't be mixed into the input-handling module. Exposes one
``std_srvs/srv/Trigger`` service (``align``), the same idiom already used
for ``servo_node/start_servo``/``stop_servo``.

``align_to_panel()`` mirrors ``ServoController.move_to_safe_pose()``'s
contract: returns ``bool``, never raises, logs a distinguishing message
per failure mode, and never re-enables Servo itself — the caller (the
'p'-key/gamepad-button handler in ``keyboard_servo_node.py``) decides
what to do next.

The panel is a completely separate Gazebo model, not part of the arm's
own robot_description — nothing else in this repo inserts it (or
anything) into MoveIt's planning scene. Without step 5 below explicitly
adding a CollisionObject for it before planning, "collision-checked"
would silently mean "collision-checked against everything except the one
object we're deliberately moving toward."

Every blocking sub-call in align_to_panel() (stop_servo, apply_planning_
scene, /move_action, /execute_trajectory) waits on a threading.Event set
from an rclpy done-callback — the same pattern keyboard_servo_node.py
uses successfully, but THERE it always runs on a background thread
separate from the ROS spin thread. Here, align_to_panel() runs FROM
_on_align_request, itself a callback invoked BY the executor — with the
default SingleThreadedExecutor, that self-deadlocks: the executor can't
get back around to invoke the done-callback that would set the Event
while it's still inside the callback that's waiting on it. Fixed by
running everything on one ReentrantCallbackGroup under a
MultiThreadedExecutor (see main()), so the service callback and its
sub-calls' response callbacks can interleave on different threads.
"""

import math
import os
import random
import socket
import threading
import xml.etree.ElementTree as ET

import rclpy
import tf2_ros
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.msg import (
    BoundingVolume, CollisionObject, Constraints, JointConstraint, MoveItErrorCodes,
    PlanningOptions, PlanningScene, PlanningSceneWorld, PositionConstraint, PositionIKRequest,
)
from controller_manager_msgs.srv import SwitchController
from indomitus_interfaces.srv import AcquireArmMotionLock, ReleaseArmMotionLock
from moveit_msgs.srv import ApplyPlanningScene, GetPositionIK
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration as TfWaitDuration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, JointState
from shape_msgs.msg import SolidPrimitive
from std_srvs.srv import Trigger

from arm_teleop.arm_motion_lock import ArmMotionBusy, arm_motion_lock
from arm_tasks.camera_target_math import (
    StandoffResult, compose_transforms, compute_standoff_distance, compute_target_tip_pose,
)

GROUP_NAME = 'indomitus_arm'
TIP_LINK = 'arm_tcp_link'  # confirmed in arm_moveit_config/config/indomitus_arm.srdf
CAMERA_OPTICAL_FRAME = 'arm_camera_optical_frame'
CONTROLLER_NAME = 'indomitus_arm_controller'  # matches move_to_safe_pose()'s controller
# The streaming controller Servo actually claims the joints with while
# running — ExecuteTrajectory needs CONTROLLER_NAME (JTC) active instead,
# see _use_trajectory_controller().
FORWARD_CONTROLLER_NAME = 'indomitus_arm_forward_position_controller'
# SRDF group's base_link, and the frame every hand-verified reachability
# check this feature was built against actually used. Goal constraints and
# the panel CollisionObject used to be submitted with
# header.frame_id=CAMERA_OPTICAL_FRAME on the theory that MoveGroup
# resolves a robot-link frame_id against the live robot state — live
# testing showed the opposite: OMPL's goal-tree sampling failed 100% of
# the time (all threads, full allowed_planning_time) for a target
# independently confirmed reachable via a direct /compute_ik call seeded
# at the current joint state. Resolving to this fixed frame ourselves via
# one TF lookup (below), instead of relying on that resolution inside
# MoveGroup, matches the exact approach that direct check used
# successfully and removes the dependency on that assumption entirely.
PLANNING_FRAME = 'arm_mount_link'

DEFAULT_PANEL_POSE_TOPIC = '/panel_pose'
DEFAULT_CAMERA_INFO_TOPIC = '/camera/camera_info'
# Must exceed keyboard_servo_node.py's ACTIVITY_INDICATOR_PRE_DELAY_SEC
# (5s) with margin: the align request only reaches this node AFTER that
# wait, so the check here has to tolerate a detection drop spanning it.
DEFAULT_MAX_PANEL_POSE_AGE_SEC = 8.0
# lookup_transform() waits up to this long for TF to catch up to
# panel_pose's own timestamp — closes a small, normal race where the
# detection's stamp is a touch ahead of the latest buffered TF sample.
TF_LOOKUP_WAIT_SEC = 0.3

# Panel collision box (panel_description/urdf/panel_macro.xacro's
# panel_base_link collision geometry). Duplicated here — see
# panel_geometry.py's KNOWN_MARKER_LOCAL_POSITIONS for the same tradeoff.
PANEL_WIDTH = 0.33
PANEL_DEPTH = 0.072
PANEL_HEIGHT = 0.45
# panel_base_link's own <collision><origin> in panel_macro.xacro — the
# fused panel_pose we receive is panel_base_link's own origin (bottom
# edge, per how panel_geometry.py's KNOWN_MARKER_LOCAL_POSITIONS were
# derived), NOT the collision box's center, so this local offset has to
# be applied before inserting the CollisionObject. Missing this made the
# box straddle the wrong volume (half sunk below the real panel, and not
# reaching its actual top) — confirmed live: OMPL's goal-tree sampling
# failed 100% of the time until this was added, because target poses
# computed relative to the (correctly-positioned) marker detections kept
# landing inside this mispositioned collision volume.
PANEL_COLLISION_LOCAL_OFFSET = ((0.0, 0.013, 0.225), (0.0, 0.0, 0.0, 1.0))
# The point we actually aim the camera at: the middle of the panel's own
# FRONT FACE, in panel_base_link's local frame — not panel_base_link's
# origin itself (near the bottom edge — see PANEL_COLLISION_LOCAL_OFFSET
# above) and not the collision box's volumetric center (that one's Y
# accounts for the solid mesh's depth, not the face plane). x=0 (panel is
# symmetric left/right: top_left x=-0.135, top_right x=+0.135 in
# panel_geometry.py), z=0.225 (half of PANEL_HEIGHT), y=-0.015 (front
# face plane, per panel_macro.xacro's own comment on panel.stl's
# geometry). Always the same fixed point relative to the panel, however
# it was detected — this is what makes align_to_panel()'s target
# deterministic/repeatable rather than depending on exactly which
# markers happened to be visible. z here is the default only — see the
# 'panel_center_z_offset' parameter below, which lets this be raised
# toward a specific feature (e.g. a row of switches) instead of the
# panel's exact vertical center.
DEFAULT_PANEL_CENTER_Z_OFFSET = 0.225  # PANEL_HEIGHT / 2 -- vertical center

# Panel-task-specific joint bounds. arm_forearm_wrist_1_joint matches
# arm_macro.xacro's own real-hardware limit (+-200deg, not +-90 — a
# strict +-pi/2 here made every standoff/z_offset combination
# unreachable, confirmed live via an exhaustive sweep); the other 4
# joints are further restricted than the real +-200deg hardware limit,
# same rationale as arm_shoulder_forearm_joint's own +-pi/2. Used for
# both the post-plan margin check (step 7) and as OMPL path constraints
# (see PANEL_JOINT_LIMIT_PATH_CONSTRAINTS below).
JOINT_LIMITS = {
    'arm_mount_base_joint': (-math.pi, math.pi),
    'arm_base_shoulder_joint': (-math.pi, math.pi),
    'arm_shoulder_forearm_joint': (-math.pi / 2, math.pi / 2),
    'arm_forearm_wrist_1_joint': (-20 * math.pi / 18, 20 * math.pi / 18),
    'arm_wrist_1_wrist_2_joint': (-math.pi, math.pi),
    'arm_wrist_2_end_effector_joint': (-math.pi, math.pi),
}


def _joint_limit_margin(name: str, position: float) -> float:
    """Fraction of name's own JOINT_LIMITS span that position clears from
    the nearer bound — shared by _compute_ik_within_panel_limits (accept)
    and _check_joint_margins (verify), so both apply the exact same
    margin fraction instead of two independently-tuned thresholds.
    """
    lower, upper = JOINT_LIMITS[name]
    return min(position - lower, upper - position) / (upper - lower)


def _joint_space_distance(a: dict, b: dict) -> float:
    """Euclidean distance between two {joint_name: position} dicts, over
    JOINT_LIMITS' own joints only — used to pick the IK solution nearest
    the arm's current state out of several valid candidates.
    """
    return math.sqrt(sum((a[name] - b[name]) ** 2 for name in JOINT_LIMITS))

# Actually keeps OMPL's sampled path inside JOINT_LIMITS, unlike the
# post-plan-only margin check below — the URDF itself allows +-2pi (real
# hardware needs that), so without this OMPL freely wanders the full
# range and _check_joint_margins() only finds out afterward, wasting a
# full plan + the 5s ERC activity delay on a plan it then has to reject.
PANEL_JOINT_LIMIT_PATH_CONSTRAINTS = Constraints(joint_constraints=[
    JointConstraint(
        joint_name=name, position=(lo + hi) / 2,
        tolerance_above=(hi - lo) / 2, tolerance_below=(hi - lo) / 2, weight=1.0)
    for name, (lo, hi) in JOINT_LIMITS.items()
])



class PanelAlignNode(Node):
    def __init__(self):
        super().__init__('panel_align_node')

        self.declare_parameter('panel_pose_topic', DEFAULT_PANEL_POSE_TOPIC)
        self.declare_parameter('camera_info_topic', DEFAULT_CAMERA_INFO_TOPIC)
        self.declare_parameter('max_panel_pose_age_sec', DEFAULT_MAX_PANEL_POSE_AGE_SEC)
        # 1.15 fit the panel in frame with barely any margin; 1.4 fit it
        # comfortably but pushed arm_mount_base_joint to only 3.4% clear of
        # its limit (need 8%, see _check_joint_margins) — confirmed live at
        # the current panel placement (arm_gazebo.launch.py's PANEL_X/Y/Z/YAW).
        # 1.3 is the compromise: more framing margin than 1.15, still clear
        # of the joint-limit rejection 1.4 hit.
        self.declare_parameter('standoff_margin_multiplier', 1.3)
        self.declare_parameter('standoff_min_floor', 0.15)
        self.declare_parameter('standoff_max_reach', 0.75)
        # True by default on explicit request: the FOV-fit standoff
        # (compute_standoff_distance) exists to keep the WHOLE panel
        # framed in the camera's view from the final resting position —
        # not needed once alignment itself is done, and real testing
        # confirmed the FOV-fit distance held the arm too far out (also
        # contributing to the joint-limit-margin rejections seen live).
        # 0.25m confirmed by hand as a good close distance for this panel/
        # camera/gripper. Set to false to go back to FOV-fit framing.
        self.declare_parameter('use_fixed_test_standoff', True)
        self.declare_parameter('fixed_test_standoff', 0.25)
        # Raises/lowers the aim point along the panel's own vertical axis,
        # relative to panel_base_link's origin (see
        # DEFAULT_PANEL_CENTER_Z_OFFSET's comment) — e.g. to aim at a row
        # of switches instead of the panel's exact geometric center. 0.35
        # confirmed by hand against the real panel's switch row (default
        # 0.225 is the exact geometric center).
        self.declare_parameter('panel_center_z_offset', 0.35)
        self.declare_parameter('joint_limit_margin_fraction', 0.08)
        self.declare_parameter('num_planning_attempts', 10)
        self.declare_parameter('allowed_planning_time', 5.0)
        self.declare_parameter('max_velocity_scaling_factor', 0.3)
        self.declare_parameter('max_acceleration_scaling_factor', 0.3)
        # OMPL's own goal-region sampler is unreliable once path_constraints
        # (JOINT_LIMITS) are combined with a Cartesian pose goal — confirmed
        # live: a target well within every joint's limit, comfortably
        # solvable by a single /compute_ik call, still made the OMPL
        # planner report "Insufficient states in sampleable goal region"
        # every time. Solving IK ourselves first (this many tries, KDL's
        # own random-restart doing the real work) and then planning a
        # JOINT-space goal to that solution sidesteps the OMPL weakness
        # entirely — same approach _align_to_remembered_locked already
        # uses successfully for replay.
        self.declare_parameter('live_align_ik_attempts', 8)
        # 'm' must not swing the arm through a different elbow/wrist
        # branch to reach an orientation-only target — review-flagged:
        # picking the IK solution nearest the current joint state only
        # bounds the ENDPOINT, not the path OMPL takes to get there. This
        # is an actual planning-time constraint (a PositionConstraint
        # path_constraint, not just a goal) that keeps TIP_LINK within
        # this radius of its CURRENT position for every point along the
        # whole trajectory, so "reorient in place" is enforced by the
        # planner itself. Generous enough for the small excursion joint
        # interpolation between two nearby joint-space solutions needs,
        # tight enough to rule out routing through a distant branch.
        self.declare_parameter('orient_gripper_path_position_radius', 0.05)
        # This 60s, summed with every other wait_for_service/server +
        # done.wait() in align_to_panel()'s sequence, must stay under
        # keyboard_servo_node.py's DEFAULT_PANEL_ALIGN_TIMEOUT — see its
        # comment for the full worst-case budget. A parameter (not a
        # literal) so tests can shrink it instead of actually waiting 60s
        # to exercise the cancel-on-timeout path.
        self.declare_parameter('execution_timeout_sec', 60.0)
        # Covers this node's own worst case (allowed_planning_time +
        # execution_timeout_sec + service-call overhead) with margin — the
        # cross-host arm_motion_lock lease, not this node's own timeouts.
        self.declare_parameter('arm_motion_lock_lease_sec', 90.0)

        self._panel_pose_topic = self.get_parameter('panel_pose_topic').value
        self._camera_info_topic = self.get_parameter('camera_info_topic').value

        self._latest_panel_pose: PoseStamped | None = None
        self._latest_camera_info: CameraInfo | None = None
        # Used to seed /compute_ik's first attempt and to pick the
        # closest-to-current solution among several valid ones (review-
        # flagged: retries without this can't guarantee exploring
        # different IK branches, and orient_gripper needs the NEAREST
        # branch specifically to actually stay "in place").
        self._latest_joint_positions: dict[str, float] = {}
        self._camera_to_tip = None  # cached at startup, see _try_cache_camera_to_tip
        self._last_status_message = ''
        # Session-scoped only (reset by restarting this node). None until
        # the first live CV+MoveIt align succeeds; every align call after
        # that replays this exact joint vector via a fresh MoveIt plan
        # (_align_to_remembered_locked) instead of re-running live CV
        # detection — see _align_to_panel_locked. {joint_name: position}.
        self._remembered_target_joints: dict | None = None
        # The panel pose (in PLANNING_FRAME) used to build the collision
        # box when _remembered_target_joints was learned — replay's
        # fallback collision reference when the panel isn't currently in
        # view (see _align_to_remembered_locked).
        self._remembered_panel_collision_pose: PoseStamped | None = None
        # TCP orientation (xyzw, PLANNING_FRAME) from the same align that
        # set _remembered_target_joints — see orient_gripper_to_remembered().
        self._remembered_target_orientation: tuple | None = None
        # Guards align_to_panel() against running twice at once — the
        # 'align' service is registered on a ReentrantCallbackGroup (see
        # module docstring) so its own sub-calls can interleave, but that
        # also means a second concurrent /panel_align/align request would
        # otherwise run this whole sequence in parallel with the first,
        # against the same physical arm.
        self._align_running = threading.Lock()
        self._orient_gripper_running = threading.Lock()

        # See module docstring: everything here shares one reentrant group
        # so the align service callback and its own sub-calls' response
        # callbacks can interleave (requires MultiThreadedExecutor, see
        # main() — a plain rclpy.spin() would deadlock).
        cb_group = ReentrantCallbackGroup()

        self.create_subscription(
            PoseStamped, self._panel_pose_topic, self._on_panel_pose, 10,
            callback_group=cb_group)
        self.create_subscription(
            CameraInfo, self._camera_info_topic, self._on_camera_info, 10,
            callback_group=cb_group)
        self.create_subscription(
            JointState, 'joint_states', self._on_joint_state, 10,
            callback_group=cb_group)

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
        self._tf_retry_timer = self.create_timer(
            1.0, self._try_cache_camera_to_tip, callback_group=cb_group)

        self._move_action_client = ActionClient(
            self, MoveGroup, 'move_action', callback_group=cb_group)
        self._execute_client = ActionClient(
            self, ExecuteTrajectory, 'execute_trajectory', callback_group=cb_group)
        self._apply_scene_client = self.create_client(
            ApplyPlanningScene, 'apply_planning_scene', callback_group=cb_group)
        self._compute_ik_client = self.create_client(
            GetPositionIK, 'compute_ik', callback_group=cb_group)
        self._stop_servo_client = self.create_client(
            Trigger, 'servo_node/stop_servo', callback_group=cb_group)
        self._switch_controller_client = self.create_client(
            SwitchController, 'controller_manager/switch_controller', callback_group=cb_group)
        self._acquire_lock_client = self.create_client(
            AcquireArmMotionLock, 'arm_motion_lock/acquire', callback_group=cb_group)
        self._release_lock_client = self.create_client(
            ReleaseArmMotionLock, 'arm_motion_lock/release', callback_group=cb_group)
        # Identifies THIS process to arm_motion_lock_server — hostname
        # covers the actual cross-host case (GS vs Jetson), pid separates
        # two runs on the same host (e.g. sim + a stray leftover process).
        self._motion_lock_holder_id = f'{socket.gethostname()}/panel_align_node/{os.getpid()}'

        self.create_service(
            Trigger, 'panel_align/align', self._on_align_request, callback_group=cb_group)
        self.create_service(
            Trigger, 'panel_align/orient_gripper', self._on_orient_gripper_request,
            callback_group=cb_group)

        self.get_logger().info('panel_align_node ready — waiting for camera<->tip TF...')

    # ---- subscriptions -------------------------------------------------

    def _on_panel_pose(self, msg: PoseStamped) -> None:
        self._latest_panel_pose = msg

    def _on_camera_info(self, msg: CameraInfo) -> None:
        self._latest_camera_info = msg

    def _on_joint_state(self, msg: JointState) -> None:
        for name, position in zip(msg.name, msg.position):
            if name in JOINT_LIMITS:
                self._latest_joint_positions[name] = position

    def _try_cache_camera_to_tip(self) -> None:
        if self._camera_to_tip is not None:
            self._tf_retry_timer.cancel()
            return
        try:
            tf = self._tf_buffer.lookup_transform(
                CAMERA_OPTICAL_FRAME, TIP_LINK, rclpy.time.Time())
        except tf2_ros.TransformException as exc:
            self.get_logger().debug(f'camera->tip TF not yet available: {exc}')
            return
        t = tf.transform.translation
        r = tf.transform.rotation
        self._camera_to_tip = ((t.x, t.y, t.z), (r.x, r.y, r.z, r.w))
        self._tf_retry_timer.cancel()
        self.get_logger().info(
            f'Cached static {CAMERA_OPTICAL_FRAME} -> {TIP_LINK} transform.'
        )

    # ---- service entry point -------------------------------------------

    def _on_align_request(self, request, response):
        response.success = self.align_to_panel()
        response.message = self._last_status_message
        return response

    def _on_orient_gripper_request(self, request, response):
        response.success = self.orient_gripper_to_remembered()
        response.message = self._last_status_message
        return response

    # ---- the actual sequence --------------------------------------------

    def align_to_panel(self) -> bool:
        """Run the align sequence, refusing to overlap a concurrent call.

        See _align_running's declaration in __init__ for why that guard
        exists — it only protects against a second concurrent request to
        THIS service, within this process. Also takes arm_motion_lock()
        (see that module) around the whole sequence, so a live align
        here can't race a home move or remembered-position replay
        submitted from the separate keyboard_servo_node process onto the
        same indomitus_arm_controller action server. A rejected
        concurrent call still gets a clean Trigger failure response
        instead of interleaving with the in-flight one.
        """
        if not self._align_running.acquire(blocking=False):
            self.get_logger().warn('Align already in progress — ignoring concurrent request.')
            return self._fail('Another panel align is already in progress.')
        try:
            try:
                with arm_motion_lock(
                        self._acquire_lock_client, self._release_lock_client,
                        self._motion_lock_holder_id,
                        self.get_parameter('arm_motion_lock_lease_sec').value):
                    return self._align_to_panel_locked()
            except ArmMotionBusy as exc:
                return self._fail(f'{exc} — aborting panel align.')
        finally:
            self._align_running.release()

    def _align_to_panel_locked(self) -> bool:
        """Replay the remembered target if one exists, else live-align.

        See _remembered_target_joints's declaration in __init__: once
        set (by a successful _align_live_locked() call), every
        subsequent align always replays it — deterministic final pose,
        matching this feature's original design intent — but through a
        fresh, collision-checked MoveIt plan every time (see
        _align_to_remembered_locked), not a raw point-to-point move.
        """
        if self._remembered_target_joints is not None:
            return self._align_to_remembered_locked()
        return self._align_live_locked()

    def orient_gripper_to_remembered(self) -> bool:
        """Rotate the gripper to the remembered panel-facing orientation
        in place — same position, new orientation. Locking mirrors
        align_to_panel().
        """
        if not self._orient_gripper_running.acquire(blocking=False):
            self.get_logger().warn('Gripper orient already in progress — ignoring concurrent request.')
            return self._fail('Another gripper orient is already in progress.')
        try:
            try:
                with arm_motion_lock(
                        self._acquire_lock_client, self._release_lock_client,
                        self._motion_lock_holder_id,
                        self.get_parameter('arm_motion_lock_lease_sec').value):
                    return self._orient_gripper_to_remembered_locked()
            except ArmMotionBusy as exc:
                return self._fail(f'{exc} — aborting gripper orient.')
        finally:
            self._orient_gripper_running.release()

    def _orient_gripper_to_remembered_locked(self) -> bool:
        """Rotate TIP_LINK to the remembered panel-facing orientation
        while keeping its CURRENT position — a MoveGroup pose goal like
        align_to_panel(), just built from where the tip already is
        instead of a freshly computed panel-relative target. Any joint
        may move to reach it.
        """
        self._last_status_message = ''

        if self._remembered_target_orientation is None:
            return self._fail(
                'No remembered panel orientation yet — align to the panel (p) at least once first.'
            )

        try:
            tf_tip = self._tf_buffer.lookup_transform(
                PLANNING_FRAME, TIP_LINK, rclpy.time.Time(),
                timeout=TfWaitDuration(seconds=TF_LOOKUP_WAIT_SEC))
        except tf2_ros.TransformException as exc:
            return self._fail(f'Could not resolve current {TIP_LINK} pose: {exc}')

        target_pose_msg = PoseStamped()
        target_pose_msg.header.frame_id = PLANNING_FRAME
        target_pose_msg.pose.position.x = tf_tip.transform.translation.x
        target_pose_msg.pose.position.y = tf_tip.transform.translation.y
        target_pose_msg.pose.position.z = tf_tip.transform.translation.z
        (target_pose_msg.pose.orientation.x, target_pose_msg.pose.orientation.y,
         target_pose_msg.pose.orientation.z, target_pose_msg.pose.orientation.w
         ) = self._remembered_target_orientation

        if not self._call_stop_servo():
            return self._fail('Could not confirm Servo stopped — aborting gripper orient.')

        if not self._use_trajectory_controller():
            return self._fail('Could not activate the trajectory controller — aborting gripper orient.')

        # Same collision-box source preference as _align_to_remembered_locked:
        # live detection if available, else the cached pose from when this
        # orientation was learned. Without this, orient_gripper planned
        # against an empty scene — a wrist rotation this close to the panel
        # needs the same protection align already gets.
        collision_pose = self._current_live_panel_collision_pose()
        if collision_pose is None:
            collision_pose = self._remembered_panel_collision_pose
        if collision_pose is not None and not self._apply_panel_collision_object(collision_pose):
            return self._fail('Could not insert panel CollisionObject into the planning scene.')

        # IK-then-joint-goal, not a Cartesian pose goal — same reason as
        # _align_live_locked (see live_align_ik_attempts' comment). IK's
        # own closest-to-current preference picks a nearby branch, but
        # only bounds the ENDPOINT — the path_constraint below is what
        # actually keeps the WHOLE motion near the current position (see
        # orient_gripper_path_position_radius's own comment).
        ik_solution = self._compute_ik_within_panel_limits(target_pose_msg)
        if ik_solution is None:
            return self._fail(
                'No IK solution within the panel task\'s own joint limits for this '
                'orientation — aborting gripper orient.'
            )
        goal_constraints = self._build_joint_goal_constraints(ik_solution)
        stay_near_current_position = PositionConstraint()
        stay_near_current_position.header = target_pose_msg.header
        stay_near_current_position.link_name = TIP_LINK
        stay_near_current_position.constraint_region = BoundingVolume(
            primitives=[SolidPrimitive(
                type=SolidPrimitive.SPHERE,
                dimensions=[self.get_parameter('orient_gripper_path_position_radius').value])],
            primitive_poses=[target_pose_msg.pose],
        )
        stay_near_current_position.weight = 1.0
        plan_result = self._request_plan(
            goal_constraints, extra_position_constraints=[stay_near_current_position])
        if plan_result is None:
            return self._fail('MoveGroup planning request timed out or was rejected.')
        if plan_result.error_code.val != MoveItErrorCodes.SUCCESS:
            return self._fail(
                f'Planning gripper orientation failed (MoveItErrorCodes.val={plan_result.error_code.val}).'
            )
        planned_trajectory = plan_result.planned_trajectory

        margin_ok, margin_reason = self._check_joint_margins(planned_trajectory)
        if not margin_ok:
            return self._fail(f'Planned pose too close to a joint limit: {margin_reason}')

        exec_ok, exec_reason = self._execute(planned_trajectory)
        if not exec_ok:
            return self._fail(f'Trajectory execution failed: {exec_reason}')

        self._last_status_message = 'Gripper oriented toward remembered panel position.'
        self.get_logger().info(self._last_status_message)
        return True

    def _align_to_remembered_locked(self) -> bool:
        self._last_status_message = ''

        if not self._call_stop_servo():
            return self._fail('Could not confirm Servo stopped — aborting panel align.')

        if not self._use_trajectory_controller():
            return self._fail(
                'Could not activate the trajectory controller — aborting panel align.'
            )

        # Prefer a CURRENT panel detection for the collision box if one's
        # available (more accurate — the panel may have been nudged, or
        # this may be a portable test rig), else fall back to the pose
        # cached when this target was first learned. Either way, collision-
        # check against SOMETHING rather than skipping the panel entirely —
        # this is the actual fix for the replay-bypasses-collision-
        # checking issue this method exists to close.
        collision_pose = self._current_live_panel_collision_pose()
        if collision_pose is None:
            collision_pose = self._remembered_panel_collision_pose
        if collision_pose is not None and not self._apply_panel_collision_object(collision_pose):
            return self._fail('Could not insert panel CollisionObject into the planning scene.')

        goal_constraints = self._build_joint_goal_constraints(self._remembered_target_joints)
        plan_result = self._request_plan(goal_constraints)
        if plan_result is None:
            return self._fail('MoveGroup planning request timed out or was rejected.')
        if plan_result.error_code.val != MoveItErrorCodes.SUCCESS:
            return self._fail(
                f'Planning to remembered position failed (MoveItErrorCodes.val={plan_result.error_code.val}).'
            )

        margin_ok, margin_reason = self._check_joint_margins(plan_result.planned_trajectory)
        if not margin_ok:
            return self._fail(f'Planned pose too close to a joint limit: {margin_reason}')

        exec_ok, exec_reason = self._execute(plan_result.planned_trajectory)
        if not exec_ok:
            return self._fail(f'Trajectory execution failed: {exec_reason}')

        self._last_status_message = 'Aligned to remembered panel position.'
        self.get_logger().info(self._last_status_message)
        return True

    def _current_live_panel_collision_pose(self) -> PoseStamped | None:
        """Same panel_pose -> PLANNING_FRAME resolution _align_live_locked
        does, for the collision box only — returns None on ANY failure
        (stale/missing pose, TF lookup failure) rather than raising, since
        callers treat this as a nice-to-have over the cached fallback.
        """
        panel_pose = self._latest_panel_pose
        if panel_pose is None:
            return None
        age = (self.get_clock().now() - rclpy.time.Time.from_msg(panel_pose.header.stamp)).nanoseconds / 1e9
        if age > self.get_parameter('max_panel_pose_age_sec').value:
            return None
        try:
            tf_mount_camera = self._tf_buffer.lookup_transform(
                PLANNING_FRAME, panel_pose.header.frame_id,
                rclpy.time.Time.from_msg(panel_pose.header.stamp),
                timeout=TfWaitDuration(seconds=TF_LOOKUP_WAIT_SEC))
        except tf2_ros.TransformException:
            return None
        t = tf_mount_camera.transform.translation
        r = tf_mount_camera.transform.rotation
        p = panel_pose.pose.position
        o = panel_pose.pose.orientation
        panel_pose_in_mount = compose_transforms(
            ((t.x, t.y, t.z), (r.x, r.y, r.z, r.w)), ((p.x, p.y, p.z), (o.x, o.y, o.z, o.w)))
        msg = PoseStamped()
        msg.header.frame_id = PLANNING_FRAME
        msg.header.stamp = panel_pose.header.stamp
        (msg.pose.position.x, msg.pose.position.y, msg.pose.position.z) = panel_pose_in_mount[0]
        (msg.pose.orientation.x, msg.pose.orientation.y,
         msg.pose.orientation.z, msg.pose.orientation.w) = panel_pose_in_mount[1]
        return msg

    def _align_live_locked(self) -> bool:
        self._last_status_message = ''

        if not self._call_stop_servo():
            return self._fail('Could not confirm Servo stopped — aborting panel align.')

        if not self._use_trajectory_controller():
            return self._fail(
                'Could not activate the trajectory controller — aborting panel align.'
            )

        panel_pose = self._latest_panel_pose
        if panel_pose is None:
            return self._fail('No panel pose has been received yet.')
        age = (self.get_clock().now() - rclpy.time.Time.from_msg(panel_pose.header.stamp)).nanoseconds / 1e9
        max_age = self.get_parameter('max_panel_pose_age_sec').value
        if age > max_age:
            return self._fail(f'Panel pose is stale ({age:.2f}s old, max {max_age:.2f}s) — panel out of view?')

        if self._camera_to_tip is None:
            return self._fail(
                f'{CAMERA_OPTICAL_FRAME} -> {TIP_LINK} TF unavailable — '
                'startup TF lookup never succeeded, refusing to guess. Restart the node.'
            )

        info = self._latest_camera_info
        if info is None or info.k[0] == 0.0 or info.k[4] == 0.0:
            return self._fail('No valid CameraInfo received yet (fx/fy unknown).')

        if self.get_parameter('use_fixed_test_standoff').value:
            standoff = StandoffResult(
                distance=self.get_parameter('fixed_test_standoff').value,
                within_bounds=True, reason='')
        else:
            standoff = compute_standoff_distance(
                fx=info.k[0], fy=info.k[4], image_width=info.width, image_height=info.height,
                panel_width=PANEL_WIDTH, panel_height=PANEL_HEIGHT,
                margin_multiplier=self.get_parameter('standoff_margin_multiplier').value,
                min_floor=self.get_parameter('standoff_min_floor').value,
                max_reach=self.get_parameter('standoff_max_reach').value,
            )
        if not standoff.within_bounds:
            return self._fail(f'Standoff distance out of range: {standoff.reason}')

        try:
            tf_mount_camera = self._tf_buffer.lookup_transform(
                PLANNING_FRAME, panel_pose.header.frame_id,
                rclpy.time.Time.from_msg(panel_pose.header.stamp),
                timeout=TfWaitDuration(seconds=TF_LOOKUP_WAIT_SEC))
        except tf2_ros.TransformException as exc:
            return self._fail(
                f'Could not resolve {panel_pose.header.frame_id} -> {PLANNING_FRAME} TF: {exc}')
        t = tf_mount_camera.transform.translation
        r = tf_mount_camera.transform.rotation
        mount_to_camera = ((t.x, t.y, t.z), (r.x, r.y, r.z, r.w))

        p = panel_pose.pose.position
        o = panel_pose.pose.orientation
        panel_pose_in_mount = compose_transforms(
            mount_to_camera, ((p.x, p.y, p.z), (o.x, o.y, o.z, o.w)))
        panel_pose_mount_msg = PoseStamped()
        panel_pose_mount_msg.header.frame_id = PLANNING_FRAME
        panel_pose_mount_msg.header.stamp = panel_pose.header.stamp
        (panel_pose_mount_msg.pose.position.x, panel_pose_mount_msg.pose.position.y,
         panel_pose_mount_msg.pose.position.z) = panel_pose_in_mount[0]
        (panel_pose_mount_msg.pose.orientation.x, panel_pose_mount_msg.pose.orientation.y,
         panel_pose_mount_msg.pose.orientation.z, panel_pose_mount_msg.pose.orientation.w
         ) = panel_pose_in_mount[1]

        # Aim at a fixed point on the panel's own front face (see
        # DEFAULT_PANEL_CENTER_Z_OFFSET), not panel_base_link's origin
        # directly — this is what makes the target always the same point
        # on the panel regardless of exactly how the pose was detected.
        panel_center_local_offset = (
            (0.0, -0.015, self.get_parameter('panel_center_z_offset').value),
            (0.0, 0.0, 0.0, 1.0),
        )
        panel_center_in_mount = compose_transforms(
            panel_pose_in_mount, panel_center_local_offset)
        target_position, target_orientation = compute_target_tip_pose(
            panel_pose_in_camera=panel_center_in_mount,
            camera_to_tip=self._camera_to_tip,
            standoff=standoff.distance,
        )
        target_pose_msg = PoseStamped()
        target_pose_msg.header.frame_id = PLANNING_FRAME
        (target_pose_msg.pose.position.x, target_pose_msg.pose.position.y,
         target_pose_msg.pose.position.z) = target_position
        (target_pose_msg.pose.orientation.x, target_pose_msg.pose.orientation.y,
         target_pose_msg.pose.orientation.z, target_pose_msg.pose.orientation.w) = target_orientation

        if not self._apply_panel_collision_object(panel_pose_mount_msg):
            return self._fail('Could not insert panel CollisionObject into the planning scene.')

        # IK-then-joint-goal, not a Cartesian pose goal — see
        # live_align_ik_attempts' own comment for why.
        ik_solution = self._compute_ik_within_panel_limits(target_pose_msg)
        if ik_solution is None:
            return self._fail(
                'No IK solution within the panel task\'s own joint limits for this target '
                '— out of reach at the current standoff/aim height.'
            )
        goal_constraints = self._build_joint_goal_constraints(ik_solution)
        plan_result = self._request_plan(goal_constraints)
        if plan_result is None:
            return self._fail('MoveGroup planning request timed out or was rejected.')
        if plan_result.error_code.val != MoveItErrorCodes.SUCCESS:
            return self._fail(f'Planning failed (MoveItErrorCodes.val={plan_result.error_code.val}).')
        planned_trajectory = plan_result.planned_trajectory

        margin_ok, margin_reason = self._check_joint_margins(planned_trajectory)
        if not margin_ok:
            return self._fail(f'Planned pose too close to a joint limit: {margin_reason}')

        exec_ok, exec_reason = self._execute(planned_trajectory)
        if not exec_ok:
            return self._fail(f'Trajectory execution failed: {exec_reason}')

        # Remember the exact joint vector just reached — every later
        # align (this session) replays THIS, via a fresh collision-
        # checked plan each time (_align_to_remembered_locked), instead
        # of re-planning a Cartesian goal (which could land on a
        # different valid joint solution each call). Cache the collision
        # pose alongside it as a fallback for when the panel isn't
        # currently in view at replay time.
        traj = planned_trajectory.joint_trajectory
        self._remembered_target_joints = dict(zip(traj.joint_names, traj.points[-1].positions))
        self._remembered_panel_collision_pose = panel_pose_mount_msg
        self._remembered_target_orientation = target_orientation

        self._last_status_message = f'Aligned to panel (standoff={standoff.distance:.3f}m).'
        self.get_logger().info(self._last_status_message)
        return True

    def _fail(self, message: str) -> bool:
        self._last_status_message = message
        self.get_logger().error(message)
        return False

    # ---- steps -----------------------------------------------------------

    def _call_stop_servo(self) -> bool:
        if not self._stop_servo_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn('servo_node/stop_servo not available')
            return False
        done = threading.Event()
        result = {}

        def _cb(fut):
            result['r'] = fut.result()
            done.set()

        self._stop_servo_client.call_async(Trigger.Request()).add_done_callback(_cb)
        if not done.wait(timeout=3.0):
            return False
        return bool(result.get('r') and result['r'].success)

    def _use_trajectory_controller(self) -> bool:
        """Switch ros2_control from the streaming controller to the JTC.

        _call_stop_servo() only stops moveit_servo's own internal loop —
        it does not touch which ros2_control controller currently owns
        the joints. While Servo is running, FORWARD_CONTROLLER_NAME is
        the active one and CONTROLLER_NAME (the JTC that
        /execute_trajectory actually sends goals to, via
        moveit_simple_controller_manager) is inactive; confirmed live,
        skipping this step made every align attempt plan successfully and
        then abort on execution (GoalStatus.STATUS_ABORTED). BEST_EFFORT
        strictness (not STRICT) because a repeat align call after a
        previous one already left the JTC active would otherwise error on
        a no-op request.
        """
        if not self._switch_controller_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn('controller_manager/switch_controller not available')
            return False
        req = SwitchController.Request()
        req.activate_controllers = [CONTROLLER_NAME]
        req.deactivate_controllers = [FORWARD_CONTROLLER_NAME]
        req.strictness = SwitchController.Request.BEST_EFFORT
        req.activate_asap = True
        done = threading.Event()
        result = {}

        def _cb(fut):
            result['r'] = fut.result()
            done.set()

        self._switch_controller_client.call_async(req).add_done_callback(_cb)
        if not done.wait(timeout=5.0):
            return False
        return bool(result.get('r') and result['r'].ok)

    def _compute_ik_within_panel_limits(self, target_pose: PoseStamped) -> dict | None:
        """Call /compute_ik repeatedly, collect every solution that clears
        JOINT_LIMITS by joint_limit_margin_fraction (the same margin
        _check_joint_margins verifies after planning — accepting one that
        only just clears the hard limit would still get rejected there),
        and return whichever is closest (joint-space) to the arm's
        current state — or the first valid one if the current state
        isn't known yet (no /joint_states received).

        Each attempt seeds the search explicitly rather than relying on
        undocumented internal KDL randomization to diversify branches
        (review-flagged): the first attempt seeds from the CURRENT joint
        state (the branch orient_gripper actually wants), every retry
        seeds from a fresh random point inside JOINT_LIMITS so repeat
        calls genuinely explore different branches instead of KDL
        possibly reconverging on the same rejected one.

        Rejects incomplete, mismatched, or non-finite solutions outright
        — a solution missing an expected joint (or empty) must never
        pass just because Python's ``all([])`` is vacuously True.
        """
        if not self._compute_ik_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().error('/compute_ik service not available')
            return None

        margin_fraction = self.get_parameter('joint_limit_margin_fraction').value
        attempts = self.get_parameter('live_align_ik_attempts').value
        if attempts <= 0:
            self.get_logger().error(
                f'live_align_ik_attempts={attempts} — refusing to run a non-positive number of IK attempts.'
            )
            return None

        current = dict(self._latest_joint_positions)
        have_current = len(current) == len(JOINT_LIMITS)
        valid_solutions = []

        for attempt in range(attempts):
            req = GetPositionIK.Request()
            req.ik_request = PositionIKRequest(
                group_name=GROUP_NAME, pose_stamped=target_pose, avoid_collisions=True)
            req.ik_request.timeout.sec = 0
            req.ik_request.timeout.nanosec = 200_000_000
            seed = current if (attempt == 0 and have_current) else {
                name: random.uniform(lo, hi) for name, (lo, hi) in JOINT_LIMITS.items()
            }
            req.ik_request.robot_state.joint_state.name = list(seed.keys())
            req.ik_request.robot_state.joint_state.position = list(seed.values())

            done = threading.Event()
            result = {}

            def _cb(fut, result=result, done=done):
                # Isolated per attempt via default-arg capture — a late
                # callback from a timed-out earlier attempt must write
                # into ITS OWN result/done, never a newer attempt's
                # (review-flagged: closing over the loop variables by
                # name, not by value, let a late arrival corrupt the
                # attempt that had already moved on).
                try:
                    result['r'] = fut.result()
                except Exception as exc:
                    result['exc'] = exc
                done.set()

            self._compute_ik_client.call_async(req).add_done_callback(_cb)
            if not done.wait(timeout=3.0):
                continue
            if 'exc' in result:
                self.get_logger().warn(f'/compute_ik call raised: {result["exc"]!r}')
                continue
            r = result.get('r')
            if r is None or r.error_code.val != MoveItErrorCodes.SUCCESS:
                continue

            names = r.solution.joint_state.name
            positions = r.solution.joint_state.position
            if len(names) != len(positions):
                self.get_logger().warn(
                    f'/compute_ik returned mismatched name/position array lengths '
                    f'({len(names)} vs {len(positions)}) — discarding.'
                )
                continue
            solution = dict(zip(names, positions))
            missing = [name for name in JOINT_LIMITS if name not in solution]
            if missing:
                self.get_logger().warn(f'/compute_ik solution missing joints {missing} — discarding.')
                continue
            solution = {name: solution[name] for name in JOINT_LIMITS}
            if not all(math.isfinite(pos) for pos in solution.values()):
                self.get_logger().warn(f'/compute_ik solution has a non-finite value — discarding: {solution}')
                continue
            if all(_joint_limit_margin(name, pos) >= margin_fraction for name, pos in solution.items()):
                valid_solutions.append(solution)

        if not valid_solutions:
            return None
        if not have_current:
            return valid_solutions[0]
        return min(valid_solutions, key=lambda sol: _joint_space_distance(sol, current))

    def _apply_panel_collision_object(self, panel_pose: PoseStamped) -> bool:
        if not self._apply_scene_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().error('/apply_planning_scene service not available')
            return False

        p = panel_pose.pose.position
        o = panel_pose.pose.orientation
        collision_center_pos, collision_center_quat = compose_transforms(
            ((p.x, p.y, p.z), (o.x, o.y, o.z, o.w)),
            PANEL_COLLISION_LOCAL_OFFSET,
        )
        collision_pose = Pose()
        (collision_pose.position.x, collision_pose.position.y,
         collision_pose.position.z) = collision_center_pos
        (collision_pose.orientation.x, collision_pose.orientation.y,
         collision_pose.orientation.z, collision_pose.orientation.w) = collision_center_quat

        co = CollisionObject()
        co.header = panel_pose.header
        co.id = 'panel'
        co.primitives = [SolidPrimitive(
            type=SolidPrimitive.BOX, dimensions=[PANEL_WIDTH, PANEL_DEPTH, PANEL_HEIGHT])]
        co.primitive_poses = [collision_pose]
        co.operation = CollisionObject.ADD

        scene = PlanningScene(is_diff=True)
        scene.world = PlanningSceneWorld(collision_objects=[co])

        req = ApplyPlanningScene.Request(scene=scene)
        done = threading.Event()
        result = {}

        def _cb(fut):
            result['r'] = fut.result()
            done.set()

        self._apply_scene_client.call_async(req).add_done_callback(_cb)
        if not done.wait(timeout=3.0):
            return False
        return bool(result.get('r') and result['r'].success)

    def _build_joint_goal_constraints(self, joint_positions: dict) -> Constraints:
        """Exact joint-space goal — used by remembered-position replay and,
        via _compute_ik_within_panel_limits' solution, by live align and
        orient_gripper too.

        Unlike a Cartesian pose constraint (a tolerance REGION, which is
        what let OMPL land on a different joint solution every call), a
        joint constraint with a tight tolerance pins down the exact final
        joint vector: whatever path gets planned there (fresh and
        collision-checked against the CURRENT scene every call), the arm
        always ends up at exactly the same pose.
        """
        jcs = []
        for name, position in joint_positions.items():
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = position
            jc.tolerance_above = 1e-4
            jc.tolerance_below = 1e-4
            jc.weight = 1.0
            jcs.append(jc)
        return Constraints(joint_constraints=jcs)

    def _request_plan(self, goal_constraints: Constraints, extra_position_constraints=None):
        if not self._move_action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('/move_action server not available')
            return None

        goal = MoveGroup.Goal()
        req = goal.request
        req.group_name = GROUP_NAME
        # Explicit on purpose: an empty pipeline_id does NOT reliably
        # fall back to the configured default_planning_pipeline (ompl) —
        # confirmed live: once arm_moveit_config also loads chomp and
        # pilz_industrial_motion_planner alongside ompl, an empty
        # pipeline_id on this MoveGroup action request resolved to chomp
        # instead, which can't plan a Cartesian (position + orientation
        # constraint) goal at all
        # ("Only joint-space goals are supported" — move_group rejects
        # the whole request with MoveItErrorCodes.INVALID_GOAL_CONSTRAINTS
        # before planning even starts).
        req.pipeline_id = 'ompl'
        req.num_planning_attempts = self.get_parameter('num_planning_attempts').value
        req.allowed_planning_time = self.get_parameter('allowed_planning_time').value
        req.max_velocity_scaling_factor = self.get_parameter('max_velocity_scaling_factor').value
        req.max_acceleration_scaling_factor = self.get_parameter('max_acceleration_scaling_factor').value
        req.start_state.is_diff = True
        req.goal_constraints = [goal_constraints]
        # Every plan this node ever requests is panel-related — safe to
        # apply unconditionally (see PANEL_JOINT_LIMIT_PATH_CONSTRAINTS).
        # extra_position_constraints (orient_gripper's own "stay near the
        # current position for the WHOLE path" bound, not just the goal)
        # rides alongside it on the same path_constraints message.
        req.path_constraints = Constraints(
            joint_constraints=list(PANEL_JOINT_LIMIT_PATH_CONSTRAINTS.joint_constraints),
            position_constraints=list(extra_position_constraints or []),
        )
        goal.planning_options = PlanningOptions(plan_only=True)

        done = threading.Event()
        result = {}

        def _result_cb(fut):
            result['r'] = fut.result().result
            done.set()

        def _goal_cb(fut):
            gh = fut.result()
            if gh is None or not gh.accepted:
                done.set()
                return
            gh.get_result_async().add_done_callback(_result_cb)

        self._move_action_client.send_goal_async(goal).add_done_callback(_goal_cb)
        if not done.wait(timeout=self.get_parameter('allowed_planning_time').value + 5.0):
            return None
        return result.get('r')

    def _check_joint_margins(self, planned_trajectory) -> tuple[bool, str]:
        points = planned_trajectory.joint_trajectory.points
        names = planned_trajectory.joint_trajectory.joint_names
        if not points:
            return False, 'planned trajectory has no points'
        last = points[-1]
        margin_fraction = self.get_parameter('joint_limit_margin_fraction').value

        worst_joint, worst_margin = None, float('inf')
        for name, position in zip(names, last.positions):
            if name not in JOINT_LIMITS:
                continue
            margin = _joint_limit_margin(name, position)
            if margin < worst_margin:
                worst_joint, worst_margin = name, margin

        if worst_joint is not None and worst_margin < margin_fraction:
            return False, f'{worst_joint} only {worst_margin:.1%} clear of its limit (need {margin_fraction:.1%})'
        return True, ''

    def _execute(self, trajectory) -> tuple[bool, str]:
        if not self._execute_client.wait_for_server(timeout_sec=5.0):
            return False, '/execute_trajectory server not available'

        goal = ExecuteTrajectory.Goal(trajectory=trajectory)
        done = threading.Event()
        result = {}
        goal_handle = {}

        def _result_cb(fut):
            result['r'] = fut.result()
            done.set()

        def _goal_cb(fut):
            gh = fut.result()
            if gh is None or not gh.accepted:
                done.set()
                return
            goal_handle['gh'] = gh
            gh.get_result_async().add_done_callback(_result_cb)

        self._execute_client.send_goal_async(goal).add_done_callback(_goal_cb)
        if not done.wait(timeout=self.get_parameter('execution_timeout_sec').value):
            # Critical: NOT cancelling here would leave the JTC goal
            # running server-side after this function returns "failed" —
            # keyboard_servo_node.py's align handler unconditionally
            # restarts Servo (switching to the streaming controller)
            # right after any align outcome, success or failure. Without
            # this cancel-and-wait, that reactivation would race an
            # execution possibly still in flight, i.e. two control paths
            # (JTC finishing the old trajectory, Servo streaming the new
            # one) briefly commanding the same joints. Cancelling and
            # waiting for the terminal state means align_to_panel() never
            # returns until the arm is actually no longer moving under
            # this goal, so it's safe for the caller to switch
            # controllers immediately after.
            gh = goal_handle.get('gh')
            if gh is not None:
                cancel_done = threading.Event()
                gh.cancel_goal_async().add_done_callback(lambda _f: cancel_done.set())
                cancel_done.wait(timeout=5.0)
                # Give the result callback a chance to arrive after the
                # cancel is acknowledged, so we don't return while the
                # goal is still mid-cancellation on the server side.
                done.wait(timeout=5.0)
            return False, 'execution timed out (goal cancelled)'

        wrapped = result.get('r')
        if wrapped is None or wrapped.status != GoalStatus.STATUS_SUCCEEDED:
            return False, f'execution goal status={wrapped.status if wrapped else "none"}'
        if wrapped.result.error_code.val != MoveItErrorCodes.SUCCESS:
            return False, f'MoveItErrorCodes.val={wrapped.result.error_code.val}'
        return True, ''


def main():
    rclpy.init()
    node = PanelAlignNode()
    # MultiThreadedExecutor, not the default rclpy.spin() — required so
    # the align service callback and its own sub-calls' response
    # callbacks (same ReentrantCallbackGroup, see __init__) can actually
    # interleave across threads instead of self-deadlocking.
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
