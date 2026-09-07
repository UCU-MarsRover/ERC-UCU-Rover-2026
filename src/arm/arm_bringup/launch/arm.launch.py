"""
Real top-level bringup for the Indomitus arm: MoveIt (move_group) +
moveit_servo, controllers, and (optionally) the on-rover half of gamepad
teleop — all under one launch command, headless by default. Lives in
arm_bringup rather than arm_moveit_config so that package can stay pure
MoveIt-Setup-Assistant-generated config/launch, safe to regenerate.

Servo streams joint *positions* (Float64MultiArray) to
indomitus_arm_forward_position_controller. That controller is spawned
inactive; arm_teleop switches JTC <-> forward around home / start_servo.

servo_node still waits until indomitus_arm_controller (JTC) has been
spawned successfully so /joint_states and the hardware stack are up.
"""

import os
import sys

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events.process import ProcessExited
from launch.launch_context import LaunchContext
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, PushRosNamespace
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch

ARM_CONTROLLER_NAME = "indomitus_arm_controller"
FORWARD_CONTROLLER_NAME = "indomitus_arm_forward_position_controller"


def load_yaml(package_name: str, relative_path: str):
    package_path = get_package_share_directory(package_name)
    absolute_path = os.path.join(package_path, relative_path)
    with open(absolute_path, "r") as f:
        return yaml.safe_load(f)


def _arg_from_argv(name: str, default: str) -> str:
    """Plain-str mappings force MoveItConfigsBuilder's single-eval xacro path — LaunchConfiguration ones desync across sub-launches."""
    prefix = f"{name}:="
    for arg in sys.argv:
        if arg.startswith(prefix):
            return arg[len(prefix):]
    return default


def _arg_from_argv_or_none(name: str):
    """Like _arg_from_argv(), but None (no fallback) instead of a default."""
    prefix = f"{name}:="
    for arg in sys.argv:
        if arg.startswith(prefix):
            return arg[len(prefix):]
    return None


def generate_launch_description() -> LaunchDescription:
    declare_use_fake_hardware_cmd = DeclareLaunchArgument(
        "use_fake_hardware",
        default_value="true",
        description="Whether to use fake hardware or real CAN bus",
    )

    end_effector = LaunchConfiguration("end_effector")
    declare_end_effector_cmd = DeclareLaunchArgument(
        "end_effector",
        default_value="jaw",
        description="'jaw', 'other_tool', or 'drill_sampling' — see arm_macro.xacro",
    )

    declare_can_interface_cmd = DeclareLaunchArgument(
        "can_interface",
        default_value="",
        description="SocketCAN interface for joints + end-effector; unset uses each component's own default.",
    )

    declare_bring_up_can_bridge_cmd = DeclareLaunchArgument(
        "bring_up_can_bridge",
        default_value="true",
        description="Bring up our own ros2_socketcan bridge; set false if rover.launch.py already did.",
    )
    bring_up_can_bridge = LaunchConfiguration("bring_up_can_bridge")

    # Default false: collision_link_reporter lives in arm_tasks, which is
    # excluded from the Jetson production image (see docker/Dockerfile's
    # SIMULATION_PKGS — arm_tasks's remaining nodes are pure network clients
    # meant to run from a dev/ground-station container, not on the Jetson
    # itself). A plain Jetson run of this launch file must not try to spawn
    # a node from a package that was never built into that image; opt in
    # explicitly with report_collisions:=true only when running from a
    # container that actually has arm_tasks built (e.g. the dev/GS one).
    declare_report_collisions_cmd = DeclareLaunchArgument(
        "report_collisions",
        default_value="false",
        description=(
            "Run collision_link_reporter alongside move_group — logs exactly "
            "which link pair is in contact (via /check_state_validity) "
            "whenever the current pose goes invalid. Read-only, cheap "
            "(2 Hz polling). Requires arm_tasks to be built on this machine "
            "(it is NOT part of the Jetson production image); leave false "
            "there and enable only from a dev/ground-station container."
        ),
    )

    declare_bring_up_gamepad_cmd = DeclareLaunchArgument(
        "bring_up_gamepad",
        default_value="true",
        description=(
            "Also bring up gamepad_servo_node (the rover-side half of "
            "gamepad teleop) here, so this one launch covers arm + on-rover "
            "gamepad control. The laptop-side half (joystick input) is "
            "still `arm_teleop/launch/gamepad_joy.launch.py`, run "
            "separately on the other machine."
        ),
    )

    declare_home_pose_name_cmd = DeclareLaunchArgument(
        "home_pose_name",
        default_value="",
        description=(
            "poses.json key that gamepad A drives to when not in sampling/"
            "drill mode. Leave empty (default) to auto-pick "
            "'{end_effector}_home' (e.g. 'jaw_home' for end_effector:=jaw) "
            "when that key exists in poses.json, else fall back to 'home'. "
            "Set explicitly only to override that auto-pick."
        ),
    )

    declare_activity_indicator_pre_delay_sec_cmd = DeclareLaunchArgument(
        "activity_indicator_pre_delay_sec",
        default_value="0.0",
        description=(
            "Seconds r/p/f/m (and gamepad A/B/Y/panel-align/orient) wait, "
            "arm untouched, before actually moving — ERC 2026 Rules, "
            "Appendix 3, REQ-OPS-080/090/100 require >= 5.0 at actual "
            "competition. Defaults to 0.0 (no wait) for bench testing — set "
            "explicitly to 5.0 (or higher) for a real competition run, "
            "nothing enforces that automatically."
        ),
    )

    # Runtime (LaunchConfiguration) form, for the IfCondition/UnlessCondition
    # gripper-spawner split below — separate from the plain-string
    # _arg_from_argv() value passed into robot_description(mappings=...),
    # which must stay a plain string (see _arg_from_argv's own docstring).
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")

    # planning_pipelines restricted to ompl on purpose — same fix as
    # arm_gazebo.launch.py (see its own comment for the full story): with
    # chomp/pilz_industrial_motion_planner also loaded, move_group picks
    # an ambiguous "planning_plugin" between them and silently falls back
    # to CHOMP even when a request explicitly asks for pipeline_id='ompl'
    # (as panel_align_node.py's align requests do), and CHOMP rejects any
    # Cartesian pose-constraint goal outright. This is the real-hardware
    # launch path, so panel align needs the same fix here as in sim.
    robot_description_mappings = {
        "use_fake_hardware": _arg_from_argv("use_fake_hardware", "true"),
        "end_effector": _arg_from_argv("end_effector", "jaw"),
    }
    can_interface_from_argv = _arg_from_argv_or_none("can_interface")
    if can_interface_from_argv is not None:
        robot_description_mappings["can_interface"] = can_interface_from_argv

    moveit_config = (
        MoveItConfigsBuilder("indomitus_arm", package_name="arm_moveit_config")
        .robot_description(mappings=robot_description_mappings)
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    servo_yaml = load_yaml("arm_moveit_config", "config/servo.yaml")
    servo_params = {"moveit_servo": servo_yaml["moveit_servo"]["ros__parameters"]}

    # Light smoothing only: heavy coeffs lag the command and, with high MIT
    # kp, the arm overshoots then waits — stop-go buzz at the Servo rate.
    smoothing_params = {"butterworth_filter_coeff": 2.0}

    # Do NOT pass robot_description_kinematics into servo_node.
    # Humble Servo prefers the KDL plugin (searchPositionIK, 5 ms timeout)
    # over the inverse Jacobian. From home, +X (W) often fails that IK so
    # cartesianServoCalcs() returns false and the last joint command is
    # republished (frozen commands, status still 0). -X (S) succeeds, so
    # teleop looks one-sided. Without a solver, Servo uses J^# which is
    # symmetric in ±X. move_group still gets KDL for Plan&Execute.
    servo_node = Node(
        package="moveit_servo",
        executable="servo_node_main",
        name="servo_node",
        namespace="arm",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            # Without this, joint_limits.yaml (per-joint max_velocity/
            # max_acceleration/position bounds) never reaches Servo's robot
            # model — it falls back to the bare URDF <limit> tags only, so
            # the wrist_1_wrist_2 singularity fence and the tuned speed
            # ceilings were silently not enforced during teleop.
            moveit_config.joint_limits,
            servo_params,
            smoothing_params,
        ],
    )

    def _on_process_exit(event: ProcessExited, context: LaunchContext):
        cmd_parts = [str(part) for part in (event.cmd or [])]

        is_spawner = any(part.endswith("spawner") for part in cmd_parts)
        has_controller_arg = ARM_CONTROLLER_NAME in cmd_parts

        if not (is_spawner and has_controller_arg):
            return None

        if event.returncode != 0:
            print(
                f"[servo_launch] Spawner for '{ARM_CONTROLLER_NAME}' "
                f"exited with code {event.returncode} — servo_node will NOT be started."
            )
            return None

        print(
            f"[servo_launch] Controller '{ARM_CONTROLLER_NAME}' is active — "
            f"starting servo_node (streams to '{FORWARD_CONTROLLER_NAME}'; "
            f"activate that controller before teleop via gamepad A / start_servo)."
        )
        return [servo_node]

    # move_group/moveit_servo are prebuilt binaries (not vendored here), so
    # their own collision-check code can't be patched to print link names —
    # this polls the same /check_state_validity service move_group already
    # exposes and logs the colliding link pair by name. See
    # arm_tasks/collision_link_reporter.py.
    collision_link_reporter = Node(
        package="arm_tasks",
        executable="collision_link_reporter",
        output="screen",
        condition=IfCondition(LaunchConfiguration("report_collisions")),
    )

    # Hosted here (Jetson, alongside controller_manager) rather than per-
    # process: keyboard_servo_node and panel_align_node both run as GS
    # clients of this same lock, over the network, for real cross-host
    # mutual exclusion — see arm_teleop/arm_motion_lock.py's own docstring.
    arm_motion_lock_server = Node(
        package="arm_teleop",
        executable="arm_motion_lock_server",
        output="screen",
    )

    # Load the streaming teleop controller inactive — JTC owns the joints until
    # arm_teleop switches controllers for Servo.
    forward_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[FORWARD_CONTROLLER_NAME, "--inactive"],
        output="screen",
    )

    demo_launch = generate_demo_launch(moveit_config)

    # Gripper controllers aren't spawned by generate_demo_launch(); only jaw
    # has finger joints, and which finger interfaces exist depends on
    # use_fake_hardware too: fake hardware (or sim) exposes both fingers,
    # but real hardware only stubs the right one (JawGripperStub — the real
    # arm has no gripper motor yet, see arm_macro.xacro) with no left-finger
    # interface at all, so spawning gripper_left_controller there would just
    # fail waiting for an interface that will never exist.
    gripper_both_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "gripper_right_controller",
            "gripper_left_controller",
            "--controller-manager-timeout", "60",
            "--switch-timeout", "60",
            "--service-call-timeout", "70",
        ],
        output="screen",
        condition=IfCondition(PythonExpression([
            "'", end_effector, "' == 'jaw' and '", use_fake_hardware, "' == 'true'"
        ])),
    )
    gripper_right_only_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "gripper_right_controller",
            "--controller-manager-timeout", "60",
            "--switch-timeout", "60",
            "--service-call-timeout", "70",
        ],
        output="screen",
        condition=IfCondition(PythonExpression([
            "'", end_effector, "' == 'jaw' and '", use_fake_hardware, "' != 'true'"
        ])),
    )

    real_hardware_condition = IfCondition(PythonExpression([
        "'", use_fake_hardware, "' != 'true'"
    ]))
    end_effector_can_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("arm_peripherals"),
                         "launch", "end_effector_can.launch.py")
        ),
        launch_arguments={"end_effector": end_effector}.items(),
        condition=real_hardware_condition,
    )

    # On-rover half of gamepad teleop — the laptop/GS half (joystick input,
    # game_controller_node) is launched separately via
    # arm_teleop/launch/gamepad_joy.launch.py on the other machine, same
    # split as rover_teleop's own joy.launch.py. Kept OUTSIDE arm_group on
    # purpose: gamepad_servo.launch.py's own gamepad_servo_node Node already
    # hardcodes namespace='arm' itself, and combining that with this file's
    # ambient PushRosNamespace('arm') double-nests everything it touches to
    # /arm/arm/... (confirmed live via `ros2 node list`/`ros2 topic list`
    # showing /arm/arm/joy, /arm/arm/keyboard_servo_node, etc. — the exact
    # same double-namespace bug this session already hit and fixed for
    # standalone RViz's "Move Group Namespace").
    gamepad_servo_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("arm_teleop"),
                         "launch", "gamepad_servo.launch.py")
        ),
        launch_arguments={
            "end_effector": end_effector,
            "home_pose_name": LaunchConfiguration("home_pose_name"),
            "activity_indicator_pre_delay_sec": LaunchConfiguration("activity_indicator_pre_delay_sec"),
        }.items(),
        condition=IfCondition(LaunchConfiguration("bring_up_gamepad")),
    )

    # Everything ROS-graph-facing for the arm lives under /arm — the rover
    # runs its own move_group/controller_manager/joint_states etc. under the
    # same bare names, so without this the two collide when run together.
    # The CAN bridge and gamepad_servo_include are deliberately kept OUTSIDE
    # this group: the CAN bridge is shared physical hardware the rover's own
    # bringup may already own, and gamepad_servo_include already sets its
    # own namespace='arm' (see its own comment above).
    arm_group = GroupAction([
        PushRosNamespace("arm"),
        *demo_launch.entities,
        arm_motion_lock_server,
        forward_spawner,
        collision_link_reporter,
        gripper_both_spawner,
        gripper_right_only_spawner,
        end_effector_can_include,
    ])

    ld = LaunchDescription()
    # DeclareLaunchArgument (unlike SetLaunchConfiguration) only applies its
    # default when the arg isn't already set — so use_rviz:=true on the CLI
    # still wins over this, and generate_demo_launch()'s own default (true)
    # never gets a chance to apply since this one runs first.
    ld.add_action(DeclareLaunchArgument("use_rviz", default_value="false"))
    ld.add_action(declare_use_fake_hardware_cmd)
    ld.add_action(declare_end_effector_cmd)
    ld.add_action(declare_can_interface_cmd)
    ld.add_action(declare_bring_up_can_bridge_cmd)
    ld.add_action(declare_report_collisions_cmd)
    ld.add_action(declare_bring_up_gamepad_cmd)
    ld.add_action(declare_home_pose_name_cmd)
    ld.add_action(declare_activity_indicator_pre_delay_sec_cmd)
    ld.add_action(arm_group)
    ld.add_action(gamepad_servo_include)

    ld.add_action(
        RegisterEventHandler(
            OnProcessExit(
                target_action=None,
                on_exit=_on_process_exit,
            )
        )
    )

    # reuses rover_bringup's ros2_socketcan bridge; filter covers only
    # jaw/astrobio/drill_sampling cmd/ack pairs (0x1A-0x1F), not joint IDs.
    # Kept unnamespaced — see arm_group's own comment above.
    can_bridge_condition = IfCondition(PythonExpression([
        "'", bring_up_can_bridge, "' == 'true' and '", use_fake_hardware, "' != 'true'"
    ]))

    can_bridge_launch_arguments = {"receiver_filters": "1A:7FE,1C:7FE,1E:7FE"}
    if can_interface_from_argv is not None:
        can_bridge_launch_arguments["interface"] = can_interface_from_argv

    can_bridge_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("rover_bringup"),
                         "launch", "can.launch.py")
        ),
        launch_arguments=can_bridge_launch_arguments.items(),
        condition=can_bridge_condition,
    )

    ld.add_action(can_bridge_include)

    return ld
