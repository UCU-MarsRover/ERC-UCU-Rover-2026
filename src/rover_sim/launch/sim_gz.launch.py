import os
import shutil
import tempfile
from string import Template

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    OpaqueFunction,
    AppendEnvironmentVariable,
    IncludeLaunchDescription,
    TimerAction,
    RegisterEventHandler,
)
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
    IfElseSubstitution,
    NotEqualsSubstitution,
)
from launch_ros.actions import Node, PushRosNamespace
from rover_bringup.launch_utils import include_launch

BASE_CONTROLLERS = [
    "joint_state_broadcaster",
    "odometry_controller",
    "diff_bar_effort_controller",
]

LEGACY_SWERVE_CONTROLLER = "swerve_controller"
SHAPE_SWERVE_CONTROLLER = "swerve_controller_test"
SWERVE_CONTROLLERS = [LEGACY_SWERVE_CONTROLLER, SHAPE_SWERVE_CONTROLLER]
DEFAULT_SWERVE_CONTROLLER = SHAPE_SWERVE_CONTROLLER

# List of available worlds.
SUPPORTED_WORLDS = ["mars_yard", "nav2_test_world"]

# List of worlds that utilize the dynamic resolution map loading feature.
SUPPORTED_RESOLUTION_WORLDS = ["mars_yard"]

# List of available mars_yard map years.
SUPPORTED_MAP_YEARS = ["2025", "2026"]

# List of available map resolutions. Only applies to the 2025 mars_yard map.
SUPPORTED_MAP_RESOLUTIONS = ["low", "medium", "high"]

# Default coordinates for rover to spawn
DEFAULT_SPAWN_X = 0.0
DEFAULT_SPAWN_Y = 0.0
DEFAULT_SPAWN_Z = 0.5

# Mesh/material extensions tracked via Git LFS that must be resolved before simulation can start.
LFS_TRACKED_EXTENSIONS = (".obj",)


def _is_unresolved_lfs_pointer(path: str) -> bool:
    """An unpulled LFS file checks out as a small text pointer instead of the real binary."""
    try:
        with open(path, "rb") as f:
            head = f.read(200)
    except OSError:
        return False
    return head.startswith(b"version https://git-lfs.github.com/spec/v1")


def check_git_lfs(rover_sim_share: str) -> None:
    """Fails fast with actionable instructions if the Git LFS mesh assets weren't pulled."""
    unresolved_files = []
    models_dir = os.path.join(rover_sim_share, "models")
    if os.path.isdir(models_dir):
        for root, _, files in os.walk(models_dir):
            for name in files:
                if name.lower().endswith(LFS_TRACKED_EXTENSIONS):
                    full_path = os.path.join(root, name)
                    if _is_unresolved_lfs_pointer(full_path):
                        unresolved_files.append(full_path)

    if not unresolved_files:
        return

    lines = [
        "", "=" * 70,
        "Simulation mesh assets are missing (Git LFS pointers were not resolved):",
    ]
    lines.extend(f"    {path}" for path in unresolved_files)
    lines += [
        "",
        "Fix it by running, from the root of your indomitus-rover-core clone:",
        "  sudo apt update && sudo apt install git-lfs   # or: brew install git-lfs (macOS)",
        "  git lfs install",
        "  git lfs pull",
        "",
        "See the 'Git LFS' section in src/rover_sim/README.md for details.",
        "=" * 70,
    ]
    raise RuntimeError("\n".join(lines))


def setup_environment() -> list:
    """Auto-detects host OS graphics and network issues, applying fixes instantly to the Python environment."""

    # IPC / Networking Fix (Forces local ZeroMQ. Prevents GUI black screen & spawner deaf loops)
    os.environ["GZ_IP"] = "127.0.0.1"
    os.environ["IGN_IP"] = "127.0.0.1"

    # Wayland GUI Crash Workaround (Forces X11 backend for Qt)
    if os.environ.get("XDG_SESSION_TYPE", "") == "wayland" or os.environ.get("WAYLAND_DISPLAY"):
        os.environ["QT_QPA_PLATFORM"] = "xcb"

    # Nvidia Optimus & EGL Deadlock Workaround (Prevents EGL dri2 crash)
    nvidia_egl_path = "/usr/share/glvnd/egl_vendor.d/10_nvidia.json"
    if os.path.exists(nvidia_egl_path):
        os.environ["__NV_PRIME_RENDER_OFFLOAD"] = "1"
        os.environ["__GLX_VENDOR_LIBRARY_NAME"] = "nvidia"
        os.environ["__EGL_VENDOR_LIBRARY_FILENAMES"] = nvidia_egl_path


def make_control_launch(context, controllers_yaml_path: str) -> list[IncludeLaunchDescription]:
    swerve_controller = LaunchConfiguration("swerve_controller").perform(context)

    return [include_launch("rover_bringup", "control.launch.py", {
        "use_sim": "true",
        "controllers_yaml": controllers_yaml_path,
        "controllers": f"{swerve_controller} " + " ".join(BASE_CONTROLLERS),
        "inactive_controllers": "",
    })]


def generate_bridge_and_rsp(context, rover_sim_share: str, rover_description_share: str, controllers_yaml_path: str, launch_tmp_dir: str, world_name_sub, model_name_sub, extra_xacro_args_sub) -> list:
    """Dynamically handles configuration switching for the bridge and robot description."""
    world = world_name_sub.perform(context)
    model = model_name_sub.perform(context)
    extra_xacro_args = extra_xacro_args_sub.perform(context)

    template_path = os.path.join(rover_sim_share, "config", "bridge_gz.yaml")

    with open(template_path) as f:
        rendered = Template(f.read()).substitute(world=world, model=model)

    bridge_yaml_path = os.path.join(launch_tmp_dir, f"bridge_{world}_{model}.yaml")
    with open(bridge_yaml_path, "w") as f:
        f.write(rendered)

    bridge_node = Node(
        package="ros_gz_bridge",
        name="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[{"use_sim_time": True}],
        arguments=["--ros-args", "-p", f"config_file:={bridge_yaml_path}"],
        output="screen",
    )

    rover_namespace = LaunchConfiguration("rover_namespace").perform(context)
    xacro_args = (
        f"use_sim:=true controllers_yaml_path:={controllers_yaml_path} "
        f"rover_namespace:={rover_namespace} {extra_xacro_args}"
    )

    rsp_launch = include_launch("rover_description", "robot_state_publisher.launch.py", {
        "xacro_file": os.path.join(rover_description_share, "urdf", "rover.xacro"),
        "xacro_args": xacro_args,
        "use_sim_time": "true",
    })

    return [bridge_node, rsp_launch]


def make_gazebo_launch(rover_sim_share: str, world_name_sub) -> IncludeLaunchDescription:
    """Abstracts Gazebo startup through the official ros_gz_sim package."""
    world_file = PathJoinSubstitution([
        rover_sim_share, "worlds", world_name_sub
    ])

    source = PythonLaunchDescriptionSource(
        os.path.join(get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py")
    )
    return IncludeLaunchDescription(source, launch_arguments={
        "gz_args": ["-r ", world_file, ".sdf"],
        "on_exit_shutdown": "True",
    }.items())


def make_spawn_node(model_name_sub, spawn_x_sub, spawn_y_sub, spawn_z_sub) -> Node:
    return Node(
        package="ros_gz_sim",
        name="rover_spawner",
        executable="create",
        arguments=[
            "-name", model_name_sub,
            "-topic", "robot_description",
            "-x", spawn_x_sub,
            "-y", spawn_y_sub,
            "-z", spawn_z_sub
        ],
        output="screen",
    )


def setup_dynamic_map(context, rover_sim_share: str, launch_tmp_dir: str, world_name_sub, map_resolution_sub, map_year_sub) -> list:
    world = world_name_sub.perform(context)

    # Only run the dynamic map generation if we are using the mars yard world
    if world not in SUPPORTED_RESOLUTION_WORLDS:
        return []

    resolution = map_resolution_sub.perform(context)
    year = map_year_sub.perform(context)

    tmp_model_dir = os.path.join(launch_tmp_dir, "mars_yard")
    tmp_meshes_dir = os.path.join(tmp_model_dir, "meshes")
    os.makedirs(tmp_meshes_dir, exist_ok=True)

    if year == "2025":
        source_meshes_dir = os.path.join(rover_sim_share, "models", "mars_yard_2025", "meshes")
        selected_obj = os.path.join(source_meshes_dir, f"mars_yard_2025_{resolution}_resolution.obj")
        target_obj = os.path.join(tmp_meshes_dir, "mars_yard.obj")

        if os.path.exists(selected_obj):
            shutil.copy(selected_obj, target_obj)
        else:
            raise RuntimeError(f"Resolution file not found: {selected_obj}")
    elif year == "2026":
        # Resolution argument is explicitly ignored for 2026 map
        source_meshes_dir = os.path.join(rover_sim_share, "models", "mars_yard_2026", "meshes")
        source_obj = os.path.join(source_meshes_dir, "mars_yard_2026.obj")

        target_obj = os.path.join(tmp_meshes_dir, "mars_yard.obj")

        if os.path.exists(source_obj):
            shutil.copy(source_obj, target_obj)
        else:
            raise RuntimeError(f"2026 Mesh file not found: {source_obj}")
    else:
        raise ValueError(f"Unknown map_year: {year}")

    sdf_content = """<?xml version="1.0" ?>
<sdf version="1.6">
  <model name="mars_yard">
    <static>true</static>
    <link name="map_link">
      <collision name="collision">
        <geometry>
          <mesh>
            <uri>model://mars_yard/meshes/mars_yard.obj</uri>
          </mesh>
        </geometry>
      </collision>
      <visual name="visual">
        <cast_shadows>false</cast_shadows>
        <geometry>
          <mesh>
            <uri>model://mars_yard/meshes/mars_yard.obj</uri>
          </mesh>
        </geometry>
        <material>
          <ambient>0.6 0.3 0.1 1</ambient>
          <diffuse>0.7 0.35 0.15 1</diffuse>
          <specular>0.1 0.1 0.1 1</specular>
        </material>
      </visual>
    </link>
  </model>
</sdf>
"""

    model_config_content = """<?xml version="1.0"?>
<model>
  <name>mars_yard</name>
  <version>1.0</version>
  <sdf version="1.6">model.sdf</sdf>
</model>"""
    
    with open(os.path.join(tmp_model_dir, "model.config"), "w") as f:
        f.write(model_config_content)

    with open(os.path.join(tmp_model_dir, "model.sdf"), "w") as f:
        f.write(sdf_content)

    return []


def generate_launch_description() -> LaunchDescription:
    setup_environment()

    rover_description_share = get_package_share_directory("rover_description")
    zed_description_share = get_package_share_directory("zed_description")
    rover_sim_share = get_package_share_directory("rover_sim")

    check_git_lfs(rover_sim_share)

    controllers_yaml_path = os.path.join(rover_sim_share, "config", "controllers.yaml")
    LAUNCH_TMP_DIR = tempfile.mkdtemp(prefix="rover_sim_")

    def cleanup_tmp_dir(context):
        if os.path.exists(LAUNCH_TMP_DIR):
            shutil.rmtree(LAUNCH_TMP_DIR)

    cleanup_handler = RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(function=cleanup_tmp_dir)]))

    is_world_name_present = NotEqualsSubstitution(LaunchConfiguration("world_name"), "")
    is_map_resolution_present = NotEqualsSubstitution(LaunchConfiguration("map_resolution"), "")
    is_map_year_present = NotEqualsSubstitution(LaunchConfiguration("map_year"), "")
    is_model_name_present = NotEqualsSubstitution(LaunchConfiguration("model_name"), "")
    is_spawn_x_present = NotEqualsSubstitution(LaunchConfiguration("spawn_x"), "")
    is_spawn_y_present = NotEqualsSubstitution(LaunchConfiguration("spawn_y"), "")
    is_spawn_z_present = NotEqualsSubstitution(LaunchConfiguration("spawn_z"), "")
    extra_xacro_args_present = NotEqualsSubstitution(LaunchConfiguration("extra_xacro_args"), "")

    world_name_sub = IfElseSubstitution(is_world_name_present, if_value=LaunchConfiguration("world_name"), else_value="mars_yard")
    map_resolution_sub = IfElseSubstitution(is_map_resolution_present, if_value=LaunchConfiguration("map_resolution"), else_value="high")
    map_year_sub = IfElseSubstitution(is_map_year_present, if_value=LaunchConfiguration("map_year"), else_value="2026")
    model_name_sub = IfElseSubstitution(is_model_name_present, if_value=LaunchConfiguration("model_name"), else_value="indomitus_rover")
    spawn_x_sub = IfElseSubstitution(is_spawn_x_present, if_value=LaunchConfiguration("spawn_x"), else_value=f"{DEFAULT_SPAWN_X}")
    spawn_y_sub = IfElseSubstitution(is_spawn_y_present, if_value=LaunchConfiguration("spawn_y"), else_value=f"{DEFAULT_SPAWN_Y}")
    spawn_z_sub = IfElseSubstitution(is_spawn_z_present, if_value=LaunchConfiguration("spawn_z"), else_value=f"{DEFAULT_SPAWN_Z}")
    extra_xacro_args_sub = IfElseSubstitution(extra_xacro_args_present, if_value=LaunchConfiguration("extra_xacro_args"), else_value="")

    return LaunchDescription([
        DeclareLaunchArgument(
            "swerve_controller",
            default_value=DEFAULT_SWERVE_CONTROLLER,
            choices=list(SWERVE_CONTROLLERS),
            description="Which swerve controller to spawn. It comes up active; "
                        "the other is not loaded at all."),
        DeclareLaunchArgument(
            "world_name",
            default_value="mars_yard",
            choices=["", *SUPPORTED_WORLDS],
            description=f"Gazebo world file to load (without .sdf extension). Available options: {SUPPORTED_WORLDS}."
        ),
        DeclareLaunchArgument(
            "map_year",
            default_value="2026",
            choices=["", *SUPPORTED_MAP_YEARS],
            description=f"The year of the map to load. Options: {SUPPORTED_MAP_YEARS}. Dynamically applies only to: mars_yard."
        ),
        DeclareLaunchArgument(
            "map_resolution",
            default_value="high",
            choices=["", *SUPPORTED_MAP_RESOLUTIONS],
            description=f"Options: {SUPPORTED_MAP_RESOLUTIONS}. Dynamically applies ONLY to: mars_yard (2025). Ignored for 2026 or other worlds."
        ),
        DeclareLaunchArgument("model_name", default_value="indomitus_rover", description="The name assigned to the robot model."),
        DeclareLaunchArgument("spawn_delay", default_value="5.0", description="Time (in seconds) to wait before spawning the robot."),
        DeclareLaunchArgument("extra_xacro_args", default_value="", description="Additional flags to pass to the URDF xacro compiler."),
        DeclareLaunchArgument("spawn_x", default_value=f"{DEFAULT_SPAWN_X}", description="Initial X coordinate (in meters)."),
        DeclareLaunchArgument("spawn_y", default_value=f"{DEFAULT_SPAWN_Y}", description="Initial Y coordinate (in meters)."),
        DeclareLaunchArgument("spawn_z", default_value=f"{DEFAULT_SPAWN_Z}", description="Initial Z coordinate (in meters)."),
        DeclareLaunchArgument(
            "rover_namespace",
            default_value=EnvironmentVariable("ROVER_NAMESPACE", default_value="rover"),
            description="ROS namespace all rover nodes/topics are pushed under (arm excluded).",
        ),

        # 1. Support modern Gazebo (Harmonic/Ionic)
        AppendEnvironmentVariable(
            name="GZ_SIM_RESOURCE_PATH",
            value=f"{os.path.dirname(rover_description_share)}:{os.path.dirname(zed_description_share)}:{LAUNCH_TMP_DIR}",
        ),

        # 2. Support Ignition Gazebo (Fortress)
        AppendEnvironmentVariable(
            name="IGN_GAZEBO_RESOURCE_PATH",
            value=f"{os.path.dirname(rover_description_share)}:{os.path.dirname(zed_description_share)}:{LAUNCH_TMP_DIR}",
        ),

        OpaqueFunction(
            function=setup_dynamic_map,
            kwargs={
                "rover_sim_share": rover_sim_share,
                "launch_tmp_dir": LAUNCH_TMP_DIR,
                "world_name_sub": world_name_sub,
                "map_resolution_sub": map_resolution_sub,
                "map_year_sub": map_year_sub
            }
        ),

        make_gazebo_launch(rover_sim_share, world_name_sub),

        GroupAction([
            PushRosNamespace(LaunchConfiguration("rover_namespace")),

            OpaqueFunction(
                function=generate_bridge_and_rsp,
                kwargs={
                    "rover_sim_share": rover_sim_share,
                    "rover_description_share": rover_description_share,
                    "controllers_yaml_path": controllers_yaml_path,
                    "launch_tmp_dir": LAUNCH_TMP_DIR,
                    "world_name_sub": world_name_sub,
                    "model_name_sub": model_name_sub,
                    "extra_xacro_args_sub": extra_xacro_args_sub
                }
            ),

            TimerAction(
                period=LaunchConfiguration("spawn_delay"),
                actions=[make_spawn_node(model_name_sub, spawn_x_sub, spawn_y_sub, spawn_z_sub)]
            ),

            include_launch("rover_localization", "ekf.launch.py", {
                "use_sim_time": "true",
            }),

            OpaqueFunction(function=make_control_launch,
                           kwargs={"controllers_yaml_path": controllers_yaml_path}),

            include_launch("rover_bringup", "twist_mux.launch.py", {
                "use_sim_time": "true",
            }),

            include_launch("rover_teleop", "drive_power.launch.py", {
                "use_sim_time": "true",
                "controller_name": LaunchConfiguration("swerve_controller"),
            }),
        ]),

        # Trigger folder cleanup safely upon launch termination
        cleanup_handler,
    ])