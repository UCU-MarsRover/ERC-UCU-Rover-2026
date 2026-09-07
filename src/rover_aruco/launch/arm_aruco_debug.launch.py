"""ArUco debug tracker for the ARM's own camera — analogous to
aruco_debug.launch.py, but for the arm instead of the rover.

Two differences from aruco_debug.launch.py:
- No usb_cam_node: the arm's camera (arm_sensors/launch/camera.launch.py,
  or Gazebo's bridge in sim) already publishes camera/image_raw +
  camera/camera_info, so this launch doesn't need its own camera source.
- No PushRosNamespace: the arm camera and everything downstream of it
  (panel_align_node, panel_pose_fuser_node) run unnamespaced, not under
  /rover — pushing this into /rover like aruco.launch.py/
  aruco_debug.launch.py do would just make the tracker deaf to the
  arm's actual topics.

Uses aruco_params.yaml (ARUCO_ORIGINAL dict), not
aruco_debug_params.yaml (4X4_50) — the panel's real markers are
ARUCO_ORIGINAL (see aruco_params.yaml's own comment), and this is for
debugging detection of THOSE, not generic printed test markers.

View the overlay in RViz: Add > By topic > aruco_tracker/debug.
"""
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def get_parameter_from_yaml(file_path, node_name, parameter_name):
    with open(file_path, "r", encoding="utf-8") as file:
        config = yaml.safe_load(file)

    try:
        return str(config[node_name]["ros__parameters"][parameter_name])
    except (KeyError, TypeError) as error:
        raise RuntimeError(
            f"Parameter '{parameter_name}' for node '{node_name}' "
            f"was not found in '{file_path}'"
        ) from error


def generate_launch_description():
    pkg_share = get_package_share_directory("rover_aruco")
    aruco_params = os.path.join(pkg_share, "config", "aruco_params.yaml")

    camera_topic = LaunchConfiguration("camera_topic")

    return LaunchDescription([
        DeclareLaunchArgument(
            "camera_topic",
            description="Camera image topic used by the ArUco tracker",
            default_value=get_parameter_from_yaml(
                aruco_params, "aruco_tracker", "cam_base_topic"
            ),
        ),
        Node(
            package="aruco_opencv",
            executable="aruco_tracker_autostart",
            name="aruco_tracker",
            output="screen",
            parameters=[
                aruco_params,
                {"cam_base_topic": camera_topic},
            ],
        ),
    ])
