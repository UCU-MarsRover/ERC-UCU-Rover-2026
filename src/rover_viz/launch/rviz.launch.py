import os
from typing import Dict, List, Optional

import launch.actions
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch.utilities import perform_substitutions
from launch_ros.actions import Node, PushRosNamespace


def urdf(mappings: Optional[Dict[str, str]] = None) -> str:
    urdf_xacro = os.path.join(
        get_package_share_directory('rover_description'),
        'urdf', 'rover.xacro',
    )
    try:
        doc = xacro.process_file(urdf_xacro, mappings=mappings or {})
        return doc.toprettyxml(indent='  ')
    except Exception as e:
        print(f"Error processing URDF: {e}")
        return ''


def launch_nodes(context: LaunchContext,
                 **substitutions: launch.substitutions.LaunchConfiguration
                 ) -> List[Node]:
    kwargs = {k: perform_substitutions(context, [v]) for k, v in substitutions.items()}

    use_sim   = kwargs.get('use_sim',   'false').lower() == 'true'
    use_nav   = kwargs.get('use_nav',   'false').lower() == 'true'
    use_rviz  = kwargs.get('use_rviz',  'true').lower()  == 'true'
    use_joint_gui = kwargs.get('use_joint_state_publisher_gui', 'true').lower() == 'true'
    # Kept as a string rather than a bool: it is handed straight to xacro, whose
    # mappings must be strings, and xacro only accepts lowercase true/false.
    use_stereo_camera = kwargs.get('use_stereo_camera', 'true').lower()

    nodes = []

    # When running alongside the simulator, robot_state_publisher and
    # joint_state_publisher are already started by the sim launch file.
    if not use_sim:
        urdf_string = urdf({'use_stereo_camera': use_stereo_camera})
        nodes.append(Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': urdf_string, 'publish_frequency': 100.0}],
            output='screen',
            arguments=['--ros-args', '--log-level', 'warn'],
            remappings=[
                ('tf', '/tf'),
                ('tf_static', '/tf_static'),
            ],
        ))

        publisher_pkg = 'joint_state_publisher_gui' if use_joint_gui else 'joint_state_publisher'
        nodes.append(Node(
            package=publisher_pkg,
            executable=publisher_pkg,
            name=publisher_pkg,
            output='screen',
        ))

    if use_rviz:
        rviz_dir = os.path.join(get_package_share_directory('rover_viz'), 'rviz')
        config_name = 'navigation.rviz' if use_nav else 'robot.rviz'
        rviz_config_file = os.path.join(rviz_dir, config_name)
        rviz_args = ['-d', rviz_config_file] if os.path.exists(rviz_config_file) else []
        nodes.append(Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=rviz_args,
        ))

    return nodes


def generate_launch_description():
    return LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            'use_sim', default_value='false',
            description='Skip robot_state_publisher and joint_state_publisher when running with the simulator.',
        ),
        launch.actions.DeclareLaunchArgument(
            'use_nav', default_value='false',
            description='Load navigation.rviz (fixed frame: map) instead of robot.rviz (fixed frame: base_link).',
        ),
        launch.actions.DeclareLaunchArgument(
            'use_rviz', default_value='true',
            description='Launch RViz2.',
        ),
        launch.actions.DeclareLaunchArgument(
            'use_joint_state_publisher_gui', default_value='true',
            description='Use joint_state_publisher_gui instead of joint_state_publisher.',
        ),
        launch.actions.DeclareLaunchArgument(
            'use_stereo_camera', default_value='true',
            description='Include the ZED2i in the robot description. Set false where the '
                        'ZED ROS 2 wrapper is unavailable: the URDF resolves '
                        '$(find zed_description), so without it the description fails to '
                        'build and RViz comes up with no robot.',
        ),
        launch.actions.DeclareLaunchArgument(
            'rover_namespace',
            default_value=EnvironmentVariable('ROVER_NAMESPACE', default_value='rover'),
            description='ROS namespace all rover nodes/topics are pushed under (arm excluded). '
                        'Must match the namespace of the rover graph being visualized.',
        ),
        launch.actions.GroupAction([
            PushRosNamespace(LaunchConfiguration('rover_namespace')),
            launch.actions.OpaqueFunction(
                function=launch_nodes,
                kwargs={
                    'use_sim':   launch.substitutions.LaunchConfiguration('use_sim'),
                    'use_nav':   launch.substitutions.LaunchConfiguration('use_nav'),
                    'use_rviz':  launch.substitutions.LaunchConfiguration('use_rviz'),
                    'use_joint_state_publisher_gui': launch.substitutions.LaunchConfiguration(
                        'use_joint_state_publisher_gui'),
                    'use_stereo_camera': launch.substitutions.LaunchConfiguration(
                        'use_stereo_camera'),
                },
            ),
        ]),
    ])