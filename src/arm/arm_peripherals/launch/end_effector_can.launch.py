from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'end_effector',
            default_value='jaw',
            description=(
                "Which tool is physically mounted right now: 'jaw', "
                "'drill_sampling', or 'astrobio' — see arm_macro.xacro. "
                'Match this to arm_bringup/arm.launch.py end_effector:=...'
            ),
        ),
        DeclareLaunchArgument(
            'load_poll_period_sec',
            default_value='0.5',
            description='jaw only: how often to request load-sensor telemetry.',
        ),
        Node(
            package='arm_peripherals',
            executable='end_effector_can_node',
            name='end_effector_can_node',
            output='screen',
            parameters=[{
                'end_effector': LaunchConfiguration('end_effector'),
                'load_poll_period_sec': LaunchConfiguration('load_poll_period_sec'),
            }],
        ),
    ])
