import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare launch arguments
    joy_dev_arg = DeclareLaunchArgument(
        'joy_dev',
        default_value='/dev/input/js0',
        description='Joystick device path'
    )

    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value='ps5_config.yaml',
        description='Configuration file name (in config folder)'
    )

    # Get config file path
    config_file = PathJoinSubstitution([
        FindPackageShare('b2w_joystick_control'),
        'config',
        LaunchConfiguration('config_file')
    ])

    # Joy node - publishes joystick raw inputs to /joy topic
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[{
            'dev': LaunchConfiguration('joy_dev'),
            'deadzone': 0.05,
            'autorepeat_rate': 10.0,
        }],
        output='screen'
    )

    # Joystick teleop node - converts /joy to /cmd_vel
    teleop_node = Node(
        package='b2w_joystick_control',
        executable='joystick_teleop_node',
        name='joystick_teleop',
        parameters=[config_file],
        output='screen'
    )

    return LaunchDescription([
        joy_dev_arg,
        config_file_arg,
        joy_node,
        teleop_node,
    ])
