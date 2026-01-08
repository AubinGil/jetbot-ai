#!/usr/bin/env python3
"""
Launch joystick teleop: joy_node + teleop_twist_joy.
Requires: ros-humble-joy, ros-humble-teleop-twist-joy
"""
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_dir = get_package_share_directory('jetbot_conversation')
    default_cfg = os.path.join(pkg_dir, 'config', 'teleop_joy.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            default_value=default_cfg,
            description='Path to teleop_joy YAML config'
        ),

        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
            parameters=[LaunchConfiguration('config_file')],
            emulate_tty=True,
        ),

        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_twist_joy',
            output='screen',
            parameters=[LaunchConfiguration('config_file')],
            remappings=[('cmd_vel', 'cmd_vel')],
            emulate_tty=True,
        ),
    ])

