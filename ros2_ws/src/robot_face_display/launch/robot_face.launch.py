#!/usr/bin/env python3
"""
Launch file for Robot Face Display
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'fullscreen',
            default_value='true',
            description='Run robot face in fullscreen mode'
        ),

        Node(
            package='robot_face_display',
            executable='robot_face_node.py',
            name='robot_face_node',
            output='screen',
            parameters=[{
                'fullscreen': LaunchConfiguration('fullscreen')
            }]
        ),
    ])
