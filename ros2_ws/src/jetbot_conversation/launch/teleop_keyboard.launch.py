#!/usr/bin/env python3
"""
Launch keyboard teleop to publish geometry_msgs/Twist on cmd_vel.
Requires: ros-humble-teleop-twist-keyboard
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='teleop_twist_keyboard',
            executable='teleop_twist_keyboard',
            name='teleop_twist_keyboard',
            output='screen',
            remappings=[('cmd_vel', 'cmd_vel')],
            emulate_tty=True,
            parameters=[{
                # No parameters supported by node, kept for symmetry
            }],
        )
    ])

