#!/usr/bin/env python3
"""
Launch file for wall detection and line following navigation
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare launch arguments
    camera_topic_arg = DeclareLaunchArgument(
        'camera_topic',
        default_value='/camera/image_raw',
        description='Camera image topic'
    )

    cmd_vel_topic_arg = DeclareLaunchArgument(
        'cmd_vel_topic',
        default_value='/cmd_vel',
        description='Velocity command topic'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )

    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('jetbot_wall_line_nav'),
            'config',
            'nav_params.yaml'
        ]),
        description='Path to configuration file'
    )

    # Wall Line Navigator Node
    wall_line_nav_node = Node(
        package='jetbot_wall_line_nav',
        executable='wall_line_navigator',
        name='wall_line_navigator',
        output='screen',
        parameters=[
            LaunchConfiguration('config_file'),
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }
        ],
        remappings=[
            ('/camera/image_raw', LaunchConfiguration('camera_topic')),
            ('/cmd_vel', LaunchConfiguration('cmd_vel_topic')),
        ]
    )

    return LaunchDescription([
        camera_topic_arg,
        cmd_vel_topic_arg,
        use_sim_time_arg,
        config_file_arg,
        wall_line_nav_node,
    ])
