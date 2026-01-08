#!/usr/bin/env python3
"""
Launch file for SLAM-Toolbox with Jetbot
Uses encoder-based odometry and LID AR
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('jetbot_vlm_nav')
    slam_config = os.path.join(pkg_dir, 'slam_config', 'mapper_params_online_async.yaml')

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    slam_params_file = LaunchConfiguration('slam_params_file')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')

    declare_slam_params_file_cmd = DeclareLaunchArgument(
        'slam_params_file',
        default_value=slam_config,
        description='Full path to the ROS2 parameters file to use for slam_toolbox')

    # SLAM Toolbox node
    start_async_slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_params_file,
            {'use_sim_time': use_sim_time}
        ],
    )

    ld = LaunchDescription()

    # Add launch arguments
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_slam_params_file_cmd)

    # Add nodes
    ld.add_action(start_async_slam_toolbox_node)

    return ld
