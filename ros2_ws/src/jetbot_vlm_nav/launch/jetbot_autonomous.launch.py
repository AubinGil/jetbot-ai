#!/usr/bin/env python3
"""
Complete Jetbot Autonomous Navigation with RPLidar and Camera
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_dir = get_package_share_directory('jetbot_vlm_nav')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    
    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('rplidar_port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('camera_device', default_value='/dev/video0'),
        
        # RPLidar
        Node(
            package='rplidar_ros',
            executable='rplidar_composition',
            name='rplidar',
            parameters=[{
                'serial_port': LaunchConfiguration('rplidar_port'),
                'frame_id': 'laser_frame',
                'angle_compensate': True,
                'scan_mode': 'Standard',
            }],
            output='screen'
        ),
        
        # Camera (using v4l2_camera for Jetson compatibility)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_dir, 'launch', 'jetbot_camera.launch.py')
            ),
            launch_arguments={'device': LaunchConfiguration('camera_device')}.items()
        ),
        
        # Nav2 Stack
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
            ),
            launch_arguments={
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'params_file': os.path.join(pkg_dir, 'config', 'nav2_params.yaml'),
            }.items()
        ),
    ])
