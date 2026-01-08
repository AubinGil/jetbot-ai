#!/usr/bin/env python3
"""
Complete Jetbot launch file with encoder-based odometry
Launches motor controller with encoder feedback, camera, and optionally LIDAR
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Launch arguments
    use_lidar = LaunchConfiguration('use_lidar')
    use_camera = LaunchConfiguration('use_camera')
    use_encoders = LaunchConfiguration('use_encoders')

    declare_use_lidar_cmd = DeclareLaunchArgument(
        'use_lidar',
        default_value='true',
        description='Whether to launch LiDAR node')

    declare_use_camera_cmd = DeclareLaunchArgument(
        'use_camera',
        default_value='true',
        description='Whether to launch camera publisher')

    declare_use_encoders_cmd = DeclareLaunchArgument(
        'use_encoders',
        default_value='true',
        description='Use encoder-based motor controller (true) or basic controller (false)')

    # Motor controller with encoder feedback
    motor_controller_encoders_node = Node(
        package='jetbot_hardware',
        executable='motor_controller_encoders',
        name='motor_controller',
        output='screen',
        parameters=[{
            'port': '/dev/serial/by-id/usb-TinyUSB_TinyUSB_Device_123456-if00',
            'baud_rate': 115200,
            'max_accel': 1.5,
            'max_angular_accel': 3.0,
            'linear_deadband': 0.01,
            'angular_deadband': 0.02,
            'publish_odom_transform': True,
            'linear_correction': 1.0,
            'angular_correction': 1.0,
        }],
        condition=IfCondition(use_encoders)
    )

    # Basic motor controller (fallback)
    motor_controller_basic_node = Node(
        package='jetbot_hardware',
        executable='motor_controller',
        name='motor_controller',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_encoders', default='false'))
    )

    # Camera publisher
    camera_node = Node(
        package='jetbot_hardware',
        executable='camera_publisher',
        name='camera_publisher',
        output='screen',
        condition=IfCondition(use_camera)
    )

    # Static transform: base_link to base_footprint
    base_link_to_footprint = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_footprint',
        arguments=['0', '0', '0.05', '0', '0', '0', 'base_footprint', 'base_link']
    )

    # Static transform: base_link to laser_frame (if using LiDAR)
    base_link_to_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_laser',
        arguments=['0', '0', '0.1', '0', '0', '0', 'base_link', 'laser_frame'],
        condition=IfCondition(use_lidar)
    )

    ld = LaunchDescription()

    # Add launch arguments
    ld.add_action(declare_use_lidar_cmd)
    ld.add_action(declare_use_camera_cmd)
    ld.add_action(declare_use_encoders_cmd)

    # Add nodes
    ld.add_action(motor_controller_encoders_node)
    ld.add_action(motor_controller_basic_node)
    ld.add_action(camera_node)
    ld.add_action(base_link_to_footprint)
    ld.add_action(base_link_to_laser)

    return ld
