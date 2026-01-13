#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('device', default_value='/dev/video0'),
        DeclareLaunchArgument('width', default_value='640'),
        DeclareLaunchArgument('height', default_value='480'),
        DeclareLaunchArgument('fps', default_value='15'),
        
        # GStreamer camera node (works better than OpenCV on Jetson)
        Node(
            package='v4l2_camera',
            executable='v4l2_camera_node',
            name='camera',
            parameters=[{
                'video_device': LaunchConfiguration('device'),
                'image_size': [
                    LaunchConfiguration('width'),
                    LaunchConfiguration('height')
                ],
                'camera_frame_id': 'camera_link',
                'output_encoding': 'rgb8',
                'framerate': LaunchConfiguration('fps'),
            }],
            remappings=[
                ('/image_raw', '/camera/image_raw'),
                ('/camera_info', '/camera/camera_info'),
            ]
        ),
    ])
