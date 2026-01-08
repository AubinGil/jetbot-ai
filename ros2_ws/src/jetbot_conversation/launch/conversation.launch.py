#!/usr/bin/env python3
"""
Launch file for JetBot Conversation Pipeline
"""
from launch import LaunchDescription
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_dir = get_package_share_directory('jetbot_conversation')
    config_file = os.path.join(pkg_dir, 'config', 'conversation.yaml')
    
    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'config_file',
            default_value=config_file,
            description='Path to configuration file'
        ),
        DeclareLaunchArgument(
            'enable_audio',
            default_value='true',
            description='Enable audio capture (microphone)'
        ),
        DeclareLaunchArgument(
            'enable_tts',
            default_value='true',
            description='Enable text-to-speech output'
        ),
        
        # Conversation node (main brain)
        Node(
            package='jetbot_conversation',
            executable='conversation_node',
            name='conversation_node',
            output='screen',
            parameters=[LaunchConfiguration('config_file')],
            emulate_tty=True,
        ),
        
        # Audio capture node (STT)
        Node(
            package='jetbot_conversation',
            executable='audio_capture_node',
            name='audio_capture_node',
            output='screen',
            parameters=[LaunchConfiguration('config_file')],
            emulate_tty=True,
            condition=IfCondition(LaunchConfiguration('enable_audio')),
        ),
        
        # TTS node
        Node(
            package='jetbot_conversation',
            executable='tts_node',
            name='tts_node',
            output='screen',
            parameters=[LaunchConfiguration('config_file')],
            emulate_tty=True,
            condition=IfCondition(LaunchConfiguration('enable_tts')),
        ),
    ])
