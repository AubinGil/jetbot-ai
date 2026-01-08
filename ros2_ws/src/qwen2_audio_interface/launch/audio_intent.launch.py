#!/usr/bin/env python3
"""
Launch file for Qwen2-Audio voice interface
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Declare launch arguments
    server_url_arg = DeclareLaunchArgument(
        'server_url',
        default_value='http://localhost:8080/v1/chat/completions',
        description='URL of the llama.cpp server'
    )

    model_name_arg = DeclareLaunchArgument(
        'model_name',
        default_value='qwen2audio',
        description='Model name to use in requests'
    )

    audio_topic_arg = DeclareLaunchArgument(
        'audio_topic',
        default_value='/mic/audio',
        description='Topic to subscribe for audio input'
    )

    intent_topic_arg = DeclareLaunchArgument(
        'intent_topic',
        default_value='/audio_intent',
        description='Topic to publish audio intent output'
    )

    sample_rate_arg = DeclareLaunchArgument(
        'sample_rate',
        default_value='16000',
        description='Audio sample rate in Hz'
    )

    # Create the node
    audio_intent_node = Node(
        package='qwen2_audio_interface',
        executable='audio_intent_node',
        name='audio_intent_node',
        output='screen',
        parameters=[{
            'server_url': LaunchConfiguration('server_url'),
            'model_name': LaunchConfiguration('model_name'),
            'audio_topic': LaunchConfiguration('audio_topic'),
            'intent_topic': LaunchConfiguration('intent_topic'),
            'sample_rate': LaunchConfiguration('sample_rate'),
        }]
    )

    return LaunchDescription([
        server_url_arg,
        model_name_arg,
        audio_topic_arg,
        intent_topic_arg,
        sample_rate_arg,
        audio_intent_node,
    ])
