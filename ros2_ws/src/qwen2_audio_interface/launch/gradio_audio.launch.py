#!/usr/bin/env python3
"""
Launch file for Qwen2-Audio Gradio Interface Node
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Generate launch description for Gradio audio node"""

    # Declare launch arguments
    gradio_url_arg = DeclareLaunchArgument(
        'gradio_url',
        default_value='https://4f127fdb7db57f9e2b.gradio.live',
        description='URL of the Gradio server'
    )

    audio_topic_arg = DeclareLaunchArgument(
        'audio_topic',
        default_value='/mic/audio',
        description='Topic to subscribe to for audio input'
    )

    intent_topic_arg = DeclareLaunchArgument(
        'intent_topic',
        default_value='/audio_intent',
        description='Topic to publish audio intent/transcription'
    )

    sample_rate_arg = DeclareLaunchArgument(
        'sample_rate',
        default_value='16000',
        description='Audio sample rate in Hz'
    )

    api_name_arg = DeclareLaunchArgument(
        'api_name',
        default_value='/predict',
        description='Gradio API endpoint name'
    )

    # Create node
    gradio_audio_node = Node(
        package='qwen2_audio_interface',
        executable='gradio_audio_node',
        name='gradio_audio_node',
        output='screen',
        parameters=[{
            'gradio_url': LaunchConfiguration('gradio_url'),
            'audio_topic': LaunchConfiguration('audio_topic'),
            'intent_topic': LaunchConfiguration('intent_topic'),
            'sample_rate': LaunchConfiguration('sample_rate'),
            'api_name': LaunchConfiguration('api_name'),
        }]
    )

    return LaunchDescription([
        gradio_url_arg,
        audio_topic_arg,
        intent_topic_arg,
        sample_rate_arg,
        api_name_arg,
        gradio_audio_node,
    ])
