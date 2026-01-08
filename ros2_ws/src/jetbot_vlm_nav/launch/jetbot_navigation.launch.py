from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    
    # Declare launch arguments
    ollama_url_arg = DeclareLaunchArgument(
        'ollama_url',
        default_value='http://localhost:11434',
        description='Ollama server URL'
    )
    
    model_arg = DeclareLaunchArgument(
        'model',
        default_value='llava',
        description='Ollama VLM model name'
    )
    
    camera_topic_arg = DeclareLaunchArgument(
        'camera_topic',
        default_value='/camera/image_raw',
        description='Camera image topic'
    )
    
    # VLM Vision Node
    vlm_vision_node = Node(
        package='jetbot_vlm_nav',
        executable='vlm_vision_node.py',
        name='vlm_vision_node',
        output='screen',
        parameters=[{
            'ollama_url': LaunchConfiguration('ollama_url'),
            'model': LaunchConfiguration('model'),
            'camera_topic': LaunchConfiguration('camera_topic'),
            'processing_rate': 2.0
        }]
    )
    
    # Behavior Tree Node
    behavior_tree_node = Node(
        package='jetbot_vlm_nav',
        executable='behavior_tree_node.py',
        name='behavior_tree_node',
        output='screen'
    )
    
    return LaunchDescription([
        ollama_url_arg,
        model_arg,
        camera_topic_arg,
        vlm_vision_node,
        behavior_tree_node
    ])
