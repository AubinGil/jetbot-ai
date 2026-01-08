from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the slam_toolbox share directory
    slam_toolbox_share = FindPackageShare('slam_toolbox')
    jetbot_hardware_share = FindPackageShare('jetbot_hardware')

    return LaunchDescription([
        DeclareLaunchArgument(
            'save_interval',
            default_value='30.0',
            description='Map auto-save interval in seconds'
        ),

        DeclareLaunchArgument(
            'map_directory',
            default_value=os.path.expanduser('~/maps'),
            description='Directory to save maps'
        ),

        DeclareLaunchArgument(
            'map_name',
            default_value='jetbot_map',
            description='Base name for saved maps'
        ),

        DeclareLaunchArgument(
            'keep_backups',
            default_value='true',
            description='Keep timestamped backups of previous maps'
        ),

        DeclareLaunchArgument(
            'slam_params_file',
            default_value='',
            description='Path to SLAM Toolbox params file (optional)'
        ),

        # SLAM Toolbox - Online Async SLAM (launched directly)
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[
                os.path.join(
                    get_package_share_directory('jetbot_hardware'),
                    'config',
                    'slam_params.yaml'
                ),
                {'use_sim_time': False}
            ]
        ),

        # Map Auto-Saver Node
        Node(
            package='jetbot_hardware',
            executable='map_auto_saver',
            name='map_auto_saver',
            output='screen',
            parameters=[{
                'save_interval': 30.0,
                'map_directory': LaunchConfiguration('map_directory'),
                'map_name': LaunchConfiguration('map_name'),
                'keep_backups': LaunchConfiguration('keep_backups'),
            }]
        ),

        # Optional: Static transform from base_link to laser_frame
        # Adjust these values based on your robot's actual sensor placement
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_laser_publisher',
            arguments=['0', '0', '0.1', '0', '0', '0', 'base_link', 'laser_frame']
        ),
    ])
