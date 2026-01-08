from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    
    # Get package directory
    pkg_dir = get_package_share_directory('jetbot_vlm_nav')
    
    # Webots launch (assumes webots_ros2 is installed)
    # You'll need to create a Webots world file for your Jetbot
    webots_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('webots_ros2_universal_robot'), 
                        'launch', 'robot_launch.py')
        ]),
        launch_arguments={
            'world': os.path.join(pkg_dir, 'worlds', 'jetbot_world.wbt')
        }.items()
    )
    
    # Camera node (Webots will publish to this topic)
    # The Webots robot should have a camera device configured
    
    # Include the main navigation launch
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_dir, 'launch', 'jetbot_navigation.launch.py')
        ])
    )
    
    return LaunchDescription([
        # webots_launch,  # Uncomment when Webots world is ready
        navigation_launch
    ])
