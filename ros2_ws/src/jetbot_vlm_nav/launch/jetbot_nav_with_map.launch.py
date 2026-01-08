#!/usr/bin/env python3
"""
Complete Jetbot Navigation with Map
Launches:
- Map Server (loads your saved map)
- AMCL (localization on the map)
- Nav2 Navigation Stack
- Hardware interfaces (if needed)
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():
    # Directories
    pkg_dir = get_package_share_directory('jetbot_vlm_nav')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    # Default map location
    default_map_path = os.path.expanduser('~/maps/jetbot_map.yaml')

    # Launch arguments
    declare_map_yaml = DeclareLaunchArgument(
        'map',
        default_value=default_map_path,
        description='Full path to map yaml file'
    )

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )

    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(pkg_dir, 'config', 'nav2_params.yaml'),
        description='Full path to Nav2 parameters file'
    )

    declare_autostart = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically startup Nav2 stack'
    )

    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz',
        default_value='false',
        description='Launch RViz for visualization'
    )

    # Get launch configurations
    map_yaml = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')
    use_rviz = LaunchConfiguration('use_rviz')

    # Map Server Node
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            {'yaml_filename': map_yaml},
            {'use_sim_time': use_sim_time}
        ]
    )

    # AMCL (Localization) Node
    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[
            params_file,
            {'use_sim_time': use_sim_time}
        ]
    )

    # Lifecycle Manager for Map Server and AMCL
    lifecycle_manager_localization = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'autostart': autostart},
            {'node_names': ['map_server', 'amcl']}
        ]
    )

    # Nav2 Bringup (Controller, Planner, etc.)
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'autostart': autostart,
        }.items()
    )

    # RViz (optional)
    rviz_config = os.path.join(pkg_dir, 'rviz', 'nav2_config.rviz')
    if not os.path.exists(rviz_config):
        rviz_config = os.path.join(nav2_bringup_dir, 'rviz', 'nav2_default_view.rviz')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        condition=IfCondition(use_rviz),
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    return LaunchDescription([
        # Declare arguments
        declare_map_yaml,
        declare_use_sim_time,
        declare_params_file,
        declare_autostart,
        declare_use_rviz,

        # Launch nodes
        map_server_node,
        amcl_node,
        lifecycle_manager_localization,
        nav2_bringup,
        rviz_node,
    ])
