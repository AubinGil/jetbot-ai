from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'enable_camera',
            default_value='false',
            description='Enable USB camera publisher'
        ),
        
        DeclareLaunchArgument(
            'enable_lidar',
            default_value='false',
            description='Enable RPLidar'
        ),
        
        DeclareLaunchArgument(
            'enable_pose_gesture',
            default_value='false',
            description='Enable YOLOv11 pose-based gesture detection'
        ),
        
        DeclareLaunchArgument(
            'camera_device',
            default_value='/dev/video0',
            description='Camera device path or index'
        ),
        
        DeclareLaunchArgument(
            'pose_model_path',
            default_value=str('/home/gildas01/yolo11n-pose.onnx'),
            description='Path to YOLOv11 pose model/engine file'
        ),
        
        # Motor controller - WORKING
        Node(
            package='jetbot_hardware',
            executable='motor_controller',
            name='motor_controller',
            output='screen',
            parameters=[{
                'port': '/dev/ttyACM0',
                'baud_rate': 115200
            }]
        ),

        # Odometry publisher - integrates cmd_vel for Nav2
        Node(
            package='jetbot_hardware',
            executable='odometry_publisher',
            name='odometry_publisher',
            output='screen'
        ),

        # RPLidar
        Node(
            package='rplidar_ros',
            executable='rplidar_node',
            name='rplidar_node',
            output='screen',
            parameters=[{
                'serial_port': '/dev/ttyUSB0',
                'serial_baudrate': 115200,
                'frame_id': 'laser_frame',
                'angle_compensate': True,
                'scan_mode': 'Standard'
            }],
            condition=IfCondition(LaunchConfiguration('enable_lidar'))
        ),
        
        # Camera publisher
        Node(
            package='jetbot_hardware',
            executable='camera_publisher',
            name='jetbot_camera_publisher',
            output='screen',
            parameters=[{
                'device': LaunchConfiguration('camera_device'),
                'width': 960,
                'height': 544,
                'fps': 15,
            }],
            condition=IfCondition(LaunchConfiguration('enable_camera'))
        ),
        
        # YOLOv11 pose gesture detector
        Node(
            package='jetbot_hardware',
            executable='pose_gesture_node',
            name='pose_gesture_node',
            output='screen',
            parameters=[{
                'camera_topic': '/camera/image_raw',
                'model_path': LaunchConfiguration('pose_model_path'),
                'device': 'cuda:0',
                'detection_rate_hz': 8.0,
                'wave_cooldown_sec': 8.0,
                'gesture_topic': 'gesture/events',
            }],
            condition=IfCondition(LaunchConfiguration('enable_pose_gesture'))
        ),
    ])
