from setuptools import find_packages, setup

package_name = 'jetbot_hardware'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/jetbot_hardware.launch.py',
            'launch/jetbot_slam.launch.py',
            'launch/jetbot_complete.launch.py'
        ]),
        ('share/' + package_name + '/config', [
            'config/slam_params.yaml'
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='gildas01',
    maintainer_email='gildas01@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'motor_controller = jetbot_hardware.motor_controller:main',
            'motor_controller_encoders = jetbot_hardware.motor_controller_with_encoders:main',
            'camera_publisher = jetbot_hardware.camera_publisher:main',
            'obstacle_avoidance = jetbot_hardware.obstacle_avoidance:main',
            'pose_gesture_node = jetbot_hardware.pose_gesture_node:main',
            'odometry_publisher = jetbot_hardware.odometry_publisher:main',
            'map_auto_saver = jetbot_hardware.map_auto_saver:main',
        ],
    },
)
