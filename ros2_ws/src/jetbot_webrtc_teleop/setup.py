from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'jetbot_webrtc_teleop'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/web', glob('web/*')),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='gildas01',
    maintainer_email='gildas01@todo.todo',
    description='WebRTC-based teleoperation for Jetbot',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'webrtc_server = jetbot_webrtc_teleop.webrtc_server:main',
        ],
    },
)
