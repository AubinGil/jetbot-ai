from setuptools import setup
import os
from glob import glob

package_name = 'jetbot_conversation'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='gildas01',
    maintainer_email='you@example.com',
    description='Intelligent conversation pipeline for JetBot',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'conversation_node = jetbot_conversation.conversation_node:main',
            'audio_capture_node = jetbot_conversation.audio_capture_node:main',
            'tts_node = jetbot_conversation.tts_node:main',
        ],
    },
)
