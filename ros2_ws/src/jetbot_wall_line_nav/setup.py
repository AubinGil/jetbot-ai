from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'jetbot_wall_line_nav'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
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
    maintainer_email='gildas01@todo.todo',
    description='Lightweight wall detection and line following for indoor navigation',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'wall_line_navigator = jetbot_wall_line_nav.wall_line_navigator:main',
        ],
    },
)
