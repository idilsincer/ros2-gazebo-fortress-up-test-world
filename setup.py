import os
from glob import glob
from setuptools import setup

package_name = 'up_week2'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.sdf')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'heartbeat_publisher = up_week2.heartbeat_publisher:main',
            'heartbeat_subscriber = up_week2.heartbeat_subscriber:main',
            'camera_monitor = up_week2.camera_monitor:main',
            'target_mover = up_week2.target_mover:main',
            'red_target_detector = up_week2.red_target_detector:main',
        ],
    },
)
