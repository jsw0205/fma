from glob import glob
import os

from setuptools import setup

package_name = 't_parking'

setup(
    name=package_name,
    version='2.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='you',
    maintainer_email='you@example.com',
    description='Rule-based left-side T reverse parking (ROS2, real vehicle)',
    license='MIT',
    entry_points={
        'console_scripts': [
            'rule_based_t_parking_node = t_parking.rule_based_t_parking_node:main',
            'scan_parking_filter = t_parking.scan_parking_filter_node:main',
            'wheel_odom_front_axle = t_parking.wheel_odom_front_axle_node:main',
        ],
    },
)
