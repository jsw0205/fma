from glob import glob
import os

from setuptools import find_packages, setup


package_name = 'parallel_parking'


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        ('share/' + package_name, ['package.xml']),
        (
            os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py'),
        ),
        (
            os.path.join('share', package_name, 'config'),
            glob('config/*.yaml'),
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='you',
    maintainer_email='you@example.com',
    description='ROS 2 package for parallel parking.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'rule_based_parallel_parking_node = '
            'parallel_parking.rule_based_parallel_parking_node:main',
        ],
    },
)
