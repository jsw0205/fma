from setuptools import find_packages, setup

package_name = 'parking_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='you',
    maintainer_email='you@example.com',
    description=(
        'CAN wheel-odometry bridge (encoder + IMU fusion) for the parking '
        'packages - reuses waypoint_follower.can_driver for the CAN frame '
        'layout instead of writing CAN itself.'
    ),
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'wheel_odom_pcan_node = parking_bridge.wheel_odom_pcan_node:main',
        ],
    },
)
