from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    package_share = Path(
        get_package_share_directory('obstacle_avoidance')
    )
    config_file = package_share / 'config' / 'obstacle_avoid.yaml'

    return LaunchDescription([
        Node(
            package='obstacle_avoidance',
            executable='obstacle_avoid_node',
            name='obstacle_avoid_node',
            output='log',
            parameters=[str(config_file)]
        )
    ])
