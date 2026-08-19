import os
from glob import glob

from setuptools import find_packages, setup

package_name = "waypoint_follower"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "waypoints"), glob("waypoints/*.csv")),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="a",
    maintainer_email="jungsunwoo020205@gmail.com",
    description="GPS waypoint following with a Stanley controller and RViz visualization",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "waypoint_follower_node = waypoint_follower.waypoint_follower_node:main",
            "arbiter_node = waypoint_follower.arbiter_node:main",
            "gps_node = waypoint_follower.gps_node:main",
            "fake_gps_node = waypoint_follower.fake_gps_node:main",
            "waypoint_recorder_node = waypoint_follower.waypoint_recorder_node:main",
            "lidar_front_distance_node = waypoint_follower.lidar_front_distance_node:main",
            "mpl_viz_node = waypoint_follower.mpl_viz_node:main",
            "imu_drift_test = waypoint_follower.imu_drift_test:main",
            "mag_calibrate = waypoint_follower.mag_calibrate:main",
            "status_monitor_node = waypoint_follower.status_monitor_node:main",
            "gps_quality_monitor = waypoint_follower.gps_quality_monitor:main",
        ],
    },
)
