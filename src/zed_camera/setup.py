from setuptools import find_packages, setup

package_name = "zed_camera"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="a",
    maintainer_email="jungsunwoo020205@gmail.com",
    description="YOLOPv2 lane-following node for the ZED2i camera",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "yolopv2_zed_node = zed_camera.yolopv2_zed_node:main",
            # Newer/current version (2026-07-27): LaneTracker-based lane
            # tracking (fixes a one-lane-visible max-steer bug in the
            # sliding-window approach used by yolopv2_zed_node), curve
            # auto-slowdown, lane_valid topic. Same ROS node name
            # ("yolopv2_zed_node") either way, so control_arbiter's
            # camera_steer_topic doesn't care which one is running.
            "yolopv2_zed_rpm_node = zed_camera.yolopv2_zed_rpm_node:main",
        ],
    },
)
