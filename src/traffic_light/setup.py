from setuptools import find_packages, setup

package_name = "traffic_light"

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
    description="OAK-D + YOLO traffic light detection node",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            # Matches the exec name from the original 노드사용법 doc.
            "test_sunny_node = traffic_light.traffic_light_node:main",
        ],
    },
)
