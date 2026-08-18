import os
from glob import glob

from setuptools import find_packages, setup

package_name = "can_odom_ros"

# 저장소 루트의 config/ 를 share 로 함께 설치 (실장 시 여기만 수정)
_repo_root = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
_cfg = os.path.join(_repo_root, "config")

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages",
         ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"),
         glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"),
         glob(os.path.join(_cfg, "*.yaml"))),
        (os.path.join("share", package_name, "config", "profiles"),
         glob(os.path.join(_cfg, "profiles", "*.yaml"))),
    ],
    install_requires=["setuptools", "python-can", "PyYAML"],
    zip_safe=True,
    maintainer="Colin LEE",
    maintainer_email="velonetic@velonetic.co.kr",
    description="2ELD2-CAN7020B CANopen ROS 2 driver with hardware-less simulator",
    license="MIT",
    entry_points={
        "console_scripts": [
            "driver_node = can_odom_ros.driver_node:main",
            "scenario_node = can_odom_ros.scenario_node:main",
        ],
    },
)
