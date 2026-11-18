#!/usr/bin/env python3
from glob import glob

from setuptools import setup

package_name = "board_pose_ros"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="nero-official-calibrate",
    maintainer_email="user@example.com",
    description="标定板位姿识别节点（ChArUco / 圆点阵列 / 棋盘格），用于 ROS2 手眼标定",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "board_pose_node = board_pose_ros.board_pose_node:main",
            "board_gen = board_pose_ros.board_gen:main",
            "charuco_calibrate = board_pose_ros.charuco_intrinsics:main",
            "board_self_check = board_pose_ros.self_check:main",
            "verify_board = board_pose_ros.check_board_in_base:main",
            "verify_handeye = board_pose_ros.verify_handeye:main",
        ],
    },
)
