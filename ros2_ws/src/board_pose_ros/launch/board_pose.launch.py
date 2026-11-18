#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""启动标定板位姿识别节点。

    ros2 launch board_pose_ros board_pose.launch.py board_type:=charuco \\
        image_topic:=/camera/camera/color/image_raw \\
        camera_info_topic:=/camera/camera/color/camera_info

也可以用参数文件启动（参数文件里的板参数最完整）：
    ros2 launch board_pose_ros board_pose.launch.py \\
        params_file:=$HOME/ros2_ws/src/board_pose_ros/config/dots.yaml
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("board_pose_ros")

    board_type = LaunchConfiguration("board_type")
    image_topic = LaunchConfiguration("image_topic")
    camera_info_topic = LaunchConfiguration("camera_info_topic")
    intrinsics_file = LaunchConfiguration("intrinsics_file")
    pose_topic = LaunchConfiguration("pose_topic")
    params_file = LaunchConfiguration("params_file")
    output_frame = LaunchConfiguration("output_frame")

    args = [
        DeclareLaunchArgument("board_type", default_value="charuco",
                              description="标定板类型: charuco | dots | checker"),
        DeclareLaunchArgument("image_topic",
                              default_value="/camera/camera/color/image_raw",
                              description="彩色图像话题（sensor_msgs/Image）"),
        DeclareLaunchArgument("camera_info_topic",
                              default_value="/camera/camera/color/camera_info",
                              description="相机信息话题（sensor_msgs/CameraInfo）"),
        DeclareLaunchArgument("intrinsics_file", default_value="",
                              description="自定义内参 YAML；非空时覆盖 camera_info 内参"),
        DeclareLaunchArgument("pose_topic", default_value="/aruco_single/pose",
                              description="输出位姿话题（默认与 handeye_calibration_ros 对齐）"),
        DeclareLaunchArgument("output_frame", default_value="",
                              description="把位姿变换到该坐标系（如 base_link）；空=相机坐标系"),
        DeclareLaunchArgument("params_file",
                              default_value=os.path.join(pkg_share, "config", "charuco.yaml"),
                              description="参数文件（会自动选 config 下的 yaml）"),
    ]

    node = Node(
        package="board_pose_ros",
        executable="board_pose_node",
        name="board_pose",
        output="screen",
        emulate_tty=True,
        parameters=[
            params_file,
            {
                "board_type": board_type,
                "image_topic": image_topic,
                "camera_info_topic": camera_info_topic,
                "intrinsics_file": intrinsics_file,
                "pose_topic": pose_topic,
                "output_frame": output_frame,
            },
        ],
    )

    def checked_start(context):
        path = os.path.expanduser(params_file.perform(context))
        if not os.path.isfile(path):
            raise RuntimeError(f"标定板配置文件不存在：{path}；请使用绝对路径。未启动识别节点。")
        return [node]

    return LaunchDescription(args + [OpaqueFunction(function=checked_start)])
