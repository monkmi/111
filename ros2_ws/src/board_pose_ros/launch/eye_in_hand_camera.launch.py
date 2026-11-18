#!/usr/bin/env python3
"""本机手内相机 + 实物板识别，按序列号固定相机，不依赖当前工作目录。"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory("board_pose_ros")
    camera = Node(
        package="realsense2_camera",
        executable="realsense2_camera_node",
        namespace="camera",
        name="camera",
        output="screen",
        parameters=[{
            "serial_no": ParameterValue("262522072403", value_type=str),
            "enable_color": True,
            "enable_depth": False,
            "rgb_camera.color_profile": "1280x720x30",
        }],
    )
    board = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(share, "launch", "board_pose.launch.py")),
        launch_arguments={
            "params_file": os.path.join(share, "config", "charuco_7x9_24mm.yaml"),
            "image_topic": "/camera/camera/color/image_raw",
            "camera_info_topic": "/camera/camera/color/camera_info",
        }.items(),
    )
    return LaunchDescription([
        LogInfo(msg="手内相机固定为 D435 262522072403；外部相机 263022078163 不用于手内采样。"
                    "请先停止占用相同相机/话题的旧相机和识别节点。"),
        camera, board,
    ])
