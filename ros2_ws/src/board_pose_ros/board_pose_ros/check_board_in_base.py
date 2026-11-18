#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""外参验证工具：把标定板位姿变换到 base_link，检查“板在基座下是否恒定”。

同一块静止的标定板，被机械臂带着相机从不同姿态观察时，只要外参
（tcp_link -> camera_link）正确，它在 base_link 下的位姿就应该基本不变。

用法::

    ros2 run board_pose_ros verify_board --ros-args \
        -p pose_topic:=/aruco_single/pose -p base_frame:=base_link

前提：已经发布了 tcp_link -> camera_link 的静态 TF（数值取标定结果），
并且 base_link -> ... -> tcp_link 的 TF 链存在（用 rviz 版 launch 或
agx_arm_description/display.launch.py 启动机械臂）。

判据：position 波动 < 5 mm，姿态波动 < 1°。
"""
from __future__ import annotations

import math
import sys

import numpy as np
import rclpy
import rclpy.time
from geometry_msgs.msg import PoseStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener


def quat_to_rotmat(x, y, z, w) -> np.ndarray:
    """四元数 -> 3x3 旋转矩阵。"""
    n = x * x + y * y + z * z + w * w
    if n < 1e-12:
        return np.eye(3)
    s = 2.0 / n
    return np.array([
        [1 - s * (y * y + z * z), s * (x * y - z * w), s * (x * z + y * w)],
        [s * (x * y + z * w), 1 - s * (x * x + z * z), s * (y * z - x * w)],
        [s * (x * z - y * w), s * (y * z + x * w), 1 - s * (x * x + y * y)],
    ])


def rotmat_to_rpy(R: np.ndarray):
    """旋转矩阵 -> 固定轴 XYZ 的 roll/pitch/yaw（弧度）。"""
    sy = math.hypot(R[0, 0], R[1, 0])
    if sy > 1e-9:
        return (math.atan2(R[2, 1], R[2, 2]),
                math.atan2(-R[2, 0], sy),
                math.atan2(R[1, 0], R[0, 0]))
    return (math.atan2(-R[1, 2], R[1, 1]), math.atan2(-R[2, 0], sy), 0.0)


class BoardInBaseChecker(Node):

    def __init__(self):
        super().__init__("verify_board")

        self.pose_topic = self.declare_parameter(
            "pose_topic", "/aruco_single/pose").value
        self.base_frame = self.declare_parameter("base_frame", "base_link").value
        self.window = int(self.declare_parameter("window", 60).value)
        self.report_every = int(self.declare_parameter("report_every", 10).value)

        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.samples = []
        self.count = 0
        self.tf_warned = False

        self.create_subscription(PoseStamped, self.pose_topic, self.on_pose, 10)
        self.get_logger().info(
            f"订阅 {self.pose_topic}，目标坐标系 {self.base_frame}")
        self.get_logger().info(
            "把标定板固定不动，移动机械臂到不同姿态并停留 2s；"
            "同一块板在 base_frame 下的位姿应恒定（波动 < 5mm / 1°）。")

    # ------------------------------------------------------------------ #
    def on_pose(self, msg: PoseStamped):
        src = msg.header.frame_id
        try:
            tr = self.buffer.lookup_transform(
                self.base_frame, src, rclpy.time.Time())
        except Exception:  # noqa: BLE001
            if not self.tf_warned:
                self.get_logger().warn(
                    f"查不到 {self.base_frame} -> {src} 的 TF："
                    "确认已发布 tcp_link->camera_link 静态 TF，"
                    "且 base_link->tcp_link 链完整（用带 rviz 的 launch 启动机械臂）")
                self.tf_warned = True
            return

        t = tr.transform.translation
        q = tr.transform.rotation
        R_t = quat_to_rotmat(q.x, q.y, q.z, q.w)
        T_t = np.array([t.x, t.y, t.z])

        p = msg.pose.position
        o = msg.pose.orientation
        R = quat_to_rotmat(o.x, o.y, o.z, o.w)
        t_vec = np.array([p.x, p.y, p.z])

        pos = R_t @ t_vec + T_t
        rpy = np.array(rotmat_to_rpy(R_t @ R))

        self.samples.append((pos, rpy))
        if len(self.samples) > self.window:
            self.samples.pop(0)
        self.count += 1
        if self.count % max(1, self.report_every) != 0:
            return

        arr_p = np.array([s[0] for s in self.samples])
        arr_r = np.array([s[1] for s in self.samples])
        p_span = float(np.linalg.norm(arr_p.max(0) - arr_p.min(0)))
        r_span = float(np.rad2deg(np.abs(arr_r.max(0) - arr_r.min(0)).max()))
        self.get_logger().info(
            f"[{self.count}] 板在 {self.base_frame}: "
            f"p=({pos[0]:+.3f}, {pos[1]:+.3f}, {pos[2]:+.3f}) m, "
            f"rpy=({np.rad2deg(rpy[0]):+.1f}, {np.rad2deg(rpy[1]):+.1f}, "
            f"{np.rad2deg(rpy[2]):+.1f})° | "
            f"滑窗波动 p={p_span * 1000:.1f} mm, r={r_span:.2f}°")


def main(args=None) -> int:
    rclpy.init(args=args)
    node = BoardInBaseChecker()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
