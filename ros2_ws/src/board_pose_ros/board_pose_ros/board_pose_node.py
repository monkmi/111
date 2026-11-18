#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""标定板位姿识别节点（ChArUco / 圆点阵列 / 棋盘格）。

发布内容（默认值与 agilex 手眼标定流程对齐）：
  * PoseStamped  /aruco_single/pose     标定板在相机坐标系下的位姿（p_cam = R * p_board + t）
  * TF           <camera_frame> -> aruco_marker_frame
  * Image        /board_pose/debug      调试图（检测点 + 坐标轴 + 状态文字）
  * String       /board_pose/status     JSON 状态：是否检出、点数、重投影误差、位姿

参数见 config/ 目录下的 yaml，或 `ros2 param describe /board_pose <name>`。
"""

from __future__ import annotations

import json
import math

import cv2
import numpy as np
import rclpy
import rclpy.time
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped, TransformStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from tf2_ros import Buffer, TransformBroadcaster, TransformListener

from board_pose_ros import detectors as D
from board_pose_ros.io_utils import load_intrinsics


def _quat_to_rotmat(x: float, y: float, z: float, w: float) -> np.ndarray:
    n = x * x + y * y + z * z + w * w
    if n < 1e-12:
        return np.eye(3)
    s = 2.0 / n
    return np.array([
        [1.0 - s * (y * y + z * z), s * (x * y - z * w), s * (x * z + y * w)],
        [s * (x * y + z * w), 1.0 - s * (x * x + z * z), s * (y * z - x * w)],
        [s * (x * z - y * w), s * (y * z + x * w), 1.0 - s * (x * x + y * y)],
    ], dtype=np.float64)


class BoardPoseNode(Node):

    def __init__(self) -> None:
        super().__init__("board_pose")
        self.bridge = CvBridge()

        # ---------------- 通用参数 ----------------
        self.board_type = str(self.declare_parameter("board_type", "charuco").value).strip().lower()
        self.image_topic = str(self.declare_parameter(
            "image_topic", "/camera/camera/color/image_raw").value)
        self.camera_info_topic = str(self.declare_parameter(
            "camera_info_topic", "/camera/camera/color/camera_info").value)
        self.intrinsics_file = str(self.declare_parameter("intrinsics_file", "").value).strip()
        self.process_rate_hz = float(self.declare_parameter("process_rate_hz", 10.0).value)
        self.max_reproj_err = float(self.declare_parameter("max_reproj_err", 3.0).value)
        self.min_points = int(self.declare_parameter("min_points", 8).value)
        self.pose_topic = str(self.declare_parameter(
            "pose_topic", "/aruco_single/pose").value)
        self.status_topic = str(self.declare_parameter(
            "status_topic", "/board_pose/status").value)
        self.debug_topic = str(self.declare_parameter(
            "debug_topic", "/board_pose/debug").value)
        self.publish_debug_image = bool(self.declare_parameter(
            "publish_debug_image", True).value)
        self.publish_tf = bool(self.declare_parameter("publish_tf", True).value)
        self.tf_child = str(self.declare_parameter("tf_child", "aruco_marker_frame").value)
        self.output_frame = str(self.declare_parameter("output_frame", "").value).strip()
        self.show_axis_length = float(self.declare_parameter("axis_length", 0.03).value)

        # ---------------- ChArUco 参数 ----------------
        self.dictionary = str(self.declare_parameter("dictionary", "DICT_4X4_50").value)
        self.squares_x = int(self.declare_parameter("squares_x", 5).value)
        self.squares_y = int(self.declare_parameter("squares_y", 7).value)
        self.square_length = float(self.declare_parameter("square_length", 0.03).value)
        self.marker_length = float(self.declare_parameter("marker_length", 0.022).value)

        # ---------------- 圆点阵列（八点板）参数 ----------------
        self.dot_rows = int(self.declare_parameter("dot_rows", 3).value)
        self.dot_cols = int(self.declare_parameter("dot_cols", 3).value)
        self.dot_spacing = float(self.declare_parameter("dot_spacing", 0.04).value)
        self.dot_missing = str(self.declare_parameter("dot_missing", "0,0").value)
        self.dot_dark_on_light = bool(self.declare_parameter("dot_dark_on_light", True).value)
        self.dot_min_area_frac = float(self.declare_parameter("dot_min_area_frac", 1.5e-4).value)
        self.dot_max_area_frac = float(self.declare_parameter("dot_max_area_frac", 2e-2).value)
        self.dot_min_circularity = float(self.declare_parameter("dot_min_circularity", 0.55).value)
        self.dot_min_fill = float(self.declare_parameter("dot_min_fill", 0.55).value)
        self.dot_blur_ksize = int(self.declare_parameter("dot_blur_ksize", 5).value)

        # ---------------- 棋盘格参数 ----------------
        self.checker_cols = int(self.declare_parameter("checker_cols", 5).value)
        self.checker_rows = int(self.declare_parameter("checker_rows", 3).value)
        self.checker_square_length = float(self.declare_parameter(
            "checker_square_length", 0.02).value)

        if self.board_type not in ("charuco", "dots", "checker"):
            raise ValueError("board_type 必须是 charuco / dots / checker，当前 %r" % self.board_type)

        # ---------------- 内参 ----------------
        self.K = None
        self.D = None
        self._intr_source = "camera_info"
        if self.intrinsics_file:
            K, Dv = load_intrinsics(self.intrinsics_file)
            self.K, self.D = K, Dv
            self._intr_source = "file:%s" % self.intrinsics_file
            self.get_logger().info(
                "已从文件加载内参 %s:\n fx=%.2f fy=%.2f cx=%.2f cy=%.2f D=%s"
                % (self.intrinsics_file, K[0, 0], K[1, 1], K[0, 2], K[1, 2],
                   np.round(self.D, 5).tolist()))

        # ---------------- 板对象 ----------------
        self.board = None
        self._auto_charuco = False
        self._auto_info = {}
        if self.board_type == "charuco":
            if self.marker_length <= 0.0:
                self.marker_length = 0.75 * self.square_length
                self.get_logger().warn(
                    "未给 marker_length，按常见比例 0.75×格子边长取 %.4fm" % self.marker_length)
            if str(self.dictionary).strip().lower() == "auto":
                self._auto_charuco = True
                self.get_logger().info(
                    "ChArUco 板: %dx%d squares, square=%.4fm, marker=%.4fm, "
                    "字典=自动（逐个尝试，并同时试横/竖两种朝向）"
                    % (self.squares_x, self.squares_y, self.square_length,
                       self.marker_length))
            else:
                self.board = D.build_charuco_board(
                    self.dictionary, self.squares_x, self.squares_y,
                    self.square_length, self.marker_length)
                self.get_logger().info(
                    "ChArUco 板: %dx%d squares, square=%.4fm, marker=%.4fm, dict=%s"
                    % (self.squares_x, self.squares_y, self.square_length,
                       self.marker_length, self.dictionary))
        elif self.board_type == "dots":
            cells, obj = D.dots_layout(self.dot_rows, self.dot_cols, self.dot_spacing,
                                       self.dot_missing)
            self.get_logger().info(
                "圆点板: %dx%d 网格, 间距=%.4fm, 缺格=%r, 有效点=%d, 板尺寸约 %.3fx%.3fm"
                % (self.dot_rows, self.dot_cols, self.dot_spacing,
                   D.parse_missing_cells(self.dot_missing), len(cells),
                   (self.dot_cols + 1) * self.dot_spacing,
                   (self.dot_rows + 1) * self.dot_spacing))
        else:
            self.get_logger().info(
                "棋盘格: 内角点 %dx%d, 方格边长=%.4fm（注意：棋盘格图案有 180° 自相似性）"
                % (self.checker_cols, self.checker_rows, self.checker_square_length))

        # ---------------- TF / ROS 接口 ----------------
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.pose_pub = self.create_publisher(PoseStamped, self.pose_topic, 10)
        self.status_pub = self.create_publisher(String, self.status_topic, 10)
        self.debug_pub = self.create_publisher(Image, self.debug_topic, 10) \
            if self.publish_debug_image else None

        self.create_subscription(CameraInfo, self.camera_info_topic,
                                 self.on_camera_info, qos_profile_sensor_data)
        self.create_subscription(Image, self.image_topic, self.on_image,
                                 qos_profile_sensor_data)

        self._hint = None          # 上一帧位姿，用于棋盘格 180° 消歧
        self._last_proc = None
        self._n_img = 0
        self._n_ok = 0
        self.get_logger().info(
            "board_pose 已启动: type=%s, image=%s, camera_info=%s, pose=%s, tf=%s->%s"
            % (self.board_type, self.image_topic, self.camera_info_topic,
               self.pose_topic, self.output_frame or "<image frame>", self.tf_child))

    # ------------------------------------------------------------------ #
    # 回调
    # ------------------------------------------------------------------ #
    def on_camera_info(self, msg: CameraInfo) -> None:
        if self.K is not None:
            return  # 用文件内参时忽略相机信息中的内参
        K = np.asarray(msg.k, dtype=np.float64).reshape(3, 3)
        Dv = np.asarray(msg.d, dtype=np.float64).reshape(-1)
        if K[0, 0] <= 0.0 or K[1, 1] <= 0.0:
            self.get_logger().warn("camera_info 内参无效（fx/fy<=0），等待有效值",
                                   throttle_duration_sec=5.0)
            return
        if Dv.size < 4:
            Dv = np.zeros(5, dtype=np.float64)
        self.K, self.D = K, Dv
        self.get_logger().info(
            "使用 camera_info 内参: fx=%.2f fy=%.2f cx=%.2f cy=%.2f distortion=%s "
            "(%dx%d)"
            % (K[0, 0], K[1, 1], K[0, 2], K[1, 2], np.round(Dv, 6).tolist(),
               msg.width, msg.height))

    def on_image(self, msg: Image) -> None:
        self._n_img += 1
        if self.K is None:
            self.get_logger().warn(
                "尚未获得相机内参：等待 %s，或用 intrinsics_file 参数指定内参文件"
                % self.camera_info_topic, throttle_duration_sec=3.0)
            return

        now = self.get_clock().now()
        if self.process_rate_hz > 0.0 and self._last_proc is not None:
            period = 1.0 / self.process_rate_hz
            if (now - self._last_proc).nanoseconds * 1e-9 < period:
                return
        self._last_proc = now

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error("图像转换失败: %s" % exc, throttle_duration_sec=5.0)
            return
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        det = self._detect(gray)
        if not det.ok:
            self._publish_status(msg, det, None)
            self.get_logger().info("未检出标定板: %s" % det.msg,
                                   throttle_duration_sec=2.0)
            if self.debug_pub is not None:
                self._publish_debug(frame, det, msg)
            return

        if det.reproj_err > self.max_reproj_err:
            self._publish_status(msg, det, None)
            self.get_logger().warn(
                "检出但重投影误差过大 (%.2f px > %.2f px)，已丢弃；请检查板参数/内参"
                % (det.reproj_err, self.max_reproj_err), throttle_duration_sec=2.0)
            if self.debug_pub is not None:
                self._publish_debug(frame, det, msg)
            return

        self._n_ok += 1
        self._hint = (det.rvec.copy(), det.tvec.copy())
        rvec, tvec, frame_used = self._to_output_frame(
            det.rvec, det.tvec, msg.header.frame_id)

        pose_msg = PoseStamped()
        pose_msg.header.stamp = msg.header.stamp
        pose_msg.header.frame_id = frame_used
        t, q = D.pose_quat_from_rvec(rvec, tvec)
        pose_msg.pose.position.x, pose_msg.pose.position.y, pose_msg.pose.position.z = (
            float(t[0]), float(t[1]), float(t[2]))
        pose_msg.pose.orientation.x = float(q[0])
        pose_msg.pose.orientation.y = float(q[1])
        pose_msg.pose.orientation.z = float(q[2])
        pose_msg.pose.orientation.w = float(q[3])
        self.pose_pub.publish(pose_msg)

        if self.publish_tf:
            tf_msg = TransformStamped()
            tf_msg.header.stamp = msg.header.stamp
            tf_msg.header.frame_id = frame_used
            tf_msg.child_frame_id = self.tf_child
            tf_msg.transform.translation.x = float(t[0])
            tf_msg.transform.translation.y = float(t[1])
            tf_msg.transform.translation.z = float(t[2])
            tf_msg.transform.rotation = pose_msg.pose.orientation
            self.tf_broadcaster.sendTransform(tf_msg)

        self._publish_status(msg, det, pose_msg)

        if self.debug_pub is not None:
            self._publish_debug(frame, det, msg, rvec=rvec, tvec=tvec,
                                frame_used=frame_used)

        self.get_logger().info(
            "检出 %s: %d 点, 重投影 %.2f px, t=(%.3f, %.3f, %.3f) m"
            % (self.board_type, det.n_used, det.reproj_err, t[0], t[1], t[2]),
            throttle_duration_sec=1.0)

    # ------------------------------------------------------------------ #
    # 内部工具
    # ------------------------------------------------------------------ #
    def _detect(self, gray: np.ndarray) -> D.Detection:
        try:
            if self.board_type == "charuco":
                if self._auto_charuco:
                    det, info = D.detect_charuco_auto(
                        gray, self.squares_x, self.squares_y,
                        self.square_length, self.marker_length, self.K, self.D,
                        min_corners=max(6, self.min_points))
                    changed = info.get("dictionary") != self._auto_info.get("dictionary") \
                        or info.get("squares_x") != self._auto_info.get("squares_x")
                    self._auto_info = info
                    if det.ok and changed:
                        self.get_logger().info(
                            "自动识别到字典 %s，板朝向 %dx%d（检出 %d 个 marker）"
                            % (info.get("dictionary"), info.get("squares_x"),
                               info.get("squares_y"), info.get("n_markers", 0)))
                    return det
                return D.detect_charuco(gray, self.board, self.K, self.D,
                                        self.dictionary,
                                        min_corners=max(6, self.min_points))
            if self.board_type == "dots":
                return D.detect_dots(
                    gray, self.dot_rows, self.dot_cols, self.dot_spacing,
                    self.K, self.D, missing=self.dot_missing,
                    min_points=max(4, self.min_points),
                    dark_on_light=self.dot_dark_on_light,
                    min_area_frac=self.dot_min_area_frac,
                    max_area_frac=self.dot_max_area_frac,
                    min_circularity=self.dot_min_circularity,
                    min_fill=self.dot_min_fill,
                    blur_ksize=self.dot_blur_ksize)
            return D.detect_checker(gray, self.checker_cols, self.checker_rows,
                                    self.checker_square_length, self.K, self.D,
                                    hint=self._hint)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error("检测异常: %s" % exc, throttle_duration_sec=5.0)
            return D.Detection(msg="检测异常: %s" % exc)

    def _to_output_frame(self, rvec, tvec, src_frame: str):
        dst = self.output_frame
        if not dst or dst == src_frame:
            return rvec, tvec, src_frame
        try:
            tr = self.tf_buffer.lookup_transform(dst, src_frame, rclpy.time.Time())
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(
                "无法把位姿变换到 %s（%s），改为在 %s 下发布；可先运行 ros2 run tf2_ros "
                "static_transform_publisher" % (dst, exc, src_frame),
                throttle_duration_sec=5.0)
            return rvec, tvec, src_frame
        q = tr.transform.rotation
        R_t = _quat_to_rotmat(q.x, q.y, q.z, q.w)
        T_t = np.array([tr.transform.translation.x,
                        tr.transform.translation.y,
                        tr.transform.translation.z], dtype=np.float64)
        R = cv2.Rodrigues(np.asarray(rvec, dtype=np.float64).reshape(3))[0]
        R_out = R_t @ R
        t_out = R_t @ np.asarray(tvec, dtype=np.float64).reshape(3) + T_t
        rvec_out = cv2.Rodrigues(R_out)[0].reshape(3)
        return rvec_out, t_out, dst

    def _publish_status(self, msg: Image, det: D.Detection, pose_msg) -> None:
        status = {
            "detected": bool(det.ok),
            "board_type": self.board_type,
            "n_points": int(det.n_used),
            "reproj_err_px": float(det.reproj_err) if math.isfinite(det.reproj_err) else None,
            "msg": det.msg,
            "frame_id": msg.header.frame_id,
            "intrinsics_source": self._intr_source,
            "frames": {"received": self._n_img, "accepted": self._n_ok},
        }
        if self._auto_charuco and self._auto_info.get("dictionary"):
            status["auto_board"] = {
                "dictionary": self._auto_info.get("dictionary"),
                "squares_x": self._auto_info.get("squares_x"),
                "squares_y": self._auto_info.get("squares_y"),
                "n_markers": self._auto_info.get("n_markers"),
            }
        if pose_msg is not None:
            status["position"] = [pose_msg.pose.position.x,
                                  pose_msg.pose.position.y,
                                  pose_msg.pose.position.z]
            status["orientation_xyzw"] = [pose_msg.pose.orientation.x,
                                          pose_msg.pose.orientation.y,
                                          pose_msg.pose.orientation.z,
                                          pose_msg.pose.orientation.w]
            status["pose_frame"] = pose_msg.header.frame_id
            status["tf_child"] = self.tf_child
        out = String()
        out.data = json.dumps(status, ensure_ascii=False)
        self.status_pub.publish(out)

    def _publish_debug(self, frame, det: D.Detection, msg: Image,
                       rvec=None, tvec=None, frame_used="") -> None:
        label = self.board_type
        if det.ok:
            label += " | %d pts | %.2fpx" % (det.n_used, det.reproj_err)
        else:
            label += " | %s" % det.msg
        img = D.draw_debug(frame, det, label=label, K=self.K, D=self.D,
                           axis_length=self.show_axis_length,
                           rvec=rvec, tvec=tvec)
        if frame_used and frame_used != msg.header.frame_id:
            cv2.putText(img, "frame: %s" % frame_used, (10, img.shape[0] - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1, cv2.LINE_AA)
        out = self.bridge.cv2_to_imgmsg(img, encoding="bgr8")
        out.header = msg.header
        self.debug_pub.publish(out)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = BoardPoseNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
