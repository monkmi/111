#!/usr/bin/env python3
"""交互式多姿态手内/手外验证，复用 board_pose_node 的图像时间戳。"""
from __future__ import annotations

import argparse
from collections import Counter, deque
from datetime import datetime
import json
from pathlib import Path
import sys
import time
from zoneinfo import ZoneInfo

import cv2

from board_pose_ros.handeye_accuracy import (
    MODES, evaluate, load_calibration, matrix_pose, pose_errors, pose_matrix,
)


def default_result_dir():
    for parent in Path(__file__).resolve().parents:
        if parent.name == "ros2_ws":
            return parent / "result"
    return Path.cwd() / "result"


def write_json(path, data):
    """原子写入，采样中断时仍保留上一次完整数据。"""
    temporary = path.with_suffix(".tmp")
    temporary.write_text(json.dumps(data, indent=2, ensure_ascii=False,
                                    allow_nan=False), encoding="utf-8")
    temporary.replace(path)


def input_diagnostics(args, counts, publishers, status=None, reason="", accepted=0,
                      tcp_error="", board_age=None):
    """不依赖 ROS 的诊断格式化，区分未发布、未检出和时间戳问题。"""
    lines = [f"本组有效帧：{accepted}/{args.frames_per_pose}"]
    for key, topic in (("image", args.image_topic), ("board", args.pose_topic),
                       ("tcp", args.tcp_topic)):
        lines.append(f"  {topic}: 收到 {counts[key]} 条，发布者 {publishers[key]} 个")
        if counts[key] == 0 and publishers[key] == 0:
            lines.append("    该话题没有发布者；核对话题名、节点启动状态和 ROS_DOMAIN_ID。")
    if status is not None:
        lines.append(f"  板识别状态：{status.get('msg') or '无错误信息'}；"
                     f"detected={status.get('detected')}，"
                     f"点数={status.get('n_points')}，"
                     f"重投影={status.get('reproj_err_px')} px")
        if status.get("detected") is False:
            lines.append("    板未被识别，识别节点不会发布板位姿；检查实物字典、格子数、遮挡及清晰度。")
        elif not status.get("pose_frame"):
            lines.append("    检测到了板但位姿未通过质量门限，检查重投影误差和识别节点日志。")
    if counts["image"] and counts["tcp"] and not counts["board"]:
        lines.append("  图像和 TCP 已到达，缺少可用的标定板位姿。")
    if board_age is not None:
        lines.append(f"  最新板位姿距当前 ROS 时间 {board_age:+.3f} 秒（正数为过去）。")
    if tcp_error:
        lines.append(f"  TCP 无效数据：{tcp_error}")
    if reason and reason != "等待图像、标定板位姿和 TCP 反馈":
        lines.append(f"  最近未通过条件：{reason}")
    return "\n".join(lines)


def save_report(directory, data, X, min_poses):
    report = evaluate(data["mode"], X, data["samples"], min_poses)
    report.update(calibration=data["calibration"],
                  calibration_pose=data["calibration_pose"],
                  camera_frame=data["settings"]["camera_frame"],
                  samples_file="samples.json")
    write_json(directory / "report.json", report)
    import csv
    with (directory / "errors.csv").open("w", newline="", encoding="utf-8-sig") as f:
        fields = ["id", "frame_count", "translation_error_mm", "rotation_error_deg",
                  "within_pose_translation_rms_mm", "within_pose_rotation_rms_deg"]
        writer = csv.DictWriter(f, fieldnames=fields, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(report["per_pose"])
    t, r = report["translation_mm"], report["rotation_deg"]
    print(f"\n{report['pose_count']} 个姿态 / {report['frame_count']} 帧")
    print(f"位置误差：平均 {t['mean']:.3f} mm，RMS {t['rms']:.3f} mm，最大 {t['max']:.3f} mm")
    print(f"旋转误差：平均 {r['mean']:.3f}°，RMS {r['rms']:.3f}°，最大 {r['max']:.3f}°")
    print(report["note"])
    for warning in report["warnings"]:
        print(f"提示：{warning}")
    print(f"报告：{directory / 'report.json'}")
    return report


def make_parser():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--mode", required=True, choices=MODES,
                   help="手内 eye_in_hand；手外 eye_to_hand（旧 JSON 不含模式，必须指定）")
    p.add_argument("--calibration", type=Path,
                   help="官方标定 JSON，省略时读取 result 下最新 *_calibration.json")
    p.add_argument("--output-dir", type=Path, default=default_result_dir(),
                   help="保存根目录，默认 ros2_ws/result")
    p.add_argument("--replay", type=Path, help="离线重算已采集的 samples.json，无需 ROS")
    p.add_argument("--tcp-topic", default="/feedback/tcp_pose")
    p.add_argument("--pose-topic", default="/aruco_single/pose")
    p.add_argument("--status-topic", default="/board_pose/status", help="标定板识别诊断状态")
    p.add_argument("--image-topic", default="/camera/camera/color/image_raw")
    p.add_argument("--camera-info-topic", default="/camera/camera/color/camera_info")
    p.add_argument("--camera-frame", default="camera_color_optical_frame",
                   help="必须与标定时板位姿的 frame_id 相同；不自动转换坐标系")
    p.add_argument("--frames-per-pose", type=int, default=5)
    p.add_argument("--min-poses", type=int, default=5, help="不足此姿态数时提示，建议采 10~20 个")
    p.add_argument("--capture-timeout", type=float, default=15.0)
    p.add_argument("--sync-tolerance", type=float, default=0.05, help="图像/TCP 最大时间差，秒")
    p.add_argument("--max-age", type=float, default=1.0, help="消息最大延迟，秒")
    p.add_argument("--settle-time", type=float, default=0.6, help="停稳检查窗口，秒")
    p.add_argument("--stable-mm", type=float, default=1.0)
    p.add_argument("--stable-deg", type=float, default=0.3)
    return p


def collect(args, directory, data, X):
    # ROS 延迟导入，--help / --replay 可以在未 source ROS 的环境运行。
    import rclpy
    from cv_bridge import CvBridge
    from geometry_msgs.msg import PoseStamped
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import CameraInfo, Image
    from std_msgs.msg import String

    def stamp_ns(msg):
        return msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec

    def ros_pose(msg):
        p, q = msg.pose.position, msg.pose.orientation
        return {"position": [p.x, p.y, p.z], "orientation": [q.x, q.y, q.z, q.w]}

    class Collector(Node):
        def __init__(self):
            super().__init__("verify_handeye")
            self.arm = deque(maxlen=2000)
            self.poses = deque(maxlen=60)
            self.images = deque(maxlen=30)
            self.bridge = CvBridge()
            self.info = None
            self.counts = Counter()
            self.status = None
            self.tcp_error = ""
            self.create_subscription(PoseStamped, args.tcp_topic, self.on_arm,
                                     qos_profile_sensor_data)
            self.create_subscription(PoseStamped, args.pose_topic, self.on_board,
                                     qos_profile_sensor_data)
            self.create_subscription(Image, args.image_topic, self.on_image,
                                     qos_profile_sensor_data)
            self.create_subscription(CameraInfo, args.camera_info_topic, self.on_info,
                                     qos_profile_sensor_data)
            self.create_subscription(String, args.status_topic, self.on_status,
                                     qos_profile_sensor_data)

        def on_board(self, msg):
            self.counts["board"] += 1
            self.poses.append(msg)

        def on_image(self, msg):
            self.counts["image"] += 1
            self.images.append(msg)

        def on_status(self, msg):
            try:
                status = json.loads(msg.data)
                if isinstance(status, dict):
                    self.status = status
            except (ValueError, TypeError):
                pass

        def on_arm(self, msg):
            self.counts["tcp"] += 1
            try:
                T = pose_matrix(**ros_pose(msg))
            except ValueError as exc:
                self.tcp_error = str(exc)
                return
            self.arm.append((stamp_ns(msg), msg, T))

        def diagnostics(self, reason, accepted):
            publishers = {key: self.count_publishers(topic) for key, topic in (
                ("image", args.image_topic), ("board", args.pose_topic), ("tcp", args.tcp_topic))}
            age = ((self.get_clock().now().nanoseconds - stamp_ns(self.poses[-1])) * 1e-9
                   if self.poses else None)
            return input_diagnostics(args, self.counts, publishers, self.status,
                                     reason, accepted, self.tcp_error, age)

        def on_info(self, msg):
            self.info = {"frame_id": msg.header.frame_id, "width": msg.width,
                         "height": msg.height, "k": list(msg.k), "d": list(msg.d),
                         "distortion_model": msg.distortion_model}

        def capture(self, sample_id):
            self.arm.clear()
            self.poses.clear()
            self.images.clear()
            self.counts.clear()
            self.status = None
            self.tcp_error = ""
            start_ns = self.get_clock().now().nanoseconds
            deadline = time.monotonic() + args.capture_timeout
            frames, rejected = [], "等待图像、标定板位姿和 TCP 反馈"
            next_diagnostic = time.monotonic() + 3.0
            reference = None
            used = set()
            while rclpy.ok() and time.monotonic() < deadline:
                rclpy.spin_once(self, timeout_sec=0.02)
                if time.monotonic() >= next_diagnostic:
                    print("采样诊断：\n" + self.diagnostics(rejected, len(frames)), flush=True)
                    next_diagnostic = time.monotonic() + 5.0
                for board in list(self.poses):
                    stamp = stamp_ns(board)
                    if stamp in used:
                        continue
                    if stamp <= start_ns:
                        rejected = ("板位姿时间戳为零" if stamp == 0 else
                                    "板位姿时间戳早于本组开始时间，请检查设备时钟或旧消息积压")
                        continue
                    now = self.get_clock().now().nanoseconds
                    if not 0 <= (now - stamp) * 1e-9 <= args.max_age:
                        rejected = "板位姿时间戳过期/超前，请检查相机与机械臂时钟"
                        continue
                    if board.header.frame_id != args.camera_frame:
                        rejected = (f"板位姿坐标系 {board.header.frame_id!r} 与 "
                                    f"--camera-frame {args.camera_frame!r} 不一致")
                        continue
                    image = next((m for m in self.images if stamp_ns(m) == stamp), None)
                    if image is None or not self.arm:
                        rejected = "未收到与板位姿时间戳完全相同的原始图像，或缺少 TCP"
                        continue
                    if max(a[0] for a in self.arm) < stamp:
                        rejected = "TCP 时间戳落后于图像，等待图像时间之后的 TCP 反馈"
                        continue  # 等图像时间之后的 TCP，以便选取最近邻。
                    robot_stamp, robot, T = min(self.arm, key=lambda a: abs(a[0] - stamp))
                    dt = abs(robot_stamp - stamp) * 1e-9
                    if dt > args.sync_tolerance:
                        rejected = f"图像/TCP 时间差 {dt:.3f}s 超过同步容差"
                        continue
                    window_start = robot_stamp - int(args.settle_time * 1e9)
                    history = sorted((a for a in self.arm if window_start <= a[0] <= robot_stamp),
                                     key=lambda a: a[0])
                    if (len(history) < 3 or history[0][0] - window_start > 100_000_000
                            or max(b[0] - a[0] for a, b in zip(history, history[1:])) > 200_000_000):
                        rejected = "停稳窗口内 TCP 反馈不足或存在断流"
                        continue
                    motion = [pose_errors(a[2], T) for a in history]
                    if any(mm > args.stable_mm or deg > args.stable_deg for mm, deg in motion):
                        rejected = "机械臂尚未停稳"
                        continue
                    try:
                        pose_matrix(**ros_pose(board))
                    except ValueError:
                        rejected = "标定板位姿包含无效数据"
                        continue
                    if reference is not None:
                        mm, deg = pose_errors(T, reference)
                        if mm > args.stable_mm or deg > args.stable_deg:
                            # 整组作废，不能把两个不同姿态合成一个采样点。
                            print("采样期间机械臂发生移动，本组已取消，请停稳后重采。")
                            return None
                    if reference is None:
                        reference = T
                    filename = f"images/pose_{sample_id:03d}_{len(frames)+1:02d}.png"
                    frame = self.bridge.imgmsg_to_cv2(image, desired_encoding="bgr8")
                    if not cv2.imwrite(str(directory / filename), frame):
                        raise OSError(f"图像保存失败：{filename}")
                    frames.append({"image": filename, "image_stamp_ns": stamp,
                                   "image_frame": image.header.frame_id,
                                   "board_frame": board.header.frame_id,
                                   "tcp_stamp_ns": robot_stamp,
                                   "tcp_frame": robot.header.frame_id,
                                   "sync_delta_ms": dt * 1000,
                                   "base_tcp": ros_pose(robot), "cam_board": ros_pose(board)})
                    used.add(stamp)
                    if len(frames) == args.frames_per_pose:
                        return {"id": sample_id, "frames": frames,
                                "camera_info": self.info}
            print("采样超时，本组未计入。\n" + self.diagnostics(rejected, len(frames)))
            return None

    rclpy.init(args=[])
    node = Collector()
    next_id = 1
    print(f"标定文件：{data['calibration']}\n模式：{args.mode}\n保存目录：{directory}")
    print("手内：板固定在桌面；手外：相机固定、板刚性固定在末端。")
    print("手动移动机械臂到不同位置并绕多轴旋转，停稳 1~2 秒后按 Enter。")
    print("Enter 采样 | d 撤销上一组 | r 查看当前误差 | q 保存报告并退出")
    try:
        while rclpy.ok():
            try:
                command = input(f"[{len(data['samples'])} 个姿态] > ").strip().lower()
            except EOFError:
                break
            if command == "":
                print(f"采集 {args.frames_per_pose} 帧，检查停稳和时间同步……", flush=True)
                sample = node.capture(next_id)
                next_id += 1
                if sample is not None:
                    data["samples"].append(sample)
                    print(f"已保存姿态 {sample['id']}，共 {len(data['samples'])} 组")
                write_json(directory / "samples.json", data)
            elif command == "d":
                if data["samples"]:
                    removed = data["samples"].pop()
                    print(f"已撤销姿态 {removed['id']}（照片保留供检查）")
                    write_json(directory / "samples.json", data)
            elif command == "r":
                if len(data["samples"]) >= 2:
                    report = evaluate(args.mode, X, data["samples"], args.min_poses)
                    print(f"当前平均误差：{report['translation_mm']['mean']:.3f} mm / "
                          f"{report['rotation_deg']['mean']:.3f}°")
                else:
                    print("至少采两个姿态后才能统计")
            elif command == "q":
                break
            else:
                print("有效操作：Enter / d / r / q")
    except KeyboardInterrupt:
        print("\n中断采样，保存已完成的姿态。")
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


def main(argv=None):
    parser = make_parser()
    args = parser.parse_args(argv)
    for name in ("frames_per_pose", "min_poses", "capture_timeout", "sync_tolerance",
                 "max_age", "settle_time", "stable_mm", "stable_deg"):
        value = getattr(args, name)
        if not 0 < value < float("inf"):
            parser.error(f"--{name.replace('_', '-')} 必须是有限正数")
    if args.settle_time < 0.2 or args.min_poses < 2:
        parser.error("--settle-time 至少 0.2 秒；--min-poses 至少 2")
    try:
        args.output_dir = args.output_dir.expanduser().resolve()
        if args.replay:
            data = json.loads(args.replay.expanduser().read_text(encoding="utf-8"))
            if args.mode != data["mode"]:
                raise ValueError("--mode 与采样记录的 mode 不一致")
            X = (load_calibration(args.calibration.expanduser()) if args.calibration else
                 pose_matrix(**data["calibration_pose"]))
        else:
            if args.calibration is None:
                candidates = sorted(args.output_dir.glob("*_calibration.json"))
                if not candidates:
                    raise ValueError(f"{args.output_dir} 下没有 *_calibration.json，请指定 --calibration")
                args.calibration = candidates[-1]
            args.calibration = args.calibration.expanduser().resolve()
            X = load_calibration(args.calibration)
            data = {"schema_version": 1, "mode": args.mode,
                    "calibration": str(args.calibration), "calibration_pose": matrix_pose(X),
                    "settings": {k: v for k, v in vars(args).items()
                                 if k not in ("calibration", "output_dir", "replay")},
                    "samples": []}
        if args.replay and args.calibration:
            data["calibration"] = str(args.calibration.expanduser().resolve())
            data["calibration_pose"] = matrix_pose(X)
        timestamp = datetime.now(ZoneInfo("Asia/Shanghai")).strftime("%Y-%m-%d_%H-%M-%S_%f")
        directory = args.output_dir / f"{timestamp}_{args.mode}_verification"
        (directory / "images").mkdir(parents=True, exist_ok=False)
        data["created_at"] = datetime.now(ZoneInfo("Asia/Shanghai")).isoformat()
        if args.replay:
            # 照片仍位于原始目录，记录来源以便追溯。
            data["source_samples"] = str(args.replay.expanduser().resolve())
            data.setdefault("image_root", str(args.replay.expanduser().resolve().parent))
        else:
            data["image_root"] = str(directory)
        write_json(directory / "samples.json", data)
        if not args.replay:
            try:
                collect(args, directory, data, X)
            finally:
                write_json(directory / "samples.json", data)
        if len(data["samples"]) < 2:
            print(f"仅有 {len(data['samples'])} 个姿态，未生成精度报告；数据已保存至 {directory}")
            return 2
        save_report(directory, data, X, args.min_poses)
        return 0
    except (OSError, ValueError, KeyError, TypeError, ImportError, cv2.error) as exc:
        print(f"验证失败：{exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
