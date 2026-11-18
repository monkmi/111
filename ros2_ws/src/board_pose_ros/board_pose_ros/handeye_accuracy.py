"""手眼验证的坐标变换和统计；不依赖 ROS，可离线重算。"""
from __future__ import annotations

import json
from pathlib import Path

import numpy as np

from board_pose_ros.detectors import quat_from_rotmat


MODES = ("eye_in_hand", "eye_to_hand")


def pose_matrix(position, orientation):
    """JSON/ROS 的 position(m), orientation(xyzw) 转为 T_parent_child。"""
    p = np.asarray(position, dtype=float)
    q = np.asarray(orientation, dtype=float)
    if p.shape != (3,) or q.shape != (4,):
        raise ValueError("position 必须为 3 元素，orientation 必须为 xyzw 四元数")
    if not np.all(np.isfinite(p)) or not np.all(np.isfinite(q)):
        raise ValueError("位姿包含 NaN/Inf")
    norm = np.linalg.norm(q)
    if norm < 1e-12:
        raise ValueError("四元数不能为零")
    x, y, z, w = q / norm
    T = np.eye(4)
    T[:3, :3] = [
        [1 - 2*(y*y + z*z), 2*(x*y - z*w), 2*(x*z + y*w)],
        [2*(x*y + z*w), 1 - 2*(x*x + z*z), 2*(y*z - x*w)],
        [2*(x*z - y*w), 2*(y*z + x*w), 1 - 2*(x*x + y*y)],
    ]
    T[:3, 3] = p
    return T


def matrix_pose(T):
    return {"position": T[:3, 3].tolist(),
            "orientation": quat_from_rotmat(T[:3, :3]).tolist()}


def load_calibration(path):
    with Path(path).open(encoding="utf-8") as f:
        data = json.load(f)
    return pose_matrix(data["position"], data["orientation"])


def fixed_board_pose(mode, X, base_tcp, cam_board):
    """手内 T_base_board；手外 T_tcp_board。X 直接读取官方结果，不取逆。"""
    if mode == "eye_in_hand":
        return base_tcp @ X @ cam_board
    if mode == "eye_to_hand":
        return np.linalg.inv(base_tcp) @ X @ cam_board
    raise ValueError(f"未知模式: {mode}")


def mean_pose(poses):
    """平移算术均值 + SO(3) chordal/SVD 均值，避开欧拉角的 ±180° 跳变。"""
    poses = np.asarray(poses)
    if len(poses) == 0:
        raise ValueError("没有可统计的位姿")
    U, _, Vt = np.linalg.svd(poses[:, :3, :3].sum(axis=0))
    T = np.eye(4)
    T[:3, :3] = U @ np.diag([1, 1, np.linalg.det(U @ Vt)]) @ Vt
    T[:3, 3] = poses[:, :3, 3].mean(axis=0)
    return T


def rotation_error_deg(R1, R2):
    delta = R1.T @ R2
    # atan2 在零角附近比 acos(trace) 更稳定，在 pi 附近也可用。
    sine = np.linalg.norm([delta[2, 1] - delta[1, 2],
                           delta[0, 2] - delta[2, 0],
                           delta[1, 0] - delta[0, 1]]) / 2
    cosine = np.clip((np.trace(delta) - 1) / 2, -1, 1)
    return float(np.rad2deg(np.arctan2(sine, cosine)))


def pose_errors(T, reference):
    return (float(np.linalg.norm(T[:3, 3] - reference[:3, 3]) * 1000),
            rotation_error_deg(reference[:3, :3], T[:3, :3]))


def statistics(values):
    values = np.asarray(values, dtype=float)
    return {"mean": float(values.mean()), "rms": float(np.sqrt(np.mean(values**2))),
            "max": float(values.max()), "std": float(values.std()),
            "p95": float(np.percentile(values, 95))}


def evaluate(mode, X, samples, min_poses=5):
    """每个机械臂姿态等权；先变换每帧，再求每姿态均值，最后统计姿态间误差。"""
    if len(samples) < 2:
        raise ValueError("至少需要两个机械臂姿态才能计算多姿态一致性误差")
    centers, per_pose = [], []
    for sample in samples:
        frames = sample["frames"]
        transformed = [fixed_board_pose(
            mode, X, pose_matrix(**f["base_tcp"]), pose_matrix(**f["cam_board"]))
            for f in frames]
        center = mean_pose(transformed)
        centers.append(center)
        jitter = [pose_errors(T, center) for T in transformed]
        per_pose.append({
            "id": sample["id"], "frame_count": len(frames),
            "fixed_board_pose": matrix_pose(center),
            "within_pose_translation_rms_mm": statistics([e[0] for e in jitter])["rms"],
            "within_pose_rotation_rms_deg": statistics([e[1] for e in jitter])["rms"],
        })
    reference = mean_pose(centers)
    errors = [pose_errors(T, reference) for T in centers]
    for row, error in zip(per_pose, errors):
        row.update(translation_error_mm=error[0], rotation_error_deg=error[1])

    # 姿态覆盖范围按各姿态第一帧 TCP 计算；避免把静止时反复拍照当作多姿态验证。
    arm = [pose_matrix(**s["frames"][0]["base_tcp"]) for s in samples]
    arm_pairs = [pose_errors(a, b) for i, a in enumerate(arm) for b in arm[i+1:]]
    warnings = []
    if len(samples) < min_poses:
        warnings.append(f"仅 {len(samples)} 个姿态，建议至少 {min_poses} 个，推荐 10~20 个")
    if max(e[0] for e in arm_pairs) < 50:
        warnings.append("TCP 平移覆盖不足 50 mm，请扩大工作区采样范围")
    if max(e[1] for e in arm_pairs) < 15:
        warnings.append("TCP 旋转覆盖不足 15°，请增加绕不同轴的旋转")
    return {
        "metric": "multi_pose_consistency_about_mean",
        "note": "这是相对均值的一致性误差，包含视觉/TCP/同步误差；不是绝对定位精度。",
        "mode": mode, "fixed_frame": "base" if mode == "eye_in_hand" else "tcp",
        "pose_count": len(samples), "frame_count": sum(len(s["frames"]) for s in samples),
        "reference_pose": matrix_pose(reference),
        "translation_mm": statistics([e[0] for e in errors]),
        "rotation_deg": statistics([e[1] for e in errors]),
        "tcp_translation_span_mm": max(e[0] for e in arm_pairs),
        "tcp_rotation_span_deg": max(e[1] for e in arm_pairs),
        "warnings": warnings, "per_pose": per_pose,
    }
