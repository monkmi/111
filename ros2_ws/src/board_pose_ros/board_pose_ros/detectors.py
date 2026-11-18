#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""标定板检测 + 平面 PnP 的纯 OpenCV 实现（不含 ROS 依赖，便于离线自测）。

支持三类标定板：
  * charuco  : OpenCV ChArUco 板（棋盘格 + ArUco 码），精度最高
  * dots     : 八点板（圆形点阵，如 3x3 缺一角 = 8 点）
  * checker  : 棋盘格内角点（8 个内角点即 4x2 或 2x4 的方格）

所有姿态均返回 “标定板坐标系 -> 相机坐标系” 的 rvec / tvec，
与 cv2.solvePnP / aruco_ros 的 /aruco_single/pose 语义一致：
    p_cam = R * p_board + t
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence, Tuple

import cv2
import numpy as np

# --------------------------------------------------------------------------- #
# 数据结构
# --------------------------------------------------------------------------- #


@dataclass
class Detection:
    """一次检测的结果。ok=False 时 msg 说明原因。"""

    ok: bool = False
    rvec: Optional[np.ndarray] = None          # (3,)  板 -> 相机
    tvec: Optional[np.ndarray] = None          # (3,)  单位 m
    reproj_err: float = float("inf")           # 像素 RMS 重投影误差
    n_found: int = 0                           # 找到的点/角点总数
    n_used: int = 0                            # 参与 PnP 的点数
    msg: str = ""
    points_3d: Optional[np.ndarray] = None     # (N,3) 板坐标系下的物点
    points_2d: Optional[np.ndarray] = None     # (N,2) 像素坐标

    def rotation(self) -> Optional[np.ndarray]:
        if self.rvec is None:
            return None
        return cv2.Rodrigues(self.rvec.astype(np.float64))[0]

    def pose_quat(self) -> Optional[Tuple[np.ndarray, np.ndarray]]:
        """返回 (translation, quaternion xyzw)。"""
        if not self.ok:
            return None
        q = quat_from_rotmat(self.rotation())
        return self.tvec.astype(np.float64), q


# --------------------------------------------------------------------------- #
# 通用数学
# --------------------------------------------------------------------------- #

_DICT_NAMES = {
    name[len("DICT_"):] if name.startswith("DICT_") else name: getattr(cv2.aruco, name)
    for name in dir(cv2.aruco)
    if name.startswith("DICT_")
}


def dictionary_ids() -> List[str]:
    """可用的 ArUco 字典名（不带 DICT_ 前缀）。"""
    return sorted(_DICT_NAMES)


def get_dictionary(name: str):
    key = name.upper()
    if key.startswith("DICT_"):
        key = key[len("DICT_"):]
    if key not in _DICT_NAMES:
        raise ValueError(
            "未知 ArUco 字典 '%s'，可用: %s" % (name, ", ".join(dictionary_ids()))
        )
    return cv2.aruco.Dictionary_get(_DICT_NAMES[key])


def quat_from_rotmat(R: np.ndarray) -> np.ndarray:
    """旋转矩阵 -> 四元数 (x, y, z, w)，数值稳定版本。"""
    R = np.asarray(R, dtype=np.float64)
    m00, m01, m02 = R[0, 0], R[0, 1], R[0, 2]
    m10, m11, m12 = R[1, 0], R[1, 1], R[1, 2]
    m20, m21, m22 = R[2, 0], R[2, 1], R[2, 2]
    tr = m00 + m11 + m22
    if tr > 0.0:
        s = math.sqrt(tr + 1.0) * 2.0
        w = 0.25 * s
        x = (m21 - m12) / s
        y = (m02 - m20) / s
        z = (m10 - m01) / s
    elif m00 > m11 and m00 > m22:
        s = math.sqrt(1.0 + m00 - m11 - m22) * 2.0
        w = (m21 - m12) / s
        x = 0.25 * s
        y = (m01 + m10) / s
        z = (m02 + m20) / s
    elif m11 > m22:
        s = math.sqrt(1.0 + m11 - m00 - m22) * 2.0
        w = (m02 - m20) / s
        x = (m01 + m10) / s
        y = 0.25 * s
        z = (m12 + m21) / s
    else:
        s = math.sqrt(1.0 + m22 - m00 - m11) * 2.0
        w = (m10 - m01) / s
        x = (m02 + m20) / s
        y = (m12 + m21) / s
        z = 0.25 * s
    q = np.array([x, y, z, w], dtype=np.float64)
    return q / np.linalg.norm(q)


def rotmat_to_rpy(R: np.ndarray) -> np.ndarray:
    """旋转矩阵 -> 固定轴 XYZ 欧拉角（等价 tf_transformations 的 euler_from_matrix）。"""
    R = np.asarray(R, dtype=np.float64)
    sy = math.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2)
    if sy > 1e-8:
        roll = math.atan2(R[2, 1], R[2, 2])
        pitch = math.atan2(-R[2, 0], sy)
        yaw = math.atan2(R[1, 0], R[0, 0])
    else:
        roll = math.atan2(-R[1, 2], R[1, 1])
        pitch = math.atan2(-R[2, 0], sy)
        yaw = 0.0
    return np.array([roll, pitch, yaw], dtype=np.float64)


def reprojection_rms(pts3d, pts2d, rvec, tvec, K, D) -> float:
    pts3d = np.asarray(pts3d, dtype=np.float64).reshape(-1, 3)
    pts2d = np.asarray(pts2d, dtype=np.float64).reshape(-1, 2)
    proj, _ = cv2.projectPoints(pts3d, np.asarray(rvec).reshape(3),
                                np.asarray(tvec).reshape(3), K, D)
    d = np.linalg.norm(proj.reshape(-1, 2) - pts2d, axis=1)
    return float(np.sqrt(np.mean(d * d)))


def pnp_candidates(pts3d, pts2d, K, D):
    """平面物点的全部 PnP 候选解。

    返回 [(facing, rms, rvec, tvec), ...]，facing 表示 “板子印刷面朝向相机”。
    IPPE 的镜像解在数学上与正解等价（点集关于面内轴转 180° 后投影相同），
    必须用 facing 把 “从背面看板” 的假解排除，否则手眼结果会整体镜像。
    """
    pts3d = np.asarray(pts3d, dtype=np.float64).reshape(-1, 1, 3)
    pts2d = np.asarray(pts2d, dtype=np.float64).reshape(-1, 1, 2)
    if len(pts3d) < 4:
        raise ValueError("PnP 至少需要 4 个点，当前 %d 个" % len(pts3d))

    raw = []
    try:
        ret, rvecs, tvecs, _ = cv2.solvePnPGeneric(
            pts3d, pts2d, K, D, flags=cv2.SOLVEPNP_IPPE
        )
    except cv2.error:
        rvecs, tvecs = [], []
    for rvec, tvec in zip(rvecs, tvecs):
        raw.append((np.asarray(rvec, dtype=np.float64).reshape(3),
                    np.asarray(tvec, dtype=np.float64).reshape(3)))
    if not raw:
        rvec, tvec = np.zeros(3), np.zeros(3)
        ok, rvec, tvec = cv2.solvePnP(pts3d, pts2d, K, D, rvec, tvec, False,
                                      flags=cv2.SOLVEPNP_ITERATIVE)
        raw.append((np.asarray(rvec, dtype=np.float64).reshape(3),
                    np.asarray(tvec, dtype=np.float64).reshape(3)))

    out = []
    flat3d = np.asarray(pts3d).reshape(-1, 3)
    for rvec, tvec in raw:
        try:
            rv, tv = cv2.solvePnPRefineLM(pts3d, pts2d, K, D, rvec, tvec)
            rvec = np.asarray(rv, dtype=np.float64).reshape(3)
            tvec = np.asarray(tv, dtype=np.float64).reshape(3)
        except cv2.error:
            pass
        R = cv2.Rodrigues(rvec)[0]
        z = (R @ flat3d.T + tvec.reshape(3, 1))[2]
        if np.any(z <= 0.0):
            continue
        facing = float(np.dot(R[:, 2], tvec)) < 0.0
        out.append((facing, reprojection_rms(pts3d, pts2d, rvec, tvec, K, D), rvec, tvec))
    return out


def solve_planar_pose(pts3d, pts2d, K, D):
    """平面物点求位姿，返回 (rvec, tvec, rms)；优先取 “正面朝向相机” 的解。"""
    cands = pnp_candidates(pts3d, pts2d, K, D)
    if not cands:
        raise ValueError("PnP 无有效解（物点可能不在相机前方）")
    facing = [c for c in cands if c[0]]
    pool = facing if facing else cands
    _, rms, rvec, tvec = min(pool, key=lambda c: c[1])
    return rvec, tvec, rms


# --------------------------------------------------------------------------- #
# ChArUco
# --------------------------------------------------------------------------- #


def default_detector_parameters(subpix: bool = True):
    params = cv2.aruco.DetectorParameters_create()
    if subpix:
        params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        params.cornerRefinementWinSize = 5
        params.cornerRefinementMaxIterations = 30
    params.adaptiveThreshWinSizeMin = 3
    params.adaptiveThreshWinSizeMax = 23
    params.adaptiveThreshWinSizeStep = 10
    params.minMarkerPerimeterRate = 0.02
    params.maxMarkerPerimeterRate = 4.0
    params.polygonalApproxAccuracyRate = 0.03
    return params


def build_charuco_board(dictionary: str, squares_x: int, squares_y: int,
                        square_length: float, marker_length: float):
    """square_length / marker_length 单位为米（与标定板实际打印尺寸一致）。"""
    if float(marker_length) >= float(square_length):
        raise ValueError("marker_length 必须小于 square_length")
    return cv2.aruco.CharucoBoard_create(
        int(squares_x), int(squares_y), float(square_length), float(marker_length),
        get_dictionary(dictionary),
    )


def charuco_board_size(board) -> Tuple[float, float]:
    """板面物理尺寸 (宽, 高)，米。"""
    return (board.getChessboardSize()[0] * board.getSquareLength(),
            board.getChessboardSize()[1] * board.getSquareLength())


def _charuco_pose_from_corners(gray, board, ch_corners, ch_ids, K, D) -> Detection:
    """由 interpolateCornersCharuco 的结果求解板位姿。"""
    board_obj = np.asarray(board.chessboardCorners, dtype=np.float64).reshape(-1, 3)
    ids_flat = np.asarray(ch_ids, dtype=np.int32).ravel()
    if board_obj.shape[0] <= int(ids_flat.max()):
        return Detection(n_found=0, msg="板参数与角点 id 不一致，请检查 squares_x/squares_y")
    obj_pts = board_obj[ids_flat]
    img_pts = np.asarray(ch_corners, dtype=np.float64).reshape(-1, 2)
    rvec, tvec, rms = solve_planar_pose(obj_pts, img_pts, K, D)
    return Detection(ok=True, rvec=rvec, tvec=tvec, reproj_err=rms,
                     n_found=len(ch_ids), n_used=len(obj_pts),
                     points_3d=obj_pts, points_2d=img_pts)


AUTO_CHARUCO_DICTIONARIES = (
    "DICT_4X4_50", "DICT_4X4_100", "DICT_4X4_250",
    "DICT_5X5_50", "DICT_5X5_100", "DICT_5X5_250",
    "DICT_6X6_50", "DICT_6X6_100", "DICT_6X6_250",
    "DICT_7X7_50", "DICT_7X7_100", "DICT_7X7_250",
    "DICT_ARUCO_ORIGINAL",
)


def detect_charuco_auto(gray, squares_x: int, squares_y: int,
                        square_length: float, marker_length: float, K, D,
                        min_corners: int = 8, subpix: bool = True,
                        dictionaries=None):
    """自动识别字典类型与板的横竖方向。

    实物板常见"只知道格子数、格子边长，不知道字典和码边长"的情况：
    这里把候选字典逐个试，并对 (squares_x, squares_y) 与转置两种朝向都试，
    取内角点最多、重投影最小的解。

    Returns:
        (Detection, info)  info 形如
        {"dictionary": "DICT_4X4_50", "squares_x": 7, "squares_y": 9, "n_markers": 48}
    """
    names = list(dictionaries or AUTO_CHARUCO_DICTIONARIES)
    params = default_detector_parameters(subpix)
    best = None
    tried = []
    for name in names:
        try:
            dic = get_dictionary(name)
        except Exception:  # noqa: BLE001
            continue
        corners, ids, _ = cv2.aruco.detectMarkers(gray, dic, parameters=params)
        n_ids = 0 if ids is None else len(ids)
        tried.append({"dictionary": name, "markers": n_ids})
        if n_ids < 4:
            continue
        for sx, sy in ((squares_x, squares_y), (squares_y, squares_x)):
            try:
                board = build_charuco_board(name, sx, sy, square_length, marker_length)
            except Exception:  # noqa: BLE001
                continue
            ret, ch_corners, ch_ids = cv2.aruco.interpolateCornersCharuco(
                corners, ids, gray, board, cameraMatrix=K, distCoeffs=D)
            if ch_corners is None or ch_ids is None or len(ch_ids) < min_corners:
                continue
            try:
                det = _charuco_pose_from_corners(gray, board, ch_corners, ch_ids, K, D)
            except Exception:  # noqa: BLE001  某个字典/朝向是错的，换下一个候选
                continue
            if not det.ok:
                continue
            score = (det.n_used, -float(det.reproj_err))
            if best is None or score > best[0]:
                best = (score, det, {"dictionary": name, "squares_x": sx,
                                     "squares_y": sy, "n_markers": n_ids,
                                     "tried": tried})
    if best is None:
        best_markers = max([t["markers"] for t in tried], default=0)
        return (Detection(msg="未识别出 ChArUco 板（尝试 %d 种字典，最多检出 %d 个 marker）"
                               % (len(tried), best_markers)),
                {"tried": tried})
    return best[1], best[2]


def detect_charuco(gray, board, K, D, dictionary: str,
                   min_corners: int = 8, subpix: bool = True) -> Detection:
    params = default_detector_parameters(subpix)
    corners, ids, _ = cv2.aruco.detectMarkers(gray, get_dictionary(dictionary),
                                              parameters=params)
    if ids is None or len(ids) == 0:
        return Detection(msg="未检测到 ArUco marker")

    ret, ch_corners, ch_ids = cv2.aruco.interpolateCornersCharuco(
        corners, ids, gray, board, cameraMatrix=K, distCoeffs=D
    )
    if ch_corners is None or ch_ids is None or len(ch_ids) < min_corners:
        n = 0 if ch_ids is None else len(ch_ids)
        return Detection(n_found=n, msg="ChArUco 内角点不足 (%d < %d)" % (n, min_corners))

    return _charuco_pose_from_corners(gray, board, ch_corners, ch_ids, K, D)


# --------------------------------------------------------------------------- #
# 棋盘格内角点
# --------------------------------------------------------------------------- #


def checker_object_points(cols: int, rows: int, square_length: float) -> np.ndarray:
    """内角点物点，板坐标系居中：X 向右、Y 向上、Z 垂直板面向外。"""
    pts = []
    for r in range(rows):
        for c in range(cols):
            pts.append([(c - (cols - 1) / 2.0) * square_length,
                        ((rows - 1) / 2.0 - r) * square_length,
                        0.0])
    return np.asarray(pts, dtype=np.float64)


def _flip_object_points_180(obj: np.ndarray) -> np.ndarray:
    """把物点绕板面法线转 180°。棋盘格（对称图案）存在这种二义性。"""
    out = np.array(obj, dtype=np.float64, copy=True)
    out[:, 0] *= -1.0
    out[:, 1] *= -1.0
    return out


def _angle_between_rvecs(rvec_a, rvec_b) -> float:
    Ra = cv2.Rodrigues(np.asarray(rvec_a, dtype=np.float64).reshape(3))[0]
    Rb = cv2.Rodrigues(np.asarray(rvec_b, dtype=np.float64).reshape(3))[0]
    return float(np.rad2deg(np.arccos(np.clip((np.trace(Ra.T @ Rb) - 1.0) / 2.0, -1.0, 1.0))))


def detect_checker(gray, cols: int, rows: int, square_length: float, K, D,
                   subpix: bool = True, hint=None) -> Detection:
    """棋盘格内角点检测。

    棋盘格图案在 180° 旋转下自相似，因此 “物点顺序” 存在 180° 二义性；
    传入 hint=(rvec, tvec)（上一帧结果）可保持多帧一致，避免位姿突然翻转。
    """
    cols, rows = int(cols), int(rows)
    if cols < 3 or rows < 3:
        return Detection(msg="OpenCV 要求棋盘格内角点 cols>2 且 rows>2（当前 %dx%d）"
                             % (cols, rows))
    flags = cv2.CALIB_CB_NORMALIZE_IMAGE | cv2.CALIB_CB_EXHAUSTIVE | cv2.CALIB_CB_ACCURACY
    found, corners = cv2.findChessboardCornersSB(gray, (cols, rows), flags)
    if not found:
        return Detection(msg="未检测到棋盘格内角点 (%dx%d)" % (cols, rows))
    corners = np.asarray(corners, dtype=np.float64).reshape(-1, 2)
    if subpix and len(corners) >= 4:
        try:
            crit = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 50, 1e-3)
            corners = cv2.cornerSubPix(gray, corners.astype(np.float32).reshape(-1, 1, 2),
                                       (5, 5), (-1, -1), crit)
            corners = corners.reshape(-1, 2)
        except cv2.error:
            pass

    obj0 = checker_object_points(cols, rows, square_length)
    best = None
    for obj in (obj0, _flip_object_points_180(obj0)):
        try:
            cands = pnp_candidates(obj, corners, K, D)
        except ValueError:
            continue
        for facing, rms, rvec, tvec in cands:
            if hint is not None and hint[0] is not None:
                key = (0, _angle_between_rvecs(hint[0], rvec), rms)
            else:
                key = (0 if facing else 1, 0.0, rms)
            if best is None or key < best[0]:
                best = (key, facing, rvec, tvec, rms, obj)
    if best is None:
        return Detection(n_found=len(corners), msg="棋盘格位姿求解失败")
    _, facing, rvec, tvec, rms, obj = best
    return Detection(ok=True, rvec=rvec, tvec=tvec, reproj_err=rms,
                     n_found=len(corners), n_used=len(corners),
                     points_3d=obj, points_2d=corners,
                     msg="ok" if facing else "ok（未能确认板面朝向）")


# --------------------------------------------------------------------------- #
# 八点板（圆点阵）
# --------------------------------------------------------------------------- #


def find_dot_centers(gray, min_area_frac: float = 1.5e-4, max_area_frac: float = 2.0e-2,
                     min_circularity: float = 0.55, min_fill: float = 0.55,
                     dark_on_light: bool = True, blur_ksize: int = 5) -> np.ndarray:
    """阈值 + 轮廓 + 圆形度/填充率筛选，返回点中心 (N,2)，亚像素级靠矩心。"""
    if gray.ndim == 3:
        gray = cv2.cvtColor(gray, cv2.COLOR_BGR2GRAY)
    h, w = gray.shape[:2]
    area_img = float(h * w)
    min_area = max(9.0, min_area_frac * area_img)
    max_area = max_area_frac * area_img

    if blur_ksize and blur_ksize > 0:
        k = blur_ksize if blur_ksize % 2 == 1 else blur_ksize + 1
        blur = cv2.GaussianBlur(gray, (k, k), 0)
    else:
        blur = gray

    mode = cv2.THRESH_BINARY_INV if dark_on_light else cv2.THRESH_BINARY
    _, th = cv2.threshold(blur, 0, 255, mode | cv2.THRESH_OTSU)
    cnts, _ = cv2.findContours(th, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    centers = []
    for cnt in cnts:
        area = cv2.contourArea(cnt)
        if area < min_area or area > max_area:
            continue
        perim = cv2.arcLength(cnt, True)
        if perim <= 0.0:
            continue
        circ = 4.0 * math.pi * area / (perim * perim)
        if circ < min_circularity:
            continue
        (_, _), radius = cv2.minEnclosingCircle(cnt)
        if radius <= 0.0:
            continue
        if area / (math.pi * radius * radius) < min_fill:
            continue
        m = cv2.moments(cnt)
        if m["m00"] <= 0.0:
            continue
        centers.append((m["m10"] / m["m00"], m["m01"] / m["m00"]))
    return np.asarray(centers, dtype=np.float64).reshape(-1, 2)


def parse_missing_cells(text: str) -> List[Tuple[int, int]]:
    """解析 '0,0;2,2' 形式的缺角描述（行,列）。"""
    cells = []
    for chunk in str(text).replace(" ", "").split(";"):
        if not chunk:
            continue
        parts = chunk.split(",")
        if len(parts) != 2:
            raise ValueError("dot_missing 格式应为 '行,列;行,列'，收到 '%s'" % text)
        cells.append((int(parts[0]), int(parts[1])))
    return cells


def dots_layout(rows: int, cols: int, spacing: float,
                missing: Optional[Sequence[Tuple[int, int]]] = None):
    """返回 (occupied_cells, object_points)。物点板坐标系居中、X 向右、Y 向上。"""
    if isinstance(missing, str):
        missing = parse_missing_cells(missing)
    missing = set(missing or [])
    cells, pts = [], []
    for r in range(rows):
        for c in range(cols):
            if (r, c) in missing:
                continue
            cells.append((r, c))
            pts.append([(c - (cols - 1) / 2.0) * spacing,
                        ((rows - 1) / 2.0 - r) * spacing,
                        0.0])
    return cells, np.asarray(pts, dtype=np.float64)


def _normalize_points(pts: np.ndarray) -> np.ndarray:
    c = pts.mean(axis=0)
    s = float(np.mean(np.linalg.norm(pts - c, axis=1))) + 1e-12
    return (pts - c) / s


def _assign_by_homography(q, cells, matched_cells, matched_idx, tol):
    """用已匹配点拟合 H，再给未匹配的格子就近认领未使用的检测点。"""
    if len(matched_idx) < 4:
        return matched_cells, matched_idx, float("inf")

    src = np.asarray([q[i] for i in matched_idx], dtype=np.float64)
    dst = np.asarray([[c[1], c[0]] for c in matched_cells], dtype=np.float64)
    H, _ = cv2.findHomography(src, dst, 0)
    if H is None:
        return matched_cells, matched_idx, float("inf")

    used = set(matched_idx)
    new_cells = list(matched_cells)
    new_idx = list(matched_idx)
    # 反投影：把每个未匹配格子投回归一化坐标再去匹配检测点
    H_inv = np.linalg.inv(H)

    for cell in cells:
        if cell in new_cells:
            continue
        p_h = H_inv @ np.array([cell[1], cell[0], 1.0])
        p = p_h[:2] / p_h[2]
        d = np.linalg.norm(q - p, axis=1)
        for idx in np.argsort(d):
            if int(idx) in used:
                continue
            if d[idx] > tol:
                break
            used.add(int(idx))
            new_cells.append(cell)
            new_idx.append(int(idx))
            break

    src = np.asarray([q[i] for i in new_idx], dtype=np.float64)
    dst = np.asarray([[c[1], c[0]] for c in new_cells], dtype=np.float64)
    H2, _ = cv2.findHomography(src, dst, 0)
    if H2 is None:
        return new_cells, new_idx, float("inf")
    proj = (H2 @ np.column_stack([src, np.ones(len(src))]).T).T
    proj = proj[:, :2] / proj[:, 2:3]
    resid = float(np.mean(np.linalg.norm(proj - dst, axis=1)))
    return new_cells, new_idx, resid


def order_dots_on_grid(points: np.ndarray, rows: int, cols: int,
                       occupied: Optional[Sequence[Tuple[int, int]]] = None,
                       tol: float = 0.30):
    """给无序点阵建立 “点 -> 网格(行,列)” 的对应关系。

    做法：以每个点为中心、取两个近邻方向作网格基向量，先仿射外推出候选格点，
    再用单应矩阵迭代精化，最后用 “占用格与实际检测点的覆盖率 + 残差” 打分，
    择优返回。镜像/180° 歧义由占用格集合（例如缺一角）自然eliminate。

    返回候选列表 [(matched_cells, matched_idx, residual), ...]（按几何可信度降序，最多 4 个）；
    失败抛 RuntimeError。
    """
    pts = np.asarray(points, dtype=np.float64).reshape(-1, 2)
    n = len(pts)
    cells = list(occupied) if occupied else [(r, c) for r in range(rows) for c in range(cols)]
    cells_set = set(cells)
    if n < 4:
        raise RuntimeError("八点板至少需要检测到 4 个点，当前 %d 个" % n)

    q = _normalize_points(pts)
    dmat = np.linalg.norm(q[:, None, :] - q[None, :, :], axis=2)
    knn = min(n - 1, 6)
    order = np.argsort(dmat, axis=1)

    span = max(rows, cols) + 2
    best = None
    cands: Dict[frozenset, tuple] = {}

    for s in range(n):
        neigh = [int(j) for j in order[s][1:knn + 1] if dmat[s, j] > 0.25]
        for ia in range(len(neigh)):
            for ib in range(len(neigh)):
                if ia == ib:
                    continue
                a, b = neigh[ia], neigh[ib]
                v1 = q[b] - q[s]          # 列方向 (+x)
                v2 = q[a] - q[s]          # 行方向 (+y)
                n1, n2 = np.linalg.norm(v1), np.linalg.norm(v2)
                if n1 < 0.3 or n2 < 0.3:
                    continue
                cosang = abs(float(np.dot(v1, v2) / (n1 * n2)))
                if cosang > 0.55:
                    continue

                # 1) 仿射外推：整格坐标 -> 最近检测点
                assign: Dict[Tuple[int, int], int] = {}
                for i in range(-span, span + 1):
                    for j in range(-span, span + 1):
                        if (i, j) == (0, 0):
                            continue
                        p = q[s] + j * v1 + i * v2
                        d = np.linalg.norm(q - p, axis=1)
                        k = int(np.argmin(d))
                        if d[k] > 0.45:
                            continue
                        if (i, j) not in assign or d[k] < assign[(i, j)][0]:
                            assign[(i, j)] = (float(d[k]), k)
                assign = {k: v[1] for k, v in assign.items()}
                # 一个检测点只能占一个格
                seen = {}
                for cell, idx in sorted(assign.items(), key=lambda kv: np.linalg.norm(
                        q[kv[1]] - (q[s] + kv[0][1] * v1 + kv[0][0] * v2))):
                    if idx not in seen:
                        seen[idx] = cell
                if len(seen) < 4:
                    continue
                rel = {cell: idx for idx, cell in seen.items()}

                # 2) 找平移量 (r0, c0) 使相对格最大程度落在占用格集合内
                shifts = {}
                for cell in rel:
                    for occ in cells:
                        shift = (cell[0] - occ[0], cell[1] - occ[1])
                        shifts[shift] = shifts.get(shift, 0) + 1
                best_shift, best_cnt = None, -1
                for shift, cnt in shifts.items():
                    cells_rel = {(c[0] - shift[0], c[1] - shift[1]) for c in rel}
                    valid = len([c for c in cells_rel if c in cells_set])
                    score = valid
                    if score > best_cnt:
                        best_cnt, best_shift = score, shift
                if best_shift is None or best_cnt < 4:
                    continue

                matched_cells, matched_idx = [], []
                for cell, idx in rel.items():
                    rc = (cell[0] - best_shift[0], cell[1] - best_shift[1])
                    if rc in cells_set:
                        matched_cells.append(rc)
                        matched_idx.append(idx)

                # 3) 单应精化
                matched_cells, matched_idx, resid = _assign_by_homography(
                    q, cells, matched_cells, matched_idx, tol
                )
                coverage = len(matched_cells) / float(len(cells))
                score = coverage * 1000.0 - resid * 400.0
                if best is None or score > best[0]:
                    best = (score, matched_cells, matched_idx, resid, coverage)
                key = frozenset(zip(matched_cells, matched_idx))
                if key not in cands or score > cands[key][0]:
                    cands[key] = (score, matched_cells, matched_idx, resid)

    if best is None:
        raise RuntimeError("无法为点阵建立网格对应关系（点太少或布局参数不符）")

    _, matched_cells, matched_idx, resid, coverage = best
    if len(matched_idx) < 4:
        raise RuntimeError("网格对应点不足 4 个")
    if resid > 0.35:
        raise RuntimeError("网格拟合残差过大 (%.3f)，请检查 dot_rows/dot_cols/dot_missing"
                           % resid)

    # 按几何得分排序返回多个候选：对称布局（如 3x3 缺角）存在镜像二义性，
    # 由 detect_dots 再用 PnP 的 "板面朝向相机" 条件挑出唯一物理解。
    ranked = sorted(cands.values(), key=lambda c: -c[0])
    out = []
    seen = set()
    for _score, mc, mi, rs in ranked:
        key = frozenset(zip(mc, mi))
        if key in seen:
            continue
        seen.add(key)
        out.append((mc, mi, rs))
        if len(out) >= 4:
            break
    return out


def detect_dots(gray, rows: int, cols: int, spacing: float, K, D,
                missing: Optional[Sequence[Tuple[int, int]]] = None,
                min_points: int = 4, dark_on_light: bool = True,
                min_area_frac: float = 1.5e-4, max_area_frac: float = 2.0e-2,
                min_circularity: float = 0.55, min_fill: float = 0.55,
                blur_ksize: int = 5) -> Detection:
    pts = find_dot_centers(gray, min_area_frac=min_area_frac, max_area_frac=max_area_frac,
                           min_circularity=min_circularity, min_fill=min_fill,
                           dark_on_light=dark_on_light, blur_ksize=blur_ksize)
    n_found = len(pts)
    if n_found < min_points:
        return Detection(n_found=n_found,
                         msg="圆点数不足 (%d < %d)" % (n_found, max(min_points, 4)))
    cells, obj = dots_layout(rows, cols, spacing, missing)
    cell_to_obj = {c: obj[i] for i, c in enumerate(cells)}
    try:
        label_cands = order_dots_on_grid(pts, rows, cols, cells)
    except RuntimeError as exc:
        return Detection(n_found=n_found, msg=str(exc))

    best = None
    for matched_cells, matched_idx, _resid in label_cands:
        obj_pts = np.asarray([cell_to_obj[c] for c in matched_cells], dtype=np.float64)
        img_pts = np.asarray([pts[i] for i in matched_idx], dtype=np.float64)
        try:
            cands = pnp_candidates(obj_pts, img_pts, K, D)
        except ValueError:
            continue
        for facing, rms, rvec, tvec in cands:
            # 先看是否正面朝向相机，再看参与点数（多者更可信），最后比重投影误差
            key = (0 if facing else 1, -len(matched_idx), rms)
            if best is None or key < best[0]:
                best = (key, rvec, tvec, rms, obj_pts, img_pts, len(matched_idx))
    if best is None:
        return Detection(n_found=n_found, msg="点阵位姿求解失败")

    _, rvec, tvec, rms, obj_pts, img_pts, n_used = best
    return Detection(ok=True, rvec=rvec, tvec=tvec, reproj_err=rms,
                     n_found=n_found, n_used=n_used,
                     points_3d=obj_pts, points_2d=img_pts)


# --------------------------------------------------------------------------- #
# 调试可视化
# --------------------------------------------------------------------------- #


def pose_quat_from_rvec(rvec, tvec):
    """(rvec, tvec) -> (translation, quaternion xyzw)。"""
    R = cv2.Rodrigues(np.asarray(rvec, dtype=np.float64).reshape(3))[0]
    return np.asarray(tvec, dtype=np.float64).reshape(3), quat_from_rotmat(R)


def draw_debug(image_bgr, det: Detection, label: str = "",
               K=None, D=None, axis_length: float = 0.03,
               rvec=None, tvec=None) -> np.ndarray:
    """画检测图。rvec/tvec 缺省用 det 的位姿（可用于画变换后的位姿）。"""
    out = image_bgr.copy()
    if rvec is not None:
        det = Detection(ok=True, rvec=np.asarray(rvec, np.float64).reshape(3),
                        tvec=np.asarray(tvec, np.float64).reshape(3),
                        points_2d=det.points_2d, points_3d=det.points_3d,
                        n_used=det.n_used, reproj_err=det.reproj_err, msg=det.msg)
    if out.ndim == 2:
        out = cv2.cvtColor(out, cv2.COLOR_GRAY2BGR)
    if det.ok and det.points_2d is not None:
        for (u, v) in det.points_2d:
            cv2.circle(out, (int(round(u)), int(round(v))), 3, (0, 255, 0), -1)
        if K is not None and det.rvec is not None:
            try:
                cv2.drawFrameAxes(out, np.asarray(K, np.float64),
                                  np.zeros(5) if D is None else np.asarray(D, np.float64),
                                  det.rvec, det.tvec, axis_length)
            except cv2.error:
                pass
    color = (0, 200, 0) if det.ok else (0, 0, 255)
    text = label or ("OK pts=%d rms=%.2fpx" % (det.n_used, det.reproj_err)
                     if det.ok else "FAIL: " + det.msg)
    cv2.putText(out, text, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2, cv2.LINE_AA)
    if det.ok and det.tvec is not None:
        cv2.putText(out, "t=[%.3f %.3f %.3f]m" % tuple(det.tvec),
                    (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 1, cv2.LINE_AA)
    return out
