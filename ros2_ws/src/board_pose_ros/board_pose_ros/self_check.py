#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""离线自测：用合成图像验证 ChArUco / 八点板 / 棋盘格 三种检测的位姿精度。

不需要相机和 ROS，直接运行：
    ros2 run board_pose_ros board_self_check
    # 或在源码目录: python3 -m board_pose_ros.self_check
"""
from __future__ import annotations

import sys

import cv2
import numpy as np

from board_pose_ros import detectors as D

IMG_W, IMG_H = 1280, 960
K = np.array([[900.0, 0.0, 640.0], [0.0, 902.0, 480.0], [0.0, 0.0, 1.0]], float)
# 合成图用零畸变渲染（单应投影不含畸变），因此检测也用零畸变，避免人为模型误差
DIST = np.zeros(5, float)


def pose_like(rvec, tvec, cam_dist=0.45, tilt_deg=(155.0, 0.0, 20.0), yaw_deg=0.0):
    """构造一个“板面朝向相机、摆在相机正前方”的位姿。

    tilt_deg=(rx, ry, rz) 为板面倾斜；yaw_deg 是在此基础上的绕板面法线旋转（真正的面内旋转）。
    """
    del rvec, tvec
    rx, ry, rz = np.deg2rad(tilt_deg)
    def rot(axis, ang):
        return cv2.Rodrigues(np.array(axis, float) * ang)[0]
    R = rot([0, 0, 1], np.deg2rad(yaw_deg) + rz) @ rot([1, 0, 0], rx) @ rot([0, 1, 0], ry)
    t = np.array([0.0, 0.0, cam_dist])
    return R, t


def compose_homography(board_px_to_board_metric, R, t, K):
    """板面像素 -> 相机像素 的单应（板面 z=0）。"""
    P = np.array([[R[0, 0], R[0, 1], t[0]],
                  [R[1, 0], R[1, 1], t[1]],
                  [R[2, 0], R[2, 1], t[2]]], float)
    H_cam = K @ P
    return H_cam @ board_px_to_board_metric


def render_board_in_camera(board_img, board_px_to_board_metric, R, t, out_size=(IMG_W, IMG_H)):
    H = compose_homography(board_px_to_board_metric, R, t, K)
    canvas = np.full((out_size[1], out_size[0]), 255, np.uint8)
    warped = cv2.warpPerspective(board_img, H, out_size, flags=cv2.INTER_AREA,
                                 borderMode=cv2.BORDER_CONSTANT, borderValue=255)
    mask = cv2.warpPerspective(np.full_like(board_img, 255), H, out_size,
                               flags=cv2.INTER_NEAREST, borderMode=cv2.BORDER_CONSTANT,
                               borderValue=0)
    canvas[mask > 0] = warped[mask > 0]
    return canvas


def report(name, det, R_gt, t_gt):
    if not det.ok:
        print("  [FAIL] %s: %s" % (name, det.msg))
        return False
    R_est = det.rotation()
    ang = np.rad2deg(np.arccos(np.clip((np.trace(R_gt.T @ R_est) - 1) / 2, -1, 1)))
    dt = np.linalg.norm(det.tvec - t_gt)
    ok = ang < 0.6 and dt < 0.002 and det.reproj_err < 1.5
    print("  [%s] %s: 点数=%d 旋转误差=%.3f° 平移误差=%.2fmm 重投影=%.3fpx"
          % ("PASS" if ok else "FAIL", name, det.n_used, ang, dt * 1000.0, det.reproj_err))
    return ok


def test_charuco():
    print("== ChArUco 板 ==")
    squares_x, squares_y = 5, 7
    square_len, marker_len = 0.030, 0.022
    board = D.build_charuco_board("4X4_50", squares_x, squares_y, square_len, marker_len)
    Wpx = int(round(squares_x * square_len * 4000))   # ~4200 px/m
    Hpx = int(round(squares_y * square_len * 4000))
    board_img = board.draw((Wpx, Hpx))
    if isinstance(board_img, tuple):
        board_img = board_img[1]
    if board_img.ndim == 3:
        board_img = cv2.cvtColor(board_img, cv2.COLOR_BGR2GRAY)
    board_img = cv2.copyMakeBorder(board_img, 0, 0, 0, 0, cv2.BORDER_CONSTANT)
    # 板面像素 -> 板面物理坐标（米）：X 向右、Y 向上
    px_to_metric = np.array([[squares_x * square_len / Wpx, 0.0, 0.0],
                             [0.0, -squares_y * square_len / Hpx, squares_y * square_len],
                             [0.0, 0.0, 1.0]], float)

    ok_all = True
    for dist, tilt in ((0.45, (205.0, 0.0, 20.0)), (0.32, (220.0, 10.0, -35.0)),
                       (0.60, (188.0, 0.0, 5.0))):
        R, t = pose_like(None, None, cam_dist=dist, tilt_deg=tilt)
        img = render_board_in_camera(board_img, px_to_metric, R, t)
        img = cv2.GaussianBlur(img, (3, 3), 0)
        det = D.detect_charuco(img, board, K, DIST, "4X4_50", min_corners=6)
        ok_all &= report("d=%.2fm tilt=%s" % (dist, tilt), det, R, t)
    return ok_all


def test_dots(rows=3, cols=3, missing=((0, 0),), spacing=0.04, dist=0.45,
              tilt=(150.0, 0.0, 35.0)):
    print("== 八点板（%dx%d 缺 %s）==" % (rows, cols, missing))
    cells, obj = D.dots_layout(rows, cols, spacing, missing)
    # 用真实尺寸画出“板面纹理”，再按单应贴到相机视角
    pad_px = 200
    px_per_m = 6000.0
    w_m = (cols + 1) * spacing
    h_m = (rows + 1) * spacing
    board_px = np.full((int(h_m * px_per_m), int(w_m * px_per_m)), 255, np.uint8)
    px_to_metric = np.array([[w_m / board_px.shape[1], 0.0, -w_m / 2.0],
                             [0.0, -h_m / board_px.shape[0], h_m / 2.0],
                             [0.0, 0.0, 1.0]], float)
    radius = int(round(0.008 * px_per_m))
    for (x, y, _z) in obj:
        u = int(round((x + w_m / 2.0) * px_per_m))
        v = int(round((h_m / 2.0 - y) * px_per_m))
        cv2.circle(board_px, (u, v), radius, 0, -1, cv2.LINE_AA)
    del pad_px

    R, t = pose_like(None, None, cam_dist=dist, tilt_deg=tilt)
    img = render_board_in_camera(board_px, px_to_metric, R, t)
    img = cv2.GaussianBlur(img, (5, 5), 0)
    det = D.detect_dots(img, rows, cols, spacing, K, DIST, missing=missing)
    return report("八点板", det, R, t)


def test_dots_robustness():
    print("== 八点板：旋转/缩放/缺检 鲁棒性 ==")
    rows, cols, missing, spacing = 3, 3, ((0, 0),), 0.04
    cells, obj = D.dots_layout(rows, cols, spacing, missing)
    w_m = (cols + 1) * spacing
    h_m = (rows + 1) * spacing
    px_per_m = 6000.0
    board_px = np.full((int(h_m * px_per_m), int(w_m * px_per_m)), 255, np.uint8)
    px_to_metric = np.array([[w_m / board_px.shape[1], 0.0, -w_m / 2.0],
                             [0.0, -h_m / board_px.shape[0], h_m / 2.0],
                             [0.0, 0.0, 1.0]], float)
    radius = int(round(0.008 * px_per_m))
    for (x, y, _z) in obj:
        u = int(round((x + w_m / 2.0) * px_per_m))
        v = int(round((h_m / 2.0 - y) * px_per_m))
        cv2.circle(board_px, (u, v), radius, 0, -1, cv2.LINE_AA)

    ok_all = True
    for yaw in (0.0, 37.0, 90.0, 180.0, 250.0):
        R, t = pose_like(None, None, cam_dist=0.42, tilt_deg=(215.0, 5.0, 25.0), yaw_deg=yaw)
        img = render_board_in_camera(board_px, px_to_metric, R, t)
        det = D.detect_dots(img, rows, cols, spacing, K, DIST, missing=missing)
        ok_all &= report("yaw=%.0f°" % yaw, det, R, t)

    # 遮挡一个点（模拟 7 点可见）
    R, t = pose_like(None, None, cam_dist=0.42, tilt_deg=(215.0, 5.0, 25.0), yaw_deg=10.0)
    img = render_board_in_camera(board_px, px_to_metric, R, t)
    det = D.detect_dots(img, rows, cols, spacing, K, DIST, missing=missing)
    if det.ok and det.points_2d is not None:
        u, v = det.points_2d[len(det.points_2d) // 2]
        cv2.circle(img, (int(u), int(v)), 30, 255, -1)
        det2 = D.detect_dots(img, rows, cols, spacing, K, DIST, missing=missing)
        ok_all &= report("遮挡1点", det2, R, t)
    return ok_all


def test_checker(cols=5, rows=3, square=0.02):
    print("== 棋盘格内角点 ==")
    px_per_m = 6000.0
    w_m = (cols + 1) * square
    h_m = (rows + 1) * square
    board_px = np.full((int(h_m * px_per_m), int(w_m * px_per_m)), 255, np.uint8)
    sq_px = int(round(square * px_per_m))
    for r in range(rows + 1):
        for c in range(cols + 1):
            if (r + c) % 2 == 0:
                cv2.rectangle(board_px, (c * sq_px, r * sq_px),
                              ((c + 1) * sq_px, (r + 1) * sq_px), 0, -1)
    px_to_metric = np.array([[w_m / board_px.shape[1], 0.0, -w_m / 2.0],
                             [0.0, -h_m / board_px.shape[0], h_m / 2.0],
                             [0.0, 0.0, 1.0]], float)

    # 棋盘格有 180° 自相似性：用上一帧结果做 hint，检查多帧是否保持同一约定
    hint = None
    prev = None
    ok_all = True
    for dist, tilt in ((0.45, (205.0, 0.0, 20.0)), (0.38, (220.0, -8.0, 40.0)),
                       (0.35, (200.0, 3.0, -12.0))):
        R, t = pose_like(None, None, cam_dist=dist, tilt_deg=tilt)
        img = render_board_in_camera(board_px, px_to_metric, R, t)
        det = D.detect_checker(img, cols, rows, square, K, DIST, hint=hint)
        if not det.ok:
            print("  [FAIL] 棋盘格: %s" % det.msg)
            ok_all = False
            continue
        R_est = det.rotation()
        ang = np.rad2deg(np.arccos(np.clip((np.trace(R.T @ R_est) - 1) / 2, -1, 1)))
        dt = np.linalg.norm(det.tvec - t)
        # 相对误差：若存在整体 180° 约定偏移，则相对量应保持一致
        rel = R.T @ R_est
        if prev is not None:
            d_rel = np.rad2deg(np.arccos(np.clip((np.trace(prev[0].T @ rel) - 1) / 2, -1, 1)))
            consistent = d_rel < 1.0
        else:
            consistent = True
        prev = (rel, dt)
        ok = consistent and det.reproj_err < 1.5 and dt < 0.003
        print("  [%s] 棋盘格 d=%.2fm: 点数=%d 相对GT旋转=%.2f° 平移误差=%.2fmm "
              "重投影=%.3fpx 多帧一致=%s"
              % ("PASS" if ok else "FAIL", dist, det.n_used, ang, dt * 1000.0,
                 det.reproj_err, consistent))
        ok_all &= ok
        hint = (det.rvec, det.tvec)
    return ok_all


def main():
    print("OpenCV %s" % cv2.__version__)
    print("字典数量: %d" % len(D.dictionary_ids()))
    results = [
        ("charuco", test_charuco()),
        ("dots", test_dots()),
        ("dots_robust", test_dots_robustness()),
        ("checker", test_checker()),
    ]
    print("\n== 汇总 ==")
    for name, ok in results:
        print("  %-12s %s" % (name, "PASS" if ok else "FAIL"))
    return 0 if all(ok for _, ok in results) else 1


if __name__ == "__main__":
    sys.exit(main())
