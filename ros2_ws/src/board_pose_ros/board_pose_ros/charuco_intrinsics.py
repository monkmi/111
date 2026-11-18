#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""离线相机内参标定（ChArUco 板或棋盘格），输出 board_pose_node 可直接用的 YAML。

示例：
    # 1) 采图：把板子摆到画面各处（近/远、左/右、边角、倾斜），保存 20~40 张到 ./intr_imgs
    # 2) 标定
    ros2 run board_pose_ros charuco_calibrate --images ./intr_imgs \\
        --out ./config/realsense_d435_intrinsics.yaml

    # 3) 使用自己测的内参跑识别节点
    ros2 run board_pose_ros board_pose_node --ros-args \\
        -p intrinsics_file:=$PWD/config/realsense_d435_intrinsics.yaml

要点：同一组图必须同一分辨率；板子要铺满画面不同区域；不要只拍正面，至少几张明显倾斜。
"""

from __future__ import annotations

import argparse
import glob
import os
import sys
from typing import List, Tuple

import cv2
import numpy as np

from board_pose_ros import detectors as D
from board_pose_ros.io_utils import save_intrinsics

IMG_EXTS = ("*.png", "*.PNG", "*.jpg", "*.JPG", "*.jpeg", "*.JPEG", "*.bmp", "*.BMP")


def collect_images(path: str) -> List[str]:
    if os.path.isfile(path):
        return [path]
    files: List[str] = []
    for ext in IMG_EXTS:
        files.extend(glob.glob(os.path.join(path, ext)))
    return sorted(set(files))


def calibrate_charuco(images, board, dictionary, min_views, image_size):
    all_corners, all_ids = [], []
    for path in images:
        gray = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
        if gray is None:
            print("  [skip] 读不出图: %s" % path)
            continue
        corn, ids = _charuco_corners(gray, board, dictionary)
        if corn is None or len(corn) < 6:
            print("  [skip] 内角点不足: %s" % os.path.basename(path))
            continue
        all_corners.append(corn)
        all_ids.append(ids)
        print("  [ok] %-40s 角点=%d" % (os.path.basename(path), len(corn)))
    if len(all_corners) < min_views:
        raise SystemExit("有效图像只有 %d 张（至少需要 %d 张），请补采"
                         % (len(all_corners), min_views))
    ret, K, Dv, rvecs, tvecs = cv2.aruco.calibrateCameraCharuco(
        all_corners, all_ids, board, image_size, None, None)
    return ret, K, Dv, all_corners, all_ids


def _charuco_corners(gray, board, dictionary):
    corners, ids, _ = cv2.aruco.detectMarkers(gray, D.get_dictionary(dictionary))
    if ids is None or len(ids) < 2:
        return None, None
    ret, cc, ci = cv2.aruco.interpolateCornersCharuco(corners, ids, gray, board)
    if cc is None or ci is None:
        return None, None
    return cc, ci


def calibrate_checker(images, cols, rows, square_length, min_views, image_size):
    obj = D.checker_object_points(cols, rows, square_length).reshape(-1, 1, 3)
    obj_points, img_points = [], []
    for path in images:
        gray = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
        if gray is None:
            print("  [skip] 读不出图: %s" % path)
            continue
        det = D.detect_checker(gray, cols, rows, square_length, np.eye(3), np.zeros(5))
        if not det.ok:
            print("  [skip] %-40s %s" % (os.path.basename(path), det.msg))
            continue
        obj_points.append(obj)
        img_points.append(det.points_2d.reshape(-1, 1, 2).astype(np.float64))
        print("  [ok] %-40s 角点=%d" % (os.path.basename(path), det.n_used))
    if len(obj_points) < min_views:
        raise SystemExit("有效图像只有 %d 张（至少需要 %d 张），请补采"
                         % (len(obj_points), min_views))
    ret, K, Dv, rvecs, tvecs = cv2.calibrateCamera(obj_points, img_points, image_size,
                                                   None, None)
    return ret, K, Dv, obj_points, img_points


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="离线相机内参标定（ChArUco / 棋盘格）")
    ap.add_argument("--images", required=True, help="图片目录或单张图片")
    ap.add_argument("--out", default="camera_intrinsics.yaml", help="输出 YAML 路径")
    ap.add_argument("--pattern", default="charuco", choices=("charuco", "checker"))
    ap.add_argument("--min-views", type=int, default=10)
    ap.add_argument("--image-size", default="", help="覆盖图像尺寸，如 640x480（默认取首图）")
    # charuco
    ap.add_argument("--dictionary", default="DICT_4X4_50")
    ap.add_argument("--squares-x", type=int, default=5)
    ap.add_argument("--squares-y", type=int, default=7)
    ap.add_argument("--square-length", type=float, default=0.03)
    ap.add_argument("--marker-length", type=float, default=0.022)
    # checker
    ap.add_argument("--checker-cols", type=int, default=5)
    ap.add_argument("--checker-rows", type=int, default=3)
    ap.add_argument("--checker-square-length", type=float, default=0.02)
    args = ap.parse_args(argv)

    images = collect_images(args.images)
    if not images:
        print("未找到图片: %s" % args.images)
        return 2
    first = cv2.imread(images[0], cv2.IMREAD_GRAYSCALE)
    if first is None:
        print("读不出首张图片: %s" % images[0])
        return 2
    if args.image_size:
        w, h = args.image_size.lower().split("x")
        image_size = (int(w), int(h))
    else:
        image_size = (first.shape[1], first.shape[0])

    print("共 %d 张图片, 分辨率 %dx%d, 模式=%s"
          % (len(images), image_size[0], image_size[1], args.pattern))

    if args.pattern == "charuco":
        board = D.build_charuco_board(args.dictionary, args.squares_x, args.squares_y,
                                      args.square_length, args.marker_length)
        rms, K, Dv, _, _ = calibrate_charuco(images, board, args.dictionary,
                                             args.min_views, image_size)
        extra = {"board_type": "charuco", "dictionary": args.dictionary,
                 "squares_x": args.squares_x, "squares_y": args.squares_y,
                 "square_length": args.square_length,
                 "marker_length": args.marker_length}
    else:
        rms, K, Dv, _, _ = calibrate_checker(images, args.checker_cols, args.checker_rows,
                                             args.checker_square_length,
                                             args.min_views, image_size)
        extra = {"board_type": "checker", "checker_cols": args.checker_cols,
                 "checker_rows": args.checker_rows,
                 "checker_square_length": args.checker_square_length}

    Dv = np.asarray(Dv, dtype=np.float64).reshape(-1)
    print("\n标定完成: 重投影 RMS = %.4f px" % rms)
    print("  fx=%.3f fy=%.3f cx=%.3f cy=%.3f" % (K[0, 0], K[1, 1], K[0, 2], K[1, 2]))
    print("  D=%s" % np.round(Dv, 6).tolist())
    if rms > 1.0:
        print("  [!] RMS > 1.0 px，内参可能偏大/偏小或板子不平：检查打印比例、"
              "是否有模糊/运动伪影、角点是否铺满画面。")

    save_intrinsics(args.out, K, Dv, image_size=image_size,
                    source="charuco_calibrate (%s, %d views, rms=%.4f px)"
                           % (args.pattern, len(images), rms),
                    extra=extra)
    print("\n已写入: %s" % args.out)
    print("启动识别节点时使用: ros2 run board_pose_ros board_pose_node --ros-args "
          "-p intrinsics_file:=%s" % os.path.abspath(args.out))
    return 0


if __name__ == "__main__":
    sys.exit(main())
