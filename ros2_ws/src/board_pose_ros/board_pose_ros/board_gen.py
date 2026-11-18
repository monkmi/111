#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""生成可打印的标定板图片（PNG）与参数文件（JSON）。

用法：
    # ChArUco: 5x7 方格, 方格 30mm, 内码 22mm
    ros2 run board_pose_ros board_gen charuco --squares-x 5 --squares-y 7 \\
        --square-length 0.03 --marker-length 0.022 --out-dir /tmp/boards

    # 八点板: 3x3 网格缺左上角, 间距 40mm
    ros2 run board_pose_ros board_gen dots --rows 3 --cols 3 --spacing 0.04 \\
        --missing "0,0" --out-dir /tmp/boards

    # 棋盘格: 5x3 内角点, 方格 20mm
    ros2 run board_pose_ros board_gen checker --cols 5 --rows 3 \\
        --square-length 0.02 --out-dir /tmp/boards

打印要点：用 100% 比例（“实际大小 / Actual size”，不要“适应页面”），
打印后用游标卡尺量一下图上 100mm 标尺，误差应 < 0.5mm。
"""

from __future__ import annotations

import argparse
import json
import os
from typing import List, Tuple

import cv2
import numpy as np

DEFAULT_DPI = 300.0
MARGIN_MM = 18.0
RULER_MM = 100.0


def _mm_to_px(mm: float, dpi: float) -> int:
    return int(round(mm / 25.4 * dpi))


def _new_canvas(width_mm: float, height_mm: float, dpi: float) -> np.ndarray:
    w = _mm_to_px(MARGIN_MM * 2 + width_mm, dpi)
    h = _mm_to_px(MARGIN_MM * 2 + height_mm + 14.0, dpi)  # 底部留标注区
    return np.full((h, w), 255, np.uint8)


def _paste(canvas: np.ndarray, board: np.ndarray, dpi: float) -> np.ndarray:
    x0 = _mm_to_px(MARGIN_MM, dpi)
    y0 = _mm_to_px(MARGIN_MM, dpi)
    out = canvas.copy()
    out[y0:y0 + board.shape[0], x0:x0 + board.shape[1]] = board
    return out


def _decorate(canvas: np.ndarray, dpi: float, title: str, subtitle: str) -> np.ndarray:
    """在底部空白区画 100mm 标尺与说明文字，便于检查打印比例。"""
    h, w = canvas.shape[:2]
    out = canvas.copy()
    x0 = _mm_to_px(MARGIN_MM, dpi)
    y_ruler = _mm_to_px(MARGIN_MM, dpi) + 0  # 由调用方先 paste 再调用
    y_ruler = h - _mm_to_px(11.0, dpi)
    x1 = x0 + _mm_to_px(RULER_MM, dpi)
    cv2.line(out, (x0, y_ruler), (x1, y_ruler), 0, max(2, int(dpi / 100)))
    for x in (x0, x1):
        cv2.line(out, (x, y_ruler - _mm_to_px(2, dpi)), (x, y_ruler + _mm_to_px(2, dpi)),
                 0, max(2, int(dpi / 180)))
    font = cv2.FONT_HERSHEY_SIMPLEX
    scale = dpi / 300.0
    cv2.putText(out, "%s" % title, (x0, _mm_to_px(9.0, dpi)), font, 0.55 * scale, 0, 1,
                cv2.LINE_AA)
    cv2.putText(out, "%s  |  ruler = %.0f mm  |  print at 100%%" % (subtitle, RULER_MM),
                (x0, _mm_to_px(13.5, dpi)), font, 0.42 * scale, 0, 1, cv2.LINE_AA)
    return out


def gen_charuco(args) -> Tuple[np.ndarray, dict]:
    from board_pose_ros import detectors as D

    board = D.build_charuco_board(args.dictionary, args.squares_x, args.squares_y,
                                  args.square_length, args.marker_length)
    w_mm = args.squares_x * args.square_length * 1000.0
    h_mm = args.squares_y * args.square_length * 1000.0
    w_px = _mm_to_px(w_mm, args.dpi)
    h_px = _mm_to_px(h_mm, args.dpi)
    img = np.asarray(board.draw((w_px, h_px)))
    canvas = _paste(_new_canvas(w_mm, h_mm, args.dpi), img, args.dpi)
    title = "ChArUco %dx%d  square=%.1fmm  marker=%.1fmm  %s" % (
        args.squares_x, args.squares_y, args.square_length * 1000.0,
        args.marker_length * 1000.0, args.dictionary)
    canvas = _decorate(canvas, args.dpi, title, "board_pose_ros board_gen")
    params = {
        "board_type": "charuco",
        "dictionary": args.dictionary,
        "squares_x": args.squares_x,
        "squares_y": args.squares_y,
        "square_length": args.square_length,
        "marker_length": args.marker_length,
        "board_size_m": [args.squares_x * args.square_length,
                         args.squares_y * args.square_length],
        "n_markers": int(len(np.asarray(board.ids).ravel())),
        "dpi": args.dpi,
        "print_scale": "100%",
        "note": "square_length 为方格边长，marker_length 为内部 ArUco 码边长（同一单位：米）",
    }
    return canvas, params


def gen_dots(args) -> Tuple[np.ndarray, dict]:
    from board_pose_ros import detectors as D

    cells, obj = D.dots_layout(args.rows, args.cols, args.spacing, args.missing)
    w_mm = (args.cols + 1) * args.spacing * 1000.0
    h_mm = (args.rows + 1) * args.spacing * 1000.0
    w_px = _mm_to_px(w_mm, args.dpi)
    h_px = _mm_to_px(h_mm, args.dpi)
    img = np.full((h_px, w_px), 255, np.uint8)
    r_px = int(round(args.radius * 1000.0 / 25.4 * args.dpi))
    for (x, y, _z) in obj:
        u = int(round((x + w_mm / 2000.0) * 1000.0 / 25.4 * args.dpi))
        v = int(round((h_mm / 2000.0 - y) * 1000.0 / 25.4 * args.dpi))
        cv2.circle(img, (u, v), r_px, 0, -1, cv2.LINE_AA)
    # 板框（打印后方便找边、便于目视检查是否被裁掉）
    cv2.rectangle(img, (2, 2), (w_px - 3, h_px - 3), 0, max(1, int(args.dpi / 300)))
    canvas = _paste(_new_canvas(w_mm, h_mm, args.dpi), img, args.dpi)
    title = "dots %dx%d  spacing=%.1fmm  r=%.1fmm  missing=%s" % (
        args.rows, args.cols, args.spacing * 1000.0, args.radius * 1000.0,
        args.missing or "none")
    canvas = _decorate(canvas, args.dpi, title, "board_pose_ros board_gen")
    params = {
        "board_type": "dots",
        "dot_rows": args.rows,
        "dot_cols": args.cols,
        "dot_spacing": args.spacing,
        "dot_missing": args.missing or "",
        "dot_radius": args.radius,
        "n_dots": len(cells),
        "board_size_m": [(args.cols + 1) * args.spacing, (args.rows + 1) * args.spacing],
        "dark_on_light": True,
        "dpi": args.dpi,
        "print_scale": "100%",
        "note": "dot_spacing 为相邻圆点中心距；dot_missing 为不打印的格位，如 \"0,0;2,2\"（行,列）",
    }
    return canvas, params


def gen_checker(args) -> Tuple[np.ndarray, dict]:
    w_mm = (args.cols + 1) * args.square_length * 1000.0
    h_mm = (args.rows + 1) * args.square_length * 1000.0
    w_px = _mm_to_px(w_mm, args.dpi)
    h_px = _mm_to_px(h_mm, args.dpi)
    img = np.full((h_px, w_px), 255, np.uint8)
    sq_px = _mm_to_px(args.square_length * 1000.0, args.dpi)
    for r in range(args.rows + 1):
        for c in range(args.cols + 1):
            if (r + c) % 2 == 0:
                cv2.rectangle(img, (c * sq_px, r * sq_px),
                              ((c + 1) * sq_px, (r + 1) * sq_px), 0, -1)
    canvas = _paste(_new_canvas(w_mm, h_mm, args.dpi), img, args.dpi)
    title = "checker %dx%d inner corners  square=%.1fmm" % (
        args.cols, args.rows, args.square_length * 1000.0)
    canvas = _decorate(canvas, args.dpi, title, "board_pose_ros board_gen")
    params = {
        "board_type": "checker",
        "checker_cols": args.cols,
        "checker_rows": args.rows,
        "checker_square_length": args.square_length,
        "board_size_m": [(args.cols + 1) * args.square_length,
                         (args.rows + 1) * args.square_length],
        "dpi": args.dpi,
        "print_scale": "100%",
        "note": "OpenCV 要求内角点 cols>2 且 rows>2；棋盘格图案有 180° 自相似性",
    }
    return canvas, params


def main(argv: List[str] | None = None) -> None:
    ap = argparse.ArgumentParser(description="生成可打印标定板 (PNG + JSON)")
    sub = ap.add_subparsers(dest="cmd", required=True)

    def common(p):
        p.add_argument("--dpi", type=float, default=DEFAULT_DPI)
        p.add_argument("--out-dir", default="./boards")
        p.add_argument("--name", default=None)

    c = sub.add_parser("charuco", help="ChArUco 板")
    common(c)
    c.add_argument("--dictionary", default="DICT_4X4_50")
    c.add_argument("--squares-x", type=int, default=5)
    c.add_argument("--squares-y", type=int, default=7)
    c.add_argument("--square-length", type=float, default=0.03)
    c.add_argument("--marker-length", type=float, default=0.022)

    d = sub.add_parser("dots", help="圆点阵列（八点板）")
    common(d)
    d.add_argument("--rows", type=int, default=3)
    d.add_argument("--cols", type=int, default=3)
    d.add_argument("--spacing", type=float, default=0.04)
    d.add_argument("--radius", type=float, default=0.008)
    d.add_argument("--missing", default="0,0", help='缺格 "row,col"，多格用分号分隔')

    k = sub.add_parser("checker", help="棋盘格")
    common(k)
    k.add_argument("--cols", type=int, default=5)
    k.add_argument("--rows", type=int, default=3)
    k.add_argument("--square-length", type=float, default=0.02)

    args = ap.parse_args(argv)
    if args.cmd == "charuco":
        img, params = gen_charuco(args)
    elif args.cmd == "dots":
        img, params = gen_dots(args)
    else:
        img, params = gen_checker(args)

    os.makedirs(args.out_dir, exist_ok=True)
    name = args.name or ("%s_%s" % (args.cmd, "board"))
    png = os.path.join(args.out_dir, name + ".png")
    js = os.path.join(args.out_dir, name + ".json")
    cv2.imwrite(png, img)
    with open(js, "w", encoding="utf-8") as fh:
        json.dump(params, fh, ensure_ascii=False, indent=2)

    print("已生成:")
    print("  图片: %s  (%dx%d px, %.0f dpi)" % (png, img.shape[1], img.shape[0], args.dpi))
    print("  参数: %s" % js)
    print("打印: 选择 100%（实际大小），打印后量一下图上的 100mm 标尺。")
    print("板尺寸: %.1f x %.1f mm" % tuple(v * 1000.0 for v in params["board_size_m"]))


if __name__ == "__main__":
    main()
