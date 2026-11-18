#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""内参文件的读写（YAML），格式与 board_pose_node 的 intrinsics_file 参数一致：

    camera_matrix: [fx, 0, cx, 0, fy, cy, 0, 0, 1]     # 也接受 3x3 嵌套
    dist_coeffs:   [k1, k2, p1, p2, k3]
    image_width:   640
    image_height:  480
    source: "factory"      # 可选，记录来源
"""

from __future__ import annotations

import os
from typing import Optional, Tuple

import numpy as np
import yaml


def load_intrinsics(path: str) -> Tuple[np.ndarray, np.ndarray]:
    """读取 K(3x3)、D(5,)。文件缺失或字段非法时抛 ValueError。"""
    if not os.path.isfile(path):
        raise ValueError("内参文件不存在: %s" % path)
    with open(path, "r", encoding="utf-8") as fh:
        data = yaml.safe_load(fh) or {}
    if "camera_matrix" not in data or "dist_coeffs" not in data:
        raise ValueError("内参文件需包含 camera_matrix 与 dist_coeffs: %s" % path)
    K = np.asarray(data["camera_matrix"], dtype=np.float64).reshape(3, 3)
    D = np.asarray(data["dist_coeffs"], dtype=np.float64).reshape(-1)
    if D.size < 4:
        raise ValueError("dist_coeffs 至少需要 4 个元素，当前 %d" % D.size)
    return K, D


def save_intrinsics(path: str, K: np.ndarray, D: np.ndarray,
                    image_size: Optional[Tuple[int, int]] = None,
                    source: str = "", extra: Optional[dict] = None) -> None:
    K = np.asarray(K, dtype=np.float64).reshape(3, 3)
    D = np.asarray(D, dtype=np.float64).reshape(-1)
    data = {
        "camera_matrix": [float(v) for v in K.reshape(-1)],
        "dist_coeffs": [float(v) for v in D],
        "model": "pinhole",
    }
    if image_size is not None:
        data["image_width"], data["image_height"] = int(image_size[0]), int(image_size[1])
    if source:
        data["source"] = source
    if extra:
        data.update(extra)
    d = os.path.dirname(os.path.abspath(path))
    if d:
        os.makedirs(d, exist_ok=True)
    with open(path, "w", encoding="utf-8") as fh:
        yaml.safe_dump(data, fh, allow_unicode=True, sort_keys=False, default_flow_style=None)
