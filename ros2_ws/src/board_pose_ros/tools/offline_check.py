#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""兼容入口：包内实现见 board_pose_ros/self_check.py。"""

import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

from board_pose_ros.self_check import main  # noqa: E402

if __name__ == "__main__":
    sys.exit(main())
