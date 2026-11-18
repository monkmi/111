#!/usr/bin/env python3
"""源码直接运行入口，不需要先 colcon build。"""
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from board_pose_ros.verify_handeye import main  # noqa: E402

if __name__ == "__main__":
    sys.exit(main())
