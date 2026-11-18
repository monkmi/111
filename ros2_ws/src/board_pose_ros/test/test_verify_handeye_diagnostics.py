"""超时诊断应指出未识别的板，而不是将其误报为所有输入缺失。"""
from collections import Counter
from types import SimpleNamespace
import unittest

from board_pose_ros.verify_handeye import input_diagnostics


class DiagnosticsTests(unittest.TestCase):
    def setUp(self):
        self.args = SimpleNamespace(frames_per_pose=5, image_topic="/image",
                                    pose_topic="/board", tcp_topic="/tcp")

    def test_board_detection_failure_with_other_inputs_present(self):
        message = input_diagnostics(
            self.args, Counter(image=60, tcp=120), {"image": 1, "board": 1, "tcp": 1},
            {"detected": False, "msg": "未检测到 ArUco marker", "n_points": 0})
        self.assertIn("图像和 TCP 已到达", message)
        self.assertIn("未检测到 ArUco marker", message)
        self.assertIn("/board: 收到 0 条，发布者 1 个", message)
        self.assertNotIn("没有发布者", message)

    def test_missing_publishers_and_bad_timestamp(self):
        message = input_diagnostics(
            self.args, Counter(board=5), {"image": 0, "board": 1, "tcp": 0},
            reason="板位姿时间戳为零", board_age=123.0)
        self.assertIn("/image: 收到 0 条，发布者 0 个", message)
        self.assertIn("ROS_DOMAIN_ID", message)
        self.assertIn("时间戳为零", message)
        self.assertIn("+123.000 秒", message)


if __name__ == "__main__":
    unittest.main()
