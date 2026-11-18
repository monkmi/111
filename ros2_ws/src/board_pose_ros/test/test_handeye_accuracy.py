"""使用已知真值和错误外参，验证两个模式的方向、误差和姿态均值。"""
import unittest

import cv2
import numpy as np

from board_pose_ros.handeye_accuracy import (
    evaluate, fixed_board_pose, matrix_pose, mean_pose, pose_matrix,
    rotation_error_deg,
)


def transform(position, rotvec):
    T = np.eye(4)
    T[:3, :3] = cv2.Rodrigues(np.asarray(rotvec, dtype=float))[0]
    T[:3, 3] = position
    return T


class HandeyeAccuracyTests(unittest.TestCase):
    def setUp(self):
        self.X = transform([0.08, -0.04, 0.2], [0.2, -0.4, 0.1])
        self.fixed = transform([0.4, -0.3, 0.8], [0.1, 0.2, -0.3])
        rng = np.random.default_rng(42)
        self.arm = [transform(rng.uniform(-0.4, 0.4, 3), rng.uniform(-1, 1, 3))
                    for _ in range(12)]

    def samples(self, mode):
        samples = []
        for i, A in enumerate(self.arm):
            B = (np.linalg.inv(self.X) @ np.linalg.inv(A) @ self.fixed
                 if mode == "eye_in_hand" else np.linalg.inv(self.X) @ A @ self.fixed)
            frame = {"base_tcp": matrix_pose(A), "cam_board": matrix_pose(B)}
            samples.append({"id": i + 1, "frames": [frame] * (i % 5 + 1)})
        return samples

    def test_known_truth_both_modes_and_bad_calibration(self):
        for mode in ("eye_in_hand", "eye_to_hand"):
            with self.subTest(mode=mode):
                samples = self.samples(mode)
                report = evaluate(mode, self.X, samples)
                self.assertLess(report["translation_mm"]["max"], 1e-9)
                self.assertLess(report["rotation_deg"]["max"], 1e-10)
                np.testing.assert_allclose(pose_matrix(**report["reference_pose"]), self.fixed,
                                           atol=1e-14)
                wrong_X = self.X @ transform([0.03, 0, 0], [0.08, 0, 0])
                wrong = evaluate(mode, wrong_X, samples)
                self.assertGreater(wrong["translation_mm"]["mean"], 5)
                self.assertGreater(wrong["rotation_deg"]["mean"], 1)
                inverse = evaluate(mode, np.linalg.inv(self.X), samples)
                self.assertGreater(inverse["translation_mm"]["mean"], 50)

    def test_known_error_and_equal_pose_weight(self):
        # 两个姿态位置相差 10mm、旋转相差 2°，到均值各为 5mm / 1°。
        samples = []
        for i, repetitions in enumerate([1, 9]):
            B = transform([i * 0.01, 0, 0], [0, 0, np.deg2rad(i * 2)])
            f = {"base_tcp": matrix_pose(np.eye(4)), "cam_board": matrix_pose(B)}
            samples.append({"id": i, "frames": [f] * repetitions})
        r = evaluate("eye_in_hand", np.eye(4), samples)
        self.assertAlmostEqual(r["translation_mm"]["mean"], 5, places=10)
        self.assertAlmostEqual(r["translation_mm"]["rms"], 5, places=10)
        self.assertAlmostEqual(r["rotation_deg"]["mean"], 1, places=10)
        self.assertTrue(r["warnings"])

    def test_rotation_wrap_and_quaternion_validation(self):
        poses = [transform([0, 0, 0], [0, 0, np.deg2rad(a)]) for a in [179, -179]]
        mean = mean_pose(poses)
        self.assertAlmostEqual(rotation_error_deg(mean[:3, :3], poses[0][:3, :3]), 1)
        self.assertAlmostEqual(np.linalg.det(mean[:3, :3]), 1)
        with self.assertRaises(ValueError):
            pose_matrix([0, 0, 0], [0, 0, 0, 0])
        with self.assertRaises(ValueError):
            pose_matrix([float("nan"), 0, 0], [0, 0, 0, 1])
        with self.assertRaises(ValueError):
            evaluate("eye_in_hand", self.X, [])
        with self.assertRaises(ValueError):
            fixed_board_pose("bad_mode", self.X, self.X, self.X)


if __name__ == "__main__":
    unittest.main()
