# nero-official-calibrate

AgileX **nero** 机械臂 + RealSense 相机的 ROS 2 手眼标定工作空间。

* **标定流程与配置说明**：[`docs/handeye_calibration_nero.md`](docs/handeye_calibration_nero.md)
* **可打印标定板**：`docs/boards/charuco_board.png`、`docs/boards/dots_board.png`（100% 打印）
* **识别/生成/内参/验证工具**：`ros2_ws/src/board_pose_ros`（见其
  [`README.md`](ros2_ws/src/board_pose_ros/README.md)）
* **手内/手外多姿态拍照精度验证**：`python3 ros2_ws/src/board_pose_ros/tools/verify_handeye.py --mode eye_in_hand`
  （手外用 `eye_to_hand`；默认读取 `ros2_ws/result` 最新标定文件，按 Enter 采样、`q` 保存误差报告）

## 目录

```
agx_arm_ros/     # 官方驱动仓库（已初始化 submodule）
ros2_ws/         # 唯一工作空间：agx_arm_ros（软链）+ handeye_calibration_ros + board_pose_ros
docs/            # 说明文档与标定板图片
```

## 获取第三方依赖

本仓库只包含自研代码（`board_pose_ros`）与文档，两个上游包未入库，克隆后需自行拉取：

```bash
cd ros2_ws/src
git clone -b ros2   https://github.com/agilexrobotics/agx_arm_ros.git
git clone -b humble https://github.com/agilexrobotics/handeye_calibration_ros.git
git -C agx_arm_ros submodule update --init --recursive   # agx_arm_urdf
```

构建产物（`build/`、`install/`、`log/`）与标定/验证结果（`result/`）已在 `.gitignore` 中排除。

## 最短路径

```bash
cd ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash

ros2 run board_pose_ros board_self_check        # 离线自检，全 PASS 说明环境正常
```

之后按 `docs/handeye_calibration_nero.md` 的「一页流程」逐步执行。

## 两个常见疑问

* **CAN 上有两台机械臂、只上电一台**：正常。未上电的臂不参与通信；两台同时在同一 CAN 模块上
  通信会因相同 CAN ID 冲突而出问题，官方要求按主从臂规则指定 `fw_version`。标定只需一台臂。
* **为什么不用 `aruco_ros`**：它只支持单个 ArUco 码，不支持 ChArUco 与八点板；`board_pose_ros`
  发布与它同名的话题 `/aruco_single/pose`，可直接替代。
