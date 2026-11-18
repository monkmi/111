# board_pose_ros

标定板（ChArUco / 圆点阵列“八点板” / 棋盘格）位姿识别与手眼标定辅助工具包。

发布的内容刻意与 `aruco_ros` 的 `/aruco_single/pose` 对齐，因此可以直接替换
`aruco_ros` 给 `handeye_calibration_ros` 的 `agx_arm_calibration` 节点供数。

## 安装

```bash
cd ~/workspace/nero-official-calibrate/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select board_pose_ros
source install/setup.bash
```

依赖：`ros-humble-cv-bridge`、`ros-humble-tf2-ros`、`python3-opencv`、`python3-numpy`、
`python3-yaml`（Humble 自带的 OpenCV 4.5.4 即可，不使用 4.7 才有的 `CharucoDetector`）。

## 可执行文件

| 命令 | 作用 |
| --- | --- |
| `ros2 run board_pose_ros board_pose_node` | 订阅图像 + 相机信息，输出标定板位姿、TF、调试图、状态 JSON |
| `ros2 run board_pose_ros board_gen {charuco,dots,checker}` | 生成可 1:1 打印的标定板 PNG（附 JSON 说明） |
| `ros2 run board_pose_ros charuco_calibrate` | 用一批图片离线标定相机内参，输出 YAML |
| `ros2 run board_pose_ros verify_board` | 验证外参：检查静止标定板在 `base_link` 下是否恒定 |
| `ros2 run board_pose_ros verify_handeye` | 手内/手外多姿态拍照验证，输出平均误差、RMS、最大误差、照片和 JSON/CSV |
| `ros2 run board_pose_ros board_self_check` | 纯离线自检（合成图像跑三种板型），不依赖 ROS 图 |

## 快速使用

```bash
# 1. 生成 5x7 ChArUco 板（30 mm 方格 / 22 mm marker，A4，100% 打印）
ros2 run board_pose_ros board_gen charuco --dpi 300 --out-dir ./boards

# 2. 启动识别节点
ros2 run board_pose_ros board_pose_node --ros-args \
  -p board_type:=charuco \
  -p image_topic:=/camera/camera/color/image_raw \
  -p camera_info_topic:=/camera/camera/color/camera_info

# 或使用 launch + 配置文件
ros2 launch board_pose_ros board_pose.launch.py board_type:=charuco

# 3. 查看结果
ros2 topic echo /aruco_single/pose
ros2 topic echo /board_pose/status
```

## 发布

| 名称 | 类型 | 说明 |
| --- | --- | --- |
| `/aruco_single/pose` | `geometry_msgs/PoseStamped` | 标定板在相机坐标系下的位姿（板系 -> 相机系） |
| `/board_pose/status` | `std_msgs/String` | JSON 状态：检出与否、点数、重投影误差、位姿、内参来源等 |
| `/board_pose/debug` | `sensor_msgs/Image` | 调试图：检测点、坐标轴、文字 |
| TF `${image_frame} -> aruco_marker_frame` | `tf2` | 板位姿 TF，可在 rviz 中查看 |

`board_type` 三种取值：`charuco`、`dots`（八点板/圆点阵列）、`checker`（棋盘格内角点）。

## 参数

完整参数见 `config/*.yaml` 与主文档附录；节点启动时会打印实际生效的板参数。

常用参数：

* `board_type`：`charuco` | `dots` | `checker`
* `intrinsics_file`：YAML（`camera_matrix` + `dist_coeffs`），设置后优先于 `camera_info` 话题
* `output_frame`：把板位姿变换到该坐标系（例如 `camera_link`）；留空则用图像自身坐标系
* `max_reproj_err`：重投影误差阈值（像素），超过则丢弃该帧
* `process_rate_hz`：处理频率上限，`0` 表示每帧都处理

## 自检

```bash
ros2 run board_pose_ros board_self_check
```

用合成图像分别渲染 ChArUco / 圆点板 / 棋盘格，检查位姿误差是否在阈值内
（角度 < 0.6°、平移 < 2 mm、重投影 < 1.5 px）。

主文档：`docs/handeye_calibration_nero.md`（仓库根目录下）。

## 手内/手外多姿态精度验证

### 本机手内相机（已按两路实拍画面确认）

* 末端相机：D435 **262522072403**。
* 外部相机：D435 **263022078163**。

手内采集必须固定末端相机序列号；只使用默认话题名不能区分两台物理相机。
停止旧相机和板识别节点后，下面一条命令同时启动正确相机与 7×9、24/15 mm 实物板识别，
从任何目录运行均可，不需要另外启动 `board_pose_node`：

```bash
ros2 launch board_pose_ros eye_in_hand_camera.launch.py
```

该入口固定相机 `262522072403`，输出 `/camera/camera/color/image_raw`，板位姿输出
`/aruco_single/pose`；验证工具默认话题与其一致。启动日志应显示所选序列号、7×9 格、
方格 `0.024`、码 `0.015`、`DICT_5X5_250`。采集工具不控制机械臂运动。

2026-11-18 13:45 那批验证照片来自外部相机，不能用其误差评价手内外参；数据保留供追溯。
旧标定 JSON 未记录相机身份，不能仅凭该文件判断原标定用了哪台相机。

先启动机械臂、相机和 `board_pose_node`（与标定时使用完全相同的板参数、内参、
`tcp_offset`、图像分辨率和 `output_frame`），再运行：

```bash
# 从仓库根目录直接运行，不需要重新编译
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
python3 ros2_ws/src/board_pose_ros/tools/verify_handeye.py \
  --mode eye_in_hand \
  --calibration ros2_ws/result/2026-11-18_11-39-38_calibration.json

# 手外：相机固定、板刚性固定在末端，改为 --mode eye_to_hand
# 编译包后也可使用 ros2 run board_pose_ros verify_handeye --mode eye_to_hand
```

结果文件不包含模式，必须根据这次标定的实际安装方式指定 `--mode`。
不传 `--calibration` 时，从 `--output-dir`（默认 `ros2_ws/result`）读取文件名最新的
`*_calibration.json`，启动时会显示实际使用文件。**不要把手内结果用于手外模式。**

手动移动到 10~20 个不同姿态，每次停稳 1~2 秒，按 **Enter** 采一组（默认 5 帧）；
绕不同轴旋转并覆盖实际工作区。`d` 撤销上一组，`r` 查看当前平均误差，`q` 输出报告退出；
Ctrl+C/输入结束也会保存已完成组。工具只订阅数据，由用户操作机械臂移动。

计算方法（`T_a_b` 把 b 系坐标变换到 a 系）：

* 手内：`T_base_board = T_base_tcp @ T_tcp_cam @ T_cam_board`，桌面上的板不动。
* 手外：`T_tcp_board = inv(T_base_tcp) @ T_base_cam @ T_cam_board`，板与 TCP 固连。
* 每帧先变换，再对每个姿态取均值；所有姿态等权，平移取算术均值，旋转取 SO(3) SVD 均值。
  每姿态相对总体均值的位置欧氏距离（mm）和旋转夹角（°）用于统计平均值、RMS、最大值、P95。
* 这些是**多姿态一致性误差**，包含视觉、机械臂、同步等误差；不能代替带外部真值的绝对精度验证。
  报告另含组内抖动，可区分静止检测波动和姿态之间的偏差。样本数/运动覆盖不足会提示。

工具使用检测成功的 `/aruco_single/pose`，寻找时间戳完全相同的原始图像，并匹配最近 TCP
（默认容差 50 ms）；要求过去 0.6 秒 TCP 变化不超过 1 mm / 0.3°，并检查同一组采集期间的运动。
过期、零时间戳、错误坐标系和不同步数据不计入。采样失败时不会混入报告。
无需发布标定静态 TF，也无需机械臂 TF 链。

常用参数：

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `--camera-frame` | `camera_color_optical_frame` | 与标定时板位姿坐标系一致；例如使用 `camera_link` 时显式修改 |
| `--tcp-topic` | `/feedback/tcp_pose` | 输入必须是基座下的 TCP 位姿；驱动 frame_id 为空时仍按此约定使用 |
| `--pose-topic` | `/aruco_single/pose` | 标定板位姿 |
| `--status-topic` | `/board_pose/status` | 识别状态，用于采样超时诊断 |
| `--image-topic` | `/camera/camera/color/image_raw` | 原始图像，须与识别节点输入一致 |
| `--camera-info-topic` | `/camera/camera/color/camera_info` | 记录相机出厂内参；自测内参应在识别节点指定 |
| `--frames-per-pose` | `5` | 每个姿态的有效帧数 |
| `--min-poses` | `5` | 少于此数量提示；至少两个姿态才能输出报告 |
| `--sync-tolerance` | `0.05` | 图像与 TCP 最大时间差，秒 |
| `--settle-time` | `0.6` | 停稳检查窗口，秒 |
| `--stable-mm` / `--stable-deg` | `1.0` / `0.3` | TCP 稳定性阈值 |
| `--capture-timeout` | `15` | 每组最长等待秒数，超时重新采样 |
| `--max-age` | `1.0` | 消息允许的最大延迟，秒 |
| `--output-dir` | `ros2_ws/result` | 结果根目录，与当前工作目录无关（源码安装时） |

结果目录 `ros2_ws/result/<北京时间>_<mode>_verification/` 包含：

* `images/`：每帧原始照片（撤销或失败组的照片可能保留，但不计入报告）。
* `samples.json`：外参快照、模式、采样配置、图像/TCP 时间戳、原始 TCP/板位姿。
* `report.json`：总体统计、每姿态误差、组内抖动和运动覆盖范围。
* `errors.csv`：每姿态误差表，可用 Excel 查看。

离线重算无需启动 ROS，可指定新的外参进行同批数据对比；照片仍保留在原采样目录：

```bash
python3 ros2_ws/src/board_pose_ros/tools/verify_handeye.py --mode eye_in_hand \
  --replay ros2_ws/result/<采样目录>/samples.json
# 可加 --calibration ros2_ws/result/<另一份结果>_calibration.json
```

### 采样超时排查

采样等待 3 秒后会显示各输入的接收计数、发布者数量和板识别状态，超时再次打印。
若图像/TCP 有数据、板位姿为 0，且状态为“未检测到 ArUco marker”或“内角点不足”，
应先修正识别节点的实物板参数和拍摄条件。仅有话题名/发布者不代表正在发布有效位姿。

默认配置是 `DICT_4X4_50`、5×7 格；若实物是 5×5 字典、7×9 格的板，应在识别节点指定
`-p dictionary:=DICT_5X5_50 -p squares_x:=7 -p squares_y:=9`，并填入实测
`square_length` 和 `marker_length`（单位米；分别是棋盘方格边长、码外侧黑边边长）。
不能只为检出而降低 `min_points`；错配模型也可能检出少量角点，却得到错误位姿。
先确认 `/board_pose/status` 显示成功并通过重投影门限，再采验证数据。
如果这些参数与原手眼标定时不同，需要用正确参数重新标定后再验证。

当前用户实物板的完整配置已保存为 `config/charuco_7x9_24mm.yaml`：7×9 格、
方格 24 mm、码边长 15 mm、`DICT_5X5_250`，最少 9 个角点、重投影门限 1 px。
先停止旧识别节点，再从仓库根目录运行：

```bash
ros2 launch board_pose_ros board_pose.launch.py \
  params_file:="$(ros2 pkg prefix --share board_pose_ros)/config/charuco_7x9_24mm.yaml"
```

参考截图中的节点名是 `charuco_detector`，不能直接用于本包的 `board_pose` 节点；
本配置已将 `min_corners`、`max_reprojection_error` 分别映射为本包的
`min_points`、`max_reproj_err`。输出仍为 `/aruco_single/pose`，验证工具无需改话题。
本机 OpenCV 4.5.4 无 `setLegacyPattern` 接口，这块奇数行板已用实拍图检查布局兼容性。
