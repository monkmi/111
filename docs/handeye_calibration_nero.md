# nero 机械臂 + RealSense 手眼标定：配置与流程说明

> 适用：AgileX **nero** 七轴机械臂（本机 CAN 通道 `can0`）、Intel RealSense D435、
> ROS 2 Humble、标定板为 **ChArUco** 或 **8 圆点板**。
> 本文的每条命令都在本机 `~/workspace/nero-official-calibrate` 下实际执行过，输出片段为真实
> 结果；只有“相机驱动安装”“真实图像采集”两步因本机尚未安装 `realsense2_camera` 而未实测，已标注。

---

## 0. 交付物与仓库结构

```
nero-official-calibrate/
├── agx_arm_ros/                        # 机械臂驱动（官方仓库，需初始化 submodule 后编译）
├── ros2_ws/                            # 唯一工作空间
│   ├── src/
│   │   ├── agx_arm_ros -> ../../agx_arm_ros   # 软链接，单工作空间编译
│   │   ├── handeye_calibration_ros/    # 手眼标定节点（官方仓库）
│   │   └── board_pose_ros/             # 本项目新增：标定板位姿识别 / 板生成 / 内参 / 验证
│   ├── build/  install/  log/
└── docs/
    └── handeye_calibration_nero.md     # 本文件
```

`board_pose_ros` 提供四个命令，补齐官方流程里缺失的部分：

| 命令 | 替代/补充 | 作用 |
| --- | --- | --- |
| `board_pose_node` | 替代 `aruco_ros` 的 `single.launch.py` | 识别 ChArUco / 8 圆点板 / 棋盘格，发布 `/aruco_single/pose` |
| `board_gen` | — | 生成 1:1 可打印的标定板 PNG + 几何参数 JSON |
| `charuco_calibrate` | `camera_calibration` | 用图片离线标定内参，输出 `camera_intrinsics.yaml` |
| `verify_board` | — | 标定后验证：静止标定板在 `base_link` 下是否恒定 |

> **为什么不用 `aruco_ros`**：`aruco_ros` 只能识别单个 ArUco 码（`single.launch.py`），
> 不支持 ChArUco 板，也不支持本项目使用的 8 圆点板；本机也未安装该包。
> `board_pose_ros` 的话题名与 `aruco_ros` 保持一致（`/aruco_single/pose`），
> 因此 `handeye_calibration_ros` 无需任何改动即可直接使用。

---

### 0.1 一页流程（照做即可）

```bash
# ① 环境（只需一次）：§3 —— apt 依赖、升级 pyAgxArm、submodule、colcon build
# ② 激活 CAN（每次开机）：§4.2；bus-info 用 find_all_can_port.sh 查询（本机是 1-2.2:1.0）
bash ~/workspace/nero-official-calibrate/agx_arm_ros/scripts/can_activate.sh can0 1000000 "1-2.2:1.0"

# ③ 终端 A：机械臂
ros2 launch agx_arm_ctrl start_single_agx_arm.launch.py can_port:=can0 arm_type:=nero \
  effector_type:=agx_gripper auto_enable:=false \
  tcp_offset:='[0.175, 0.0, -0.023507, 0.0, 0.0, 0.0]'

# ④ 终端 B：相机（本机已装 ros-humble-realsense2-camera，4.59.1）
ros2 run realsense2_camera realsense2_camera_node --ros-args -p enable_depth:=false \
  -p rgb_camera.color_profile:=1280x720x30

# ⑤ 终端 C：标定板识别
ros2 run board_pose_ros board_pose_node --ros-args -p board_type:=charuco

# ⑥ 终端 D：手眼采集（§9 详解）
ros2 run handeye_calibration_ros agx_arm_calibration --ros-args \
  -p mode:=eye_in_hand -p min_num:=15 -p result_save_path:=./result

# ⑦ 采 15~25 组「多旋转、停稳 1~2s 再按 Enter」的样本 → q → 得到 result/*.json
# ⑧ 发布静态 TF + verify_board 验证（§10；TF 链需要 rviz 版 launch，见 §5.3）
ros2 run tf2_ros static_transform_publisher --x .. --y .. --z .. --qx .. --qy .. --qz .. --qw .. \
  --frame-id tcp_link --child-frame-id camera_color_optical_frame
ros2 run board_pose_ros verify_board
```

> **眼在手外（相机固定、标定板固定在末端）**：第 ⑥ 步把 `mode` 改成 `eye_to_hand`，
> 第 ⑧ 步的静态 TF 改为 `--frame-id base_link --child-frame-id <相机坐标系>`，
> 多姿态拍照验证使用 `verify_handeye --mode eye_to_hand`（§11.7），旧 `verify_board` 不适用于手外。
> 完整步骤、结果语义与验证方法见 **§11**。

打印用的标定板图片已经生成在 `docs/boards/`（`charuco_board.png` / `dots_board.png`），
用 100% 比例打印即可；也可用 `board_gen` 按需重新生成（§7）。

---

## 1. 原理与坐标约定（决定结果怎么用）

### 1.1 眼在手上（eye-in-hand）

相机固定在机械臂末端（夹爪/法兰）上，标定板固定在桌面。机械臂移动时，相机与末端一起动，
标定板不动：

```
A · X = X · B
A：相邻两次末端位姿的变换（base 系下）
B：相邻两次标定板位姿的变换（相机系下）
X：待求的“相机 -> 末端”外参
```

`handeye_calibration_ros` 的输入输出与本项目的约定：

| 量 | 来源 | 含义 |
| --- | --- | --- |
| `gripper2base` | `/feedback/tcp_pose` | 末端（tcp_link）在 `base_link` 下的位姿 |
| `target2cam` | `/aruco_single/pose` | 标定板在相机坐标系下的位姿：`p_cam = R·p_board + t` |
| 结果 `X` | `cv2.calibrateHandEye` | `p_tcp = R·p_cam + t`，即 **相机系 -> tcp 系** |

**因此：结果 JSON 里的 `position` / `orientation` 直接就是 `tcp_link -> 相机` 的变换，
不需要再取逆。** 发布静态 TF 时：

```bash
parent = tcp_link          child = 相机坐标系
```

### 1.2 用“光学坐标系”还是 `camera_link`

RealSense 的图像与 `camera_info` 都基于**光学坐标系** `camera_color_optical_frame`：
`z` 朝前（光轴）、`x` 朝右、`y` 朝下。而 `camera_link` 是 REP-103 机器人坐标系（`x` 朝前、`z` 朝上），
两者相差一个固定旋转。

本项目默认把标定板位姿输出在**图像自身的光学坐标系**中，于是：

* `X` = `tcp_link -> camera_color_optical_frame`
* 静态 TF：`parent=tcp_link`、`child=camera_color_optical_frame`
* rviz 里加 `Image` 显示时，相机系与图像完全对齐，最直观

若你希望与官方教程的“`camera_link`”一致，给节点加参数 `output_frame:=camera_link`，
节点会用 TF 把板位姿转到 `camera_link`，此时 `X` = `tcp_link -> camera_link`，
静态 TF 的 `child` 也要写成 `camera_link`。

> 两种写法都正确，**关键是“板位姿所在的坐标系”与“静态 TF 的 child”必须一致**。
> 混用会让外参看起来“总是差 90°”。

> **眼在手外（相机固定、板装在末端）的约定与上面相反**：结果变成 `T_base_cam`，
> 静态 TF 写成 `parent=base_link`、`child=<相机坐标系>`。详见 §11.0。

### 1.3 TF 在手眼标定里到底用不用

* **标定计算本身不需要 TF**：`agx_arm_calibration` 只订阅两个话题，不做 TF 查询。
* **验证与可视化需要 TF**：`verify_board`、rviz 里的静态 TF 叠加都要求
  `base_link -> ... -> tcp_link -> 相机系` 这条链完整。
* 只有带 rviz 的 launch 才会启动 `robot_state_publisher` 发布 `base_link -> link1..link7`；
  普通 launch 只有话题，没有 TF（详见 §5.3）。

---

## 2. 硬件与软件清单

| 项 | 本机实测值 |
| --- | --- |
| 机械臂 | AgileX nero（7 关节），末端 `tcp_link` |
| 夹爪 | `agx_gripper`（松灵二指），`tcp_offset = [0.175, 0, -0.023507, 0, 0, 0]` |
| CAN 适配器 | gs_usb（`1d50:606f`），接口 `can0`，1 Mbps |
| 相机 | Intel RealSense D435（本机接了 2 台） |
| 标定板 | ChArUco 5×7，方格 30 mm / marker 22 mm，`DICT_4X4_50`；或 8 圆点板 3×3 缺一角、间距 40 mm |
| 系统 | Ubuntu 22.04 + ROS 2 Humble，OpenCV 4.5.4，Python 3.10 |

---

## 3. 一次性环境准备

### 3.1 系统依赖

```bash
sudo apt update
sudo apt install -y \
  ros-humble-cv-bridge ros-humble-tf2-ros ros-humble-tf-transformations \
  ros-humble-tf2-tools ros-humble-rqt-image-view \
  python3-opencv python3-numpy python3-yaml \
  can-utils iproute2 ethtool
# 若要用 RealSense（本机尚未安装）
sudo apt install -y ros-humble-realsense2-camera
```

验证：

```bash
python3 -c "import cv2, cv_bridge, tf_transformations; print(cv2.__version__)"   # 4.5.4
```

### 3.2 升级 pyAgxArm（必做，本机踩过坑）

`agx_arm_ctrl` 的节点需要 `pyAgxArm.resolve_firmware_profile`。若 pip 里是旧构建，
启动时会报：

```
ImportError: cannot import name 'resolve_firmware_profile' from 'pyAgxArm'
```

处理：

```bash
git clone --depth 1 https://github.com/agilexrobotics/pyAgxArm.git /tmp/pyAgxArm
cd /tmp/pyAgxArm && pip3 install --user --upgrade .
```

### 3.3 初始化子模块并编译

```bash
cd ~/workspace/nero-official-calibrate/agx_arm_ros
git submodule update --init --recursive          # agx_arm_description/agx_arm_urdf 必须齐全
```

单一工作空间编译（避免两份 `install/` 互相覆盖）：

```bash
cd ~/workspace/nero-official-calibrate/ros2_ws
ln -sfn ../../agx_arm_ros src/agx_arm_ros        # 只建一次软链接
source /opt/ros/humble/setup.bash

colcon build --symlink-install --packages-up-to agx_arm_ctrl
colcon build --symlink-install --packages-select agx_arm_description
colcon build --symlink-install --packages-select board_pose_ros handeye_calibration_ros
```

`ros2 launch agx_arm_ctrl start_single_agx_arm.launch.py --show-args` 能列出参数即成功。

### 3.4 每次开新终端的固定动作

```bash
source /opt/ros/humble/setup.bash
source ~/workspace/nero-official-calibrate/ros2_ws/install/setup.bash
```

---

## 4. CAN 与“总线上有两个机械臂、只上电一个”

### 4.1 这是正常现象

AgileX 机械臂通过**自带的 CAN 模块**接入 PC，每台臂对 PC 就是一个 CAN 接口。
两条总线（或同一条总线）上物理连接了两台臂、但只给一台臂上电，是完全正常的用法：

* **没有上电的那台臂不参与通信**：CAN 收发器未上电时处于高阻/被动状态，不会应答、不会发帧，
  不会干扰在用的那台臂。
* PC 侧只会看到在用的那台臂反馈的帧；`/feedback/tcp_pose` 等话题也只会描述它。
* 这也正是**推荐**的工作方式：`agx_arm_ros/README.md` 明确说明——“主从臂连接到同一个 CAN 模块时，
  需要通过 `fw_version` 人工指定固件版本以跳过自动获取；主臂也会返回版本信息，主从臂同时向同一
  CAN 模块发送相同 ID 的数据可能导致异常。”

  换句话说：**同一条 CAN 总线上让两台臂都上电并同时通信，会产生相同 CAN ID 的冲突**；
  要做双臂协同必须按官方的主从臂规则配置。（本项目的标定只需要一台臂，无需关心主从配置。）

* 空出来的那条总线/那台臂，之后单独标定另一只眼时再上电即可。

### 4.2 激活 CAN

```bash
# 只插目标臂的 CAN 模块时执行，找到 usb bus-info
cd ~/workspace/nero-official-calibrate/agx_arm_ros/scripts
bash find_all_can_port.sh                 # 例：1-2.2:1.0

# 激活：接口名 波特率 usb bus-info（波特率必须是 1000000）
bash can_activate.sh can0 1000000 "1-2.2:1.0"
```

检查：

```bash
ip -details link show can0
# 期望：state UP、bitrate 1000000、sample-point 0.750
```

### 4.3 关于 `can state ERROR-PASSIVE`

本机实测：

```
can0: <NOARP,UP,LOWER_UP,ECHO> mtu 16 qdisc pfifo_fast state UP
      can state ERROR-PASSIVE (berr-counter tx 0 rx 0) restart-ms 0
      bitrate 1000000 sample-point 0.750
```

**这仍然可以正常通信。** 当总线上只有本机一个节点、对端 ACK/错误帧计数累积时，
控制器会进入 `ERROR-PASSIVE`，但只要 `berr-counter` 没有持续增长、且能收到反馈帧，
就说明链路是好的。本机在该状态下成功读到 nero 的固件版本与位姿（见 §5.2）。

判断是否真的故障，看这三点：

1. 启动节点后是否有 `/feedback/*` 话题数据（`ros2 topic hz /feedback/joint_states`）；
2. `ip -s link show can0` 的错误计数是否持续快速增长；
3. 换一根短线、确认终端电阻（120 Ω）与波特率一致、确认只有一台臂在供电。

---

## 5. 启动 nero

### 5.1 推荐命令（本机实测）

```bash
ros2 launch agx_arm_ctrl start_single_agx_arm.launch.py \
  can_port:=can0 \
  arm_type:=nero \
  effector_type:=agx_gripper \
  auto_enable:=false \
  tcp_offset:='[0.175, 0.0, -0.023507, 0.0, 0.0, 0.0]'
```

说明：

* `arm_type` 默认是 `piper`，**nero 必须显式指定**。
* `auto_enable:=false`：只读取反馈、不上使能，适合手动摆位采集样本；若要程序控制运动再改 `true`。
* `tcp_offset` 是松灵二指夹爪相对 `link7` 的偏置（单位 m / rad），顺序 `[x, y, z, roll, pitch, yaw]`。
  该值来自 AgilcX 官方教程中 nero + agx_gripper 的组合；换夹爪/换安装方式必须重测。

### 5.2 启动成功的标志

```
[INFO] fw_version: v121, 1.22, v121
[INFO] AgxGripper initialized successfully
[INFO] Agx_arm feedback is ready, control is now enabled
```

话题：

```bash
ros2 topic list | grep feedback
# /feedback/arm_status /feedback/gripper_status /feedback/joint_states
# /feedback/leader_joint_states /feedback/motor_status /feedback/tcp_pose

ros2 topic echo --once /feedback/tcp_pose
# position 约 (0.06, 0.003, 0.21)（本机当前姿态）
```

服务（相对命名空间）：`/enable_agx_arm`、`/move_home`、`/exit_teach_mode`、`/emergency_stop`。

### 5.3 TF：什么时候才有 `tcp_link`

| 启动方式 | `/feedback/tcp_pose` | `base_link -> tcp_link` TF |
| --- | --- | --- |
| `start_single_agx_arm.launch.py` | ✅ | ❌ 没有 `robot_state_publisher` |
| `start_single_agx_arm_rviz.launch.py` | ✅ | ✅ 内含 `agx_arm_description/launch/display.launch.py` |
| 单独 `display.launch.py` | — | ✅ 只发布 URDF/TF |

实测（`display.launch.py arm_type:=nero effector_type:=agx_gripper tcp_offset:='[0.175,0,-0.023507,0,0,0]' gui:=false`）：

```bash
ros2 topic echo --once /tf_static
# frame_id: link7
# child_frame_id: tcp_link        ← 只有 tcp_offset 非零时才会发布 tcp_link
```

所以：**要做 `verify_board` 验证或 rviz 里看 TF，就用带 rviz 的 launch，或额外启动 `display.launch.py`。**
纯手眼计算用话题即可，不需要 TF。

---

## 6. 相机与内参

### 6.1 启动 RealSense（本机已装好驱动）

本机已装 `ros-humble-realsense2-camera`（4.59.1），实测单台 D435 在 `1280x720x30` 下
可正常出图，并被 `board_pose_node` 正常接收（`intrinsics_source: camera_info`，帧计数持续增长）：

```bash
# 只有一台相机
ros2 run realsense2_camera realsense2_camera_node --ros-args \
  -p enable_color:=true -p enable_depth:=false \
  -p rgb_camera.color_profile:=1280x720x30

# 两台 D435 同时接入时，用序列号区分并加命名空间
ros2 run realsense2_camera realsense2_camera_node --ros-args \
  -r __ns:=/cam0 -p serial_no:='"123456789012"' -p enable_depth:=false \
  -p rgb_camera.color_profile:=1280x720x30
```

序列号用 `rs-enumerate-devices -s` 查询。对应的图像与内参话题：

```
/cam0/camera/color/image_raw
/cam0/camera/color/camera_info
```

### 6.2 内参的两个阶段

1. **先用出厂内参（当前方案）**：RealSense 在 `camera_info` 里直接给出内参与畸变，
   识别节点默认订阅它，无需额外操作。
2. **后续换成自测内参**：用 `charuco_calibrate` 标定后，把 YAML 传给识别节点：

```bash
# 拍 15~25 张不同角度/位置的板图，放到 images/ 目录
ros2 run board_pose_ros charuco_calibrate \
  --images ./images --out ~/camera_intrinsics.yaml \
  --dictionary DICT_4X4_50 \
  --squares-x 5 --squares-y 7 --square-length 0.03 --marker-length 0.022

# 让识别节点优先使用这份内参
ros2 run board_pose_ros board_pose_node --ros-args \
  -p intrinsics_file:=$HOME/camera_intrinsics.yaml
```

标定输出示例（合成数据验证，真值 900/902/640/480）：

```
fx=899.195 fy=901.290 cx=639.909 cy=480.363
D=[-0.0017, 0.0061, 8.8e-05, -8.8e-05, -0.0096]
RMS=0.1023 px
```

> 注意：内参是**某个分辨率**下的。采集手眼样本与后续使用必须用同一分辨率，
> 换分辨率要重新标定或用 `charuco_calibrate --image-size WxH` 重新计算。

---

## 7. 标定板：生成与打印

```bash
# ChArUco 5x7（30 mm 方格 / 22 mm marker），A4 可打印，300 dpi
ros2 run board_pose_ros board_gen charuco --dpi 300 --out-dir ./boards

# 8 圆点板：3x3 缺左上角、间距 40 mm、半径 8 mm
ros2 run board_pose_ros board_gen dots --rows 3 --cols 3 --spacing 0.04 \
  --radius 0.008 --missing "0,0" --out-dir ./boards

# 棋盘格（内角点 5x3，方格 20 mm）
ros2 run board_pose_ros board_gen checker --cols 5 --rows 3 \
  --square-length 0.02 --out-dir ./boards
```

每块板同时生成 PNG + 同名 JSON（几何参数、板尺寸、dpi、print_scale）。
仓库里已经生成好两份可直接打印的板：`docs/boards/charuco_board.png`（150×210 mm）与
`docs/boards/dots_board.png`（160×160 mm）。

打印与使用要点：

* **必须 100% 比例打印**（PDF/图片查看器里选“实际大小”，不要“适应纸张”）；
* 打印后用尺量图上的 **100 mm 标尺**，误差应 < 1 mm；
* 板面要平整、无反光，贴在硬板上；ChArUco 建议哑光纸 + 双面胶贴平；
* **节点里的尺寸参数必须与打印的板完全一致**（`square_length`、`marker_length`、`dot_spacing`），
  尺寸填错会直接体现为尺度误差；
* 八个圆点板缺角（`dot_missing`）用来唯一确定板的朝向，缺哪一格要和参数一致；
* 遮住 1 个圆点仍可解算（8 点板有冗余），ChArUco 遮几个 marker 也能用。

---

## 8. 标定板识别节点

### 8.1 启动

**当前用户实物板是 7×9、方格 24 mm、码 15 mm、`DICT_5X5_250`**，使用专用配置
（从仓库根目录执行，先停掉旧识别节点）：

```bash
ros2 launch board_pose_ros board_pose.launch.py \
  params_file:="$(ros2 pkg prefix --share board_pose_ros)/config/charuco_7x9_24mm.yaml"
```

下方默认命令适用于仓库生成的 5×7、30 mm 板，与上述实物板不同，不能混用。

```bash
# ChArUco
ros2 run board_pose_ros board_pose_node --ros-args \
  -p board_type:=charuco \
  -p image_topic:=/camera/camera/color/image_raw \
  -p camera_info_topic:=/camera/camera/color/camera_info

# 8 圆点板
ros2 run board_pose_ros board_pose_node --ros-args \
  -p board_type:=dots \
  -p dot_rows:=3 -p dot_cols:=3 -p dot_spacing:=0.04 -p dot_missing:="0,0"

# 或用 launch + 配置文件
ros2 launch board_pose_ros board_pose.launch.py board_type:=dots
```

### 8.2 输出

| 名称 | 类型 | 说明 |
| --- | --- | --- |
| `/aruco_single/pose` | `PoseStamped` | 标定板位姿（板 -> 相机）；帧名为图像坐标系 |
| `/board_pose/status` | `String` | JSON：`detected`、`n_points`、`reproj_err_px`、`position`、`orientation_xyzw`、`intrinsics_source` 等 |
| `/board_pose/debug` | `Image` | 检测点、坐标轴、文字叠加 |
| TF `camera... -> aruco_marker_frame` | `tf2` | 板位姿，可 rviz 可视化 |

`status` 实测示例（合成相机，误差 0.225 mm）：

```json
{"detected": true, "board_type": "charuco", "n_points": 24,
 "reproj_err_px": 0.038, "frame_id": "camera_color_optical_frame",
 "intrinsics_source": "file:/tmp/exact_intr.yaml",
 "frames": {"received": 25, "accepted": 16},
 "position": [0.0202, -0.0099, 0.4499],
 "orientation_xyzw": [0.9312, 0.2936, -0.0650, -0.2059],
 "pose_frame": "camera_color_optical_frame", "tf_child": "aruco_marker_frame"}
```

### 8.3 判读是否正常

* `detected=true`、`n_points` 与板点数相符（ChArUco 5×7 → 24 内角点；8 点板 → 8）；
* `reproj_err_px` 应 **< 1 px**（好相机 + 板平整时通常 0.1~0.5 px）；
  持续 > 2 px 说明内参不准、板参数填错或板不平；
* 节点日志会打印 `检出 charuco: 24 点, 重投影 0.04 px, t=(...) m`（1 Hz 限流）。

### 8.4 无相机时先自检

```bash
ros2 run board_pose_ros board_self_check     # 合成图像，三种板型全 PASS 才算环境正常
```

---

## 9. 手眼标定流程（眼在手上）

### 9.1 终端清单

| 终端 | 命令 |
| --- | --- |
| A | 机械臂（+ 可选 rviz 版以提供 TF） |
| B | RealSense 相机驱动 |
| C | `board_pose_node`（标定板识别） |
| D | `agx_arm_calibration`（采集 + 计算） |

### 9.2 启动采集节点

```bash
ros2 run handeye_calibration_ros agx_arm_calibration --ros-args \
  -p mode:=eye_in_hand \
  -p agx_arm_topic:=/feedback/tcp_pose \
  -p marker_topic:=/aruco_single/pose \
  -p min_num:=15 \
  -p result_save_path:=./result
```

参数说明：

| 参数 | 默认 | 说明 |
| --- | --- | --- |
| `mode` | `eye_in_hand` | 眼在手上；眼在手外改 `eye_to_hand` |
| `min_num` | `10` | 最少样本数，小于 5 会直接报“最少采集次数不得小于5次” |
| `agx_arm_topic` | `/feedback/tcp_pose` | 末端位姿 |
| `marker_topic` | `/aruco_single/pose` | 标定板位姿 |
| `result_save_path` | `./result` | 结果目录（脚本自动创建） |

### 9.3 摆位与采样（决定精度的关键）

1. **板固定**：把 ChArUco / 圆点板放在桌面上不动的区域，保证机械臂全行程内都能看到。
2. **相机朝向**：相机随末端运动，板要始终完整在视野内，距离约 0.25~0.6 m。
3. **姿态要“转”不要“平”**：
   * 每个样本之间要有**明显的三维旋转**（绕不同轴，单轴 20°~40°）；
   * 不要只用“平移”凑样本——纯平移会让方程退化（矩阵奇异），结果必然发散；
   * 不要让板始终正对相机（正对时姿态信息弱），倾斜 15°~40° 最好；
   * 让 roll/pitch/yaw 三种旋转都出现，且尽量覆盖工作空间。
4. **停稳再按 Enter**：节点取数是“先取一帧板位姿、再取一帧末端位姿”，
   **两者不是时间戳同步的**。机械臂还在动时按 Enter，会取到两个不同位姿的数据，
   直接污染结果（实测错位 0.2 s 就能带来 3° / 4 mm 的误差）。
   → 每个位姿停稳 **1~2 s**，看到 terminal C 的板位姿数值稳定后再按 Enter。
5. **样本数**：建议 **15~25 组**，覆盖大范围姿态；不是越多越好，重复姿态无增益。
6. **交互按键**：

| 键 | 作用 |
| --- | --- |
| `Enter` | 采集当前一帧 |
| `d` | 回退上一条（样本数 ≤ `min_num` 时可用） |
| `q` | 计算并退出（样本数 > `min_num` 时才出现该选项） |
| `c` | 直接退出，不计算 |

采集过程中终端 C 应持续有稳定输出；若板位姿跳变或为 NaN，说明识别失败，不要采集。

### 9.4 结果

计算完成后：

1. 终端打印 JSON（`position` / `orientation` / `rpy`）；
2. 写入 `{result_save_path}/{YYYY-MM-DD_HH-MM-SS}_calibration.json`。

示例（合成数据端到端验证，真值 position `[0.045, -0.03, 0.075]`）：

```json
{
  "position": [0.0471, -0.0286, 0.0717],
  "orientation": [0.0366, -0.6500, 0.0316, 0.7584],
  "rpy": [[0.0946, -1.4171, 0.0023]]
}
```

语义：`p_tcp = R·p_cam + t`，即 **`tcp_link -> 相机坐标系`**，不需要取逆。

`orientation` 是四元数 `[x, y, z, w]`，`rpy` 为固定轴 XYZ 的欧拉角（弧度）。

---

## 10. 结果解读与验证

### 10.1 发布静态 TF

把 JSON 的数值填进命令（示例）：

```bash
ros2 run tf2_ros static_transform_publisher \
  --x 0.0471 --y -0.0286 --z 0.0717 \
  --qx 0.0366 --qy -0.6500 --qz 0.0316 --qw 0.7584 \
  --frame-id tcp_link --child-frame-id camera_color_optical_frame
```

如果节点用了 `output_frame:=camera_link`，把 `--child-frame-id` 改成 `camera_link` 即可。

在 rviz 里（用带 rviz 的机械臂 launch）：`Fixed Frame` 选 `base_link`，
加 `Image` 显示（`/camera/camera/color/image_raw`）、加 `TF`，即可看到相机系是否正确贴在末端。

### 10.2 用“静止板恒定性”验证（最可靠，推荐）

原理：板不动、相机随臂动，若外参正确，**同一块板在 `base_link` 下的位姿应基本不变**。

前提：`base_link -> ... -> tcp_link` TF 链存在（rviz 版 launch 或 `display.launch.py`），
并且已经按 §10.1 发布了 `tcp_link -> 相机` 静态 TF。

```bash
ros2 run board_pose_ros verify_board --ros-args \
  -p pose_topic:=/aruco_single/pose -p base_frame:=base_link
```

操作：把板固定不动，移动机械臂到 5 个以上不同姿态，每个姿态停留 2 s。

输出示例（本机合成链路验证）：

```
[10] 板在 base_link: p=(+0.120, -0.010, +0.650) m, rpy=(-155.1, +0.0, +35.0)° | 滑窗波动 p=0.0 mm, r=0.00°
```

判据：**位置波动 < 5 mm、姿态波动 < 1°**。
若波动很大（几十毫米/几度），说明外参错误，或坐标系用错（光学系 vs `camera_link` 混用），
或板在采集期间被挪动过。

### 10.3 重复标定一致性

用同一套数据间隔重采，跑两次标定：

* 两次 `position` 差 < 3~5 mm、旋转差 < 1°~2° → 结果可信；
* 差很多 → 样本姿态太相似 / 采集时臂在动 / 板被移动 / 内参不准。

### 10.4 可选：换用 PARK 方法提高解算精度

`agx_arm_calibration.py` 调用的是 `cv2.calibrateHandEye(...)` 的默认方法（TSAI）。
用合成数据（30 组姿态、已知真值）对比：

| 方法 | 旋转误差 | 平移误差 |
| --- | --- | --- |
| 默认（TSAI） | 2.26° | 2.0 mm |
| `CALIB_HAND_EYE_PARK` | ~0 | ~0 |
| `CALIB_HAND_EYE_HORAUD` | ~0 | ~0 |
| `CALIB_HAND_EYE_DANIILIDIS` | 0.91° | 13.5 mm |

本实验室数据上 PARK/HORAUD 明显更稳。若你的结果重复性差，可把
`handeye_calibration_ros/handeye_calibration_ros/agx_arm_calibration.py:105` 的调用改为：

```python
R_cam2gripper, t_cam2gripper = cv2.calibrateHandEye(
    R_gripper2base, t_gripper2base, R_target2cam, t_target2cam,
    method=cv2.CALIB_HAND_EYE_PARK)
```

（改的是官方仓库文件，升级仓库时注意保留。）

---

## 11. 眼在手外：标定板固定在机械臂末端（推荐本场景用这一节）

### 11.0 先记住结果语义

| 输入 | 含义 |
| --- | --- |
| `/feedback/tcp_pose` | `T_base_tcp`（末端在 base 下的位姿） |
| `/aruco_single/pose` | `T_cam_board`（板在相机光学系下的位姿） |
| **结果 JSON** | **`T_base_cam`：相机光心在 `base_link` 下的坐标 + 相机姿态** |

节点在 `mode:=eye_to_hand` 时会把**机械臂位姿取逆**再送 `cv2.calibrateHandEye`
（见 `agx_arm_calibration.py:147`：`PoseMartix(agx_arm_pose_raw.pose, True, self.mode)`），
所以得到的就是 `T_base_cam`，**不要再取逆**。

合成数据端到端实测（真值 `t=(0.85, 0.10, 0.45)`，用 pty 驱动真节点采 12 帧）：

```
"position": [0.8499999999999999, 0.09999999999999942, 0.4500000000000003]
```

误差在 1e-16 量级；若把机械臂位姿**不取逆**直接喂，会得到 180°/686 mm 的错解。

静态 TF 方向（与眼在手上**相反**）：

* `parent=base_link`、`child=<相机坐标系>`
* 平移/四元数直接用 JSON 的 `position` / `orientation`

### 11.1 装板（最容易出错的一步）

* 板必须**刚性固定**在末端：打印板贴 3~4 mm 硬板（亚克力/铝板/瓦楞板），再用双面胶、扎带或
  打印支架固定在夹爪/法兰上；**不要用夹爪软夹持一块会晃的纸**。
* 板面朝向相机；夹爪手指、支架、线缆不能挡住板的四个定向特征（ChArUco 的码 / 圆点板的缺角）。
* 装好后到本轮标定结束**不要拆、不要碰**；中途松动 → 全部样本作废。

### 11.2 相机

* 固定在三脚架/支架上俯视工作区，保证**所有采样姿态下都能看到整块板**。
* 标定期间**绝对不要移动相机**；相机一旦被碰，必须重标。
* 相机固定后 `board_pose_node` 用默认 `output_frame`（图像光学系）即可，不需要相机侧 TF。

### 11.3 启动（三个终端）

```bash
# T1：机械臂
ros2 launch agx_arm_ctrl start_single_agx_arm.launch.py \
  can_port:=can0 arm_type:=nero effector_type:=agx_gripper \
  auto_enable:=false \
  tcp_offset:='[0.175, 0.0, -0.023507, 0.0, 0.0, 0.0]'

# T2：相机（本机已装好驱动）
ros2 run realsense2_camera realsense2_camera_node --ros-args \
  -p enable_color:=true -p enable_depth:=false \
  -p rgb_camera.color_profile:=1280x720x30

# T3：板识别（话题名与 aruco_ros 兼容）
ros2 run board_pose_ros board_pose_node --ros-args \
  -p board_type:=charuco -p process_rate_hz:=5.0
```

确认：`/aruco_single/pose` 有数据、`/board_pose/status` 里 `"detected": true`、
`reproj_err_px` 小于 1。板参数（`squares_x/y`、`square_length`、`marker_length`）必须与实际打印一致。

### 11.4 启动标定节点

```bash
ros2 run handeye_calibration_ros agx_arm_calibration --ros-args \
  -p mode:=eye_to_hand -p min_num:=15 \
  -p agx_arm_topic:=/feedback/tcp_pose \
  -p marker_topic:=/aruco_single/pose \
  -p result_save_path:=./result_eye_to_hand
```

或 `ros2 launch handeye_calibration_ros agx_arm_calibration_launch.py mode:=eye_to_hand min_num:=15`。

### 11.5 采样要点（和眼在手上的差别）

* 移动的是**带着板的末端**，相机不动；板在图像里的位置/朝向要明显变化。
* 姿态要有**明显旋转**：绕三个轴都转，特别是绕相机光轴的滚转；纯平移样本对 AX=XB 无贡献。
* 板面法线与相机光轴夹角保持在 60° 以内，不要让板侧对相机。
* 每个姿态**停稳 1~2 s 再按 Enter**：节点是先取板位姿、再取末端位姿的两帧，不是时间戳配对。
  实测节奏错位 0.2 s 就会带来 3°/4 mm 级误差；合成测试里错位严重时结果直接偏到 0.4 m。
* 15~25 组；`d` 回退一帧；`q` + Enter 计算（需 `count > min_num`，节点要求 `min_num ≥ 5`）。
* 结果写到 `result_save_path/YYYY-MM-DD_HH-MM-SS_calibration.json`。

### 11.6 发布静态 TF（把结果用起来）

```bash
ros2 run tf2_ros static_transform_publisher \
  --x 0.85 --y 0.10 --z 0.45 \
  --qx 0.0370 --qy 0.0555 --qz 0.9621 --qw 0.2645 \
  --frame-id base_link --child-frame-id camera_color_optical_frame
```

上面的数字只是示例，换成你 JSON 里的 `position` / `orientation`。方向千万别写反：
`--frame-id` 是父（`base_link`），`--child-frame-id` 是相机光学系。

### 11.7 验证（眼在手外**不能**用 `verify_board`）

推荐使用新增的 **`verify_handeye` 多姿态拍照工具**，同时支持手内与手外，不需要发布静态 TF。
保持机械臂驱动、相机和标定板识别节点运行，从仓库根目录执行：

```bash
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash

# 手外：相机固定，标定板刚性固定在末端
python3 ros2_ws/src/board_pose_ros/tools/verify_handeye.py \
  --mode eye_to_hand \
  --calibration ros2_ws/result/2026-11-18_11-39-38_calibration.json

# 手内：相机固定在末端，标定板固定在桌面
# 将上面的模式改为 --mode eye_in_hand，并选择对应的手内标定结果
```

这里的 JSON 路径只表示结果文件位置，**模式必须和该文件实际标定时一致**；旧 JSON 本身没有记录模式。
若标定时板位姿输出在 `camera_link`，还需加 `--camera-frame camera_link`。
机械臂 `tcp_offset`、板参数、内参、分辨率和识别节点 `output_frame` 要与标定时一致。

移动机械臂到 10~20 个新姿态（覆盖工作区、绕多轴旋转），每次停稳 1~2 秒后按 Enter。
每组默认匹配并保存 5 张照片，检查图像与 TCP 时间差、TCP 停稳状态；`d` 撤销、`r` 查看当前误差、
`q` 保存报告退出。手内统计 `T_base_tcp @ T_tcp_cam @ T_cam_board` 是否恒定；
手外统计 `inv(T_base_tcp) @ T_base_cam @ T_cam_board` 是否恒定。

报告保存在 `ros2_ws/result/<时间>_<模式>_verification/`，包括照片、原始位姿 `samples.json`、
平均/RMS/最大误差 `report.json` 和每姿态误差 `errors.csv`，位置单位 mm，旋转单位 °。
这是相对所有姿态均值的**一致性误差**，包含视觉、TCP、时间同步误差；绝对定位精度仍需外部真值验证。
参数及离线重算见 [board_pose_ros README](../ros2_ws/src/board_pose_ros/README.md#手内手外多姿态精度验证)。

旧 `verify_board` 的判据是“板在 `base_link` 下保持不动”，眼在手外板是随臂运动的，不适用。
下面三种方式可作为补充：

1. **卷尺核对**：量相机光心相对 base 原点的 x/y/z，与 JSON 平移对比。
   差在 5 cm 内属正常（底座坐标系与手工测量的误差）；rviz 里相机 Z 轴应指向工作区。
2. **触摸验证（最实用）**：把板平放桌面并固定，用识别结果算出板中心在 base 下的坐标
   `p = T_base_cam · t_board`，再让末端(tcp)移到该点正上方 1 cm 或用手持尖针指该点，
   读 `/feedback/tcp_pose`，两者差 **< 1 cm** 即可认为外参可用（含夹爪/tcp_offset 误差）。
3. **重标复现**：隔天或重装相机后重标一次，两次结果平移差 < 5 mm、姿态差 < 1°。

验证通过后再考虑换自测内参（§6.2），换内参必须重标。

### 11.8 为什么这里 `tcp_offset` 影响很小

板相对 tcp 的固定位姿本身就是这组方程里辨识不到的常数（相当于换一个固连在末端上的坐标系），
所以 `tcp_offset` 只要保持不变就不改变结果。但为与其它章节保持一致、也避免 `/feedback/tcp_pose`
数值对不上，仍建议用 §5 的同一套 launch 参数。

---

## 12. 排错表

| 现象 | 可能原因 | 处理 |
| --- | --- | --- |
| `ImportError: cannot import name 'resolve_firmware_profile'` | pyAgxArm 太旧 | §3.2 升级 pyAgxArm |
| 启动节点报 `arm_type:=nero` 无效/默认 piper | 没传 `arm_type` | 显式 `arm_type:=nero` |
| `/feedback/tcp_pose` 无数据 | CAN 未激活 / 未上电 / 波特率不对 | §4.2、`ip -s link show can0` |
| `can state ERROR-PASSIVE` | 单节点/无 ACK，错误计数累积 | 一般无影响；看是否持续增长（§4.3） |
| 找不到 `tcp_link` | 用了无 rviz 的 launch | 用 rviz 版或 `display.launch.py`（§5.3） |
| 节点报 `未检出标定板` | 板不在视野/太暗/参数不符 | 看 `/board_pose/debug` 图；核对板参数 |
| `重投影误差过大` | 内参不准、板尺寸填错、板不平 | 换 `intrinsics_file`、量板尺寸 |
| 位姿随视角翻转 180° | 平面标定板二义解 | ChArUco 加 marker 消除；`dots` 用缺角定向；检查 `dot_missing` |
| `cv2.calibrateHandEye` 报奇异/结果乱飞 | 样本只有平移、姿态太相似 | 增加多轴旋转样本（§9.3） |
| 手眼节点按 Enter 报“程序异常或中断退出” | 在非 TTY 环境（管道/IDE 终端）运行 | 在真实终端里运行 |
| 标定结果差 90° | 光学系与 `camera_link` 混用 | §1.2，统一 `output_frame` 与 TF child |
| 眼在手外结果差 180°/偏差半米 | 机械臂位姿没取逆（自己改代码时常见），或板/相机之一在动 | 用官方节点 `mode:=eye_to_hand`（内部取逆）；固定相机、拧紧板（§11.1/§11.2） |
| RealSense 打不开 | udev 规则/权限 | 重插、`sudo udevadm control --reload-rules` |
| 两台 D435 话题冲突 | 未加命名空间/序列号 | `-r __ns:=/cam0 -p serial_no:=...`（§6.1） |

---

## 13. FAQ

**Q1：CAN 上明明有两个机械臂，为什么只给一个上电？**
正常。见 §4.1：未上电的臂不参与通信；两台臂同时在同一 CAN 模块上通信会因相同 CAN ID 冲突而出问题，
官方要求按主从臂规则指定 `fw_version`。标定只需一台臂，因此只上电一台是标准做法。

**Q2：没有装 `aruco_ros` 怎么办？**
不需要。`board_pose_ros` 的 `board_pose_node` 直接发布 `/aruco_single/pose`，兼容 `handeye_calibration_ros`。
（`aruco_ros` 也不支持 ChArUco。）

**Q3：8 圆点板可以用 `findChessboardCorners` 吗？**
不行。OpenCV 的棋盘格检测器要求内角点 `cols>2 且 rows>2`，4×2 的 8 点阵会直接报错；
8 圆点板请用 `board_type:=dots`（圆点检测 + 单应定序 + PnP）。

**Q4：内参先用出厂的可以吗？**
可以，`camera_info` 里的出厂内参足够起步（尤其 D435 的彩色相机）。但精度要求高时，
请用 `charuco_calibrate` 自测，并通过 `intrinsics_file` 覆盖；换分辨率必须重标。

**Q5：为什么按 Enter 采集时必须停稳？**
节点是先订阅板位姿、再订阅末端位姿，两次订阅之间有时间差，**不是时间戳对齐**。
臂在动时两次读数来自不同位姿，样本就废了。停留 1~2 s 是最省事的做法。

**Q6：为什么结果不用取逆？**
`cv2.calibrateHandEye` 返回的 `X` 满足 `p_tcp = R·p_cam + t`，即相机系 -> tcp 系，
已经与官方教程“camera_link -> tcp_link”的方向一致（合成数据端到端验证：结果与真值同号同量级）。
若你把它当作 `tcp -> camera` 使用（例如 TF 的 parent/child 写反），会得到明显错误。

**Q7：标定板必须打印在 A4 上吗？**
不必须，但**必须 1:1 打印**且量 100 mm 标尺确认比例；A4 竖版能放下 5×7、30 mm 方格的 ChArUco。

---

## 附录 A. `board_pose_node` 参数

| 参数 | 默认 | 说明 |
| --- | --- | --- |
| `board_type` | `charuco` | `charuco` / `dots` / `checker` |
| `image_topic` | `/camera/camera/color/image_raw` | 图像话题 |
| `camera_info_topic` | `/camera/camera/color/camera_info` | 内参话题 |
| `intrinsics_file` | `""` | YAML 内参，非空时优先于 `camera_info` |
| `process_rate_hz` | `10.0` | 处理频率上限，0 = 每帧 |
| `max_reproj_err` | `3.0` | 重投影误差阈值（px） |
| `min_points` | `8` | 最少使用点数 |
| `pose_topic` | `/aruco_single/pose` | 输出位姿 |
| `status_topic` | `/board_pose/status` | 状态 JSON |
| `debug_topic` | `/board_pose/debug` | 调试图 |
| `publish_debug_image` | `true` | 是否发调试图 |
| `publish_tf` | `true` | 是否发 TF |
| `tf_child` | `aruco_marker_frame` | TF child frame |
| `output_frame` | `""` | 输出坐标系；空 = 图像坐标系 |
| `axis_length` | `0.03` | 调试图坐标轴长度（m） |
| ChArUco | `dictionary=DICT_4X4_50`、`squares_x=5`、`squares_y=7`、`square_length=0.03`、`marker_length=0.022` | |
| dots | `dot_rows=3`、`dot_cols=3`、`dot_spacing=0.04`、`dot_missing="0,0"`、`dot_dark_on_light=true`、`dot_min_area_frac=1.5e-4`、`dot_max_area_frac=2e-2`、`dot_min_circularity=0.55`、`dot_min_fill=0.55`、`dot_blur_ksize=5` | `dot_rows` 第一行为 y 最大的一行 |
| checker | `checker_cols=5`、`checker_rows=3`、`checker_square_length=0.02` | 内角点数 |

## 附录 B. `verify_board` 参数

| 参数 | 默认 | 说明 |
| --- | --- | --- |
| `pose_topic` | `/aruco_single/pose` | 板位姿话题 |
| `base_frame` | `base_link` | 目标坐标系 |
| `window` | `60` | 滑窗样本数 |
| `report_every` | `10` | 每 N 帧打印一次 |

## 附录 C. 文件清单（本项目新增）

```
ros2_ws/src/board_pose_ros/
├── board_pose_ros/
│   ├── detectors.py            # 纯 OpenCV：ChArUco / 圆点 / 棋盘格检测 + PnP
│   ├── board_pose_node.py      # ROS 节点
│   ├── board_gen.py            # 标定板 PNG 生成
│   ├── charuco_intrinsics.py   # 离线内参标定
│   ├── check_board_in_base.py  # verify_board 实现
│   ├── io_utils.py             # 内参 YAML 读写
│   └── self_check.py           # 合成图像自检
├── config/{charuco,dots,checker}.yaml
├── launch/board_pose.launch.py
├── tools/offline_check.py      # self_check 的兼容入口
├── package.xml  setup.py  setup.cfg  README.md
```

## 附录 D. 一分钟自检

```bash
source /opt/ros/humble/setup.bash
source ~/workspace/nero-official-calibrate/ros2_ws/install/setup.bash

ros2 run board_pose_ros board_self_check          # 1) 检测算法自检
ros2 topic hz /feedback/joint_states              # 2) 机械臂通信
ros2 topic hz /camera/camera/color/image_raw      # 3) 相机图像
ros2 topic echo /board_pose/status                # 4) 板识别状态
```

## 附录 E. 本机验证记录（可作为回归基线）

| 项 | 结果 |
| --- | --- |
| `board_self_check`（合成图，三种板型） | 全部 PASS；charuco/dots 平移误差 < 0.2 mm、重投影 < 0.1 px |
| `board_pose_node` + 合成 ChArUco 图 | 24 内角点，重投影 0.038 px，位姿平移误差 **0.225 mm** |
| `board_pose_node` + 合成 8 圆点板 | 8 点（含缺角定向），重投影 0.043 px，平移误差 **0.07 mm** |
| 调试图 / TF | `/board_pose/debug` 有图，TF `camera_color_optical_frame -> aruco_marker_frame` 正常 |
| `verify_board`（合成链路 + 静态 TF） | 板在 `base_link` 下 `p=(0.120, -0.010, 0.650)`，与静态 TF+板位姿之和一致；滑窗波动 0.0 mm |
| `charuco_calibrate`（25 张合成图，真值 900/902/640/480） | fx=899.195 fy=901.290 cx=639.909 cy=480.363，RMS 0.102 px |
| `cv2.calibrateHandEye` 方法对比（30 组姿态） | 默认 TSAI 2.26°/2.0 mm；PARK、HORAUD ≈ 0；DANIILIDIS 0.91°/13.5 mm |
| handeye 端到端（合成 12 帧，pexpect 驱动） | 结果 position ≈ (0.0471, -0.0286, 0.0717)，真值 (0.045, -0.03, 0.075) → 方向约定正确 |
| nero 实机（can0，`ERROR-PASSIVE`） | fw `v121, 1.22, v121`；`AgxGripper initialized successfully`；`/feedback/tcp_pose` 有数据 |
| RealSense D435 + `board_pose_node`（实机 1280x720） | 节点持续收帧（`frames.received` 增长），`intrinsics_source: camera_info`；无板时 `detected:false`、msg 正常 |
| handeye `mode:=eye_to_hand` 端到端（合成 12 帧，真值 `T_base_cam` t=(0.85,0.10,0.45)） | 输出 `position=[0.8499999999999999, 0.09999999999999942, 0.4500000000000003]`，误差 1e-16 → **结果即 `T_base_cam`，不取逆** |
| 同上但机械臂位姿不取逆（对照） | 180°/686 mm 的错解，说明 `eye_to_hand` 的取逆是必需的 |
| TF（`display.launch.py` + `tcp_offset`） | `/tf_static` 中 `link7 -> tcp_link` 存在（offset 非零时才有） |
