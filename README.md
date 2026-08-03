# Linker A7 Lite Tactile Manipulation

![Ubuntu](https://img.shields.io/badge/Ubuntu-22.04-E95420?logo=ubuntu&logoColor=white)
![ROS 2](https://img.shields.io/badge/ROS%202-Humble-22314E?logo=ros&logoColor=white)
![MoveIt 2](https://img.shields.io/badge/MoveIt%202-Planning-2A6DB0)
![License](https://img.shields.io/badge/License-Apache--2.0-2E8B57)

面向 **Linker A7 Lite 右臂 + O6 / L20 Lite 右手** 的 ROS 2 触觉抓取工程。
机械臂由 MoveIt 2 进行碰撞规划，通过原生 `ros2_control` 控制器连续执行轨迹；
灵巧手通过 Linkerbot Python SDK 读取指尖触觉并动态调节夹持力度。

```text
pregrasp -> grasp -> tactile contact -> force regulate
         -> lift -> hold -> place -> release
```

当前提供固定工位水瓶 Demo。后续翻书、插卡和拧灯泡任务可以复用运动规划、
接触检测、力度保持和滑移保护等基础能力。

> [!CAUTION]
> 本项目会驱动真实机械臂。首次运行必须空载或使用轻软物体，清空工作区，
> 保持物理急停和断电开关可触达，并先完成 mock 规划测试。

## 目录

- [功能](#功能)
- [系统架构](#系统架构)
- [硬件与环境](#硬件与环境)
- [快速开始](#快速开始)
- [水瓶 Demo](#水瓶-demo)
- [配置说明](#配置说明)
- [调试工具](#调试工具)
- [ROS 2 接口](#ros-2-接口)
- [常见问题](#常见问题)
- [安全说明](#安全说明)

## 功能

- A7 Lite C++ SocketCAN `ros2_control` 硬件接口
- 100 Hz `joint_trajectory_controller` 连续轨迹跟踪
- MoveIt 2 关节规划、碰撞检查与轨迹执行
- O6 / L20 Lite 可切换灵巧手驱动
- 五指触觉基线、接触检测、力度闭环与滑移保护
- 一条服务命令自动执行完整水瓶抓放流程
- PyQt 五指触觉热力图
- iPhone OctoStream RTSP 相机接入
- iPhone 单目内参和固定相机 eye-to-hand 标定
- A7 Lite mesh、桌面碰撞体和末端手部碰撞包围盒
- rosbag 自动记录机械臂、灵巧手和触觉数据

## 系统架构

```text
                         +----------------------+
                         |       MoveIt 2       |
                         | planning + collision |
                         +----------+-----------+
                                    |
                         FollowJointTrajectory
                                    |
+-------------+ CAN0 +--------------v---------------+
| A7 Lite arm | <--> | ros2_control + A7 C++ driver |
+-------------+      +------------------------------+

+-------------+ CAN1 +-------------------+     +------------------+
| O6/L20 Lite | <--> | Linkerbot SDK hand | --> | tactile control  |
+-------------+      +-------------------+     +---------+--------+
                                                         |
                                               +---------v--------+
                                               | water bottle task |
                                               +------------------+
```

核心包：

| 包 | 作用 |
| --- | --- |
| `linker_a7_ros2_control` | A7 Lite CAN 协议、硬件接口和安全服务 |
| `linker_manip_interfaces` | 自定义 message、service 和 action |
| `linker_manipulation` | 手部驱动、MoveIt 配置、触觉抓取、Demo 和可视化 |

## 硬件与环境

| 项目 | 当前默认值 |
| --- | --- |
| 操作系统 | Ubuntu 22.04 |
| ROS 2 | Humble |
| 机械臂 | A7 Lite 右臂，电机 ID `51..57` |
| 灵巧手 | O6 右手，可切换 L20 Lite |
| 机械臂 CAN | `can0`，1 Mbps |
| 灵巧手 CAN | `can1`，1 Mbps |
| 机械臂关节 | `R1_JOINT` 至 `R7_JOINT` |
| 基坐标系 | `base_link` / SDK `maestro` |
| TCP | `tcp_link` |

仓库结构：

```text
src/
  linker_a7_ros2_control/       # A7 Lite C++ ros2_control hardware
  linker_manip_interfaces/      # ROS 2 messages, services, actions
  linker_manipulation/
    config/
      robot.yaml                # 当前主配置，A7 Lite + O6
      robot_o6.yaml
      robot_l20lite.yaml
      moveit/
      ros2_control/
    launch/
      water_bottle_demo.launch.py
      ros2_control_moveit.launch.py
      iphone_camera.launch.py
      iphone_intrinsic_calibration.launch.py
      iphone_hand_eye_calibration.launch.py
      iphone_calibrated_camera.launch.py
      tactile_heatmap.launch.py
    linker_manipulation/        # Python nodes
    urdf/                       # A7 Lite mesh and collision model
    rviz/
```

## 快速开始

### 1. 安装依赖

```bash
sudo apt-get update
sudo apt-get install -y \
  can-utils \
  python3-pip \
  python3-pyqt5 \
  ros-humble-moveit \
  ros-humble-pinocchio
```

安装 ROS 包依赖：

```bash
cd ~/robot_dev
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
```

安装 Linkerbot Python SDK。A7/A7 Lite 的 SDK 位姿接口需要 `kinetix` / Pinocchio：

```bash
python3 -m pip install --user python-can "pydantic>=2" "linkerbot-py[kinetix]"
python3 -c "import pinocchio; print(pinocchio.__version__)"
```

也可以使用本地 SDK checkout：

```bash
export LINKERBOT_SDK_PATH=/path/to/linkerbot-python-sdk
```

### 2. 构建工作区

```bash
cd ~/robot_dev
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

每个新终端都需要：

```bash
source /opt/ros/humble/setup.bash
source ~/robot_dev/install/setup.bash
```

### 3. 初始化 CAN

建议先连接 CAN 适配器并配置接口，再给机械臂和灵巧手上电。

```bash
sudo ip link set can0 down 2>/dev/null || true
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 txqueuelen 100
sudo ip link set can0 up

sudo ip link set can1 down 2>/dev/null || true
sudo ip link set can1 type can bitrate 1000000
sudo ip link set can1 txqueuelen 100
sudo ip link set can1 up
```

检查接口：

```bash
ip -details -statistics link show can0
ip -details -statistics link show can1
```

正常状态应包含：

```text
state UP
can state ERROR-ACTIVE
bitrate 1000000
qlen 100
```

### 4. 先做 mock 测试

mock 模式不会连接真机，适合检查 URDF、控制器、MoveIt 和桌面碰撞模型：

```bash
ros2 launch linker_manipulation ros2_control_moveit.launch.py \
  use_mock_hardware:=true \
  allow_execution:=true
```

确认以下控制器为 `active`：

```bash
ros2 control list_controllers
```

### 5. 启动真机

关闭 mock launch、旧的 `bringup.launch.py` 以及其他占用 `can0`/`can1` 的进程，
然后启动完整水瓶 Demo 链路：

```bash
ros2 launch linker_manipulation water_bottle_demo.launch.py \
  use_mock_hardware:=false
```

默认同时启动 RViz。无图形环境使用：

```bash
ros2 launch linker_manipulation water_bottle_demo.launch.py \
  use_mock_hardware:=false \
  use_rviz:=false
```

启动后检查：

```bash
ros2 control list_controllers
ros2 topic hz /joint_states
ros2 topic hz /linker/hand/tactile
```

预期结果：

- `joint_state_broadcaster` 为 `active`
- `a7_arm_controller` 为 `active`
- `/joint_states` 持续更新
- `/linker/hand/tactile` 约为 30 Hz

## 水瓶 Demo

### 工位准备

1. 将水瓶放在示教时使用的固定位置。
2. 确认桌面、机械臂和灵巧手模型与现场方向一致。
3. 确保灵巧手未接触物体，便于动作开始时采集触觉基线。
4. 清空机械臂到 `pregrasp` 路径附近的人员和物体。

### 执行一次

另开终端：

```bash
source /opt/ros/humble/setup.bash
source ~/robot_dev/install/setup.bash
ros2 service call /linker/demo/run_once std_srvs/srv/Trigger {}
```

一次调用会自动完成：

```text
open -> tactile baseline -> pregrasp -> grasp
     -> close until contact -> force regulate
     -> lift -> hold -> place -> release
```

中途取消：

```bash
ros2 service call /linker/demo/cancel std_srvs/srv/Trigger {}
```

接触物体后如果流程失败或取消，控制器会保持当前夹持，避免水瓶在半空松脱。
先人工托住物体，再打开灵巧手：

```bash
ros2 service call /linker/hand/open std_srvs/srv/Trigger {}
```

`demo.record_bag: true` 时，每次运行都会在 `bags/` 下创建带时间戳的 rosbag。

## 配置说明

主配置文件为 [`src/linker_manipulation/config/robot.yaml`](src/linker_manipulation/config/robot.yaml)。

### 机械臂和灵巧手

```yaml
arm:
  type: A7lite
  side: right
  can: can0

hand:
  type: O6
  side: right
  can: can1
  tactile_shape: [10, 4]
```

O6 与 L20 Lite 的默认差异：

| 型号 | 驱动关节数 | 单指触觉矩阵 | 配置文件 |
| --- | ---: | ---: | --- |
| O6 | 6 | `10x4` | `robot_o6.yaml` |
| L20 Lite | 10 | `12x6` | `robot_l20lite.yaml` |

切换到 L20 Lite：

```bash
ros2 launch linker_manipulation water_bottle_demo.launch.py \
  use_mock_hardware:=false \
  config_path:=$HOME/robot_dev/src/linker_manipulation/config/robot_l20lite.yaml
```

每个手型配置都独立保存张开角度、预抓角度、速度、扭矩和最大闭合角度。

### MoveIt 关节目标

推荐使用示教得到的关节目标执行固定工位任务：

```yaml
moveit:
  velocity_scaling: 0.40
  acceleration_scaling: 0.35
  joint_targets:
    pregrasp: [...]
    grasp: [...]
    lift: [...]
    place: [...]
```

| 目标 | 含义 |
| --- | --- |
| `pregrasp` | 水瓶上方或前方的安全预抓位置 |
| `grasp` | 手指能够包围水瓶的抓取位置 |
| `lift` | 抓稳后离开桌面的抬升位置 |
| `place` | 放置水瓶的位置 |

记录当前机械臂关节角：

```bash
ros2 run linker_manipulation joint_target_recorder_node --ros-args \
  -p joint_state_topic:=/joint_states \
  -p target_name:=pregrasp
```

将输出写入 `moveit.joint_targets`，再分别记录 `grasp`、`lift` 和 `place`。

单独规划并执行一个目标：

```bash
ros2 run linker_manipulation moveit_goal_node --ros-args \
  -p goal_type:=joint \
  -p joint_target_name:=pregrasp \
  -p execute:=true
```

`velocity_scaling` 和 `acceleration_scaling` 是 MoveIt 轨迹比例。调整后需要重启
launch；使用 `--symlink-install` 时，修改现有 YAML 不需要重新构建。

### 触觉闭环

```yaml
grasp:
  active_fingers: [thumb, index, middle]
  baseline_samples: 10
  contact_threshold: 12.0
  force_low: 20.0
  force_high: 45.0
  hold_sec: 5.0
  slip_drop_ratio: 0.30
```

- 基线：无接触时连续采样，用于扣除传感器零点偏差
- 接触：指尖分数超过 `contact_threshold`
- 调力：触觉低于 `force_low` 时继续闭合，高于 `force_high` 时小幅张开
- 滑移：抬升或保持阶段触觉下降超过 `slip_drop_ratio` 时补偿或中止

### 桌面碰撞模型

当前 URDF 内置桌面：

```text
center: x=0.70, y=0.0, z=-0.325
size:   x=0.8,  y=2.0, z=0.05
top_z:  -0.30
```

末端还包含 `0.12 x 0.10 x 0.18 m` 的保守手部碰撞包围盒。
实际桌面或手部尺寸变化后，必须同步修改
[`a7_lite_right_mesh.urdf`](src/linker_manipulation/urdf/a7_lite_right_mesh.urdf)。

## 调试工具

### iPhone RTSP 相机

确认 Ubuntu 虚拟机已连接 iPhone 热点，并在 OctoStream 中启动视频流。默认地址：

```text
rtsp://172.20.10.1:554/stream
```

启动 ROS 2 相机节点：

```bash
ros2 launch linker_manipulation iphone_camera.launch.py
```

发布的话题：

| Topic | Type |
| --- | --- |
| `/iphone_camera/image_raw` | `sensor_msgs/msg/Image` |
| `/iphone_camera/camera_info` | `sensor_msgs/msg/CameraInfo` |

检查图像：

```bash
ros2 topic hz /iphone_camera/image_raw
ros2 topic echo --once /iphone_camera/camera_info
ros2 run rqt_image_view rqt_image_view /iphone_camera/image_raw
```

当前 OctoStream 视频为竖屏 `720x1280`、30 FPS。如需顺时针旋转为横屏：

```bash
ros2 launch linker_manipulation iphone_camera.launch.py rotation_degrees:=90
```

虚拟机 CPU 或 DDS 带宽不足时，可以降低发布分辨率和帧率：

```bash
ros2 launch linker_manipulation iphone_camera.launch.py \
  resize_width:=360 resize_height:=640 publish_rate_hz:=15.0
```

节点只保留最新视频帧，并在 RTSP 断开后自动重连。未提供 `camera_info_url` 时会发布
未标定的 `CameraInfo`；用于视觉测量和机械臂定位前，需要完成相机内参和手眼标定。
标定前应先确定最终的旋转角度和发布分辨率，标定后不要再改变这两个参数。

### iPhone 单目手眼标定

当前工具按 **eye-to-hand** 配置：iPhone 固定在机械臂外部，求解
`base_link -> iphone_camera_optical_frame`。整个流程使用同一块 ChArUco 标定板。

> [!IMPORTANT]
> OctoStream 必须始终使用同一个镜头、分辨率、旋转角度和缩放比例。标定完成后移动
> iPhone、切换镜头、改变数字变焦或改变 `rotation_degrees` / `resize_*`，都需要重新标定。

#### 1. 打印标定板

生成默认 `7x5` ChArUco 板：方格边长 `30 mm`，标记边长 `22 mm`。

```bash
mkdir -p ~/robot_dev/calibration
ros2 run linker_manipulation charuco_board_generator \
  --output ~/robot_dev/calibration/charuco_7x5_30mm.pdf
```

打印时选择 **A4 横向、实际大小/100%、关闭适应页面**。打印后用尺确认任意方格边长
为 `30.0 mm`，再粘到平整、不会弯曲的硬板上。

#### 2. 标定 iPhone 内参

此阶段标定板可以手持，机械臂不需要启动。

```bash
ros2 launch linker_manipulation iphone_intrinsic_calibration.launch.py \
  rotation_degrees:=0
```

可选：查看角点检测画面。缺少工具时先安装 `ros-humble-rqt-image-view`。

```bash
ros2 run rqt_image_view rqt_image_view \
  /iphone_calibration/intrinsics/debug_image
```

让标定板依次出现在画面中央、四角、近处、远处，并改变俯仰和偏航角。每次保持清晰、
静止后采集一次，建议采集 `20~30` 组：

```bash
ros2 service call /iphone_calibration/intrinsics/capture \
  std_srvs/srv/Trigger {}
```

采集完成后求解：

```bash
ros2 service call /iphone_calibration/intrinsics/solve \
  std_srvs/srv/Trigger {}
```

成功后内参保存为：

```text
~/.ros/camera_info/iphone_15_pro.yaml
```

建议 RMS 小于 `1.0 px`；程序允许的上限为 `1.5 px`。失败时清空样本重新采集：

```bash
ros2 service call /iphone_calibration/intrinsics/reset \
  std_srvs/srv/Trigger {}
```

#### 3. 标定相机到 `base_link`

将 iPhone 刚性固定在工位上。再将标定板刚性固定到机械臂末端，整个采集过程中标定板
不能相对 `tcp_link` 滑动。标定板与 `tcp_link` 之间的具体偏移不需要手工测量。

终端 1 启动真机控制和 TF：

```bash
ros2 launch linker_manipulation ros2_control_moveit.launch.py \
  use_mock_hardware:=false \
  allow_execution:=true
```

终端 2 启动相机和 eye-to-hand 标定节点：

```bash
ros2 launch linker_manipulation iphone_hand_eye_calibration.launch.py \
  rotation_degrees:=0
```

通过 RViz 或 MoveIt 将机械臂移动到 `12~20` 个不同姿态。每个姿态都要满足：

- 标定板完整出现在相机画面内，角点清晰
- 机械臂完全停止后等待约 1 秒再采集
- 位置覆盖约 `10~20 cm`，朝向绕至少两个轴变化 `20~40 deg`
- iPhone 和标定板安装位置全程不变

每个姿态调用一次：

```bash
ros2 service call /iphone_calibration/hand_eye/capture \
  std_srvs/srv/Trigger {}
```

完成后求解：

```bash
ros2 service call /iphone_calibration/hand_eye/solve \
  std_srvs/srv/Trigger {}
```

成功后外参保存为：

```text
~/.ros/camera_info/iphone_to_base.yaml
```

程序默认要求平移一致性 RMS 小于 `30 mm`、旋转一致性 RMS 小于 `3 deg`。结果不合格时：

```bash
ros2 service call /iphone_calibration/hand_eye/reset \
  std_srvs/srv/Trigger {}
```

#### 4. 使用标定结果

后续使用下面的 launch 同时加载内参和静态 TF：

```bash
ros2 launch linker_manipulation iphone_calibrated_camera.launch.py \
  rotation_degrees:=0
```

检查内参和 TF：

```bash
ros2 topic echo --once /iphone_camera/camera_info
ros2 run tf2_ros tf2_echo base_link iphone_camera_optical_frame
```

移动标定板到多个已知位置进行验证。用于抓取前，建议桌面附近的三维定位误差控制在
`10~20 mm` 以内；若误差随距离或画面位置明显变化，优先重新做内参标定。

### 触觉热力图

硬件驱动运行后，在有图形界面的终端执行：

```bash
ros2 launch linker_manipulation tactile_heatmap.launch.py
```

查看原始值：

```bash
ros2 launch linker_manipulation tactile_heatmap.launch.py delta_mode:=false
```

固定颜色范围：

```bash
ros2 launch linker_manipulation tactile_heatmap.launch.py \
  auto_scale:=false cmap:=turbo vmax:=80.0
```

### 单独测试机械臂控制链路

只启动 A7 Lite、MoveIt 和 `ros2_control`：

```bash
ros2 launch linker_manipulation ros2_control_moveit.launch.py \
  use_mock_hardware:=false \
  allow_execution:=true
```

### SDK 直连调试

`bringup.launch.py` 保留用于 SDK 驱动、`MoveArm.action` 和 TCP 位姿调试：

```bash
ros2 launch linker_manipulation bringup.launch.py
```

> [!IMPORTANT]
> SDK `bringup.launch.py` 与原生 `ros2_control` 都会占用机械臂 CAN，不能同时运行。
> 完整水瓶 Demo 应使用 `water_bottle_demo.launch.py`。

SDK 模式下可调用：

```bash
ros2 service call /linker/arm/enable std_srvs/srv/Trigger {}
ros2 service call /linker/arm/home std_srvs/srv/Trigger {}
ros2 topic echo --once /linker/arm/tcp_pose
```

### 测试

```bash
cd ~/robot_dev
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon test --packages-select \
  linker_a7_ros2_control \
  linker_manip_interfaces \
  linker_manipulation
colcon test-result --verbose
```

## ROS 2 接口

### Topics

| Topic | Type | 说明 |
| --- | --- | --- |
| `/joint_states` | `sensor_msgs/msg/JointState` | ros2_control 机械臂关节状态 |
| `/a7_arm_controller/controller_state` | `control_msgs/msg/JointTrajectoryControllerState` | 轨迹跟踪状态 |
| `/linker/arm/state` | `sensor_msgs/msg/JointState` | SDK 模式机械臂状态 |
| `/linker/arm/tcp_pose` | `linker_manip_interfaces/msg/TcpPose` | SDK 模式 TCP 位姿 |
| `/linker/hand/state` | `linker_manip_interfaces/msg/HandState` | 灵巧手角度、速度和扭矩 |
| `/linker/hand/tactile` | `linker_manip_interfaces/msg/HandTactile` | 五指触觉矩阵和分数 |
| `/iphone_camera/image_raw` | `sensor_msgs/msg/Image` | iPhone RTSP 原始图像 |
| `/iphone_camera/camera_info` | `sensor_msgs/msg/CameraInfo` | iPhone 相机内参与分辨率 |

### Services

| Service | Type | 说明 |
| --- | --- | --- |
| `/linker/demo/run_once` | `std_srvs/srv/Trigger` | 执行一次完整 Demo |
| `/linker/demo/cancel` | `std_srvs/srv/Trigger` | 取消当前 Demo |
| `/linker/grasp/calibrate_baseline` | `std_srvs/srv/Trigger` | 重新采集触觉基线 |
| `/linker/hand/open` | `std_srvs/srv/Trigger` | 打开灵巧手 |
| `/linker/hand/set_angles` | `linker_manip_interfaces/srv/SetHandAngles` | 设置手指角度 |
| `/linker/arm/emergency_stop` | `std_srvs/srv/Trigger` | 立即失能机械臂电机 |
| `/linker/arm/disable` | `std_srvs/srv/Trigger` | 失能机械臂电机 |

### Actions

| Action | Type | 说明 |
| --- | --- | --- |
| `/linker/grasp` | `linker_manip_interfaces/action/Grasp` | 触觉抓取状态机 |
| `/linker/arm/move_arm` | `linker_manip_interfaces/action/MoveArm` | SDK 模式关节、Pose 和直线运动 |
| `/a7_arm_controller/follow_joint_trajectory` | `control_msgs/action/FollowJointTrajectory` | ros2_control 轨迹执行 |

## 常见问题

### A7 Lite 电机无响应

典型日志：

```text
Motors [51, 52, 53, 54, 55, 56, 57] did not respond
```

依次检查机械臂供电、急停、CAN 接口对应关系和接收计数：

```bash
ip -details -statistics link show can0
```

若 `TX` 增长而 `RX` 始终为 `0`，优先检查机械臂供电、CAN-H/CAN-L、终端电阻和适配器接线。

### 找不到 controller_manager

```text
Could not contact service /controller_manager/list_controllers
```

说明 `ros2_control_node` 未启动或已经退出。查看启动终端中的第一条错误，并确认没有
其他进程占用 `can0`。

### MoveIt 规划失败

确认关节状态持续发布：

```bash
ros2 topic hz /joint_states
```

然后检查目标关节角是否在限位内、目标或路径是否与桌面碰撞，以及机械臂当前状态是否
与 RViz 一致。固定工位优先使用 `moveit.joint_targets`，Pose goal 仅用于运动学和 TCP
已经准确标定的场景。

### 机械臂运动过慢

调整 `robot.yaml`：

```yaml
moveit:
  velocity_scaling: 0.40
  acceleration_scaling: 0.35
```

这两个值决定 MoveIt 生成轨迹的实际速度比例。修改后重启 launch 生效。

### 热力图没有响应

先检查消息频率和原始数据：

```bash
ros2 topic hz /linker/hand/tactile
ros2 topic echo --once /linker/hand/tactile
```

如果消息持续到达但按压时原始值不变，问题位于触觉硬件、SDK 轮询、CAN 或供电链路，
不是热力图界面。窗口无法打开时检查：

```bash
echo $DISPLAY
```

### 自定义消息类型 invalid

重新加载工作区并刷新 ROS 2 daemon：

```bash
source /opt/ros/humble/setup.bash
source ~/robot_dev/install/setup.bash
ros2 daemon stop
ros2 daemon start
```

## 安全说明

- 首次执行只使用轻软物体，速度从低到高逐步调整
- 真机启动前确认 `use_mock_hardware:=false` 是有意操作
- 不要同时运行两个占用同一 CAN 接口的驱动
- 执行前在 RViz 中确认整条路径不穿过桌面和机械臂自身
- `/linker/arm/emergency_stop` 会直接失能 7 个电机，触发后应重启真机控制链路
- 当前手部碰撞体是保守包围盒，精细操作前应替换为准确手部 URDF
- 当前 `tcp_offset` 为零，翻书、插卡、拧灯泡前必须完成 TCP 标定

## Roadmap

- 水瓶碰撞体附着与放置场景更新
- TCP 和手眼标定工具
- 视觉定位与抓取目标自动生成
- 翻书任务原语
- 卡片对孔与柔顺插入
- 灯泡旋拧与触觉滑移控制

## 致谢

本项目基于 [Linkerbot Python SDK](https://github.com/linker-bot/linkerbot-python-sdk)
开发，并使用 [linker-bot](https://github.com/linker-bot) 公开仓库中的机械臂模型和协议资料。

## License

ROS 包清单声明为 Apache-2.0。
