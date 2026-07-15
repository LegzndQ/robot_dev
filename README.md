# Linker A7 Lite + LinkerHand Tactile Grasp

![ROS 2](https://img.shields.io/badge/ROS%202-Humble-22314E)
![Ubuntu](https://img.shields.io/badge/Ubuntu-22.04-E95420)
![License](https://img.shields.io/badge/License-Apache--2.0-green)

基于 ROS2 Humble 的 Linker A7/A7 Lite 机械臂 + LinkerHand 灵巧手触觉抓取工程。

当前目标是先跑通一个固定工位、无视觉的稳定抓取闭环：

```text
approach -> tactile close -> force regulate -> lift -> place -> release
```

后续翻书、插卡、拧灯泡等 demo 可以复用同一套运动、接触检测和触觉调力原语。

## Highlights

- A7/A7 Lite 机械臂 ROS2 driver
- O6 / L20 Lite 灵巧手 ROS2 driver
- `MoveArm.action`：关节、位姿、直线运动
- `Grasp.action`：接触检测、力度闭环、滑移保护
- YAML 配置固定工位 demo 位姿
- PyQt 五指触觉热力图
- RViz2 A7 Lite mesh 可视化
- SocketCAN 双总线 bring-up 流程

## Hardware Defaults

| 项目 | 默认值 |
| --- | --- |
| 系统 | Ubuntu 22.04 |
| ROS | ROS2 Humble |
| 机械臂 | 右臂 `A7lite` |
| 灵巧手 | 右手 `O6`，可切换 `L20lite` |
| Arm CAN | `can0`, 1 Mbps |
| Hand CAN | `can1`, 1 Mbps |
| 坐标系 | `maestro` |

## Repository Layout

```text
src/
  linker_manip_interfaces/
    msg/                 # TcpPose, HandState, tactile messages
    srv/                 # SetHandAngles
    action/              # MoveArm, Grasp
  linker_manipulation/
    config/robot.yaml
    launch/
      bringup.launch.py
      tactile_heatmap.launch.py
      visualize.launch.py
    linker_manipulation/
      a7_driver_node.py
      l20lite_driver_node.py   # generic hand driver entry, supports O6 / L20 Lite
      grasp_controller_node.py
      demo_task_node.py
      tactile_heatmap_node.py
    rviz/
    urdf/
```

## Quick Start

### 1. Install Dependencies

```bash
sudo apt-get update
sudo apt-get install -y \
  can-utils \
  python3-pip \
  python3-pyqt5 \
  ros-humble-pinocchio
```

```bash
python3 -m pip install --user python-can "pydantic>=2"
```

A7/A7 Lite 需要 Linkerbot SDK 的 `kinetix` / Pinocchio 支持：

```bash
# 如果环境中可直接安装 SDK
python3 -m pip install --user "linkerbot-py[kinetix]"

# 或使用本地 SDK checkout
export LINKERBOT_SDK_PATH=/path/to/linkerbot-python-sdk
```

检查 Pinocchio：

```bash
python3 -c "import pinocchio; print(pinocchio.__version__)"
```

### 2. Build

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

### 3. Bring Up CAN

```bash
sudo ip link set can0 down 2>/dev/null || true
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up

sudo ip link set can1 down 2>/dev/null || true
sudo ip link set can1 type can bitrate 1000000
sudo ip link set can1 up
```

检查：

```bash
ip -details -statistics link show can0
ip -details -statistics link show can1
```

正常应看到：

```text
state UP
can state ERROR-ACTIVE
bitrate 1000000
```

### 4. Launch

```bash
ros2 launch linker_manipulation bringup.launch.py
```

正常日志：

```text
Connected A7lite right arm on can0 (maestro frame).
Connected O6 right hand on can1.
```

## Configuration

主配置文件：

```text
src/linker_manipulation/config/robot.yaml
```

核心字段：

```yaml
arm:
  type: A7lite
  side: right
  can: can0
  world_frame: maestro

hand:
  type: O6
  side: right
  can: can1
  joint_count: 6
  tactile_shape: [10, 4]

grasp:
  active_fingers: [thumb, index, middle]
  force_low: 20.0
  force_high: 45.0
  approach_mode: pose
  approach_waypoints: [safe_above_table_pose]
  poses:
    safe_above_table_pose: [0.0, 0.20, -0.12, 1.85, 0.0, 1.57]
    pregrasp_pose: [0.0, 0.30, -0.15, 1.85, 0.0, 1.57]
    grasp_pose: [0.0, 0.30, -0.21, 1.85, 0.0, 1.57]
    lift_pose: [0.0, 0.30, -0.12, 1.85, 0.0, 1.57]
    place_pose: [0.10, 0.30, -0.18, 1.85, 0.0, 1.57]
```

位姿格式：

```text
[x, y, z, rx, ry, rz]
```

- `x y z`：TCP 位置，单位米
- `rx ry rz`：TCP 姿态，单位弧度
- 当前默认坐标系为 `maestro`

修改 `robot.yaml` 后，建议重建并重启 launch：

```bash
colcon build --symlink-install --packages-select linker_manipulation
source install/setup.bash
```

也可以直接指定源码里的配置文件：

```bash
ros2 launch linker_manipulation bringup.launch.py \
  config_path:=/home/chuanqi/robot_dev/src/linker_manipulation/config/robot.yaml
```

切换手型：

```bash
# O6, 当前默认
ros2 launch linker_manipulation bringup.launch.py \
  config_path:=/home/chuanqi/robot_dev/src/linker_manipulation/config/robot_o6.yaml

# L20 Lite
ros2 launch linker_manipulation bringup.launch.py \
  config_path:=/home/chuanqi/robot_dev/src/linker_manipulation/config/robot_l20lite.yaml
```

## Basic Control

查看主要话题：

```bash
ros2 topic list -t
ros2 topic echo --once /linker/arm/tcp_pose
ros2 topic echo --once /linker/hand/state
ros2 topic echo --once /linker/hand/tactile
```

机械臂上使能和回零：

```bash
ros2 service call /linker/arm/enable std_srvs/srv/Trigger {}
ros2 service call /linker/arm/home std_srvs/srv/Trigger {}
```

打开灵巧手：

```bash
ros2 service call /linker/hand/open std_srvs/srv/Trigger {}
```

急停：

```bash
ros2 service call /linker/arm/emergency_stop std_srvs/srv/Trigger {}
```

只测试 `pregrasp_pose`：

```bash
ros2 action send_goal /linker/arm/move_arm linker_manip_interfaces/action/MoveArm "{
  mode: 1,
  target_joints: [],
  target_pose: {x: 0.0, y: 0.30, z: -0.18, rx: 1.85, ry: 0.0, rz: 1.57},
  joint_velocities: [0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15],
  joint_accelerations: [2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0],
  max_velocity: 0.0,
  max_angular_velocity: 0.0,
  acceleration: 0.0,
  angular_acceleration: 0.0
}"
```

## Run Demo

第一次运行请使用海绵、泡沫块、小纸盒等轻软物体，并确保急停可用。

校准触觉基线：

```bash
ros2 service call /linker/grasp/calibrate_baseline std_srvs/srv/Trigger {}
```

执行一次抓取：

```bash
ros2 service call /linker/demo/run_once std_srvs/srv/Trigger {}
```

默认流程：

```text
enable -> home -> open -> calibrate baseline -> pregrasp
-> grasp -> close until contact -> force regulate
-> lift -> hold -> place -> release
```

## Tactile Heatmap

启动硬件后，另开一个有图形界面的终端：

```bash
ros2 launch linker_manipulation tactile_heatmap.launch.py
```

窗口显示五个指尖触觉矩阵。O6 为 `10x4`，L20 Lite 为 `12x6`。

| 字段 | 含义 |
| --- | --- |
| `msg #` | 收到的触觉消息数 |
| `raw max` | SDK 原始触觉最大值 |
| `view max` | 扣除基线后的显示最大值 |
| `Delta` | 显示 `当前值 - 基线` |
| `Set baseline` | 重新记录无接触基线 |
| `Auto scale` | 自动调整颜色范围 |

常用参数：

```bash
# 看原始值
ros2 launch linker_manipulation tactile_heatmap.launch.py delta_mode:=false

# 换配色并固定上限
ros2 launch linker_manipulation tactile_heatmap.launch.py cmap:=turbo vmax:=80.0
```

如果按压手指时 `raw max` 不变，问题不在 GUI，而在触觉模块、SDK 轮询、CAN 接线或灵巧手供电链路。

## RViz2 Visualization

启动硬件：

```bash
ros2 launch linker_manipulation bringup.launch.py
```

启动 RViz2：

```bash
ros2 launch linker_manipulation visualize.launch.py
```

SSH 或无显示环境只发布 TF：

```bash
ros2 launch linker_manipulation visualize.launch.py use_rviz:=false
```

数据流：

```text
/linker/arm/state -> robot_state_publisher -> /tf -> rviz2
```

默认模型：

```text
src/linker_manipulation/urdf/a7_lite_right_mesh.urdf
```

## MoveIt2 Table Collision

URDF 已内置桌子 collision box：

```text
center: x=0.70, y=0.0, z=-0.325
size:   x=0.8, y=2.0, z=0.05
top_z:  -0.30
```

URDF 同时包含一个临时末端手部碰撞包围盒：

```text
link:   hand_collision_proxy
size:   x=0.12, y=0.10, z=0.18
origin: tcp_link 下方 0.09m
```

这是 O6/L20 Lite 的保守近似模型，用来避免末端低空扫过桌面。拿到精确手部 URDF 后应替换为真实手模型，或按实测尺寸调整这个 proxy。

启动 MoveIt2 规划/可视化：

```bash
ros2 launch linker_manipulation bringup.launch.py
ros2 launch linker_manipulation moveit_table.launch.py
```

无图形界面只启动 `move_group` 和 TF：

```bash
ros2 launch linker_manipulation moveit_table.launch.py use_rviz:=false
```

这个 launch 会启动：

- `robot_state_publisher`
- `move_group`
- 可选 `rviz2`

并将 MoveIt 的 `joint_states` remap 到：

```text
/linker/arm/state
```

当前 MoveIt2 配置用于规划和碰撞检查，默认 `allow_trajectory_execution=false`，不会直接控制真实机械臂。执行到硬件前，需要先确认规划轨迹安全，再接入轨迹到 SDK `move_j` waypoint 的执行层。

## Public Interfaces

主要话题：

| Topic | Type |
| --- | --- |
| `/linker/arm/state` | `sensor_msgs/msg/JointState` |
| `/linker/arm/tcp_pose` | `linker_manip_interfaces/msg/TcpPose` |
| `/linker/hand/state` | `linker_manip_interfaces/msg/HandState` |
| `/linker/hand/tactile` | `linker_manip_interfaces/msg/HandTactile` |

主要服务：

- `/linker/arm/enable`
- `/linker/arm/disable`
- `/linker/arm/home`
- `/linker/arm/emergency_stop`
- `/linker/hand/open`
- `/linker/hand/set_angles`
- `/linker/grasp/calibrate_baseline`
- `/linker/demo/run_once`

主要 action：

- `/linker/arm/move_arm`
- `/linker/grasp/grasp`

## Troubleshooting

### 自定义消息类型 invalid

通常是终端没有 source 工作区：

```bash
source /opt/ros/humble/setup.bash
source ~/robot_dev/install/setup.bash
ros2 daemon stop
ros2 daemon start
```

### A7/A7 Lite motors did not respond

典型日志：

```text
Motors [51, 52, 53, 54, 55, 56, 57] did not respond
```

检查：

- 机械臂电源和急停
- `robot.yaml` 中的 `arm.type`、`arm.side`、`arm.can`
- 机械臂是否确实接在 `can0`
- `ip -details -statistics link show can0`

### 触觉热力图没反应

先看 topic 是否在发布：

```bash
ros2 topic hz /linker/hand/tactile
```

再打印五指统计值：

```bash
python3 - <<'PY'
import rclpy
from linker_manip_interfaces.msg import HandTactile

rclpy.init()
node = rclpy.create_node("tactile_probe")

def cb(msg):
    parts = []
    for f in msg.fingers:
        vals = list(f.values)
        parts.append(f"{f.finger}: score={f.score:.1f}, max={max(vals)}, mean={sum(vals)/len(vals):.1f}")
    print(" | ".join(parts), flush=True)

node.create_subscription(HandTactile, "/linker/hand/tactile", cb, 10)
rclpy.spin(node)
PY
```

如果按压指尖时 `max` 不变，说明底层触觉数据没有变化，优先检查触觉硬件、SDK 轮询、CAN 和供电。

### Demo 路线不自然

- `demo.enable_and_home: true` 会在 demo 前调用 SDK `arm.home()`
- `home_pose` 目前只是 YAML 参考位姿，不会覆盖 `arm.home()`
- `grasp.approach_mode: pose` 使用 `move_p`
- `grasp.approach_mode: linear` 使用 `move_l`
- 如需更自然的路线，可在 `grasp.approach_waypoints` 增加安全中间位姿

## Safety

- 首次运行只抓轻软物体
- 调 pose 时降低速度，先高后低
- 确保急停和机械臂活动空间安全
- 运行 `/linker/demo/run_once` 前清空工作区
- 当前 `tcp_offset` 为 `[0, 0, 0]`，精细操作前需要标定手爪相对法兰的 TCP

## Roadmap

- 示教工具：保存当前 `/linker/arm/tcp_pose` 到 YAML
- 灵巧手抓取中心 TCP 标定
- rosbag 记录与触觉曲线分析
- MoveIt2 和碰撞模型集成
- 翻书、插卡、拧灯泡任务模板

## License

Apache-2.0, as declared by the ROS package manifests.
