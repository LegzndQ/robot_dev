# Linker A7 Lite + L20 Lite ROS2 Tactile Grasp Demo

这是一个基于 ROS2 Humble 的机械臂 + 灵巧手触觉抓取实验工程，用于 Linker A7/A7 Lite 机械臂和 Linker L20 Lite 灵巧手的固定工位抓取 demo。

当前默认配置面向：

- Ubuntu 22.04
- ROS2 Humble
- 右臂 `A7lite`
- 右手 `L20lite`
- 机械臂 CAN：`can0`
- 灵巧手 CAN：`can1`
- 坐标系：`maestro`

工程目标是先跑通一个最小闭环：

```text
机械臂到预抓取位姿 -> 手指闭合 -> 指尖触觉调力 -> 抬起 -> 放置 -> 松手
```

后续可以在同一套原语上继续扩展翻书、插卡、拧灯泡等 demo。

## Package Overview

```text
src/
  linker_manip_interfaces/
    msg/
    srv/
    action/
  linker_manipulation/
    config/robot.yaml
    launch/bringup.launch.py
    linker_manipulation/
      a7_driver_node.py
      l20lite_driver_node.py
      grasp_controller_node.py
      demo_task_node.py
```

### `linker_manip_interfaces`

自定义 ROS2 接口包：

- `TcpPose.msg`：机械臂 TCP 位姿，字段为 `x y z rx ry rz`
- `HandState.msg`：L20 Lite 的角度、速度、扭矩、温度
- `FingerTactile.msg`：单个手指的 `12x6` 指尖传感器矩阵和触觉分数
- `HandTactile.msg`：五指触觉数据
- `SetHandAngles.srv`：设置 L20 Lite 10 个关节角度
- `MoveArm.action`：机械臂关节/位姿/直线运动
- `Grasp.action`：触觉抓取状态机

### `linker_manipulation`

运行节点包：

- `a7_driver_node`：封装 A7/A7 Lite SDK，发布机械臂状态和 TCP 位姿
- `l20lite_driver_node`：封装 L20 Lite SDK，发布手部状态和触觉数据
- `grasp_controller_node`：执行触觉闭环抓取
- `demo_task_node`：一键触发固定工位 demo

## Dependencies

### System

```bash
sudo apt-get update
sudo apt-get install -y \
  python3-pip \
  can-utils \
  ros-humble-pinocchio
```

### Python

```bash
python3 -m pip install --user python-can "pydantic>=2"
```

### Linkerbot SDK

本工程会优先导入系统中已安装的 `linkerbot` 包。如果没有安装，会自动 fallback 到：

```text
vendor/linkerbot-python-sdk/src
```

因此仓库中需要保留 Linkerbot Python SDK：

```text
vendor/linkerbot-python-sdk
```

A7/A7 Lite 的运动学依赖 Pinocchio。推荐通过 apt 安装：

```bash
sudo apt-get install -y ros-humble-pinocchio
```

验证：

```bash
python3 -c "import pinocchio; print(pinocchio.__version__)"
```

## CAN Setup

默认：

- `can0`：A7/A7 Lite 机械臂
- `can1`：L20 Lite 灵巧手
- bitrate：`1000000`

启动 CAN：

```bash
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up
sudo ip link set can1 type can bitrate 1000000
sudo ip link set can1 up
```

检查状态：

```bash
ip -details link show can0
ip -details link show can1
```

正常状态应类似：

```text
state UP
can state ERROR-ACTIVE
bitrate 1000000
```

## Build

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

验证接口：

```bash
ros2 interface show linker_manip_interfaces/msg/TcpPose
ros2 interface show linker_manip_interfaces/action/MoveArm
ros2 interface show linker_manip_interfaces/action/Grasp
```

## Configuration

主要配置在：

```text
src/linker_manipulation/config/robot.yaml
```

关键字段：

```yaml
arm:
  type: A7lite
  side: right
  can: can0
  world_frame: maestro
  tcp_offset: [0.0, 0.0, 0.0]

hand:
  type: L20lite
  side: right
  can: can1

grasp:
  force_low: 20.0
  force_high: 45.0
  action_timeout_sec: 75.0
  poses:
    pregrasp_pose: [0.0, 0.30, -0.18, 1.85, 0.0, 1.57]
    grasp_pose: [0.0, 0.30, -0.25, 1.85, 0.0, 1.57]
    lift_pose: [0.0, 0.30, -0.20, 1.85, 0.0, 1.57]
    place_pose: [0.10, 0.30, -0.25, 1.85, 0.0, 1.57]
```

如果修改了源码里的 `robot.yaml`，需要重新构建并重启 launch：

```bash
colcon build --symlink-install --packages-select linker_manipulation
source install/setup.bash
```

然后停止旧 launch，重新启动。

### Pose Meaning

所有 demo pose 均为机械臂 TCP 位姿：

```text
[x, y, z, rx, ry, rz]
```

- `x y z`：位置，单位米
- `rx ry rz`：姿态，单位弧度
- 当前默认坐标系为 `maestro`

几个路点含义：

- `pregrasp_pose`：预抓取位姿，靠近物体但未接触
- `grasp_pose`：抓取位姿，手指在这里闭合并进行触觉调力
- `lift_pose`：抓住后抬起的位置
- `place_pose`：放置位置

注意：`home_pose` 目前是预留配置字段，默认 demo 里实际执行的是 SDK 的 `arm.home()`，也就是 7 个关节回到零位，而不是移动到 `home_pose`。

## Launch

```bash
source /opt/ros/humble/setup.bash
source ~/robot_dev/install/setup.bash
ros2 launch linker_manipulation bringup.launch.py
```

正常日志应包含：

```text
Connected A7lite right arm on can0 (maestro frame).
Connected L20 Lite right hand on can1.
```

## Topics

```bash
ros2 topic list -t
```

主要 topic：

```text
/linker/arm/state        sensor_msgs/msg/JointState
/linker/arm/tcp_pose     linker_manip_interfaces/msg/TcpPose
/linker/hand/state       linker_manip_interfaces/msg/HandState
/linker/hand/tactile     linker_manip_interfaces/msg/HandTactile
```

查看一次数据：

```bash
ros2 topic echo --once /linker/arm/tcp_pose
ros2 topic echo --once /linker/hand/state
ros2 topic echo --once /linker/hand/tactile
```

如果出现 `The passed message type is invalid`，通常是当前终端没有 source 工作区：

```bash
source /opt/ros/humble/setup.bash
source ~/robot_dev/install/setup.bash
ros2 daemon stop
ros2 daemon start
```

## Basic Hardware Tests

### Enable and Home Arm

```bash
ros2 service call /linker/arm/enable std_srvs/srv/Trigger {}
ros2 service call /linker/arm/home std_srvs/srv/Trigger {}
```

成功响应示例：

```text
success=True, message='A7lite arm enabled'
success=True, message='A7lite arm homed'
```

### Open Hand

```bash
ros2 service call /linker/hand/open std_srvs/srv/Trigger {}
```

### Test Only `pregrasp_pose`

这只移动机械臂，不执行抓取：

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

## Tactile Baseline

抓取前建议先做触觉基线校准：

```bash
ros2 service call /linker/grasp/calibrate_baseline std_srvs/srv/Trigger {}
```

含义是：在手指没有接触物体时，记录一小段指尖传感器空载值，后续抓取时使用：

```text
当前触觉值 - 空载基线值 = 接触/受力增量
```

这会让接触检测、力度调节和滑移检测更稳定。

## Run Demo

确认机械臂运动空间安全、手边有急停后，执行：

```bash
ros2 service call /linker/demo/run_once std_srvs/srv/Trigger {}
```

默认 demo 流程：

```text
enable
home
open hand
calibrate tactile baseline
move to pregrasp_pose
set pregrasp hand angles
move linearly to grasp_pose
close until tactile contact
regulate force
lift
hold
place
release
```

## Troubleshooting

### A7/A7 Lite Motors Did Not Respond

现象：

```text
Motors [51, 52, 53, 54, 55, 56, 57] did not respond
```

检查：

- `arm.type` 是否正确：`A7` 或 `A7lite`
- `arm.side` 是否正确：右臂为 `right`
- 机械臂是否真的接在 `can0`
- 机械臂电源和急停是否正常
- CAN 是否 `UP` 且 `ERROR-ACTIVE`

### Action Timeout During Approach

现象：

```text
MoveArm phase timed out: approach
```

可能原因：

- 机械臂起点离 `pregrasp_pose` 很远
- 速度设置较低，`action_timeout_sec` 太短
- IK 规划或实体运动耗时较长

可以调大：

```yaml
grasp:
  action_timeout_sec: 75.0
```

### Hand Set Angles Ambiguous Truth Value

旧版本曾出现：

```text
The truth value of an array with more than one element is ambiguous
```

这是 ROS2 固定长度数组被当作 NumPy 数组时触发的 Python 判断问题，当前代码已修复。

### `home_pose` Does Not Affect Route

当前 demo 使用的是 SDK：

```text
arm.home()
```

它让 7 个关节回到零位，不读取 YAML 里的 `home_pose`。如果希望从当前位置直接跑 demo，可以配置：

```yaml
demo:
  enable_and_home: false
```

## Safety Notes

- 第一次运行 demo 时请使用轻软物体，例如海绵块、泡沫块、小纸盒。
- 不要一开始抓硬物、贵重物或易碎物。
- 调 pose 时优先使用较高的 `z`，确认安全后再逐步降低。
- `tcp_offset` 当前默认为 `[0, 0, 0]`，因此 TCP 可能是机械臂默认末端/法兰附近，不一定是灵巧手指尖中心。要做精细操作，需要后续标定灵巧手相对法兰的偏移。

## Roadmap

- 标定 L20 Lite 抓取中心的 `tcp_offset`
- 增加示教保存工具，把当前 `/linker/arm/tcp_pose` 写入 YAML
- 增加 MoveIt2/碰撞模型支持
- 增加翻书、插卡、拧灯泡任务模板
- 增加 rosbag 自动记录和触觉曲线分析工具
