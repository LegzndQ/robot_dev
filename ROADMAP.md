# Linker A7 Lite 触觉操作项目路线图

本文档描述 A7 Lite 右臂搭配 O6 / L20 Lite 灵巧手的后续开发方向。
目标是将当前固定工位水瓶 Demo，从“可以运行”提升为稳定、可复用、可扩展的
触觉操作平台，并为翻书、插卡和拧灯泡等精细操作建立基础。

## 当前基础

项目目前已经具备：

- A7 Lite 原生 C++ SocketCAN `ros2_control` 硬件接口
- 100 Hz `joint_trajectory_controller` 轨迹执行
- MoveIt 2 运动规划与桌面碰撞检查
- O6 / L20 Lite 灵巧手切换能力
- 五指触觉采集、基线扣除、接触检测和力度闭环
- 固定工位水瓶抓取、抬升、保持、放置和释放流程
- RViz 机械臂可视化与 PyQt 触觉热力图
- rosbag 自动记录

## 总体目标

```text
可运行 Demo
    -> 可重复、可诊断
    -> 可配置任务平台
    -> 视觉与柔顺操作
    -> 翻书、插卡、拧灯泡
```

| 阶段 | 重点 | 阶段目标 |
| --- | --- | --- |
| P0 | 标定与稳定性 | 水瓶连续抓取 30 次，成功率不低于 90% |
| P1 | 场景与任务抽象 | 不修改代码即可切换物体、手型和任务流程 |
| P2 | 视觉与柔顺控制 | 物体位置变化后仍能自主接近、抓取或插入 |
| P3 | 精细操作场景 | 完成翻书、插卡和拧灯泡 Demo |

## P0：稳定可靠的水瓶 Demo

### 1. TCP 与机器人模型标定

当前 `tcp_offset` 为零，灵巧手碰撞模型也是保守包围盒。需要完成：

- 标定法兰到灵巧手抓取中心的 TCP
- 为 O6 和 L20 Lite 分别保存 TCP
- 建立准确的 O6 / L20 Lite URDF 和碰撞模型
- 实测桌面相对 `base_link` 的位置和姿态
- 校验七个关节的零位、方向和 offset
- 校验 RViz、MoveIt 和真机关节姿态一致性

交付物：

- `config/calibration/o6_tcp.yaml`
- `config/calibration/l20lite_tcp.yaml`
- 精确灵巧手 URDF/mesh
- TCP 与关节零位标定工具

验收标准：

- TCP 重复到达误差满足固定工位抓取需求
- RViz 与真机运动方向一致
- 手部和桌面碰撞模型不出现明显穿模

### 2. 动态 Planning Scene

新增 `planning_scene_manager_node`：

1. 抓取前将水瓶作为圆柱体加入 MoveIt 场景。
2. 抓稳后将水瓶附着到 `tcp_link`。
3. 搬运过程中检查水瓶、桌面和机械臂之间的碰撞。
4. 放置后将水瓶从 TCP 分离并重新加入环境。
5. 任务结束后清理临时碰撞物体。

MoveIt Planning Scene 支持物体的添加、移除、附着和分离：
[MoveIt Planning Scene ROS API](https://moveit.picknik.ai/main/doc/examples/planning_scene_ros_api/planning_scene_ros_api_tutorial.html)

交付物：

- `/linker/scene/add_object`
- `/linker/scene/attach_object`
- `/linker/scene/detach_object`
- `/linker/scene/remove_object`
- 水瓶、卡片、书本和灯泡的几何配置

### 3. 启动前检查

新增 `system_preflight_node`，执行 Demo 前自动检查：

- `can0/can1` 是否存在并处于 UP 状态
- A7 Lite 七个电机状态是否持续更新
- `joint_state_broadcaster` 和 `a7_arm_controller` 是否 active
- 触觉消息频率和时间戳是否正常
- 五指触觉基线噪声是否在允许范围内
- 当前关节是否在限位内
- MoveIt 场景是否包含桌面
- 目标关节是否存在且可规划
- 急停或失能状态是否已经触发

检查不通过时必须拒绝执行，并返回明确的失败原因。

交付物：

- `/linker/system/preflight` service
- `/linker/system/health` topic
- CAN、关节、触觉和控制器诊断信息

### 4. 抓取指标与回归测试

新增 `grasp_metrics_node`，每次 Demo 记录：

- 成功或失败
- 失败阶段和原因
- 各阶段耗时
- 每指首次接触时间
- 每指峰值、均值和稳定值
- 抬升阶段触觉下降比例
- 轨迹跟踪误差
- CAN 超时、丢帧和错误计数
- 物体是否发生滑落

建议建立固定回归测试：

| 测试 | 次数 | 通过条件 |
| --- | ---: | --- |
| 空载运动 | 10 | 无碰撞、无超限、无控制器中止 |
| 轻水瓶抓取 | 30 | 成功率不低于 90% |
| 触觉无接触 | 5 | 正确超时，不继续抬升 |
| 抓取力过高 | 5 | 自动张开或安全中止 |
| 抬升滑移 | 5 | 补偿夹紧或保持抓取后中止 |
| CAN 中断 | 3 | 停止执行并报告具体总线 |
| 急停 | 3 | 七个电机立即失能 |

## P1：可配置任务平台

### 5. 统一灵巧手抽象

为 O6 和 L20 Lite 建立统一接口：

```text
open()
set_angles()
read_state()
read_tactile()
normalize_force()
close_until_contact()
regulate_force()
detect_slip()
```

上层任务只选择手型和抓取 profile，不直接依赖关节数量或触觉矩阵尺寸。

建议 profile：

```yaml
profiles:
  bottle:
    active_fingers: [thumb, index, middle]
    force_band: [20.0, 45.0]
  card:
    active_fingers: [thumb, index]
    force_band: [8.0, 18.0]
  page:
    active_fingers: [thumb, index]
    force_band: [4.0, 10.0]
  bulb:
    active_fingers: [thumb, index, middle, ring, pinky]
    force_band: [25.0, 55.0]
```

### 6. YAML 任务原语

把任务流程从单个状态机中抽离，改为可配置任务：

```yaml
task: water_bottle
steps:
  - move_joint: pregrasp
  - move_cartesian: grasp
  - tactile_close: bottle
  - attach_object: bottle
  - move_joint: lift
  - tactile_hold: 5.0
  - move_joint: place
  - detach_object: bottle
  - open_hand: true
```

建议支持的任务原语：

- `move_joint`
- `move_cartesian`
- `servo_until_contact`
- `tactile_close`
- `tactile_hold`
- `wait`
- `attach_object`
- `detach_object`
- `open_hand`
- `retry`
- `recover`

复杂任务可以接入 MoveIt Task Constructor，将任务拆分为相互依赖的规划阶段：
[MoveIt Task Constructor](https://moveit.picknik.ai/main/doc/concepts/moveit_task_constructor/moveit_task_constructor.html)

### 7. 示教与调试界面

建立统一 PyQt 调试面板：

- 实时查看机械臂关节和 TCP
- 查看五指触觉热力图
- 使能、失能和急停
- 打开灵巧手、设置手指角度
- 记录当前关节目标
- 编辑并保存任务 YAML
- 单步规划和执行
- 展示当前任务阶段与失败原因
- 查看最近一次抓取指标和 rosbag 路径

## P2：视觉与柔顺控制

### 8. 视觉定位

建议分阶段实现：

1. 使用 AprilTag/ArUco 完成固定工位坐标标定。
2. 接入 RGB-D 相机并估计桌面平面。
3. 完成相机到 `base_link` 的手眼标定。
4. 检测水瓶、书本、卡片和灯座位姿。
5. 根据物体位姿动态生成 `pregrasp` 和 `grasp`。
6. 将检测结果同步到 MoveIt Planning Scene。

第一阶段只需要解决厘米级工位偏差，不必立即实现通用物体识别。

### 9. MoveIt Servo

MoveIt 负责较大范围的无碰撞规划，末端接触前的最后几厘米使用 MoveIt Servo
进行实时微调：

```text
MoveIt planning: home -> pregrasp
MoveIt Servo:    pregrasp -> contact/search
Tactile loop:    grasp/hold/slip recovery
```

MoveIt Servo 支持关节速度、末端速度和末端 Pose 命令，并包含碰撞、奇异点、
关节限位和运动平滑检查：
[MoveIt Servo](https://moveit.picknik.ai/main/doc/examples/realtime_servo/realtime_servo_tutorial.html)

适用场景：

- 根据视觉误差持续修正 TCP
- 接近卡槽时低速搜索
- 指尖接触后立即停止
- 翻页过程中的切向滑动
- 灯泡轴线的小范围对准

### 10. 腕部六维力传感器与柔顺控制

指尖触觉适合判断局部接触和抓紧程度，但插卡和拧灯泡还需要测量末端整体受力
与扭矩。建议增加六维 F/T 传感器，并接入 `admittance_controller`。

官方控制器支持 TCP wrench 输入以及六个方向的质量、阻尼和刚度配置：
[ros2_control Admittance Controller](https://control.ros.org/humble/doc/ros2_controllers/admittance_controller/doc/userdoc.html)

开发顺序：

1. 标定传感器零点、坐标系和重力补偿。
2. 发布标准 wrench state interface。
3. 先开放单轴柔顺，例如插卡方向。
4. 增加最大力、最大位移和最大速度限制。
5. 再逐步开放其他方向，避免六轴同时调试。

## P3：精细操作 Demo

### 11. 翻书

关键能力：

- 视觉定位页角和书脊
- 指尖低力接触
- 沿页面切向滑动
- 单页分离检测
- 翻页过程持续触觉保持
- 页面脱离或多页粘连后的恢复

建议流程：

```text
locate corner -> approach -> light contact -> slide
              -> lift one page -> turn -> release
```

### 12. 插卡

关键能力：

- 双指精细夹持
- 卡片和插槽视觉定位
- 卡片平面与槽口姿态对齐
- 低速接触检测
- 柔顺插入和小范围搜索
- 侧向力与插入力限制

建议流程：

```text
grasp card -> visual align -> servo approach
           -> contact -> compliant search -> insert
```

### 13. 拧灯泡

关键能力：

- 灯泡轴线检测
- 五指包络抓取
- 旋转轴对齐
- 腕部扭矩上限
- 指尖滑移检测和重新握持
- 多次旋转后的角度累计

建议流程：

```text
align axis -> enveloping grasp -> rotate
           -> detect slip/torque -> regrasp -> finish
```

需要提前确认 A7 Lite 腕部关节行程、线缆缠绕限制和灯泡最大允许扭矩。

## 推荐实施顺序

### Sprint 1：安全和可观测性

- `system_preflight_node`
- 统一错误码和失败阶段
- 触觉、CAN 和控制器诊断
- 水瓶 Demo 指标记录

### Sprint 2：场景闭环

- 水瓶 CollisionObject
- attach/detach
- TCP 和桌面标定
- 30 次连续抓取回归测试

### Sprint 3：任务抽象

- O6/L20 Lite 统一适配层
- 抓取 profile
- YAML 任务原语
- 示教与调试面板

### Sprint 4：视觉和接触运动

- 相机外参和手眼标定
- 水瓶动态位姿
- MoveIt Servo 最后几厘米接近
- 视觉与触觉联合停止条件

### Sprint 5：精细操作

- 腕部六维力传感器
- 单轴柔顺插入
- 插卡 Demo
- 翻书和拧灯泡原型

## 最近的三个开发任务

建议从以下三个模块开始：

1. `system_preflight_node`：避免在 CAN、关节或触觉异常时启动任务。
2. `planning_scene_manager_node`：实现水瓶的 add、attach、detach 和 remove。
3. `grasp_metrics_node`：自动统计成功率、阶段耗时和失败原因。

完成这三个模块后，项目将从固定脚本转变为具备安全检查、场景闭环和量化验证的
触觉操作平台。
