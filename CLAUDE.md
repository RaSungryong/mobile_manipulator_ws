# Mobile Manipulator Workspace — 项目指南

## 项目概述

本项目是一个 **移动操作机器人系统**，结合移动底盘与 Fairino FR10v6 六轴协作机械臂，实现：

- 基于 AprilTag 视觉标记的自主导航（纯追踪算法）
- 指定位置的机械臂扫描任务
- 表面粗糙度（Ra 值）的在线预测（ResNet3D + ONNX Runtime）
- 支持真实机器人与仿真（Gazebo / Isaac Sim）两种模式

---

## 技术栈

| 类别 | 技术 |
|------|------|
| 机器人操作系统 | ROS Noetic（Ubuntu 20.04） |
| 编程语言 | Python 3.8，C++17（GCC 9.4） |
| 构建系统 | Catkin（CMake 3.16） |
| 运动规划 | MoveIt!（FR10v6 配置） |
| 硬件控制 | ros_control，Fairino SDK（XML-RPC/TCP） |
| 视觉定位 | dt_apriltags（tag36h11，60mm 标记） |
| 相机支持 | Basler（PyPylon）/ USB 相机（OpenCV） |
| 距离传感器 | Keyence DL-EN1（TCP，端口 64000） |
| 机器学习推理 | ONNX Runtime（CPU），ResNet3D |
| 坐标变换 | scipy，numpy |

---

## 目录结构

```
mobile_manipulator_ws/
├── src/
│   ├── apriltag_nav/               # 核心导航与任务执行包
│   │   ├── scripts/                # Python 节点（主要业务逻辑）
│   │   ├── config/                 # 机器人参数与地图配置
│   │   ├── task/csv/               # 扫描任务 CSV 定义文件
│   │   ├── model/                  # ML 模型权重（已排除在 Claude 上下文外）
│   │   └── launch/                 # 启动文件
│   ├── robot_msgs/                 # 自定义 ROS 消息定义
│   ├── fairino_sdk/                # Fairino 机器人 SDK 封装
│   └── frcobot_ros/                # 机械臂相关包集合
│       ├── frcobot_description/    # URDF/Xacro 机器人模型
│       ├── frcobot_hw/             # C++ 硬件状态节点
│       ├── fr10v6_moveit_config/   # MoveIt 标准配置
│       ├── fr10v6_vision_moveit_config/        # MoveIt 视觉配置
│       ├── fr10v6_vision_251219_moveit_config/ # MoveIt 最新视觉配置
│       └── ros_control_boilerplate/frrobot_control/ # ros_control 硬件接口（C++）
├── build/                          # 构建产物（不纳入版本控制）
└── devel/                          # Catkin 开发空间（不纳入版本控制）
```

---

## 核心模块说明

### 1. `task_executor.py` — 任务协调器

**路径**：`src/apriltag_nav/scripts/task_executor.py`

系统主入口节点。维护状态机（IDLE / MOVING / ARRIVED / SCANNING / SCAN_DONE / ERROR），协调移动底盘与机械臂的顺序执行。

**ROS 接口**：
- 订阅 `/task_command` (String) — 接收外部任务指令
- 订阅 `/scan_finished` (Bool) — 接收机械臂扫描完成信号

**支持的指令**：
```
TASK <name>       # 执行预定义任务（如 scan_joints_line1）
GOTO <tag_id>     # 导航到指定 AprilTag
STOP              # 紧急停止
STATE             # 查询当前状态
EXEC <python>     # 调试：执行任意 Python 代码
EVAL <expr>       # 调试：求值 Python 表达式
```

---

### 2. `robot_controller.py` — 移动底盘导航

**路径**：`src/apriltag_nav/scripts/robot_controller.py`

实现基于 AprilTag 的视觉伺服导航，采用纯追踪（Pure Pursuit）算法与 S 型速度曲线。

**ROS 接口**：
- 订阅 `/rgb` (Image)，`/camera_info` (CameraInfo)，`/odom` (Odometry)
- 发布 `/cmd_vel` (Twist) — 底盘速度指令
- 发布 `/robot_pose` (Pose2DWithFlag) — 机器人位姿（供机械臂使用）

**关键参数**（`config/robot.yaml`）：
- 最大线速度：0.05 m/s；最大角速度：0.25 rad/s
- 前向 Pure Pursuit 增益：1.0；后向增益：0.8
- 导航超时：8 秒

---

### 3. `arm_controllerrealwithscan.py` — 真实机械臂控制

**路径**：`src/apriltag_nav/scripts/arm_controllerrealwithscan.py`

用于真实机器人的机械臂控制节点，集成扫描、ONNX 推理与 Keyence 闭环控制。

**ROS 接口**：
- 订阅 `/robot_pose` (Pose2DWithFlag)，`keyence/value` (Float32)
- 发布 `/scan_finished` (Bool)，`/scan/ra_value` (Float32)，`/scan/point_result` (String)

**相机优先级**：Basler（PyPylon） → USB 相机（OpenCV）

**仿真版本**：`arm_controller.py`（使用 MoveIt Action Client，不含传感器逻辑）

---

### 4. `map_manager.py` — 地图与路径规划

**路径**：`src/apriltag_nav/scripts/map_manager.py`

管理 AprilTag 拓扑地图（`config/map.yaml`），实现 BFS 最短路径规划。

**地图概述**：
- 151 个 AprilTag（ID：100–147，400–415，500–508）
- Tag 类型：DOCK、PIVOT、MOVE、WORK
- 工作区域：A（入口水平走廊）、B/D（左侧）、C/E（右侧）

---

### 5. `task_manager.py` — 任务加载器

**路径**：`src/apriltag_nav/scripts/task_manager.py`

从 `task/csv/` 目录加载 CSV 扫描任务定义，并支持运行时动态生成 GOTO 任务。

**扫描点模式**：
- `joint` 模式：直接指定关节角度（弧度）
- `pose` 模式：笛卡尔位姿（x, y, z, rx, ry, rz）

---

### 6. `keyence_dlen1_node.py` — 距离传感器节点

**路径**：`src/apriltag_nav/scripts/keyence_dlen1_node.py`

通过 TCP 轮询 Keyence DL-EN1 传感器（60 Hz），发布距离值。

**ROS 接口**：
- 发布 `keyence/raw` (Int32)，`keyence/value` (Float32)
- 默认地址：192.168.1.5:64000

---

### 7. `frrobot_hw_interface.cpp` — 硬件控制接口

**路径**：`src/frcobot_ros/ros_control_boilerplate/frrobot_control/src/frrobot_hw_interface.cpp`

ROS Control 硬件接口，通过 TCP（192.168.58.2:8080）与机器人控制器通信。

- **Write**：发送 `ServoJ` 关节角度指令（弧度→角度）
- **Read**：读取 `GetActualJointPosRadian` 当前关节位置
- 控制频率：125 Hz

---

## ROS 话题总览

| 话题 | 消息类型 | 方向 | 说明 |
|------|---------|------|------|
| `/rgb` | sensor_msgs/Image | 相机→导航 | AprilTag 图像输入 |
| `/odom` | nav_msgs/Odometry | 底盘→导航 | 里程计反馈 |
| `/cmd_vel` | geometry_msgs/Twist | 导航→底盘 | 底盘速度指令 |
| `/robot_pose` | robot_msgs/Pose2DWithFlag | 导航→臂控 | 移动底盘位姿 |
| `/task_command` | std_msgs/String | 外部→执行器 | 任务指令 |
| `/scan_finished` | std_msgs/Bool | 臂控→执行器 | 扫描完成信号 |
| `keyence/value` | std_msgs/Float32 | 传感器→臂控 | 距离测量值 |
| `/scan/ra_value` | std_msgs/Float32 | 臂控→外部 | Ra 粗糙度值 |
| `/scan/point_result` | std_msgs/String | 臂控→外部 | JSON 扫描结果 |
| `frcobot_status` | frcobot_hw/status | 硬件→监控 | 机器人状态 |

---

## 系统架构方案

### 方案 A（当前实现）— 单进程整合架构

`task_executor.py` 直接 Python import `ArmController`（`arm_controllerrealwithscan.py`），所有控制逻辑在同一 ROS 节点进程内运行。Basler 相机通过 PyPylon API 在进程内直接访问。

```
[mobile_manipulator_system 进程]
├── RobotController     → 订阅 /rgb (USB 相机)，发布 /cmd_vel          [ROS 通信]
├── ArmController       → 直接调用 CameraInterface (Basler PyPylon)   [Python 函数调用]
│                       → 直接调用 Fairino SDK (TCP)                   [Python 函数调用]
│                       → 订阅 keyence/value, /robot_pose              [ROS 通信]
│                       → 发布 /scan_finished, /scan/ra_value          [ROS 通信]
└── TaskManager / MapManager
```

**优点**：结构简单，无消息序列化开销
**缺点**：Basler 崩溃影响整个进程；无法独立重启硬件节点；单元测试困难

---

### 方案 B（规划中）— 分布式 ROS 节点架构

参照 `robot_controller.py` 的设计思路，将所有硬件访问封装为独立 ROS 节点，节点间**仅通过 ROS 话题和服务通信**，不再使用 Python import 方式耦合。

**目标节点拓扑**：

```
[keyence_dlen1_node]  ──→  keyence/value (Float32)          ──→┐
[basler_camera_node]  ──→  /basler/image_raw (Image bgr8)   ──→├──→ [arm_controller_ros]
[robot_controller]    ──→  /robot_pose (Pose2DWithFlag)     ──→┘           │
                                                                             │
[task_executor]  ──→  /arm/scan_command (String/JSON)  ──→ [arm_controller_ros]
                 ←──  /scan_finished (Bool)             ←──
                 ←──  /scan/ra_value (Float32)          ←──
                 ←──  /scan/point_result (String)       ←──
```

**规划新增节点**：

| 节点文件 | 功能 | 对应方案 A 中的角色 |
|---------|------|-------------------|
| `basler_camera_node.py` | 将 Basler 帧发布为 `/basler/image_raw` | `CameraInterface`（内嵌于 ArmController） |
| `arm_controller_ros.py` | 订阅传感器话题，接收扫描指令，控制 Fairino 机械臂 | `arm_controllerrealwithscan.py`（被 import） |

**规划新增话题**：

| 话题 | 消息类型 | 方向 | 说明 |
|------|---------|------|------|
| `/basler/image_raw` | sensor_msgs/Image (bgr8) | basler_node → arm_controller_ros | Basler 实时帧 |
| `/basler/camera_info` | sensor_msgs/CameraInfo | basler_node → arm_controller_ros | 相机内参（备用） |
| `/arm/scan_command` | std_msgs/String (JSON) | task_executor → arm_controller_ros | 扫描点列表序列化字符串 |

**方案 B 节点骨架**（参考，实现时从此扩展）：

```python
# basler_camera_node.py — Basler 帧发布节点
class BaslerCameraNode:
    def __init__(self):
        rospy.init_node('basler_camera_node')
        self.camera = CameraInterface()
        self.camera.initialize()
        self.camera.start_grabbing()
        self.pub = rospy.Publisher('/basler/image_raw', Image, queue_size=1)
        self.bridge = CvBridge()

    def run(self):
        rate = rospy.Rate(rospy.get_param('~fps', 30))
        while not rospy.is_shutdown():
            frame = self.camera.grab_frame()
            if frame is not None:
                self.pub.publish(self.bridge.cv2_to_imgmsg(frame, 'bgr8'))
            rate.sleep()
```

```python
# arm_controller_ros.py — 核心订阅/发布骨架
rospy.Subscriber('/basler/image_raw', Image, self._image_cb,   queue_size=1)  # cache latest frame
rospy.Subscriber('keyence/value',     Float32, self._keyence_cb, queue_size=1)
rospy.Subscriber('/robot_pose',       Pose2DWithFlag, self._pose_cb, queue_size=1)
rospy.Subscriber('/arm/scan_command', String, self._scan_cmd_cb, queue_size=1)  # trigger scan

# task_executor.py — 触发扫描（方案 B）
scan_payload = json.dumps(scan_points)
self.scan_cmd_pub.publish(String(data=scan_payload))
# then wait for /scan_finished as before
```

**优点**：各节点独立重启；可单独 rostopic echo 观察每个数据流；硬件故障不影响上层逻辑
**现有代码保持不变**，方案 B 为全新文件，不替换方案 A

---

## 自定义消息类型

**`robot_msgs/Pose2DWithFlag`** — 带标志位的 2D 位姿
```
std_msgs/Header header
float64 x, y, theta, theta_web
bool flag
int32 id
```

**`robot_msgs/NavDebugStatus`** — 导航调试状态（含区域、偏移、进度等）

**`frcobot_hw/status`** — 机器人硬件状态（关节位置、力矩、IO 等）

---

## 代码规范与命名约定

### Python
- 类名：`PascalCase`（如 `RobotController`、`TaskExecutor`）
- 方法/函数：`snake_case`（如 `move_to_tag`、`execute_scan_points`）
- ROS 节点私有参数：使用 `~` 前缀（如 `rospy.get_param('~max_speed', 0.05)`）
- 常量：全大写（如 `MAX_LINEAR_SPEED = 0.05`）

### ROS 话题命名
- 使用小写加斜杠分隔（如 `/scan_finished`、`keyence/value`）
- 扫描结果统一挂在 `/scan/` 命名空间下

### C++
- 类名：`PascalCase`（如 `FRRobotHWInterface`）
- 方法名：`camelCase`（如 `doWrite`、`doRead`）
- 头文件保护：`#pragma once`

### 代码注释
- **注释统一使用英文**（包括代码内所有行注释、块注释、TODO 等）

### 配置文件
- 所有可调参数放入 `config/robot.yaml`，不硬编码在脚本中
- IP 地址、端口等运行时参数通过 ROS 参数服务器注入

---

## 构建方式

```bash
# 首次构建
cd /home/lcl/mobile_manipulator_ws
catkin_make

# 加载开发环境
source devel/setup.bash

# 仅构建特定包
catkin_make --only-pkg-with-deps apriltag_nav
```

---

## 运行方式

### 真实机器人模式

```bash
# 终端 1：启动 ROS Master
roscore

# 终端 2：启动任务执行器（含底盘导航与机械臂控制）
cd /home/lcl/mobile_manipulator_ws
source devel/setup.bash
rosrun apriltag_nav task_executor.py

# 终端 3：（可选）启动 Keyence 传感器节点
rosrun apriltag_nav keyence_dlen1_node.py
```

### Isaac Sim 仿真模式

```bash
# 参照 readme.txt 启动 Isaac Sim 后执行
roslaunch fr10v6_vision_251219_moveit_config fr10v6_vision_isaac_execution.launch
rosrun apriltag_nav task_executor.py
```

### 发送任务指令

```bash
# 执行预定义任务
rostopic pub -1 /task_command std_msgs/String "TASK scan_joints_line1"

# 导航到指定标记
rostopic pub -1 /task_command std_msgs/String "GOTO 108"

# 查询状态
rostopic pub -1 /task_command std_msgs/String "STATE"

# 紧急停止
rostopic pub -1 /task_command std_msgs/String "STOP"
```

### 调试工具

```bash
# 使用 send_debug_cmd.py 发送调试指令
rosrun apriltag_nav send_debug_cmd.py "EVAL self.state.name"
rosrun apriltag_nav send_debug_cmd.py "EXEC self.arm.move_to_home()"
```

---

## 常见问题

### Q: 机器人 IP 连接失败
**A**：检查 `robot.yaml` 中的 IP 地址与网络连接：
- 机械臂控制器：192.168.58.2:8080（`frrobot_hw_interface.cpp`）
- Keyence 传感器：192.168.1.5:64000（`keyence_dlen1_node.py`）

### Q: 相机无法启动 / PyPylon 报错
**A**：节点会自动切换到 USB 相机模式。若强制使用 USB 相机，在启动参数中设置 `use_webcam:=true`。

### Q: ONNX 模型推理报错
**A**：确认模型文件存在：
```
src/apriltag_nav/model/exported/resnet3D.onnx
src/apriltag_nav/model/exported/resnet3D_gray.onnx
```
模型文件较大（~254M），不纳入 Claude 上下文，需单独管理。

### Q: AprilTag 检测不稳定
**A**：检查 `robot.yaml` 中的 `tag_size`（默认 0.06m）是否与实际标记尺寸一致，并确认相机内参 `/camera_info` 话题正确发布。

### Q: 导航超时（8秒）
**A**：修改 `robot.yaml` 中的 `nav_timeout` 参数（单位：秒）。超时后系统进入 ERROR 状态，需重新发送 TASK 指令。

### Q: `catkin_make` 报找不到 `robot_msgs`
**A**：确认先编译依赖包：
```bash
catkin_make --only-pkg-with-deps robot_msgs
catkin_make
```
