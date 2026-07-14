# motor_ros

ROS 2 电机控制节点，通过 CAN 总线控制达妙（Damiao）系列电机，支持多腿足机器人关节驱动，支持最高1000Hz控制频率。

## 目录

- [概述](#概述)
- [支持的电机](#支持的电机)
- [项目结构](#项目结构)
- [依赖](#依赖)
- [编译](#编译)
  - [编译 sockcanpp](#编译-sockcanpp)
  - [编译 motor_ros](#编译-motor_ros)
- [配置](#配置)
  - [全局参数](#全局参数)
  - [电机定义](#电机定义)
  - [腿部定义](#腿部定义)
- [运行](#运行)
  - [CAN 接口初始化](#can-接口初始化)
  - [启动节点](#启动节点)
- [ROS 2 接口](#ros-2-接口)
  - [订阅 Topic](#订阅-topic)
  - [发布 Topic](#发布-topic)
- [控制模式](#控制模式)

---

## 概述

`motor_ros` 是一个 ROS 2 (`ament_cmake`) 软件包，用于通过 SocketCAN 接口控制达妙系列伺服电机。项目采用四腿机器人架构，每条腿配备 4 个电机（共 16 个），通过独立的 CAN 通道（`can0` ~ `can3`）进行通信。

核心功能：

- **MIT 力位混合控制**：同时控制位置、速度、力矩，配合 KP/KD 增益
- **多 CAN 通道支持**：每条腿独立 CAN 接口，200Hz 控制频率
- **YAML 配置驱动**：电机参数、腿部结构、方向映射全部通过 YAML 文件配置
- **线程安全架构**：发送队列 + 独立收发线程，不阻塞主控制循环
- **CAN 总线错误检测**：完善的错误处理与日志上报

## 支持的电机

### DM 系列

| 型号 | 峰值转矩 (N·m) | 备注 |
|------|:-------------:|------|
| DM4310 | 10 | 标准电压 |
| DM4310_48V | 10 | 48V 高压版 |
| DM4340 | 28 | |
| DM4340_48V | 28 | 48V 高压版 |
| DM6006 | 12 | |
| DM8006 | 20 | |
| DM8009 | 54 | |
| DM10010L | 200 | 低速大扭矩 |
| DM10010 | 200 | |
| DMH3510 | 1 | 小力矩 |
| DMH6215 | 10 | |
| DMG6220 | 10 | |

### RS 系列

| 型号 | 备注 |
|------|------|
| RS00 | DQ50 / T17 |
| RS01 | |
| RS02 | |
| RS03 | DQ50 |
| RS04 | |
| RS05 | DQ33 / T17 |
| RS06 | DQ20 / T60 |

## 项目结构

```
damiao_hscant/
├── CMakeLists.txt              # CMake 构建文件
├── package.xml                 # ROS 2 包描述
├── README.md                   # 本文件
├── conf/
│   ├── motors.yaml             # 默认四腿电机配置
│   ├── damiao_wheel.yaml       # 达妙轮式配置
│   ├── damiao_foot.yaml        # 达妙足式配置
│   └── rs_wheel.yaml           # RS 轮式配置
├── include/
│   └── motor.h                 # 电机控制库头文件
├── launch/
│   ├── motor_node.launch.py    # 通用启动文件（支持 --config 参数）
│   └── setup_node.launch.py    # CAN 初始化 + 节点启动一键脚本
├── lib/
│   └── libsockcanpp/           # SocketCAN C++ 封装库
├── msg/
│   ├── Command.msg             # 控制指令消息
│   └── State.msg               # 状态反馈消息
├── scripts/
│   └── setup_can.sh            # CAN 接口初始化脚本
└── src/
    ├── motor.cpp               # 电机控制核心实现
    └── motor_node.cpp          # ROS 2 节点主程序
```

## 依赖

| 依赖 | 版本要求 | 用途 |
|------|:-------:|------|
| ROS 2 (rclcpp) | Humble 或更高 | ROS 通信框架 |
| CMake | >= 3.23 | 构建系统 |
| yaml-cpp | - | YAML 配置文件解析 |
| libsockcanpp | - | SocketCAN C++ 封装 |
| pthread | - | 多线程支持 |
| C++ Standard | 17 | 语言标准 |

## 编译

> **注意**：编译 `sockcanpp` 需要 CMake > 3.10，而 ROS 1 支持的最高 CMake 版本为 3.5，无法直接编译。此项目已为后续更新保留 `lib/sockcanpp`。

### 编译 sockcanpp

首先修改 `lib/libsockcanpp/CMakeLists.txt` 第 5 行：

```cmake
option(BUILD_SHARED_LIBS "Build shared libraries (.so) instead of static ones (.a)" OFF)
```

然后下载新版 CMake 并编译：

```shell
# 下载新版 CMake
wget https://cmake.org/files/LatestRelease/cmake-4.2.1-linux-x86_64.tar.gz
tar zxvf cmake-4.2.1-linux-x86_64.tar.gz

# 编译 libsockcanpp
cd lib/libsockcanpp
mkdir build
cd build
/path/to/cmake ..
make -j

# 复制产物
cp -r ../include ../../lib/libsockcanpp-prebuild
cp libsockcanpp.a ../../lib/libsockcanpp-prebuild
```

### 编译 motor_ros

```shell
# 在 ROS 2 workspace 中
cd ~/ros2_ws
colcon build --packages-select motor_ros
source install/setup.bash
```

## 配置

所有电机和腿部配置通过 YAML 文件管理。默认配置文件为 `conf/motors.yaml`。

### 全局参数

```yaml
global:
  zero_position: 1.5707963267948   # 零点位置（π/2 弧度）
  calf_gear_ratio: 1.5              # 小腿电机减速比
```

### 电机定义

```yaml
motors:
  FL_M0:                    # 电机名称（自由命名）
    motor_type: DM8006      # 电机型号
    slave_id: 0x01          # 从机 CAN ID
    master_id: 0x11         # 主机 CAN ID
```

### 腿部定义

每条腿代表一组通过同一 CAN 通道控制的电机集合：

```yaml
legs:
  FL:                                  # 腿名称
    name: "Front Left"                 # 显示名称
    can_port: "can0"                   # CAN 接口
    motors: [FL_M0, FL_M1, FL_M2, FL_M3]  # 电机列表
    directions: [1, 1, 1, -1]          # 方向系数（1 或 -1）
    zero_position_indices: 2           # 需要零点偏移的电机索引
    zero_position_offset: 1            # 零点偏移符号
```

**字段说明**：

- `motors`：引用 `motors` 段定义的电机名称，顺序决定指令/状态数组中的排列
- `directions`：方向系数，`1` 为正转，`-1` 为反转
- `zero_position_indices`：指定需要应用零点偏置的电机在 motors 列表中的索引
- `zero_position_offset`：零点偏移方向，`1` 表示减去，`-1` 表示加上

## 运行

### CAN 接口初始化

在运行节点前，需要先初始化 CAN 接口：

```shell
# 手动初始化（以 can0 为例）
sudo ip link set down can0
sudo ip link set can0 txqueuelen 100
sudo ip link set can0 type can bitrate 1000000 sample-point 0.8
sudo ip link set up can0
```

或使用提供的脚本一键初始化全部 4 路 CAN：

```shell
ros2 launch motor_ros setup_node.launch.py
```

> 该脚本会配置 `can0` ~ `can3`，波特率 1Mbps，采样点 0.8。

### 启动节点

```shell
# 使用默认配置
ros2 launch motor_ros motor_node.launch.py

# 指定自定义配置文件
ros2 launch motor_ros motor_node.launch.py config_file:=/path/to/config.yaml
```

节点启动后将以 200Hz 频率运行控制循环，日志输出示例：

```
[INFO] [motor_node]: Configuration loaded successfully
[INFO] [motor_node]: Initializing leg: Front Left (CAN port can0)
[INFO] [motor_node]: Initializing leg: Front Right (CAN port can1)
[INFO] [motor_node]: Initializing leg: Back Left (CAN port can2)
[INFO] [motor_node]: Initializing leg: Back Right (CAN port can3)
Motor Initial Positions:
  FL[0] (ID: 0x01): 0.5234
  FL[1] (ID: 0x02): -0.1234
  ...
Total motors: 16
[INFO] [motor_node]: Motor node started, running at 200Hz
```

## ROS 2 接口

### 订阅 Topic

**`/dm_cmd`** (`motor_ros/msg/Command`)

控制指令，数组中各元素按 `legs` 配置中的顺序排列（先按腿名排序，再按每条腿的 motors 列表排序）：

```
string[] joint_names    # 关节名称（与 motors 配置一一对应）
float64[] pos           # 目标位置 (rad)
float64[] vel           # 目标速度 (rad/s)
float64[] kp            # 位置增益
float64[] kd            # 速度增益
float64[] tau           # 前馈力矩 (N·m)
```

### 发布 Topic

**`/dm_states`** (`motor_ros/msg/State`)

电机状态反馈：

```
string[] joint_names    # 关节名称
float64[] pos           # 当前位置 (rad)
float64[] vel           # 当前速度 (rad/s)
float64[] tau           # 当前力矩 (N·m)
```

## 控制模式

项目支持三种控制模式，通过不同的 CAN ID 偏移量区分：

| 模式 | CAN ID 偏移 | 函数 | 说明 |
|------|:---------:|------|------|
| MIT 力位混合 | 0x000 | `control_mit()` | 同时控制 q, dq, tau，配合 kp/kd |
| 位置-速度 | 0x100 | `control_pos_vel()` | 直接位置+速度指令 |
| 纯速度 | 0x200 | `control_vel()` | 纯速度控制 |

默认使用 **MIT 模式**，上位机通过 `/dm_cmd` topic 下发 kp, kd, pos, vel, tau 五个参数。

### MIT 控制参数说明

```
control_mit(motor, kp, kd, q, dq, tau)

  kp  - 位置比例增益 (0 ~ KP_MAX)
  kd  - 速度阻尼增益 (0 ~ KD_MAX)
  q   - 目标位置 (-Q_MAX ~ Q_MAX)
  dq  - 目标速度 (-DQ_MAX ~ DQ_MAX)
  tau - 前馈力矩 (-TAU_MAX ~ TAU_MAX)
```

各电机的 KP_MAX、KD_MAX、Q_MAX、DQ_MAX、TAU_MAX 限制值定义在 `src/motor.cpp` 的 `limit_param` 数组中。

---

## 许可证

待定
