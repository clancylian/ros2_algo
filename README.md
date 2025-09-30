# ROS2 多算法管理系统工作空间

这个工作空间包含两个主要的ROS2包：`multi_algo_manager`（多算法管理器）和`multi_algo_interfaces`（多算法接口定义），用于实现ROS2环境下的多算法并行管理和结果发布。

## 工作空间结构

```
ros2_ws/
├── .gitignore                  # Git忽略文件
├── README.md                   # 工作空间说明文档
└── src/
    ├── multi_algo_interfaces/  # 接口定义包
    │   ├── CMakeLists.txt      # CMake构建文件
    │   ├── package.xml         # 包信息文件
    │   └── srv/                # 服务定义目录
    │       └── AlgoControl.srv # 算法控制服务定义
    └── multi_algo_manager/     # 算法管理器包
        ├── README.txt          # 原有文档
        ├── launch/             # 启动文件目录
        │   └── multi_algo_manager.launch.py # 主启动文件
        ├── multi_algo_manager/ # 包源代码目录
        │   ├── __init__.py     # Python包初始化文件
        │   ├── algo_manager_node.py # 算法管理器节点
        │   ├── algos/          # 算法实现目录
        │   │   ├── __init__.py # 算法包初始化文件
        │   │   ├── channel_monitor.py # 通道监测算法
        │   │   ├── device_check.py    # 设备检查算法
        │   │   ├── line_integrity.py  # 线路完整性算法
        │   │   └── trash_detect.py    # 垃圾检测算法
        │   └── srv/            # 服务定义目录（已移至接口包）
        ├── package.xml         # ROS2包信息文件
        ├── resource/           # 资源文件目录
        ├── setup.cfg           # 安装配置文件
        └── setup.py            # Python包安装脚本
```

## 包说明

### 1. multi_algo_interfaces

这是一个接口定义包，使用C++ (ament_cmake)构建，主要用于定义系统中使用的服务接口。

#### 提供的服务
- **AlgoControl.srv**：用于控制算法的启动和停止
  - 请求部分：
    - `string algo_name`：要控制的算法名称
    - `string action`：动作指令（"start" 或 "stop"）
  - 响应部分：
    - `bool success`：操作是否成功
    - `string message`：操作结果消息

### 2. multi_algo_manager

这是一个Python实现的算法管理包，负责算法的管理、启动、停止和结果发布。

#### 主要功能
- 提供算法管理器节点（AlgoManagerNode）
- 支持通过Service调用按需启动和停止算法
- 算法在独立线程中运行
- 通过Topic发布算法运行结果

#### 包含的算法
1. **垃圾检测算法（TrashDetect）**：检测环境中的垃圾
2. **通道监测算法（ChannelMonitor）**：监测通道拥堵情况
3. **设备检查算法（DeviceCheck）**：检查设备状态
4. **线路完整性算法（LineIntegrity）**：检查线路完整性

## 环境搭建

### 系统要求
- Ubuntu 20.04 LTS
- ROS2 Humble Hawksbill

### 1. 更新系统

```bash
sudo apt update
sudo apt upgrade -y
sudo apt install -y build-essential git python3-colcon-common-extensions python3-pip
```

### 2. 安装 ROS2 Humble

```bash
# 设置 ROS2 源
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2.list'

sudo apt update
sudo apt install -y ros-humble-desktop
```

### 3. 配置环境

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 4. 安装 Python 依赖

```bash
pip3 install -U setuptools
```

## 编译项目

### 1. 安装依赖

在工作空间根目录执行：

```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```

### 2. 编译所有包

```bash
colcon build
```

编译完成后，设置环境变量：

```bash
source install/setup.bash
```

> **注意**：每次新开终端都需要执行上面的source命令来设置环境变量

## 启动与测试

### 1. 启动主控节点

```bash
ros2 launch multi_algo_manager multi_algo_manager.launch.py
```

> 启动后，算法默认处于未启动状态

### 2. 按需启动算法

#### 启动垃圾检测算法

```bash
ros2 service call /algo_control multi_algo_interfaces/srv/AlgoControl "{algo_name: 'TrashDetect', action: 'start'}"
```

#### 启动通道拥堵监测算法

```bash
ros2 service call /algo_control multi_algo_interfaces/srv/AlgoControl "{algo_name: 'ChannelMonitor', action: 'start'}"
```

#### 启动设备检查算法

```bash
ros2 service call /algo_control multi_algo_interfaces/srv/AlgoControl "{algo_name: 'DeviceCheck', action: 'start'}"
```

#### 启动线路完整性算法

```bash
ros2 service call /algo_control multi_algo_interfaces/srv/AlgoControl "{algo_name: 'LineIntegrity', action: 'start'}"
```

### 3. 停止算法

```bash
ros2 service call /algo_control multi_algo_interfaces/srv/AlgoControl "{algo_name: '算法名称', action: 'stop'}"
```

### 4. 查看算法结果

#### 查看统一结果Topic

```bash
ros2 topic echo /algo_result
```

#### 查看单个算法结果Topic

例如，查看垃圾检测算法结果：

```bash
ros2 topic echo /algo_result/trash_detect
```

## 测试技巧

1. **同时启动多个算法**，验证独立线程运行，结果Topic会交替打印消息
2. **启动/停止算法**，确认线程停止时不再发布消息
3. **Service调用错误处理**：
   - 尝试使用错误的算法名称
   - 尝试使用错误的action参数（如'startt'或'stoppp'）
   - 观察返回message是否正确反映错误情况

## 可视化工具

安装并使用rqt查看Topic：

```bash
sudo apt install ros-humble-rqt ros-humble-rqt-common-plugins
rqt
```

在rqt中订阅`/algo_result`，可直观查看实时结果。

## 系统架构

系统架构主要包含以下部分：

1. **接口定义层**：由`multi_algo_interfaces`包提供，定义服务接口
2. **算法管理器节点**：负责接收Service请求，管理各算法的启动和停止
3. **算法模块**：多种独立算法实现，每个算法在独立线程中运行
4. **Service接口**：用于控制算法的启动和停止
5. **Topic接口**：用于发布算法运行结果

## 功能说明

### 算法管理器节点（AlgoManagerNode）
- 初始化并管理所有算法实例
- 提供`/algo_control`服务，用于启动/停止指定算法
- 发布统一的`/algo_result`话题，汇总所有算法结果

### 支持的算法
1. **垃圾检测算法（TrashDetect）**：检测环境中的垃圾
2. **通道监测算法（ChannelMonitor）**：监测通道拥堵情况
3. **设备检查算法（DeviceCheck）**：检查设备状态
4. **线路完整性算法（LineIntegrity）**：检查线路完整性

## 总结

1. 系统基于Ubuntu 20.04 + ROS2 Humble开发
2. 采用接口与实现分离的设计模式
3. 通过colcon工具编译所有包
4. 使用launch文件启动主控节点
5. 通过Service调用按需启动/停止算法
6. 通过Topic订阅获取算法结果

这个系统可以帮助您灵活地管理多个算法，按需启动和停止，同时获取实时的算法运行结果。


colcon build --packages-select multi_algo_interfaces
source install/setup.bash
ros2 interface show multi_algo_interfaces/srv/AlgoControl

colcon build --packages-select multi_algo_manager --symlink-install
source install/setup.bash
ros2 launch multi_algo_manager multi_algo_manager.launch.py
ros2 service call /algo_control multi_algo_interfaces/srv/AlgoControl "{algo_name: 'TrashDetect', action: 'start'}"
ros2 topic echo /algo_result