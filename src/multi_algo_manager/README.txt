老大，我给你整理一份在 **Ubuntu 20.04** 下跑这个 ROS2 多算法 Service/Topic DEMO 的完整环境搭建和测试步骤，让你能快速跑起来。

---

## 1️⃣ 安装基础环境

### 1.1 更新系统

```bash
sudo apt update
sudo apt upgrade -y
sudo apt install -y build-essential git python3-colcon-common-extensions python3-pip
```

### 1.2 安装 ROS2

我们假设使用 **ROS2 Humble Hawksbill**（官方推荐 Ubuntu 20.04 版本）。

```bash
# 设置 ROS2 源
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2.list'

sudo apt update
sudo apt install -y ros-humble-desktop
```

### 1.3 配置环境

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 1.4 安装 Python 依赖

```bash
pip3 install -U setuptools
```

---

## 2️⃣ 创建工作空间并放置 DEMO

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone <your_multi_algo_manager_repo_url> multi_algo_manager
```

或者直接把前面整理的 `multi_algo_manager/` 文件夹放到 `~/ros2_ws/src/` 下。

---

## 3️⃣ 安装依赖

在工作空间根目录：

```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```

---

## 4️⃣ 编译包

```bash
colcon build --packages-select multi_algo_manager
```

编译完成后：

```bash
source install/setup.bash
```

> 每次新开终端记得 source 一次

---

## 5️⃣ 运行 DEMO

### 5.1 启动主控节点（算法默认不启动）

```bash
ros2 launch multi_algo_manager multi_algo_manager.launch.py
```

### 5.2 按需启动算法

启动垃圾检测算法：

```bash
ros2 service call /algo_control multi_algo_manager/srv/AlgoControl "{algo_name: 'TrashDetect', action: 'start'}"
```

启动通道拥堵监测：

```bash
ros2 service call /algo_control multi_algo_manager/srv/AlgoControl "{algo_name: 'ChannelMonitor', action: 'start'}"
```

停止某算法：

```bash
ros2 service call /algo_control multi_algo_manager/srv/AlgoControl "{algo_name: 'TrashDetect', action: 'stop'}"
```

### 5.3 查看结果

统一结果 Topic：

```bash
ros2 topic echo /algo_result
```

或者查看单个算法 Topic，例如垃圾检测：

```bash
ros2 topic echo /algo_result/trash_detect
```

---

## 6️⃣ 测试小技巧

1. **同时启动多个算法**，验证独立线程运行，结果 Topic 会交替打印消息。
2. **启动/停止算法**，确认线程停止时不再发布消息。
3. **Service 调用错误处理**：

   * 错误算法名称
   * 错误 action (`startt` / `stoppp`)
   * 观察返回 message 是否正确

---

## 7️⃣ 可选优化

* 安装 **rqt** 查看 Topic：

```bash
sudo apt install ros-humble-rqt ros-humble-rqt-common-plugins
rqt
```

* 在 rqt 中订阅 `/algo_result`，可直观查看实时结果。

---

老大，总结：

1. Ubuntu 20.04 + ROS2 Humble
2. colcon build 多算法包
3. launch 主控节点
4. Service 调用按需启动/停止算法
5. Topic 订阅获取中间结果

这样就能完整跑起整个 DEMO，按需控制算法，同时测试线程和消息推送。

---

我可以帮你画一张 **操作流程图**，从终端 Service 调用 → 算法线程运行 → Topic 发布 → 上层查看结果，帮助你直观理解整个流程。

你希望我画吗？
