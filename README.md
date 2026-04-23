# 三机械臂同步搬运系统设计

这是一个基于 ROS1 Noetic + MoveIt 的三机械臂协同搬运工程。当前仓库主要完成了三臂模型、运动规划、视觉坐标转换、协同调度和 Docker 仿真环境，重点是让它在 Windows 10 上也能稳定跑起来，不依赖真实硬件。

## 目前实现了什么

| 模块 | 作用 |
| --- | --- |
| 三机械臂模型 | 提供三机械臂 URDF、STL 网格和显示 launch，可以在 RViz 中查看整体结构 |
| MoveIt 规划配置 | 提供规划组、关节限制、运动学参数、控制器、RViz 和 Gazebo 配置 |
| 视觉目标转换 | `openmv_bridge.py` 读取 OpenMV 串口数据，`block_transformer.py` 将像素坐标转换成世界坐标并发布目标点 |
| 协同调度控制 | `three_arm_coordinator.py` 负责目标接收、优先级调度、冲突检测、安全抬升、抓取、搬运、放置和夹爪控制 |
| Windows 10 仿真 | 通过 Docker + ROS Noetic + 脚本化启动，在没有真实硬件的情况下跑通流程 |
| 测试脚本 | 保留单臂、调试和流程测试脚本，方便后续调参和扩展 |

## 核心数据流

```text
OpenMV / 串口识别结果
  -> `/openmv/blocks`
  -> `block_transformer.py`
  -> `/target/P` `/target/Y` `/target/G`
  -> `three_arm_coordinator.py`
  -> MoveIt 规划与执行
```

## 目录说明

| 路径 | 作用 |
| --- | --- |
| `catkin_ws/src/my_arm_description` | 三机械臂的模型描述包，包含 URDF、STL 网格和显示用 launch |
| `catkin_ws/src/my_arm_description/urdf` | 机械臂与三臂总装的 URDF 文件 |
| `catkin_ws/src/my_arm_description/meshes` | 机械臂零件的 STL 模型文件 |
| `catkin_ws/src/my_arm_description/launch` | 机械臂模型显示相关的启动文件 |
| `catkin_ws/src/my_three_arms_moveit_config` | MoveIt 配置包，包含规划组、控制器、RViz、Gazebo 等配置 |
| `catkin_ws/src/my_three_arms_moveit_config/config` | MoveIt 参数文件，如关节限制、运动学、规划器和控制器配置 |
| `catkin_ws/src/my_three_arms_moveit_config/launch` | MoveIt 各类启动文件，如 demo、move_group、RViz 和 Gazebo |
| `catkin_ws/src/three_arm_demo` | 业务逻辑包，包含视觉转换、串口桥接、协同调度和测试脚本 |
| `catkin_ws/src/three_arm_demo/scripts` | Python 节点和调试脚本，核心是 `openmv_bridge.py`、`block_transformer.py`、`three_arm_coordinator.py` |
| `docker/noetic` | Docker 镜像构建文件和容器入口脚本 |
| `scripts` | 一键启动、构建、烟雾测试和 VcXsrv 启动脚本 |
| `docker-compose.noetic.yml` | Noetic 仿真环境的 Docker Compose 配置 |
| `RUN_DOCKER_NOETIC_WIN10.md` | Windows 10 下的完整运行说明 |
| `catkin_ws/src/CMakeLists.txt` | catkin 工作空间入口文件 |

## 快速启动

```powershell
powershell -ExecutionPolicy Bypass -File .\scripts\run_noetic_smoke.ps1
```

### Windows 图形化一键启动

```powershell
# MoveIt + RViz（默认）
powershell -ExecutionPolicy Bypass -File .\scripts\run_noetic_gui.ps1

# Gazebo + MoveIt
powershell -ExecutionPolicy Bypass -File .\scripts\run_noetic_gui.ps1 -Mode gazebo
```

该脚本会自动完成：启动 VcXsrv、启动 Docker 容器、安装依赖、编译工作区并拉起图形化仿真。

### Windows 一键演示抓取

```powershell
# MoveIt + RViz 抓取演示（默认）
powershell -ExecutionPolicy Bypass -File .\scripts\run_noetic_grasp_demo.ps1

# Gazebo + MoveIt 抓取演示
powershell -ExecutionPolicy Bypass -File .\scripts\run_noetic_grasp_demo.ps1 -Mode gazebo

# 已经编译过可跳过编译
powershell -ExecutionPolicy Bypass -File .\scripts\run_noetic_grasp_demo.ps1 -SkipBuild
```

该脚本会自动完成：启动图形仿真、启动 `three_arm_coordinator.py`、并自动发布三组目标点触发三机械臂抓取与搬运流程。

## 原生 Ubuntu 运行

下面步骤适用于 Ubuntu 20.04 + ROS Noetic。若是 Ubuntu 22.04，建议继续使用仓库里的 Docker 方案。

### 1. 安装依赖

```bash
sudo apt update
sudo apt install -y curl gnupg lsb-release
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros1.list'
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt update
sudo apt install -y \
  ros-noetic-desktop-full \
  ros-noetic-moveit \
  ros-noetic-cv-bridge \
  ros-noetic-ros-control \
  ros-noetic-ros-controllers \
  ros-noetic-gazebo-ros-pkgs \
  ros-noetic-gazebo-ros-control \
  ros-noetic-rviz \
  ros-noetic-robot-state-publisher \
  ros-noetic-joint-state-publisher \
  ros-noetic-joint-state-publisher-gui \
  ros-noetic-tf \
  ros-noetic-tf2-ros \
  ros-noetic-std-msgs \
  ros-noetic-visualization-msgs \
  ros-noetic-sensor-msgs \
  ros-noetic-xacro \
  python3-rosdep \
  python3-serial \
  python3-numpy \
  python3-opencv
sudo rosdep init
rosdep update
```

### 2. 编译工作区

```bash
cd <仓库根目录>/catkin_ws
source /opt/ros/noetic/setup.bash
rosdep install --from-paths src --ignore-src -r -y
catkin_make
source devel/setup.bash
```

如果脚本没有执行权限，先运行 `chmod +x catkin_ws/src/three_arm_demo/scripts/*.py`。

### 3. 启动 MoveIt 仿真

```bash
roslaunch my_three_arms_moveit_config demo.launch
```

这个模式会启动 fake controller 和 RViz，适合先检查模型和规划是否正常。

### 4. 启动 Gazebo 仿真

```bash
roslaunch my_three_arms_moveit_config demo_gazebo.launch
```

这个模式会同时启动 Gazebo 和 MoveIt，适合做更完整的联调。

### 5. 启动业务节点

建议在不同终端分别执行：

```bash
rosrun three_arm_demo block_transformer.py
rosrun three_arm_demo three_arm_coordinator.py
rosrun three_arm_demo openmv_bridge.py
```

其中 `openmv_bridge.py` 只有在接了 OpenMV 串口模块时才需要运行。如果没有 OpenMV，可以先用 `test_single_arm.py`、`test.py`、`test2.py` 做单臂或调试验证。

## 说明

当前仓库以仿真和流程验证为主。真实机械臂与相机接入时，主要通过 OpenMV 串口和 MoveIt 接口继续对接。
