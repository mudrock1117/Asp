# s3_60_eco_2024 ROS2 Package

这是一个用于S3-60 Eco 2024机器人的ROS2 Humble包。

## 功能特性

- 完整的URDF模型定义
- RViz2可视化支持
- Gazebo仿真支持
- 关节状态发布器GUI
- 机器人状态发布器

## 依赖项

确保您的系统已安装以下ROS2 Humble包：

```bash
sudo apt install ros-humble-robot-state-publisher
sudo apt install ros-humble-joint-state-publisher-gui
sudo apt install ros-humble-gazebo-ros-pkgs
sudo apt install ros-humble-rviz2
sudo apt install ros-humble-xacro
sudo apt install ros-humble-tf2-ros
sudo apt install ros-humble-std-msgs
```

```bash
sudo apt install ros-humble-joint-state-publisher-gui
```

## 构建

在工作空间的根目录中运行：

```bash
colcon build --packages-select s3_60_eco_2024
```

## 使用方法

### 1. 显示机器人模型（RViz2）

```bash
# 使用完整版本（包含mesh文件）
ros2 launch s3_60_eco_2024 display.launch.py
```

这将启动：
- joint_state_publisher_gui：用于交互式控制关节
- robot_state_publisher：发布机器人状态
- rviz2：3D可视化


## 文件结构

```
s3_60_eco_2024/
├── config/                    # 配置文件
│   └── joint_names_s3_60_eco_2024.yaml  # 关节名称配置
├── launch/                    # Launch文件
│   ├── display.launch.py     # RViz2显示（原始版本）
├── meshes/                   # 3D模型文件
│   ├── base_link.STL
│   ├── link1.STL
│   ├── link2.STL
│   ├── link3.STL
│   ├── link4.STL
│   ├── link5.STL
│   └── link6.STL
├── rviz/                     # RViz2配置
│   └── urdf.rviz
├── textures/                 # 纹理文件
├── urdf/                     # URDF文件
│   ├── s3_60_eco_2024.csv
│   ├── s3_60_eco_2024.urdf  # 原始URDF
├── CMakeLists.txt
├── package.xml
└── README.md
```

## 配置文件说明

### config/joint_names_s3_60_eco_2024.yaml
这个文件定义了机器人的关节名称，用于：
- 指定哪些关节由joint_state_publisher_gui控制
- 配置控制器管理的关节
- 定义运动规划中使用的关节

当前配置包含6个关节：joint1, joint2, joint3, joint4, joint5, joint6

## 许可证

Apache-2.0 