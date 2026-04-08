# realsense-ros-mcp

[![Python 3.8+](https://img.shields.io/badge/python-3.8+-blue.svg)](https://www.python.org/downloads/)
[![ROS2 Humble](https://img.shields.io/badge/ROS2-Humble-blue.svg)](https://docs.ros.org/en/humble/)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![RealSense](https://img.shields.io/badge/Intel-RealSense-blue.svg)](https://www.intelrealsense.com/)

Intel RealSense ROS2 MCP Server — 封装 realsense2_camera 的 ROS2 接口，提供 25 个 MCP tools，让 AI 助手能够通过 ROS2 控制 Intel RealSense 深度相机。

[English](./README.md) | **中文**

---

## 功能特性

### ROS2 节点管理
- `launch_camera` — 启动 realsense2_camera ROS2 节点
- `stop_camera` — 停止相机节点
- `list_active_cameras` — 列出活动相机进程

### Topic 操作
- `list_topics` — 列出 ROS2 topics
- `get_topic_info` — 获取 topic 类型和发布者
- `get_topic_hz` — 测量 topic 发布频率
- `capture_color_image` — 从 color topic 捕获彩色图
- `capture_depth_image` — 从 depth topic 捕获深度图
- `capture_rgbd` — 捕获对齐的 RGBD 图像对
- `capture_infrared` — 捕获红外图
- `get_imu_data` — 获取 IMU 数据
- `capture_pointcloud` — 捕获点云 (PLY/PCD)

### 服务调用
- `get_device_info` — 调用 DeviceInfo 服务

### 参数控制
- `get_camera_parameters` — 获取所有参数
- `set_camera_parameter` — 设置参数
- `enable_filter` — 启用/禁用滤波器
- `set_depth_profile` — 设置深度流分辨率和帧率
- `set_color_profile` — 设置彩色流分辨率和帧率

### TF 和标定
- `get_camera_info` — 获取 CameraInfo（内参）
- `get_tf` — 查询 TF 变换
- `get_extrinsics` — 获取外参

### 多相机
- `launch_multi_cameras` — 批量启动多相机
- `capture_multi_frames` — 多相机同步捕帧

### ROS2 诊断
- `check_ros2_status` — 检查 ROS2 环境
- `list_ros2_nodes` — 列出所有节点

---

## 快速开始

### 前提条件

```bash
# 安装 ROS2 Humble (Ubuntu 22.04)
# 参考: https://docs.ros.org/en/humble/Installation.html

# 安装 realsense2_camera 包
sudo apt install ros-humble-realsense2-camera

# 或使用源码安装
# cd ~/ros2_ws/src
# git clone https://github.com/IntelRealSense/realsense-ros.git -b ros2-development
# cd ..
# rosdep install -i --from-path src --rosdistro humble -y
# colcon build
```

### 安装

```bash
# 确保 ROS2 已 source
source /opt/ros/humble/setup.bash

# 安装 Python 依赖
pip install -r requirements.txt

# 测试 server
python mcp_server.py --test

# 运行 server (stdio)
python mcp_server.py

# Demo (不经过 MCP)
python tests/demo_ros_capture.py
```

### MCP 配置

添加到 OpenClaw `config.yaml`:

```yaml
mcp:
  servers:
    realsense-ros:
      command: bash
      args: ["-c", "source /opt/ros/humble/setup.bash && python3 /path/to/realsense-ros-mcp/mcp_server.py"]
```

或使用 Claude Desktop 配置:

```json
{
  "mcpServers": {
    "realsense-ros": {
      "command": "bash",
      "args": ["-c", "source /opt/ros/humble/setup.bash && python3 /path/to/realsense-ros-mcp/mcp_server.py"]
    }
  }
}
```

---

## 硬件要求

- Intel RealSense D400/L500 系列相机
  - D435/D435i (推荐)
  - D455
  - D415
  - L515
- ROS2 Humble (Ubuntu 22.04)
- realsense2_camera 包已安装
- USB 3.0 连接 (必须)
- Python 3.8+

---

## 示例用法

```python
# 检查 ROS2 状态
check_ros2_status()

# 启动相机节点
launch_camera(serial="12345", namespace="camera")

# 列出 topics
list_topics()
# 返回: ["/camera/color/image_raw", "/camera/depth/image_rect_raw", ...]

# 捕获彩色图像
capture_color_image(topic="/camera/color/image_raw", filename="color.png")

# 获取相机参数
get_camera_parameters(node_namespace="camera")

# 设置参数
set_camera_parameter(node_namespace="camera", param_name="depth_module.emitter_enabled", param_value=True)

# 停止相机
stop_camera(namespace="camera")
```

---

## 架构

```
┌─────────────────────────────────────────────────────────┐
│                     MCP Client                          │
│              (Claude / OpenClaw / etc.)                 │
└────────────────────┬────────────────────────────────────┘
                     │ stdio
                     ▼
┌─────────────────────────────────────────────────────────┐
│                realsense-ros-mcp                        │
│              (mcp_server.py / FastMCP)                  │
└────────────────────┬────────────────────────────────────┘
                     │
                     ▼
┌─────────────────────────────────────────────────────────┐
│                   bridge.py                             │
│        (ROS2Bridge - rclpy / ros2cli integration)       │
└────────────────────┬────────────────────────────────────┘
                     │
                     ▼
┌─────────────────────────────────────────────────────────┐
│              ROS2 Humble + realsense2_camera            │
│              (ROS2 nodes / topics / services)           │
└────────────────────┬────────────────────────────────────┘
                     │ USB 3.0
                     ▼
┌─────────────────────────────────────────────────────────┐
│             Intel RealSense Camera                      │
│                (D435/D455/L515/etc.)                    │
└─────────────────────────────────────────────────────────┘
```

---

## 与 librealsense-mcp 的区别

| 特性 | librealsense-mcp | realsense-ros-mcp |
|------|------------------|-------------------|
| 底层 | pyrealsense2 直接调用 | ROS2 topics/services |
| 依赖 | pyrealsense2 | ROS2 Humble + realsense2_camera |
| 适用场景 | 独立应用、轻量级 | ROS2 机器人系统集成 |
| TF 支持 | ❌ | ✅ |
| 点云格式 | PLY | PLY / PCD |
| 滤波器 | SDK 内置 | ROS2 参数动态配置 |
| 多机协同 | 手动管理 | ROS2 原生支持 |

---

## 测试

```bash
# 确保 ROS2 环境已设置
source /opt/ros/humble/setup.bash

# 运行测试
cd tests
python demo_ros_capture.py
```

---

## 相关项目

- [librealsense-mcp](https://github.com/ros-claw/librealsense-mcp) — 直接 SDK 版本的 RealSense MCP Server
- [realsense-ros](https://github.com/IntelRealSense/realsense-ros) — 官方的 ROS2 RealSense 包

---

## 许可证

MIT License - 详见 [LICENSE](LICENSE)

---

**Made with ❤️ for the ROS2 robotics community**
