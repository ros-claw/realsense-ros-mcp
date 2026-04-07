#!/usr/bin/env python3
"""
realsense-ros-mcp — MCP Server for Intel RealSense via ROS2 (realsense2_camera).

Exposes ROS2 node launch management, topic subscriptions, parameter control,
service calls, TF queries, and multi-camera operations as MCP tools.

Usage:
    python mcp_server.py              # stdio transport (default)
    python mcp_server.py --test       # dry-run import test
"""

import sys
import os
import json
import logging
from typing import Any, Dict, List, Optional

from mcp.server.fastmcp import FastMCP

# Ensure the package directory is importable
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from bridge import ROS2Bridge, DEFAULT_OUTPUT_DIR
import safety_guard as sg

# ── Logging ───────────────────────────────────────────────────────────────────
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(name)s] %(levelname)s %(message)s",
)
logger = logging.getLogger("realsense_ros.mcp")

# ── MCP Server ────────────────────────────────────────────────────────────────
mcp = FastMCP(
    "realsense-ros-mcp",
    instructions="Intel RealSense ROS2 MCP Server — 封装 realsense2_camera 的 ROS2 接口，"
                 "提供节点 launch 管理、topic 订阅捕帧、参数控制、TF 查询和多相机协同。",
)

# Lazy bridge singleton
_bridge: Optional[ROS2Bridge] = None


def _get_bridge() -> ROS2Bridge:
    global _bridge
    if _bridge is None:
        _bridge = ROS2Bridge()
    return _bridge


def _ok(data: Dict[str, Any]) -> str:
    """Wrap result as JSON string."""
    return json.dumps(data, ensure_ascii=False, indent=2)


def _err(msg: str) -> str:
    return json.dumps({"error": msg}, ensure_ascii=False, indent=2)


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# 1. ROS2 节点管理
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

@mcp.tool()
def launch_camera(
    serial: str,
    camera_name: str,
    camera_namespace: str = "",
    enable_color: bool = True,
    enable_depth: bool = True,
    enable_infra: bool = False,
    enable_imu: bool = False,
    enable_pointcloud: bool = False,
    enable_align_depth: bool = True,
    width: int = 640,
    height: int = 480,
    fps: int = 30,
) -> str:
    """启动一个 realsense2_camera ROS2 节点。

    Args:
        serial: 设备序列号，例如 "231122070092"
        camera_name: 相机唯一名称（用于 topic 前缀，如 "camera1"）
        camera_namespace: ROS2 命名空间（默认同 camera_name）
        enable_color: 启用彩色流
        enable_depth: 启用深度流
        enable_infra: 启用红外流 (infra1 + infra2)
        enable_imu: 启用 IMU 流 (加速度计+陀螺仪)
        enable_pointcloud: 启用点云输出
        enable_align_depth: 启用深度对齐到彩色
        width: 图像宽度 (320-1920)
        height: 图像高度 (240-1080)
        fps: 帧率 (1-90)
    """
    try:
        ok, msg = sg.validate_serial(serial)
        sg.check_or_raise(ok, msg)
        ok, msg = sg.validate_camera_name(camera_name)
        sg.check_or_raise(ok, msg)
        ok, msg = sg.validate_resolution(width, height)
        sg.check_or_raise(ok, msg)
        ok, msg = sg.validate_fps(fps)
        sg.check_or_raise(ok, msg)

        result = _get_bridge().launch_camera(
            serial=serial, camera_name=camera_name,
            camera_namespace=camera_namespace,
            enable_color=enable_color, enable_depth=enable_depth,
            enable_infra=enable_infra, enable_imu=enable_imu,
            enable_pointcloud=enable_pointcloud,
            enable_align_depth=enable_align_depth,
            width=width, height=height, fps=fps,
        )
        return _ok(result)
    except (ValueError, RuntimeError) as e:
        return _err(str(e))


@mcp.tool()
def stop_camera(camera_name: str) -> str:
    """停止指定的 realsense2_camera ROS2 节点。

    Args:
        camera_name: 启动时指定的相机名称
    """
    try:
        ok, msg = sg.validate_camera_name(camera_name)
        sg.check_or_raise(ok, msg)
        result = _get_bridge().stop_camera(camera_name)
        return _ok(result)
    except (ValueError, RuntimeError) as e:
        return _err(str(e))


@mcp.tool()
def list_active_cameras() -> str:
    """列出所有由本 bridge 管理的活动 ROS2 相机节点进程。"""
    try:
        result = _get_bridge().list_active_cameras()
        return _ok(result)
    except Exception as e:
        return _err(str(e))


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# 2. Topic 操作
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

@mcp.tool()
def list_topics(camera_name: Optional[str] = None) -> str:
    """列出 ROS2 topics，可按相机名过滤。

    Args:
        camera_name: 相机名称（可选，传入则只显示该相机相关 topic）
    """
    try:
        result = _get_bridge().list_topics(camera_name=camera_name)
        return _ok(result)
    except Exception as e:
        return _err(str(e))


@mcp.tool()
def get_topic_info(topic_name: str) -> str:
    """获取指定 ROS2 topic 的类型和发布者信息。

    Args:
        topic_name: 完整 topic 名称（如 "/camera1/camera1/color/image_raw"）
    """
    try:
        result = _get_bridge().get_topic_info(topic_name)
        return _ok(result)
    except Exception as e:
        return _err(str(e))


@mcp.tool()
def get_topic_hz(topic_name: str, window: int = 5) -> str:
    """测量 ROS2 topic 的发布频率。

    Args:
        topic_name: 完整 topic 名称
        window: 采样消息数量
    """
    try:
        result = _get_bridge().get_topic_hz(topic_name, window=window)
        return _ok(result)
    except Exception as e:
        return _err(str(e))


@mcp.tool()
def capture_color_image(camera_name: str, save_path: str = "", timeout_sec: float = 5.0) -> str:
    """从 ROS2 color topic 订阅一帧彩色图像并保存为 PNG。

    Args:
        camera_name: 相机名称
        save_path: 保存路径（默认 /tmp/realsense-ros/<camera_name>_color.png）
        timeout_sec: 订阅超时秒数 (0.1-30)
    """
    try:
        ok, msg = sg.validate_camera_name(camera_name)
        sg.check_or_raise(ok, msg)
        ok, msg = sg.validate_timeout(timeout_sec)
        sg.check_or_raise(ok, msg)
        if save_path:
            ok, msg = sg.validate_file_path(save_path)
            sg.check_or_raise(ok, msg)
        else:
            save_path = os.path.join(DEFAULT_OUTPUT_DIR, f"{camera_name}_color.png")

        result = _get_bridge().capture_color_image(camera_name, save_path, timeout_sec)
        return _ok(result)
    except (ValueError, RuntimeError) as e:
        return _err(str(e))


@mcp.tool()
def capture_depth_image(
    camera_name: str,
    save_path: str = "",
    aligned: bool = True,
    colorize: bool = True,
    timeout_sec: float = 5.0,
) -> str:
    """从 ROS2 depth topic 订阅一帧深度图像并保存。

    Args:
        camera_name: 相机名称
        save_path: 保存路径（默认自动生成）
        aligned: 使用 aligned_depth_to_color topic（深度对齐到彩色坐标系）
        colorize: True=JET 伪彩色可视化, False=原始 16-bit 深度值
        timeout_sec: 订阅超时秒数
    """
    try:
        ok, msg = sg.validate_camera_name(camera_name)
        sg.check_or_raise(ok, msg)
        ok, msg = sg.validate_timeout(timeout_sec)
        sg.check_or_raise(ok, msg)
        if save_path:
            ok, msg = sg.validate_file_path(save_path)
            sg.check_or_raise(ok, msg)
        else:
            suffix = "aligned_depth" if aligned else "depth"
            save_path = os.path.join(DEFAULT_OUTPUT_DIR, f"{camera_name}_{suffix}.png")

        result = _get_bridge().capture_depth_image(camera_name, save_path, aligned, colorize, timeout_sec)
        return _ok(result)
    except (ValueError, RuntimeError) as e:
        return _err(str(e))


@mcp.tool()
def capture_rgbd(
    camera_name: str,
    color_path: str = "",
    depth_path: str = "",
    aligned: bool = True,
    timeout_sec: float = 5.0,
) -> str:
    """捕获对齐的 RGBD 图像对（颜色+深度）并分别保存。

    Args:
        camera_name: 相机名称
        color_path: 彩色图保存路径
        depth_path: 深度图保存路径
        aligned: 使用对齐深度
        timeout_sec: 每帧超时秒数
    """
    try:
        ok, msg = sg.validate_camera_name(camera_name)
        sg.check_or_raise(ok, msg)
        if not color_path:
            color_path = os.path.join(DEFAULT_OUTPUT_DIR, f"{camera_name}_rgbd_color.png")
        if not depth_path:
            depth_path = os.path.join(DEFAULT_OUTPUT_DIR, f"{camera_name}_rgbd_depth.png")
        if color_path:
            ok, msg = sg.validate_file_path(color_path)
            sg.check_or_raise(ok, msg)
        if depth_path:
            ok, msg = sg.validate_file_path(depth_path)
            sg.check_or_raise(ok, msg)

        result = _get_bridge().capture_rgbd(camera_name, color_path, depth_path, aligned, timeout_sec)
        return _ok(result)
    except (ValueError, RuntimeError) as e:
        return _err(str(e))


@mcp.tool()
def capture_infrared(camera_name: str, save_path: str = "", index: int = 1, timeout_sec: float = 5.0) -> str:
    """从 ROS2 infrared topic 捕获一帧红外图像。

    Args:
        camera_name: 相机名称
        save_path: 保存路径
        index: 红外传感器索引（1 或 2）
        timeout_sec: 超时秒数
    """
    try:
        ok, msg = sg.validate_camera_name(camera_name)
        sg.check_or_raise(ok, msg)
        ok, msg = sg.validate_infrared_index(index)
        sg.check_or_raise(ok, msg)
        if not save_path:
            save_path = os.path.join(DEFAULT_OUTPUT_DIR, f"{camera_name}_infra{index}.png")
        if save_path:
            ok, msg = sg.validate_file_path(save_path)
            sg.check_or_raise(ok, msg)

        result = _get_bridge().capture_infrared(camera_name, save_path, index, timeout_sec)
        return _ok(result)
    except (ValueError, RuntimeError) as e:
        return _err(str(e))


@mcp.tool()
def get_imu_data(camera_name: str, timeout_sec: float = 5.0) -> str:
    """获取 IMU 数据（方向、角速度、线加速度）。

    Args:
        camera_name: 相机名称
        timeout_sec: 超时秒数
    """
    try:
        ok, msg = sg.validate_camera_name(camera_name)
        sg.check_or_raise(ok, msg)
        result = _get_bridge().get_imu_data(camera_name, timeout_sec)
        return _ok(result)
    except (ValueError, RuntimeError) as e:
        return _err(str(e))


@mcp.tool()
def capture_pointcloud(camera_name: str, save_path: str = "", timeout_sec: float = 5.0) -> str:
    """从 ROS2 点云 topic 捕获点云并保存为 PLY/PCD 文件。

    Args:
        camera_name: 相机名称
        save_path: 保存路径（.ply 或 .pcd，默认 .pcd）
        timeout_sec: 超时秒数
    """
    try:
        ok, msg = sg.validate_camera_name(camera_name)
        sg.check_or_raise(ok, msg)
        if not save_path:
            save_path = os.path.join(DEFAULT_OUTPUT_DIR, f"{camera_name}_pointcloud.pcd")
        if save_path:
            ok, msg = sg.validate_file_path(save_path)
            sg.check_or_raise(ok, msg)

        result = _get_bridge().capture_pointcloud(camera_name, save_path, timeout_sec)
        return _ok(result)
    except (ValueError, RuntimeError) as e:
        return _err(str(e))


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# 3. 服务调用
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

@mcp.tool()
def get_device_info(camera_name: str) -> str:
    """通过 ROS2 DeviceInfo 服务获取设备信息（设备名、序列号、固件版本等）。

    Args:
        camera_name: 相机名称
    """
    try:
        ok, msg = sg.validate_camera_name(camera_name)
        sg.check_or_raise(ok, msg)
        result = _get_bridge().get_device_info(camera_name)
        return _ok(result)
    except (ValueError, RuntimeError) as e:
        return _err(str(e))


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# 4. 参数控制
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

@mcp.tool()
def get_camera_parameters(camera_name: str) -> str:
    """获取 ROS2 相机节点的所有参数。

    Args:
        camera_name: 相机名称
    """
    try:
        ok, msg = sg.validate_camera_name(camera_name)
        sg.check_or_raise(ok, msg)
        result = _get_bridge().get_camera_parameters(camera_name)
        return _ok(result)
    except (ValueError, RuntimeError) as e:
        return _err(str(e))


@mcp.tool()
def set_camera_parameter(camera_name: str, param_name: str, value: str) -> str:
    """设置 ROS2 相机节点的参数。

    Args:
        camera_name: 相机名称
        param_name: 参数名（如 "spatial_filter.enable", "depth_module.exposure"）
        value: 参数值（字符串形式）
    """
    try:
        ok, msg = sg.validate_camera_name(camera_name)
        sg.check_or_raise(ok, msg)
        result = _get_bridge().set_camera_parameter(camera_name, param_name, value)
        return _ok(result)
    except (ValueError, RuntimeError) as e:
        return _err(str(e))


@mcp.tool()
def enable_filter(camera_name: str, filter_name: str, enable: bool = True) -> str:
    """启用/禁用深度滤波器（通过 ROS2 参数动态设置）。

    Args:
        camera_name: 相机名称
        filter_name: 滤波器名称（spatial/temporal/decimation/hole_filling/disparity/hdr_merge）
        enable: True=启用, False=禁用
    """
    try:
        ok, msg = sg.validate_camera_name(camera_name)
        sg.check_or_raise(ok, msg)
        ok, msg = sg.validate_filter_name(filter_name)
        sg.check_or_raise(ok, msg)
        result = _get_bridge().enable_filter(camera_name, filter_name, enable)
        return _ok(result)
    except (ValueError, RuntimeError) as e:
        return _err(str(e))


@mcp.tool()
def set_depth_profile(camera_name: str, width: int, height: int, fps: int) -> str:
    """设置深度流分辨率和帧率（通过 ROS2 参数）。

    Args:
        camera_name: 相机名称
        width: 宽度
        height: 高度
        fps: 帧率
    """
    try:
        ok, msg = sg.validate_camera_name(camera_name)
        sg.check_or_raise(ok, msg)
        ok, msg = sg.validate_resolution(width, height)
        sg.check_or_raise(ok, msg)
        ok, msg = sg.validate_fps(fps)
        sg.check_or_raise(ok, msg)
        result = _get_bridge().set_depth_profile(camera_name, width, height, fps)
        return _ok(result)
    except (ValueError, RuntimeError) as e:
        return _err(str(e))


@mcp.tool()
def set_color_profile(camera_name: str, width: int, height: int, fps: int) -> str:
    """设置彩色流分辨率和帧率（通过 ROS2 参数）。

    Args:
        camera_name: 相机名称
        width: 宽度
        height: 高度
        fps: 帧率
    """
    try:
        ok, msg = sg.validate_camera_name(camera_name)
        sg.check_or_raise(ok, msg)
        ok, msg = sg.validate_resolution(width, height)
        sg.check_or_raise(ok, msg)
        ok, msg = sg.validate_fps(fps)
        sg.check_or_raise(ok, msg)
        result = _get_bridge().set_color_profile(camera_name, width, height, fps)
        return _ok(result)
    except (ValueError, RuntimeError) as e:
        return _err(str(e))


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# 5. TF 和标定
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

@mcp.tool()
def get_camera_info(camera_name: str, stream: str = "color") -> str:
    """获取 CameraInfo（相机内参矩阵 K、畸变系数 D、投影矩阵 P）。

    Args:
        camera_name: 相机名称
        stream: 流名称（color/depth/infra1/infra2/aligned_depth_to_color）
    """
    try:
        ok, msg = sg.validate_camera_name(camera_name)
        sg.check_or_raise(ok, msg)
        ok, msg = sg.validate_stream_name(stream)
        sg.check_or_raise(ok, msg)
        result = _get_bridge().get_camera_info(camera_name, stream)
        return _ok(result)
    except (ValueError, RuntimeError) as e:
        return _err(str(e))


@mcp.tool()
def get_tf(camera_name: str, from_frame: str = "depth", to_frame: str = "color") -> str:
    """查询 TF 变换（两个坐标系之间的位姿变换）。

    Args:
        camera_name: 相机名称（用于构建 frame ID，如 camera1_depth_frame）
        from_frame: 源坐标系后缀（depth/color/infra1 等）
        to_frame: 目标坐标系后缀
    """
    try:
        ok, msg = sg.validate_camera_name(camera_name)
        sg.check_or_raise(ok, msg)
        result = _get_bridge().get_tf(camera_name, from_frame, to_frame)
        return _ok(result)
    except (ValueError, RuntimeError) as e:
        return _err(str(e))


@mcp.tool()
def get_extrinsics(camera_name: str) -> str:
    """获取 depth-to-color 外参（旋转矩阵 + 平移向量）。

    Args:
        camera_name: 相机名称
    """
    try:
        ok, msg = sg.validate_camera_name(camera_name)
        sg.check_or_raise(ok, msg)
        result = _get_bridge().get_extrinsics(camera_name)
        return _ok(result)
    except (ValueError, RuntimeError) as e:
        return _err(str(e))


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# 6. 多相机
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

@mcp.tool()
def launch_multi_cameras(configs: str) -> str:
    """批量启动多个 RealSense 相机 ROS2 节点。

    Args:
        configs: JSON 数组字符串，每项包含 serial, camera_name 及可选参数。
                 示例: '[{"serial":"231122070092","camera_name":"camera1"},{"serial":"147322071616","camera_name":"camera2"}]'
    """
    try:
        cfg_list = json.loads(configs)
        if not isinstance(cfg_list, list):
            return _err("configs 必须是 JSON 数组")

        results = []
        for cfg in cfg_list:
            serial = cfg.get("serial", "")
            name = cfg.get("camera_name", "")
            if not serial or not name:
                results.append({"error": "缺少 serial 或 camera_name", "config": cfg})
                continue
            try:
                result = _get_bridge().launch_camera(
                    serial=serial,
                    camera_name=name,
                    camera_namespace=cfg.get("camera_namespace", ""),
                    enable_color=cfg.get("enable_color", True),
                    enable_depth=cfg.get("enable_depth", True),
                    enable_infra=cfg.get("enable_infra", False),
                    enable_imu=cfg.get("enable_imu", False),
                    enable_pointcloud=cfg.get("enable_pointcloud", False),
                    enable_align_depth=cfg.get("enable_align_depth", True),
                    width=cfg.get("width", 640),
                    height=cfg.get("height", 480),
                    fps=cfg.get("fps", 30),
                )
                results.append(result)
            except Exception as e:
                results.append({"camera_name": name, "error": str(e)})

        return _ok({"results": results})
    except json.JSONDecodeError as e:
        return _err(f"JSON 解析失败: {e}")


@mcp.tool()
def capture_multi_frames(camera_names: str, save_dir: str = "", aligned: bool = True) -> str:
    """从多个相机同时捕获 RGBD 帧并保存。

    Args:
        camera_names: JSON 数组字符串，相机名称列表。示例: '["camera1","camera2"]'
        save_dir: 保存目录（默认 /tmp/realsense-ros/multi/）
        aligned: 使用对齐深度
    """
    try:
        names = json.loads(camera_names)
        if not isinstance(names, list):
            return _err("camera_names 必须是 JSON 数组")

        if not save_dir:
            save_dir = os.path.join(DEFAULT_OUTPUT_DIR, "multi")
        os.makedirs(save_dir, exist_ok=True)

        results = []
        for name in names:
            try:
                color_path = os.path.join(save_dir, f"{name}_color.png")
                depth_path = os.path.join(save_dir, f"{name}_depth.png")
                r = _get_bridge().capture_rgbd(name, color_path, depth_path, aligned=aligned)
                results.append(r)
            except Exception as e:
                results.append({"camera_name": name, "error": str(e)})

        return _ok({"results": results, "save_dir": save_dir})
    except json.JSONDecodeError as e:
        return _err(f"JSON 解析失败: {e}")


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# 7. ROS2 诊断
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

@mcp.tool()
def check_ros2_status() -> str:
    """检查 ROS2 环境状态（ROS_DISTRO、daemon、rclpy 可用性）。"""
    try:
        result = _get_bridge().check_ros2_status()
        return _ok(result)
    except Exception as e:
        return _err(str(e))


@mcp.tool()
def list_ros2_nodes() -> str:
    """列出所有当前运行的 ROS2 节点。"""
    try:
        result = _get_bridge().list_ros2_nodes()
        return _ok(result)
    except Exception as e:
        return _err(str(e))


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# Entry point
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

if __name__ == "__main__":
    if "--test" in sys.argv:
        print("✅ realsense-ros-mcp server imported successfully")
        print(f"   Tools registered: {len(mcp._tool_manager._tools)}")
        for name in sorted(mcp._tool_manager._tools.keys()):
            print(f"   - {name}")
        sys.exit(0)

    mcp.run(transport="stdio")
