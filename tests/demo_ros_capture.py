#!/usr/bin/env python3
"""
demo_ros_capture.py — 完整演示：检查 ROS2 -> 启动相机 -> 捕获 RGBD -> 点云 -> 停止

直接调用 bridge.py API，不经过 MCP。
用法: python tests/demo_ros_capture.py [serial] [camera_name]
"""

import sys
import os
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from bridge import ROS2Bridge

OUTPUT_DIR = "/tmp/realsense-ros/demo"
DEFAULT_SERIAL = "231122070092"
DEFAULT_CAM_NAME = "demo_cam"


def main() -> None:
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    bridge = ROS2Bridge()

    serial = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_SERIAL
    cam_name = sys.argv[2] if len(sys.argv) > 2 else DEFAULT_CAM_NAME

    print("=" * 60)
    print("RealSense ROS2 Demo Capture")
    print("=" * 60)

    # ── 1. 检查 ROS2 环境 ──
    print("\n🔍 检查 ROS2 环境...")
    status = bridge.check_ros2_status()
    print(f"   ROS_DISTRO: {status.get('ros_distro', 'unknown')}")
    print(f"   rclpy: {'✅' if status.get('rclpy_available') else '❌'}")
    print(f"   daemon: {status.get('daemon_status', 'unknown')}")

    if not status.get("rclpy_available"):
        print(f"\n❌ ROS2 不可用: {status.get('rclpy_error', 'unknown')}")
        print("   请确保已 source /opt/ros/humble/setup.bash")
        sys.exit(1)

    # ── 2. 初始化 ROS2 ──
    print("\n🚀 初始化 ROS2 bridge...")
    result = bridge.init_ros2()
    print(f"   {result['message']}")

    # ── 3. 列出已有节点 ──
    print("\n📋 当前 ROS2 节点:")
    nodes = bridge.list_ros2_nodes()
    for n in nodes.get("nodes", []):
        print(f"   {n}")

    # ── 4. 启动相机 ──
    print(f"\n📷 启动相机: {cam_name} (serial={serial})...")
    result = bridge.launch_camera(
        serial=serial,
        camera_name=cam_name,
        enable_color=True,
        enable_depth=True,
        enable_align_depth=True,
        enable_pointcloud=True,
        width=640, height=480, fps=15,
    )
    if not result.get("success"):
        print(f"   ❌ 启动失败: {result.get('message')}")
        sys.exit(1)
    print(f"   ✅ PID: {result.get('pid')}")

    # ── 5. 等待节点就绪 ──
    print("\n⏳ 等待节点就绪 (15s)...")
    for i in range(15, 0, -1):
        sys.stdout.write(f"\r   {i}s... ")
        sys.stdout.flush()
        time.sleep(1)
    print("\r   ✅ 就绪!    ")

    # ── 6. 列出 topics ──
    print(f"\n📡 相机 topics:")
    topics = bridge.list_topics(camera_name=cam_name)
    for t in topics.get("topics", [])[:10]:
        print(f"   {t['topic']} [{t['type']}]")
    total_topics = len(topics.get("topics", []))
    if total_topics > 10:
        print(f"   ... 共 {total_topics} 个 topics")

    # ── 7. 捕获彩色图 ──
    print(f"\n🎨 捕获彩色图...")
    color_path = os.path.join(OUTPUT_DIR, f"{cam_name}_color.png")
    result = bridge.capture_color_image(cam_name, color_path, timeout_sec=10.0)
    if result.get("success"):
        print(f"   ✅ {result['path']} ({result['width']}x{result['height']})")
    else:
        print(f"   ❌ {result.get('message')}")

    # ── 8. 捕获深度图（对齐+伪彩色） ──
    print(f"\n🌈 捕获深度图 (对齐+伪彩色)...")
    depth_path = os.path.join(OUTPUT_DIR, f"{cam_name}_aligned_depth.png")
    result = bridge.capture_depth_image(cam_name, depth_path, aligned=True, colorize=True, timeout_sec=10.0)
    if result.get("success"):
        print(f"   ✅ {result['path']}")
    else:
        print(f"   ❌ {result.get('message')}")

    # ── 9. 捕获 RGBD ──
    print(f"\n🔗 捕获 RGBD 对...")
    result = bridge.capture_rgbd(
        cam_name,
        os.path.join(OUTPUT_DIR, f"{cam_name}_rgbd_color.png"),
        os.path.join(OUTPUT_DIR, f"{cam_name}_rgbd_depth.png"),
        aligned=True,
        timeout_sec=10.0,
    )
    if result.get("success"):
        print(f"   ✅ color: {result['color']['path']}")
        print(f"   ✅ depth: {result['depth']['path']}")
    else:
        print(f"   ❌ {result.get('message')}")

    # ── 10. CameraInfo (内参) ──
    print(f"\n📐 CameraInfo (color)...")
    info = bridge.get_camera_info(cam_name, "color")
    if info.get("success"):
        print(f"   {info['width']}x{info['height']}, model={info['distortion_model']}")
        K = info.get("K", [])
        if len(K) >= 9:
            print(f"   K: fx={K[0]:.1f}, fy={K[4]:.1f}, cx={K[2]:.1f}, cy={K[5]:.1f}")
    else:
        print(f"   ❌ {info.get('message')}")

    # ── 11. 点云 ──
    print(f"\n☁️  捕获点云...")
    pc_path = os.path.join(OUTPUT_DIR, f"{cam_name}_pointcloud.ply")
    result = bridge.capture_pointcloud(cam_name, pc_path, timeout_sec=10.0)
    if result.get("success"):
        print(f"   ✅ {result['path']} ({result['num_points']} 个点)")
    else:
        print(f"   ❌ {result.get('message')}")

    # ── 12. DeviceInfo 服务 ──
    print(f"\n📋 DeviceInfo 服务...")
    info = bridge.get_device_info(cam_name)
    if info.get("success"):
        print(f"   device: {info.get('device_name')}")
        print(f"   serial: {info.get('serial_number')}")
        print(f"   FW: {info.get('firmware_version')}")
        print(f"   USB: {info.get('usb_type_descriptor')}")
    else:
        print(f"   ❌ {info.get('message')}")

    # ── 13. 停止 ──
    print(f"\n⏹  停止相机...")
    result = bridge.stop_camera(cam_name)
    print(f"   {result.get('message')}")

    bridge.shutdown_ros2()

    # ── 汇总 ──
    print(f"\n{'='*60}")
    print(f"✅ Demo 完成! 输出文件在: {OUTPUT_DIR}")
    if os.path.isdir(OUTPUT_DIR):
        files = os.listdir(OUTPUT_DIR)
        for f in sorted(files):
            size = os.path.getsize(os.path.join(OUTPUT_DIR, f))
            print(f"   {f} ({size:,} bytes)")
    print(f"{'='*60}\n")


if __name__ == "__main__":
    main()
