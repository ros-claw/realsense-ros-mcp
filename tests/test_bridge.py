#!/usr/bin/env python3
"""
test_bridge.py — 测试 ROS2Bridge 功能。
需要 ROS2 环境和 RealSense 硬件。
"""

import sys
import os
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

PASSED = 0
FAILED = 0

SERIAL_1 = "231122070092"
SERIAL_2 = "147322071616"
CAM_NAME = "test_cam1"


def test(name: str, condition: bool, detail: str = "") -> None:
    global PASSED, FAILED
    if condition:
        PASSED += 1
        print(f"  ✅ {name}")
    else:
        FAILED += 1
        print(f"  ❌ {name}: {detail}")


def main() -> None:
    global PASSED, FAILED

    print("\n=== realsense-ros-mcp Bridge Tests (需要 ROS2 + 硬件) ===\n")

    from bridge import ROS2Bridge

    bridge = ROS2Bridge()

    # ── Test 1: ROS2 环境检测 ──
    print("[1] ROS2 Status")
    status = bridge.check_ros2_status()
    test("check_ros2_status returns dict", isinstance(status, dict))
    test("has ros_distro", "ros_distro" in status)
    print(f"    ROS_DISTRO: {status.get('ros_distro')}")
    print(f"    rclpy available: {status.get('rclpy_available')}")

    if not status.get("rclpy_available"):
        print("\n⚠️  ROS2 不可用，跳过硬件测试")
        print(f"    Error: {status.get('rclpy_error', 'unknown')}")
        sys.exit(1)

    # ── Test 2: ROS2 初始化 ──
    print("\n[2] ROS2 Init")
    result = bridge.init_ros2()
    test("init_ros2 succeeds", result.get("success", False), result.get("message", ""))

    # ── Test 3: 节点列表 ──
    print("\n[3] Node List (before launch)")
    nodes = bridge.list_ros2_nodes()
    test("list_ros2_nodes returns dict", isinstance(nodes, dict))
    if nodes.get("success"):
        print(f"    Active nodes: {nodes.get('nodes', [])}")

    # ── Test 4: 启动相机 ──
    print("\n[4] Launch Camera")
    result = bridge.launch_camera(
        serial=SERIAL_1,
        camera_name=CAM_NAME,
        enable_color=True,
        enable_depth=True,
        enable_align_depth=True,
        width=640, height=480, fps=15,
    )
    test("launch_camera succeeds", result.get("success", False), result.get("message", ""))

    if result.get("success"):
        print(f"    PID: {result.get('pid')}")
        print("    等待节点启动 (10s)...")
        time.sleep(10)

    # ── Test 5: 活动相机列表 ──
    print("\n[5] Active Cameras")
    cameras = bridge.list_active_cameras()
    test("list_active_cameras returns dict", isinstance(cameras, dict))
    cam_list = cameras.get("cameras", [])
    test("has launched camera", any(c.get("camera_name") == CAM_NAME for c in cam_list))

    # ── Test 6: Topic 列表 ──
    print("\n[6] Topics")
    topics = bridge.list_topics(camera_name=CAM_NAME)
    test("list_topics returns dict", isinstance(topics, dict))
    topic_list = topics.get("topics", [])
    print(f"    Found {len(topic_list)} topics for {CAM_NAME}")
    for t in topic_list[:5]:
        print(f"      {t['topic']} [{t['type']}]")

    # ── Test 7: 捕获彩色图 ──
    print("\n[7] Capture Color Image")
    color_path = f"/tmp/realsense-ros/{CAM_NAME}_test_color.png"
    result = bridge.capture_color_image(CAM_NAME, color_path, timeout_sec=10.0)
    test("capture_color_image succeeds", result.get("success", False), result.get("message", ""))
    if result.get("success"):
        test("file exists", os.path.isfile(result.get("path", "")))
        print(f"    Saved: {result.get('path')} ({result.get('width')}x{result.get('height')})")

    # ── Test 8: 捕获深度图 ──
    print("\n[8] Capture Depth Image")
    depth_path = f"/tmp/realsense-ros/{CAM_NAME}_test_depth.png"
    result = bridge.capture_depth_image(CAM_NAME, depth_path, aligned=True, colorize=True, timeout_sec=10.0)
    test("capture_depth_image succeeds", result.get("success", False), result.get("message", ""))
    if result.get("success"):
        test("file exists", os.path.isfile(result.get("path", "")))

    # ── Test 9: RGBD ──
    print("\n[9] Capture RGBD")
    result = bridge.capture_rgbd(
        CAM_NAME,
        f"/tmp/realsense-ros/{CAM_NAME}_test_rgbd_color.png",
        f"/tmp/realsense-ros/{CAM_NAME}_test_rgbd_depth.png",
        timeout_sec=10.0,
    )
    test("capture_rgbd succeeds", result.get("success", False), result.get("message", ""))

    # ── Test 10: CameraInfo ──
    print("\n[10] CameraInfo")
    info = bridge.get_camera_info(CAM_NAME, "color")
    test("get_camera_info succeeds", info.get("success", False), info.get("message", ""))
    if info.get("success"):
        print(f"    {info.get('width')}x{info.get('height')}, model={info.get('distortion_model')}")

    # ── Test 11: 参数 ──
    print("\n[11] Camera Parameters")
    params = bridge.get_camera_parameters(CAM_NAME)
    test("get_camera_parameters returns result", isinstance(params, dict))
    if params.get("success"):
        param_text = params.get("parameters", "")
        print(f"    Parameters (first 200 chars): {param_text[:200]}...")

    # ── Cleanup ──
    print("\n[Cleanup] Stop camera")
    result = bridge.stop_camera(CAM_NAME)
    test("stop_camera succeeds", result.get("success", False))

    bridge.shutdown_ros2()

    # ── Summary ──
    total = PASSED + FAILED
    print(f"\n{'='*50}")
    print(f"Results: {PASSED}/{total} passed, {FAILED} failed")
    if FAILED == 0:
        print("✅ All bridge tests passed!")
    else:
        print(f"❌ {FAILED} tests failed")
    print()

    sys.exit(0 if FAILED == 0 else 1)


if __name__ == "__main__":
    main()
