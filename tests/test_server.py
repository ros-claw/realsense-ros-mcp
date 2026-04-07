#!/usr/bin/env python3
"""
test_server.py — 验证 realsense-ros-mcp server 能正常导入和初始化。
不需要 ROS2 环境或硬件即可运行。
"""

import sys
import os

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

PASSED = 0
FAILED = 0


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

    print("\n=== realsense-ros-mcp Server Tests ===\n")

    # ── Test 1: safety_guard 可导入 ──
    print("[1] Import safety_guard")
    try:
        import safety_guard as sg
        test("import safety_guard", True)
        test("validate_resolution exists", callable(getattr(sg, "validate_resolution", None)))
        test("validate_camera_name exists", callable(getattr(sg, "validate_camera_name", None)))
        test("validate_file_path exists", callable(getattr(sg, "validate_file_path", None)))
        test("check_or_raise exists", callable(getattr(sg, "check_or_raise", None)))
    except Exception as e:
        test("import safety_guard", False, str(e))

    # ── Test 2: safety_guard 校验逻辑 ──
    print("\n[2] SafetyGuard validation")
    import safety_guard as sg

    ok, _ = sg.validate_resolution(640, 480)
    test("valid resolution 640x480", ok)

    ok, _ = sg.validate_resolution(4096, 2160)
    test("reject resolution 4096x2160", not ok)

    ok, _ = sg.validate_fps(30)
    test("valid fps 30", ok)

    ok, _ = sg.validate_fps(120)
    test("reject fps 120", not ok)

    ok, _ = sg.validate_camera_name("camera1")
    test("valid camera_name 'camera1'", ok)

    ok, _ = sg.validate_camera_name("camera 1")
    test("reject camera_name with space", not ok)

    ok, _ = sg.validate_camera_name("")
    test("reject empty camera_name", not ok)

    ok, _ = sg.validate_serial("231122070092")
    test("valid serial", ok)

    ok, _ = sg.validate_file_path("/tmp/realsense-ros/test.png")
    test("valid file path /tmp/...", ok)

    ok, _ = sg.validate_file_path("/etc/passwd")
    test("reject /etc/passwd", not ok)

    ok, _ = sg.validate_file_path("relative/path.png")
    test("reject relative path", not ok)

    ok, _ = sg.validate_timeout(5.0)
    test("valid timeout 5.0", ok)

    ok, _ = sg.validate_timeout(60.0)
    test("reject timeout 60.0", not ok)

    ok, _ = sg.validate_filter_name("spatial")
    test("valid filter 'spatial'", ok)

    ok, _ = sg.validate_filter_name("unknown_filter")
    test("reject unknown filter", not ok)

    ok, _ = sg.validate_stream_name("color")
    test("valid stream 'color'", ok)

    ok, _ = sg.validate_stream_name("unknown")
    test("reject unknown stream", not ok)

    ok, _ = sg.validate_infrared_index(1)
    test("valid infrared index 1", ok)

    ok, _ = sg.validate_infrared_index(3)
    test("reject infrared index 3", not ok)

    # ── Test 3: bridge 可导入 ──
    print("\n[3] Import bridge")
    try:
        import bridge
        test("import bridge", True)
        test("ROS2Bridge class exists", hasattr(bridge, "ROS2Bridge"))
        test("DEFAULT_OUTPUT_DIR defined", hasattr(bridge, "DEFAULT_OUTPUT_DIR"))
    except Exception as e:
        test("import bridge", False, str(e))

    # ── Test 4: bridge 方法签名 ──
    print("\n[4] Bridge method signatures")
    from bridge import ROS2Bridge
    expected_methods = [
        "init_ros2", "shutdown_ros2", "launch_camera", "stop_camera",
        "list_active_cameras", "list_topics", "get_topic_info", "get_topic_hz",
        "capture_color_image", "capture_depth_image", "capture_rgbd",
        "capture_infrared", "get_imu_data", "capture_pointcloud",
        "get_device_info", "get_camera_info", "get_tf", "get_extrinsics",
        "get_camera_parameters", "set_camera_parameter", "enable_filter",
        "set_depth_profile", "set_color_profile",
        "check_ros2_status", "list_ros2_nodes",
    ]
    for method in expected_methods:
        test(f"method: {method}", hasattr(ROS2Bridge, method) and callable(getattr(ROS2Bridge, method)))

    # ── Test 5: launch 命令构建 ──
    print("\n[5] Launch command building")
    cmd = ROS2Bridge._build_launch_cmd(
        serial="231122070092",
        camera_name="camera1",
        camera_namespace="camera1",
        enable_color=True,
        enable_depth=True,
        enable_infra=False,
        enable_imu=False,
        enable_pointcloud=False,
        enable_align_depth=True,
        profile="640,480,30",
    )
    test("cmd is list", isinstance(cmd, list))
    test("cmd starts with ros2", cmd[0] == "ros2")
    test("cmd has serial", any("231122070092" in c for c in cmd))
    test("cmd has camera_name", any("camera1" in c for c in cmd))
    test("cmd has enable_color=true", any("enable_color:=true" in c for c in cmd))

    # ── Test 6: topic prefix ──
    print("\n[6] Topic prefix")
    prefix = ROS2Bridge._topic_prefix("camera1")
    test("topic prefix format", prefix == "/camera1/camera1")

    # ── Test 7: mcp_server 可导入 ──
    print("\n[7] Import mcp_server")
    try:
        import mcp_server
        test("import mcp_server", True)
        test("mcp object exists", hasattr(mcp_server, "mcp"))
    except Exception as e:
        test("import mcp_server", False, str(e))

    # ── Test 8: MCP tool 函数签名 ──
    print("\n[8] MCP tool function signatures")
    if not hasattr(sys.modules.get("mcp_server", None), "mcp"):
        print("  ⚠️  mcp_server 未成功导入，跳过 tool 签名检查")
        total = PASSED + FAILED
        print(f"\n{'='*50}")
        print(f"Results: {PASSED}/{total} passed, {FAILED} failed")
        sys.exit(0 if FAILED == 0 else 1)
    import mcp_server
    expected_tools = [
        "launch_camera", "stop_camera", "list_active_cameras",
        "list_topics", "get_topic_info", "get_topic_hz",
        "capture_color_image", "capture_depth_image", "capture_rgbd",
        "capture_infrared", "get_imu_data", "capture_pointcloud",
        "get_device_info",
        "get_camera_parameters", "set_camera_parameter", "enable_filter",
        "set_depth_profile", "set_color_profile",
        "get_camera_info", "get_tf", "get_extrinsics",
        "launch_multi_cameras", "capture_multi_frames",
        "check_ros2_status", "list_ros2_nodes",
    ]
    for tool_name in expected_tools:
        exists = hasattr(mcp_server, tool_name) and callable(getattr(mcp_server, tool_name))
        test(f"tool function: {tool_name}", exists)

    # ── Summary ──
    total = PASSED + FAILED
    print(f"\n{'='*50}")
    print(f"Results: {PASSED}/{total} passed, {FAILED} failed")
    if FAILED == 0:
        print("✅ All tests passed!")
    else:
        print("❌ Some tests failed")
    print()

    sys.exit(0 if FAILED == 0 else 1)


if __name__ == "__main__":
    main()
