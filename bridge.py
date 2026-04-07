"""
ROS2Bridge - Bridge between MCP Server and ROS2 realsense2_camera.

Manages ROS2 node lifecycle, camera launch processes, topic subscriptions,
service calls, and parameter operations. Designed for single-frame capture
with spin_once timeout pattern (non-blocking).

Based on sdk_to_mcp framework with SDK metadata tracking and safety constraints.

Requirements:
    - ROS2 Humble (source /opt/ros/humble/setup.bash)
    - realsense2_camera package installed
    - cv_bridge, sensor_msgs, rclpy from ROS2

SDK Metadata:
    Name: realsense-ros (realsense2_camera)
    Version: 4.55.1+ (ROS2 Humble)
    Source: https://github.com/IntelRealSense/realsense-ros
    Docs: https://github.com/IntelRealSense/realsense-ros/blob/ros2-development/README.md
    License: Apache-2.0
"""

import os
import sys
import json
import signal
import struct
import subprocess
import threading
import time
import logging
from typing import Dict, List, Optional, Any, Tuple
from dataclasses import dataclass, field, asdict
from datetime import datetime

logger = logging.getLogger("realsense_mcp.bridge")

# ── SDK Metadata ─────────────────────────────────────────────────────────────

@dataclass
class SDKMetadata:
    """
    SDK 元数据 - 基于 sdk_to_mcp 框架
    
    跟踪版本信息、源代码引用和依赖关系，
    确保 MCP server 与底层 SDK 保持同步。
    """
    name: str = "realsense-ros"
    version: str = "4.55.1+"
    protocol: str = "ROS2/topics/services"
    source_url: str = "https://github.com/IntelRealSense/realsense-ros"
    doc_url: str = "https://github.com/IntelRealSense/realsense-ros/blob/ros2-development/README.md"
    license: str = "Apache-2.0"
    hardware_models: List[str] = field(default_factory=lambda: [
        "D435", "D435i", "D455", "D415", "D405", "L515"
    ])
    dependencies: Dict[str, str] = field(default_factory=lambda: {
        "ros2": "humble",
        "realsense2_camera": ">=4.55.1",
        "python": ">=3.8"
    })
    checksum: str = ""
    extracted_date: str = field(default_factory=lambda: datetime.now().isoformat())
    notes: str = "Intel RealSense ROS2 Driver"
    
    def to_dict(self) -> Dict[str, Any]:
        """转换为字典格式"""
        return asdict(self)
    
    @classmethod
    def get_instance(cls) -> "SDKMetadata":
        """获取 SDK 元数据实例"""
        return cls()


# Default image output directory
DEFAULT_OUTPUT_DIR = "/tmp/realsense-ros"

# ROS2 environment setup script
ROS2_SETUP_BASH = "/opt/ros/humble/setup.bash"

# --------------------------------------------------------------------------- #
# Lazy imports for ROS2 modules — they are only available after sourcing ROS2  #
# --------------------------------------------------------------------------- #
_rclpy = None
_Node = None
_Image = None
_CameraInfo = None
_Imu = None
_PointCloud2 = None
_CvBridge = None
_cv_bridge_broken = False  # True if cv_bridge exists but crashes (numpy ABI mismatch)
_DeviceInfo = None
_GetParameters = None
_SetParameters = None
_Parameter = None
_ParameterValue = None
_ParameterType = None

_ros2_imported = False
_ros2_import_error: Optional[str] = None


def _try_import_ros2() -> bool:
    """
    Attempt to import ROS2 Python modules.

    Returns:
        True if all imports succeeded, False otherwise.
    """
    global _rclpy, _Node, _Image, _CameraInfo, _Imu, _PointCloud2
    global _CvBridge, _DeviceInfo
    global _GetParameters, _SetParameters, _Parameter, _ParameterValue, _ParameterType
    global _ros2_imported, _ros2_import_error

    if _ros2_imported:
        return True
    if _ros2_import_error is not None:
        return False

    global _cv_bridge_broken

    try:
        import rclpy
        from rclpy.node import Node
        from sensor_msgs.msg import Image, CameraInfo, Imu, PointCloud2

        _rclpy = rclpy
        _Node = Node
        _Image = Image
        _CameraInfo = CameraInfo
        _Imu = Imu
        _PointCloud2 = PointCloud2

        # Skip cv_bridge entirely — it segfaults with numpy 2.x due to ABI mismatch.
        # We use pure-numpy image conversion instead (see _imgmsg_to_numpy).
        _CvBridge = None
        _cv_bridge_broken = True
        logger.info("Using pure-numpy image conversion (cv_bridge skipped for numpy 2.x safety)")

        try:
            from realsense2_camera_msgs.srv import DeviceInfo
            _DeviceInfo = DeviceInfo
        except ImportError:
            logger.warning("realsense2_camera_msgs not found; DeviceInfo service unavailable")
            _DeviceInfo = None

        try:
            from rcl_interfaces.srv import GetParameters, SetParameters
            from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
            _GetParameters = GetParameters
            _SetParameters = SetParameters
            _Parameter = Parameter
            _ParameterValue = ParameterValue
            _ParameterType = ParameterType
        except ImportError:
            logger.warning("rcl_interfaces not found; parameter services unavailable")

        _ros2_imported = True
        logger.info("ROS2 Python modules imported successfully")
        return True

    except ImportError as exc:
        _ros2_import_error = str(exc)
        logger.error(f"Failed to import ROS2 modules: {exc}")
        return False


def _ensure_output_dir(path: str) -> None:
    """Create parent directories for the given file path."""
    dirpath = os.path.dirname(path)
    if dirpath:
        os.makedirs(dirpath, exist_ok=True)


class ROS2Bridge:
    """
    Bridge class that wraps ROS2 interactions for the RealSense MCP Server.

    Responsibilities:
        - Manage a single rclpy node for subscriptions and service calls
        - Launch / stop realsense2_camera processes via subprocess
        - Capture single frames from image / depth / IMU / pointcloud topics
        - Query and set ROS2 parameters
        - Query TF transforms
        - Thread-safe access to shared state
    """

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._node: Optional[Any] = None  # rclpy Node
        self._processes: Dict[str, subprocess.Popen] = {}  # camera_name -> Popen
        self._cv_bridge: Optional[Any] = None  # CvBridge instance
        self._rclpy_initialized = False
        self._output_dir = DEFAULT_OUTPUT_DIR
        os.makedirs(self._output_dir, exist_ok=True)

    # ------------------------------------------------------------------ #
    # ROS2 Lifecycle                                                      #
    # ------------------------------------------------------------------ #

    def init_ros2(self) -> Dict[str, Any]:
        """
        Initialize rclpy and create the bridge node.

        Returns:
            Dict with 'success' and 'message' keys.
        """
        if not _try_import_ros2():
            return {"success": False, "message": f"ROS2 import failed: {_ros2_import_error}"}

        with self._lock:
            if self._rclpy_initialized:
                return {"success": True, "message": "ROS2 already initialized"}

            try:
                _rclpy.init()
                self._rclpy_initialized = True
            except RuntimeError:
                # Already initialized in this process
                self._rclpy_initialized = True

            try:
                self._node = _rclpy.create_node("realsense_mcp_bridge")
                self._cv_bridge = _CvBridge() if _CvBridge is not None else None
                logger.info("ROS2 bridge node created" + (" (cv_bridge unavailable, using numpy)" if self._cv_bridge is None else ""))
                return {"success": True, "message": "ROS2 initialized, bridge node created"}
            except Exception as exc:
                return {"success": False, "message": f"Failed to create node: {exc}"}

    def shutdown_ros2(self) -> Dict[str, Any]:
        """
        Shutdown rclpy and destroy the bridge node.

        Returns:
            Dict with 'success' and 'message' keys.
        """
        with self._lock:
            # Stop all camera processes first
            for name in list(self._processes.keys()):
                self._stop_process(name)

            if self._node is not None:
                try:
                    self._node.destroy_node()
                except Exception:
                    pass
                self._node = None

            if self._rclpy_initialized:
                try:
                    _rclpy.shutdown()
                except Exception:
                    pass
                self._rclpy_initialized = False

            self._cv_bridge = None
            return {"success": True, "message": "ROS2 shutdown complete"}

    def _ensure_node(self) -> None:
        """Ensure the ROS2 node is available, raise RuntimeError if not."""
        if self._node is None:
            result = self.init_ros2()
            if not result["success"]:
                raise RuntimeError(f"ROS2 node unavailable: {result['message']}")

    # ------------------------------------------------------------------ #
    # Launch Process Management                                           #
    # ------------------------------------------------------------------ #

    def launch_camera(
        self,
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
    ) -> Dict[str, Any]:
        """
        Launch a realsense2_camera node via ros2 launch.

        Args:
            serial: Device serial number.
            camera_name: Unique name for this camera node.
            camera_namespace: ROS2 namespace (defaults to camera_name if empty).
            enable_color: Enable color stream.
            enable_depth: Enable depth stream.
            enable_infra: Enable infrared streams (infra1 + infra2).
            enable_imu: Enable gyro + accel streams.
            enable_pointcloud: Enable point cloud output.
            enable_align_depth: Enable depth-to-color alignment.
            width: Stream width in pixels.
            height: Stream height in pixels.
            fps: Frames per second.

        Returns:
            Dict with launch status.
        """
        with self._lock:
            if camera_name in self._processes:
                proc = self._processes[camera_name]
                if proc.poll() is None:
                    return {"success": False, "message": f"Camera '{camera_name}' is already running (PID {proc.pid})"}
                else:
                    # Process exited, clean up
                    del self._processes[camera_name]

            ns = camera_namespace if camera_namespace else camera_name
            profile = f"{width},{height},{fps}"

            cmd = self._build_launch_cmd(
                serial=serial,
                camera_name=camera_name,
                camera_namespace=ns,
                enable_color=enable_color,
                enable_depth=enable_depth,
                enable_infra=enable_infra,
                enable_imu=enable_imu,
                enable_pointcloud=enable_pointcloud,
                enable_align_depth=enable_align_depth,
                profile=profile,
            )

            logger.info(f"Launching camera '{camera_name}': {' '.join(cmd)}")

            try:
                env = self._build_ros2_env()
                proc = subprocess.Popen(
                    cmd,
                    env=env,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                    preexec_fn=os.setsid,  # Create new process group for clean kill
                )
                self._processes[camera_name] = proc
                return {
                    "success": True,
                    "message": f"Camera '{camera_name}' launched (PID {proc.pid})",
                    "pid": proc.pid,
                    "camera_name": camera_name,
                    "serial": serial,
                    "namespace": ns,
                }
            except Exception as exc:
                return {"success": False, "message": f"Failed to launch camera: {exc}"}

    @staticmethod
    def _build_launch_cmd(
        serial: str,
        camera_name: str,
        camera_namespace: str,
        enable_color: bool,
        enable_depth: bool,
        enable_infra: bool,
        enable_imu: bool,
        enable_pointcloud: bool,
        enable_align_depth: bool,
        profile: str,
    ) -> List[str]:
        """
        Build the ros2 launch command list.

        Returns:
            List of command arguments.
        """
        b = lambda v: "true" if v else "false"
        cmd = [
            "ros2", "launch", "realsense2_camera", "rs_launch.py",
            f"serial_no:='{serial}'",
            f"camera_name:={camera_name}",
            f"camera_namespace:={camera_namespace}",
            f"enable_color:={b(enable_color)}",
            f"enable_depth:={b(enable_depth)}",
            f"enable_infra1:={b(enable_infra)}",
            f"enable_infra2:={b(enable_infra)}",
            f"enable_gyro:={b(enable_imu)}",
            f"enable_accel:={b(enable_imu)}",
            f"pointcloud.enable:={b(enable_pointcloud)}",
            f"align_depth.enable:={b(enable_align_depth)}",
            f"depth_module.depth_profile:={profile}",
            f"rgb_camera.color_profile:={profile}",
        ]
        return cmd

    @staticmethod
    def _build_ros2_env() -> Dict[str, str]:
        """
        Build environment dict with ROS2 sourced.

        Returns:
            Environment dict.
        """
        env = os.environ.copy()
        # Ensure ROS2 paths are in the environment (they should be if the
        # user sourced setup.bash before starting the server)
        if "AMENT_PREFIX_PATH" not in env:
            # Try to source from the setup script
            try:
                result = subprocess.run(
                    ["bash", "-c", f"source {ROS2_SETUP_BASH} && env"],
                    capture_output=True, text=True, timeout=10,
                )
                if result.returncode == 0:
                    for line in result.stdout.splitlines():
                        if "=" in line:
                            k, _, v = line.partition("=")
                            env[k] = v
            except Exception:
                pass
        return env

    def stop_camera(self, camera_name: str) -> Dict[str, Any]:
        """
        Stop a running camera launch process.

        Args:
            camera_name: Name of the camera to stop.

        Returns:
            Dict with stop status.
        """
        with self._lock:
            return self._stop_process(camera_name)

    def _stop_process(self, camera_name: str) -> Dict[str, Any]:
        """
        Internal: stop process by camera name (caller must hold lock).

        Args:
            camera_name: Camera name key.

        Returns:
            Dict with status.
        """
        if camera_name not in self._processes:
            return {"success": False, "message": f"No running process for camera '{camera_name}'"}

        proc = self._processes[camera_name]
        if proc.poll() is not None:
            del self._processes[camera_name]
            return {"success": True, "message": f"Camera '{camera_name}' already exited (code {proc.returncode})"}

        try:
            # Kill the whole process group
            os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
            try:
                proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
                proc.wait(timeout=3)
        except Exception as exc:
            logger.warning(f"Error stopping camera '{camera_name}': {exc}")

        del self._processes[camera_name]
        return {"success": True, "message": f"Camera '{camera_name}' stopped"}

    def list_active_cameras(self) -> Dict[str, Any]:
        """
        List all camera processes managed by this bridge.

        Returns:
            Dict with list of active cameras.
        """
        with self._lock:
            cameras = []
            dead = []
            for name, proc in self._processes.items():
                if proc.poll() is None:
                    cameras.append({"camera_name": name, "pid": proc.pid, "status": "running"})
                else:
                    cameras.append({"camera_name": name, "pid": proc.pid, "status": "exited", "returncode": proc.returncode})
                    dead.append(name)
            # Clean up dead processes
            for name in dead:
                del self._processes[name]
            return {"success": True, "cameras": cameras}

    def stop_all(self) -> None:
        """Stop all camera processes."""
        with self._lock:
            for name in list(self._processes.keys()):
                self._stop_process(name)

    # ------------------------------------------------------------------ #
    # Topic Operations                                                    #
    # ------------------------------------------------------------------ #

    def list_topics(self, camera_name: Optional[str] = None) -> Dict[str, Any]:
        """
        List ROS2 topics, optionally filtered by camera name.

        Args:
            camera_name: If provided, filter topics containing this camera name.

        Returns:
            Dict with topic list.
        """
        try:
            env = self._build_ros2_env()
            result = subprocess.run(
                ["ros2", "topic", "list", "-t"],
                capture_output=True, text=True, timeout=10, env=env,
            )
            if result.returncode != 0:
                return {"success": False, "message": f"ros2 topic list failed: {result.stderr.strip()}"}

            topics = []
            for line in result.stdout.strip().splitlines():
                line = line.strip()
                if not line:
                    continue
                # Format: /topic/name [type]
                parts = line.split(" ", 1)
                topic_name = parts[0]
                topic_type = parts[1].strip("[]") if len(parts) > 1 else "unknown"

                if camera_name and f"/{camera_name}/" not in topic_name:
                    continue
                topics.append({"topic": topic_name, "type": topic_type})

            return {"success": True, "topics": topics}
        except subprocess.TimeoutExpired:
            return {"success": False, "message": "ros2 topic list timed out"}
        except FileNotFoundError:
            return {"success": False, "message": "ros2 command not found — is ROS2 sourced?"}

    def get_topic_info(self, topic_name: str) -> Dict[str, Any]:
        """
        Get type and publisher info for a topic.

        Args:
            topic_name: Full topic name.

        Returns:
            Dict with topic info.
        """
        try:
            env = self._build_ros2_env()
            result = subprocess.run(
                ["ros2", "topic", "info", topic_name],
                capture_output=True, text=True, timeout=10, env=env,
            )
            if result.returncode != 0:
                return {"success": False, "message": f"Failed: {result.stderr.strip()}"}
            return {"success": True, "topic": topic_name, "info": result.stdout.strip()}
        except subprocess.TimeoutExpired:
            return {"success": False, "message": "Timed out"}
        except FileNotFoundError:
            return {"success": False, "message": "ros2 command not found"}

    def get_topic_hz(self, topic_name: str, window: int = 5) -> Dict[str, Any]:
        """
        Measure topic publish rate.

        Args:
            topic_name: Full topic name.
            window: Number of messages to sample.

        Returns:
            Dict with rate info.
        """
        try:
            env = self._build_ros2_env()
            result = subprocess.run(
                ["ros2", "topic", "hz", topic_name, "-w", str(window)],
                capture_output=True, text=True, timeout=15, env=env,
            )
            output = result.stdout.strip() or result.stderr.strip()
            return {"success": True, "topic": topic_name, "hz_info": output}
        except subprocess.TimeoutExpired:
            return {"success": True, "topic": topic_name, "hz_info": "Measurement timed out (no messages or very low rate)"}
        except FileNotFoundError:
            return {"success": False, "message": "ros2 command not found"}

    # ------------------------------------------------------------------ #
    # Frame Capture                                                       #
    # ------------------------------------------------------------------ #

    def _subscribe_one_msg(self, topic: str, msg_type: Any, timeout_sec: float = 5.0) -> Optional[Any]:
        """
        Subscribe to a topic and capture a single message using spin_once.

        Uses BEST_EFFORT QoS to match realsense2_camera's sensor data QoS.

        Args:
            topic: Topic name.
            msg_type: ROS2 message type class.
            timeout_sec: Max wait time in seconds.

        Returns:
            The captured message, or None on timeout.
        """
        self._ensure_node()
        captured = {"msg": None}

        def _cb(msg: Any) -> None:
            captured["msg"] = msg

        # realsense2_camera publishes with SensorDataQoS (best_effort).
        # A reliable subscriber CANNOT receive from a best_effort publisher.
        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        sub = self._node.create_subscription(msg_type, topic, _cb, sensor_qos)
        deadline = time.monotonic() + timeout_sec
        try:
            while captured["msg"] is None and time.monotonic() < deadline:
                remaining = max(0.05, deadline - time.monotonic())
                _rclpy.spin_once(self._node, timeout_sec=min(remaining, 0.5))
        finally:
            self._node.destroy_subscription(sub)

        return captured["msg"]

    @staticmethod
    def _imgmsg_to_numpy(msg: Any, desired_encoding: str = "passthrough") -> "np.ndarray":
        """
        Convert a ROS2 sensor_msgs/Image to a numpy array.
        Pure-numpy fallback when cv_bridge is unavailable (numpy 2.x ABI mismatch).

        Args:
            msg: sensor_msgs/Image message.
            desired_encoding: Target encoding ('passthrough', 'bgr8', 'mono8', etc.)

        Returns:
            numpy array of the image.
        """
        import numpy as np

        encoding = msg.encoding.lower()
        h, w = msg.height, msg.width

        # Encoding -> (numpy dtype, channels)
        _ENC_MAP = {
            "bgr8": (np.uint8, 3),
            "rgb8": (np.uint8, 3),
            "bgra8": (np.uint8, 4),
            "rgba8": (np.uint8, 4),
            "mono8": (np.uint8, 1),
            "8uc1": (np.uint8, 1),
            "mono16": (np.uint16, 1),
            "16uc1": (np.uint16, 1),
            "32fc1": (np.float32, 1),
        }

        dtype, channels = _ENC_MAP.get(encoding, (np.uint8, 1))
        data = np.frombuffer(bytes(msg.data), dtype=dtype)

        if channels == 1:
            img = data.reshape((h, w))
        else:
            img = data.reshape((h, w, channels))

        # Handle encoding conversion
        target = desired_encoding.lower()
        if target == "passthrough" or target == encoding:
            return img

        # rgb8 -> bgr8
        if encoding == "rgb8" and target == "bgr8":
            return img[:, :, ::-1].copy()
        # bgr8 -> rgb8
        if encoding == "bgr8" and target == "rgb8":
            return img[:, :, ::-1].copy()

        return img

    def _convert_image(self, msg: Any, desired_encoding: str = "passthrough") -> "np.ndarray":
        """Convert ROS Image msg to numpy, using cv_bridge if available, numpy fallback otherwise."""
        if self._cv_bridge is not None:
            return self._cv_bridge.imgmsg_to_cv2(msg, desired_encoding=desired_encoding)
        return self._imgmsg_to_numpy(msg, desired_encoding)

    @staticmethod
    def _topic_prefix(camera_name: str) -> str:
        """Build topic prefix from camera name (/<camera_name>/<camera_name>/)."""
        return f"/{camera_name}/{camera_name}"

    def capture_color_image(self, camera_name: str, save_path: str, timeout_sec: float = 5.0) -> Dict[str, Any]:
        """
        Capture one color frame and save as PNG.

        Args:
            camera_name: Camera name (used to build topic).
            save_path: Absolute path to save the image.
            timeout_sec: Timeout in seconds.

        Returns:
            Dict with capture result.
        """
        import cv2
        import numpy as np

        topic = f"{self._topic_prefix(camera_name)}/color/image_raw"
        msg = self._subscribe_one_msg(topic, _Image, timeout_sec)
        if msg is None:
            return {"success": False, "message": f"Timeout waiting for color image on {topic}"}

        try:
            cv_image = self._convert_image(msg, desired_encoding="bgr8")
            _ensure_output_dir(save_path)
            cv2.imwrite(save_path, cv_image)
            h, w = cv_image.shape[:2]
            return {
                "success": True,
                "path": save_path,
                "width": w,
                "height": h,
                "encoding": msg.encoding,
                "topic": topic,
            }
        except Exception as exc:
            return {"success": False, "message": f"Failed to convert/save color image: {exc}"}

    def capture_depth_image(
        self,
        camera_name: str,
        save_path: str,
        aligned: bool = True,
        colorize: bool = True,
        timeout_sec: float = 5.0,
    ) -> Dict[str, Any]:
        """
        Capture one depth frame and save as PNG.

        Args:
            camera_name: Camera name.
            save_path: Absolute path to save the image.
            aligned: Use aligned_depth_to_color topic if True.
            colorize: Apply colormap for visualization if True.
            timeout_sec: Timeout in seconds.

        Returns:
            Dict with capture result.
        """
        import cv2
        import numpy as np

        prefix = self._topic_prefix(camera_name)
        if aligned:
            topic = f"{prefix}/aligned_depth_to_color/image_raw"
        else:
            topic = f"{prefix}/depth/image_rect_raw"

        msg = self._subscribe_one_msg(topic, _Image, timeout_sec)
        if msg is None:
            return {"success": False, "message": f"Timeout waiting for depth image on {topic}"}

        try:
            depth_image = self._convert_image(msg, desired_encoding="passthrough")
            _ensure_output_dir(save_path)

            if colorize:
                # Normalize to 0-255 and apply colormap
                depth_normalized = cv2.normalize(depth_image, None, 0, 255, cv2.NORM_MINMAX)
                depth_colored = cv2.applyColorMap(depth_normalized.astype(np.uint8), cv2.COLORMAP_JET)
                cv2.imwrite(save_path, depth_colored)
            else:
                cv2.imwrite(save_path, depth_image)

            h, w = depth_image.shape[:2]
            return {
                "success": True,
                "path": save_path,
                "width": w,
                "height": h,
                "encoding": msg.encoding,
                "topic": topic,
                "aligned": aligned,
                "colorized": colorize,
            }
        except Exception as exc:
            return {"success": False, "message": f"Failed to convert/save depth image: {exc}"}

    def capture_rgbd(
        self,
        camera_name: str,
        color_path: str,
        depth_path: str,
        aligned: bool = True,
        timeout_sec: float = 5.0,
    ) -> Dict[str, Any]:
        """
        Capture aligned RGBD pair (color + depth).

        Args:
            camera_name: Camera name.
            color_path: Path to save color image.
            depth_path: Path to save depth image.
            aligned: Use aligned depth.
            timeout_sec: Timeout for each capture.

        Returns:
            Dict with both capture results.
        """
        color_result = self.capture_color_image(camera_name, color_path, timeout_sec)
        if not color_result["success"]:
            return color_result

        depth_result = self.capture_depth_image(camera_name, depth_path, aligned=aligned, colorize=True, timeout_sec=timeout_sec)
        if not depth_result["success"]:
            return depth_result

        return {
            "success": True,
            "color": color_result,
            "depth": depth_result,
        }

    def capture_infrared(
        self,
        camera_name: str,
        save_path: str,
        index: int = 1,
        timeout_sec: float = 5.0,
    ) -> Dict[str, Any]:
        """
        Capture one infrared frame and save as PNG.

        Args:
            camera_name: Camera name.
            save_path: Path to save.
            index: Infrared sensor index (1 or 2).
            timeout_sec: Timeout.

        Returns:
            Dict with capture result.
        """
        import cv2

        topic = f"{self._topic_prefix(camera_name)}/infra{index}/image_rect_raw"
        msg = self._subscribe_one_msg(topic, _Image, timeout_sec)
        if msg is None:
            return {"success": False, "message": f"Timeout waiting for infrared image on {topic}"}

        try:
            cv_image = self._convert_image(msg, desired_encoding="passthrough")
            _ensure_output_dir(save_path)
            cv2.imwrite(save_path, cv_image)
            h, w = cv_image.shape[:2]
            return {"success": True, "path": save_path, "width": w, "height": h, "topic": topic}
        except Exception as exc:
            return {"success": False, "message": f"Failed to save infrared image: {exc}"}

    def get_imu_data(self, camera_name: str, timeout_sec: float = 5.0) -> Dict[str, Any]:
        """
        Get a single IMU sample.

        Args:
            camera_name: Camera name.
            timeout_sec: Timeout.

        Returns:
            Dict with IMU orientation, angular velocity, and linear acceleration.
        """
        topic = f"{self._topic_prefix(camera_name)}/imu"
        msg = self._subscribe_one_msg(topic, _Imu, timeout_sec)
        if msg is None:
            return {"success": False, "message": f"Timeout waiting for IMU data on {topic}"}

        return {
            "success": True,
            "topic": topic,
            "orientation": {
                "x": msg.orientation.x,
                "y": msg.orientation.y,
                "z": msg.orientation.z,
                "w": msg.orientation.w,
            },
            "angular_velocity": {
                "x": msg.angular_velocity.x,
                "y": msg.angular_velocity.y,
                "z": msg.angular_velocity.z,
            },
            "linear_acceleration": {
                "x": msg.linear_acceleration.x,
                "y": msg.linear_acceleration.y,
                "z": msg.linear_acceleration.z,
            },
        }

    def capture_pointcloud(
        self,
        camera_name: str,
        save_path: str,
        timeout_sec: float = 5.0,
    ) -> Dict[str, Any]:
        """
        Capture a point cloud and save as PCD file.

        Args:
            camera_name: Camera name.
            save_path: Path to save (recommended .pcd or .ply extension).
            timeout_sec: Timeout.

        Returns:
            Dict with capture result.
        """
        import numpy as np

        topic = f"{self._topic_prefix(camera_name)}/depth/color/points"
        msg = self._subscribe_one_msg(topic, _PointCloud2, timeout_sec)
        if msg is None:
            return {"success": False, "message": f"Timeout waiting for point cloud on {topic}"}

        try:
            # Parse PointCloud2 manually to avoid open3d dependency
            points = self._parse_pointcloud2(msg)
            _ensure_output_dir(save_path)

            if save_path.endswith(".ply"):
                self._save_ply(save_path, points)
            else:
                self._save_pcd(save_path, points)

            return {
                "success": True,
                "path": save_path,
                "num_points": len(points),
                "topic": topic,
            }
        except Exception as exc:
            return {"success": False, "message": f"Failed to save point cloud: {exc}"}

    @staticmethod
    def _parse_pointcloud2(msg: Any) -> "np.ndarray":
        """
        Parse PointCloud2 message to Nx3 numpy array of XYZ coordinates.

        Args:
            msg: sensor_msgs/PointCloud2 message.

        Returns:
            Numpy array of shape (N, 3).
        """
        import numpy as np

        # Find XYZ field offsets
        field_map = {f.name: f for f in msg.fields}
        x_off = field_map["x"].offset
        y_off = field_map["y"].offset
        z_off = field_map["z"].offset
        point_step = msg.point_step
        data = bytes(msg.data)

        points = []
        for i in range(msg.width * msg.height):
            base = i * point_step
            x = struct.unpack_from("f", data, base + x_off)[0]
            y = struct.unpack_from("f", data, base + y_off)[0]
            z = struct.unpack_from("f", data, base + z_off)[0]
            # Filter NaN / inf points
            if not (np.isnan(x) or np.isnan(y) or np.isnan(z) or np.isinf(x) or np.isinf(y) or np.isinf(z)):
                points.append((x, y, z))

        return np.array(points, dtype=np.float32) if points else np.empty((0, 3), dtype=np.float32)

    @staticmethod
    def _save_pcd(path: str, points: "np.ndarray") -> None:
        """Save points as ASCII PCD file."""
        n = len(points)
        with open(path, "w") as f:
            f.write("# .PCD v0.7 - Point Cloud Data file format\n")
            f.write("VERSION 0.7\n")
            f.write("FIELDS x y z\n")
            f.write("SIZE 4 4 4\n")
            f.write("TYPE F F F\n")
            f.write("COUNT 1 1 1\n")
            f.write(f"WIDTH {n}\n")
            f.write("HEIGHT 1\n")
            f.write("VIEWPOINT 0 0 0 1 0 0 0\n")
            f.write(f"POINTS {n}\n")
            f.write("DATA ascii\n")
            for pt in points:
                f.write(f"{pt[0]:.6f} {pt[1]:.6f} {pt[2]:.6f}\n")

    @staticmethod
    def _save_ply(path: str, points: "np.ndarray") -> None:
        """Save points as ASCII PLY file."""
        n = len(points)
        with open(path, "w") as f:
            f.write("ply\n")
            f.write("format ascii 1.0\n")
            f.write(f"element vertex {n}\n")
            f.write("property float x\n")
            f.write("property float y\n")
            f.write("property float z\n")
            f.write("end_header\n")
            for pt in points:
                f.write(f"{pt[0]:.6f} {pt[1]:.6f} {pt[2]:.6f}\n")

    # ------------------------------------------------------------------ #
    # Service Calls                                                       #
    # ------------------------------------------------------------------ #

    def get_device_info(self, camera_name: str) -> Dict[str, Any]:
        """
        Call the DeviceInfo service.

        Args:
            camera_name: Camera name.

        Returns:
            Dict with device information.
        """
        if _DeviceInfo is None:
            return {"success": False, "message": "DeviceInfo service type not available"}

        self._ensure_node()
        service_name = f"/{camera_name}/{camera_name}/device_info"
        client = self._node.create_client(_DeviceInfo, service_name)

        try:
            if not client.wait_for_service(timeout_sec=5.0):
                return {"success": False, "message": f"Service {service_name} not available"}

            req = _DeviceInfo.Request()
            future = client.call_async(req)
            _rclpy.spin_until_future_complete(self._node, future, timeout_sec=5.0)

            if future.result() is None:
                return {"success": False, "message": "Service call returned None"}

            result = future.result()
            return {
                "success": True,
                "device_name": result.device_name,
                "serial_number": result.serial_number,
                "firmware_version": result.firmware_version,
                "usb_type_descriptor": result.usb_type_descriptor,
                "firmware_update_id": result.firmware_update_id,
                "sensors": result.sensors,
                "physical_port": result.physical_port,
            }
        except Exception as exc:
            return {"success": False, "message": f"Service call failed: {exc}"}
        finally:
            self._node.destroy_client(client)

    # ------------------------------------------------------------------ #
    # Camera Info (Intrinsics)                                            #
    # ------------------------------------------------------------------ #

    def get_camera_info(self, camera_name: str, stream: str = "color") -> Dict[str, Any]:
        """
        Get CameraInfo (intrinsics) for a stream.

        Args:
            camera_name: Camera name.
            stream: Stream name (color, depth, infra1, infra2, aligned_depth_to_color).

        Returns:
            Dict with camera intrinsics.
        """
        stream_topic_map = {
            "color": "color/camera_info",
            "depth": "depth/camera_info",
            "infra1": "infra1/camera_info",
            "infra2": "infra2/camera_info",
            "aligned_depth_to_color": "aligned_depth_to_color/camera_info",
        }
        suffix = stream_topic_map.get(stream, "color/camera_info")
        topic = f"{self._topic_prefix(camera_name)}/{suffix}"

        msg = self._subscribe_one_msg(topic, _CameraInfo, timeout_sec=5.0)
        if msg is None:
            return {"success": False, "message": f"Timeout waiting for CameraInfo on {topic}"}

        return {
            "success": True,
            "topic": topic,
            "width": msg.width,
            "height": msg.height,
            "distortion_model": msg.distortion_model,
            "D": list(msg.d),
            "K": list(msg.k),
            "R": list(msg.r),
            "P": list(msg.p),
        }

    # ------------------------------------------------------------------ #
    # TF                                                                  #
    # ------------------------------------------------------------------ #

    def get_tf(self, camera_name: str, from_frame: str = "depth", to_frame: str = "color") -> Dict[str, Any]:
        """
        Query TF transform between two frames.

        Args:
            camera_name: Camera name (used to build frame IDs).
            from_frame: Source frame suffix (e.g., 'depth', 'color').
            to_frame: Target frame suffix.

        Returns:
            Dict with transform data.
        """
        source = f"{camera_name}_{from_frame}_frame"
        target = f"{camera_name}_{to_frame}_frame"

        try:
            env = self._build_ros2_env()
            result = subprocess.run(
                ["ros2", "run", "tf2_ros", "tf2_echo", source, target],
                capture_output=True, text=True, timeout=10, env=env,
            )
            output = result.stdout.strip() or result.stderr.strip()
            return {"success": True, "source_frame": source, "target_frame": target, "tf_output": output}
        except subprocess.TimeoutExpired:
            return {"success": True, "source_frame": source, "target_frame": target, "tf_output": "Timeout (frames may not be published yet)"}
        except FileNotFoundError:
            return {"success": False, "message": "ros2/tf2_ros not found"}

    def get_extrinsics(self, camera_name: str) -> Dict[str, Any]:
        """
        Get depth-to-color extrinsics by subscribing to the extrinsics topic.

        Args:
            camera_name: Camera name.

        Returns:
            Dict with extrinsics data.
        """
        topic = f"{self._topic_prefix(camera_name)}/extrinsics/depth_to_color"

        try:
            # Try to import the Extrinsics msg type
            from realsense2_camera_msgs.msg import Extrinsics as ExtrinsicsMsg
            msg = self._subscribe_one_msg(topic, ExtrinsicsMsg, timeout_sec=5.0)
            if msg is None:
                return {"success": False, "message": f"Timeout waiting for extrinsics on {topic}"}
            return {
                "success": True,
                "topic": topic,
                "rotation": list(msg.rotation),
                "translation": list(msg.translation),
            }
        except ImportError:
            return {"success": False, "message": "Extrinsics message type not available"}
        except Exception as exc:
            return {"success": False, "message": f"Failed to get extrinsics: {exc}"}

    # ------------------------------------------------------------------ #
    # Parameter Operations                                                #
    # ------------------------------------------------------------------ #

    def get_camera_parameters(self, camera_name: str) -> Dict[str, Any]:
        """
        Get all parameters for a camera node via ros2 param dump.

        Args:
            camera_name: Camera name.

        Returns:
            Dict with parameter data.
        """
        node_name = f"/{camera_name}/{camera_name}"
        try:
            env = self._build_ros2_env()
            result = subprocess.run(
                ["ros2", "param", "dump", node_name],
                capture_output=True, text=True, timeout=10, env=env,
            )
            if result.returncode != 0:
                return {"success": False, "message": f"Failed to dump params: {result.stderr.strip()}"}
            return {"success": True, "node": node_name, "parameters": result.stdout.strip()}
        except subprocess.TimeoutExpired:
            return {"success": False, "message": "Timed out"}
        except FileNotFoundError:
            return {"success": False, "message": "ros2 command not found"}

    def set_camera_parameter(self, camera_name: str, param_name: str, value: str) -> Dict[str, Any]:
        """
        Set a parameter on a camera node.

        Args:
            camera_name: Camera name.
            param_name: Parameter name.
            value: Parameter value as string.

        Returns:
            Dict with result.
        """
        node_name = f"/{camera_name}/{camera_name}"
        try:
            env = self._build_ros2_env()
            result = subprocess.run(
                ["ros2", "param", "set", node_name, param_name, value],
                capture_output=True, text=True, timeout=10, env=env,
            )
            if result.returncode != 0:
                return {"success": False, "message": f"Failed: {result.stderr.strip()}"}
            return {"success": True, "node": node_name, "param": param_name, "value": value, "output": result.stdout.strip()}
        except subprocess.TimeoutExpired:
            return {"success": False, "message": "Timed out"}
        except FileNotFoundError:
            return {"success": False, "message": "ros2 command not found"}

    def enable_filter(self, camera_name: str, filter_name: str, enable: bool = True) -> Dict[str, Any]:
        """
        Enable or disable a depth filter.

        Args:
            camera_name: Camera name.
            filter_name: Filter name (spatial, temporal, decimation, hole_filling, etc.).
            enable: True to enable, False to disable.

        Returns:
            Dict with result.
        """
        # Map common short names to full parameter names
        param_name = f"{filter_name}_filter.enable"
        value = "true" if enable else "false"
        return self.set_camera_parameter(camera_name, param_name, value)

    def set_depth_profile(self, camera_name: str, width: int, height: int, fps: int) -> Dict[str, Any]:
        """
        Set depth stream profile (resolution and FPS).

        Args:
            camera_name: Camera name.
            width: Width in pixels.
            height: Height in pixels.
            fps: Frames per second.

        Returns:
            Dict with result.
        """
        profile = f"{width},{height},{fps}"
        return self.set_camera_parameter(camera_name, "depth_module.depth_profile", profile)

    def set_color_profile(self, camera_name: str, width: int, height: int, fps: int) -> Dict[str, Any]:
        """
        Set color stream profile (resolution and FPS).

        Args:
            camera_name: Camera name.
            width: Width in pixels.
            height: Height in pixels.
            fps: Frames per second.

        Returns:
            Dict with result.
        """
        profile = f"{width},{height},{fps}"
        return self.set_camera_parameter(camera_name, "rgb_camera.color_profile", profile)

    # ------------------------------------------------------------------ #
    # ROS2 Diagnostics                                                    #
    # ------------------------------------------------------------------ #

    def check_ros2_status(self) -> Dict[str, Any]:
        """
        Check ROS2 environment status.

        Returns:
            Dict with ROS2 status information.
        """
        result: Dict[str, Any] = {"success": True}

        # Check ROS_DISTRO
        result["ros_distro"] = os.environ.get("ROS_DISTRO", "not set")

        # Check if ros2 CLI is available
        env = self._build_ros2_env()
        try:
            proc = subprocess.run(["ros2", "doctor", "--report"], capture_output=True, text=True, timeout=15, env=env)
            result["ros2_doctor"] = proc.stdout.strip()[:2000] if proc.stdout else proc.stderr.strip()[:2000]
        except subprocess.TimeoutExpired:
            result["ros2_doctor"] = "ros2 doctor timed out"
        except FileNotFoundError:
            result["ros2_doctor"] = "ros2 command not found"
            result["success"] = False

        # Check rclpy import
        result["rclpy_available"] = _try_import_ros2()
        if _ros2_import_error:
            result["rclpy_error"] = _ros2_import_error

        # Check daemon
        try:
            proc = subprocess.run(["ros2", "daemon", "status"], capture_output=True, text=True, timeout=5, env=env)
            result["daemon_status"] = proc.stdout.strip()
        except Exception:
            result["daemon_status"] = "unknown"

        return result

    def list_ros2_nodes(self) -> Dict[str, Any]:
        """
        List all ROS2 nodes.

        Returns:
            Dict with node list.
        """
        try:
            env = self._build_ros2_env()
            proc = subprocess.run(["ros2", "node", "list"], capture_output=True, text=True, timeout=10, env=env)
            if proc.returncode != 0:
                return {"success": False, "message": proc.stderr.strip()}
            nodes = [n.strip() for n in proc.stdout.strip().splitlines() if n.strip()]
            return {"success": True, "nodes": nodes}
        except subprocess.TimeoutExpired:
            return {"success": False, "message": "Timed out"}
        except FileNotFoundError:
            return {"success": False, "message": "ros2 command not found"}
