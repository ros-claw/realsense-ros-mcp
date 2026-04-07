"""
SafetyGuard - Parameter validation for RealSense ROS2 MCP Server.

Enforces safe limits on resolution, FPS, camera names, file paths,
topic names, and timeouts to prevent misconfiguration or abuse.
"""

import os
import re
import logging
from typing import Tuple

logger = logging.getLogger("realsense_mcp.safety")

# --- Limits ---
MAX_WIDTH = 1920
MAX_HEIGHT = 1080
MIN_FPS = 1
MAX_FPS = 90
MAX_TIMEOUT_SEC = 30.0
MIN_TIMEOUT_SEC = 0.1

# Allowed characters for camera_name: alphanumeric + underscore
_CAMERA_NAME_RE = re.compile(r"^[a-zA-Z0-9_]+$")

# ROS2 topic name pattern: starts with /, segments of alphanumeric/underscore
_TOPIC_NAME_RE = re.compile(r"^(/[a-zA-Z0-9_]+)+$")

# Valid filter names
VALID_FILTERS = {"spatial", "temporal", "decimation", "hole_filling", "disparity", "hdr_merge", "rotation"}

# Valid stream names for camera_info
VALID_STREAMS = {"color", "depth", "infra1", "infra2", "aligned_depth_to_color"}


def validate_resolution(width: int, height: int) -> Tuple[bool, str]:
    """
    Validate image resolution.

    Args:
        width: Image width in pixels.
        height: Image height in pixels.

    Returns:
        Tuple of (is_valid, error_message). error_message is empty if valid.
    """
    if not isinstance(width, int) or not isinstance(height, int):
        return False, f"Resolution must be integers, got width={type(width).__name__}, height={type(height).__name__}"
    if width < 1 or height < 1:
        return False, f"Resolution must be positive, got {width}x{height}"
    if width > MAX_WIDTH or height > MAX_HEIGHT:
        return False, f"Resolution {width}x{height} exceeds maximum {MAX_WIDTH}x{MAX_HEIGHT}"
    return True, ""


def validate_fps(fps: int) -> Tuple[bool, str]:
    """
    Validate frames per second.

    Args:
        fps: Frames per second value.

    Returns:
        Tuple of (is_valid, error_message).
    """
    if not isinstance(fps, int):
        return False, f"FPS must be an integer, got {type(fps).__name__}"
    if fps < MIN_FPS or fps > MAX_FPS:
        return False, f"FPS {fps} out of range [{MIN_FPS}, {MAX_FPS}]"
    return True, ""


def validate_camera_name(name: str) -> Tuple[bool, str]:
    """
    Validate camera name (alphanumeric and underscores only).

    Args:
        name: Camera name string.

    Returns:
        Tuple of (is_valid, error_message).
    """
    if not isinstance(name, str) or not name:
        return False, "Camera name must be a non-empty string"
    if len(name) > 64:
        return False, f"Camera name too long ({len(name)} chars, max 64)"
    if not _CAMERA_NAME_RE.match(name):
        return False, f"Camera name '{name}' contains invalid characters (only alphanumeric and underscore allowed)"
    return True, ""


def validate_serial(serial: str) -> Tuple[bool, str]:
    """
    Validate device serial number.

    Args:
        serial: Serial number string.

    Returns:
        Tuple of (is_valid, error_message).
    """
    if not isinstance(serial, str) or not serial:
        return False, "Serial number must be a non-empty string"
    if not re.match(r"^[a-zA-Z0-9_-]+$", serial):
        return False, f"Serial number '{serial}' contains invalid characters"
    return True, ""


def validate_file_path(path: str) -> Tuple[bool, str]:
    """
    Validate file path for safety (no path traversal, must be absolute or under /tmp).

    Args:
        path: File path string.

    Returns:
        Tuple of (is_valid, error_message).
    """
    if not isinstance(path, str) or not path:
        return False, "File path must be a non-empty string"

    # Normalize the path
    norm_path = os.path.normpath(path)

    # Block path traversal
    if ".." in norm_path.split(os.sep):
        return False, f"Path traversal detected in '{path}'"

    # Must be an absolute path
    if not os.path.isabs(norm_path):
        return False, f"Path must be absolute, got '{path}'"

    # Restrict to safe directories
    safe_prefixes = ("/tmp/", "/home/")
    if not any(norm_path.startswith(p) for p in safe_prefixes):
        return False, f"Path '{norm_path}' is outside allowed directories ({', '.join(safe_prefixes)})"

    return True, ""


def validate_topic_name(topic: str) -> Tuple[bool, str]:
    """
    Validate ROS2 topic name format.

    Args:
        topic: Topic name string (e.g., '/camera/color/image_raw').

    Returns:
        Tuple of (is_valid, error_message).
    """
    if not isinstance(topic, str) or not topic:
        return False, "Topic name must be a non-empty string"
    if not _TOPIC_NAME_RE.match(topic):
        return False, f"Invalid topic name format: '{topic}'"
    return True, ""


def validate_timeout(timeout_sec: float) -> Tuple[bool, str]:
    """
    Validate timeout value.

    Args:
        timeout_sec: Timeout in seconds.

    Returns:
        Tuple of (is_valid, error_message).
    """
    if not isinstance(timeout_sec, (int, float)):
        return False, f"Timeout must be a number, got {type(timeout_sec).__name__}"
    if timeout_sec < MIN_TIMEOUT_SEC or timeout_sec > MAX_TIMEOUT_SEC:
        return False, f"Timeout {timeout_sec}s out of range [{MIN_TIMEOUT_SEC}, {MAX_TIMEOUT_SEC}]"
    return True, ""


def validate_filter_name(filter_name: str) -> Tuple[bool, str]:
    """
    Validate depth filter name.

    Args:
        filter_name: Filter name (spatial, temporal, decimation, hole_filling, etc.).

    Returns:
        Tuple of (is_valid, error_message).
    """
    if not isinstance(filter_name, str) or not filter_name:
        return False, "Filter name must be a non-empty string"
    if filter_name not in VALID_FILTERS:
        return False, f"Unknown filter '{filter_name}', valid: {sorted(VALID_FILTERS)}"
    return True, ""


def validate_stream_name(stream: str) -> Tuple[bool, str]:
    """
    Validate stream name for camera_info lookup.

    Args:
        stream: Stream name (color, depth, infra1, infra2, aligned_depth_to_color).

    Returns:
        Tuple of (is_valid, error_message).
    """
    if not isinstance(stream, str) or not stream:
        return False, "Stream name must be a non-empty string"
    if stream not in VALID_STREAMS:
        return False, f"Unknown stream '{stream}', valid: {sorted(VALID_STREAMS)}"
    return True, ""


def validate_infrared_index(index: int) -> Tuple[bool, str]:
    """
    Validate infrared camera index.

    Args:
        index: Infrared sensor index (1 or 2).

    Returns:
        Tuple of (is_valid, error_message).
    """
    if index not in (1, 2):
        return False, f"Infrared index must be 1 or 2, got {index}"
    return True, ""


def check_or_raise(valid: bool, msg: str) -> None:
    """
    Raise ValueError if validation failed.

    Args:
        valid: Validation result.
        msg: Error message.

    Raises:
        ValueError: If valid is False.
    """
    if not valid:
        logger.warning(f"Validation failed: {msg}")
        raise ValueError(msg)
