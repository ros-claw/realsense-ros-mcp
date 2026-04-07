"""
SafetyGuard - 增强版参数校验模块 (ROS2 版本)

基于 sdk_to_mcp 安全约束系统设计，提供结构化的安全校验，
针对 ROS2 RealSense 接口优化。

Features:
    - SafetyConstraint dataclass 定义安全约束
    - 分级安全级别 (CRITICAL/HIGH/MEDIUM/LOW)
    - 完整的错误定义和恢复建议
    - ROS2 特定的参数验证（topic、namespace、node 名称）
"""

import os
import re
import logging
from typing import Tuple, Optional, Dict, Any
from dataclasses import dataclass
from enum import Enum

logger = logging.getLogger("realsense_mcp.safety")


# ── 安全级别枚举 ─────────────────────────────────────────────────────────────

class SafetyLevel(Enum):
    """安全分类等级"""
    CRITICAL = "critical"      # 立即物理危险
    HIGH = "high"              # 潜在硬件损坏
    MEDIUM = "medium"          # 操作问题
    LOW = "low"                # 信息警告
    NONE = "none"              # 无安全顾虑


# ── 安全约束定义 ─────────────────────────────────────────────────────────────

@dataclass
class SafetyConstraint:
    """
    参数安全约束定义
    
    Attributes:
        parameter: 参数名称
        min_value: 最小允许值
        max_value: 最大允许值
        units: 物理单位 (pixels, fps, seconds, etc.)
        safety_level: 安全级别
        description: 人类可读的描述
        hardware_limit: 是否是硬件强制限制
        software_guard: 软件是否应该强制执行此限制
    """
    parameter: str
    min_value: Optional[float]
    max_value: Optional[float]
    units: str
    safety_level: SafetyLevel
    description: str
    hardware_limit: bool = True
    software_guard: bool = True

    def validate(self, value: float) -> Tuple[bool, str]:
        """验证值是否符合约束"""
        if self.min_value is not None and value < self.min_value:
            return False, (
                f"{self.parameter}={value}{self.units} 低于最小值 "
                f"{self.min_value}{self.units}"
            )
        if self.max_value is not None and value > self.max_value:
            return False, (
                f"{self.parameter}={value}{self.units} 超过最大值 "
                f"{self.max_value}{self.units}"
            )
        return True, "OK"


# ── 错误定义 ─────────────────────────────────────────────────────────────────

@dataclass
class ErrorDefinition:
    """
    SDK 错误代码定义
    
    Attributes:
        code: 错误代码
        name: 错误标识符
        description: 人类可读的描述
        severity: 错误严重程度 (critical/error/warning/info)
        recoverable: 错误是否可恢复
        suggested_action: 建议的修复步骤
    """
    code: str
    name: str
    description: str
    severity: str
    recoverable: bool
    suggested_action: str


# ── ROS2 RealSense 安全约束表 ─────────────────────────────────────────────────

SAFETY_CONSTRAINTS: Dict[str, SafetyConstraint] = {
    "width": SafetyConstraint(
        parameter="width",
        min_value=320,
        max_value=1920,
        units="pixels",
        safety_level=SafetyLevel.HIGH,
        description="图像宽度必须在相机传感器支持范围内",
        hardware_limit=True,
        software_guard=True,
    ),
    "height": SafetyConstraint(
        parameter="height",
        min_value=240,
        max_value=1080,
        units="pixels",
        safety_level=SafetyLevel.HIGH,
        description="图像高度必须在相机传感器支持范围内",
        hardware_limit=True,
        software_guard=True,
    ),
    "fps": SafetyConstraint(
        parameter="fps",
        min_value=1,
        max_value=90,
        units="fps",
        safety_level=SafetyLevel.MEDIUM,
        description="帧率必须在相机支持范围内",
        hardware_limit=True,
        software_guard=True,
    ),
    "timeout": SafetyConstraint(
        parameter="timeout",
        min_value=0.1,
        max_value=30.0,
        units="seconds",
        safety_level=SafetyLevel.LOW,
        description="超时时间，防止无限等待",
        hardware_limit=False,
        software_guard=True,
    ),
    "downsample": SafetyConstraint(
        parameter="downsample",
        min_value=1,
        max_value=100,
        units="factor",
        safety_level=SafetyLevel.LOW,
        description="降采样步长",
        hardware_limit=False,
        software_guard=True,
    ),
}


# ── ROS2 RealSense 错误定义表 ─────────────────────────────────────────────────

ERROR_DEFINITIONS: Dict[str, ErrorDefinition] = {
    "NODE_NOT_FOUND": ErrorDefinition(
        code="RSR001",
        name="NODE_NOT_FOUND",
        description="指定的 ROS2 节点未找到",
        severity="error",
        recoverable=True,
        suggested_action="检查节点名称和命名空间是否正确，确认相机节点已启动",
    ),
    "TOPIC_NOT_PUBLISHING": ErrorDefinition(
        code="RSR002",
        name="TOPIC_NOT_PUBLISHING",
        description="Topic 没有数据发布",
        severity="warning",
        recoverable=True,
        suggested_action="检查相机节点是否正常运行，确认 topic 名称正确",
    ),
    "SERVICE_NOT_AVAILABLE": ErrorDefinition(
        code="RSR003",
        name="SERVICE_NOT_AVAILABLE",
        description="请求的服务不可用",
        severity="error",
        recoverable=True,
        suggested_action="确认相机节点已启动，服务名称正确",
    ),
    "PARAM_SET_FAILED": ErrorDefinition(
        code="RSR004",
        name="PARAM_SET_FAILED",
        description="参数设置失败",
        severity="error",
        recoverable=True,
        suggested_action="检查参数名称和值是否在有效范围内",
    ),
    "ROS2_NOT_SOURCED": ErrorDefinition(
        code="RSR005",
        name="ROS2_NOT_SOURCED",
        description="ROS2 环境未正确设置",
        severity="critical",
        recoverable=True,
        suggested_action="运行 source /opt/ros/humble/setup.bash 设置 ROS2 环境",
    ),
    "LAUNCH_FAILED": ErrorDefinition(
        code="RSR006",
        name="LAUNCH_FAILED",
        description="相机节点启动失败",
        severity="error",
        recoverable=True,
        suggested_action="检查设备连接，确认 serial 正确，查看日志获取详细信息",
    ),
    "FRAME_CAPTURE_TIMEOUT": ErrorDefinition(
        code="RSR007",
        name="FRAME_CAPTURE_TIMEOUT",
        description="帧捕获超时",
        severity="warning",
        recoverable=True,
        suggested_action="增加超时时间，检查相机数据流是否正常",
    ),
    "INVALID_NAMESPACE": ErrorDefinition(
        code="RSR008",
        name="INVALID_NAMESPACE",
        description="无效的 ROS2 命名空间",
        severity="error",
        recoverable=True,
        suggested_action="使用有效的 ROS2 命名空间（字母、数字、下划线）",
    ),
}


# ── 常量 ──────────────────────────────────────────────────────────────────────

# 禁止写入的系统目录前缀
FORBIDDEN_PATH_PREFIXES = (
    "/etc", "/usr", "/bin", "/sbin", "/boot", "/lib",
    "/proc", "/sys", "/dev", "/var/run", "/run",
)

# 允许写入的目录白名单
ALLOWED_PATH_PREFIXES = (
    "/tmp/",
    "/home/",
)

# 有效的滤波器名称
VALID_FILTERS = {"spatial", "temporal", "decimation", "hole_filling", "disparity", "hdr_merge", "rotation"}

# 有效的流名称
VALID_STREAMS = {"color", "depth", "infra1", "infra2", "aligned_depth_to_color"}

# 正则表达式
_CAMERA_NAME_RE = re.compile(r"^[a-zA-Z0-9_]+$")
_TOPIC_NAME_RE = re.compile(r"^(/[a-zA-Z0-9_]+)+$")


# ── 异常类 ───────────────────────────────────────────────────────────────────

class SafetyError(Exception):
    """安全校验失败异常"""
    pass


# ── SafetyGuard 类 ───────────────────────────────────────────────────────────

class SafetyGuard:
    """
    参数安全校验器 - ROS2 RealSense 增强版
    
    基于 SafetyConstraint 定义，提供结构化的参数验证。
    包含 ROS2 特定的验证（topic、namespace、node 名称）。
    """

    @staticmethod
    def validate_resolution(width: int, height: int) -> Tuple[bool, str]:
        """验证图像分辨率"""
        if not isinstance(width, int) or not isinstance(height, int):
            return False, f"分辨率必须为整数, 实际: width={type(width).__name__}, height={type(height).__name__}"
        if width < 1 or height < 1:
            return False, f"分辨率必须为正数, 实际: {width}x{height}"
        
        valid_w, msg_w = SAFETY_CONSTRAINTS["width"].validate(width)
        if not valid_w:
            return False, msg_w
        valid_h, msg_h = SAFETY_CONSTRAINTS["height"].validate(height)
        if not valid_h:
            return False, msg_h
        return True, "OK"

    @staticmethod
    def validate_fps(fps: int) -> Tuple[bool, str]:
        """验证帧率"""
        if not isinstance(fps, int):
            return False, f"fps 必须为整数, 实际: {type(fps).__name__}"
        return SAFETY_CONSTRAINTS["fps"].validate(fps)

    @staticmethod
    def validate_timeout(timeout: float) -> Tuple[bool, str]:
        """验证超时时间"""
        if not isinstance(timeout, (int, float)):
            return False, f"timeout 必须为数值, 实际: {type(timeout).__name__}"
        return SAFETY_CONSTRAINTS["timeout"].validate(float(timeout))

    @staticmethod
    def validate_camera_name(name: str) -> Tuple[bool, str]:
        """验证相机名称（用于 ROS2 命名空间）"""
        if not name or not isinstance(name, str):
            return False, "相机名称不能为空"
        if not _CAMERA_NAME_RE.match(name):
            return False, f"相机名称只能包含字母、数字和下划线: {name}"
        return True, "OK"

    @staticmethod
    def validate_namespace(namespace: str) -> Tuple[bool, str]:
        """验证 ROS2 命名空间"""
        if not namespace or not isinstance(namespace, str):
            return False, "命名空间不能为空"
        if not namespace.startswith("/"):
            return False, f"命名空间必须以 / 开头: {namespace}"
        if not _TOPIC_NAME_RE.match(namespace):
            return False, f"命名空间格式无效: {namespace}"
        return True, "OK"

    @staticmethod
    def validate_topic_name(topic: str) -> Tuple[bool, str]:
        """验证 ROS2 Topic 名称"""
        if not topic or not isinstance(topic, str):
            return False, "Topic 名称不能为空"
        if not _TOPIC_NAME_RE.match(topic):
            return False, f"Topic 名称格式无效: {topic}"
        return True, "OK"

    @staticmethod
    def validate_file_path(path: str) -> Tuple[bool, str]:
        """验证文件路径是否安全"""
        if not path or not isinstance(path, str):
            return False, "文件路径不能为空"

        abs_path = os.path.realpath(os.path.abspath(path))

        for prefix in FORBIDDEN_PATH_PREFIXES:
            if abs_path.startswith(prefix):
                return False, f"禁止写入系统目录: {prefix}"

        parent = os.path.dirname(abs_path)
        if parent and not os.path.exists(parent):
            try:
                os.makedirs(parent, exist_ok=True)
            except OSError as e:
                return False, f"无法创建目录 {parent}: {e}"

        return True, "OK"

    @staticmethod
    def validate_filter_name(filter_name: str) -> Tuple[bool, str]:
        """验证滤波器名称"""
        if filter_name not in VALID_FILTERS:
            return False, f"未知滤波器: {filter_name}，有效选项: {VALID_FILTERS}"
        return True, "OK"

    @staticmethod
    def validate_stream_name(stream: str) -> Tuple[bool, str]:
        """验证流名称"""
        if stream not in VALID_STREAMS:
            return False, f"未知流类型: {stream}，有效选项: {VALID_STREAMS}"
        return True, "OK"

    @staticmethod
    def validate_downsample(downsample: int) -> Tuple[bool, str]:
        """验证降采样倍率"""
        if not isinstance(downsample, int):
            return False, f"downsample 必须为整数, 实际: {type(downsample).__name__}"
        return SAFETY_CONSTRAINTS["downsample"].validate(downsample)

    @staticmethod
    def check(valid: bool, message: str) -> None:
        """如果校验失败，抛出 SafetyError"""
        if not valid:
            logger.warning(f"SafetyGuard 拒绝: {message}")
            raise SafetyError(message)

    @staticmethod
    def get_constraint(parameter: str) -> Optional[SafetyConstraint]:
        """获取参数的安全约束定义"""
        return SAFETY_CONSTRAINTS.get(parameter)

    @staticmethod
    def get_error_definition(error_name: str) -> Optional[ErrorDefinition]:
        """获取错误定义"""
        return ERROR_DEFINITIONS.get(error_name)

    @staticmethod
    def list_constraints() -> Dict[str, SafetyConstraint]:
        """列出所有安全约束"""
        return SAFETY_CONSTRAINTS.copy()

    @staticmethod
    def list_errors() -> Dict[str, ErrorDefinition]:
        """列出所有错误定义"""
        return ERROR_DEFINITIONS.copy()