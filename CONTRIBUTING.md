# Contributing to realsense-ros-mcp

感谢你对 realsense-ros-mcp 的兴趣！我们欢迎各种形式的贡献。

## 如何贡献

### 报告 Bug

如果你发现了 bug，请创建一个 issue，包含：
- 问题描述
- 复现步骤
- 期望行为 vs 实际行为
- 环境信息（操作系统、ROS2 版本、相机型号）
- 相关日志或错误信息

### 提交功能请求

有新功能想法？创建 issue 并描述：
- 功能的用途
- 预期的工作方式
- 可能的实现思路（可选）

### 提交代码

1. Fork 这个仓库
2. 创建你的功能分支 (`git checkout -b feature/amazing-feature`)
3. 提交你的修改 (`git commit -m 'Add some amazing feature'`)
4. 推送到分支 (`git push origin feature/amazing-feature`)
5. 创建一个 Pull Request

### 代码规范

- 遵循 PEP 8 风格指南
- 添加适当的 docstrings
- 更新相关文档
- 确保测试通过

## 开发环境设置

```bash
# 确保 ROS2 已安装并 source
source /opt/ros/humble/setup.bash

# Clone 你的 fork
git clone https://github.com/your-username/realsense-ros-mcp.git
cd realsense-ros-mcp

# 创建虚拟环境
python -m venv venv
source venv/bin/activate  # Windows: venv\Scripts\activate

# 安装依赖
pip install -r requirements.txt
pip install -r requirements-dev.txt  # 开发依赖

# 运行测试
python -m pytest tests/
```

## ROS2 特定注意事项

- 确保代码在 ROS2 Humble 上测试通过
- 使用标准的 ROS2 消息类型
- 遵循 ROS2 命名约定
- 正确管理 ROS2 节点生命周期

## 代码审查流程

- 所有 PR 都需要至少一个审查者的批准
- CI 检查必须通过
- 保持 commit 信息清晰有意义

## 行为准则

- 友善和尊重地对待其他贡献者
- 接受建设性的批评
- 关注对社区最有利的事情

## 有问题？

随时创建一个 issue 或加入我们的讨论！

---

## Contributing (English)

Thank you for your interest in realsense-ros-mcp! We welcome contributions of all kinds.

### Reporting Bugs

When reporting bugs, please include:
- Description of the issue
- Steps to reproduce
- Expected vs actual behavior
- Environment details (OS, ROS2 version, camera model)
- Relevant logs or error messages

### ROS2 Specific Notes

- Ensure code works with ROS2 Humble
- Use standard ROS2 message types
- Follow ROS2 naming conventions
- Properly manage ROS2 node lifecycle

Questions? Feel free to open an issue!