# 贡献指南

## 开发环境

纯算法开发可在 Python 虚拟环境中进行：

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -e .
pip install black
python3 -m unittest discover -s tests/unit -v
```

完整仿真使用 Ubuntu、ROS 2 Jazzy 和 Gazebo Harmonic，按 README 安装系统依赖，再加载 `scripts/env.sh`。普通 venv 默认不能导入系统 ROS 包。

## 代码组织

- 保持感知、控制和图像算法独立于 ROS，消息适配放入 `ros/`。
- 使用 `excavator_safety.*` 绝对导入，避免依赖执行目录的顶层脚本导入。
- 资源路径使用 `excavator_safety.paths`；生成数据不提交到 Git。
- 物理量采用 SI 单位，车辆坐标为 X 向前、Y 向左、Z 向上。
- 目标检测与控制不读取场景真值。

## 提交前检查

```bash
python3 -m black --check src tests
python3 -m unittest discover -s tests/unit -v
python3 -m compileall -q src tests
bash -n scripts/*.sh
```

变更感知或控制行为时增加针对性回归测试；变更进程管理或 ROS 接口时运行相关 Gazebo 集成测试，并在 Pull Request 中说明环境与结果。文档修改应同步更新可执行命令和迁移说明。

Issue 请提供复现步骤、环境版本及相关日志片段。Pull Request 描述问题、最终行为和验证结果，保持改动聚焦。
