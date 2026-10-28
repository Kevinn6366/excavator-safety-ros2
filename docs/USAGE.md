# 运行指南

## 环境与启动

从仓库根目录执行：

```bash
source scripts/env.sh
python3 -m excavator_safety.assets.download
./scripts/start.sh --gui
```

- `./scripts/start.sh`：不显示 Gazebo GUI，但仍渲染相机数据。
- `./scripts/start_dashboard.sh`：先启动网页，随后通过网页启动仿真。
- `./scripts/stop.sh`：停止仿真，保留网页。
- `MONITOR_HOST`、`MONITOR_PORT`：Web 监听地址和端口。
- `DISPLAY`、`XAUTHORITY`：继承当前桌面会话；没有 DISPLAY 时默认 `:0`。
- `LIBGL_ALWAYS_SOFTWARE`：默认 `1`，可设置为 `0` 使用已配置的 GPU。

在使用 `:1` 显示号的虚拟机中，先执行 `export DISPLAY=:1`。相机渲染需要有效图形会话，服务器模式不等于完全无图形依赖。

ROS 域为 33，Gazebo partition 为 `excavator_safety`。不要同时启动同一 checkout 的多个仿真。

## 实验复现

先启动仿真，再在另一个终端执行：

```bash
cd excavator-safety-ros2
source /opt/ros/jazzy/setup.bash
source scripts/env.sh
export ROS_DOMAIN_ID=33 GZ_PARTITION=excavator_safety
python3 -m excavator_safety.evaluation.scenario
python3 -m excavator_safety.evaluation.metrics
python3 tests/integration/integration_path.py
```

场景布局与动态测试会移动目标和车辆，运行时不要同时驾驶。场景评价先采样后计算指标，保持同一次运行的数据时间轴。

其他入口：

```bash
python3 -m excavator_safety.evaluation.demo_layout
python3 -m excavator_safety.evaluation.verify_drive
python3 tests/integration/integration_os.py
python3 tests/integration/validate_lifecycle.py
```

## 数据与故障定位

| 位置 | 内容 |
|---|---|
| `data/frame_*.npz` | RGB、深度、内参、点云与时间 |
| `data/events.jsonl` | 检测与控制日志 |
| `data/metrics.json` | 当前场景统计 |
| `history/` | 重新启动前的数据归档 |
| `logs/` | Gazebo、桥接器与安全节点日志 |
| `evidence/` | 动态测试结果 |

网页无相机数据时，先查看 `logs/gazebo.log` 的渲染状态，再检查桥接与安全节点日志。找不到 Python 包时，重新 `source scripts/env.sh`；找不到 ROS 包时，使用系统 Python 并加载 `/opt/ros/jazzy/setup.bash`。
