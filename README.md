# Excavator Safety · ROS 2 + Gazebo

面向工程车辆作业场景的视觉感知、分级预警与自动紧急制动仿真。使用 ZX200 挖掘机模型、四路 RGB-D 相机、里程计和接触传感器，通过 ROS 2 构成感知—决策—执行闭环，并提供可驾驶的 Web 座舱。

![Web 座舱](docs/images/web_multi.png)

## 功能

- 四方向 RGB / 深度影像，HSV 阈值分割、连通域分析、深度反投影和点云定位。
- 基于 RGB-D 的正交鸟瞰环视和周边目标显示。
- 同时评估当前运动与驾驶请求的预测轨迹，支持前进、倒车、转弯 AEB 和反向脱离。
- WASD 驾驶、线性加速、松油门减速、空格制动；速度上限可调至 50 km/h。
- 双栏 Web 视图、仿真启停与重置、最多 80 个随机目标及碰撞告警。
- 24 组可重复场景、标定扰动分析、单元测试及动态驾驶验证脚本。

![系统架构](docs/images/architecture-1.png)

## 环境

已验证环境：Ubuntu 24.04 ARM64、ROS 2 Jazzy、Gazebo Harmonic。运行前安装 ROS 2 Jazzy 与 Gazebo Harmonic，并配置 ROS 软件源。在该环境中安装依赖：

```bash
sudo apt update
sudo apt install ros-jazzy-ros-gz ros-jazzy-cv-bridge \
  ros-jazzy-message-filters ros-jazzy-sensor-msgs-py \
  python3-opencv python3-numpy python3-scipy python3-yaml
```

Web 服务使用 Python 标准库，无 Node.js 构建步骤。`requirements.txt` 用于不依赖 ROS 的算法测试；完整仿真应使用能导入 ROS Python 包的系统 Python。

## 获取与启动

```bash
git clone https://github.com/Kevinn6366/excavator-safety-ros2.git
cd excavator-safety-ros2
python3 bootstrap_sources.py  # 下载固定版本 ZX200 模型，保留许可与文件校验信息
./start.sh --gui             # 启动 Gazebo GUI、ROS 安全节点和 Web 服务
```

打开 <http://localhost:8080>。远程浏览器使用 Linux 主机的 IP 地址。也可执行 `./start.sh` 只启动传感器渲染与服务，或通过 `./start_dashboard.sh` 先启动网页，再从网页启动仿真。

渲染需要有效的图形会话。启动脚本继承 `DISPLAY` 和 `XAUTHORITY`；若终端没有 `DISPLAY`，默认使用 `:0`。虚拟机使用其他显示号时先设置 `export DISPLAY=:1`。软件渲染默认开启，可通过 `LIBGL_ALWAYS_SOFTWARE=0` 使用已配置的 GPU。

`./stop.sh` 停止仿真，网页服务保持运行，以便再次启动。首次运行生成 `models/`、`worlds/` 和 `bridge.yaml`，无需提交机器相关的绝对路径。

## 驾驶与场景

| 操作 | 行为 |
|---|---|
| W / S | 按住前进 / 倒车 |
| A / D | 按住左右转向，可与 W / S 组合 |
| 松开 W / S | 以 0.25 m/s² 线性减速 |
| 空格 / 停车按钮 | 主动制动 |
| 速度滑块 | 设置 0–50 km/h 上限 |
| 场景面板 | 重置、随机配置人员和障碍物 |

平移加速度为 0.4 m/s²，制动减速度为 1 m/s²，转向角速度上限为 0.55 rad/s。方向反转先减速至零，再向相反方向加速。AEB 依据目标距离、车速和轨迹判断；碰撞由实际 Gazebo 接触消息触发提示。

## 算法与数据来源

四路相机输出 320×240 RGB-D 图像，频率 10 Hz，有效深度 0.1–15 m。图像颜色分割与深度过滤得到目标位置，再统一转换到车辆坐标系。控制器利用里程计和驾驶请求预测车体轨迹，设置安全、提醒、告警、危险四级状态，分别执行正常驾驶、限速与 AEB。

鸟瞰图通过深度反投影生成，中央车顶素材由同一 ZX200 模型离线渲染。目标位置来自传感器检测。场景设定坐标仅供布局和独立测试评价使用。

本项目采用颜色几何目标、固定动臂姿态和速度斜坡制动模型，适用于课程仿真。50 km/h 为实验上限；15 m 传感器量程下可能发生制动距离不足，系统保留有限减速度和接触反馈。

## 测试与复现

无需启动 ROS 的单元测试：

```bash
python3 -m unittest discover -v
```

保持仿真运行，另开终端：

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=33 GZ_PARTITION=excavator_safety
python3 scenario.py
python3 evaluate.py
python3 integration_path.py
```

场景测试会移动目标和车辆，运行时不要同时驾驶。`scenario.py` 与 `evaluate.py` 依次生成采样和评价结果；`integration_path.py` 验证前后障碍脱离、转弯制动、滑行减速与碰撞后驶离。

课程实验的 24 组场景全部检出，保存 480 帧，单帧处理耗时 P95 为 33.41 ms，场景出现到正确状态的观测时差 P95 为 0.404 s。该时差包含场景服务和状态采样时间。原始统计摘要见 [docs/metrics.json](docs/metrics.json)，完整评价口径见 [docs/VALIDATION.md](docs/VALIDATION.md)。

## 代码结构

| 文件 | 职责 |
|---|---|
| `core.py` | 感知、坐标转换、净空计算与运动风险控制 |
| `safety_node.py` | ROS 消息同步、决策、速度发布与日志 |
| `birdseye.py` | RGB-D 正交鸟瞰合成 |
| `web_monitor.py`、`web/dist/` | Web 服务、双栏座舱与驾驶交互 |
| `sim_manager.py` | 仿真生命周期和随机布局 |
| `bootstrap_sources.py`、`import_model.py` | 固定版本模型下载与适配 |
| `build_project.py` | 生成 SDF 世界和桥接配置 |
| `test_*.py` | 纯算法单元测试 |
| `scenario.py`、`evaluate.py`、`integration_*.py` | 场景评价与动态测试 |

运行数据写入 `data/`，重启前的数据归档到 `history/`，日志写入 `logs/`。这些目录不纳入版本控制。

## 开源许可

原创代码采用 [Apache-2.0](LICENSE)。ZX200 模型来自 [pwri-opera/OperaSim-AGX](https://github.com/pwri-opera/OperaSim-AGX)，下载固定提交 `1b401d7cc97e11ad6a1c6807562b99c756366e4e`；模型和派生车顶图的归属、修改说明见 [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md)。
