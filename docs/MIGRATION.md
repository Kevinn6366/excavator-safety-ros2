# 从根目录脚本迁移

本次重构保留算法行为、Web 接口、数据格式和仿真参数，改为 `src/excavator_safety` 包布局。

先执行 `source scripts/env.sh`，再使用下表入口：

| 旧入口 | 新入口 |
|---|---|
| `./start.sh --gui` | `./scripts/start.sh --gui` |
| `./start_dashboard.sh` | `./scripts/start_dashboard.sh` |
| `./stop.sh` | `./scripts/stop.sh` |
| `python3 bootstrap_sources.py` | `python3 -m excavator_safety.assets.download` |
| `python3 import_model.py` | `python3 -m excavator_safety.assets.import_model` |
| `python3 build_project.py` | `python3 -m excavator_safety.assets.build_world` |
| `python3 scenario.py` | `python3 -m excavator_safety.evaluation.scenario` |
| `python3 evaluate.py` | `python3 -m excavator_safety.evaluation.metrics` |
| `python3 integration_path.py` | `python3 tests/integration/integration_path.py` |
| `python3 -m unittest discover -v` | `python3 -m unittest discover -s tests/unit -v` |

`core.py` 拆分为 `perception.geometry`、`perception.detection`、`control.controller` 与 `control.risk`；公共配置位于 `config.py`。`web/dist/` 移至 `frontend/`。下载的模型和原有 `data/` 目录无需迁移。

升级前停止旧版仿真；新版清理器识别新的模块启动命令，不负责停止其他目录中的旧进程。
