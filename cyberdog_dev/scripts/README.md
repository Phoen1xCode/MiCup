# scripts/

启动与维护脚本。**实机 / 仿真都通过 `launch.sh` 进入**，不要在终端里手敲 `python3 -m perception.xxx`，否则容易漏拉某个感知节点。

## 文件说明

| 文件 | 作用 |
|---|---|
| `launch.sh` | 主启动脚本。三步：①后台拉起 LiDAR/slope 节点和统一 `python3 -m perception <detector>` 目标检测节点；②拉起 `core.localization.odom_broadcaster`（odom → TF）；③前台跑 `python3 main.py --mode $MODE --stages $STAGES`。脚本退出时 `trap` 自动 `kill $(jobs -p)` 清理所有后台子进程。用法：`bash scripts/launch.sh sim 1` / `bash scripts/launch.sh real 1-6`。 |
| `clean.sh` | 清理 Python 缓存。删除 `__pycache__` 目录、`.pytest_cache` 目录、`*.pyc / *.pyo` 文件。可选参数：路径（默认当前目录）。pytest 报奇怪的 ImportError 时优先跑它。 |

## 迁移说明

`launch.sh` 是 template 中 `start.sh` 的**重写**，不是原样迁移：

| 维度 | `template/start.sh` | `scripts/launch.sh` |
|---|---|---|
| 项目根目录 | 硬编码 `PROJECT_BASE_DIR="/home/mi/MI/static_nav"` | 用 `$(cd "$(dirname "$0")/.." && pwd)` 自动定位 |
| 启动相机 | `ros2 run camera_test camera_server` + `ros2 service call /camera_service ...` | **未包含**，交给上游（Gazebo / 实机系统）自行启动 |
| 启动感知 | `python3 detector/arrow_publisher.py`、`qrcode_publisher.py` 等 ROS Node 脚本 | `python3 -m perception <name> --mode "$MODE"` 模块化调用，支持 sim/real 切换；LiDAR 走 `python3 -m perception.lidar_corridor` |
| 启动 odom | `python3 motion/utils/odom_broadcaster.py` | `python3 -m core.localization.odom_broadcaster --mode "$MODE"` |
| 主入口 | 不启动（在 `start.sh` 之外另跑 `main.py`） | `python3 main.py --mode "$MODE" --stages "$STAGES"`，把主入口也纳入脚本 |
| 关闭信号 | `trap cleanup SIGINT SIGTERM` + `kill $(jobs -p)` | 完全相同 |

`clean.sh` 为新增，template 没有对应物。
