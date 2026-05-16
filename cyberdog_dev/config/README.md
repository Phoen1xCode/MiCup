# config/

配置中心。把所有"环境相关 / 易调参数"集中到 TOML 文件，避免硬编码散落在代码里。支持 `sim` / `real` 双模式切换。

## 文件说明

| 文件 | 作用 |
|---|---|
| `__init__.py` | 包标记。 |
| `loader.py` | TOML 加载工具。`_load_toml(path)` 优先尝试 `tomllib`（Python 3.11+）→ `tomli` → 第三方 `toml` 包；`load_topics(path, mode)` 返回 `sim` 或 `real` 段的 topic 字典；`load_stage_params(path, stage_id)` 返回 `stageN` 段的参数字典。 |
| `topics.toml` | ROS2 topic / LCM namespace 配置。`[sim]` 段用 `/scan`、`/odom_out`、`/rgb_camera/image_raw`；`[real]` 段用 `/mi_desktop_48_b0_2d_5f_ba_36/*`。**所有节点启动时都通过这里查 topic 名**，不在代码里写死。 |
| `hsv.toml` | 各颜色目标的 HSV 阈值。条目如 `[sim.orange_ball]`、`[real.red_pole]`，每个目标可写 `lower/upper` 单段或 `lower1/upper1, lower2/upper2` 双段（红色用）。`perception/_color_detector_node.py` 启动时按 mode 读取。 |
| `stage_params.toml` | 各赛段的速度、距离、超时、阈值。条目如 `[stage1]` 含 `forward_speed`、`visual_lateral_gain`、`max_time` 等；Stage 在 `__init__` 时通过 `load_stage_params()` 读入，写到 `self.p`。 |

## 迁移说明

**本目录全部为新增，template 中没有对应文件。**

但内容来源全部是从 `template/` 里**抽出来**的硬编码：

| 本文件条目 | 在 template 中的原位置 |
|---|---|
| `topics.toml [real]` 的命名空间 `/mi_desktop_48_b0_2d_5f_ba_36` | 散落在 `template/main.py`、`template/motion/utils/odom_broadcaster.py`、`template/ekf_fusion.py`、`template/visual.py` 等 |
| `topics.toml` 的 `camera_topic = "/image"` | `template/detector/*_publisher.py` 里写死 |
| `topics.toml` 的 `imu_topic = "/camera/imu"` | `template/ekf_fusion.py` 里写死 |
| `hsv.toml` 中红色双区间 `LOWER_RED1=[0,40,70] / LOWER_RED2=[170,120,70]` | `template/custom_tasks.py` 中 `run_height_limit_task` 的 HSV 常量 |
| `hsv.toml` 中黄色 `[25,45,5] ~ [72,255,255]` | `template/main.py` 调用 `run_yellow_light_task` 时传入的 `hsv_lower/hsv_upper` |
| `stage_params.toml` 中各 `forward_speed=0.2`、`area_threshold` 等 | `template/main.py` 和 `template/custom_tasks.py` 里散落的实参 |

> 设计意图：让 sim → real 迁移、HSV 调阈值、改速度时**不动 .py 代码**，只改 TOML。这是 template 当年最痛的地方（坐标硬编码满天飞），新架构在这一步上彻底重写。
