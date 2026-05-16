# perception/

感知层：将传感器原始数据（相机图像、LiDAR 扫描）转换为结构化的检测结果，通过 ROS2 topic 发布 JSON 消息供 Stage 层消费。

## 架构

```text
相机 /image ──┬── color_detector.py ── /perception/orange_ball (JSON)
              │                     ── /perception/football (JSON)
              │                     ── /perception/red_pole (JSON)
              │                     ── /perception/block_obstacle (JSON)
              │                     ── /perception/coke (JSON)
              │                     ── /perception/lane_edge (JSON)
              │                     ── /perception/dashed_line (JSON)
              └── (配置来自 detectors.py 注册表)

LiDAR /scan  ─┬── lidar_corridor.py ── /perception/lidar_corridor (Vector3)
              └── slope.py          ── /perception/slope (JSON)

全部 topic ────── hub.py (PerceptionHub) ──→ Stage 层通过 ctx.perception 消费
```

## 文件说明

| 文件 | 类型 | 说明 |
|---|---|---|
| `__init__.py` | 包标记 | 空文件 |
| `__main__.py` | 入口 | `python -m perception` 的 CLI 入口，委托给 `detectors.main()` |
| `detectors.py` | 注册表 | 检测器配置注册表 + CLI 入口。定义所有 HSV 检测器的 topic/key/参数，提供 `run()` 和 `list_detectors()` 接口 |
| `hub.py` | 核心 | **PerceptionHub** — 感知聚合中心。订阅全部 9 个 `/perception/*` topic，将 JSON 解析为类型化 dataclass（`ObjDet`、`BallDet`、`PoleDet`、`CorridorState`、`LaneEdges`、`DashedLineDet`、`SlopeState`）。线程安全，`NO_RETURN = 99.9` 表示无数据 |
| `vision_utils.py` | 工具 | 纯视觉函数库，无 ROS 依赖。提供：HSV 范围判断、bbox 提取、方位角估计（bearing）、针孔模型距离估计、彩色目标检测（`detect_colored_objects_hsv`）、车道线检测（`detect_lane_edges_hsv`）、虚线检测（`detect_dashed_line_hsv`） |
| `color_detector.py` | 运行时 | 共享 ROS2 节点运行时工厂。`run_object_detector()` 订阅相机 topic、做 HSV 分割、发布 JSON 检测结果；`run_scalar_detector()` 用于车道线和虚线的标量检测。所有颜色检测节点共用此运行时 |
| `lidar_corridor.py` | 节点 | **LidarCorridorNode** — 订阅 LaserScan，将扫描分为左/前/右三个扇区（45-85°、-12~12°、-85~-45°），计算各扇区中位数距离，发布为 `geometry_msgs/Vector3` |
| `slope.py` | 节点 | **SlopeDetectorNode** — 订阅 LaserScan，使用 RANSAC 线段拟合检测前方坡道/斜面。迭代筛选最接近目标长度的线段，经滑动窗口平滑后发布 JSON（detected/angle_deg/distance_m/midpoint_m/length_m） |

## 检测器注册表

所有 HSV 相机检测器在 `detectors.py` 的 `_REGISTRY` 中统一配置：

| 名称 | 类型 | Topic | HSV Key | 真实宽度 |
|------|------|-------|---------|---------|
| `orange_ball` | obj | `/perception/orange_ball` | `orange_ball` | 0.2m |
| `football` | obj | `/perception/football` | `football_dark` | 0.2m |
| `coke` | obj | `/perception/coke` | `coke` | 0.12m |
| `red_pole` | obj | `/perception/red_pole` | `red_pole` | 0.1m |
| `block_obstacle` | obj | `/perception/block_obstacle` | `gray_block` | 0.2m |
| `lane_edge` | scalar | `/perception/lane_edge` | — | — |
| `dashed_line` | scalar | `/perception/dashed_line` | — | — |

## 用法

### 列出所有检测器

```bash
python -m perception list
```

### 启动单个检测器节点

```bash
python -m perception red_pole
python -m perception orange_ball
python -m perception lane_edge
```

### 同时启动多个检测器

```bash
# 在 launch 文件或终端中分别启动
python -m perception red_pole &
python -m perception block_obstacle &
python -m perception coke &
python -m perception lane_edge &
python -m perception dashed_line &
```

### 程序调用

```python
from perception.detectors import run, list_detectors

# 列出所有检测器
names = list_detectors()  # ["orange_ball", "football", ...]

# 启动检测器（阻塞，会 spin ROS2 节点）
run("red_pole")
```

### PerceptionHub（在 main.py 中使用）

```python
from perception.hub import PerceptionHub

hub = PerceptionHub()
hub.attach_node(ros2_node)  # 订阅所有 /perception/* topic

# 读取最新检测结果
balls = hub.latest_orange_balls()
corridor = hub.latest_lidar_corridor()
edges = hub.latest_lane_edges()
```

## 数据流

1. 各检测节点独立运行，订阅相机或 LiDAR topic
2. 检测结果以 JSON 字符串发布到 `/perception/*` topic
3. `PerceptionHub` 在 Stage 初始化时通过 `attach_node(ros_node)` 订阅所有 topic
4. Stage 层通过 `ctx.perception.latest_orange_balls()`、`ctx.perception.latest_lidar_corridor()` 等方法获取最新检测结果

## 迁移说明

| 当前文件 | 模板来源 | 变更 |
|---|---|---|
| `vision_utils.py` | **全新** | 模板的视觉检测逻辑内嵌在 `custom_tasks.py`（黄灯/限高杆检测）和 `detector/arrow_publisher.py`（箭头检测）中，直接操作 ROS Image。本项目将其抽取为纯函数，增加 bearing/距离估计和车道线/虚线检测 |
| `color_detector.py` | **全新** | 模板中 `detector/` 下每个 publisher 独立实现 ROS 节点订阅/发布。本项目抽取为共享运行时，检测节点只需一行调用 |
| `detectors.py` | **全新** | 将原来 7 个独立 wrapper 文件的配置合并为统一注册表 |
| `hub.py` | **全新** | 模板中 `main.py` 直接调用各个 subscriber reader（`ArrowDirectionReader`、`QRCodeReader` 等）。本项目统一为 `PerceptionHub` 聚合层 |
| `lidar_corridor.py` | `template/lidar_detect.py` | 逻辑简化：模板用 RANSAC 做线段拟合和最优线段选择（386 行），本项目改为三扇区中位数距离（~60 行），更适合走廊居中场景 |
| `slope.py` | `template/lidar_adjust.py` | 保留 RANSAC 线段拟合 + 迭代筛选 + 滑动窗口平滑核心算法。移除 OpenCV 可视化（~100 行），移除三个独立 topic 改为单 JSON topic，参数从 `stage_params.toml` 读取 |

## 未迁移的模板感知模块

| 模板文件 | 原功能 | 状态 |
|---|---|---|
| `detector/arrow_publisher.py` | 绿色箭头方向检测 | 本赛事无箭头题目，未迁移 |
| `detector/qrcode_publisher.py` | QR 码扫描（pyzbar） | 本赛事无 QR 码题目，未迁移 |
| `detector/text_publisher.py` | OCR 文字识别（百度云 API） | 本赛事无 OCR 题目，未迁移 |
| `custom_tasks.py` | 黄灯检测 + 限高杆检测 | 黄灯未迁移（本赛事无此题）；限高杆检测逻辑已迁移至 `red_pole` + Stage 4 交互 |
| `ekf_fusion.py` | EKF 里程计 + IMU 融合 | 本项目直接使用 TF2 位姿，暂未迁移 |
| `visual.py` | OpenCV 里程计可视化 | 调试工具，未迁移 |
