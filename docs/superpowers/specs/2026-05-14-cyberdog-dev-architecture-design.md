# cyberdog_dev 系统架构重设计

- **日期**：2026-05-14
- **范围**：重构 `MiCup/cyberdog_dev/` 系统架构、整合 stage1、为 6 个赛段设计详细技术方案、迁移示例代码、给出迁移说明与后续开发指南
- **状态**：设计已确认，待写实施计划

---

## 1. 背景与目标

### 1.1 现状问题

`MiCup/cyberdog_dev/` 当前是一份 AI 生成的投机性框架，存在以下问题：

1. **包名嵌套冗余**：`cyberdog_dev/cyberdog_competition/` 路径里 cyberdog 出现两次。
2. **巨型文件**：`cyberdog_competition/tasks.py` 把 6 个赛段全塞进一个类（269 行）。
3. **职责混杂**：`cyberdog_competition/config.py` 同时管动作原语、视觉阈值、赛段 fallback 时序。
4. **存在两套并行控制抽象**：`cyberdog_competition/control.py`（dataclass 风格，未在仿真验证过）与 `example/stage1/utils/cyberdog_api.py`（31KB 大类，已验证）。
5. **无效文件**：`Robot_Ctrl.py` 是 0 字节空文件；`hello_camera.py` / `hello_walk.py` 散落顶层；`customized_gait/`、`toml/` 是空目录。
6. **从未跑过**：`SOLUTION_ANALYSIS.md` 自己承认"当前代码是可运行框架和保守策略，不应直接宣称满分完成"。

### 1.2 目标

- 以 `example/demo1/` 的模块切分方式为蓝本重建分层架构（demo1 已验证、模块边界清晰）。
- 整合 `example/stage1/` 的雷达走廊算法为赛段 1 的实现。
- 为 6 个赛段各设计详细技术方案并写出可在 Gazebo 调参的初版代码。
- 把示例项目中可复用的底层模块迁移到工作目录。
- 迁移采用纯增量策略，原有文件暂不删除。

### 1.3 关键决策（已与用户确认）

| 决策项 | 选择 | 理由 |
|---|---|---|
| 控制抽象基础 | `example/demo1/` 的模块切分 | 已验证可跑、模块化好、控制/位姿/语音/动作分文件 |
| 重设计范围 | 重构 + 6 赛段详细技术方案 + 初版代码 | 用户要求最完整范围 |
| 目录布局 | 按赛段拆 `stages/` + 按模块拆 `core/` `perception/` | 路径与赛段一一对应，零基础也能快速定位 |
| Stage 内部模式 | 类 + Phase enum 状态机 | 可中断/可恢复、状态明确、好调试，与 stage1.py 现有写法一致 |
| 运行模式 | 仅 `sim` / `real`，不做 dry-run | dry-run 是投机产物，删除后架构更干净 |
| 迁移策略 | 纯增量，原文件暂不删除 | 任何时候能 git diff 看清新增、出问题可回退对照 |
| ekf_fusion.py | 保留，作为可选位姿源 | TF 漂移严重时备用 |
| git 分支 | `feat/framework` | 已有分支，本次重构在此进行 |

---

## 2. 整体分层架构

### 2.1 四层架构

```
┌─────────────────────────────────────────────────┐
│  Application 层    main.py                       │  顶层调度
├─────────────────────────────────────────────────┤
│  Stage 层         stages/stageN_*.py             │  每赛段独立状态机
├─────────────────────────────────────────────────┤
│  Perception 层    perception/{lidar,*_detector}  │  独立感知节点
├─────────────────────────────────────────────────┤
│  Core 层          core/{robot_ctrl,pose,nav,...} │  底层 LCM + TF + 动作原语
└─────────────────────────────────────────────────┘
            ↕
   LCM (control + 心跳) | ROS2 (传感器/感知 topic)
```

### 2.2 各层职责（"一句话"原则）

| 层 | 职责 | 不做的事 |
|---|---|---|
| Core | 把命令变成机器狗动作；把 TF 变成 `(x,y,yaw)` | 任何"决策"、任何视觉处理 |
| Perception | 把图像/雷达变成"检测结果 + topic" | 任何"动作"、任何赛段逻辑 |
| Stage | 用 Phase 状态机编排该赛段的"看→想→做"循环 | 直接发 LCM、直接处理像素 |
| App | 决定跑哪几个 stage、参数解析、错误回收 | 任何赛段细节 |

### 2.3 三个核心设计原则

1. **跨层只通过接口对话**：Stage 只看到 `ctx.dog.stand()` / `ctx.perception.latest_orange_balls()`，不知道下面是 LCM 还是 ROS2。
2. **感知独立进程**：每个 `perception/*.py` 都是能独立运行的 ROS2 节点，通过 topic 发布结果；Stage 通过 `PerceptionHub` 同步取最新结果，不阻塞 tick 循环。检测器节点崩了返回空，Stage 自行降级。
3. **sim / real 通过 config 切换**：代码只有一份，HSV 阈值、topic 名、LCM 命名空间由配置文件区分环境。

### 2.4 数据流

```
       相机帧/scan ──→ perception/*.py 节点 ──→ ROS2 topic
                                                    │
                                              PerceptionHub
                                                    │
       LCM odom ──→ Tf2PoseMonitor ──┐               │
                                    └→ StageContext ←┘
                                          │
                                   Stage.tick() 循环 (~20 Hz)
                                          │
                              ctx.dog.set_velocity(...)
                                          │
                                   LcmDogController
                                          │
                                  LCM robot_control_cmd
```

---

## 3. 接口约定（核心契约）

### 3.1 Stage 基类与状态枚举

```python
# core/stage_base.py
from enum import Enum, auto
from dataclasses import dataclass

class StageStatus(Enum):
    RUNNING = auto()        # 正在执行
    SUCCEEDED = auto()      # 完成结束条件
    FAILED = auto()         # 永久失败，不可恢复
    NEED_HELP = auto()      # 卡死，等待语音/触摸触发恢复（合法人机协作）

@dataclass
class StageResult:
    stage_id: int
    name: str
    status: StageStatus
    notes: list[str]
    elapsed_sec: float

class Stage:
    """所有赛段的基类。子类必须实现 tick()。"""
    stage_id: int = 0
    name: str = ""

    def __init__(self, ctx: "StageContext"):
        self.ctx = ctx
        self.start_time = 0.0
        self.notes: list[str] = []

    def on_enter(self) -> None: ...        # 进入赛段时调用一次（设原点等）
    def tick(self) -> StageStatus: ...     # 状态机的一次推进，必须实现
    def on_exit(self) -> None: ...         # 退出时清理（停车等）
    def max_duration_sec(self) -> float:   # 超时上限
        return 180.0
```

`tick()` 而非 `run()`：让顶层调度能穿插超时检查、外部中断、语音监听，程序不会被某个 stage 永久阻塞。

### 3.2 StageContext（依赖注入容器）

```python
# core/stage_context.py
@dataclass
class StageContext:
    dog: DogController          # 控制接口
    pose: PoseMonitor           # 位姿读取
    perception: PerceptionHub   # 所有检测器的统一入口
    voice: VoiceController      # 语音播报+识别
    logger: Logger
    mode: RunMode               # sim / real
```

Stage 永远只通过 `ctx.xxx` 拿能力，不 import 任何具体实现。

### 3.3 DogController 接口（基于 demo1 的 Robot_Ctrl.py 精简）

```python
class DogController(Protocol):
    # 离散动作（阻塞，等完成）
    def stand(self, *, hold: float = 2.0) -> bool: ...
    def lie_down(self, *, hold: float = 1.0) -> bool: ...
    def execute_discrete(self, mode: int, gait_id: int, **kwargs) -> bool: ...

    # 连续速度控制（非阻塞，自动心跳）
    def set_velocity(self, vx: float, vy: float, wz: float,
                     *, body_height: float = 0.0,
                     step_height: tuple = (0.06, 0.06)) -> None: ...
    def stop(self) -> None: ...

    # 自定义步态（上传 toml）
    def load_custom_gait(self, gait_dir: Path) -> bool: ...
    def execute_custom_gait(self) -> bool: ...
```

### 3.4 PoseMonitor 接口

```python
class PoseMonitor(Protocol):
    def get_xy_yaw(self) -> tuple[float, float, float]: ...      # 米, 米, 弧度
    def set_origin_here(self) -> None: ...                        # 每赛段开始时清零
    def distance_from_origin_along(self, axis: str) -> float: ...  # "X"/"Y"
```

sim 与 real 共享同一份 TF 实现（demo1 的 RobotPoseMonitor）。TF 漂移严重时可启动 `core/ekf_fusion.py` 融合 odom + IMU 发布 `/odom_fused`，PoseMonitor 改订阅它，接口不变。

### 3.5 PerceptionHub —— Stage 获取检测结果的唯一入口

```python
# perception/hub.py
class PerceptionHub:
    """聚合所有检测器，提供同步的"取最新结果"接口。"""

    def latest_orange_balls(self, *, max_age_sec: float = 0.5) -> list[BallDet]: ...
    def latest_red_poles(self) -> list[PoleDet]: ...
    def latest_footballs(self) -> list[BallDet]: ...
    def latest_coke_bottles(self) -> list[ObjDet]: ...
    def latest_block_obstacles(self) -> list[ObjDet]: ...
    def latest_arrows(self) -> Optional[ArrowDet]: ...
    def latest_lane_edges(self) -> LaneEdges: ...
    def latest_dashed_line(self) -> Optional[DashedLineDet]: ...
    def latest_lidar_corridor(self) -> CorridorState: ...  # 左/前/右距离 + 居中误差
```

每个 `latest_*` 内部订阅一个 ROS2 topic（由对应 `perception/*.py` 节点发布），线程安全地返回最近一帧。

核心数据类：

```python
@dataclass
class BallDet:
    bbox: tuple[int, int, int, int]
    center_px: tuple[float, float]
    area_px: float
    bearing_rad: float    # 相对相机中心的方位角，stage 用这个对准
    distance_m: float     # 用直径反推的粗略距离
    confidence: float
```

`PoleDet` / `ObjDet` / `ArrowDet` / `LaneEdges` / `DashedLineDet` / `CorridorState` 在 `perception/hub.py` 中一并定义，字段在实施阶段细化。

### 3.6 RunMode 与工厂

```python
class RunMode(Enum):
    SIM = "sim"      # Gazebo 仿真（初赛）
    REAL = "real"    # 实机（决赛）

def build_context(mode: RunMode) -> StageContext:
    return StageContext(
        dog=LcmDogController(),
        pose=Tf2PoseMonitor(),
        perception=Ros2PerceptionHub([...]),
        voice=Ros2VoiceController(namespace=NS_BY_MODE[mode]),
        mode=mode,
    )
```

sim 与 real 的差异通过配置处理，不通过两套实现：
- HSV 阈值（仿真光照 vs 实机光照）→ `config/hsv.toml`
- 相机/雷达 topic 名 → `config/topics.toml`
- LCM 命名空间 → `NS_BY_MODE`

---

## 4. 6 个赛段技术方案

每段格式：任务本质 → Phase 状态机 → 感知依赖 → 关键算法 → 兜底策略 → 可复用代码。

### 4.1 Stage 1 · 石径探路（固定赛道型）

**任务本质**：趴下站起 → 过石板路 → 第一弯道 → 后腿出虚线。

**Phase 状态机**：
```
RECOVERY_STAND → STABILIZE → STRAIGHT_TO_BEND
              → TURNING → STRAIGHT_TO_EXIT → DONE
```

**感知依赖**：`perception.lidar`（雷达三扇区中位距，前/左/右）；可选 `perception.lane_edge`（黄边修正）。

**关键算法**：
- `STRAIGHT_TO_BEND`：根据左右距离差做 PD 居中；前方距离 < 1.05m 且一侧 > 1.15m 时进入 TURNING。
- `TURNING`：选择更开阔的一侧，用复合速度 `(0.3, 0, ±0.52)`，估算累计转角 ≥ 90° 退出。

**兜底策略**：
- 雷达全失效 → 时序模式（直行 8s → 弧线左转 4s → 直行 3s）。
- 黄线检测可用时，给居中环加一项"画面中黄线左右像素差"作为辅助误差。

**可复用代码**：`example/stage1/stage1.py` 几乎可直接迁移（已验证），改成 Stage 子类即可。

### 4.2 Stage 2 · 荒野寻珠（视觉搜索型，今年难点）

**任务本质**：4×4 球阵，每行每列恰好 1 颗橙球（强约束），固定 3 颗浅蓝（R4C4、R4C3、R3C4），撞所有 4 颗橙球后走左上角出口。

**Phase 状态机**：
```
ENTER → SWEEP_SCAN (180° 转身扫一遍记录所有橙球大致方位)
      → APPROACH_NEXT (选最近未撞的目标，视觉伺服靠近)
      → BUMP (距离 < 0.3m 时短促前冲撞击)
      → BACKOFF + CHECK_HIT (后退 + 用"球在画面里晃动"做命中确认)
      → 重复 4 次
      → GO_EXIT → DONE
```

**感知依赖**：`perception.orange_ball`（HSV + 圆度过滤）、`perception.lidar`（避免撞浅蓝球）。

**关键算法**：
- 入场时 `set_origin_here()`，撞击顺序用每行每列约束 + 当前位置贪心（最近邻）确定。
- 视觉伺服：`bearing_rad` > 0.05 rad → 用 wz 调向；`distance_m` > 0.5m → 用 vx 推进；< 0.3m → 触发 BUMP。
- 防误撞：撞击前用激光雷达验证前方 0.3m 内有物体；目标球距离/方位与浅蓝球差异大时才撞。

**兜底策略**：
- 视觉完全失败 → 按"每行每列 1 颗"先验做盲扫 Z 字形（很可能扣分但保底）。
- 撞击后 stuck 检测：5 秒位置无变化 → 后退 1m 重新接近。

**可复用代码**：HSV 阈值起点用 `cyberdog_competition/config.py` 的 orange `(5,80,80)-(25,255,255)`；视觉伺服模板参考 `example/stage1/utils/yellow_light_detector.py` 的 approach 算法。

### 4.3 Stage 3 · 曲道冲锋（固定赛道型）

**任务本质**：曲线→直线→曲线，到右上角出口，纯赛道跟随无交互。

**Phase 状态机**：
```
ENTER → FOLLOW_CORRIDOR (持续走廊居中)
      → DETECT_EXIT (前方距离突变 + 右侧开阔)
      → STRAIGHT_TO_EXIT → DONE
```

**感知依赖**：`perception.lidar`（核心）；`perception.lane_edge`（备用）。

**关键算法**：
- 复用 Stage 1 的走廊居中 PD，速度调高（0.32 m/s）。
- 曲线段：左右距离差自然变化 → 横向修正自动适应；直线段：左右稳定 → 拉直前进。

**兜底策略**：
- 雷达失效 → 视觉黄边检测改用 `(22,80,80)-(38,255,255)` HSV。
- 时序兜底：弧右 4s → 直 4s → 弧左 4s（仅作最终保底）。

**可复用代码**：完全复用 Stage 1 的走廊居中函数（合并到 `core/nav.py` 的 `lane_follow_pd()`）。

### 4.4 Stage 4 · 深隧寻珍（最高难度，胜负手）

**任务本质**：横通道 + 3 竖通道；找 3 目标（可乐/橙球/足球）做指定交互 + 语音播报；避 3 障碍（限高杆/不可越障碍）；自选顺序；前腿碰独木桥起点结束。

**Phase 状态机**：
```
ENTER → SCAN_LANE_i (i=0,1,2 横向移到每条竖通道入口)
      ↓ 分支决策：
    见限高杆 → ANNOUNCE("识别到限高杆") → LOW_WALK 通过
    见不可越障碍 → ANNOUNCE("识别到无法跨越障碍") → DETOUR 借虚线绕行
    见可乐瓶 → ANNOUNCE("识别到可乐瓶") → APPROACH+BUMP
    见橙球 → ANNOUNCE("识别到橙色小球") → APPROACH+BUMP
    见足球 → ANNOUNCE("识别到足球") → APPROACH+KICK
      ↓
  GO_BRIDGE_START (前腿碰独木桥起点) → DONE
```

**感知依赖**：5 个并行检测器同时跑——`orange_ball`/`football`/`coke`/`red_pole`/`block_obstacle`，全部走 ROS2 topic 异步发布。

**关键算法**：
- 限高杆 vs 不可越障碍的区分（雷达 + 视觉互补）：
  - 限高杆：视觉检测到红色 + 高宽比 ≥ 8（长条状）；雷达扫不到该位置的连续返回（底部 40cm 是空气）。
  - 不可越障碍：视觉检测到灰色方块；雷达连续返回（实心）。
- 语音必须在交互前播报：状态机严格 `ANNOUNCE → wait 1s → INTERACT`，每个目标只播报一次（用 `announced_set` 记忆）。
- 限高杆步态：复用 `gait/limit_pole/*.toml` + `core/gaits/low_walk` 步态。
- 绕行虚线：用 `core/nav.align_axis_by_strafing` 横向 1m → 直行 2m → 横向 -1m 回到原通道。

**兜底策略**：
- 检测器都失效 → 顺序保守脚本：每条竖通道前进 2m 看是否撞到东西，撞到了播报"识别到 XX" + 后退 → 下一条。
- 限高杆识别失败但碰撞了 → IMU 检测 pitch 突变 → 立即趴下重试。

**可复用代码**：红色限高杆检测来自 `demo1/custom_tasks.py` 的 `_detect_red_pole`；自定义步态来自 `demo1/motion/special_action/limit/*.toml`。

### 4.5 Stage 5 · 孤梁稳渡（视觉伺服型）

**任务本质**：独木桥跟随 + 4 足越虚线后跳下，不能身体撞桥。

**Phase 状态机**：
```
ENTER → SLOW_BRIDGE_WALK (低速 + 低步高 + 居中)
      → DETECT_DASHED_LINE (视觉检测虚线 + 4 足全过)
      → JUMP_DOWN (执行跳下步态)
      → DONE
```

**感知依赖**：`perception.lidar`（独木桥边沿 RANSAC 拟合）+ `perception.dashed_line`（虚线检测）。

**关键算法**：
- 用雷达扫左右两条边沿线 + RANSAC 求中线 → 计算偏离量做居中（复用 `demo1/lidar_adjust.py` 的最佳直线挑选逻辑：长度接近 1m + 距离最近）。
- 虚线检测：HSV 找白色短段 + 计数；4 足都过用机器人长度（约 60cm）+ 位置估计。
- IMU 监控 roll 角，超过 15° 立即停下重新居中。

**兜底策略**：纯位置式——从 stage 入口走固定距离后强制跳下（独木桥总长固定）。

**可复用代码**：`demo1/lidar_adjust.py` 的最佳直线挑选算法。

### 4.6 Stage 6 · 撷金建功（视觉伺服 + 终点对齐）

**任务本质**：从出口踢球 → 走到终点圈 → 趴下（四足在圈内）。

**Phase 状态机**：
```
ENTER → FIND_BALL → ALIGN_BEHIND_BALL (走到球后方对齐球门方向)
      → KICK (高速前冲)
      → CHECK_BALL_GONE (确认球已踢出)
      → APPROACH_FINISH_CIRCLE → LIE_DOWN_IN_CIRCLE → DONE
```

**感知依赖**：`perception.football`（黑白球检测）+ `perception.lidar`（终点圈定位）+ 视觉找终点圈（彩色边界）。

**关键算法**：
- 球-球门-狗三点几何：找到球的位置后，计算"球到球门的方向向量"，狗需走到球的反方向延长线上再踢。
- 踢球用高速前冲：vx = 0.55 m/s × 1.5s。
- 终点圈定位：圈是固定位置，赛段入口后已知大致方向，用 `nav.align_axis_by_driving_forward` 走到目标坐标。

**兜底策略**：找不到球 → 按出口位置直接 KICK 盲踢 → 走预设距离到终点圈。

**可复用代码**：足球检测从 `cyberdog_competition/vision.py` 的 `detect_football` 起步（黑白配色 grayscale 阈值），再加圆度过滤。

### 4.7 各赛段依赖与风险总览

| 赛段 | Core 必需 | Perception 必需 | Perception 备用 | 风险 |
|---|---|---|---|---|
| 1 | basic_action + nav | lidar | lane_edge | 低（已验证） |
| 2 | basic_action + nav | orange_ball | lidar (防撞) | 中（视觉敏感） |
| 3 | basic_action + nav | lidar | lane_edge | 低（复用 1） |
| 4 | + custom_gait + voice | orange/football/coke/red_pole/block | lidar | 高（5 检测器并行） |
| 5 | basic_action | lidar (RANSAC) | dashed_line | 中（平衡敏感） |
| 6 | basic_action | football | lidar | 中（球门几何） |

---

## 5. 目标目录结构

```
cyberdog_dev/
├── main.py                      # 顶层调度：解析 --mode/--stages，tick 循环
├── config/
│   ├── hsv.toml                 # 各检测器 HSV 阈值（sim/real 两套）
│   ├── topics.toml              # 相机/雷达 topic 名（sim/real）
│   └── stage_params.toml        # 各赛段速度/距离阈值/超时
├── core/
│   ├── robot_ctrl.py            # ← demo1/motion/utils/Robot_Ctrl.py
│   ├── pose_monitor.py          # ← demo1/motion/utils/RobotPoseMonitor.py
│   ├── voice.py                 # ← demo1/motion/utils/VoiceController.py
│   ├── basic_action.py          # ← demo1/motion/utils/basic_action.py
│   ├── nav.py                   # ← demo1/motion/utils/nav.py + stage1 走廊居中
│   ├── stage_base.py            # 新写：Stage / StageStatus / StageResult
│   ├── stage_context.py         # 新写：StageContext + build_context
│   ├── ekf_fusion.py            # ← demo1/ekf_fusion.py（可选位姿融合节点）
│   ├── odom_broadcaster.py      # ← demo1/motion/utils/odom_broadcaster.py
│   ├── gaits/
│   │   ├── stone_walk.py        # ← example/stage1/utils/stone_plate_walk.py
│   │   └── low_walk.py          # ← example/stage1/utils/low_moonwalk.py
│   └── lcm_type/                # ← 现有 cyberdog_dev/lcm_type/（移动位置）
├── perception/
│   ├── hub.py                   # 新写：PerceptionHub + 数据类
│   ├── _thresholds.py           # ← cyberdog_competition/config.py 的 HSV 常数
│   ├── lidar.py                 # ← demo1/lidar_detect.py + lidar_adjust.py 合并
│   ├── arrow.py                 # ← demo1/detector/arrow_publisher+subscribe.py
│   ├── orange_ball.py           # 新写（HSV + 圆度）
│   ├── red_pole.py              # ← demo1/custom_tasks.py 的 _detect_red_pole
│   ├── football.py              # 新写（黑白 + 圆度）
│   ├── coke.py                  # 新写
│   ├── block_obstacle.py        # 新写（灰色方块）
│   ├── lane_edge.py             # 新写（黄边 HSV）
│   └── dashed_line.py           # 新写（白色虚线段）
├── stages/
│   ├── stage1_stone_path.py     # ← example/stage1/stage1.py 重构成 Stage 类
│   ├── stage2_orange_balls.py   # 新写
│   ├── stage3_curve_dash.py     # 新写（复用 nav 走廊居中）
│   ├── stage4_tunnel_search.py  # 新写（最大工作量）
│   ├── stage5_bridge.py         # 新写
│   └── stage6_kick.py           # 新写
├── gait/
│   └── limit_pole/              # ← demo1/motion/special_action/limit/*.toml
├── scripts/
│   ├── dev.sh                   # ← MiCup/scripts/dev.sh（已有）
│   └── launch.sh                # ← MiCup/scripts/launch.sh（已有，需改）
└── README.md                    # 重写：新架构说明 + 开发指南
```

注：现有 `cyberdog_dev/` 下的 `cyberdog_competition/`、`hello_*.py`、`Robot_Ctrl.py`、旧 `main.py`、`SOLUTION_ANALYSIS.md`、`lcm_type/`、`customized_gait/`、`toml/` 在迁移期间全部保留，新代码写在新目录，验证通过后再单独清理。

---

## 6. 迁移说明

### 6.1 lcm_type 核对结果

经 diff 验证：

- `cyberdog_dev/lcm_type/` 与 `demo2/lcm_type/` 逐字节相同。
- `cyberdog_dev/lcm_type/` 与 demo1 的 lcm 文件功能等价（LCM fingerprint 一致，通信兼容），demo1 只是旧式写法、一个文件塞多个类。

结论：`cyberdog_dev/lcm_type/` 本身就是干净的自动生成版，已正确完整。`core/lcm_type/` 直接用现有的 `cyberdog_dev/lcm_type/`，迁移时只是移动位置。

### 6.2 迁移时必须做的修改（不是纯拷贝）

| 文件 | 必须改什么 | 原因 |
|---|---|---|
| `core/robot_ctrl.py` | 删掉硬编码 `/home/mi/MI/...` 路径，改 `Path(__file__).parent` | demo1 写死了实机路径 |
| `core/basic_action.py` | 删掉 `execute_custom_gait_sequence` 里的绝对路径；删掉去年赛段专用动作 | 去年地图相关 |
| `core/nav.py` | 保留 `align_yaw_to_target` / `align_axis_*`；新增 `lane_follow_pd()`（从 stage1 抽） | 走廊居中要复用 |
| `perception/lidar.py` | `cv2.findContours` 三返回值 → 检测 OpenCV 版本兼容；合并 detect/adjust 两个 RANSAC | demo1 是 OpenCV3 写法 |
| `perception/*.py` 所有检测器 | 统一成"独立 ROS2 节点 + 发布 topic"模板；topic 名从 `config/topics.toml` 读 | 现在 demo1 的 detector 各写各的 |
| `stages/stage1_stone_path.py` | `Stage1Runner(Node)` → `Stage1StonePath(Stage)`；`control_loop` 拆成 `tick()` | 适配新基类 |
| `core/ekf_fusion.py` | 删掉硬编码 topic 名，改从 config 读 | demo1 写死了 `/mi_desktop_xxx/odom_out` |

### 6.3 迁移顺序（按依赖关系）

```
第 1 批（无依赖，先搬）：
  core/lcm_type/  →  core/robot_ctrl.py  →  core/pose_monitor.py
  core/voice.py  →  core/basic_action.py

第 2 批（依赖第 1 批）：
  core/nav.py  →  core/stage_base.py  →  core/stage_context.py
  core/gaits/  →  gait/limit_pole/  →  core/ekf_fusion.py  →  core/odom_broadcaster.py

第 3 批（感知，互相独立可并行）：
  perception/_thresholds.py  →  perception/lidar.py  →  perception/arrow.py
  →  perception/red_pole.py  →  perception/hub.py
  →  其余新检测器（orange_ball/football/coke/block_obstacle/lane_edge/dashed_line）

第 4 批（赛段，依赖前三批）：
  stages/stage1（迁移+重构，先跑通验证整链路 —— 关键里程碑）
  →  stages/stage3（复用 1 的走廊）
  →  stages/stage2 → stage5 → stage6 → stage4（最难的最后）

第 5 批：
  config/  →  main.py  →  scripts/  →  README.md
```

**关键里程碑**：第 4 批的 stage1 跑通 = 整个 Core + Perception + Stage 链路验证通过。

### 6.4 迁移策略

迁移是纯增量的：`demo1/`、`example/stage1/` 等示例目录完全不动；`cyberdog_dev/` 下现有文件暂时全部保留，新代码写在新目录。等新架构 stage1 在 Gazebo 跑通、整链路验证通过后，再单独做一次清理 commit 删除废弃文件。新旧并存期间 `main.py` 用新的、旧的留着随时回退对照。

---

## 7. 后续开发指南

### 7.1 如何新增/调试一个赛段

```
1. 在 stages/ 新建 stageN_xxx.py，继承 Stage
2. 定义 Phase enum + 在 tick() 里写状态转移
3. 在 main.py 的 STAGE_REGISTRY 注册
4. 单独跑：python3 main.py --mode sim --stages N
5. 调参：改 config/stage_params.toml，不改代码
```

### 7.2 如何新增一个检测器

```
1. perception/ 新建 xxx_detector.py，套用"独立 ROS2 节点"模板
2. 订阅相机/雷达 topic，发布检测结果 topic
3. 在 perception/hub.py 加一个 latest_xxx() 方法
4. 单独验证：python3 -m perception.xxx_detector（带可视化窗口）
5. HSV 阈值写进 config/hsv.toml，不硬编码
```

### 7.3 调试分层策略（出问题时从下往上查）

| 现象 | 先查哪一层 | 怎么查 |
|---|---|---|
| 狗不动 | Core | `python3 -m core.robot_ctrl` 发测试命令 |
| 狗动作乱 | Core | 检查 LCM 心跳频率、life_count |
| 位姿数据不对 | Core | `ros2 topic echo /tf`，看 odom→base_link_leg |
| 检测不到目标 | Perception | 单跑 detector 看可视化窗口 + 调 HSV |
| 检测到了但不动作 | Stage | 看 tick() 日志，打印当前 phase |
| 赛段间衔接错位 | App | 看 main.py 的 stage 切换日志 |

### 7.4 启动流程（scripts/launch.sh 重写后）

```bash
# 1. 启动感知节点（后台，各自独立进程）
python3 -m perception.lidar &
python3 -m perception.orange_ball &
# ... 按 --stages 需要的检测器启动
# 2. 启动位姿广播
python3 -m core.odom_broadcaster &
# 3. 启动主程序
python3 main.py --mode sim --stages 1-6
```

### 7.5 测试策略

- **Core 层**：可写纯逻辑单测——`nav.py` 的角度归一化、`align_yaw` 的误差计算等纯函数。
- **Perception 层**：用静态图片测试——存几张 Gazebo 截图到 `tests/fixtures/`，断言检测器能框出目标。
- **Stage 层**：无法离线测，只能在 Gazebo 里跑——但 Phase enum 让你能打印状态轨迹，对照"预期状态序列"人工核对。
- **回归**：每次改完一个赛段，跑一遍 `--stages 1-N` 确认没破坏前面的。

### 7.6 git 分支策略

- 本次重构在 `feat/framework` 分支进行。
- 里程碑 commit：每完成一个迁移批次（见 6.3）提交一次。
- stage1 跑通 = 第一个"可演示" commit。

---

## 8. 范围边界与已知风险

- 本设计未验证仿真真实坐标、相机视角、步态稳定性、碰撞判定——所有具体数值（速度、距离阈值、HSV 范围、超时）都是保守初值，必须进 Gazebo 后逐段调参。
- Stage 4 的 5 检测器并行会占算力，是否合并为单节点多次推理留待实施阶段实测后决定。
- Stage 6 的球-球门-狗三点几何公式留待实施阶段细化。
- 仿真到实机的迁移差异（相机焦距、光照、namespace）通过 config 文件吸收，但实机标定仍需决赛前现场完成。
