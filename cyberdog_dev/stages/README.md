# stages/

赛段层。每个文件 = 一个赛段 = 一个 Phase 状态机。Stage 只通过 `ctx.dog`（动作）、`ctx.pose`（位姿）、`ctx.perception`（感知快照）、`ctx.voice`（语音）这四个入口拿能力，**不直接发 LCM、不直接处理像素**。每个 Stage 继承 `core.stage_base.Stage`，实现 `on_enter / tick / on_exit / max_duration_sec`。`main.py` 以 20Hz 调 `tick()` 直到返回 `SUCCEEDED / FAILED / NEED_HELP`。

## 文件说明

| 文件 | 作用 |
|---|---|
| `__init__.py` | 包标记。 |
| `stage1_stone_path.py` | **赛段 1 · 石径探路**。Phase: `RECOVERY_STAND → STABILIZE → STRAIGHT_TO_BEND → TURNING → STRAIGHT_TO_EXIT → DONE`。靠 `perception.lane_edges`（视觉黄边线）走廊居中，雷达 corridor 兜底；估计转弯角，过弯后再直行出口。 |
| `stage2_orange_balls.py` | **赛段 2 · 荒野寻珠**。按 `perception.orange_balls` 接近最近的橙色球 → 撞击 → 退出。需要规避浅蓝球的误碰。 |
| `stage3_curve_dash.py` | **赛段 3 · 曲道冲锋**。复用 `_lane_follow` 的走廊居中思路；用 `dashed_line` 和 `lane_edges` 引导高速过曲道。 |
| `stage4_tunnel_search.py` | **赛段 4 · 深隧寻珍**。最复杂：包含目标识别（`PoleDet` 限高杆 / `ObjDet` 可乐 / `BallDet` 球）、五句强制语音播报、低身钻杆（调 `core.gaits.low_walk.execute_low_walk`）、绕障动作。 |
| `stage5_bridge.py` | **赛段 5 · 孤梁稳渡**。桥面慢行 + 用 `DashedLineDet` 判断已越过独木桥虚线 + 超时跳下保护。 |
| `stage6_kick.py` | **赛段 6 · 撷金建功**。对准足球（`BallDet`）→ 朝出口方向踢球 → 终点趴下。 |

## 迁移说明

**本目录的 stage2 ~ stage6 全部为新增**（针对 2026 荒野寻宝赛题），template 里没有对应物。

`stage1_stone_path.py` 是**重构**而非原样迁移：

| 维度 | 源（README 标注的 `example/stage1/stage1.py`，本仓库 template 中无） | 本文件 |
|---|---|---|
| 类型 | 继承 `rclpy.Node`，靠 `create_timer` 跑 `control_loop` | 继承 `core.stage_base.Stage`，由 `main.py` 的 tick 循环驱动 |
| 感知接入 | 自己订阅 image/scan topic | 通过 `ctx.perception.lane_edges / corridor` 拿快照 |
| 动作下发 | 自己组装 LCM cmd | 通过 `ctx.dog.set_velocity_command()` |
| 参数来源 | 写在类成员变量里 | `config/stage_params.toml [stage1]` |

template 中的 `template/main.py`（去年仓储物流：`go_garage_scan / go_curve_arrow / go_arrow_uphill / back_curve_put` 等大状态机）**整体未迁移**——赛题完全不同，硬编码坐标全部作废。
