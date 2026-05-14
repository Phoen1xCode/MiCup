# 2026 小米杯参赛代码 cyberdog_dev

## 架构设计

### 四层架构

```text
┌─────────────────────────────────────────────────┐
│  Application 层  main.py                        │  顶层调度
├─────────────────────────────────────────────────┤
│  Stage 层        stages/stageN_*.py             │  每赛段独立状态机
├─────────────────────────────────────────────────┤
│  Perception 层   perception/{lidar,*_detector}  │  独立感知节点
├─────────────────────────────────────────────────┤
│  Core 层         core/{robot_ctrl,pose,nav,...} │  底层 LCM + TF + 动作原语
└─────────────────────────────────────────────────┘
            ↕
   LCM (control + 心跳) | ROS2 (传感器/感知 topic)
```

### 各层职责

| 层 | 职责 | 不做的事 |
|---|---|---|
| Core | 把命令变成机器狗动作；把 TF 变成 `(x,y,yaw)` | 任何"决策"、任何视觉处理 |
| Perception | 把图像/雷达变成"检测结果 + topic" | 任何"动作"、任何赛段逻辑 |
| Stage | 用 Phase 状态机编排该赛段的"看→想→做"循环 | 直接发 LCM、直接处理像素 |
| App | 决定跑哪几个 stage、参数解析、错误回收 | 任何赛段细节 |

## 目录结构

```text
core/         底层控制与位姿（LCM 控制、TF 位姿、动作原语、语音、低身步态、Stage 基类）
perception/   感知节点（激光雷达走廊检测、颜色/目标检测、PerceptionHub）
stages/       每个赛段一个 Phase 状态机
config/       sim/real topic 配置、各赛段参数
main.py       入口：解析参数、组装 context、跑 tick 循环
scripts/      启动脚本
tests/        纯逻辑单元测试（pytest）
```

## 运行（需 ROS2 环境 / Gazebo 容器）

```bash
bash scripts/launch.sh sim 1        # 仿真跑赛段 1
bash scripts/launch.sh sim 1-6      # 仿真跑全部赛段
bash scripts/launch.sh real 1       # 实机跑赛段 1
```

## 本机开发（macOS，无 ROS2）

```bash
python3 -m pip install -r requirements-dev.txt
python3 -m pytest                   # 跑纯逻辑单元测试
python3 main.py --help              # 验证入口 argparse
python3 -m compileall -q core perception stages config main.py
bash -n scripts/launch.sh
```

## 当前进度

- [x] Plan 1：Core/Perception 地基 + Stage 1（石径探路）
- [x] Stage 2：荒野寻珠状态机，按橙色球检测结果接近、撞击、退出
- [x] Stage 3：曲道冲锋状态机，复用走廊居中控制
- [x] Stage 4：深隧寻珍状态机，包含目标/障碍语音播报、低身通过和绕障动作
- [x] Stage 5：孤梁稳渡状态机，支持桥面慢行、虚线检测和超时跳下
- [x] Stage 6：撷金建功状态机，支持足球检测对准、踢球、终点趴下
- [x] 感知合同：橙色球、足球、红色限高杆、方块障碍、可乐、黄边线、白虚线

## Gazebo 验证清单

当前代码已经具备 1-6 赛段的可执行架构，但 HSV 阈值、相机距离估计、桥面控制和各赛段时间参数仍需在 Gazebo 中逐段调参。

```bash
python3 -m pytest -v
python3 -m compileall -q core perception stages config main.py
python3 main.py --help
python3 -c "from main import parse_stages; print(parse_stages('1-6'))"
bash -n scripts/launch.sh

bash scripts/launch.sh sim 1
bash scripts/launch.sh sim 2
bash scripts/launch.sh sim 3
bash scripts/launch.sh sim 4
bash scripts/launch.sh sim 5
bash scripts/launch.sh sim 6
bash scripts/launch.sh sim 1-6
```

Gazebo 中重点观察：

- `/perception/*` topic 是否稳定输出 JSON，尤其是橙色球、足球、红杆、可乐和虚线。
- Stage 2 是否只撞击橙色球，避免误碰浅蓝球。
- Stage 4 五句语音是否与赛题完全一致，限高杆是否全程低身通过。
- Stage 5 是否在四足越过独木桥虚线后再跳下。
- Stage 6 足球是否从出口方向踢出，终点动作是否趴下。

## 迁移来源

- `core/` 多数模块来自 `example/demo1/motion/utils/`（已验证）
- `stages/stage1_stone_path.py` 重构自 `example/stage1/stage1.py`
- `stages/stage2_orange_balls.py` 至 `stages/stage6_kick.py` 为本次架构内新增的保守状态机
- 旧的 `cyberdog_competition/` 包暂保留，待 1-6 赛段在 Gazebo 验证后清理
