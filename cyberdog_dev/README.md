# cyberdog_dev - 2026 小米杯参赛代码

四层架构（Core / Perception / Stage / App），详见
`../docs/superpowers/specs/2026-05-14-cyberdog-dev-architecture-design.md`。

## 目录结构

```text
core/         底层控制与位姿（LCM 控制、TF 位姿、动作原语、闭环对齐、Stage 基类）
perception/   感知节点（激光雷达走廊检测、PerceptionHub）
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
```

## 当前进度

- [x] Plan 1：Core/Perception 地基 + Stage 1（石径探路）
- [ ] Plan 2-6：Stage 2-6、语音、自定义步态、其余检测器

## 迁移来源

- `core/` 多数模块来自 `example/demo1/motion/utils/`（已验证）
- `stages/stage1_stone_path.py` 重构自 `example/stage1/stage1.py`
- 旧的 `cyberdog_competition/` 包暂保留，待 Stage 1 在 Gazebo 验证后清理
