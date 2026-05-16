#!/usr/bin/env python3
"""2026 MiCup CyberDog 比赛入口。

用法：
    python3 main.py --mode sim --stages 1
    python3 main.py --mode sim --stages 1-6
    python3 main.py --mode sim --stages all
    python3 main.py --mode real --stages 1,3,5

需在 ROS2 环境（Gazebo 容器 / 实机）中运行。感知节点与 odom 广播
需提前由 scripts/launch.sh 启动。
"""

import argparse
import sys
import time


def parse_stages(value: str):
    if value in ("all", "1-6"):
        return [1, 2, 3, 4, 5, 6]
    result = []
    for part in value.split(","):
        part = part.strip()
        if not part:
            continue
        if "-" in part:
            left, right = part.split("-")
            result.extend(range(int(left), int(right) + 1))
        else:
            result.append(int(part))
    for n in result:
        if n < 1 or n > 6:
            raise argparse.ArgumentTypeError("stages must be in 1..6")
    return result


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="2026 MiCup CyberDog competition runner"
    )
    parser.add_argument(
        "--mode",
        default="sim",
        choices=["sim", "real"],
        help="sim = Gazebo 仿真，real = 实机",
    )
    parser.add_argument(
        "--stages", default="1-6", type=parse_stages, help="要跑的赛段，如 1-6 或 1,3,5"
    )
    return parser


def run_stage(stage, ctx) -> "StageResult":
    """以 ~20Hz tick 一个赛段直到结束或超时。"""
    from core.framework.stage import StageResult, StageStatus

    stage.on_enter()
    start = time.monotonic()
    status = StageStatus.RUNNING
    try:
        while status == StageStatus.RUNNING:
            status = stage.tick()
            if time.monotonic() - start > stage.max_duration_sec():
                ctx.logger.warn(f"[{stage.name}] 超时，强制结束")
                status = StageStatus.FAILED
                break
            time.sleep(0.05)
    finally:
        stage.on_exit()
    elapsed = time.monotonic() - start
    return StageResult(
        stage_id=stage.stage_id,
        name=stage.name,
        status=status,
        notes=list(stage.notes),
        elapsed_sec=elapsed,
    )


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)

    from core.framework.context import RunMode, build_context
    from stages.stage1_stone_path import Stage1StonePath
    from stages.stage2_orange_balls import Stage2OrangeBalls
    from stages.stage3_curve_dash import Stage3CurveDash
    from stages.stage4_tunnel_search import Stage4TunnelSearch
    from stages.stage5_bridge import Stage5Bridge
    from stages.stage6_kick import Stage6Kick

    STAGE_REGISTRY = {
        1: Stage1StonePath,
        2: Stage2OrangeBalls,
        3: Stage3CurveDash,
        4: Stage4TunnelSearch,
        5: Stage5Bridge,
        6: Stage6Kick,
    }

    ctx = build_context(RunMode(args.mode))
    results = []
    try:
        for stage_id in args.stages:
            stage_cls = STAGE_REGISTRY.get(stage_id)
            if stage_cls is None:
                ctx.logger.warn(f"赛段 {stage_id} 尚未实现，跳过")
                continue
            ctx.logger.info(f"=== 开始赛段 {stage_id} ===")
            results.append(run_stage(stage_cls(ctx), ctx))
    except KeyboardInterrupt:
        ctx.logger.warn("收到中断，停止机器狗")
        ctx.dog.set_velocity_command(0.0, 0.0, 0.0)

    print("\n运行总结")
    for r in results:
        print(f"  S{r.stage_id} {r.name}: {r.status.name} ({r.elapsed_sec:.1f}s)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
