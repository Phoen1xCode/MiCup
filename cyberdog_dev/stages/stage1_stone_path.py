"""Stage 1 · 石径探路 -- Phase 状态机（spec 4.1）。

重构自 example/stage1/stage1.py：去掉 ROS2 Node 继承与 timer，
control_loop 内容搬进 tick()，传感器走 ctx.perception，动作走 ctx.dog。
"""

import time
from enum import Enum, auto
from pathlib import Path

from core.lane_follow import (
    VisualLaneFollowParams,
    compute_visual_lane_follow_correction,
)
from core.framework.stage import Stage, StageStatus


class Phase(Enum):
    RECOVERY_STAND = auto()
    STABILIZE = auto()
    STRAIGHT_TO_BEND = auto()
    TURNING = auto()
    STRAIGHT_TO_EXIT = auto()
    DONE = auto()


class Stage1StonePath(Stage):
    stage_id = 1
    name = "石径探路"

    def __init__(self, ctx):
        super().__init__(ctx)
        self.phase = Phase.RECOVERY_STAND
        self.phase_start = 0.0
        self.turn_direction = 0.0
        self.estimated_turn_angle = 0.0
        self.last_tick_time = 0.0
        self.recovery_stand_requested = False
        config_dir = Path(__file__).resolve().parent.parent / "config"
        from config.loader import load_stage_params
        self.p = load_stage_params(config_dir / "stage_params.toml", stage_id=1)

    def on_enter(self) -> None:
        self.start_time = time.monotonic()
        self.phase_start = self.start_time
        self.last_tick_time = self.start_time
        self.recovery_stand_requested = False
        self.ctx.pose.set_origin_here()
        self.ctx.logger.info(f"[{self.name}] 进入，phase=RECOVERY_STAND")

    def max_duration_sec(self) -> float:
        return float(self.p["max_time"])

    def _switch(self, phase: Phase) -> None:
        self.phase = phase
        self.phase_start = time.monotonic()
        self.ctx.logger.info(f"[{self.name}] -> {phase.name}")

    def _visual_lane_command(self, lane_edges):
        params = VisualLaneFollowParams(
            forward_speed=float(self.p["forward_speed"]),
            lateral_gain=float(self.p["visual_lateral_gain"]),
            max_lateral=float(self.p["visual_max_lateral"]),
            min_confidence=float(self.p["visual_lane_confidence"]),
        )
        return compute_visual_lane_follow_correction(lane_edges, params)

    def _visual_turn_direction(self, lane_edges) -> float:
        if lane_edges.turn_hint == "left":
            return 1.0
        if lane_edges.turn_hint == "right":
            return -1.0
        return 1.0

    def tick(self) -> StageStatus:
        now = time.monotonic()
        dt = max(0.0, now - self.last_tick_time)
        self.last_tick_time = now
        elapsed = now - self.phase_start

        lane_edges = self.ctx.perception.latest_lane_edges()

        if self.phase == Phase.RECOVERY_STAND:
            if not self.recovery_stand_requested:
                self.ctx.dog.execute_discrete_action(mode=12, gait_id=0, wait_for_completion=True)
                self.recovery_stand_requested = True
            if time.monotonic() - self.phase_start >= self.p["stand_time"]:
                self._switch(Phase.STABILIZE)
            return StageStatus.RUNNING

        if self.phase == Phase.STABILIZE:
            self.ctx.dog.set_velocity_command(0.0, 0.0, 0.0)
            if elapsed >= self.p["stabilize_time"]:
                self._switch(Phase.STRAIGHT_TO_BEND)
            return StageStatus.RUNNING

        if self.phase == Phase.STRAIGHT_TO_BEND:
            vx, vy, wz = self._visual_lane_command(lane_edges)
            self.ctx.dog.set_velocity_command(vx, vy, wz)
            bend_by_visual = (
                elapsed >= self.p["min_straight_time"]
                and lane_edges.horizontal_confidence >= self.p["visual_horizontal_confidence"]
            )
            bend_by_timeout = elapsed >= self.p["max_straight_time"]
            if bend_by_visual or bend_by_timeout:
                self.turn_direction = self._visual_turn_direction(lane_edges)
                self.estimated_turn_angle = 0.0
                self._switch(Phase.TURNING)
            return StageStatus.RUNNING

        if self.phase == Phase.TURNING:
            self.ctx.dog.set_velocity_command(
                self.p["turn_forward_speed"], 0.0,
                self.turn_direction * self.p["turn_yaw_speed"])
            self.estimated_turn_angle += abs(self.p["turn_yaw_speed"]) * dt
            turned_enough = self.estimated_turn_angle >= self.p["turn_angle"]
            timed_out = elapsed >= self.p["max_turn_time"]
            if elapsed >= self.p["min_turn_time"] and (turned_enough or timed_out):
                self._switch(Phase.STRAIGHT_TO_EXIT)
            return StageStatus.RUNNING

        if self.phase == Phase.STRAIGHT_TO_EXIT:
            self.ctx.dog.set_velocity_command(self.p["exit_speed"], 0.0, 0.0)
            if elapsed >= self.p["finish_straight_time"]:
                self._switch(Phase.DONE)
            return StageStatus.RUNNING

        self.ctx.dog.set_velocity_command(0.0, 0.0, 0.0)
        return StageStatus.SUCCEEDED

    def on_exit(self) -> None:
        self.ctx.dog.set_velocity_command(0.0, 0.0, 0.0)
        self.ctx.logger.info(f"[{self.name}] 退出")
