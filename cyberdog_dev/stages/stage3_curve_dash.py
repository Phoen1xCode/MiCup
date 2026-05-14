"""Stage 3 · 曲道冲锋."""

import time
from enum import Enum, auto
from pathlib import Path

from core._lane_follow import LaneFollowParams
from core.nav import lane_follow_pd
from core.stage_base import Stage, StageStatus
from config.loader import load_stage_params


class Phase(Enum):
    ENTER = auto()
    FOLLOW_CORRIDOR = auto()
    STRAIGHT_TO_EXIT = auto()
    DONE = auto()


class Stage3CurveDash(Stage):
    stage_id = 3
    name = "曲道冲锋"

    def __init__(self, ctx):
        super().__init__(ctx)
        config_dir = Path(__file__).resolve().parent.parent / "config"
        self.p = load_stage_params(config_dir / "stage_params.toml", stage_id=3)
        self.phase = Phase.ENTER
        self.phase_start = 0.0

    def on_enter(self) -> None:
        self.start_time = time.monotonic()
        self.phase_start = self.start_time
        self.ctx.pose.set_origin_here()
        self.ctx.logger.info(f"[{self.name}] 进入")

    def max_duration_sec(self) -> float:
        return float(self.p["max_time"])

    def _switch(self, phase: Phase) -> None:
        self.phase = phase
        self.phase_start = time.monotonic()
        self.ctx.logger.info(f"[{self.name}] -> {phase.name}")

    def tick(self) -> StageStatus:
        elapsed = time.monotonic() - self.phase_start
        corridor = self.ctx.perception.latest_lidar_corridor()

        if self.phase == Phase.ENTER:
            self._switch(Phase.FOLLOW_CORRIDOR)
            return StageStatus.RUNNING

        if self.phase == Phase.FOLLOW_CORRIDOR:
            params = LaneFollowParams(
                forward_speed=float(self.p["forward_speed"]),
                lateral_gain=float(self.p["lateral_gain"]),
                max_lateral=float(self.p["max_lateral"]),
                front_stop_distance=float(self.p["front_stop_distance"]),
            )
            lane_follow_pd(self.ctx.dog, corridor, params)
            exit_visible = (
                elapsed >= self.p["min_follow_time"]
                and corridor.front >= self.p["exit_front_clear"]
                and corridor.right >= self.p["open_side_threshold"]
            )
            if exit_visible or elapsed >= self.p["max_follow_time"]:
                self._switch(Phase.STRAIGHT_TO_EXIT)
            return StageStatus.RUNNING

        if self.phase == Phase.STRAIGHT_TO_EXIT:
            self.ctx.dog.set_velocity(self.p["exit_speed"], 0.0, 0.0)
            if elapsed >= self.p["exit_time"]:
                self._switch(Phase.DONE)
            return StageStatus.RUNNING

        self.ctx.dog.stop()
        return StageStatus.SUCCEEDED

    def on_exit(self) -> None:
        self.ctx.dog.stop()
