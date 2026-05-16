"""Stage 5 · 孤梁稳渡."""

import time
from enum import Enum, auto
from pathlib import Path

from core.framework.stage import Stage, StageStatus
from config.loader import load_stage_params


class Phase(Enum):
    ENTER = auto()
    WALK_BRIDGE = auto()
    CLEAR_DASHED_LINE = auto()
    JUMP_DOWN = auto()
    DONE = auto()


class Stage5Bridge(Stage):
    stage_id = 5
    name = "孤梁稳渡"

    def __init__(self, ctx):
        super().__init__(ctx)
        config_dir = Path(__file__).resolve().parent.parent / "config"
        self.p = load_stage_params(config_dir / "stage_params.toml", stage_id=5)
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

    def _bridge_lateral_speed(self) -> float:
        edges = self.ctx.perception.latest_lane_edges()
        if edges.confidence <= 0.0:
            return 0.0
        raw = -(edges.left_offset_px + edges.right_offset_px) * self.p["lateral_gain"]
        return max(-self.p["max_lateral"], min(self.p["max_lateral"], raw))

    def _dashed_line_ready(self, elapsed: float) -> bool:
        dashed = self.ctx.perception.latest_dashed_line()
        return (
            elapsed >= self.p["min_walk_time"]
            and dashed is not None
            and dashed.confidence >= 0.5
        )

    def tick(self) -> StageStatus:
        elapsed = time.monotonic() - self.phase_start

        if self.phase == Phase.ENTER:
            self._switch(Phase.WALK_BRIDGE)
            return StageStatus.RUNNING

        if self.phase == Phase.WALK_BRIDGE:
            self.ctx.dog.set_velocity_command(
                self.p["bridge_speed"], self._bridge_lateral_speed(), 0.0)
            if self._dashed_line_ready(elapsed):
                self._switch(Phase.CLEAR_DASHED_LINE)
            elif elapsed >= self.p["max_walk_time"]:
                self._switch(Phase.JUMP_DOWN)
            return StageStatus.RUNNING

        if self.phase == Phase.CLEAR_DASHED_LINE:
            self.ctx.dog.set_velocity_command(self.p["bridge_speed"], 0.0, 0.0)
            if elapsed >= self.p["clear_dashed_time"]:
                self._switch(Phase.JUMP_DOWN)
            return StageStatus.RUNNING

        if self.phase == Phase.JUMP_DOWN:
            self.ctx.dog.set_velocity_command(self.p["jump_speed"], 0.0, 0.0)
            if elapsed >= self.p["jump_time"]:
                self._switch(Phase.DONE)
            return StageStatus.RUNNING

        self.ctx.dog.set_velocity_command(0.0, 0.0, 0.0)
        return StageStatus.SUCCEEDED

    def on_exit(self) -> None:
        self.ctx.dog.set_velocity_command(0.0, 0.0, 0.0)
