"""Stage 2 · 荒野寻珠."""

import time
from enum import Enum, auto
from pathlib import Path

from core.stage_base import Stage, StageStatus
from config.loader import load_stage_params


class Phase(Enum):
    ENTER = auto()
    SWEEP_SCAN = auto()
    APPROACH_NEXT = auto()
    BUMP = auto()
    BACKOFF = auto()
    GO_EXIT = auto()
    DONE = auto()


class Stage2OrangeBalls(Stage):
    stage_id = 2
    name = "荒野寻珠"

    def __init__(self, ctx):
        super().__init__(ctx)
        config_dir = Path(__file__).resolve().parent.parent / "config"
        self.p = load_stage_params(config_dir / "stage_params.toml", stage_id=2)
        self.phase = Phase.ENTER
        self.phase_start = 0.0
        self.hit_count = 0

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

    def _select_target(self):
        balls = self.ctx.perception.latest_orange_balls()
        if not balls:
            return None
        return min(balls, key=lambda b: b.distance_m)

    def tick(self) -> StageStatus:
        elapsed = time.monotonic() - self.phase_start

        if self.phase == Phase.ENTER:
            self._switch(Phase.SWEEP_SCAN)
            return StageStatus.RUNNING

        if self.phase == Phase.SWEEP_SCAN:
            self.ctx.dog.set_velocity(0.0, 0.0, self.p["search_turn_speed"])
            if self._select_target() is not None or elapsed >= 2.0:
                self._switch(Phase.APPROACH_NEXT)
            return StageStatus.RUNNING

        if self.phase == Phase.APPROACH_NEXT:
            target = self._select_target()
            if target is None:
                self.ctx.dog.set_velocity(0.0, 0.0, self.p["search_turn_speed"])
                return StageStatus.RUNNING
            if target.distance_m <= self.p["hit_distance"]:
                self._switch(Phase.BUMP)
                return StageStatus.RUNNING
            wz = target.bearing_rad * self.p["approach_turn_gain"]
            self.ctx.dog.set_velocity(self.p["approach_speed"], 0.0, wz)
            return StageStatus.RUNNING

        if self.phase == Phase.BUMP:
            self.ctx.dog.set_velocity(self.p["bump_speed"], 0.0, 0.0)
            if elapsed >= self.p["bump_time"]:
                self.hit_count += 1
                self._switch(Phase.BACKOFF)
            return StageStatus.RUNNING

        if self.phase == Phase.BACKOFF:
            self.ctx.dog.set_velocity(self.p["backoff_speed"], 0.0, 0.0)
            if elapsed >= self.p["backoff_time"]:
                if self.hit_count >= self.p["target_count"]:
                    self._switch(Phase.GO_EXIT)
                else:
                    self._switch(Phase.APPROACH_NEXT)
            return StageStatus.RUNNING

        if self.phase == Phase.GO_EXIT:
            self.ctx.dog.set_velocity(self.p["exit_speed"], 0.0, 0.0)
            if elapsed >= self.p["exit_time"]:
                self._switch(Phase.DONE)
            return StageStatus.RUNNING

        self.ctx.dog.stop()
        return StageStatus.SUCCEEDED

    def on_exit(self) -> None:
        self.ctx.dog.stop()
