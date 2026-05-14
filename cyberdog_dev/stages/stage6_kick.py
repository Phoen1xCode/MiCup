"""Stage 6 · 撷金建功."""

import time
from enum import Enum, auto
from pathlib import Path

from core.stage_base import Stage, StageStatus
from config.loader import load_stage_params


class Phase(Enum):
    ENTER = auto()
    FIND_BALL = auto()
    ALIGN_BEHIND_BALL = auto()
    KICK = auto()
    APPROACH_FINISH_CIRCLE = auto()
    LIE_DOWN_IN_CIRCLE = auto()
    DONE = auto()


class Stage6Kick(Stage):
    stage_id = 6
    name = "撷金建功"

    def __init__(self, ctx):
        super().__init__(ctx)
        config_dir = Path(__file__).resolve().parent.parent / "config"
        self.p = load_stage_params(config_dir / "stage_params.toml", stage_id=6)
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

        if self.phase == Phase.ENTER:
            self._switch(Phase.FIND_BALL)
            return StageStatus.RUNNING

        if self.phase == Phase.FIND_BALL:
            ball = self.ctx.perception.latest_football()
            if ball is None:
                self.ctx.dog.set_velocity(0.0, 0.0, self.p["search_turn_speed"])
                if elapsed >= 1.0:
                    self._switch(Phase.KICK)
                return StageStatus.RUNNING
            self.ctx.dog.set_velocity(0.0, 0.0, ball.bearing_rad * self.p["align_turn_gain"])
            self._switch(Phase.ALIGN_BEHIND_BALL)
            return StageStatus.RUNNING

        if self.phase == Phase.ALIGN_BEHIND_BALL:
            ball = self.ctx.perception.latest_football()
            if ball is None or abs(ball.bearing_rad) < 0.08:
                self._switch(Phase.KICK)
            else:
                self.ctx.dog.set_velocity(self.p["approach_speed"], 0.0,
                                          ball.bearing_rad * self.p["align_turn_gain"])
            return StageStatus.RUNNING

        if self.phase == Phase.KICK:
            self.ctx.dog.set_velocity(self.p["kick_speed"], 0.0, 0.0)
            if elapsed >= self.p["kick_time"]:
                self._switch(Phase.APPROACH_FINISH_CIRCLE)
            return StageStatus.RUNNING

        if self.phase == Phase.APPROACH_FINISH_CIRCLE:
            self.ctx.dog.set_velocity(self.p["finish_speed"], 0.0, 0.0)
            if elapsed >= self.p["finish_time"]:
                self._switch(Phase.LIE_DOWN_IN_CIRCLE)
            return StageStatus.RUNNING

        if self.phase == Phase.LIE_DOWN_IN_CIRCLE:
            self.ctx.dog.lie_down(hold=0.0)
            if elapsed >= self.p["lie_down_time"]:
                self._switch(Phase.DONE)
            return StageStatus.RUNNING

        self.ctx.dog.stop()
        return StageStatus.SUCCEEDED

    def on_exit(self) -> None:
        self.ctx.dog.stop()
