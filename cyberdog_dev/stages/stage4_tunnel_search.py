"""Stage 4 · 深隧寻珍."""

import time
from enum import Enum, auto
from pathlib import Path

from core.gaits.low_walk import execute_low_walk
from core.stage_base import Stage, StageStatus
from config.loader import load_stage_params


class Phase(Enum):
    ENTER = auto()
    SCAN_LANE = auto()
    INTERACT = auto()
    SHIFT_LANE = auto()
    GO_BRIDGE = auto()
    DONE = auto()


PHRASES = {
    "red_pole": "识别到限高杆",
    "block_obstacle": "识别到无法跨越障碍",
    "coke": "识别到可乐瓶",
    "orange_ball": "识别到橙色小球",
    "football": "识别到足球",
}


class Stage4TunnelSearch(Stage):
    stage_id = 4
    name = "深隧寻珍"
    required_targets = {"coke", "orange_ball", "football"}
    required_obstacles = {"red_pole", "block_obstacle"}

    def __init__(self, ctx):
        super().__init__(ctx)
        config_dir = Path(__file__).resolve().parent.parent / "config"
        self.p = load_stage_params(config_dir / "stage_params.toml", stage_id=4)
        self.phase = Phase.ENTER
        self.phase_start = 0.0
        self.lane_index = 0
        self.current_kind = None
        self.announced: set[str] = set()
        self.handled_targets: set[str] = set()
        self.handled_obstacles: set[str] = set()

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

    def _complete(self) -> bool:
        return (
            self.required_targets <= self.handled_targets
            and self.required_obstacles <= self.handled_obstacles
        )

    def _announce(self, kind: str) -> None:
        if kind in self.announced:
            return
        self.ctx.voice.say(PHRASES[kind])
        self.announced.add(kind)

    def _select_candidate(self):
        if self.ctx.perception.latest_red_poles() and "red_pole" not in self.handled_obstacles:
            return "red_pole", self.ctx.perception.latest_red_poles()[0]
        if (
            self.ctx.perception.latest_block_obstacles()
            and "block_obstacle" not in self.handled_obstacles
        ):
            return "block_obstacle", self.ctx.perception.latest_block_obstacles()[0]
        if self.ctx.perception.latest_coke_bottles() and "coke" not in self.handled_targets:
            return "coke", self.ctx.perception.latest_coke_bottles()[0]
        if self.ctx.perception.latest_orange_balls() and "orange_ball" not in self.handled_targets:
            return "orange_ball", self.ctx.perception.latest_orange_balls()[0]
        football = self.ctx.perception.latest_football()
        if football is not None and "football" not in self.handled_targets:
            return "football", football
        return None, None

    def _handle_current(self, elapsed: float) -> None:
        kind = self.current_kind
        if kind == "red_pole":
            execute_low_walk(self.ctx.dog, self.p["low_walk_time"], self.p["low_walk_speed"])
            self.handled_obstacles.add(kind)
            self._switch(Phase.SCAN_LANE)
            return

        if kind == "block_obstacle":
            self.ctx.dog.set_velocity(self.p["detour_speed"], self.p["lane_shift_speed"], 0.0)
            if elapsed >= self.p["detour_time"]:
                self.handled_obstacles.add(kind)
                self._switch(Phase.SCAN_LANE)
            return

        if kind == "football":
            self.ctx.dog.set_velocity(self.p["kick_speed"], 0.0, 0.0)
            if elapsed >= self.p["kick_time"]:
                self.handled_targets.add(kind)
                self._switch(Phase.SCAN_LANE)
            return

        self.ctx.dog.set_velocity(self.p["bump_speed"], 0.0, 0.0)
        if elapsed >= self.p["bump_time"]:
            self.handled_targets.add(kind)
            self._switch(Phase.SCAN_LANE)

    def tick(self) -> StageStatus:
        elapsed = time.monotonic() - self.phase_start

        if self.phase == Phase.ENTER:
            self._switch(Phase.SCAN_LANE)
            return StageStatus.RUNNING

        if self.phase == Phase.SCAN_LANE:
            if self._complete():
                self._switch(Phase.GO_BRIDGE)
                return StageStatus.RUNNING

            kind, _ = self._select_candidate()
            if kind is not None:
                self.current_kind = kind
                self._announce(kind)
                self._switch(Phase.INTERACT)
                return StageStatus.RUNNING

            self.ctx.dog.set_velocity(self.p["approach_speed"], 0.0, 0.0)
            if elapsed >= self.p["lane_scan_time"]:
                self._switch(Phase.SHIFT_LANE)
            return StageStatus.RUNNING

        if self.phase == Phase.INTERACT:
            self._handle_current(elapsed)
            return StageStatus.RUNNING

        if self.phase == Phase.SHIFT_LANE:
            self.ctx.dog.set_velocity(0.0, self.p["lane_shift_speed"], 0.0)
            if elapsed >= self.p["lane_shift_time"]:
                self.lane_index = min(self.lane_index + 1, self.p["lane_count"] - 1)
                self._switch(Phase.SCAN_LANE)
            return StageStatus.RUNNING

        if self.phase == Phase.GO_BRIDGE:
            self.ctx.dog.set_velocity(self.p["bridge_speed"], 0.0, 0.0)
            if elapsed >= self.p["bridge_time"]:
                self._switch(Phase.DONE)
            return StageStatus.RUNNING

        self.ctx.dog.stop()
        return StageStatus.SUCCEEDED

    def on_exit(self) -> None:
        self.ctx.dog.stop()
