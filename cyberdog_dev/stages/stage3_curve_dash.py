"""Stage 3 · 曲道冲锋."""

import time
from enum import Enum, auto
from pathlib import Path

from core.lane_follow import LaneFollowParams, VisualLaneFollowParams, compute_visual_lane_follow_correction
from core.nav import lane_follow_pd
from core.framework.stage import Stage, StageStatus
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

    def _visual_lane_available(self, lane_edges) -> bool:
        return lane_edges.confidence >= self.p["visual_lane_confidence"]

    def _follow_visual_lane(self, lane_edges) -> None:
        params = VisualLaneFollowParams(
            forward_speed=float(self.p["forward_speed"]),
            lateral_gain=float(self.p["visual_lateral_gain"]),
            max_lateral=float(self.p["visual_max_lateral"]),
            min_confidence=float(self.p["visual_lane_confidence"]),
        )
        vx, vy, wz = compute_visual_lane_follow_correction(lane_edges, params)
        self.ctx.dog.set_velocity_command(vx, vy, wz)

    def tick(self) -> StageStatus:
        elapsed = time.monotonic() - self.phase_start
        corridor = self.ctx.perception.latest_lidar_corridor()
        lane_edges = self.ctx.perception.latest_lane_edges()
        dashed_line = self.ctx.perception.latest_dashed_line()

        if self.phase == Phase.ENTER:
            self._switch(Phase.FOLLOW_CORRIDOR)
            return StageStatus.RUNNING

        if self.phase == Phase.FOLLOW_CORRIDOR:
            if self._visual_lane_available(lane_edges):
                self._follow_visual_lane(lane_edges)
            else:
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
            exit_by_dashed = (
                elapsed >= self.p["min_follow_time"]
                and dashed_line is not None
                and dashed_line.confidence >= self.p["visual_exit_confidence"]
            )
            if exit_by_dashed or exit_visible or elapsed >= self.p["max_follow_time"]:
                self._switch(Phase.STRAIGHT_TO_EXIT)
            return StageStatus.RUNNING

        if self.phase == Phase.STRAIGHT_TO_EXIT:
            self.ctx.dog.set_velocity_command(self.p["exit_speed"], 0.0, 0.0)
            if elapsed >= self.p["exit_time"]:
                self._switch(Phase.DONE)
            return StageStatus.RUNNING

        self.ctx.dog.set_velocity_command(0.0, 0.0, 0.0)
        return StageStatus.SUCCEEDED

    def on_exit(self) -> None:
        self.ctx.dog.set_velocity_command(0.0, 0.0, 0.0)
