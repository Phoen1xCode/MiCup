"""PerceptionHub -- Stage 获取检测结果的唯一入口（spec 3.5）。"""

from __future__ import annotations

import json
import threading
from dataclasses import dataclass
from typing import Optional

NO_RETURN = 99.9


@dataclass
class ObjDet:
    label: str
    bbox: tuple[int, int, int, int]
    center_px: tuple[float, float]
    area_px: float
    bearing_rad: float = 0.0
    distance_m: float = NO_RETURN
    confidence: float = 0.0


@dataclass
class BallDet:
    bbox: tuple[int, int, int, int]
    center_px: tuple[float, float]
    area_px: float
    bearing_rad: float
    distance_m: float
    confidence: float


@dataclass
class PoleDet:
    bbox: tuple[int, int, int, int]
    center_px: tuple[float, float]
    area_px: float
    bearing_rad: float
    confidence: float

    @property
    def aspect_ratio(self) -> float:
        _, _, w, h = self.bbox
        return h / max(1, w)


@dataclass
class CorridorState:
    """左/前/右三方向距离（米）。无返回用 NO_RETURN(99.9) 表示。"""
    left: float = NO_RETURN
    front: float = NO_RETURN
    right: float = NO_RETURN


@dataclass
class LaneEdges:
    left_offset_px: float = 0.0
    right_offset_px: float = 0.0
    center_offset_px: float = 0.0
    left_confidence: float = 0.0
    right_confidence: float = 0.0
    horizontal_confidence: float = 0.0
    turn_hint: str = ""
    confidence: float = 0.0


@dataclass
class DashedLineDet:
    center_px: tuple[float, float]
    confidence: float


@dataclass
class SlopeState:
    """LiDAR 坡道检测结果。"""
    detected: bool = False
    angle_deg: float = 0.0
    distance_m: float = NO_RETURN
    midpoint_m: float = NO_RETURN
    length_m: float = 0.0
    status: str = ""


class PerceptionHub:
    """聚合所有检测器，提供同步的"取最新结果"接口。"""

    def __init__(self):
        self._lock = threading.Lock()
        self._corridor = CorridorState()
        self._orange_balls: list[BallDet] = []
        self._footballs: list[BallDet] = []
        self._coke_bottles: list[ObjDet] = []
        self._red_poles: list[PoleDet] = []
        self._block_obstacles: list[ObjDet] = []
        self._lane_edges = LaneEdges()
        self._dashed_line: Optional[DashedLineDet] = None
        self._slope = SlopeState()
        self._node = None

    def attach_node(self, node):
        """绑定一个已创建的 rclpy Node，订阅感知 topic。"""
        from geometry_msgs.msg import Vector3
        from std_msgs.msg import String

        self._node = node
        node.create_subscription(
            Vector3, "/perception/lidar_corridor", self._on_corridor, 10)
        node.create_subscription(
            String, "/perception/orange_ball", self._on_orange_balls, 10)
        node.create_subscription(
            String, "/perception/football", self._on_footballs, 10)
        node.create_subscription(
            String, "/perception/coke", self._on_coke_bottles, 10)
        node.create_subscription(
            String, "/perception/red_pole", self._on_red_poles, 10)
        node.create_subscription(
            String, "/perception/block_obstacle", self._on_block_obstacles, 10)
        node.create_subscription(
            String, "/perception/lane_edge", self._on_lane_edges, 10)
        node.create_subscription(
            String, "/perception/dashed_line", self._on_dashed_line, 10)
        node.create_subscription(
            String, "/perception/slope", self._on_slope, 10)

    def _on_corridor(self, msg):
        with self._lock:
            self._corridor = CorridorState(left=msg.x, front=msg.y, right=msg.z)

    def _on_orange_balls(self, msg):
        with self._lock:
            self._orange_balls = [_ball_from_dict(item) for item in _load_list(msg.data)]

    def _on_footballs(self, msg):
        with self._lock:
            self._footballs = [_ball_from_dict(item) for item in _load_list(msg.data)]

    def _on_coke_bottles(self, msg):
        with self._lock:
            self._coke_bottles = [_obj_from_dict("coke", item) for item in _load_list(msg.data)]

    def _on_red_poles(self, msg):
        with self._lock:
            self._red_poles = [_pole_from_dict(item) for item in _load_list(msg.data)]

    def _on_block_obstacles(self, msg):
        with self._lock:
            self._block_obstacles = [_obj_from_dict("block", item) for item in _load_list(msg.data)]

    def _on_lane_edges(self, msg):
        data = _load_dict(msg.data)
        with self._lock:
            self._lane_edges = LaneEdges(
                left_offset_px=float(data.get("left_offset_px", 0.0)),
                right_offset_px=float(data.get("right_offset_px", 0.0)),
                center_offset_px=float(data.get("center_offset_px", 0.0)),
                left_confidence=float(data.get("left_confidence", 0.0)),
                right_confidence=float(data.get("right_confidence", 0.0)),
                horizontal_confidence=float(data.get("horizontal_confidence", 0.0)),
                turn_hint=str(data.get("turn_hint", "")),
                confidence=float(data.get("confidence", 0.0)),
            )

    def _on_dashed_line(self, msg):
        data = _load_dict(msg.data)
        with self._lock:
            if not data:
                self._dashed_line = None
            else:
                self._dashed_line = DashedLineDet(
                    center_px=_center(data), confidence=float(data.get("confidence", 0.0)))

    def _on_slope(self, msg):
        data = _load_dict(msg.data)
        with self._lock:
            self._slope = SlopeState(
                detected=bool(data.get("detected", False)),
                angle_deg=float(data.get("angle_deg", 0.0) or 0.0),
                distance_m=float(data.get("distance_m", NO_RETURN) or NO_RETURN),
                midpoint_m=float(data.get("midpoint_m", NO_RETURN) or NO_RETURN),
                length_m=float(data.get("length_m", 0.0) or 0.0),
                status=str(data.get("status", "")),
            )

    def latest_lidar_corridor(self) -> CorridorState:
        """返回最近一帧走廊状态。无数据时返回默认（全 NO_RETURN）。"""
        with self._lock:
            return self._corridor

    def latest_orange_balls(self, *, max_age_sec: float = 0.5) -> list[BallDet]:
        with self._lock:
            return list(self._orange_balls)

    def latest_footballs(self) -> list[BallDet]:
        with self._lock:
            return list(self._footballs)

    def latest_football(self) -> Optional[BallDet]:
        with self._lock:
            return self._footballs[0] if self._footballs else None

    def latest_coke_bottles(self) -> list[ObjDet]:
        with self._lock:
            return list(self._coke_bottles)

    def latest_red_poles(self) -> list[PoleDet]:
        with self._lock:
            return list(self._red_poles)

    def latest_block_obstacles(self) -> list[ObjDet]:
        with self._lock:
            return list(self._block_obstacles)

    def latest_lane_edges(self) -> LaneEdges:
        with self._lock:
            return self._lane_edges

    def latest_dashed_line(self) -> Optional[DashedLineDet]:
        with self._lock:
            return self._dashed_line

    def latest_slope(self) -> SlopeState:
        with self._lock:
            return self._slope


def _load_list(raw: str) -> list[dict]:
    data = json.loads(raw or "[]")
    return data if isinstance(data, list) else []


def _load_dict(raw: str) -> dict:
    data = json.loads(raw or "{}")
    return data if isinstance(data, dict) else {}


def _bbox(data: dict) -> tuple[int, int, int, int]:
    return tuple(int(v) for v in data.get("bbox", (0, 0, 0, 0)))


def _center(data: dict) -> tuple[float, float]:
    return tuple(float(v) for v in data.get("center_px", (0.0, 0.0)))


def _ball_from_dict(data: dict) -> BallDet:
    return BallDet(
        bbox=_bbox(data),
        center_px=_center(data),
        area_px=float(data.get("area_px", 0.0)),
        bearing_rad=float(data.get("bearing_rad", 0.0)),
        distance_m=float(data.get("distance_m", NO_RETURN)),
        confidence=float(data.get("confidence", 0.0)),
    )


def _obj_from_dict(label: str, data: dict) -> ObjDet:
    return ObjDet(
        label=str(data.get("label", label)),
        bbox=_bbox(data),
        center_px=_center(data),
        area_px=float(data.get("area_px", 0.0)),
        bearing_rad=float(data.get("bearing_rad", 0.0)),
        distance_m=float(data.get("distance_m", NO_RETURN)),
        confidence=float(data.get("confidence", 0.0)),
    )


def _pole_from_dict(data: dict) -> PoleDet:
    return PoleDet(
        bbox=_bbox(data),
        center_px=_center(data),
        area_px=float(data.get("area_px", 0.0)),
        bearing_rad=float(data.get("bearing_rad", 0.0)),
        confidence=float(data.get("confidence", 0.0)),
    )
