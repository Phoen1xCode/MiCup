"""走廊居中 PD 计算（纯函数，无机器人/ROS 依赖，可单测）。"""

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class LaneFollowParams:
    forward_speed: float = 0.2
    lateral_gain: float = 0.12
    max_lateral: float = 0.08
    front_stop_distance: float = 0.45


@dataclass(frozen=True)
class VisualLaneFollowParams:
    forward_speed: float = 0.2
    lateral_gain: float = 0.002
    max_lateral: float = 0.08
    min_confidence: float = 0.4


def compute_lane_follow_correction(left: float, right: float, front: float,
                                   params: LaneFollowParams) -> tuple:
    """根据左/前/右三个距离算出 (vx, vy, wz) 速度指令。

    约定：vy 正方向为左。left/right/front 为米；无返回用 float('inf')。
    """
    vx = params.forward_speed
    if math.isfinite(front) and front < params.front_stop_distance:
        vx = 0.0

    vy = 0.0
    if math.isfinite(left) and math.isfinite(right):
        raw = (left - right) * params.lateral_gain
        vy = max(-params.max_lateral, min(params.max_lateral, raw))

    wz = 0.0
    return (vx, vy, wz)


def compute_visual_lane_follow_correction(edges, params: VisualLaneFollowParams) -> tuple:
    """根据视觉黄线中心偏差算 (vx, vy, wz)。

    约定：center_offset_px 正值表示赛道中心在图像右侧，机器人应向右移，
    而控制接口 vy 正方向为左，因此 vy 取负。
    """
    vx = params.forward_speed
    if edges.confidence < params.min_confidence:
        return (vx, 0.0, 0.0)

    raw_vy = -float(edges.center_offset_px) * params.lateral_gain
    vy = max(-params.max_lateral, min(params.max_lateral, raw_vy))
    if abs(vy) < 1e-9:
        vy = 0.0
    return (vx, vy, 0.0)
