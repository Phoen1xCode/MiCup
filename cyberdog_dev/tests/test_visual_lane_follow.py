from core.lane_follow import (
    VisualLaneFollowParams,
    compute_visual_lane_follow_correction,
)
from perception.hub import LaneEdges


PARAMS = VisualLaneFollowParams(
    forward_speed=0.2,
    lateral_gain=0.002,
    max_lateral=0.08,
    min_confidence=0.4,
)


def test_visual_lane_centered_goes_straight():
    edges = LaneEdges(
        left_offset_px=-30.0,
        right_offset_px=30.0,
        center_offset_px=0.0,
        confidence=0.9,
    )
    vx, vy, wz = compute_visual_lane_follow_correction(edges, PARAMS)
    assert vx == 0.2
    assert vy == 0.0
    assert wz == 0.0


def test_visual_lane_shifted_right_strafes_right():
    edges = LaneEdges(
        left_offset_px=-20.0,
        right_offset_px=60.0,
        center_offset_px=20.0,
        confidence=0.9,
    )
    _, vy, _ = compute_visual_lane_follow_correction(edges, PARAMS)
    assert vy < 0.0


def test_visual_lane_low_confidence_falls_back_to_straight():
    edges = LaneEdges(center_offset_px=50.0, confidence=0.1)
    vx, vy, wz = compute_visual_lane_follow_correction(edges, PARAMS)
    assert vx == 0.2
    assert vy == 0.0
    assert wz == 0.0


def test_visual_lane_lateral_correction_is_clamped():
    edges = LaneEdges(center_offset_px=200.0, confidence=0.9)
    _, vy, _ = compute_visual_lane_follow_correction(edges, PARAMS)
    assert abs(vy) == PARAMS.max_lateral
