from core._lane_follow import compute_lane_follow_correction, LaneFollowParams


PARAMS = LaneFollowParams(
    forward_speed=0.2,
    lateral_gain=0.12,
    max_lateral=0.08,
    front_stop_distance=0.45,
)


def test_centered_goes_straight():
    # 左右等距 -> 不横移，全速前进
    vx, vy, wz = compute_lane_follow_correction(left=1.0, right=1.0, front=2.0, params=PARAMS)
    assert vx == 0.2
    assert vy == 0.0
    assert wz == 0.0


def test_closer_to_left_strafes_right():
    # 左边更近(left<right) -> 向右横移(vy 为负)
    vx, vy, wz = compute_lane_follow_correction(left=0.5, right=1.5, front=2.0, params=PARAMS)
    assert vx == 0.2
    assert vy < 0.0


def test_closer_to_right_strafes_left():
    vx, vy, wz = compute_lane_follow_correction(left=1.5, right=0.5, front=2.0, params=PARAMS)
    assert vy > 0.0


def test_lateral_correction_is_clamped():
    # 极端偏差 -> 横移速度被钳在 max_lateral
    vx, vy, wz = compute_lane_follow_correction(left=0.1, right=5.0, front=2.0, params=PARAMS)
    assert abs(vy) <= PARAMS.max_lateral


def test_front_blocked_stops_forward():
    # 前方近于 front_stop_distance -> 停止前进
    vx, vy, wz = compute_lane_follow_correction(left=1.0, right=1.0, front=0.3, params=PARAMS)
    assert vx == 0.0


def test_infinite_side_reading_falls_back_to_straight():
    # 一侧无返回(inf) -> 不做横向修正，仅前进
    vx, vy, wz = compute_lane_follow_correction(left=float("inf"), right=1.0, front=2.0, params=PARAMS)
    assert vy == 0.0
    assert vx == 0.2
