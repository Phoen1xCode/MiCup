from perception.vision_utils import (
    HsvRange,
    bbox_from_mask,
    bearing_from_center,
    detect_dashed_line_hsv,
    detect_lane_edges_hsv,
    estimate_distance_by_width,
    in_hsv_range,
)


def test_hsv_range_accepts_pixel_inside_bounds():
    orange = HsvRange(lower=(5, 80, 80), upper=(25, 255, 255))
    assert in_hsv_range((12, 120, 160), orange)
    assert not in_hsv_range((40, 120, 160), orange)


def test_bbox_from_mask_returns_enclosing_box():
    mask = [
        [False, False, False, False],
        [False, True, True, False],
        [False, True, False, False],
    ]
    assert bbox_from_mask(mask) == (1, 1, 2, 2)


def test_bearing_sign_matches_image_side():
    assert bearing_from_center(center_x=80.0, image_width=100, horizontal_fov_rad=1.0) > 0.0
    assert bearing_from_center(center_x=20.0, image_width=100, horizontal_fov_rad=1.0) < 0.0


def test_distance_estimate_decreases_with_larger_width():
    far = estimate_distance_by_width(real_width_m=0.2, pixel_width=20, focal_px=400)
    near = estimate_distance_by_width(real_width_m=0.2, pixel_width=80, focal_px=400)
    assert near < far


def _blank_hsv(width=100, height=80):
    return [[(0, 0, 0) for _ in range(width)] for _ in range(height)]


def _paint_rect(image, x0, y0, x1, y1, pixel):
    for y in range(y0, y1):
        for x in range(x0, x1):
            image[y][x] = pixel


def test_detect_lane_edges_returns_centered_offsets():
    image = _blank_hsv()
    yellow = (30, 180, 220)
    _paint_rect(image, 16, 36, 20, 80, yellow)
    _paint_rect(image, 80, 36, 84, 80, yellow)

    result = detect_lane_edges_hsv(image, [HsvRange((22, 80, 80), (38, 255, 255))])

    assert result["confidence"] > 0.5
    assert result["left_offset_px"] < 0.0
    assert result["right_offset_px"] > 0.0
    assert abs(result["center_offset_px"]) <= 1.0


def test_detect_lane_edges_reports_near_horizontal_turn_line():
    image = _blank_hsv()
    yellow = (30, 180, 220)
    _paint_rect(image, 12, 36, 16, 80, yellow)
    _paint_rect(image, 12, 66, 88, 72, yellow)

    result = detect_lane_edges_hsv(image, [HsvRange((22, 80, 80), (38, 255, 255))])

    assert result["horizontal_confidence"] >= 0.5
    assert result["turn_hint"] in ("left", "right", "")


def test_detect_dashed_line_returns_near_white_line():
    image = _blank_hsv()
    white = (0, 0, 230)
    _paint_rect(image, 35, 60, 65, 64, white)

    result = detect_dashed_line_hsv(image, [HsvRange((0, 0, 190), (180, 60, 255))])

    assert result is not None
    assert result["confidence"] > 0.0
    assert result["center_px"][1] >= 60.0
