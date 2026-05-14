from perception.vision_utils import (
    HsvRange,
    bbox_from_mask,
    bearing_from_center,
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
