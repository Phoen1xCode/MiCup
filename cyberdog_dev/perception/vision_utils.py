"""Pure vision helpers shared by lightweight detector nodes."""

import math
from dataclasses import dataclass
from typing import Iterable, Optional, Sequence


@dataclass(frozen=True)
class HsvRange:
    lower: tuple[int, int, int]
    upper: tuple[int, int, int]


def in_hsv_range(pixel: Sequence[int], hsv_range: HsvRange) -> bool:
    return all(lo <= int(value) <= hi for value, lo, hi in zip(pixel, hsv_range.lower, hsv_range.upper))


def bbox_from_mask(mask: Sequence[Sequence[bool]]) -> Optional[tuple[int, int, int, int]]:
    xs: list[int] = []
    ys: list[int] = []
    for y, row in enumerate(mask):
        for x, value in enumerate(row):
            if value:
                xs.append(x)
                ys.append(y)
    if not xs:
        return None
    min_x, max_x = min(xs), max(xs)
    min_y, max_y = min(ys), max(ys)
    return (min_x, min_y, max_x - min_x + 1, max_y - min_y + 1)


def bearing_from_center(center_x: float, image_width: int, horizontal_fov_rad: float = 1.047) -> float:
    if image_width <= 0:
        return 0.0
    normalized = (center_x - (image_width / 2.0)) / (image_width / 2.0)
    return normalized * (horizontal_fov_rad / 2.0)


def estimate_distance_by_width(real_width_m: float, pixel_width: float, focal_px: float = 420.0) -> float:
    if pixel_width <= 0:
        return math.inf
    return (real_width_m * focal_px) / pixel_width


def det_from_bbox(label: str, bbox: tuple[int, int, int, int], image_width: int,
                  real_width_m: float = 0.2, confidence: float = 0.5) -> dict:
    x, y, w, h = bbox
    center = (x + w / 2.0, y + h / 2.0)
    return {
        "label": label,
        "bbox": [x, y, w, h],
        "center_px": [center[0], center[1]],
        "area_px": float(w * h),
        "bearing_rad": bearing_from_center(center[0], image_width),
        "distance_m": estimate_distance_by_width(real_width_m, w),
        "confidence": confidence,
    }


def detect_colored_objects_hsv(image_hsv: Sequence[Sequence[Sequence[int]]],
                               hsv_ranges: Iterable[HsvRange],
                               label: str,
                               min_area_px: float = 40.0,
                               real_width_m: float = 0.2) -> list[dict]:
    mask = []
    ranges = list(hsv_ranges)
    for row in image_hsv:
        mask.append([any(in_hsv_range(pixel, r) for r in ranges) for pixel in row])
    bbox = bbox_from_mask(mask)
    if bbox is None:
        return []
    _, _, w, h = bbox
    if w * h < min_area_px:
        return []
    image_width = len(image_hsv[0]) if image_hsv and image_hsv[0] else 0
    return [det_from_bbox(label, bbox, image_width=image_width, real_width_m=real_width_m, confidence=0.6)]
