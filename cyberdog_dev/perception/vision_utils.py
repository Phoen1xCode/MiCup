"""Pure vision helpers shared by lightweight detector nodes."""

from __future__ import annotations

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


def _shape(image_hsv: Sequence[Sequence[Sequence[int]]]) -> tuple[int, int]:
    height = len(image_hsv)
    width = len(image_hsv[0]) if height and image_hsv[0] is not None else 0
    return height, width


def _mask_points(image_hsv: Sequence[Sequence[Sequence[int]]],
                 hsv_ranges: Iterable[HsvRange],
                 *,
                 y_min_ratio: float = 0.0) -> list[tuple[int, int]]:
    ranges = list(hsv_ranges)
    height, width = _shape(image_hsv)
    y_min = int(height * y_min_ratio)
    points: list[tuple[int, int]] = []
    for y, row in enumerate(image_hsv):
        if y < y_min:
            continue
        for x, pixel in enumerate(row):
            if any(in_hsv_range(pixel, r) for r in ranges):
                points.append((x, y))
    return points


def detect_lane_edges_hsv(image_hsv: Sequence[Sequence[Sequence[int]]],
                          hsv_ranges: Iterable[HsvRange]) -> dict:
    """检测赛道黄边线，返回 PerceptionHub.LaneEdges 可解析的 dict。

    算法故意保持简单：只看图像下半部分的黄色像素，按左右半区估计边线
    中心，并用近场横向黄线作为转弯提示。Gazebo 调参时主要调 HSV 与
    Stage 阈值，不在这里引入复杂拟合。
    """
    height, width = _shape(image_hsv)
    if width <= 0 or height <= 0:
        return {
            "left_offset_px": 0.0,
            "right_offset_px": 0.0,
            "center_offset_px": 0.0,
            "left_confidence": 0.0,
            "right_confidence": 0.0,
            "horizontal_confidence": 0.0,
            "turn_hint": "",
            "confidence": 0.0,
        }

    points = _mask_points(image_hsv, hsv_ranges, y_min_ratio=0.45)
    center_x = width / 2.0
    left_xs = [x for x, _ in points if x < center_x]
    right_xs = [x for x, _ in points if x >= center_x]
    min_side_pixels = max(8, int(height * width * 0.002))

    left_conf = min(1.0, len(left_xs) / float(min_side_pixels))
    right_conf = min(1.0, len(right_xs) / float(min_side_pixels))

    left_offset = 0.0
    right_offset = 0.0
    if left_xs:
        left_offset = (sum(left_xs) / len(left_xs)) - center_x
    if right_xs:
        right_offset = (sum(right_xs) / len(right_xs)) - center_x

    if left_xs and right_xs:
        lane_center = ((sum(left_xs) / len(left_xs)) + (sum(right_xs) / len(right_xs))) / 2.0
        confidence = min(left_conf, right_conf)
    elif left_xs:
        # 单边线兜底：赛道宽度未知时保持半幅画面余量，避免压线。
        lane_center = (sum(left_xs) / len(left_xs)) + width * 0.32
        confidence = left_conf * 0.6
    elif right_xs:
        lane_center = (sum(right_xs) / len(right_xs)) - width * 0.32
        confidence = right_conf * 0.6
    else:
        lane_center = center_x
        confidence = 0.0

    bottom_y = int(height * 0.72)
    near_points = [(x, y) for x, y in points if y >= bottom_y]
    row_counts: dict[int, int] = {}
    for _, y in near_points:
        row_counts[y] = row_counts.get(y, 0) + 1
    horizontal_pixels = max(row_counts.values()) if row_counts else 0
    horizontal_conf = min(1.0, horizontal_pixels / max(1.0, width * 0.45))

    turn_hint = ""
    if horizontal_conf >= 0.5:
        left_near = sum(1 for x, _ in near_points if x < center_x)
        right_near = len(near_points) - left_near
        if left_near > right_near * 1.2:
            turn_hint = "left"
        elif right_near > left_near * 1.2:
            turn_hint = "right"

    return {
        "left_offset_px": float(left_offset),
        "right_offset_px": float(right_offset),
        "center_offset_px": float(lane_center - center_x),
        "left_confidence": float(left_conf),
        "right_confidence": float(right_conf),
        "horizontal_confidence": float(horizontal_conf),
        "turn_hint": turn_hint,
        "confidence": float(confidence),
    }


def detect_dashed_line_hsv(image_hsv: Sequence[Sequence[Sequence[int]]],
                           hsv_ranges: Iterable[HsvRange]) -> Optional[dict]:
    """检测近场白色虚线/横线，返回 DashedLineDet 可解析的 dict。"""
    height, width = _shape(image_hsv)
    if width <= 0 or height <= 0:
        return None

    points = _mask_points(image_hsv, hsv_ranges, y_min_ratio=0.45)
    if not points:
        return None

    min_pixels = max(6, int(width * 0.12))
    if len(points) < min_pixels:
        return None

    xs = [x for x, _ in points]
    ys = [y for _, y in points]
    bbox_w = max(xs) - min(xs) + 1
    bbox_h = max(ys) - min(ys) + 1
    if bbox_w < width * 0.15 or bbox_w < bbox_h * 2.0:
        return None

    return {
        "center_px": [float(sum(xs) / len(xs)), float(sum(ys) / len(ys))],
        "confidence": min(1.0, len(points) / float(max(1, width * 4))),
    }
