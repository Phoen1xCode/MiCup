"""纯逻辑单元测试：RANSAC 回归器和线段筛选。"""

from __future__ import annotations

import math

import numpy as np

from perception.slope import SlopeDetector, custom_ransac_regressor


# ---------------------------------------------------------------------------
# custom_ransac_regressor
# ---------------------------------------------------------------------------

def test_ransac_returns_correct_slope_for_known_line():
    rng = np.random.default_rng(42)
    x = np.linspace(0, 5, 50)
    y = 2.0 * x + 1.0 + rng.normal(0, 0.01, len(x))
    slope, intercept, mask = custom_ransac_regressor(x, y, threshold=0.05)
    assert slope is not None
    assert abs(slope - 2.0) < 0.1
    assert abs(intercept - 1.0) < 0.2


def test_ransac_returns_none_for_too_few_points():
    x = np.array([1.0])
    y = np.array([2.0])
    slope, intercept, mask = custom_ransac_regressor(x, y)
    assert slope is None
    assert intercept is None
    assert mask is None


def test_ransac_handles_noisy_data_with_outliers():
    rng = np.random.default_rng(99)
    x = np.linspace(0, 3, 40)
    y = 0.5 * x + 0.0 + rng.normal(0, 0.02, len(x))
    x_out = np.array([0.5, 1.5, 2.5])
    y_out = np.array([10.0, -10.0, 8.0])
    x_all = np.concatenate([x, x_out])
    y_all = np.concatenate([y, y_out])
    slope, intercept, mask = custom_ransac_regressor(x_all, y_all, threshold=0.1)
    assert slope is not None
    assert abs(slope - 0.5) < 0.15


def test_ransac_returns_none_when_inliers_below_threshold():
    rng = np.random.default_rng(7)
    x = rng.uniform(0, 10, 20)
    y = rng.uniform(0, 10, 20)
    slope, intercept, mask = custom_ransac_regressor(
        x, y, threshold=0.001, min_inliers_for_valid_model=15)
    assert slope is None


# ---------------------------------------------------------------------------
# SlopeDetector.find_best_line_by_criteria
# ---------------------------------------------------------------------------


def _make_detector(**overrides):
    params = {
        "forward_angle_range_deg": 60.0,
        "min_points_for_fit": 10,
        "ransac_threshold_m": 0.03,
        "smoothing_queue_size": 3,
        "max_distance_std_dev": 0.15,
        "max_angle_std_dev_deg": 5.0,
        "target_length_m": 1.0,
        "length_tie_tolerance_m": 0.1,
    }
    params.update(overrides)
    return SlopeDetector(params)


def test_find_best_line_selects_closest_to_target_length():
    det = _make_detector(target_length_m=1.0, min_points_for_fit=5, ransac_threshold_m=0.05)
    rng = np.random.default_rng(42)
    # 短线段：沿 y 方向长度 ~0.3
    y_short = np.linspace(0, 0.3, 30) + rng.normal(0, 0.002, 30)
    x_short = np.full(30, 5.0) + rng.normal(0, 0.002, 30)
    # 接近 1m 的线段：沿 y 方向长度 ~1.0
    y_good = np.linspace(0, 1.0, 30) + rng.normal(0, 0.002, 30)
    x_good = np.full(30, 0.0) + rng.normal(0, 0.002, 30)
    x_all = np.concatenate([x_short, x_good])
    y_all = np.concatenate([y_short, y_good])
    best, candidates = det.find_best_line_by_criteria(x_all, y_all)
    assert best is not None
    assert len(candidates) >= 2
    # 选中的线段应比短线段更接近目标 1.0m
    assert best["length"] > 0.5


def test_find_best_line_returns_none_for_empty_input():
    det = _make_detector()
    best, candidates = det.find_best_line_by_criteria(np.array([]), np.array([]))
    assert best is None
    assert candidates == []


def test_find_best_line_tie_breaks_by_nearest_distance():
    det = _make_detector(target_length_m=1.0, length_tie_tolerance_m=0.5,
                         min_points_for_fit=5, ransac_threshold_m=0.05)
    rng = np.random.default_rng(11)
    # 近处线段：y 变化 ~1.0，x≈0（距离近）
    y_near = np.linspace(0, 1.0, 30) + rng.normal(0, 0.003, 30)
    x_near = np.full(30, 0.0) + rng.normal(0, 0.003, 30)
    # 远处线段：y 变化 ~1.0，x≈4（距离远）
    y_far = np.linspace(0, 1.0, 30) + rng.normal(0, 0.003, 30)
    x_far = np.full(30, 4.0) + rng.normal(0, 0.003, 30)
    x_all = np.concatenate([x_near, x_far])
    y_all = np.concatenate([y_near, y_far])
    best, candidates = det.find_best_line_by_criteria(x_all, y_all)
    assert best is not None
    assert len(candidates) >= 2
    # 近处线段距离更小
    assert best["distance"] < 3.0
