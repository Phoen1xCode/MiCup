"""激光雷达坡道检测 ROS2 节点。

订阅 LaserScan，使用 RANSAC 线段拟合检测前方坡道/斜面，
发布到 /perception/slope（JSON）。

独立运行：python3 -m perception.slope --mode sim
"""

from __future__ import annotations

import argparse
import json
import math
from collections import deque
from pathlib import Path

import numpy as np

from config.loader import load_topics, load_params_section


def custom_ransac_regressor(x_points, y_points, ransac_iterations=100,
                            threshold=0.05, min_inliers_for_valid_model=10):
    """纯 numpy RANSAC 线性回归。返回 (slope, intercept, inlier_mask) 或 (None, None, None)。"""
    best_inlier_mask = None
    max_inliers = 0
    n_points = len(x_points)
    if n_points < 2:
        return None, None, None

    for _ in range(ransac_iterations):
        sample_indices = np.random.choice(n_points, 2, replace=False)
        x_sample, y_sample = x_points[sample_indices], y_points[sample_indices]
        if abs(x_sample[1] - x_sample[0]) < 1e-9:
            continue
        m_temp = (y_sample[1] - y_sample[0]) / (x_sample[1] - x_sample[0])
        b_temp = y_sample[0] - m_temp * x_sample[0]
        distances = np.abs(y_points - (m_temp * x_points.flatten() + b_temp))
        current_inlier_mask = distances < threshold
        num_inliers = np.sum(current_inlier_mask)
        if num_inliers > max_inliers:
            max_inliers = num_inliers
            best_inlier_mask = current_inlier_mask

    if max_inliers >= min_inliers_for_valid_model and best_inlier_mask is not None:
        x_inliers = x_points[best_inlier_mask]
        y_inliers = y_points[best_inlier_mask]
        try:
            final_model_params = np.polyfit(x_inliers.flatten(), y_inliers.flatten(), 1)
            return final_model_params[0], final_model_params[1], best_inlier_mask
        except np.linalg.LinAlgError:
            return None, None, None
    return None, None, None


class SlopeDetector:
    """坡道检测纯算法，无 ROS 依赖。"""

    def __init__(self, params: dict):
        self.forward_angle_range_deg = float(params.get("forward_angle_range_deg", 60.0))
        self.min_points_for_fit = int(params.get("min_points_for_fit", 10))
        self.ransac_threshold = float(params.get("ransac_threshold_m", 0.03))
        self.max_dist_std = float(params.get("max_distance_std_dev", 0.15))
        self.max_angle_std_rad = math.radians(float(params.get("max_angle_std_dev_deg", 5.0)))
        self.target_length = float(params.get("target_length_m", 1.0))
        self.tie_tolerance = float(params.get("length_tie_tolerance_m", 0.1))
        queue_size = int(params.get("smoothing_queue_size", 3))
        self.distance_history: deque = deque(maxlen=queue_size)
        self.angle_history: deque = deque(maxlen=queue_size)

    def find_best_line_by_criteria(self, x_coords, y_coords):
        """迭代寻找所有线段，按长度接近目标值 + 距离最近规则筛选最佳线段。"""
        candidate_lines = []
        remaining_indices = np.arange(len(x_coords))

        while len(remaining_indices) >= self.min_points_for_fit:
            current_x = x_coords[remaining_indices]
            current_y = y_coords[remaining_indices]

            m, b, local_inlier_mask = custom_ransac_regressor(
                current_y.flatten(), current_x.flatten(), threshold=self.ransac_threshold,
                min_inliers_for_valid_model=self.min_points_for_fit,
            )

            if m is None:
                break

            inlier_indices_in_remaining = np.where(local_inlier_mask)[0]
            original_inlier_indices = remaining_indices[inlier_indices_in_remaining]

            inlier_y = y_coords[original_inlier_indices]

            y_min, y_max = np.min(inlier_y), np.max(inlier_y)
            x_at_ymin = m * y_min + b
            x_at_ymax = m * y_max + b
            length = math.sqrt((x_at_ymax - x_at_ymin) ** 2 + (y_max - y_min) ** 2)

            distance = abs(b) / math.sqrt(1 + m ** 2)

            candidate_lines.append({
                "m": m, "b": b, "length": length, "distance": distance,
                "inlier_indices": original_inlier_indices,
            })

            remaining_indices = np.delete(remaining_indices, inlier_indices_in_remaining)

        if not candidate_lines:
            return None, []

        for line in candidate_lines:
            line["score"] = abs(line["length"] - self.target_length)

        candidate_lines.sort(key=lambda x: x["score"])
        best_score = candidate_lines[0]["score"]

        tied_candidates = [
            line for line in candidate_lines
            if abs(line["score"] - best_score) < self.tie_tolerance
        ]

        if len(tied_candidates) > 1:
            tied_candidates.sort(key=lambda x: x["distance"])

        return tied_candidates[0], candidate_lines

    def process_scan(self, ranges, range_min, range_max, angle_min, angle_increment):
        """处理一帧 LaserScan 数据，返回结果字典。"""
        ranges = np.array(ranges, dtype=float)
        ranges[np.isinf(ranges) | (ranges < range_min) | (ranges > range_max)] = np.nan

        center_index = int((-angle_min) / angle_increment)
        angle_half_rad = math.radians(self.forward_angle_range_deg / 2.0)
        index_span = int(angle_half_rad / angle_increment)
        start_index = max(0, center_index - index_span)
        end_index = min(len(ranges) - 1, center_index + index_span)

        forward_indices = np.arange(start_index, end_index + 1)
        valid_indices = forward_indices[~np.isnan(ranges[forward_indices])]
        valid_ranges = ranges[valid_indices]
        angles = angle_min + valid_indices * angle_increment

        if len(valid_ranges) < self.min_points_for_fit:
            return {"detected": False, "status": f"总点数不足 (<{self.min_points_for_fit})"}

        x_coords = valid_ranges * np.cos(angles)
        y_coords = valid_ranges * np.sin(angles)

        best_line, _ = self.find_best_line_by_criteria(x_coords, y_coords)

        if best_line is None:
            return {"detected": False, "status": "未找到符合条件的直线"}

        raw_angle_rad = math.atan(best_line["m"])
        raw_distance = best_line["distance"]

        self.distance_history.append(raw_distance)
        self.angle_history.append(raw_angle_rad)

        if len(self.distance_history) < self.distance_history.maxlen:
            return {
                "detected": False,
                "status": f"push queue ({len(self.distance_history)}/{self.distance_history.maxlen})",
            }

        dist_std_dev = np.std(self.distance_history)
        angle_std_dev = np.std(self.angle_history)
        if dist_std_dev > self.max_dist_std or angle_std_dev > self.max_angle_std_rad:
            return {"detected": False, "status": "data unstable"}

        final_dist = float(np.mean(self.distance_history))
        final_angle_rad = float(np.mean(self.angle_history))
        final_angle_deg = math.degrees(final_angle_rad)

        inlier_x = x_coords[best_line["inlier_indices"]]
        inlier_y = y_coords[best_line["inlier_indices"]]
        midpoint_m = float(math.sqrt(np.mean(inlier_x) ** 2 + np.mean(inlier_y) ** 2))

        return {
            "detected": True,
            "angle_deg": final_angle_deg,
            "distance_m": final_dist,
            "midpoint_m": midpoint_m,
            "length_m": best_line["length"],
            "status": f"available (L={best_line['length']:.2f}m, D={best_line['distance']:.2f}m)",
        }


class SlopeDetectorNode:
    """ROS2 节点包装器，将 SlopeDetector 接入 LaserScan topic。"""

    def __init__(self, node, scan_topic: str, params: dict):
        from rclpy.qos import QoSProfile, QoSReliabilityPolicy
        from sensor_msgs.msg import LaserScan
        from std_msgs.msg import String

        self._node = node
        self._logger = node.get_logger()
        self._detector = SlopeDetector(params)

        qos = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        node.create_subscription(LaserScan, scan_topic, self._on_scan, qos)
        self.pub = node.create_publisher(String, "/perception/slope", 10)
        self._logger.info(f"坡道检测节点已启动，订阅: {scan_topic}")

    def _on_scan(self, msg):
        result = self._detector.process_scan(
            msg.ranges, msg.range_min, msg.range_max, msg.angle_min, msg.angle_increment)

        if not result["detected"]:
            self._logger.warn(f"坡道数据无效: {result['status']}")

        from std_msgs.msg import String
        out = String()
        out.data = json.dumps(result, ensure_ascii=False)
        self.pub.publish(out)


def main(args=None):
    import rclpy
    from rclpy.node import Node

    parser = argparse.ArgumentParser(description="Lidar slope detector")
    parser.add_argument("--mode", default="sim", choices=["sim", "real"])
    parsed, _ = parser.parse_known_args(args)

    config_dir = Path(__file__).resolve().parent.parent / "config"
    topics = load_topics(config_dir / "topics.toml", mode=parsed.mode)
    params = load_params_section(config_dir / "stage_params.toml", "slope")

    rclpy.init(args=args)
    node = Node("perception_slope")
    detector = SlopeDetectorNode(node, scan_topic=topics["scan_topic"], params=params)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
