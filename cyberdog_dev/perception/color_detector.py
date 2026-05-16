"""Shared lightweight ROS2 camera detector runtime."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from config.loader import load_topics
from perception.vision_utils import (
    HsvRange,
    detect_colored_objects_hsv,
    detect_dashed_line_hsv,
    detect_lane_edges_hsv,
)


def _load_hsv_config(path: Path, mode: str, key: str) -> list[HsvRange]:
    data = __import__("config.loader", fromlist=["_load_toml"])._load_toml(path)
    section = data[mode][key]
    if "lower" in section:
        return [HsvRange(lower=tuple(section["lower"]), upper=tuple(section["upper"]))]
    return [
        HsvRange(lower=tuple(section["lower1"]), upper=tuple(section["upper1"])),
        HsvRange(lower=tuple(section["lower2"]), upper=tuple(section["upper2"])),
    ]


def run_object_detector(node_name: str, publish_topic: str, hsv_key: str, label: str,
                        real_width_m: float = 0.2, min_area_px: float = 80.0, args=None):
    import rclpy
    from sensor_msgs.msg import Image
    from std_msgs.msg import String
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, QoSReliabilityPolicy

    parser = argparse.ArgumentParser(description=f"{node_name} detector")
    parser.add_argument("--mode", default="sim", choices=["sim", "real"])
    parsed, _ = parser.parse_known_args(args)

    config_dir = Path(__file__).resolve().parent.parent / "config"
    topics = load_topics(config_dir / "topics.toml", mode=parsed.mode)
    hsv_ranges = _load_hsv_config(config_dir / "hsv.toml", parsed.mode, hsv_key)

    class DetectorNode(Node):
        def __init__(self):
            super().__init__(node_name)
            qos = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT)
            self.pub = self.create_publisher(String, publish_topic, 10)
            self.create_subscription(Image, topics["camera_topic"], self._on_image, qos)
            self.get_logger().info(f"{node_name} 已启动，订阅: {topics['camera_topic']}")

        def _on_image(self, msg):
            detections = []
            try:
                import cv2
                import numpy as np
                image = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, -1))
                hsv = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)
                detections = detect_colored_objects_hsv(
                    hsv, hsv_ranges, label=label, min_area_px=min_area_px, real_width_m=real_width_m)
            except Exception as exc:
                self.get_logger().warn(f"{node_name} image handling failed: {exc}", throttle_duration_sec=2.0)
            out = String()
            out.data = json.dumps(detections, ensure_ascii=False)
            self.pub.publish(out)

    rclpy.init(args=args)
    node = DetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


def run_scalar_detector(node_name: str, publish_topic: str, args=None):
    import rclpy
    from sensor_msgs.msg import Image
    from std_msgs.msg import String
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, QoSReliabilityPolicy

    parser = argparse.ArgumentParser(description=f"{node_name} detector")
    parser.add_argument("--mode", default="sim", choices=["sim", "real"])
    parsed, _ = parser.parse_known_args(args)

    config_dir = Path(__file__).resolve().parent.parent / "config"
    topics = load_topics(config_dir / "topics.toml", mode=parsed.mode)
    hsv_key = "yellow_lane" if "lane_edge" in node_name else "white_dashed"
    hsv_ranges = _load_hsv_config(config_dir / "hsv.toml", parsed.mode, hsv_key)

    class DetectorNode(Node):
        def __init__(self):
            super().__init__(node_name)
            qos = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT)
            self.pub = self.create_publisher(String, publish_topic, 10)
            self.create_subscription(Image, topics["camera_topic"], self._on_image, qos)
            self.get_logger().info(f"{node_name} 已启动，订阅: {topics['camera_topic']}")

        def _on_image(self, msg):
            out = String()
            result = {}
            try:
                import cv2
                import numpy as np
                image = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, -1))
                hsv = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)
                if "lane_edge" in node_name:
                    result = detect_lane_edges_hsv(hsv, hsv_ranges)
                else:
                    result = detect_dashed_line_hsv(hsv, hsv_ranges) or {}
            except Exception as exc:
                self.get_logger().warn(f"{node_name} image handling failed: {exc}", throttle_duration_sec=2.0)
            out.data = json.dumps(result, ensure_ascii=False)
            self.pub.publish(out)

    rclpy.init(args=args)
    node = DetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
