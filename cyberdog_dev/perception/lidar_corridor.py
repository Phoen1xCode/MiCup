"""激光雷达走廊检测 ROS2 节点。

订阅 LaserScan，把前方扇区分成 左/前/右 三块算中位距，
发布到 /perception/lidar_corridor（geometry_msgs/Vector3，
x=left, y=front, z=right；无返回用一个大值 99.9 表示）。

独立运行：python3 -m perception.lidar_corridor --mode sim
"""

import argparse
import math
from pathlib import Path

import rclpy
from geometry_msgs.msg import Vector3
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import LaserScan

from config.loader import load_topics

NO_RETURN = 99.9


def _finite_median(values):
    data = sorted(v for v in values if math.isfinite(v) and 0.02 < v < 11.5)
    return data[len(data) // 2] if data else NO_RETURN


class LidarCorridorNode(Node):
    def __init__(self, scan_topic: str):
        super().__init__("perception_lidar_corridor")
        qos = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(LaserScan, scan_topic, self._on_scan, qos)
        self.pub = self.create_publisher(Vector3, "/perception/lidar_corridor", 10)
        self.get_logger().info(f"走廊检测节点已启动，订阅: {scan_topic}")

    def _sector(self, msg: LaserScan, deg_min: float, deg_max: float):
        n = len(msg.ranges)
        lo = max(0, int((math.radians(deg_min) - msg.angle_min) / msg.angle_increment))
        hi = min(n, int((math.radians(deg_max) - msg.angle_min) / msg.angle_increment) + 1)
        return list(msg.ranges[lo:hi])

    def _on_scan(self, msg: LaserScan):
        front = _finite_median(self._sector(msg, -12, 12))
        left = _finite_median(self._sector(msg, 45, 85))
        right = _finite_median(self._sector(msg, -85, -45))
        out = Vector3()
        out.x, out.y, out.z = left, front, right
        self.pub.publish(out)


def main(args=None):
    parser = argparse.ArgumentParser(description="Lidar corridor detector")
    parser.add_argument("--mode", default="sim", choices=["sim", "real"])
    parsed, _ = parser.parse_known_args(args)

    config_dir = Path(__file__).resolve().parent.parent / "config"
    topics = load_topics(config_dir / "topics.toml", mode=parsed.mode)

    rclpy.init(args=args)
    node = LidarCorridorNode(scan_topic=topics["scan_topic"])
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
