"""Odom -> TF broadcaster（原 test_tf_broadcaster.py）。"""


def config_dir_from_here():
    from pathlib import Path

    return Path(__file__).resolve().parents[2] / "config"


def _build_node_class():
    from geometry_msgs.msg import TransformStamped
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from tf2_ros import TransformBroadcaster

    class OdomToTFBroadcaster(Node):
        def __init__(self, odom_topic_name: str):
            super().__init__('odom_tf_broadcaster_node')
            self.tf_broadcaster = TransformBroadcaster(self)
            self.subscription = self.create_subscription(
                Odometry, odom_topic_name, self.odom_callback, 20)
            self.get_logger().info(f"Odom广播节点已启动，正在监听: {odom_topic_name}")

        def odom_callback(self, msg: Odometry):
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = msg.header.frame_id
            t.child_frame_id = msg.child_frame_id
            t.transform.translation.x = msg.pose.pose.position.x
            t.transform.translation.y = msg.pose.pose.position.y
            t.transform.translation.z = msg.pose.pose.position.z
            t.transform.rotation = msg.pose.pose.orientation
            self.tf_broadcaster.sendTransform(t)

    return OdomToTFBroadcaster


class OdomToTFBroadcaster:
    def __init__(self, odom_topic_name: str):
        raise RuntimeError("OdomToTFBroadcaster requires ROS2; use main() in ROS2 runtime")


def main(args=None):
    import argparse

    import rclpy
    from config.loader import load_topics

    parser = argparse.ArgumentParser(description="Odom -> TF broadcaster")
    parser.add_argument("--mode", default="sim", choices=["sim", "real"])
    parsed, _ = parser.parse_known_args(args)

    config_dir = config_dir_from_here()
    topics = load_topics(config_dir / "topics.toml", mode=parsed.mode)

    rclpy.init(args=args)
    node_cls = _build_node_class()
    global OdomToTFBroadcaster
    OdomToTFBroadcaster = node_cls
    node = OdomToTFBroadcaster(odom_topic_name=topics["odom_topic"])
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
