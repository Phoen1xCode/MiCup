# odom_broadcaster.py (原 test_tf_broadcaster.py)

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
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

def main(args=None):
    import argparse
    from pathlib import Path
    from config.loader import load_topics

    parser = argparse.ArgumentParser(description="Odom -> TF broadcaster")
    parser.add_argument("--mode", default="sim", choices=["sim", "real"])
    parsed, _ = parser.parse_known_args(args)

    config_dir = Path(__file__).resolve().parent.parent / "config"
    topics = load_topics(config_dir / "topics.toml", mode=parsed.mode)

    rclpy.init(args=args)
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
