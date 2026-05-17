"""StageContext 依赖容器、RunMode、build_context 工厂"""

from dataclasses import dataclass
from enum import Enum
from typing import Any


class RunMode(Enum):
    SIM = "sim"
    REAL = "real"


@dataclass
class StageContext:
    """Stage 运行时的依赖容器。Stage 只通过 ctx.xxx 拿能力。"""

    dog: Any
    pose: Any
    perception: Any
    voice: Any
    logger: Any
    mode: RunMode


def build_context(mode: RunMode) -> StageContext:
    """根据运行模式组装 StageContext。

    依赖 rclpy/lcm，只能在 ROS2 环境（Gazebo 容器/实机）中调用。
    """
    import rclpy
    from perception.hub import PerceptionHub
    from rclpy.node import Node

    from core.framework.voice import VoiceController
    from core.localization.pose_monitor import RobotPoseMonitor
    from core.robot_ctrl import ConsoleLogger, RobotCtrl

    if not rclpy.ok():
        rclpy.init(args=None)

    logger = ConsoleLogger()
    dog = RobotCtrl(logger=logger, enable_odom_lcm=False, cmd_heartbeat_hz=20.0)
    dog.start()

    pose = RobotPoseMonitor()

    perception = PerceptionHub()
    hub_node = Node("perception_hub_subscriber")
    perception.attach_node(hub_node)

    return StageContext(
        dog=dog,
        pose=pose,
        perception=perception,
        voice=VoiceController(logger=logger),
        logger=logger,
        mode=mode,
    )
