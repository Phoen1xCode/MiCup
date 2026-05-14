"""PerceptionHub -- Stage 获取检测结果的唯一入口（spec 3.5）。

本计划（Plan 1）只实现 Stage 1 需要的 latest_lidar_corridor()。
后续计划会补充 orange_ball / red_pole / football 等。
"""

import threading
from dataclasses import dataclass

NO_RETURN = 99.9


@dataclass
class CorridorState:
    """左/前/右三方向距离（米）。无返回用 NO_RETURN(99.9) 表示。"""
    left: float = NO_RETURN
    front: float = NO_RETURN
    right: float = NO_RETURN


class PerceptionHub:
    """聚合所有检测器，提供同步的"取最新结果"接口。"""

    def __init__(self):
        self._lock = threading.Lock()
        self._corridor = CorridorState()
        self._node = None

    def attach_node(self, node):
        """绑定一个已创建的 rclpy Node，订阅感知 topic。"""
        from geometry_msgs.msg import Vector3

        self._node = node
        node.create_subscription(
            Vector3, "/perception/lidar_corridor", self._on_corridor, 10)

    def _on_corridor(self, msg):
        with self._lock:
            self._corridor = CorridorState(left=msg.x, front=msg.y, right=msg.z)

    def latest_lidar_corridor(self) -> CorridorState:
        """返回最近一帧走廊状态。无数据时返回默认（全 NO_RETURN）。"""
        with self._lock:
            return self._corridor
