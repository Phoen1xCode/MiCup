"""pytest 共享配置。

开发机无 ROS2/LCM/Gazebo，只测纯逻辑模块。需要 rclpy/lcm 的模块
不在此处导入--它们的功能验证在 Gazebo 中进行。
"""
import sys
from pathlib import Path

# 让 tests 能 import 到 cyberdog_dev 包
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
