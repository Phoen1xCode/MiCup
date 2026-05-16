# core/

底层控制层。把"LCM 命令 / TF 位姿 / 动作原语 / 走廊 PD / Stage 协议"全部封装在这一层，给 `stages/` 和 `main.py` 用。本层**不做**视觉处理，也**不写**任何赛段逻辑。

## 文件说明

| 文件 | 作用 |
|---|---|
| `__init__.py` | 包标记，空文件。 |
| `robot_ctrl.py` | **核心**。`RobotCtrl` 类，4 条 LCM 线程：20Hz 命令心跳、状态接收、odom 接收、自定义步态 ACK 接收。对外暴露 `set_velocity_command()`（连续速度）、`execute_discrete_action()`（站立/趴下/急停的阻塞调用）、`load_and_execute_custom_gait()`（下发 toml 步态）。还包含 `ConsoleLogger`，不依赖 rclpy 也能打日志。 |
| `basic_action.py` | 把 `RobotCtrl` 包成"人话"动作：`stand_up / speed_stand / lie_down / stop_motion / move_straight_timed / move_lateral_timed / turn_in_place_timed / walk_in_arc_timed / execute_custom_gait_sequence`。全是**开环、基于时间**的控制（`duration = distance / speed`）。 |
| `nav.py` | 闭环位姿校准。`align_yaw_to_target` 动态速度对准世界偏航；`align_axis_by_driving_forward` 用前后走校准 X/Y；`align_axis_by_strafing` 用横向平移校准；`navigate_to_exact_pose` 三阶段组合。全部基于 `pose_monitor` 给的 TF 反馈。 |
| `pose_monitor.py` | `RobotPoseMonitor` 节点。20Hz 通过 `tf2_ros` 查 `odom → base_link_leg` 的 TF，缓存 `(x, y, z, yaw)`。提供 `set_origin_here()` 把当前位姿记作零点；`get_current_pose()` 返回相对零点的位姿。 |
| `odom_broadcaster.py` | 独立 ROS2 节点。订阅 `/{namespace}/odom_out`（来自 `config/topics.toml`），转成 `odom → base_link_leg` 的 TF 广播。**必须比 PoseMonitor 先启动**，否则 TF 查询失败。 |
| `voice.py` | `VoiceController`（精简版）。当前仅 `say(text)` 打日志 + 入 `history`，未接真实 TTS/ASR。Stage 4 的强制语音播报靠它。 |
| `_lane_follow.py` | 走廊居中纯函数。`compute_lane_follow_correction(left, right, front, params)`（雷达三向距）和 `compute_visual_lane_follow_correction(lane_edges, params)`（视觉黄边线偏差）返回 `(vx, vy, wz)`。无 ROS 依赖，可单测。 |
| `stage_base.py` | Stage 协议。`Stage` 基类约定 `on_enter / tick / on_exit / max_duration_sec`；`StageStatus`（RUNNING/SUCCEEDED/FAILED/NEED_HELP）；`StageResult` 数据类。 |
| `stage_context.py` | `StageContext` 依赖容器（`dog/pose/perception/voice/logger/mode`）和 `RunMode` 枚举。`build_context(mode)` 工厂函数在 ROS2 环境中拼装出 Context，给所有 Stage 共享。 |

## 迁移说明

源项目 `/Users/phoen1xcode/Projects/MiCup/template/`（与 `example/demo1/motion/utils/` 一致）。

| 本目录文件 | 源文件 | 改动 |
|---|---|---|
| `robot_ctrl.py` | `motion/utils/Robot_Ctrl.py` | 仅修改 import 路径 `motion.utils.*_lcmt` → `core.lcm_type.*_lcmt`，逻辑零修改。 |
| `basic_action.py` | `motion/utils/basic_action.py` | 仅清理行尾空白，函数实现完全相同。 |
| `nav.py` | `motion/utils/nav.py` | 修改 import；新增 `from core._lane_follow import compute_lane_follow_correction, LaneFollowParams`，方便 Stage 直接用闭环。 |
| `pose_monitor.py` | `motion/utils/RobotPoseMonitor.py` | 重命名（驼峰 → 蛇形）；新增 `set_origin_here()` 别名以匹配 Stage 调用约定。`reference_frame='odom'`、`robot_base_frame='base_link_leg'` 保持一致。 |
| `odom_broadcaster.py` | `motion/utils/odom_broadcaster.py` | 把硬编码 topic `/mi_desktop_48_b0_2d_5f_ba_36/odom_out` 改成构造参数，配合 `config/topics.toml` 切换 sim/real。 |
| `voice.py` | `motion/utils/VoiceController.py` | **大幅精简（211 行 → 17 行）**。原版的真实 ROS2 TTS/ASR（`AudioPlayExtend / AudioTextPlay / /asr_text` 订阅、`listen_for_command` 阻塞匹配）**全部砍掉**，只剩 `say()` 写日志。如需赛场真实语音，需补回。 |

**本目录新增（template 中没有）**：

- `stage_base.py`、`stage_context.py` — Stage 协议与依赖容器，是新四层架构的骨架。
- `_lane_follow.py` — 走廊居中 PD 控制，对应 template 里散落在 `main.py / lidar_detect.py` 中的"对齐"思路，但抽成可单测的纯函数。
