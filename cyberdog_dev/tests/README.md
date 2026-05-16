# tests/

纯逻辑单元测试（pytest）。**不依赖 ROS2、不依赖 rclpy、不依赖真实机器狗**——用假 ctx / 假 perception 喂数据，验证 Stage 状态机转移、控制律计算、配置加载、感知数据合同等。

本机 macOS 也能跑：

```bash
python3 -m pip install -r requirements-dev.txt
python3 -m pytest
```

## 文件说明

| 文件 | 测试目标 |
|---|---|
| `__init__.py` | 包标记。 |
| `conftest.py` | pytest 共享配置；把项目根加入 `sys.path`，让 `import core.* / stages.*` 在 tests/ 下可用。 |
| `test_config_loader.py` | `config.loader.load_topics` / `load_stage_params` 的解析正确性。 |
| `test_core_adapters.py` | `core.gaits.low_walk.execute_low_walk` 行为；`core.voice.VoiceController` 的 `say()` 记录。 |
| `test_lane_follow.py` | `core._lane_follow.compute_lane_follow_correction` 在居中 / 偏左 / 偏右 / 前方堵塞场景下的 `(vx, vy, wz)` 输出。 |
| `test_visual_lane_follow.py` | `compute_visual_lane_follow_correction` 视觉版本：用 `LaneEdges` 假数据驱动。 |
| `test_perception_contracts.py` | `BallDet / PoleDet / ObjDet / CorridorState / LaneEdges / DashedLineDet` 等数据类的字段、默认值、几何属性（如 `PoleDet.aspect_ratio`）。 |
| `test_perception_hub.py` | `PerceptionHub` 的取值接口与初始状态。 |
| `test_vision_utils.py` | `perception.vision_utils` 纯函数：`HsvRange` 判定、`bbox_from_mask`、`bearing_from_center` 等。 |
| `test_python38_annotations.py` | AST 扫描源码，保证所有类型标注与 Python 3.8 兼容（实机狗运行环境是 3.8）。 |
| `test_stage_base.py` | `Stage / StageStatus / StageResult` 协议契约。 |
| `test_stage_context.py` | `RunMode` / `StageContext` 数据类。 |
| `test_stage1.py` ~ `test_stage6.py` | 每个 Stage 的 Phase 转移：构造 FakeCtx + FakePerception，按预期顺序喂检测结果，断言 Phase 切换和最终 `StageStatus`。 |

## 迁移说明

**本目录全部为新增，template 中没有任何单元测试。**

源项目的"测试"方式是手写脚本（`template/test.py`、`template/test_1.py`、`template/demo.py`），需要连上真实狗或 LCM 网络才能跑——无法在 PC 上验证，迭代慢。

新架构把这部分**完全废弃**，改成 pytest 单元测试：

- 所有可单测的逻辑（控制律、配置、数据合同、Stage 状态机）抽成纯函数 / 注入式依赖。
- 走 ROS2 / LCM 的部分（`core.robot_ctrl`、`core.pose_monitor`、`core.odom_broadcaster`、`perception` 节点）放在 `core/` 顶层，**不进 tests/**，靠 Gazebo 仿真和 `scripts/launch.sh` 集成验证。

这是 template → cyberdog_dev 最大的工程化改造之一（也是写 `test_python38_annotations.py` 这种校验文件的动机：实机 Python 3.8，本机开发 3.11+，必须保证类型标注向下兼容）。
