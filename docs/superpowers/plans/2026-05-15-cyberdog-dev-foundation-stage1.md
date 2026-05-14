# cyberdog_dev 地基 + Stage 1 实施计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 按四层架构重建 `cyberdog_dev/` 的 Core/Perception 地基，并把 `example/stage1/` 重构为 `Stage1StonePath` 类，达成 spec 的"Stage 1 跑通"里程碑。

**Architecture:** 四层（Core/Perception/Stage/App）。本计划只覆盖 Stage 1 所需的最小子集：Core 层迁移 demo1 的已验证模块、Perception 层迁移激光雷达、Stage 层把 stage1.py 重构成 Phase 状态机类、App 层写最小 main.py。纯增量——原有 `cyberdog_competition/` 等文件不动。

**Tech Stack:** Python 3.10、ROS2（rclpy/sensor_msgs/tf2）、LCM、numpy、opencv-python、toml、pytest。

**Scope note:** 这是 cyberdog_dev 架构重构系列的 **Plan 1 / 共 ~6 个**。本计划产出"Stage 1 可在 Gazebo 独立跑通"的完整软件。Stage 2-6、voice、自定义步态、ekf_fusion、其余检测器在后续计划中实现（依据 spec 第 6.3 节迁移批次）。

**测试现实约束：** 开发机是 macOS，无 ROS2/LCM/Gazebo。因此：
- **纯逻辑单元**（角度数学、PD 计算、状态机转移）→ 真正的 pytest TDD，本机可跑。
- **迁移类任务**（搬运 + 适配含 rclpy/lcm 的文件）→ 验证手段是 `python3 -m py_compile` 语法检查 + 结构断言；**功能验证延后到 Gazebo**，每个此类任务末尾显式标注。

**源文件位置：**
- demo1：`/Users/phoen1xcode/Projects/MiCup/example/demo1/`
- stage1：`/Users/phoen1xcode/Projects/MiCup/example/stage1/`
- 工作目录：`/Users/phoen1xcode/Projects/MiCup/MiCup/cyberdog_dev/`（下文相对路径均相对于此）
- git 分支：`feat/framework`

---

## File Structure

本计划创建/修改的文件及其单一职责：

| 文件 | 职责 |
|---|---|
| `core/__init__.py` `perception/__init__.py` `stages/__init__.py` `config/__init__.py` | 包标记 |
| `core/lcm_type/` | LCM 消息类型（从现有 `cyberdog_dev/lcm_type/` 移入） |
| `core/robot_ctrl.py` | LCM 控制 + 心跳 + 离散动作 + 速度控制 + 自定义步态上传 |
| `core/pose_monitor.py` | 监听 TF，提供 `get_xy_yaw` / `set_origin_here` / `distance_from_origin_along` |
| `core/basic_action.py` | 开环动作原语：站立/趴下/直行/平移/转向/弧线 |
| `core/_lane_follow.py` | 纯函数：走廊居中 PD 计算（无机器人依赖，可单测） |
| `core/nav.py` | 闭环对齐：`align_yaw_to_target` / `align_axis_*` + `lane_follow_pd` 包装 |
| `core/stage_base.py` | `StageStatus` 枚举、`StageResult` 数据类、`Stage` 基类 |
| `core/stage_context.py` | `RunMode` 枚举、`StageContext` 容器、`build_context` 工厂 |
| `core/odom_broadcaster.py` | 把 `/odom_out` 转成 TF 树的 ROS2 节点 |
| `config/topics.toml` | sim/real 两套 topic 名与 LCM 命名空间 |
| `config/stage_params.toml` | 各赛段速度/距离阈值/超时（本计划只填 Stage 1） |
| `config/loader.py` | 读取 toml 配置的辅助函数 |
| `perception/lidar.py` | 激光雷达走廊检测 ROS2 节点（三扇区中位距，Stage 1 用） |
| `perception/hub.py` | `PerceptionHub` + 检测结果数据类（本计划只实现 lidar corridor） |
| `stages/stage1_stone_path.py` | `Stage1StonePath(Stage)` —— Stage 1 的 Phase 状态机 |
| `main.py` | 入口：解析 `--mode/--stages`、`build_context`、tick 循环、`STAGE_REGISTRY` |
| `scripts/launch.sh` | 重写：启动感知节点 + odom 广播 + 主程序 |
| `tests/` | pytest 单元测试（纯逻辑部分） |
| `requirements-dev.txt` | 开发依赖（pytest） |
| `README.md` | 重写：新架构说明 |

---

## Task 1: 包骨架与 pytest 脚手架

**Files:**
- Create: `core/__init__.py`, `perception/__init__.py`, `stages/__init__.py`, `config/__init__.py`
- Create: `tests/__init__.py`, `tests/conftest.py`
- Create: `requirements-dev.txt`
- Create: `pytest.ini`

- [ ] **Step 1: 创建目录与空包标记文件**

Run:
```bash
cd /Users/phoen1xcode/Projects/MiCup/MiCup/cyberdog_dev
mkdir -p core/lcm_type core/gaits perception stages config gait/limit_pole tests scripts
touch core/__init__.py perception/__init__.py stages/__init__.py config/__init__.py tests/__init__.py
```

- [ ] **Step 2: 创建 `requirements-dev.txt`**

```text
pytest>=7.0
```

- [ ] **Step 3: 创建 `pytest.ini`**

```ini
[pytest]
testpaths = tests
python_files = test_*.py
```

- [ ] **Step 4: 创建 `tests/conftest.py`**

```python
"""pytest 共享配置。

开发机无 ROS2/LCM/Gazebo，只测纯逻辑模块。需要 rclpy/lcm 的模块
不在此处导入——它们的功能验证在 Gazebo 中进行。
"""
import sys
from pathlib import Path

# 让 tests 能 import 到 cyberdog_dev 包
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
```

- [ ] **Step 5: 安装 pytest 并验证可发现**

Run:
```bash
cd /Users/phoen1xcode/Projects/MiCup/MiCup/cyberdog_dev
python3 -m pip install -r requirements-dev.txt
python3 -m pytest --collect-only
```
Expected: `collected 0 items`（无测试，但无错误）

- [ ] **Step 6: Commit**

```bash
cd /Users/phoen1xcode/Projects/MiCup/MiCup
git add cyberdog_dev/core cyberdog_dev/perception cyberdog_dev/stages cyberdog_dev/config cyberdog_dev/tests cyberdog_dev/requirements-dev.txt cyberdog_dev/pytest.ini
git commit -m "chore(cyberdog_dev): scaffold package structure and pytest setup"
```

---

## Task 2: 迁移 core/lcm_type/

**Files:**
- Create: `core/lcm_type/` (6 个文件，从现有 `lcm_type/` 复制)

**背景：** spec 第 6.1 节已验证现有 `cyberdog_dev/lcm_type/` 与 demo2 逐字节相同、且 LCM fingerprint 与 demo1 兼容。本任务只是复制到新位置（原 `lcm_type/` 保留不动）。

- [ ] **Step 1: 复制 lcm_type 文件到 core/**

Run:
```bash
cd /Users/phoen1xcode/Projects/MiCup/MiCup/cyberdog_dev
cp lcm_type/__init__.py lcm_type/robot_control_cmd_lcmt.py lcm_type/robot_control_response_lcmt.py \
   lcm_type/file_send_lcmt.py lcm_type/file_recv_lcmt.py lcm_type/localization_lcmt.py \
   lcm_type/simulator_lcmt.py core/lcm_type/
ls core/lcm_type/
```
Expected: 列出 7 个文件（含 `__init__.py`）

- [ ] **Step 2: 语法检查**

Run:
```bash
cd /Users/phoen1xcode/Projects/MiCup/MiCup/cyberdog_dev
python3 -m py_compile core/lcm_type/*.py && echo "COMPILE OK"
```
Expected: `COMPILE OK`

- [ ] **Step 3: 验证可导入**

Run:
```bash
cd /Users/phoen1xcode/Projects/MiCup/MiCup/cyberdog_dev
python3 -c "from core.lcm_type.robot_control_cmd_lcmt import robot_control_cmd_lcmt; m = robot_control_cmd_lcmt(); print('len(vel_des) =', len(m.vel_des))"
```
Expected: `len(vel_des) = 3`

- [ ] **Step 4: Commit**

```bash
cd /Users/phoen1xcode/Projects/MiCup/MiCup
git add cyberdog_dev/core/lcm_type
git commit -m "feat(cyberdog_dev): migrate lcm_type into core layer"
```

---

## Task 3: 迁移 core/robot_ctrl.py

**Files:**
- Create: `core/robot_ctrl.py` (从 `example/demo1/motion/utils/Robot_Ctrl.py` 迁移)

**背景：** demo1 的 `Robot_Ctrl.py` 已验证可用，无 `/home/` 硬编码路径。主要适配：① 改 import 路径指向 `core.lcm_type`；② 类名 `RobotCtrl` 保留，新增 spec 3.3 要求的方法别名。该文件依赖 `lcm`，开发机不一定有——本任务只做语法检查，功能验证在 Gazebo。

- [ ] **Step 1: 复制文件**

Run:
```bash
cd /Users/phoen1xcode/Projects/MiCup/MiCup/cyberdog_dev
cp ../../example/demo1/motion/utils/Robot_Ctrl.py core/robot_ctrl.py
```

- [ ] **Step 2: 修正 import 路径**

在 `core/robot_ctrl.py` 顶部，把所有 `from motion.utils.xxx import` 改成 `from core.xxx import`。具体替换：

```python
# 原（demo1 写法）：
from motion.utils.robot_control_cmd_lcmt import robot_control_cmd_lcmt
from motion.utils.robot_control_response_lcmt import robot_control_response_lcmt
from motion.utils.file_send_lcmt import file_send_lcmt
from motion.utils.localization_lcmt import localization_lcmt
from motion.utils.file_recv_lcmt import file_recv_lcmt

# 改为：
from core.lcm_type.robot_control_cmd_lcmt import robot_control_cmd_lcmt
from core.lcm_type.robot_control_response_lcmt import robot_control_response_lcmt
from core.lcm_type.file_send_lcmt import file_send_lcmt
from core.lcm_type.localization_lcmt import localization_lcmt
from core.lcm_type.file_recv_lcmt import file_recv_lcmt
```

- [ ] **Step 3: 在 RobotCtrl 类末尾添加 spec 3.3 要求的接口方法**

`core/robot_ctrl.py` 的 `RobotCtrl` 类已有 `execute_discrete_action`、`set_velocity_command`、`load_and_execute_custom_gait`、`start`。在类的最后一个方法之后，添加以下符合 spec 接口命名的薄包装（保持 spec `DogController` 协议一致）：

```python
    # ---- spec DogController 协议适配 ----
    def stand(self, *, hold: float = 2.0) -> bool:
        ok = self.execute_discrete_action(mode=12, gait_id=0, duration_ms=0,
                                          wait_for_completion=True, wait_timeout_sec=hold + 8.0)
        time.sleep(hold)
        return ok

    def lie_down(self, *, hold: float = 1.0) -> bool:
        ok = self.execute_discrete_action(mode=7, gait_id=0, duration_ms=0,
                                          wait_for_completion=True, wait_timeout_sec=hold + 8.0)
        time.sleep(hold)
        return ok

    def set_velocity(self, vx: float, vy: float, wz: float, *,
                     body_height: float = 0.0,
                     step_height: tuple = (0.06, 0.06)) -> None:
        self.set_velocity_command(vx, vy, wz, body_height=body_height,
                                  step_height=step_height)

    def stop(self) -> None:
        self.set_velocity_command(0.0, 0.0, 0.0)
```

注意：`set_velocity_command` 的签名以 `core/robot_ctrl.py` 中实际定义为准；若现有签名不接受 `body_height`/`step_height` 关键字，则在 `set_velocity` 内改为构造对应的 `robot_control_cmd_lcmt` 字段（参考类中已有的 `_current_lcm_cmd` 用法）。此适配的正确性在 Gazebo 验证。

- [ ] **Step 4: 语法检查**

Run:
```bash
cd /Users/phoen1xcode/Projects/MiCup/MiCup/cyberdog_dev
python3 -m py_compile core/robot_ctrl.py && echo "COMPILE OK"
```
Expected: `COMPILE OK`

- [ ] **Step 5: Commit**

```bash
cd /Users/phoen1xcode/Projects/MiCup/MiCup
git add cyberdog_dev/core/robot_ctrl.py
git commit -m "feat(cyberdog_dev): migrate RobotCtrl into core layer with DogController adapter"
```

**Gazebo 待验证：** `stand()` / `set_velocity()` / `stop()` 实际能驱动机器狗。

---

## Task 4: 迁移 core/pose_monitor.py

**Files:**
- Create: `core/pose_monitor.py` (从 `example/demo1/motion/utils/RobotPoseMonitor.py` 迁移)

**背景：** demo1 的 `RobotPoseMonitor` 是纯 TF 监听节点，无硬编码路径。适配：① 类不动；② 新增 spec 3.4 要求的方法 `get_xy_yaw` / `set_origin_here` / `distance_from_origin_along`。依赖 rclpy/tf2——只做语法检查。

- [ ] **Step 1: 复制文件**

Run:
```bash
cd /Users/phoen1xcode/Projects/MiCup/MiCup/cyberdog_dev
cp ../../example/demo1/motion/utils/RobotPoseMonitor.py core/pose_monitor.py
```

- [ ] **Step 2: 在 RobotPoseMonitor 类中添加 spec 3.4 接口方法**

`core/pose_monitor.py` 的 `RobotPoseMonitor` 类已有内部状态 `_current_x_tf` / `_current_y_tf` / `_current_yaw_from_tf_rad` 和偏移量 `_initial_x_offset` 等、以及 `_data_lock`。在类末尾添加：

```python
    # ---- spec PoseMonitor 协议适配 ----
    def get_xy_yaw(self) -> tuple:
        """返回 (x, y, yaw)，单位 米/米/弧度，已扣除原点偏移。"""
        with self._data_lock:
            if self._current_x_tf is None:
                return (0.0, 0.0, 0.0)
            return (
                self._current_x_tf - self._initial_x_offset,
                self._current_y_tf - self._initial_y_offset,
                self._current_yaw_from_tf_rad - self._initial_yaw_offset_rad,
            )

    def set_origin_here(self) -> None:
        """把当前位姿设为原点（每个赛段开始时调用）。"""
        self.set_current_pose_as_origin()

    def distance_from_origin_along(self, axis: str) -> float:
        """沿 'X' 或 'Y' 轴相对原点的位移（米）。"""
        x, y, _ = self.get_xy_yaw()
        if axis.upper() == "X":
            return x
        if axis.upper() == "Y":
            return y
        raise ValueError(f"axis must be 'X' or 'Y', got {axis!r}")
```

注意：`set_current_pose_as_origin` 与字段名 `_initial_x_offset` 等以 `core/pose_monitor.py` 中实际定义为准；若字段名不同则相应调整。

- [ ] **Step 3: 语法检查**

Run:
```bash
cd /Users/phoen1xcode/Projects/MiCup/MiCup/cyberdog_dev
python3 -m py_compile core/pose_monitor.py && echo "COMPILE OK"
```
Expected: `COMPILE OK`

- [ ] **Step 4: Commit**

```bash
cd /Users/phoen1xcode/Projects/MiCup/MiCup
git add cyberdog_dev/core/pose_monitor.py
git commit -m "feat(cyberdog_dev): migrate RobotPoseMonitor into core layer with PoseMonitor adapter"
```

**Gazebo 待验证：** TF 数据正确、`set_origin_here` 后位移读数归零。

---

## Task 5: 迁移 core/basic_action.py

**Files:**
- Create: `core/basic_action.py` (从 `example/demo1/motion/utils/basic_action.py` 迁移)

**背景：** demo1 的 `basic_action.py` 含 `stand_up`/`speed_stand`/`lie_down`/`move_straight_timed`/`turn_in_place_timed`/`move_lateral_timed`/`walk_in_arc_timed`/`execute_custom_gait_sequence`。Stage 1 需要前几个开环原语。`execute_custom_gait_sequence` 接收 toml 路径作参数（调用方传路径，函数本身无硬编码），可保留。只做语法检查。

- [ ] **Step 1: 复制文件**

Run:
```bash
cd /Users/phoen1xcode/Projects/MiCup/MiCup/cyberdog_dev
cp ../../example/demo1/motion/utils/basic_action.py core/basic_action.py
```

- [ ] **Step 2: 修正 import 路径（如有）**

检查 `core/basic_action.py` 顶部 import。若有 `from motion.utils.xxx import`，改为 `from core.xxx import`。若只 import 标准库（`os`/`time`/`math`），则无需改动。

Run:
```bash
cd /Users/phoen1xcode/Projects/MiCup/MiCup/cyberdog_dev
grep -n '^import\|^from' core/basic_action.py
```
按输出决定是否需要改。

- [ ] **Step 3: 语法检查 + 导入验证**

Run:
```bash
cd /Users/phoen1xcode/Projects/MiCup/MiCup/cyberdog_dev
python3 -m py_compile core/basic_action.py && echo "COMPILE OK"
python3 -c "import core.basic_action as ba; print([f for f in dir(ba) if not f.startswith('_')])"
```
Expected: `COMPILE OK`，且打印出含 `move_straight_timed`、`turn_in_place_timed`、`stand_up`、`lie_down` 的函数列表。

- [ ] **Step 4: Commit**

```bash
cd /Users/phoen1xcode/Projects/MiCup/MiCup
git add cyberdog_dev/core/basic_action.py
git commit -m "feat(cyberdog_dev): migrate basic_action primitives into core layer"
```

**Gazebo 待验证：** 各开环动作时长/速度标定。

---

## Task 6: 创建 core/_lane_follow.py（纯函数，真 TDD）

**Files:**
- Create: `core/_lane_follow.py`
- Test: `tests/test_lane_follow.py`

**背景：** 走廊居中是 Stage 1/3 的核心。把 PD 计算抽成无机器人依赖的纯函数，本机可单测。

- [ ] **Step 1: 写失败的测试**

`tests/test_lane_follow.py`:
```python
from core._lane_follow import compute_lane_follow_correction, LaneFollowParams


PARAMS = LaneFollowParams(
    forward_speed=0.2,
    lateral_gain=0.12,
    max_lateral=0.08,
    front_stop_distance=0.45,
)


def test_centered_goes_straight():
    # 左右等距 -> 不横移，全速前进
    vx, vy, wz = compute_lane_follow_correction(left=1.0, right=1.0, front=2.0, params=PARAMS)
    assert vx == 0.2
    assert vy == 0.0
    assert wz == 0.0


def test_closer_to_left_strafes_right():
    # 左边更近(left<right) -> 向右横移(vy 为负)
    vx, vy, wz = compute_lane_follow_correction(left=0.5, right=1.5, front=2.0, params=PARAMS)
    assert vx == 0.2
    assert vy < 0.0


def test_closer_to_right_strafes_left():
    vx, vy, wz = compute_lane_follow_correction(left=1.5, right=0.5, front=2.0, params=PARAMS)
    assert vy > 0.0


def test_lateral_correction_is_clamped():
    # 极端偏差 -> 横移速度被钳在 max_lateral
    vx, vy, wz = compute_lane_follow_correction(left=0.1, right=5.0, front=2.0, params=PARAMS)
    assert abs(vy) <= PARAMS.max_lateral


def test_front_blocked_stops_forward():
    # 前方近于 front_stop_distance -> 停止前进
    vx, vy, wz = compute_lane_follow_correction(left=1.0, right=1.0, front=0.3, params=PARAMS)
    assert vx == 0.0


def test_infinite_side_reading_falls_back_to_straight():
    # 一侧无返回(inf) -> 不做横向修正，仅前进
    vx, vy, wz = compute_lane_follow_correction(left=float("inf"), right=1.0, front=2.0, params=PARAMS)
    assert vy == 0.0
    assert vx == 0.2
```

- [ ] **Step 2: 运行测试确认失败**

Run:
```bash
cd /Users/phoen1xcode/Projects/MiCup/MiCup/cyberdog_dev
python3 -m pytest tests/test_lane_follow.py -v
```
Expected: FAIL —— `ModuleNotFoundError: No module named 'core._lane_follow'`

- [ ] **Step 3: 写实现**

`core/_lane_follow.py`:
```python
"""走廊居中 PD 计算（纯函数，无机器人/ROS 依赖，可单测）。"""

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class LaneFollowParams:
    forward_speed: float = 0.2      # 正常前进速度 m/s
    lateral_gain: float = 0.12      # 左右距离差 -> 横移速度 的比例增益
    max_lateral: float = 0.08       # 横移速度上限 m/s
    front_stop_distance: float = 0.45  # 前方距离小于此值则停止前进 m


def compute_lane_follow_correction(left: float, right: float, front: float,
                                   params: LaneFollowParams) -> tuple:
    """根据左/前/右三个距离算出 (vx, vy, wz) 速度指令。

    约定：vy 正方向为左。left/right/front 为米；无返回用 float('inf')。
    """
    vx = params.forward_speed
    if math.isfinite(front) and front < params.front_stop_distance:
        vx = 0.0

    vy = 0.0
    if math.isfinite(left) and math.isfinite(right):
        # 偏左(left 小) -> 需要向右(vy 负)；故用 (left - right)
        raw = (left - right) * params.lateral_gain
        vy = max(-params.max_lateral, min(params.max_lateral, raw))

    wz = 0.0
    return (vx, vy, wz)
```

- [ ] **Step 4: 运行测试确认通过**

Run:
```bash
cd /Users/phoen1xcode/Projects/MiCup/MiCup/cyberdog_dev
python3 -m pytest tests/test_lane_follow.py -v
```
Expected: 6 passed

- [ ] **Step 5: Commit**

```bash
cd /Users/phoen1xcode/Projects/MiCup/MiCup
git add cyberdog_dev/core/_lane_follow.py cyberdog_dev/tests/test_lane_follow.py
git commit -m "feat(cyberdog_dev): add pure lane-follow PD computation with tests"
```

---

## Task 7: 迁移 + 扩展 core/nav.py

**Files:**
- Create: `core/nav.py` (从 `example/demo1/motion/utils/nav.py` 迁移 + 新增 `lane_follow_pd`)

**背景：** demo1 的 `nav.py` 含 `align_yaw_to_target`/`align_axis_by_driving_forward`/`align_axis_by_strafing`/`navigate_to_exact_pose`，依赖 `basic_action`。新增 `lane_follow_pd`——用 Task 6 的纯函数 + 实际机器人调用。

- [ ] **Step 1: 复制文件**

Run:
```bash
cd /Users/phoen1xcode/Projects/MiCup/MiCup/cyberdog_dev
cp ../../example/demo1/motion/utils/nav.py core/nav.py
```

- [ ] **Step 2: 修正 import 路径**

把 `core/nav.py` 顶部的：
```python
from motion.utils.basic_action import turn_in_place_timed, move_straight_timed, move_lateral_timed
```
改为：
```python
from core.basic_action import turn_in_place_timed, move_straight_timed, move_lateral_timed
from core._lane_follow import compute_lane_follow_correction, LaneFollowParams
```

- [ ] **Step 3: 在 core/nav.py 末尾新增 lane_follow_pd**

```python
def lane_follow_pd(robot_ctrl, corridor, params: LaneFollowParams) -> tuple:
    """走廊居中：读取 corridor 距离 -> 计算速度 -> 下发给机器人。

    Args:
        robot_ctrl: 实现 set_velocity(vx, vy, wz) 的控制器。
        corridor:   含 .left / .front / .right 三个 float 属性的对象
                    （来自 PerceptionHub.latest_lidar_corridor()）。
        params:     LaneFollowParams 调参对象。
    Returns:
        实际下发的 (vx, vy, wz)。
    """
    vx, vy, wz = compute_lane_follow_correction(
        left=corridor.left, right=corridor.right, front=corridor.front, params=params,
    )
    robot_ctrl.set_velocity(vx, vy, wz)
    return (vx, vy, wz)
```

- [ ] **Step 4: 语法检查**

Run:
```bash
cd /Users/phoen1xcode/Projects/MiCup/MiCup/cyberdog_dev
python3 -m py_compile core/nav.py && echo "COMPILE OK"
```
Expected: `COMPILE OK`

- [ ] **Step 5: Commit**

```bash
cd /Users/phoen1xcode/Projects/MiCup/MiCup
git add cyberdog_dev/core/nav.py
git commit -m "feat(cyberdog_dev): migrate nav module and add lane_follow_pd wrapper"
```

**Gazebo 待验证：** `align_yaw_to_target` 闭环收敛、`lane_follow_pd` 居中效果。

---

## Task 8: 创建 core/stage_base.py（真 TDD）

**Files:**
- Create: `core/stage_base.py`
- Test: `tests/test_stage_base.py`

**背景：** spec 3.1 的 `Stage` 基类、`StageStatus`、`StageResult`。纯逻辑，本机可测。

- [ ] **Step 1: 写失败的测试**

`tests/test_stage_base.py`:
```python
import pytest
from core.stage_base import Stage, StageStatus, StageResult


def test_stage_status_has_four_states():
    names = {s.name for s in StageStatus}
    assert names == {"RUNNING", "SUCCEEDED", "FAILED", "NEED_HELP"}


def test_stage_result_fields():
    r = StageResult(stage_id=1, name="石径探路", status=StageStatus.SUCCEEDED,
                    notes=["ok"], elapsed_sec=12.3)
    assert r.stage_id == 1
    assert r.status is StageStatus.SUCCEEDED
    assert r.notes == ["ok"]


def test_base_stage_tick_must_be_overridden():
    s = Stage(ctx=object())
    with pytest.raises(NotImplementedError):
        s.tick()


def test_subclass_can_implement_tick():
    class Dummy(Stage):
        stage_id = 9
        name = "dummy"
        def tick(self):
            return StageStatus.SUCCEEDED

    d = Dummy(ctx=object())
    assert d.tick() is StageStatus.SUCCEEDED
    assert d.stage_id == 9


def test_default_max_duration_is_positive():
    s = Stage(ctx=object())
    assert s.max_duration_sec() > 0


def test_lifecycle_hooks_are_noops_by_default():
    s = Stage(ctx=object())
    # 不应抛异常
    s.on_enter()
    s.on_exit()
```

- [ ] **Step 2: 运行测试确认失败**

Run:
```bash
cd /Users/phoen1xcode/Projects/MiCup/MiCup/cyberdog_dev
python3 -m pytest tests/test_stage_base.py -v
```
Expected: FAIL —— `ModuleNotFoundError: No module named 'core.stage_base'`

- [ ] **Step 3: 写实现**

`core/stage_base.py`:
```python
"""Stage 基类与状态类型（spec 3.1）。"""

from dataclasses import dataclass, field
from enum import Enum, auto
from typing import List


class StageStatus(Enum):
    RUNNING = auto()        # 正在执行
    SUCCEEDED = auto()      # 完成结束条件
    FAILED = auto()         # 永久失败，不可恢复
    NEED_HELP = auto()      # 卡死，等待语音/触摸触发恢复（合法人机协作）


@dataclass
class StageResult:
    stage_id: int
    name: str
    status: StageStatus
    notes: List[str] = field(default_factory=list)
    elapsed_sec: float = 0.0


class Stage:
    """所有赛段的基类。子类必须实现 tick()。"""

    stage_id: int = 0
    name: str = ""

    def __init__(self, ctx):
        self.ctx = ctx
        self.start_time = 0.0
        self.notes: List[str] = []

    def on_enter(self) -> None:
        """进入赛段时调用一次（设原点等）。默认空操作。"""

    def tick(self) -> StageStatus:
        """状态机的一次推进。子类必须实现。"""
        raise NotImplementedError("Stage 子类必须实现 tick()")

    def on_exit(self) -> None:
        """退出赛段时清理（停车等）。默认空操作。"""

    def max_duration_sec(self) -> float:
        """该赛段的超时上限（秒）。"""
        return 180.0
```

- [ ] **Step 4: 运行测试确认通过**

Run:
```bash
cd /Users/phoen1xcode/Projects/MiCup/MiCup/cyberdog_dev
python3 -m pytest tests/test_stage_base.py -v
```
Expected: 6 passed

- [ ] **Step 5: Commit**

```bash
cd /Users/phoen1xcode/Projects/MiCup/MiCup
git add cyberdog_dev/core/stage_base.py cyberdog_dev/tests/test_stage_base.py
git commit -m "feat(cyberdog_dev): add Stage base class and status types with tests"
```

---

## Task 9: 创建 config/ 配置文件与加载器（真 TDD）

**Files:**
- Create: `config/topics.toml`
- Create: `config/stage_params.toml`
- Create: `config/loader.py`
- Test: `tests/test_config_loader.py`

**背景：** spec 3.6 要求 sim/real 差异通过 config 吸收。本计划只填 Stage 1 需要的键。

- [ ] **Step 1: 创建 `config/topics.toml`**

```toml
# sim/real 两套 topic 名与 LCM 命名空间。
# demo1 里散落的 /mi_desktop_48_b0_2d_5f_ba_36 命名空间集中到这里。

[sim]
namespace = ""
scan_topic = "/scan"
odom_topic = "/odom_out"
camera_topic = "/rgb_camera/image_raw"
imu_topic = "/imu"

[real]
namespace = "/mi_desktop_48_b0_2d_5f_ba_36"
scan_topic = "/mi_desktop_48_b0_2d_5f_ba_36/scan"
odom_topic = "/mi_desktop_48_b0_2d_5f_ba_36/odom_out"
camera_topic = "/mi_desktop_48_b0_2d_5f_ba_36/image"
imu_topic = "/camera/imu"
```

- [ ] **Step 2: 创建 `config/stage_params.toml`**

```toml
# 各赛段速度/距离阈值/超时。本计划只填 Stage 1，后续计划补充 2-6。
# 所有数值为 Gazebo 保守初值，进仿真后逐段调参。

[stage1]
stand_time = 4.5            # 站立持续 s
stabilize_time = 1.2        # 站立后稳定 s
forward_speed = 0.2         # 直行速度 m/s
exit_speed = 0.3            # 冲出阶段速度 m/s
turn_forward_speed = 0.3    # 转弯时前进速度 m/s
turn_yaw_speed = 0.52       # 转弯角速度 rad/s
turn_angle = 1.57           # 预期转弯弧度（90°）
bend_front_threshold = 1.05 # 前方距离小于此值认为接近弯道 m
open_side_threshold = 1.15  # 一侧距离大于此值认为开阔 m
exit_front_clear = 1.65     # 转弯后前方需大于此值才安全 m
min_straight_time = 6.0     # 最短直行时间 s（防过早转弯）
max_straight_time = 16.0    # 最长直行时间 s（超时强制转弯）
min_turn_time = 2.0
max_turn_time = 5.5
finish_straight_time = 4.5  # 冲出阶段直行时间 s
max_time = 45.0             # 赛段总超时 s
lateral_gain = 0.12         # 走廊居中横移增益
max_lateral = 0.08          # 横移速度上限 m/s
front_stop_distance = 0.45  # 走廊前方停止距离 m
```

- [ ] **Step 3: 写失败的测试**

`tests/test_config_loader.py`:
```python
from pathlib import Path
from config.loader import load_topics, load_stage_params

CONFIG_DIR = Path(__file__).resolve().parent.parent / "config"


def test_load_topics_sim():
    t = load_topics(CONFIG_DIR / "topics.toml", mode="sim")
    assert t["scan_topic"] == "/scan"
    assert t["namespace"] == ""


def test_load_topics_real():
    t = load_topics(CONFIG_DIR / "topics.toml", mode="real")
    assert t["namespace"] == "/mi_desktop_48_b0_2d_5f_ba_36"
    assert t["scan_topic"].startswith("/mi_desktop")


def test_load_topics_rejects_unknown_mode():
    import pytest
    with pytest.raises(KeyError):
        load_topics(CONFIG_DIR / "topics.toml", mode="bogus")


def test_load_stage_params_stage1():
    p = load_stage_params(CONFIG_DIR / "stage_params.toml", stage_id=1)
    assert p["forward_speed"] == 0.2
    assert p["max_time"] == 45.0
```

- [ ] **Step 4: 运行测试确认失败**

Run:
```bash
cd /Users/phoen1xcode/Projects/MiCup/MiCup/cyberdog_dev
python3 -m pytest tests/test_config_loader.py -v
```
Expected: FAIL —— `ModuleNotFoundError: No module named 'config.loader'`

- [ ] **Step 5: 写实现**

`config/loader.py`:
```python
"""toml 配置加载辅助。"""

from pathlib import Path
from typing import Dict

import toml


def load_topics(path: Path, mode: str) -> Dict[str, str]:
    """读取 topics.toml，返回指定 mode（'sim'/'real'）的 topic 字典。"""
    data = toml.load(path)
    if mode not in data:
        raise KeyError(f"topics.toml 中没有 mode={mode!r}，可选: {list(data)}")
    return data[mode]


def load_stage_params(path: Path, stage_id: int) -> Dict[str, float]:
    """读取 stage_params.toml，返回指定赛段的参数字典。"""
    data = toml.load(path)
    key = f"stage{stage_id}"
    if key not in data:
        raise KeyError(f"stage_params.toml 中没有 {key}，可选: {list(data)}")
    return data[key]
```

- [ ] **Step 6: 运行测试确认通过**

Run:
```bash
cd /Users/phoen1xcode/Projects/MiCup/MiCup/cyberdog_dev
python3 -m pytest tests/test_config_loader.py -v
```
Expected: 4 passed

- [ ] **Step 7: Commit**

```bash
cd /Users/phoen1xcode/Projects/MiCup/MiCup
git add cyberdog_dev/config/topics.toml cyberdog_dev/config/stage_params.toml cyberdog_dev/config/loader.py cyberdog_dev/tests/test_config_loader.py
git commit -m "feat(cyberdog_dev): add config files and loader with tests"
```

---

## Task 10: 迁移 core/odom_broadcaster.py

**Files:**
- Create: `core/odom_broadcaster.py` (从 `example/demo1/motion/utils/odom_broadcaster.py` 迁移)

**背景：** demo1 的 `odom_broadcaster.py` 把 `/odom_out` 转 TF，但 topic 名硬编码在第 13 行 `'/mi_desktop_48_b0_2d_5f_ba_36/odom_out'`。改成从 `config/topics.toml` 读 + 接受 `--mode` 命令行参数。依赖 rclpy——只做语法检查。

- [ ] **Step 1: 复制文件**

Run:
```bash
cd /Users/phoen1xcode/Projects/MiCup/MiCup/cyberdog_dev
cp ../../example/demo1/motion/utils/odom_broadcaster.py core/odom_broadcaster.py
```

- [ ] **Step 2: 改成从 config 读 topic 名**

把 `core/odom_broadcaster.py` 的 `OdomToTFBroadcaster.__init__` 里：
```python
        odom_topic_name = '/mi_desktop_48_b0_2d_5f_ba_36/odom_out'
        # odom_topic_name = '/mi_desktop_48_b0_2d_7b_06_89/odom_out'
```
改为构造函数接收参数：
```python
    def __init__(self, odom_topic_name: str):
```
并把 `main()` 改为：
```python
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
```

- [ ] **Step 3: 语法检查**

Run:
```bash
cd /Users/phoen1xcode/Projects/MiCup/MiCup/cyberdog_dev
python3 -m py_compile core/odom_broadcaster.py && echo "COMPILE OK"
```
Expected: `COMPILE OK`

- [ ] **Step 4: Commit**

```bash
cd /Users/phoen1xcode/Projects/MiCup/MiCup
git add cyberdog_dev/core/odom_broadcaster.py
git commit -m "feat(cyberdog_dev): migrate odom_broadcaster with config-driven topic"
```

**Gazebo 待验证：** `python3 -m core.odom_broadcaster --mode sim` 能发布 TF。

---

## Task 11: 创建 perception/lidar.py 走廊检测节点

**Files:**
- Create: `perception/lidar.py`

**背景：** spec 要求 perception 节点独立运行 + 发布 topic。本任务创建激光雷达节点：订阅 `/scan`，把雷达分成左/前/右三扇区算中位距，发布 `CorridorState` 到 `/perception/lidar_corridor`。Stage 1 只需要三扇区距离（走廊居中 + 弯道检测），不需要 demo1 的完整 RANSAC 直线拟合（那留给 Stage 5 独木桥）。依赖 rclpy——只做语法检查。

- [ ] **Step 1: 创建 `perception/lidar.py`**

```python
"""激光雷达走廊检测 ROS2 节点。

订阅 LaserScan，把前方扇区分成 左/前/右 三块算中位距，
发布到 /perception/lidar_corridor（geometry_msgs/Vector3，
x=left, y=front, z=right；无返回用一个大值 99.9 表示）。

独立运行：python3 -m perception.lidar --mode sim
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

NO_RETURN = 99.9  # 无有效返回时填充的"很远"距离


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
```

- [ ] **Step 2: 语法检查**

Run:
```bash
cd /Users/phoen1xcode/Projects/MiCup/MiCup/cyberdog_dev
python3 -m py_compile perception/lidar.py && echo "COMPILE OK"
```
Expected: `COMPILE OK`

- [ ] **Step 3: Commit**

```bash
cd /Users/phoen1xcode/Projects/MiCup/MiCup
git add cyberdog_dev/perception/lidar.py
git commit -m "feat(cyberdog_dev): add lidar corridor detection node"
```

**Gazebo 待验证：** `ros2 topic echo /perception/lidar_corridor` 输出合理的左/前/右距离。

---

## Task 12: 创建 perception/hub.py（PerceptionHub + 数据类）

**Files:**
- Create: `perception/hub.py`
- Test: `tests/test_perception_hub.py`

**背景：** spec 3.5 的 `PerceptionHub` 是 Stage 取检测结果的唯一入口。本计划只实现 Stage 1 需要的 `latest_lidar_corridor()`。`CorridorState` 数据类纯逻辑可测；订阅部分依赖 rclpy 只做语法检查。

- [ ] **Step 1: 写失败的测试（只测纯数据类与降级逻辑）**

`tests/test_perception_hub.py`:
```python
from perception.hub import CorridorState


def test_corridor_state_fields():
    c = CorridorState(left=1.0, front=2.0, right=1.5)
    assert c.left == 1.0
    assert c.front == 2.0
    assert c.right == 1.5


def test_corridor_state_default_is_no_return():
    # 默认值应是"很远"，让 Stage 在没数据时不会误判为撞墙
    c = CorridorState()
    assert c.left > 10.0
    assert c.front > 10.0
    assert c.right > 10.0
```

- [ ] **Step 2: 运行测试确认失败**

Run:
```bash
cd /Users/phoen1xcode/Projects/MiCup/MiCup/cyberdog_dev
python3 -m pytest tests/test_perception_hub.py -v
```
Expected: FAIL —— `ModuleNotFoundError: No module named 'perception.hub'`

- [ ] **Step 3: 写实现**

`perception/hub.py`:
```python
"""PerceptionHub —— Stage 获取检测结果的唯一入口（spec 3.5）。

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
    """聚合所有检测器，提供同步的"取最新结果"接口。

    内部用一个 ROS2 节点订阅各 perception topic，线程安全地缓存最新一帧。
    """

    def __init__(self):
        self._lock = threading.Lock()
        self._corridor = CorridorState()
        self._node = None  # 由 attach_node() 注入，便于测试时不依赖 rclpy

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
```

- [ ] **Step 4: 运行测试确认通过**

Run:
```bash
cd /Users/phoen1xcode/Projects/MiCup/MiCup/cyberdog_dev
python3 -m pytest tests/test_perception_hub.py -v
```
Expected: 2 passed

- [ ] **Step 5: 语法检查整个文件**

Run:
```bash
cd /Users/phoen1xcode/Projects/MiCup/MiCup/cyberdog_dev
python3 -m py_compile perception/hub.py && echo "COMPILE OK"
```
Expected: `COMPILE OK`

- [ ] **Step 6: Commit**

```bash
cd /Users/phoen1xcode/Projects/MiCup/MiCup
git add cyberdog_dev/perception/hub.py cyberdog_dev/tests/test_perception_hub.py
git commit -m "feat(cyberdog_dev): add PerceptionHub with lidar corridor access"
```

---

## Task 13: 创建 core/stage_context.py

**Files:**
- Create: `core/stage_context.py`
- Test: `tests/test_stage_context.py`

**背景：** spec 3.2/3.6 的 `StageContext` 容器、`RunMode` 枚举、`build_context` 工厂。`RunMode` 枚举纯逻辑可测；`build_context` 依赖 rclpy/lcm 只做语法检查。

- [ ] **Step 1: 写失败的测试（只测 RunMode 与 StageContext 数据类）**

`tests/test_stage_context.py`:
```python
from core.stage_context import RunMode, StageContext


def test_runmode_has_sim_and_real():
    assert RunMode.SIM.value == "sim"
    assert RunMode.REAL.value == "real"


def test_runmode_from_string():
    assert RunMode("sim") is RunMode.SIM
    assert RunMode("real") is RunMode.REAL


def test_stage_context_holds_dependencies():
    # 用占位对象验证字段存在、可读
    ctx = StageContext(
        dog="DOG", pose="POSE", perception="PERC",
        voice="VOICE", logger="LOG", mode=RunMode.SIM,
    )
    assert ctx.dog == "DOG"
    assert ctx.mode is RunMode.SIM
```

- [ ] **Step 2: 运行测试确认失败**

Run:
```bash
cd /Users/phoen1xcode/Projects/MiCup/MiCup/cyberdog_dev
python3 -m pytest tests/test_stage_context.py -v
```
Expected: FAIL —— `ModuleNotFoundError: No module named 'core.stage_context'`

- [ ] **Step 3: 写实现**

`core/stage_context.py`:
```python
"""StageContext 依赖容器、RunMode、build_context 工厂（spec 3.2/3.6）。"""

from dataclasses import dataclass
from enum import Enum
from typing import Any


class RunMode(Enum):
    SIM = "sim"      # Gazebo 仿真（初赛）
    REAL = "real"    # 实机（决赛）


@dataclass
class StageContext:
    """Stage 运行时的依赖容器。Stage 只通过 ctx.xxx 拿能力。"""
    dog: Any            # DogController（core.robot_ctrl.RobotCtrl）
    pose: Any           # PoseMonitor（core.pose_monitor.RobotPoseMonitor）
    perception: Any     # PerceptionHub
    voice: Any          # VoiceController（Plan 1 暂为 None）
    logger: Any
    mode: RunMode


def build_context(mode: RunMode) -> StageContext:
    """根据运行模式组装 StageContext。

    依赖 rclpy/lcm，只能在 ROS2 环境（Gazebo 容器/实机）中调用。
    """
    import rclpy
    from rclpy.node import Node

    from core.pose_monitor import RobotPoseMonitor
    from core.robot_ctrl import RobotCtrl, ConsoleLogger
    from perception.hub import PerceptionHub

    if not rclpy.ok():
        rclpy.init(args=None)

    logger = ConsoleLogger()
    dog = RobotCtrl(ros2_logger=logger, enable_odom_lcm=False, cmd_heartbeat_hz=20.0)
    dog.start()

    pose = RobotPoseMonitor()

    perception = PerceptionHub()
    hub_node = Node("perception_hub_subscriber")
    perception.attach_node(hub_node)

    return StageContext(
        dog=dog,
        pose=pose,
        perception=perception,
        voice=None,  # Plan 1 不实现语音；Stage 4 计划再补
        logger=logger,
        mode=mode,
    )
```

注意：`RobotCtrl` / `ConsoleLogger` / `RobotPoseMonitor` 的构造参数以 Task 3/4 迁移后 `core/robot_ctrl.py`、`core/pose_monitor.py` 中的实际签名为准；若不同则相应调整。`build_context` 的运行正确性在 Gazebo 验证。

- [ ] **Step 4: 运行测试确认通过**

Run:
```bash
cd /Users/phoen1xcode/Projects/MiCup/MiCup/cyberdog_dev
python3 -m pytest tests/test_stage_context.py -v
```
Expected: 3 passed

- [ ] **Step 5: 语法检查**

Run:
```bash
cd /Users/phoen1xcode/Projects/MiCup/MiCup/cyberdog_dev
python3 -m py_compile core/stage_context.py && echo "COMPILE OK"
```
Expected: `COMPILE OK`

- [ ] **Step 6: Commit**

```bash
cd /Users/phoen1xcode/Projects/MiCup/MiCup
git add cyberdog_dev/core/stage_context.py cyberdog_dev/tests/test_stage_context.py
git commit -m "feat(cyberdog_dev): add StageContext, RunMode and build_context factory"
```

---

## Task 14: 重构 stages/stage1_stone_path.py

**Files:**
- Create: `stages/stage1_stone_path.py` (从 `example/stage1/stage1.py` 重构)

**背景：** `example/stage1/stage1.py` 是 `Stage1Runner(Node)`——一个自带 ROS2 timer 的节点，`control_loop` 是状态机。重构为 `Stage1StonePath(Stage)`：去掉 Node 继承、去掉 timer，`control_loop` 的内容搬进 `tick()`，传感器数据改从 `ctx.perception.latest_lidar_corridor()` 取，动作改用 `ctx.dog`。Phase 枚举沿用原文件。依赖链含 rclpy——只做语法检查 + Phase 转移纯逻辑测试。

- [ ] **Step 1: 创建 `stages/stage1_stone_path.py`**

```python
"""Stage 1 · 石径探路 —— Phase 状态机（spec 4.1）。

重构自 example/stage1/stage1.py：去掉 ROS2 Node 继承与 timer，
control_loop 内容搬进 tick()，传感器走 ctx.perception，动作走 ctx.dog。
"""

import time
from enum import Enum, auto
from pathlib import Path

from core.stage_base import Stage, StageStatus
from core.config_helpers import load_stage_params_for  # 见 Step 2 说明


class Phase(Enum):
    RECOVERY_STAND = auto()      # 站立恢复
    STABILIZE = auto()           # 短暂稳定
    STRAIGHT_TO_BEND = auto()    # 直行至弯道
    TURNING = auto()             # 转弯
    STRAIGHT_TO_EXIT = auto()    # 直行冲出虚线
    DONE = auto()                # 完成


class Stage1StonePath(Stage):
    stage_id = 1
    name = "石径探路"

    def __init__(self, ctx):
        super().__init__(ctx)
        self.phase = Phase.RECOVERY_STAND
        self.phase_start = 0.0
        self.turn_direction = 0.0
        self.estimated_turn_angle = 0.0
        self.last_tick_time = 0.0
        # 从 config 读参数
        config_dir = Path(__file__).resolve().parent.parent / "config"
        from config.loader import load_stage_params
        self.p = load_stage_params(config_dir / "stage_params.toml", stage_id=1)

    def on_enter(self) -> None:
        self.start_time = time.monotonic()
        self.phase_start = self.start_time
        self.last_tick_time = self.start_time
        self.ctx.pose.set_origin_here()
        self.ctx.logger.info(f"[{self.name}] 进入，phase=RECOVERY_STAND")

    def max_duration_sec(self) -> float:
        return float(self.p["max_time"])

    def _switch(self, phase: Phase) -> None:
        self.phase = phase
        self.phase_start = time.monotonic()
        self.ctx.logger.info(f"[{self.name}] -> {phase.name}")

    def tick(self) -> StageStatus:
        now = time.monotonic()
        dt = max(0.0, now - self.last_tick_time)
        self.last_tick_time = now
        elapsed = now - self.phase_start

        corridor = self.ctx.perception.latest_lidar_corridor()
        front, left, right = corridor.front, corridor.left, corridor.right

        # ----- RECOVERY_STAND -----
        if self.phase == Phase.RECOVERY_STAND:
            self.ctx.dog.stand(hold=0.0)
            if elapsed >= self.p["stand_time"]:
                self._switch(Phase.STABILIZE)
            return StageStatus.RUNNING

        # ----- STABILIZE -----
        if self.phase == Phase.STABILIZE:
            self.ctx.dog.stop()
            if elapsed >= self.p["stabilize_time"]:
                self._switch(Phase.STRAIGHT_TO_BEND)
            return StageStatus.RUNNING

        # ----- STRAIGHT_TO_BEND -----
        if self.phase == Phase.STRAIGHT_TO_BEND:
            self.ctx.dog.set_velocity(self.p["forward_speed"], 0.0, 0.0)
            bend_by_lidar = (
                elapsed >= self.p["min_straight_time"]
                and front <= self.p["bend_front_threshold"]
                and max(left, right) >= self.p["open_side_threshold"]
            )
            bend_by_timeout = elapsed >= self.p["max_straight_time"]
            if bend_by_lidar or bend_by_timeout:
                self.turn_direction = 1.0 if left >= right else -1.0
                self.estimated_turn_angle = 0.0
                self._switch(Phase.TURNING)
            return StageStatus.RUNNING

        # ----- TURNING -----
        if self.phase == Phase.TURNING:
            self.ctx.dog.set_velocity(
                self.p["turn_forward_speed"], 0.0,
                self.turn_direction * self.p["turn_yaw_speed"])
            self.estimated_turn_angle += abs(self.p["turn_yaw_speed"]) * dt
            front_clear = front >= self.p["exit_front_clear"]
            turned_enough = self.estimated_turn_angle >= self.p["turn_angle"]
            timed_out = elapsed >= self.p["max_turn_time"]
            if elapsed >= self.p["min_turn_time"] and (
                    (front_clear and turned_enough) or timed_out):
                self._switch(Phase.STRAIGHT_TO_EXIT)
            return StageStatus.RUNNING

        # ----- STRAIGHT_TO_EXIT -----
        if self.phase == Phase.STRAIGHT_TO_EXIT:
            self.ctx.dog.set_velocity(self.p["exit_speed"], 0.0, 0.0)
            if elapsed >= self.p["finish_straight_time"]:
                self._switch(Phase.DONE)
            return StageStatus.RUNNING

        # ----- DONE -----
        self.ctx.dog.stop()
        return StageStatus.SUCCEEDED

    def on_exit(self) -> None:
        self.ctx.dog.stop()
        self.ctx.logger.info(f"[{self.name}] 退出")
```

注意：上面 import 了 `core.config_helpers` 但实际未用——**删掉那一行 import**（`from core.config_helpers import ...`）。配置加载用的是 `from config.loader import load_stage_params`（已在 `__init__` 内）。

- [ ] **Step 2: 删除无用 import**

把 `stages/stage1_stone_path.py` 第 11 行 `from core.config_helpers import load_stage_params_for` 删除（该模块不存在、也不需要）。

- [ ] **Step 3: 语法检查**

Run:
```bash
cd /Users/phoen1xcode/Projects/MiCup/MiCup/cyberdog_dev
python3 -m py_compile stages/stage1_stone_path.py && echo "COMPILE OK"
```
Expected: `COMPILE OK`

- [ ] **Step 4: 写 Phase 转移的纯逻辑测试**

`tests/test_stage1.py`:
```python
"""Stage1 的纯逻辑测试：用假 ctx 验证 Phase 转移，不依赖 ROS2。"""
import time
from stages.stage1_stone_path import Stage1StonePath, Phase
from core.stage_base import StageStatus


class FakeDog:
    def __init__(self): self.calls = []
    def stand(self, *, hold=0.0): self.calls.append(("stand", hold))
    def stop(self): self.calls.append(("stop",))
    def set_velocity(self, vx, vy, wz): self.calls.append(("vel", vx, vy, wz))


class FakeCorridor:
    def __init__(self, left, front, right):
        self.left, self.front, self.right = left, front, right


class FakePerception:
    def __init__(self, corridor): self._c = corridor
    def latest_lidar_corridor(self): return self._c


class FakePose:
    def set_origin_here(self): pass


class FakeLogger:
    def info(self, msg): pass


def make_ctx(corridor):
    class Ctx: pass
    c = Ctx()
    c.dog = FakeDog()
    c.pose = FakePose()
    c.perception = FakePerception(corridor)
    c.logger = FakeLogger()
    return c


def test_starts_in_recovery_stand():
    ctx = make_ctx(FakeCorridor(1.0, 2.0, 1.0))
    s = Stage1StonePath(ctx)
    s.on_enter()
    assert s.phase is Phase.RECOVERY_STAND


def test_recovery_stand_advances_after_stand_time():
    ctx = make_ctx(FakeCorridor(1.0, 2.0, 1.0))
    s = Stage1StonePath(ctx)
    s.on_enter()
    # 伪造时间：把 phase_start 往前拨超过 stand_time
    s.phase_start -= s.p["stand_time"] + 0.1
    status = s.tick()
    assert status is StageStatus.RUNNING
    assert s.phase is Phase.STABILIZE


def test_done_phase_returns_succeeded():
    ctx = make_ctx(FakeCorridor(2.0, 2.0, 2.0))
    s = Stage1StonePath(ctx)
    s.on_enter()
    s.phase = Phase.DONE
    assert s.tick() is StageStatus.SUCCEEDED


def test_straight_to_bend_triggers_turn_on_lidar():
    # 前方近 + 左侧开阔 -> 进入 TURNING，且选左转
    ctx = make_ctx(FakeCorridor(left=1.5, front=0.9, right=0.5))
    s = Stage1StonePath(ctx)
    s.on_enter()
    s.phase = Phase.STRAIGHT_TO_BEND
    s.phase_start -= s.p["min_straight_time"] + 0.1
    s.tick()
    assert s.phase is Phase.TURNING
    assert s.turn_direction == 1.0  # left >= right -> 左转
```

- [ ] **Step 5: 运行测试确认通过**

Run:
```bash
cd /Users/phoen1xcode/Projects/MiCup/MiCup/cyberdog_dev
python3 -m pytest tests/test_stage1.py -v
```
Expected: 4 passed

- [ ] **Step 6: Commit**

```bash
cd /Users/phoen1xcode/Projects/MiCup/MiCup
git add cyberdog_dev/stages/stage1_stone_path.py cyberdog_dev/tests/test_stage1.py
git commit -m "feat(cyberdog_dev): refactor stage1 into Stage1StonePath phase machine"
```

**Gazebo 待验证：** 完整赛段一在仿真中走通（站立→石板→弯道→出虚线）。

---

## Task 15: 创建 main.py 入口

**Files:**
- Create: `main.py`

**背景：** spec 第 7 节。main.py 解析 `--mode/--stages`，`build_context`，注册 `STAGE_REGISTRY`，跑 tick 循环（带超时检查）。依赖 rclpy——只做语法检查 + `--help` 验证（argparse 部分不依赖 rclpy，可在本机跑）。

- [ ] **Step 1: 创建 `main.py`**

```python
#!/usr/bin/env python3
"""MiCup 2026 CyberDog 比赛入口。

用法：
    python3 main.py --mode sim --stages 1
    python3 main.py --mode sim --stages 1-6
    python3 main.py --mode real --stages 1,3,5

需在 ROS2 环境（Gazebo 容器 / 实机）中运行。感知节点与 odom 广播
需提前由 scripts/launch.sh 启动。
"""

import argparse
import sys
import time

# STAGE_REGISTRY 的 import 放在 main() 内部，避免 --help 时触发 rclpy 依赖链


def parse_stages(value: str):
    if value in ("all", "1-6"):
        return [1, 2, 3, 4, 5, 6]
    result = []
    for part in value.split(","):
        part = part.strip()
        if not part:
            continue
        if "-" in part:
            lo, hi = part.split("-")
            result.extend(range(int(lo), int(hi) + 1))
        else:
            result.append(int(part))
    for n in result:
        if n < 1 or n > 6:
            raise argparse.ArgumentTypeError("stages must be in 1..6")
    return result


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="MiCup 2026 CyberDog competition runner")
    parser.add_argument("--mode", default="sim", choices=["sim", "real"],
                        help="运行模式：sim=Gazebo 仿真，real=实机")
    parser.add_argument("--stages", default="1-6", type=parse_stages,
                        help="要跑的赛段，如 1-6 或 1,3,5")
    return parser


def run_stage(stage, ctx) -> "StageResult":
    """以 ~20Hz tick 一个赛段直到结束或超时。"""
    from core.stage_base import StageStatus, StageResult

    stage.on_enter()
    start = time.monotonic()
    status = StageStatus.RUNNING
    try:
        while status == StageStatus.RUNNING:
            status = stage.tick()
            if time.monotonic() - start > stage.max_duration_sec():
                ctx.logger.warn(f"[{stage.name}] 超时，强制结束")
                status = StageStatus.FAILED
                break
            time.sleep(0.05)  # ~20 Hz
    finally:
        stage.on_exit()
    elapsed = time.monotonic() - start
    return StageResult(stage_id=stage.stage_id, name=stage.name,
                       status=status, notes=list(stage.notes), elapsed_sec=elapsed)


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)

    from core.stage_context import RunMode, build_context
    from stages.stage1_stone_path import Stage1StonePath

    # 赛段注册表。后续计划补充 2-6。
    STAGE_REGISTRY = {
        1: Stage1StonePath,
    }

    ctx = build_context(RunMode(args.mode))
    results = []
    try:
        for stage_id in args.stages:
            stage_cls = STAGE_REGISTRY.get(stage_id)
            if stage_cls is None:
                ctx.logger.warn(f"赛段 {stage_id} 尚未实现，跳过")
                continue
            ctx.logger.info(f"=== 开始赛段 {stage_id} ===")
            results.append(run_stage(stage_cls(ctx), ctx))
    except KeyboardInterrupt:
        ctx.logger.warn("收到中断，停止机器狗")
        ctx.dog.stop()

    print("\n运行总结")
    for r in results:
        print(f"  S{r.stage_id} {r.name}: {r.status.name} ({r.elapsed_sec:.1f}s)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
```

- [ ] **Step 2: 语法检查**

Run:
```bash
cd /Users/phoen1xcode/Projects/MiCup/MiCup/cyberdog_dev
python3 -m py_compile main.py && echo "COMPILE OK"
```
Expected: `COMPILE OK`

- [ ] **Step 3: 验证 argparse（不触发 rclpy）**

Run:
```bash
cd /Users/phoen1xcode/Projects/MiCup/MiCup/cyberdog_dev
python3 main.py --help
```
Expected: 打印 usage，含 `--mode` 和 `--stages`，无 ImportError（因为 rclpy import 在 main() 内）。

- [ ] **Step 4: 验证 parse_stages 逻辑**

Run:
```bash
cd /Users/phoen1xcode/Projects/MiCup/MiCup/cyberdog_dev
python3 -c "from main import parse_stages; print(parse_stages('1-6')); print(parse_stages('1,3,5')); print(parse_stages('2-4'))"
```
Expected: `[1, 2, 3, 4, 5, 6]` / `[1, 3, 5]` / `[2, 3, 4]`

- [ ] **Step 5: Commit**

```bash
cd /Users/phoen1xcode/Projects/MiCup/MiCup
git add cyberdog_dev/main.py
git commit -m "feat(cyberdog_dev): add competition entrypoint with tick loop"
```

**Gazebo 待验证：** `python3 main.py --mode sim --stages 1` 完整跑通赛段一。

---

## Task 16: 重写 scripts/launch.sh 与 README.md

**Files:**
- Create: `scripts/launch.sh`
- Create: `README.md` （覆盖现有的 `cyberdog_dev/README.md`——注意：现有 README 是未提交的 untracked 文件，本任务用新内容覆盖它）

- [ ] **Step 1: 创建 `scripts/launch.sh`**

```bash
#!/bin/bash
# cyberdog_dev 启动脚本：感知节点 + odom 广播 + 主程序。
# 用法：bash scripts/launch.sh sim 1
#       bash scripts/launch.sh real 1-6

set -e
MODE="${1:-sim}"
STAGES="${2:-1-6}"
DEV_DIR="$(cd "$(dirname "$0")/.." && pwd)"
cd "$DEV_DIR"

cleanup() {
    echo "停止所有后台进程..."
    kill $(jobs -p) 2>/dev/null || true
}
trap cleanup SIGINT SIGTERM

echo "[1/3] 启动激光雷达走廊检测节点..."
python3 -m perception.lidar --mode "$MODE" &

echo "[2/3] 启动 odom -> TF 广播..."
python3 -m core.odom_broadcaster --mode "$MODE" &

sleep 2  # 等感知节点就绪

echo "[3/3] 启动主程序 (mode=$MODE stages=$STAGES)..."
python3 main.py --mode "$MODE" --stages "$STAGES"

cleanup
```

- [ ] **Step 2: 赋予执行权限**

Run:
```bash
cd /Users/phoen1xcode/Projects/MiCup/MiCup/cyberdog_dev
chmod +x scripts/launch.sh
bash -n scripts/launch.sh && echo "SHELL SYNTAX OK"
```
Expected: `SHELL SYNTAX OK`

- [ ] **Step 3: 覆盖 `README.md`**

```markdown
# cyberdog_dev — 2026 小米杯参赛代码

四层架构（Core / Perception / Stage / App），详见
`../docs/superpowers/specs/2026-05-14-cyberdog-dev-architecture-design.md`。

## 目录结构

```
core/         底层控制与位姿（LCM 控制、TF 位姿、动作原语、闭环对齐、Stage 基类）
perception/   感知节点（激光雷达走廊检测、PerceptionHub）
stages/       每个赛段一个 Phase 状态机
config/       sim/real topic 配置、各赛段参数
main.py       入口：解析参数、组装 context、跑 tick 循环
scripts/      启动脚本
tests/        纯逻辑单元测试（pytest）
```

## 运行（需 ROS2 环境 / Gazebo 容器）

```bash
bash scripts/launch.sh sim 1        # 仿真跑赛段 1
bash scripts/launch.sh sim 1-6      # 仿真跑全部赛段
bash scripts/launch.sh real 1       # 实机跑赛段 1
```

## 本机开发（macOS，无 ROS2）

```bash
python3 -m pip install -r requirements-dev.txt
python3 -m pytest                   # 跑纯逻辑单元测试
python3 main.py --help              # 验证入口 argparse
```

## 当前进度

- [x] Plan 1：Core/Perception 地基 + Stage 1（石径探路）
- [ ] Plan 2-6：Stage 2-6、语音、自定义步态、其余检测器

## 迁移来源

- `core/` 多数模块来自 `example/demo1/motion/utils/`（已验证）
- `stages/stage1_stone_path.py` 重构自 `example/stage1/stage1.py`
- 旧的 `cyberdog_competition/` 包暂保留，待 Stage 1 在 Gazebo 验证后清理
```

- [ ] **Step 4: 跑全部单元测试确认无回归**

Run:
```bash
cd /Users/phoen1xcode/Projects/MiCup/MiCup/cyberdog_dev
python3 -m pytest -v
```
Expected: 全部 passed（lane_follow 6 + stage_base 6 + config 4 + hub 2 + stage_context 3 + stage1 4 = 25；以实际为准，关键是无 FAIL）

- [ ] **Step 5: Commit**

```bash
cd /Users/phoen1xcode/Projects/MiCup/MiCup
git add cyberdog_dev/scripts/launch.sh cyberdog_dev/README.md
git commit -m "docs(cyberdog_dev): add launch script and rewrite README for new architecture"
```

---

## 完成标准

本计划完成后应达到：

1. `cyberdog_dev/` 下 `core/` `perception/` `stages/` `config/` 四个新目录就位，结构符合 spec 第 5 节。
2. `python3 -m pytest` 全绿（纯逻辑单元测试覆盖 lane_follow / stage_base / config / hub / stage_context / stage1 phase 转移）。
3. 所有迁移文件 `python3 -m py_compile` 通过。
4. `python3 main.py --help` 正常输出。
5. 旧的 `cyberdog_competition/`、`hello_*.py`、旧 `main.py` 等**原样保留未删除**（纯增量）。

**未覆盖（后续计划）：** Gazebo 中实际跑通 Stage 1、Stage 2-6、`core/voice.py`、`core/gaits/`、`core/ekf_fusion.py`、`perception/` 其余检测器、清理旧文件。

**关键 Gazebo 验证清单**（执行完本计划后，进容器手动验证）：
- `python3 -m core.odom_broadcaster --mode sim` 发布 TF
- `python3 -m perception.lidar --mode sim` 发布 `/perception/lidar_corridor`
- `python3 main.py --mode sim --stages 1` 完整走通赛段一
