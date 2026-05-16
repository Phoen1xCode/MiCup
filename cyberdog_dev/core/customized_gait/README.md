# core/gaits/

特殊步态适配器。每个文件包一种"特殊动作"，给 Stage 调用，内部最终也是发 LCM 速度/步态参数。

## 文件说明

| 文件 | 作用 |
|---|---|
| `__init__.py` | 包标记。 |
| `low_walk.py` | `execute_low_walk(dog, duration_sec, speed)`。让狗以低身高（`body_height=0.16m`）、低抬腿（`step_height=0.03m`）行走指定秒数，专门用于 **Stage 4 钻限高杆**。结束自动 `stop()`。 |

## 迁移说明

**本目录全部为新增，template 中没有对应文件。**

但思路与 template 的"自定义步态"模块相关：

- 源项目通过 `template/motion/special_action/limit/*.toml`（`Gait_Def_limit.toml`、`Gait_Params_limit.toml`、`Usergait_List.toml`）下发完整的"钻限高杆"自定义步态，并在 `template/custom_tasks.py` 的 `run_height_limit_task` 中调用 `basic.execute_custom_gait_sequence()` 执行。
- 本目录的 `low_walk.py` 采用**更轻量的方案**：不下发自定义 TOML，直接调 `dog.set_velocity()` 把 `body_height` 和 `step_height` 设小一些，靠通用步态低身通过。

> 如果 Gazebo 验证发现"低身行走"不够通过限高杆，可以把源项目的三个 TOML 文件迁回（建议放到 `core/gaits/limit_pole/`），并在这里新增一个 `limit_pole.py` 调用 `dog.load_and_execute_custom_gait()`。
