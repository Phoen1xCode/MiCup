# 重构计划：`load_and_execute_custom_gait` 函数

> 目标：将 `core/robot_ctrl.py` 中 280 行的 `load_and_execute_custom_gait` 拆分为职责单一的小函数

---

## 一、新对话需要阅读的文件

按阅读顺序排列，标注每个文件的作用：

### 1. 理解官方协议（必读）

| 文件 | 作用 |
|---|---|
| `example/loco_hl_example/customized_gait/main.py` | **官方自定义步态示例**，理解参数转换、文件发送、序列执行的完整流程 |
| `example/loco_hl_example/customized_gait/Gait_Params_moonwalk.toml` | 步态参数源文件格式（`type=usergait` 的结构） |
| `example/loco_hl_example/customized_gait/Gait_Def_moonwalk.toml` | 步态定义文件格式（关节轨迹数据） |
| `example/loco_hl_example/customized_gait/Usergait_List.toml` | 步态执行序列格式（mode/gait_id/duration） |
| `example/loco_hl_example/basic_motion/main.py` | 官方基础运动示例，理解 mode/gait_id/duration/life_count 的含义 |

### 2. 理解当前实现（必读）

| 文件 | 作用 |
|---|---|
| `core/robot_ctrl.py` | **被重构的目标文件**，重点看 `load_and_execute_custom_gait`（约 516-825 行） |
| `core/basic_action.py` | 基础动作封装层，重构后新函数应放在这里 |
| `core/lcm_type/file_send_lcmt.py` | LCM 文件发送消息类型定义 |
| `core/lcm_type/file_recv_lcmt.py` | LCM 文件接收（ACK）消息类型定义 |
| `core/lcm_type/robot_control_cmd_lcmt.py` | LCM 命令消息类型，理解所有字段（mode/gait_id/vel_des 等） |

### 3. 理解调用关系（必读）

| 文件 | 作用 |
|---|---|
| `core/stage_context.py` | StageContext 依赖容器，理解 `ctx.dog` 如何获取 RobotCtrl 实例 |
| `stages/` 目录下的赛段文件 | 查看是否有赛段调用了自定义步态 |

### 4. 参考资料（可选）

| 文件 | 作用 |
|---|---|
| `docs/运动控制.md` | mode/gait_id 含义速查表 |
| `wiki/` 目录下的相关文档 | 如果有的话 |

---

## 二、现有代码问题清单

### 问题 1：职责过多，函数过长（280 行）

一个函数做了三件不同的事：

- **Step 1**（570-631 行）：参数格式转换（`Gait_Params_xxx.toml` → `_full.toml`）
- **Step 2+3**（633-707 行）：文件发送 + ACK 等待（重复逻辑）
- **Step 4**（709-825 行）：步态序列执行

### 问题 2：ACK 等待机制可能失败

官方代码直接发文件 + `time.sleep(0.5)`，不等 ACK。当前实现在 `user_gait_result` 通道上等 ACK，如果固件不发 ACK，整个函数超时返回 False。

### 问题 3：Step 4 的 gait_id=110 分支冗余

733-794 行直接操作 `_current_lcm_cmd` 字段，但 `Usergait_List.toml` 中这些字段通常都是 0（实际参数已编码在 `full_params` 中）。大量 `if "xxx" in step_cmd_data` 判断是无用代码。

### 问题 4：`_get_next_master_life_count()` 冗余调用

796 行手动调用，但 `_cmd_send_loop` 已在每次循环中递增 `life_count`。

### 问题 5：Step 4 失败时仍然返回 True

809-816 行，`execute_discrete_action` 返回 False 时只打日志，不 `return False`。

### 问题 6：docstring 中有不存在的参数

`action_wait_timeout_sec` 在签名中不存在。

### 问题 7：Step 2 和 Step 3 代码高度重复

两段 ACK 等待逻辑几乎完全一样，只是文件路径不同。

---

## 三、重构方案

### 3.1 拆分后的函数结构

```
core/robot_ctrl.py
├── _convert_gait_params_to_full()    ← Step 1: 参数转换
├── _send_gait_file_with_ack()        ← Step 2/3: 文件发送 + ACK
└── execute_gait_sequence()           ← Step 4: 序列执行

core/basic_action.py
└── execute_custom_gait_sequence()    ← 编排函数：调用上面三个函数
```

### 3.2 详细设计

#### 函数 1：`RobotCtrl._convert_gait_params_to_full`

```python
def _convert_gait_params_to_full(
    self,
    gait_params_file_path: str,
    output_dir: str = ".",
) -> str:
    """
    读取 Gait_Params_xxx.toml，转换为控制板可接受的 _full.toml 格式。

    转换规则：
    - type=usergait → mode=11, gait_id=110
    - body_vel_des → vel_des
    - body_pos_des[0:3] → rpy_des, body_pos_des[3:6] → pos_des
    - landing_pos_des → foot_pose + ctrl_point
    - step_height: 4元数组 → 2元编码 (ceil(h*1000) + ceil(h*1000)*1000)
    - weight → acc_des
    - use_mpc_traj → value
    - landing_gain → contact (floor(gain*10))
    - mu → ctrl_point[2]

    :return: 生成的 _full.toml 文件路径
    :raises FileNotFoundError: 源文件不存在
    :raises Exception: 解析或写入错误
    """
```

- 输入：`Gait_Params_xxx.toml` 路径
- 输出：`Gait_Params_xxx_full.toml` 路径
- 职责：纯数据转换，不涉及 LCM 通信

#### 函数 2：`RobotCtrl._send_gait_file_with_ack`

```python
def _send_gait_file_with_ack(
    self,
    file_path: str,
    timeout_sec: float = 5.0,
    use_ack: bool = True,
) -> bool:
    """
    通过 user_gait_file 通道发送步态文件。

    :param file_path: 要发送的文件路径
    :param timeout_sec: ACK 等待超时时间
    :param use_ack: 是否等待 ACK（False 则用 time.sleep(0.5) 代替）
    :return: 发送成功返回 True
    """
```

- 输入：文件路径
- 输出：是否成功
- 职责：文件读取 + LCM 发送 + ACK 等待
- `use_ack=False` 时回退到官方的 `time.sleep(0.5)` 方式

#### 函数 3：`RobotCtrl.execute_gait_sequence`

```python
def execute_gait_sequence(
    self,
    user_gait_list_path: str,
) -> bool:
    """
    按 Usergait_List.toml 的顺序执行步态序列。

    对每个步骤：
    - gait_id=110：设置 mode/gait_id/duration，sleep duration
    - 其他 gait_id：调用 execute_discrete_action(wait_for_completion=True)

    :return: 全部成功返回 True，任一失败返回 False
    """
```

- 输入：`Usergait_List.toml` 路径
- 输出：是否全部成功
- 职责：读取序列文件，按顺序执行

#### 函数 4：`basic_action.execute_custom_gait_sequence`（编排函数）

```python
def execute_custom_gait_sequence(
    ctrl,
    gait_params_file_path: str,
    gait_def_file_path: str,
    user_gait_list_file_path: str,
    base_working_dir: str = ".",
    use_ack: bool = True,
    ack_timeout_sec: float = 5.0,
) -> bool:
    """
    自定义步态完整流程：转换参数 → 发送定义 → 发送参数 → 执行序列。

    对应官方 customized_gait/main.py 的完整流程。
    """
    # Step 1: 参数转换
    full_params_path = ctrl._convert_gait_params_to_full(
        gait_params_file_path, base_working_dir
    )

    # Step 2: 发送步态定义文件
    if not ctrl._send_gait_file_with_ack(gait_def_file_path, ack_timeout_sec, use_ack):
        return False
    time.sleep(0.5)

    # Step 3: 发送 full 参数文件
    if not ctrl._send_gait_file_with_ack(full_params_path, ack_timeout_sec, use_ack):
        return False
    time.sleep(0.1)

    # Step 4: 执行序列
    return ctrl.execute_gait_sequence(user_gait_list_file_path)
```

---

## 四、重构步骤

### Step 1：提取 `_convert_gait_params_to_full`

1. 从 `load_and_execute_custom_gait` 的 570-631 行提取参数转换逻辑
2. 放入 `RobotCtrl` 类中作为私有方法
3. 返回生成的 `_full.toml` 文件路径
4. 单独测试：给一个 `Gait_Params_moonwalk.toml`，验证生成的 `_full.toml` 内容正确

### Step 2：提取 `_send_gait_file_with_ack`

1. 从 633-707 行提取文件发送 + ACK 等待逻辑
2. 添加 `use_ack` 参数，`False` 时用 `time.sleep(0.5)` 替代 ACK 等待
3. 单独测试：发送一个文件，验证 ACK 机制（或 sleep 回退）

### Step 3：提取 `execute_gait_sequence`

1. 从 709-825 行提取序列执行逻辑
2. 简化 gait_id=110 分支：只设置 mode/gait_id/duration，去掉冗余字段拷贝
3. 修复失败时的返回值：`execute_discrete_action` 返回 False 时 `return False`
4. 单独测试：给一个 `Usergait_List.toml`，验证序列执行

### Step 4：在 `basic_action.py` 中创建编排函数

1. 创建 `execute_custom_gait_sequence`，按顺序调用上面三个函数
2. 保持与原函数相同的参数签名（兼容现有调用方）

### Step 5：删除原函数

1. 从 `RobotCtrl` 中删除 `load_and_execute_custom_gait`
2. 更新所有调用方（搜索 `load_and_execute_custom_gait`）
3. 运行测试确保不破坏现有功能

---

## 五、重构后的调用关系

```
Stage 子类
    │
    ▼
basic_action.execute_custom_gait_sequence(ctrl, params, def, list)
    │
    ├── ctrl._convert_gait_params_to_full(params, dir) → full_path
    │
    ├── ctrl._send_gait_file_with_ack(def, timeout, use_ack)
    │       └── _lcm_publisher.publish(...)
    │       └── _gait_file_ack_event.wait(...) 或 time.sleep(0.5)
    │
    ├── time.sleep(0.5)
    │
    ├── ctrl._send_gait_file_with_ack(full_path, timeout, use_ack)
    │
    ├── time.sleep(0.1)
    │
    └── ctrl.execute_gait_sequence(list)
            └── 对每个步骤：
                ├── gait_id=110 → 直接设置 _current_lcm_cmd + sleep
                └── 其他 → ctrl.execute_discrete_action(wait=True)
```

---

## 六、测试计划

### 单元测试（不需要机器人）

| 测试 | 验证内容 |
|---|---|
| `test_convert_gait_params` | 给定 `Gait_Params_moonwalk.toml`，验证生成的 `_full.toml` 字段正确 |
| `test_convert_gait_params_missing_file` | 文件不存在时抛出 `FileNotFoundError` |
| `test_convert_gait_params_unknown_type` | 未知 type 被跳过，不报错 |

### 集成测试（需要 Gazebo 仿真）

| 测试 | 验证内容 |
|---|---|
| `test_send_gait_file_no_ack` | `use_ack=False` 时文件发送成功 |
| `test_execute_gait_sequence` | 完整流程：转换 → 发送 → 执行 moonwalk |
| `test_execute_gait_sequence_failure` | 某步骤失败时返回 False |
