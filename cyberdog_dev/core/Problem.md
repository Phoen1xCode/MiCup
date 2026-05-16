# robot_ctrl.py 实现 vs 官方代码：逻辑差异分析

> 对比对象：`core/robot_ctrl.py`（你的实现） vs `example/loco_hl_example/basic_motion/main.py` + `customized_gait/main.py`（官方示例）

---

## 差异 1：`_response_handler` 中对 gait_id 的检查 — 更正确

**官方代码** (`basic_motion/main.py:105-110`)：
```python
def msg_handler(self, channel, data):
    self.rec_msg = robot_control_response_lcmt().decode(data)
    if self.rec_msg.order_process_bar >= 95:
        self.mode_ok = self.rec_msg.mode
    else:
        self.mode_ok = 0
```
然后 `Wait_finish(mode, gait_id)` 检查 `self.mode_ok == mode and self.gait_ok == gait_id`。

**问题**：官方代码的 `gait_ok` **从未被更新**，永远是初始值 `0`。这意味着 `Wait_finish(62, 2)` 永远不会返回 True（因为 `gait_ok` 始终是 0，不等于 2）。官方示例碰巧只用了 `gait_id=0` 的动作所以没暴露这个 bug。

**你的代码** (`robot_ctrl.py:290-308`)：
```python
if (self._expected_completion_mode != -1
    and msg.mode == self._expected_completion_mode
    and msg.gait_id == self._expected_completion_gait_id
    and msg.order_process_bar >= 95):
```

同时检查了 `mode` 和 `gait_id`，是**更正确**的实现。不会有问题。

---

## 差异 2：心跳发送机制 — 等效，频率更高

**官方代码** (`send_publish` 方法)：
```python
while self.runing:
    self.send_lock.acquire()
    if self.delay_cnt > 20:  # 每 20*5ms = 100ms 发一次 = 10Hz
        self.lc_s.publish("robot_control_cmd", self.cmd_msg.encode())
        self.delay_cnt = 0
    self.delay_cnt += 1
    self.send_lock.release()
    time.sleep(0.005)
```
- 只在 `delay_cnt > 20` 时才发送，否则空转
- `Send_cmd` 设置 `delay_cnt = 50` 强制立即发送下一次
- 实际频率约 10Hz

**你的代码** (`_cmd_send_loop`)：
```python
while self._is_running:
    with self._cmd_lock:
        # 深拷贝 _current_lcm_cmd → cmd_to_send_on_loop
        # 递增 life_count
    self._lc_publisher.publish(channel, payload)
    time.sleep(0.05)  # 50ms = 20Hz
```
- 每次循环都发送，无条件
- 20Hz 频率

**影响**：你的代码发送更频繁（20Hz vs 10Hz），这**没有问题**，反而更可靠。控制板能处理更高频率的心跳。

---

## 差异 3：自定义步态 gait_id=110 时的等待方式 — 更稳健

**官方代码** (`customized_gait/main.py:81-99`)：
```python
for step in steps['step']:
    cmd_msg.mode = step['mode']
    cmd_msg.gait_id = step['gait_id']
    cmd_msg.duration = step['duration']
    cmd_msg.life_count += 1
    # ... 设置所有字段 ...
    lcm_cmd.publish("robot_control_cmd", cmd_msg.encode())
    time.sleep(0.1)  # 每步之间只 sleep 100ms
```
- 官方对**所有步骤**（包括 gait_id=110）都只 sleep 100ms
- 然后继续下一步，让控制板自己按 duration 执行
- 最后发 15 秒心跳维持

**你的代码** (`robot_ctrl.py:654-728`)：
```python
if gait_id == 110:
    # 设置命令字段 ...
    self._get_next_master_life_count()
    if duration_ms > 0:
        time.sleep(duration_ms / 1000.0)  # 等完整个 duration
    else:
        time.sleep(0.1)
```
- 你对 gait_id=110 的步骤，sleep 了**完整的 duration 时间**

**分析 `Usergait_List.toml` 中的实际步骤**：
```toml
# Step 1: mode=12, gait_id=0, duration=5000  (站立)
# Step 2: mode=62, gait_id=110, duration=4320 (自定义步态)
# Step 3: mode=7, gait_id=0, duration=4000   (趴下)
```

- Step 1 (mode=12)：你的代码调用 `execute_discrete_action(wait_for_completion=True)`，等 `order_process_bar >= 95`。**正确**。
- Step 2 (gait_id=110)：你 sleep 4.32 秒。控制板按 duration 执行。**等价于官方**。
- Step 3 (mode=7)：你调用 `execute_discrete_action(wait_for_completion=True)`。PureDamper 很快完成。**正确**。

**结论**：这个差异**不会导致故障**，你的实现实际上更稳健（等确认完成再继续）。

---

## 差异 4：自定义步态中 `_get_next_master_life_count()` 的冗余调用 — 无害

你的代码在 gait_id=110 分支里手动调了一次 `_get_next_master_life_count()`（721行）。但发送循环已经在每次迭代中递增 `life_count` 了。这个额外调用只是让 `life_count` 多跳了一个值，**不会影响功能**，但属于冗余代码。

---

## 差异 5：步态文件 ACK 等待 — 可能超时，需实测

**官方代码**：
```python
lcm_usergait.publish("user_gait_file", usergait_msg.encode())
time.sleep(0.5)  # 直接 sleep，不等 ACK
```

**你的代码**：
```python
self._lc_publisher.publish(self._user_gait_file_channel, usergait_msg.encode())
if self._gait_file_ack_event.wait(timeout=wait_for_file_ack_timeout_sec):
    # 检查 success
```

你在 `user_gait_result` 通道上订阅了 ACK。**问题**：控制板是否真的会在 `user_gait_result` 通道上回复 `file_recv_lcmt`？这取决于固件版本。如果固件不发 ACK，你的代码会**等待 5 秒后超时返回 False**，整个自定义步态流程会中断。

**建议**：在实际环境中测试这个 ACK 机制。如果固件不支持 ACK，应该去掉等待，改用 `time.sleep(0.5)` 跟官方一致。

---

## 差异 6：`pos_des` 初始化处理 — 基本没问题

**官方代码**：`mode=12` 站立时，不设 `pos_des`，用控制板默认值。

**你的代码** `execute_discrete_action` (370-383行)：
```python
current_body_height = self._current_lcm_cmd.pos_des[2] if len(self._current_lcm_cmd.pos_des) == 3 else 0.28
if pos_des is not None:
    self._current_lcm_cmd.pos_des = list(pos_des)
else:
    self._current_lcm_cmd.pos_des = [0.0, 0.0, current_body_height if mode != 7 else 0.0]
```

`robot_control_cmd_lcmt.__init__` 初始化 `pos_des = [0.0, 0.0, 0.0]`。如果在 `set_velocity_command` 之前就调用 `execute_discrete_action`（比如第一个动作就是站立），`current_body_height` 会是 `0.0`。但这**不影响站立动作**——站立时 `pos_des` 不是关键参数，控制板自己决定站立高度。

---

## 总结：能否正常运转？

| 差异点 | 影响 | 是否会导致故障 |
|---|---|---|
| gait_id 检查更严格 | 更正确 | 不会 |
| 20Hz vs 10Hz 心跳 | 更可靠 | 不会 |
| gait_id=110 等 duration | 更稳健 | 不会 |
| life_count 冗余递增 | 无害 | 不会 |
| **步态文件 ACK 等待** | **可能超时** | **可能，取决于固件** |
| pos_des 初始值 0 | 不影响站立 | 不会 |

**结论**：实现在逻辑上是正确的，大部分差异是**更稳健**的改进（多检查了 gait_id、等确认完成再继续）。唯一需要实测确认的是**步态文件 ACK 机制**——如果控制板固件不发 ACK，`load_and_execute_custom_gait` 会在文件发送步骤超时失败。建议在 Gazebo 仿真中先测试这个流程。
