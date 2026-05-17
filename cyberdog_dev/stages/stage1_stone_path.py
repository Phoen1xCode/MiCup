"""Stage 1 · 石径探路 -- Phase 状态机

开环行走 + TickMotion 动作原语，不依赖视觉循迹。
RECOVERY_STAND → STABILIZE → STRAIGHT_TO_BEND → TURNING → STRAIGHT_TO_EXIT → DONE
"""

import time  # 计时：记录 phase 开始时间、计算 elapsed
from enum import Enum, auto  # 定义状态机的 Phase 枚举
from pathlib import Path  # 定位 config 目录的 TOML 配置文件

from core.framework.stage import (  # 赛段基类 + 状态枚举(RUNNING/SUCCEEDED/FAILED)
    Stage,
    StageStatus,
)
from core.tick_motion import TickMotion  # tick 兼容的非阻塞动作原语（开环+闭环）


# =====================================================================
# 状态机 Phase 定义
# 按执行顺序：站立 → 稳定 → 直走到弯 → 转弯 → 直走到出口 → 完成
# =====================================================================
class Phase(Enum):
    RECOVERY_STAND = auto()  # 起始状态：机器人从趴下恢复站立，只发一次离散动作
    STABILIZE = auto()  # 站立后原地等待，让 IMU/关节稳定，避免初始抖动
    STRAIGHT_TO_BEND = auto()  # 沿石板路直走，直到估计到达弯道位置
    TURNING = auto()  # 在弯道处原地旋转指定角度
    STRAIGHT_TO_EXIT = auto()  # 转弯后继续直走，离开弯道虚线进入下一赛段
    DONE = auto()  # 终态：返回 SUCCEEDED


# =====================================================================
# Stage1 赛段实现
# =====================================================================
class Stage1StonePath(Stage):
    stage_id = 1  # 赛段编号，用于注册表索引
    name = "石径探路"  # 赛段名称，用于日志输出

    def __init__(self, ctx):
        super().__init__(
            ctx
        )  # 调用基类构造器，保存 ctx（dog/pose/perception/voice/logger）
        self.motion = TickMotion(
            ctx
        )  # TickMotion 实例：所有开环/闭环动作通过它执行，非阻塞
        self.phase = Phase.RECOVERY_STAND  # 状态机初始状态：从站立恢复开始
        self.phase_start = 0.0  # 当前 phase 的开始时间戳（monotonic），用于计算 elapsed
        self.turn_direction = 1.0  # 转弯方向：1.0=左转，-1.0=右转，从 TOML 读取
        self.recovery_stand_requested = False  # 标记站立动作是否已发送，避免重复发送

        # 加载 stage_params.toml 中 [stage1] 的参数（速度、距离、时间等）
        config_dir = (
            Path(__file__).resolve().parent.parent / "config"
        )  # cyberdog_dev/config/
        from config.loader import load_stage_params

        self.p = load_stage_params(config_dir / "stage_params.toml", stage_id=1)

    def on_enter(self) -> None:
        """进入赛段时调用一次。初始化时间、设原点。"""
        # 记录赛段绝对开始时间，用于 max_duration_sec 超时判断
        self.start_time = time.monotonic()
        self.phase_start = self.start_time  # 当前 phase 的开始时间 = 赛段开始时间
        self.recovery_stand_requested = False  # 重置站立标记，允许新一次站立动作
        self.ctx.pose.set_origin_here()  # 把当前机器人位姿设为坐标原点，后续位姿相对此处计算
        self.ctx.logger.info(f"[{self.name}] 进入，phase=RECOVERY_STAND")

    def max_duration_sec(self) -> float:
        """赛段超时上限（秒），超过后 main.py 的 run_stage() 会强制结束"""
        return float(self.p["max_time"])

    def _switch(self, phase: Phase) -> None:
        """切换状态机 phase，重置 phase_start 计时器，输出日志"""
        self.phase = phase  # 更新当前 phase
        self.phase_start = time.monotonic()  # 重置该 phase 的起始时间
        self.ctx.logger.info(f"[{self.name}] -> {phase.name}")

    def tick(self) -> StageStatus:
        """状态机主循环，由 main.py 以 ~20Hz (50ms 间隔) 调用。
        每次调用必须尽快返回，不能阻塞。
        返回 RUNNING 表示继续，SUCCEEDED 表示赛段完成。"""

        # 计算当前 phase 已经过的时间（秒）
        elapsed = time.monotonic() - self.phase_start

        # ==============================================================
        # Phase 1: RECOVERY_STAND — 恢复站立
        # 机器人起始状态是趴下的，需要先站起来。
        # execute_discrete_action(mode=12, gait_id=0) 是官方的"恢复站立"离散动作。
        # wait_for_completion=True 会阻塞直到机器人站稳（由 LCM 响应确认）。
        # 只发一次，通过 recovery_stand_requested 标记防止重复发送。
        # 站立后额外等待 stand_time 秒让传感器稳定。
        # ==============================================================
        if self.phase == Phase.RECOVERY_STAND:
            if not self.recovery_stand_requested:  # 第一次进入时才发送站立指令
                self.ctx.dog.execute_discrete_action(
                    mode=12,
                    gait_id=0,
                    wait_for_completion=True,  # mode=12: RECOVERY_STAND
                )
                self.recovery_stand_requested = True  # 标记已发送，后续 tick 跳过
            if (
                time.monotonic() - self.phase_start >= self.p["stand_time"]
            ):  # 等待 stand_time 秒
                self._switch(Phase.STABILIZE)  # → 进入稳定阶段
            return StageStatus.RUNNING  # 本 phase 未完成，继续

        # ==============================================================
        # Phase 2: STABILIZE — 原地稳定
        # 持续发送零速度指令，让机器人保持站立不动。
        # 等待 stabilize_time 秒后进入直走阶段。
        # 目的是让 IMU、关节编码器等传感器读数稳定下来。
        # ==============================================================
        if self.phase == Phase.STABILIZE:
            self.ctx.dog.set_velocity_command(0.0, 0.0, 0.0)  # 零速：不走不转
            if elapsed >= self.p["stabilize_time"]:  # 稳定时间到
                self._switch(Phase.STRAIGHT_TO_BEND)  # → 进入直走阶段
            return StageStatus.RUNNING

        # ==============================================================
        # Phase 3: STRAIGHT_TO_BEND — 沿石板路直走到弯道
        # 开环行走：以 forward_speed 匀速前进 straight_distance 米。
        # TickMotion.move_straight() 内部计算 duration = distance / speed，
        # 每次 update() 检查时间是否到达，到达后自动停速并返回 True。
        # is_idle() 确保只触发一次动作，不会重复发送。
        # ==============================================================
        if self.phase == Phase.STRAIGHT_TO_BEND:
            if self.motion.is_idle():  # 动作队列为空，首次触发
                self.motion.move_straight(
                    self.p["straight_distance"],  # 目标距离 (m)，从 TOML 读取
                    self.p["forward_speed"],  # 前进速度 (m/s)，石板路保守值
                )
            if self.motion.update():  # 推进一 tick，True = 走完了
                self.turn_direction = float(
                    self.p["turn_direction"]
                )  # 记录转弯方向（左/右）
                self._switch(Phase.TURNING)  # → 进入转弯阶段
            return StageStatus.RUNNING

        # ==============================================================
        # Phase 4: TURNING — 弯道处原地转弯
        # 开环旋转：以 turn_speed_dps 角速度原地转 turn_angle_deg 度。
        # TickMotion.turn_in_place() 内部计算旋转所需时间，
        # 每次 update() 检查时间是否到达，到达后自动停速并返回 True。
        # 不使用闭环校准，石板路弯道开环精度足够。
        # ==============================================================
        if self.phase == Phase.TURNING:
            if self.motion.is_idle():  # 动作队列为空，首次触发
                self.motion.turn_in_place(
                    float(self.p["turn_angle_deg"]),  # 转弯角度 (°)，通常 90
                    float(self.p["turn_speed_dps"]),  # 角速度 (°/s)，石板路保守值
                )
            if self.motion.update():  # 推进一 tick，True = 转完了
                self._switch(Phase.STRAIGHT_TO_EXIT)  # → 进入出弯直走阶段
            return StageStatus.RUNNING

        # ==============================================================
        # Phase 5: STRAIGHT_TO_EXIT — 转弯后直走离开弯道
        # 与 STRAIGHT_TO_BEND 同理，开环直走 exit_distance 米。
        # 走完后后腿足底已离开弯道虚线，赛段结束。
        # ==============================================================
        if self.phase == Phase.STRAIGHT_TO_EXIT:
            if self.motion.is_idle():  # 动作队列为空，首次触发
                self.motion.move_straight(
                    self.p["exit_distance"],  # 出弯距离 (m)
                    self.p["exit_speed"],  # 出弯速度 (m/s)
                )
            if self.motion.update():  # 推进一 tick，True = 走完了
                self._switch(Phase.DONE)  # → 进入完成状态
            return StageStatus.RUNNING

        # ==============================================================
        # Phase 6: DONE — 赛段完成
        # 发零速确保机器人停住，返回 SUCCEEDED 通知 main.py 结束本赛段。
        # 理论上 DONE 状态下 tick 不会被调用，此为安全兜底。
        # ==============================================================
        self.ctx.dog.set_velocity_command(0.0, 0.0, 0.0)  # 保险停速
        return StageStatus.SUCCEEDED  # 告诉框架：赛段成功完成

    def on_exit(self) -> None:
        """退出赛段时调用。清理动作状态，确保机器人停住。"""
        self.motion.stop()  # 取消 TickMotion 当前动作 + 发零速
        self.ctx.logger.info(f"[{self.name}] 退出")
