"""TickMotion — tick 兼容的非阻塞动作原语。

所有 Stage 通过 self.motion = TickMotion(ctx) 复用，在 tick() 中调用：
    if self.motion.is_idle():
        self.motion.move_straight(2.0, 0.2)
    if self.motion.update():   # 每 tick 调用一次，返回 True 表示动作完成
        self._switch(next_phase)
"""

from __future__ import annotations

import math
import time


def _normalize_angle(rad: float) -> float:
    while rad > math.pi:
        rad -= 2 * math.pi
    while rad < -math.pi:
        rad += 2 * math.pi
    return rad


class TickMotion:
    """tick 兼容的非阻塞动作原语，所有赛段复用。"""

    def __init__(self, ctx):
        self.ctx = ctx
        self._action = None

    # ------------------------------------------------------------------
    # 公开控制接口
    # ------------------------------------------------------------------

    def is_idle(self) -> bool:
        return self._action is None

    def stop(self) -> None:
        self.ctx.dog.set_velocity_command(0.0, 0.0, 0.0)
        self._action = None

    def update(self) -> bool:
        """每 tick 调用一次。返回 True 表示当前动作已完成。"""
        if self._action is None:
            return True
        kind = self._action["type"]
        if kind.startswith("open_"):
            return self._update_open_loop()
        if kind == "closed_yaw":
            return self._update_closed_yaw()
        if kind == "closed_pos":
            return self._update_closed_pos()
        return True

    # ------------------------------------------------------------------
    # 开环动作
    # ------------------------------------------------------------------

    def move_straight(
        self,
        distance_m: float,
        speed_mps: float,
        body_height: float = 0.28,
        gait_id: int = 26,
    ) -> None:
        if abs(distance_m) < 1e-4 or speed_mps <= 0:
            return
        vx = speed_mps if distance_m > 0 else -speed_mps
        self.ctx.dog.set_velocity_command(vx, 0.0, 0.0, body_height=body_height, gait_id=gait_id)
        self._action = {
            "type": "open_straight",
            "vx": vx, "vy": 0.0, "wz": 0.0,
            "body_height": body_height, "gait_id": gait_id,
            "start": time.monotonic(),
            "duration": abs(distance_m) / speed_mps,
        }

    def turn_in_place(
        self,
        angle_deg: float,
        angular_speed_dps: float,
        body_height: float = 0.28,
        gait_id: int = 26,
    ) -> None:
        if abs(angle_deg) < 0.1 or angular_speed_dps <= 0:
            return
        wz = math.radians(angular_speed_dps) if angle_deg > 0 else -math.radians(angular_speed_dps)
        self.ctx.dog.set_velocity_command(0.0, 0.0, wz, body_height=body_height, gait_id=gait_id)
        self._action = {
            "type": "open_turn",
            "vx": 0.0, "vy": 0.0, "wz": wz,
            "body_height": body_height, "gait_id": gait_id,
            "start": time.monotonic(),
            "duration": math.radians(abs(angle_deg)) / math.radians(angular_speed_dps),
        }

    def move_lateral(
        self,
        distance_m: float,
        speed_mps: float,
        body_height: float = 0.28,
        gait_id: int = 26,
    ) -> None:
        if abs(distance_m) < 1e-4 or speed_mps <= 0:
            return
        vy = speed_mps if distance_m > 0 else -speed_mps
        self.ctx.dog.set_velocity_command(0.0, vy, 0.0, body_height=body_height, gait_id=gait_id)
        self._action = {
            "type": "open_lateral",
            "vx": 0.0, "vy": vy, "wz": 0.0,
            "body_height": body_height, "gait_id": gait_id,
            "start": time.monotonic(),
            "duration": abs(distance_m) / speed_mps,
        }

    def walk_in_arc(
        self,
        linear_speed_mps: float,
        angular_speed_dps: float,
        duration_s: float,
        body_height: float = 0.28,
        gait_id: int = 26,
    ) -> None:
        if duration_s <= 0:
            return
        wz = math.radians(angular_speed_dps)
        self.ctx.dog.set_velocity_command(
            float(linear_speed_mps), 0.0, wz,
            body_height=body_height, gait_id=gait_id,
        )
        self._action = {
            "type": "open_arc",
            "vx": float(linear_speed_mps), "vy": 0.0, "wz": wz,
            "body_height": body_height, "gait_id": gait_id,
            "start": time.monotonic(),
            "duration": duration_s,
        }

    # ------------------------------------------------------------------
    # 闭环动作
    # ------------------------------------------------------------------

    def align_yaw(
        self,
        target_yaw_deg: float,
        tolerance_deg: float = 2.0,
        max_angular_speed_dps: float = 15.0,
        min_angular_speed_dps: float = 3.0,
        max_step_deg: float = 15.0,
        max_attempts: int = 10,
    ) -> None:
        self._action = {
            "type": "closed_yaw",
            "target_rad": math.radians(target_yaw_deg),
            "tolerance_rad": math.radians(tolerance_deg),
            "max_wz": math.radians(max_angular_speed_dps),
            "min_wz": math.radians(min_angular_speed_dps),
            "max_step_rad": math.radians(max_step_deg),
            "max_attempts": max_attempts,
            "attempts": 0,
            "sub_phase": 0,       # 0 = 测量, 1 = 等待旋转完成
            "step_action": None,  # 当前旋转步的计时信息
        }

    def align_position(
        self,
        target_x: float,
        target_y: float,
        tolerance_m: float = 0.02,
        speed_mps: float = 0.15,
        max_step_m: float = 0.15,
        turn_speed_dps: float = 15.0,
        turn_tolerance_deg: float = 5.0,
        max_attempts: int = 20,
    ) -> None:
        self._action = {
            "type": "closed_pos",
            "target_x": target_x,
            "target_y": target_y,
            "tolerance_m": tolerance_m,
            "speed_mps": speed_mps,
            "max_step_m": max_step_m,
            "turn_speed_dps": turn_speed_dps,
            "turn_tolerance_rad": math.radians(turn_tolerance_deg),
            "max_attempts": max_attempts,
            "attempts": 0,
            "sub_phase": 0,       # 0 = 测量, 1 = 等待旋转, 2 = 等待直行
            "step_action": None,
        }

    # ------------------------------------------------------------------
    # 内部更新逻辑
    # ------------------------------------------------------------------

    def _update_open_loop(self) -> bool:
        a = self._action
        if time.monotonic() - a["start"] >= a["duration"]:
            self.ctx.dog.set_velocity_command(0.0, 0.0, 0.0, body_height=a["body_height"], gait_id=a["gait_id"])
            self._action = None
            return True
        return False

    def _update_closed_yaw(self) -> bool:
        a = self._action

        if a["sub_phase"] == 0:
            if a["attempts"] >= a["max_attempts"]:
                self.ctx.dog.set_velocity_command(0.0, 0.0, 0.0)
                self._action = None
                return True
            a["attempts"] += 1

            _, _, cur_yaw = self.ctx.pose.get_xy_yaw()
            error = _normalize_angle(a["target_rad"] - cur_yaw)

            if abs(error) <= a["tolerance_rad"]:
                self.ctx.dog.set_velocity_command(0.0, 0.0, 0.0)
                self._action = None
                return True

            step_rad = max(-a["max_step_rad"], min(error, a["max_step_rad"]))
            abs_err = abs(error)
            speed = max(a["min_wz"], min(abs_err, a["max_wz"]))
            step_dur = abs(step_rad) / speed if speed > 0 else 0.01

            wz = speed if step_rad > 0 else -speed
            self.ctx.dog.set_velocity_command(0.0, 0.0, wz)
            a["step_action"] = {"start": time.monotonic(), "duration": step_dur}
            a["sub_phase"] = 1
            return False

        if a["sub_phase"] == 1:
            sa = a["step_action"]
            if time.monotonic() - sa["start"] >= sa["duration"]:
                self.ctx.dog.set_velocity_command(0.0, 0.0, 0.0)
                if a["attempts"] >= a["max_attempts"]:
                    self._action = None
                    return True
                a["sub_phase"] = 0
            return False

        return False

    def _update_closed_pos(self) -> bool:
        a = self._action

        if a["sub_phase"] == 0:
            if a["attempts"] >= a["max_attempts"]:
                self.ctx.dog.set_velocity_command(0.0, 0.0, 0.0)
                self._action = None
                return True
            a["attempts"] += 1

            x, y, _ = self.ctx.pose.get_xy_yaw()
            dx = a["target_x"] - x
            dy = a["target_y"] - y
            dist = math.hypot(dx, dy)

            if dist <= a["tolerance_m"]:
                self.ctx.dog.set_velocity_command(0.0, 0.0, 0.0)
                self._action = None
                return True

            bearing = math.atan2(dy, dx)
            step = min(dist, a["max_step_m"])
            dur = step / a["speed_mps"] if a["speed_mps"] > 0 else 0.1

            self.ctx.dog.set_velocity_command(a["speed_mps"], 0.0, 0.0)
            a["step_action"] = {
                "start": time.monotonic(),
                "duration": dur,
                "target_bearing": bearing,
            }
            a["sub_phase"] = 2
            return False

        if a["sub_phase"] == 1:
            sa = a["step_action"]
            if time.monotonic() - sa["start"] >= sa["duration"]:
                self.ctx.dog.set_velocity_command(0.0, 0.0, 0.0)
                if a["attempts"] >= a["max_attempts"]:
                    self._action = None
                    return True
                a["sub_phase"] = 0
            return False

        if a["sub_phase"] == 2:
            sa = a["step_action"]
            if time.monotonic() - sa["start"] >= sa["duration"]:
                self.ctx.dog.set_velocity_command(0.0, 0.0, 0.0)
                if a["attempts"] >= a["max_attempts"]:
                    self._action = None
                    return True
                a["sub_phase"] = 0
            return False

        return False
