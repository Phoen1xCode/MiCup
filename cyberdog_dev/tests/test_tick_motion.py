"""tests/test_tick_motion.py — TickMotion 单元测试（不依赖 ROS2/LCM）。"""

from __future__ import annotations

import math
import time
from unittest.mock import MagicMock

import pytest

from core.tick_motion import TickMotion, _normalize_angle


# ------------------------------------------------------------------
# fixtures
# ------------------------------------------------------------------

def _make_ctx():
    """构造最小 StageContext mock，不需要 ROS2 环境。"""
    ctx = MagicMock()
    ctx.dog = MagicMock()
    ctx.pose = MagicMock()
    ctx.pose.get_xy_yaw.return_value = (0.0, 0.0, 0.0)
    ctx.logger = MagicMock()
    return ctx


# ------------------------------------------------------------------
# _normalize_angle
# ------------------------------------------------------------------

class TestNormalizeAngle:
    def test_zero(self):
        assert _normalize_angle(0.0) == 0.0

    def test_positive_wrap(self):
        assert abs(_normalize_angle(3 * math.pi) - math.pi) < 1e-9

    def test_negative_wrap(self):
        assert abs(_normalize_angle(-3 * math.pi) + math.pi) < 1e-9

    def test_in_range(self):
        assert _normalize_angle(1.0) == 1.0


# ------------------------------------------------------------------
# 基本状态
# ------------------------------------------------------------------

class TestIdleAndStop:
    def test_initially_idle(self):
        m = TickMotion(_make_ctx())
        assert m.is_idle()
        assert m.update() is True

    def test_stop_clears_action(self):
        ctx = _make_ctx()
        m = TickMotion(ctx)
        m.move_straight(1.0, 0.2)
        assert not m.is_idle()
        m.stop()
        assert m.is_idle()
        ctx.dog.set_velocity_command.assert_called_with(0.0, 0.0, 0.0)

    def test_noop_on_zero_distance(self):
        m = TickMotion(_make_ctx())
        m.move_straight(0.0, 0.2)
        assert m.is_idle()

    def test_noop_on_zero_angle(self):
        m = TickMotion(_make_ctx())
        m.turn_in_place(0.0, 30.0)
        assert m.is_idle()


# ------------------------------------------------------------------
# 开环: move_straight
# ------------------------------------------------------------------

class TestMoveStraight:
    def test_sends_velocity_and_completes(self):
        ctx = _make_ctx()
        m = TickMotion(ctx)
        m.move_straight(0.4, 0.2)  # 0.4m / 0.2m/s = 2.0s

        assert not m.is_idle()
        ctx.dog.set_velocity_command.assert_called_with(0.2, 0.0, 0.0, body_height=0.28, gait_id=26)

        # 模拟时间流逝：还没到
        m._action["start"] = time.monotonic() - 1.0
        assert m.update() is False

        # 模拟时间流逝：到了
        m._action["start"] = time.monotonic() - 2.5
        assert m.update() is True
        assert m.is_idle()

    def test_negative_distance_goes_backward(self):
        ctx = _make_ctx()
        m = TickMotion(ctx)
        m.move_straight(-1.0, 0.2)
        ctx.dog.set_velocity_command.assert_called_with(-0.2, 0.0, 0.0, body_height=0.28, gait_id=26)


# ------------------------------------------------------------------
# 开环: turn_in_place
# ------------------------------------------------------------------

class TestTurnInPlace:
    def test_left_turn(self):
        ctx = _make_ctx()
        m = TickMotion(ctx)
        m.turn_in_place(90.0, 30.0)  # 90° / 30°/s = 3s

        _, kwargs = ctx.dog.set_velocity_command.call_args
        assert kwargs.get("body_height", 0.28) == 0.28
        args = ctx.dog.set_velocity_command.call_args[0]
        assert args[2] > 0  # positive wz = left turn

    def test_right_turn(self):
        ctx = _make_ctx()
        m = TickMotion(ctx)
        m.turn_in_place(-90.0, 30.0)
        args = ctx.dog.set_velocity_command.call_args[0]
        assert args[2] < 0  # negative wz = right turn


# ------------------------------------------------------------------
# 开环: move_lateral
# ------------------------------------------------------------------

class TestMoveLateral:
    def test_left(self):
        ctx = _make_ctx()
        m = TickMotion(ctx)
        m.move_lateral(0.5, 0.1)
        ctx.dog.set_velocity_command.assert_called_with(0.0, 0.1, 0.0, body_height=0.28, gait_id=26)

    def test_right(self):
        ctx = _make_ctx()
        m = TickMotion(ctx)
        m.move_lateral(-0.5, 0.1)
        ctx.dog.set_velocity_command.assert_called_with(0.0, -0.1, 0.0, body_height=0.28, gait_id=26)


# ------------------------------------------------------------------
# 开环: walk_in_arc
# ------------------------------------------------------------------

class TestWalkInArc:
    def test_sends_combined_velocity(self):
        ctx = _make_ctx()
        m = TickMotion(ctx)
        m.walk_in_arc(0.2, 15.0, 5.0)
        args = ctx.dog.set_velocity_command.call_args[0]
        assert abs(args[0] - 0.2) < 1e-6
        assert abs(args[2] - math.radians(15.0)) < 1e-6

    def test_completes_after_duration(self):
        ctx = _make_ctx()
        m = TickMotion(ctx)
        m.walk_in_arc(0.2, 15.0, 2.0)
        m._action["start"] = time.monotonic() - 2.5
        assert m.update() is True


# ------------------------------------------------------------------
# 闭环: align_yaw
# ------------------------------------------------------------------

class TestAlignYaw:
    def test_already_aligned(self):
        ctx = _make_ctx()
        ctx.pose.get_xy_yaw.return_value = (0.0, 0.0, 0.0)
        m = TickMotion(ctx)
        m.align_yaw(0.0, tolerance_deg=2.0)
        assert m.update() is True  # 误差为 0，直接完成

    def test_needs_correction(self):
        ctx = _make_ctx()
        ctx.pose.get_xy_yaw.return_value = (0.0, 0.0, 0.0)
        m = TickMotion(ctx)
        m.align_yaw(90.0, tolerance_deg=2.0)

        # 第一次 update：应进入旋转步
        result = m.update()
        assert result is False
        assert not m.is_idle()
        assert m._action["sub_phase"] == 1  # 等待旋转完成

    def test_max_attempts_exceeded(self):
        ctx = _make_ctx()
        ctx.pose.get_xy_yaw.return_value = (0.0, 0.0, 0.0)
        m = TickMotion(ctx)
        m.align_yaw(90.0, tolerance_deg=0.001, max_attempts=1)

        # 第一次 update 进入旋转步
        m.update()
        # 模拟旋转完成
        m._action["step_action"]["start"] = time.monotonic() - 10.0
        m.update()  # sub_phase 回到 0，attempts 变 2 > max_attempts
        assert m.is_idle()


# ------------------------------------------------------------------
# 闭环: align_position
# ------------------------------------------------------------------

class TestAlignPosition:
    def test_already_at_target(self):
        ctx = _make_ctx()
        ctx.pose.get_xy_yaw.return_value = (1.0, 2.0, 0.0)
        m = TickMotion(ctx)
        m.align_position(1.0, 2.0, tolerance_m=0.05)
        assert m.update() is True

    def test_needs_movement(self):
        ctx = _make_ctx()
        ctx.pose.get_xy_yaw.return_value = (0.0, 0.0, 0.0)
        m = TickMotion(ctx)
        m.align_position(1.0, 0.0, tolerance_m=0.02)

        result = m.update()
        assert result is False
        assert not m.is_idle()
        assert m._action["sub_phase"] == 2  # 等待直行完成
