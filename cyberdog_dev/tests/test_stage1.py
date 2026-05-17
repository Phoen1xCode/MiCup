"""Stage1 纯逻辑测试：用假 ctx 验证 Phase 转移，不依赖 ROS2。"""
import time
from stages.stage1_stone_path import Stage1StonePath, Phase
from core.framework.stage import StageStatus


class FakeDog:
    def __init__(self): self.calls = []
    def execute_discrete_action(self, **kwargs): self.calls.append(("execute_discrete_action", kwargs))
    def set_velocity_command(self, vx, vy, wz, **kw): self.calls.append(("vel", vx, vy, wz))


class FakePose:
    def set_origin_here(self): pass
    def get_xy_yaw(self): return (0.0, 0.0, 0.0)


class FakeLogger:
    def info(self, msg): pass


def make_ctx():
    class Ctx: pass
    c = Ctx()
    c.dog = FakeDog()
    c.pose = FakePose()
    c.logger = FakeLogger()
    return c


def test_starts_in_recovery_stand():
    ctx = make_ctx()
    s = Stage1StonePath(ctx)
    s.on_enter()
    assert s.phase is Phase.RECOVERY_STAND


def test_recovery_stand_advances_after_stand_time():
    ctx = make_ctx()
    s = Stage1StonePath(ctx)
    s.on_enter()
    s.phase_start -= s.p["stand_time"] + 0.1
    status = s.tick()
    assert status is StageStatus.RUNNING
    assert s.phase is Phase.STABILIZE


def test_recovery_stand_command_is_sent_only_once():
    ctx = make_ctx()
    s = Stage1StonePath(ctx)
    s.on_enter()
    s.tick()
    s.tick()
    stand_calls = [c for c in ctx.dog.calls if c[0] == "execute_discrete_action"]
    assert len(stand_calls) == 1


def test_stabilize_advances_after_stabilize_time():
    ctx = make_ctx()
    s = Stage1StonePath(ctx)
    s.on_enter()
    s.phase = Phase.STABILIZE
    s.phase_start = time.monotonic() - s.p["stabilize_time"] - 0.1
    s.tick()
    assert s.phase is Phase.STRAIGHT_TO_BEND


def test_straight_to_bend_walks_forward():
    ctx = make_ctx()
    s = Stage1StonePath(ctx)
    s.on_enter()
    s.phase = Phase.STRAIGHT_TO_BEND
    s.phase_start = time.monotonic()
    s.tick()
    # 应该发送前进速度指令
    assert len(ctx.dog.calls) >= 1
    last = ctx.dog.calls[-1]
    assert last[0] == "vel"
    assert last[1] > 0  # vx > 0 前进


def test_turning_sends_rotation_command():
    ctx = make_ctx()
    s = Stage1StonePath(ctx)
    s.on_enter()
    s.phase = Phase.TURNING
    s.phase_start = time.monotonic()
    s.tick()
    assert len(ctx.dog.calls) >= 1
    last = ctx.dog.calls[-1]
    assert last[0] == "vel"
    assert last[3] != 0  # wz != 0 有旋转


def test_done_phase_returns_succeeded():
    ctx = make_ctx()
    s = Stage1StonePath(ctx)
    s.on_enter()
    s.phase = Phase.DONE
    assert s.tick() is StageStatus.SUCCEEDED


def test_on_exit_stops_motion():
    ctx = make_ctx()
    s = Stage1StonePath(ctx)
    s.on_enter()
    s.phase = Phase.STRAIGHT_TO_BEND
    s.tick()  # 启动一个动作
    s.on_exit()
    # 最后一次调用应该是停止
    assert ctx.dog.calls[-1] == ("vel", 0.0, 0.0, 0.0)
