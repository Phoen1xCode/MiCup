"""Stage1 的纯逻辑测试：用假 ctx 验证 Phase 转移，不依赖 ROS2。"""
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
    ctx = make_ctx(FakeCorridor(left=1.5, front=0.9, right=0.5))
    s = Stage1StonePath(ctx)
    s.on_enter()
    s.phase = Phase.STRAIGHT_TO_BEND
    s.phase_start -= s.p["min_straight_time"] + 0.1
    s.tick()
    assert s.phase is Phase.TURNING
    assert s.turn_direction == 1.0
