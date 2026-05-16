"""Stage1 的纯逻辑测试：用假 ctx 验证 Phase 转移，不依赖 ROS2。"""
from stages.stage1_stone_path import Stage1StonePath, Phase
from core.framework.stage import StageStatus
from perception.hub import LaneEdges


class FakeDog:
    def __init__(self): self.calls = []
    def execute_discrete_action(self, **kwargs): self.calls.append(("execute_discrete_action", kwargs))
    def set_velocity_command(self, vx, vy, wz, **kw): self.calls.append(("vel", vx, vy, wz))


class FakePerception:
    def __init__(self, lane_edges=None):
        self._lane_edges = lane_edges or LaneEdges()

    def latest_lane_edges(self):
        return self._lane_edges


class FakePose:
    def set_origin_here(self): pass


class FakeLogger:
    def info(self, msg): pass


def make_ctx(lane_edges=None):
    class Ctx: pass
    c = Ctx()
    c.dog = FakeDog()
    c.pose = FakePose()
    c.perception = FakePerception(lane_edges)
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


def test_recovery_stand_command_is_sent_only_once_before_stabilize():
    ctx = make_ctx()
    s = Stage1StonePath(ctx)
    s.on_enter()

    s.tick()
    s.tick()

    stand_calls = [call for call in ctx.dog.calls if call[0] == "execute_discrete_action"]
    assert len(stand_calls) == 1


def test_done_phase_returns_succeeded():
    ctx = make_ctx()
    s = Stage1StonePath(ctx)
    s.on_enter()
    s.phase = Phase.DONE
    assert s.tick() is StageStatus.SUCCEEDED


def test_straight_to_bend_triggers_turn_on_visual_horizontal_line():
    lane_edges = LaneEdges(
        left_offset_px=-40.0,
        right_offset_px=40.0,
        center_offset_px=0.0,
        confidence=0.8,
        horizontal_confidence=0.85,
        turn_hint="left",
    )
    ctx = make_ctx(lane_edges)
    s = Stage1StonePath(ctx)
    s.on_enter()
    s.phase = Phase.STRAIGHT_TO_BEND
    s.phase_start -= s.p["min_straight_time"] + 0.1
    s.tick()
    assert s.phase is Phase.TURNING
    assert s.turn_direction == 1.0


def test_straight_to_bend_uses_visual_lane_lateral_correction():
    lane_edges = LaneEdges(
        center_offset_px=30.0,
        confidence=0.9,
    )
    ctx = make_ctx(lane_edges)
    s = Stage1StonePath(ctx)
    s.on_enter()
    s.phase = Phase.STRAIGHT_TO_BEND
    s.tick()
    assert ctx.dog.calls[-1][0] == "vel"
    assert ctx.dog.calls[-1][2] < 0.0
