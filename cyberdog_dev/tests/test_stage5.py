from core.framework.stage import StageStatus
from perception.hub import DashedLineDet, LaneEdges
from stages.stage5_bridge import Phase, Stage5Bridge


class FakeDog:
    def __init__(self): self.calls = []
    def set_velocity_command(self, vx, vy, wz, **kw): self.calls.append(("vel", vx, vy, wz))


class FakePose:
    def set_origin_here(self): pass


class FakeLogger:
    def info(self, msg): pass


class FakePerception:
    def __init__(self, lane_edges=None, dashed_line=None):
        self.lane_edges = lane_edges or LaneEdges()
        self.dashed_line = dashed_line

    def latest_lane_edges(self):
        return self.lane_edges

    def latest_dashed_line(self):
        return self.dashed_line


def make_stage(lane_edges=None, dashed_line=None):
    class Ctx: pass
    ctx = Ctx()
    ctx.dog = FakeDog()
    ctx.pose = FakePose()
    ctx.logger = FakeLogger()
    ctx.perception = FakePerception(lane_edges, dashed_line)
    stage = Stage5Bridge(ctx)
    stage.on_enter()
    return stage, ctx


def test_stage5_walks_bridge_slowly():
    stage, ctx = make_stage(LaneEdges(left_offset_px=10.0, right_offset_px=-4.0, confidence=0.9))
    stage.phase = Phase.WALK_BRIDGE
    assert stage.tick() is StageStatus.RUNNING
    assert ctx.dog.calls[-1][0] == "vel"
    assert ctx.dog.calls[-1][1] == stage.p["bridge_speed"]


def test_stage5_dashed_line_enters_jump_after_min_walk():
    dashed = DashedLineDet(center_px=(120.0, 180.0), confidence=0.8)
    stage, _ = make_stage(dashed_line=dashed)
    stage.phase = Phase.WALK_BRIDGE
    stage.phase_start -= stage.p["min_walk_time"] + 0.1
    stage.tick()
    assert stage.phase is Phase.CLEAR_DASHED_LINE


def test_stage5_clears_dashed_line_before_jump():
    stage, ctx = make_stage()
    stage.phase = Phase.CLEAR_DASHED_LINE
    stage.phase_start -= stage.p["clear_dashed_time"] + 0.1
    stage.tick()
    assert stage.phase is Phase.JUMP_DOWN
    assert ctx.dog.calls[-1][0] == "vel"


def test_stage5_timeout_enters_jump_without_dashed_line():
    stage, _ = make_stage()
    stage.phase = Phase.WALK_BRIDGE
    stage.phase_start -= stage.p["max_walk_time"] + 0.1
    stage.tick()
    assert stage.phase is Phase.JUMP_DOWN


def test_stage5_done_succeeds():
    stage, _ = make_stage()
    stage.phase = Phase.DONE
    assert stage.tick() is StageStatus.SUCCEEDED
