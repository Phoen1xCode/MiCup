from core.framework.stage import StageStatus
from perception.hub import DashedLineDet, LaneEdges
from stages.stage3_curve_dash import Phase, Stage3CurveDash


class FakeDog:
    def __init__(self): self.calls = []
    def set_velocity_command(self, vx, vy, wz, **kw): self.calls.append(("vel", vx, vy, wz))


class FakeCorridor:
    def __init__(self, left=1.0, front=2.0, right=1.0):
        self.left, self.front, self.right = left, front, right


class FakePerception:
    def __init__(self, corridor, lane_edges=None, dashed_line=None):
        self.corridor = corridor
        self.lane_edges = lane_edges or LaneEdges()
        self.dashed_line = dashed_line

    def latest_lidar_corridor(self):
        return self.corridor

    def latest_lane_edges(self):
        return self.lane_edges

    def latest_dashed_line(self):
        return self.dashed_line


class FakePose:
    def set_origin_here(self): pass


class FakeLogger:
    def info(self, msg): pass


def make_stage(corridor, lane_edges=None, dashed_line=None):
    class Ctx: pass
    ctx = Ctx()
    ctx.dog = FakeDog()
    ctx.pose = FakePose()
    ctx.perception = FakePerception(corridor, lane_edges, dashed_line)
    ctx.logger = FakeLogger()
    stage = Stage3CurveDash(ctx)
    stage.on_enter()
    return stage, ctx


def test_stage3_enter_moves_to_follow():
    stage, _ = make_stage(FakeCorridor())
    assert stage.tick() is StageStatus.RUNNING
    assert stage.phase is Phase.FOLLOW_CORRIDOR


def test_stage3_detects_exit_from_open_corridor():
    stage, _ = make_stage(FakeCorridor(left=0.8, front=2.0, right=1.4))
    stage.phase = Phase.FOLLOW_CORRIDOR
    stage.phase_start -= stage.p["min_follow_time"] + 0.1
    stage.tick()
    assert stage.phase is Phase.STRAIGHT_TO_EXIT


def test_stage3_follow_timeout_enters_exit():
    stage, _ = make_stage(FakeCorridor(left=0.8, front=0.8, right=0.8))
    stage.phase = Phase.FOLLOW_CORRIDOR
    stage.phase_start -= stage.p["max_follow_time"] + 0.1
    stage.tick()
    assert stage.phase is Phase.STRAIGHT_TO_EXIT


def test_stage3_prefers_visual_lane_when_lidar_has_no_returns():
    edges = LaneEdges(center_offset_px=25.0, confidence=0.9)
    stage, ctx = make_stage(FakeCorridor(left=99.9, front=99.9, right=99.9), edges)
    stage.phase = Phase.FOLLOW_CORRIDOR
    stage.tick()
    assert ctx.dog.calls[-1][0] == "vel"
    assert ctx.dog.calls[-1][2] < 0.0


def test_stage3_detects_exit_from_visual_dashed_line():
    dashed = DashedLineDet(center_px=(120.0, 180.0), confidence=0.8)
    stage, _ = make_stage(FakeCorridor(left=99.9, front=99.9, right=99.9), dashed_line=dashed)
    stage.phase = Phase.FOLLOW_CORRIDOR
    stage.phase_start -= stage.p["min_follow_time"] + 0.1
    stage.tick()
    assert stage.phase is Phase.STRAIGHT_TO_EXIT


def test_stage3_done_succeeds():
    stage, _ = make_stage(FakeCorridor())
    stage.phase = Phase.DONE
    assert stage.tick() is StageStatus.SUCCEEDED
