from core.stage_base import StageStatus
from stages.stage3_curve_dash import Phase, Stage3CurveDash


class FakeDog:
    def __init__(self): self.calls = []
    def set_velocity(self, vx, vy, wz): self.calls.append(("vel", vx, vy, wz))
    def stop(self): self.calls.append(("stop",))


class FakeCorridor:
    def __init__(self, left=1.0, front=2.0, right=1.0):
        self.left, self.front, self.right = left, front, right


class FakePerception:
    def __init__(self, corridor): self.corridor = corridor
    def latest_lidar_corridor(self): return self.corridor


class FakePose:
    def set_origin_here(self): pass


class FakeLogger:
    def info(self, msg): pass


def make_stage(corridor):
    class Ctx: pass
    ctx = Ctx()
    ctx.dog = FakeDog()
    ctx.pose = FakePose()
    ctx.perception = FakePerception(corridor)
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


def test_stage3_done_succeeds():
    stage, _ = make_stage(FakeCorridor())
    stage.phase = Phase.DONE
    assert stage.tick() is StageStatus.SUCCEEDED
