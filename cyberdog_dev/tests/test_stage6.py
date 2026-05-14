from core.stage_base import StageStatus
from perception.hub import BallDet
from stages.stage6_kick import Phase, Stage6Kick


class FakeDog:
    def __init__(self): self.calls = []
    def set_velocity(self, vx, vy, wz): self.calls.append(("vel", vx, vy, wz))
    def stop(self): self.calls.append(("stop",))
    def lie_down(self, *, hold=0.0): self.calls.append(("lie_down", hold)); return True


class FakePose:
    def set_origin_here(self): pass


class FakeLogger:
    def info(self, msg): pass


class FakePerception:
    def __init__(self, football=None): self.football = football
    def latest_football(self): return self.football


def make_stage(football=None):
    class Ctx: pass
    ctx = Ctx()
    ctx.dog = FakeDog()
    ctx.pose = FakePose()
    ctx.logger = FakeLogger()
    ctx.perception = FakePerception(football)
    stage = Stage6Kick(ctx)
    stage.on_enter()
    return stage, ctx


def test_stage6_fallback_kicks_without_ball():
    stage, _ = make_stage()
    stage.phase = Phase.FIND_BALL
    stage.phase_start -= 1.1
    stage.tick()
    assert stage.phase is Phase.KICK


def test_stage6_aligns_to_visible_ball():
    ball = BallDet((0, 0, 20, 20), (10.0, 10.0), 400.0, 0.3, 0.6, 0.9)
    stage, ctx = make_stage(ball)
    stage.phase = Phase.FIND_BALL
    stage.tick()
    assert stage.phase is Phase.ALIGN_BEHIND_BALL
    assert ctx.dog.calls[-1][0] == "vel"


def test_stage6_lie_down_reaches_done():
    stage, ctx = make_stage()
    stage.phase = Phase.LIE_DOWN_IN_CIRCLE
    stage.phase_start -= stage.p["lie_down_time"] + 0.1
    status = stage.tick()
    assert status is StageStatus.RUNNING
    assert stage.phase is Phase.DONE
    assert ctx.dog.calls[-1][0] == "lie_down"
