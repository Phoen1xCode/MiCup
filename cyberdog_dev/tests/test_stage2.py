from core.stage_base import StageStatus
from perception.hub import BallDet
from stages.stage2_orange_balls import Phase, Stage2OrangeBalls


class FakeDog:
    def __init__(self): self.calls = []
    def set_velocity(self, vx, vy, wz): self.calls.append(("vel", vx, vy, wz))
    def stop(self): self.calls.append(("stop",))


class FakePose:
    def set_origin_here(self): pass


class FakeLogger:
    def info(self, msg): pass


class FakePerception:
    def __init__(self, balls): self.balls = balls
    def latest_orange_balls(self): return self.balls


def make_stage(balls):
    class Ctx: pass
    ctx = Ctx()
    ctx.dog = FakeDog()
    ctx.pose = FakePose()
    ctx.logger = FakeLogger()
    ctx.perception = FakePerception(balls)
    stage = Stage2OrangeBalls(ctx)
    stage.on_enter()
    return stage, ctx


def ball(distance, bearing=0.0):
    return BallDet((0, 0, 10, 10), (5.0, 5.0), 100.0, bearing, distance, 0.8)


def test_stage2_selects_nearest_orange_ball():
    stage, _ = make_stage([ball(1.2), ball(0.6)])
    target = stage._select_target()
    assert target.distance_m == 0.6


def test_stage2_approaches_visible_target():
    stage, ctx = make_stage([ball(0.8, bearing=0.2)])
    stage.phase = Phase.APPROACH_NEXT
    assert stage.tick() is StageStatus.RUNNING
    assert ctx.dog.calls[-1][0] == "vel"
    assert stage.phase is Phase.APPROACH_NEXT


def test_stage2_bumps_close_target():
    stage, _ = make_stage([ball(0.2)])
    stage.phase = Phase.APPROACH_NEXT
    stage.tick()
    assert stage.phase is Phase.BUMP


def test_stage2_goes_exit_after_four_hits():
    stage, _ = make_stage([ball(0.2)])
    stage.hit_count = 4
    stage.phase = Phase.BACKOFF
    stage.phase_start -= stage.p["backoff_time"] + 0.1
    stage.tick()
    assert stage.phase is Phase.GO_EXIT
