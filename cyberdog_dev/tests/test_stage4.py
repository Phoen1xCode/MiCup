from core.framework.stage import StageStatus
from perception.hub import BallDet, ObjDet, PoleDet
from stages.stage4_tunnel_search import Phase, Stage4TunnelSearch


class FakeDog:
    def __init__(self): self.calls = []
    def set_velocity_command(self, vx, vy, wz, **kw): self.calls.append(("vel", vx, vy, wz, kw))


class FakePose:
    def set_origin_here(self): pass


class FakeLogger:
    def info(self, msg): pass


class FakeVoice:
    def __init__(self): self.history = []
    def say(self, text): self.history.append(text)


class FakePerception:
    def __init__(self, *, poles=None, blocks=None, cokes=None, balls=None, football=None):
        self.poles = poles or []
        self.blocks = blocks or []
        self.cokes = cokes or []
        self.balls = balls or []
        self.football = football

    def latest_red_poles(self): return self.poles
    def latest_block_obstacles(self): return self.blocks
    def latest_coke_bottles(self): return self.cokes
    def latest_orange_balls(self): return self.balls
    def latest_football(self): return self.football


def obj(label, distance=0.8):
    return ObjDet(label, (0, 0, 10, 10), (5.0, 5.0), 100.0, 0.0, distance, 0.8)


def pole():
    return PoleDet((0, 0, 10, 60), (5.0, 30.0), 600.0, 0.0, 0.9)


def ball():
    return BallDet((0, 0, 10, 10), (5.0, 5.0), 100.0, 0.0, 0.5, 0.8)


def make_stage(perception=None):
    class Ctx: pass
    ctx = Ctx()
    ctx.dog = FakeDog()
    ctx.pose = FakePose()
    ctx.logger = FakeLogger()
    ctx.voice = FakeVoice()
    ctx.perception = perception or FakePerception()
    stage = Stage4TunnelSearch(ctx)
    stage.on_enter()
    return stage, ctx


def test_stage4_selects_safety_obstacle_before_targets():
    perception = FakePerception(
        blocks=[obj("block")],
        cokes=[obj("coke")],
        balls=[ball()],
        football=ball(),
    )
    stage, _ = make_stage(perception)
    kind, _ = stage._select_candidate()
    assert kind == "block_obstacle"


def test_stage4_announces_exact_phrase_once():
    stage, ctx = make_stage()
    stage._announce("coke")
    stage._announce("coke")
    assert ctx.voice.history == ["识别到可乐瓶"]


def test_stage4_limit_pole_uses_low_walk(monkeypatch):
    calls = []

    def fake_low_walk(dog, duration_sec, speed):
        calls.append((dog, duration_sec, speed))
        return True

    monkeypatch.setattr("stages.stage4_tunnel_search.execute_low_walk", fake_low_walk)
    stage, ctx = make_stage(FakePerception(poles=[pole()]))
    stage.phase = Phase.SCAN_LANE
    stage.tick()
    assert ctx.voice.history == ["识别到限高杆"]
    assert stage.phase is Phase.INTERACT

    stage.tick()
    assert calls == [(ctx.dog, stage.p["low_walk_time"], stage.p["low_walk_speed"])]
    assert "red_pole" in stage.handled_obstacles


def test_stage4_far_target_is_approached_before_interaction():
    stage, ctx = make_stage(FakePerception(cokes=[obj("coke", distance=1.2)]))
    stage.phase = Phase.SCAN_LANE
    stage.tick()
    assert ctx.voice.history == ["识别到可乐瓶"]
    assert stage.phase is Phase.APPROACH_ITEM

    stage.tick()
    assert stage.phase is Phase.APPROACH_ITEM
    assert ctx.dog.calls[-1][0] == "vel"
    assert ctx.dog.calls[-1][1] == stage.p["approach_speed"]


def test_stage4_close_target_enters_interaction():
    stage, _ = make_stage(FakePerception(cokes=[obj("coke", distance=0.35)]))
    stage.phase = Phase.SCAN_LANE
    stage.tick()
    assert stage.phase is Phase.INTERACT


def test_stage4_lane_shift_advances_scan_lane():
    stage, ctx = make_stage()
    stage.phase = Phase.SHIFT_LANE
    stage.phase_start -= stage.p["lane_shift_time"] + 0.1
    stage.tick()
    assert stage.lane_index == 1
    assert stage.phase is Phase.SCAN_LANE
    assert ctx.dog.calls[-1][0] == "vel"


def test_stage4_goes_bridge_after_required_items_handled():
    stage, _ = make_stage()
    stage.phase = Phase.SCAN_LANE
    stage.handled_targets = {"coke", "orange_ball", "football"}
    stage.handled_obstacles = {"red_pole", "block_obstacle"}
    stage.tick()
    assert stage.phase is Phase.GO_BRIDGE


def test_stage4_done_succeeds():
    stage, _ = make_stage()
    stage.phase = Phase.DONE
    assert stage.tick() is StageStatus.SUCCEEDED
