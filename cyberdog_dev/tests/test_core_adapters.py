from core.customized_gait.low_walk import execute_low_walk
from core.framework.voice import VoiceController


class FakeLogger:
    def __init__(self): self.messages = []
    def info(self, msg): self.messages.append(("info", msg))
    def warn(self, msg): self.messages.append(("warn", msg))


class FakeDog:
    def __init__(self): self.calls = []
    def set_velocity_command(self, vx, vy, wz, **kwargs): self.calls.append((vx, vy, wz, kwargs))


def test_voice_controller_records_spoken_text():
    logger = FakeLogger()
    voice = VoiceController(logger=logger)
    voice.say("识别到足球")
    assert voice.history == ["识别到足球"]
    assert logger.messages[-1] == ("info", "语音播报: 识别到足球")


def test_low_walk_sets_velocity_and_stops():
    dog = FakeDog()
    assert execute_low_walk(dog, duration_sec=0.0, speed=0.12)
    assert dog.calls[0][0:3] == (0.12, 0.0, 0.0)
    assert dog.calls[-1] == (0.0, 0.0, 0.0, {})
