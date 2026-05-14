from core.stage_context import RunMode, StageContext


def test_runmode_has_sim_and_real():
    assert RunMode.SIM.value == "sim"
    assert RunMode.REAL.value == "real"


def test_runmode_from_string():
    assert RunMode("sim") is RunMode.SIM
    assert RunMode("real") is RunMode.REAL


def test_stage_context_holds_dependencies():
    ctx = StageContext(
        dog="DOG", pose="POSE", perception="PERC",
        voice="VOICE", logger="LOG", mode=RunMode.SIM,
    )
    assert ctx.dog == "DOG"
    assert ctx.mode is RunMode.SIM
