import pytest
from core.stage_base import Stage, StageStatus, StageResult


def test_stage_status_has_four_states():
    names = {s.name for s in StageStatus}
    assert names == {"RUNNING", "SUCCEEDED", "FAILED", "NEED_HELP"}


def test_stage_result_fields():
    r = StageResult(stage_id=1, name="石径探路", status=StageStatus.SUCCEEDED,
                    notes=["ok"], elapsed_sec=12.3)
    assert r.stage_id == 1
    assert r.status is StageStatus.SUCCEEDED
    assert r.notes == ["ok"]


def test_base_stage_tick_must_be_overridden():
    s = Stage(ctx=object())
    with pytest.raises(NotImplementedError):
        s.tick()


def test_subclass_can_implement_tick():
    class Dummy(Stage):
        stage_id = 9
        name = "dummy"

        def tick(self):
            return StageStatus.SUCCEEDED

    d = Dummy(ctx=object())
    assert d.tick() is StageStatus.SUCCEEDED
    assert d.stage_id == 9


def test_default_max_duration_is_positive():
    s = Stage(ctx=object())
    assert s.max_duration_sec() > 0


def test_lifecycle_hooks_are_noops_by_default():
    s = Stage(ctx=object())
    s.on_enter()
    s.on_exit()
