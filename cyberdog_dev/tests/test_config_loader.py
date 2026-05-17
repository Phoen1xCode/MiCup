from pathlib import Path
from config.loader import load_topics, load_stage_params

CONFIG_DIR = Path(__file__).resolve().parent.parent / "config"


def test_load_topics_sim():
    t = load_topics(CONFIG_DIR / "topics.toml", mode="sim")
    assert t["scan_topic"] == "/scan"
    assert t["namespace"] == ""


def test_load_topics_real():
    t = load_topics(CONFIG_DIR / "topics.toml", mode="real")
    assert t["namespace"] == "/mi_desktop_48_b0_2d_5f_ba_36"
    assert t["scan_topic"].startswith("/mi_desktop")


def test_load_topics_rejects_unknown_mode():
    import pytest
    with pytest.raises(KeyError):
        load_topics(CONFIG_DIR / "topics.toml", mode="bogus")


def test_load_stage_params_stage1():
    p = load_stage_params(CONFIG_DIR / "stage_params.toml", stage_id=1)
    assert p["forward_speed"] > 0.0
    assert p["max_time"] == 45.0
    assert p["straight_distance"] > 0.0
    assert p["turn_angle_deg"] > 0.0
