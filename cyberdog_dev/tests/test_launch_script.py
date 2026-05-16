from pathlib import Path


LAUNCH = Path(__file__).resolve().parent.parent / "scripts" / "launch.sh"


def test_launch_script_uses_existing_module_entrypoints():
    text = LAUNCH.read_text()

    assert "python3 - <<'PY'" in text
    assert '("toml", "lcm")' in text
    assert "__import__(module)" in text
    assert "perception.lidar_corridor" in text
    assert "perception.orange_ball" not in text
    assert "python3 -m perception \"$node\" --mode \"$MODE\"" in text
    assert "python3 -m core.localization.odom_broadcaster --mode \"$MODE\"" in text
