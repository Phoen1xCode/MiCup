from pathlib import Path

from core.localization import odom_broadcaster


def test_odom_broadcaster_uses_project_config_dir():
    expected = Path(odom_broadcaster.__file__).resolve().parents[2] / "config"

    assert odom_broadcaster.config_dir_from_here() == expected
    assert (expected / "topics.toml").exists()
