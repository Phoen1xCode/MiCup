"""toml 配置加载辅助。"""

import importlib
import sys
from pathlib import Path
from typing import Dict


def _load_toml(path: Path):
    """加载 TOML，避开仓库内旧 `toml/` 空目录对第三方包的遮蔽。"""
    try:
        import tomllib
        return tomllib.loads(path.read_text())
    except ModuleNotFoundError:
        pass

    try:
        import tomli
        return tomli.loads(path.read_text())
    except ModuleNotFoundError:
        pass

    project_dir = Path(__file__).resolve().parent.parent
    original_path = list(sys.path)
    previous_toml = sys.modules.pop("toml", None)
    try:
        sys.path = [
            entry for entry in sys.path
            if entry and Path(entry).resolve() != project_dir
        ]
        toml = importlib.import_module("toml")
        return toml.load(path)
    finally:
        sys.path = original_path
        if previous_toml is not None:
            sys.modules["toml"] = previous_toml


def load_topics(path: Path, mode: str) -> Dict[str, str]:
    """读取 topics.toml，返回指定 mode（'sim'/'real'）的 topic 字典。"""
    data = _load_toml(path)
    if mode not in data:
        raise KeyError(f"topics.toml 中没有 mode={mode!r}，可选: {list(data)}")
    return data[mode]


def load_stage_params(path: Path, stage_id: int) -> Dict[str, float]:
    """读取 stage_params.toml，返回指定赛段的参数字典。"""
    data = _load_toml(path)
    key = f"stage{stage_id}"
    if key not in data:
        raise KeyError(f"stage_params.toml 中没有 {key}，可选: {list(data)}")
    return data[key]
