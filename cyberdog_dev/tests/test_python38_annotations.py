import ast
from pathlib import Path


RUNTIME_DIRS = ("core", "perception", "stages")
PEP585_NAMES = {"dict", "list", "set", "tuple"}


def _uses_runtime_pep585_annotations(path: Path) -> bool:
    tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
    for node in ast.walk(tree):
        if isinstance(node, ast.Subscript) and isinstance(node.value, ast.Name):
            if node.value.id in PEP585_NAMES:
                return True
    return False


def _has_future_annotations(path: Path) -> bool:
    lines = path.read_text(encoding="utf-8").splitlines()
    return "from __future__ import annotations" in lines[:5]


def test_python38_pep585_annotations_are_deferred():
    root = Path(__file__).resolve().parents[1]
    offenders = []
    for dirname in RUNTIME_DIRS:
        for path in (root / dirname).rglob("*.py"):
            if _uses_runtime_pep585_annotations(path) and not _has_future_annotations(path):
                offenders.append(path.relative_to(root).as_posix())
    assert offenders == []
