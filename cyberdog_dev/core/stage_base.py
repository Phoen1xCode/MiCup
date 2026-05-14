"""Stage 基类与状态类型（spec 3.1）。"""

from dataclasses import dataclass, field
from enum import Enum, auto
from typing import List


class StageStatus(Enum):
    RUNNING = auto()
    SUCCEEDED = auto()
    FAILED = auto()
    NEED_HELP = auto()


@dataclass
class StageResult:
    stage_id: int
    name: str
    status: StageStatus
    notes: List[str] = field(default_factory=list)
    elapsed_sec: float = 0.0


class Stage:
    """所有赛段的基类。子类必须实现 tick()。"""

    stage_id: int = 0
    name: str = ""

    def __init__(self, ctx):
        self.ctx = ctx
        self.start_time = 0.0
        self.notes: List[str] = []

    def on_enter(self) -> None:
        """进入赛段时调用一次（设原点等）。默认空操作。"""

    def tick(self) -> StageStatus:
        """状态机的一次推进。子类必须实现。"""
        raise NotImplementedError("Stage 子类必须实现 tick()")

    def on_exit(self) -> None:
        """退出赛段时清理（停车等）。默认空操作。"""

    def max_duration_sec(self) -> float:
        """该赛段的超时上限（秒）。"""
        return 180.0
