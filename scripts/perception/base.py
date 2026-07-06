"""Abstract perception interface — classical now, DL later."""

from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple


@dataclass
class Detection:
    color: str
    shape: str
    center: Tuple[int, int]
    area: float
    confidence: float = 1.0
    world_pos: Optional[Tuple[float, float, float]] = None


@dataclass
class ScanResult:
    detections: List[Detection] = field(default_factory=list)
    counts: Dict[str, int] = field(default_factory=dict)
    mission_complete: bool = False
    new_objects: int = 0


class PerceptionBackend(ABC):
    @abstractmethod
    def load_config(self, mission_config) -> None:
        pass

    @abstractmethod
    def detect(
        self,
        bgr_image,
        depth=None,
        camera_info=None,
        robot_pose=None,
        mission_state=None,
    ) -> ScanResult:
        pass

    @abstractmethod
    def reset_counts(self) -> None:
        pass

    @abstractmethod
    def get_counts(self) -> Dict[str, int]:
        pass
