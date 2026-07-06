"""Abstract tracker interface for follow-after-detect (future stage)."""

from __future__ import annotations

from abc import ABC, abstractmethod
from typing import Optional

from geometry_msgs.msg import Twist

from perception.base import Detection


class TrackerBackend(ABC):
    @abstractmethod
    def init_track(self, detection: Detection, depth_frame=None) -> None:
        pass

    @abstractmethod
    def update(self, bgr_image, depth_frame=None) -> Optional[Twist]:
        pass

    @abstractmethod
    def is_active(self) -> bool:
        pass

    @abstractmethod
    def stop(self) -> None:
        pass
