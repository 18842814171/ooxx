"""No-op tracker — placeholder until point_cloud_target_track / DL integration."""

from __future__ import annotations

from typing import Optional

from geometry_msgs.msg import Twist

from perception.base import Detection
from tracking.base import TrackerBackend


class StubTracker(TrackerBackend):
    def init_track(self, detection: Detection, depth_frame=None) -> None:
        pass

    def update(self, bgr_image, depth_frame=None) -> Optional[Twist]:
        return None

    def is_active(self) -> bool:
        return False

    def stop(self) -> None:
        pass


def create_tracker_backend(name: str) -> TrackerBackend:
    if name in ('stub', 'point_cloud'):
        return StubTracker()
    raise ValueError('Unknown tracking backend: {}'.format(name))
