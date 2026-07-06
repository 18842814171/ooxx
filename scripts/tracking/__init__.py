"""Tracking backend interface — stub now, point-cloud / DL later."""

from tracking.base import TrackerBackend
from tracking.stub import StubTracker

__all__ = ['TrackerBackend', 'StubTracker']
