"""Topic-based vision — bridges search_navigation target_detector via /target_current."""

from __future__ import annotations

import threading
from typing import Dict, Optional

import rospy
from std_msgs.msg import String

from config_loader import MissionConfig
from mission_state import MissionState
from perception.base import PerceptionBackend, ScanResult


class TopicVisionBackend(PerceptionBackend):
    """Use target_detector.py output instead of inline camera processing."""

    def __init__(self):
        self.config: Optional[MissionConfig] = None
        self._required: Dict[str, int] = {}
        self._state_ref: Optional[MissionState] = None
        self._lock = threading.RLock()
        self._latest_color = 'None'
        rospy.Subscriber('/target_current', String, self._detect_cb, queue_size=10)

    def _detect_cb(self, msg: String) -> None:
        with self._lock:
            self._latest_color = msg.data

    def set_mission_state(self, state: MissionState) -> None:
        self._state_ref = state

    def load_config(self, mission_config: MissionConfig) -> None:
        self.config = mission_config
        self._required = mission_config.required_counts()
        self.reset_counts()

    def reset_counts(self) -> None:
        with self._lock:
            self._latest_color = 'None'
        if self._state_ref:
            self._state_ref.counts = {k: 0 for k in self._required}
            self._state_ref.target_records = []
            self._state_ref._next_object_id = 1
            self._state_ref._known_positions = {k: [] for k in self._required}

    def get_counts(self) -> Dict[str, int]:
        if self._state_ref:
            return dict(self._state_ref.counts)
        return {}

    def detect(
        self,
        bgr_image=None,
        depth=None,
        camera_info=None,
        robot_pose=None,
        mission_state=None,
    ) -> ScanResult:
        """20260706：已禁用 — 话题感知不再登记颜色形状。"""
        state = mission_state or self._state_ref
        counts = state.counts if state else {}
        return ScanResult(counts=dict(counts), mission_complete=False)
        # state = mission_state or self._state_ref
        # if self.config is None or state is None:
        #     counts = state.counts if state else {}
        #     complete = state.is_mission_complete() if state else False
        #     return ScanResult(counts=dict(counts), mission_complete=complete)
        #
        # with self._lock:
        #     color = self._latest_color
        #
        # new_objects = 0
        # if color and color not in ('None', ''):
        #     for target in self.config.targets:
        #         if target.color != color:
        #             continue
        #         key = '{}/{}'.format(target.color, target.shape)
        #         if state.counts.get(key, 0) >= target.count:
        #             continue
        #         if state.try_register_target(target.color, target.shape, (160, 120), None):
        #             new_objects += 1
        #             rospy.loginfo('Registered target: %s/%s', target.color, target.shape)
        #             break
        #
        # return ScanResult(
        #     counts=dict(state.counts),
        #     mission_complete=state.is_mission_complete(),
        #     new_objects=new_objects,
        # )
