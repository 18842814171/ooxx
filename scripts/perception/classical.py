"""Classical color + shape identification — extends color_detect_demo.py."""

from __future__ import annotations

import math
from typing import Dict, List, Optional, Set, Tuple

import cv2
import numpy as np

from config_loader import MissionConfig
from depth_projection import pixel_to_world
from mission_state import MissionState
from perception.base import Detection, PerceptionBackend, ScanResult


def _val_map(x, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min


class ClassicalBackend(PerceptionBackend):
    def __init__(self):
        self.config: Optional[MissionConfig] = None
        self._required: Dict[str, int] = {}
        self._pending: List[List[Detection]] = []
        self._state_ref: Optional[MissionState] = None

    def set_mission_state(self, state: MissionState) -> None:
        self._state_ref = state

    def load_config(self, mission_config: MissionConfig) -> None:
        self.config = mission_config
        self._required = mission_config.required_counts()
        self.reset_counts()

    def reset_counts(self) -> None:
        self._pending = []
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
        bgr_image,
        depth=None,
        camera_info=None,
        robot_pose=None,
        mission_state=None,
    ) -> ScanResult:
        """20260706：已禁用 — 感知层不再驱动任务计数与结束。"""
        state = mission_state or self._state_ref
        counts = state.counts if state else {}
        return ScanResult(counts=dict(counts), mission_complete=False)
        # state = mission_state or self._state_ref
        # if self.config is None or bgr_image is None or state is None:
        #     counts = state.counts if state else {}
        #     complete = state.is_mission_complete() if state else False
        #     return ScanResult(counts=dict(counts), mission_complete=complete)
        #
        # frame = bgr_image.copy()
        # detections = self._detect_frame(frame, depth, camera_info, robot_pose)
        # new_objects = self._accumulate(detections, state)
        #
        # return ScanResult(
        #     detections=detections,
        #     counts=dict(state.counts),
        #     mission_complete=state.is_mission_complete(),
        #     new_objects=new_objects,
        # )

    def _detect_frame(self, img, depth, camera_info, robot_pose) -> List[Detection]:
        assert self.config is not None
        cfg = self.config
        proc_w, proc_h = cfg.vision.proc_size
        img_h, img_w = img.shape[:2]

        resized = cv2.resize(img, (proc_w, proc_h), interpolation=cv2.INTER_NEAREST)
        blurred = cv2.GaussianBlur(resized, (3, 3), 3)
        lab = cv2.cvtColor(blurred, cv2.COLOR_BGR2LAB)

        results: List[Detection] = []
        target_shapes: Set[str] = {t.shape for t in cfg.targets}

        for color_name in cfg.active_colors:
            if color_name not in cfg.colors:
                continue
            spec = cfg.colors[color_name]
            mask = cv2.inRange(lab, spec.lab_min, spec.lab_max)
            mask = cv2.erode(mask, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))
            mask = cv2.dilate(mask, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

            for contour in contours:
                area = abs(cv2.contourArea(contour))
                if area < cfg.vision.min_contour_area:
                    continue
                shape = self._classify_shape(contour, target_shapes)
                if shape is None or not self._matches_target(color_name, shape):
                    continue

                moments = cv2.moments(contour)
                if moments['m00'] == 0:
                    continue
                cx = int(moments['m10'] / moments['m00'])
                cy = int(moments['m01'] / moments['m00'])
                full_cx = int(_val_map(cx, 0, proc_w, 0, img_w))
                full_cy = int(_val_map(cy, 0, proc_h, 0, img_h))

                world_pos = None
                if depth is not None and camera_info is not None and robot_pose is not None:
                    world_pos = pixel_to_world(full_cx, full_cy, depth, camera_info, robot_pose)

                confidence = min(1.0, area / max(cfg.vision.confirm_area, 1.0))
                results.append(Detection(
                    color_name, shape, (full_cx, full_cy), area, confidence, world_pos
                ))

        return results

    def _matches_target(self, color: str, shape: str) -> bool:
        assert self.config is not None
        for target in self.config.targets:
            if target.color == color and target.shape == shape:
                return True
        return False

    def _classify_shape(self, contour, allowed: Set[str]) -> Optional[str]:
        assert self.config is not None
        perimeter = cv2.arcLength(contour, True)
        if perimeter <= 0:
            return None

        if 'circle' in allowed:
            circularity = 4.0 * math.pi * abs(cv2.contourArea(contour)) / (perimeter * perimeter)
            circle_spec = self.config.shapes.get('circle')
            min_circ = 0.75
            if circle_spec:
                min_circ = float(circle_spec.params.get('min_circularity', 0.75))
            if circularity >= min_circ:
                return 'circle'

        for shape_name in ('triangle', 'rectangle'):
            if shape_name not in allowed or shape_name not in self.config.shapes:
                continue
            spec = self.config.shapes[shape_name].params
            vertices = int(spec.get('vertices', 3 if shape_name == 'triangle' else 4))
            eps_ratio = float(spec.get('epsilon_ratio', 0.04))
            approx = cv2.approxPolyDP(contour, eps_ratio * perimeter, True)
            if len(approx) != vertices:
                continue
            if shape_name == 'rectangle' and len(approx) == 4:
                pts = approx.reshape(4, 2)
                w = np.linalg.norm(pts[0] - pts[1])
                h = np.linalg.norm(pts[1] - pts[2])
                if w <= 0 or h <= 0:
                    continue
                aspect = min(w, h) / max(w, h)
                min_ar = float(spec.get('min_aspect_ratio', 0.3))
                max_ar = float(spec.get('max_aspect_ratio', 3.0))
                if aspect < min_ar or aspect > max_ar:
                    continue
            return shape_name
        return None

    def _accumulate(self, detections: List[Detection], state: MissionState) -> int:
        assert self.config is not None
        debounce = max(1, self.config.vision.debounce_frames)
        self._pending.append(detections)
        if len(self._pending) < debounce:
            return 0

        window = self._pending[-debounce:]
        stable = self._stable_in_window(window)
        new_objects = 0

        for det in stable:
            registered = state.try_register_target(
                det.color,
                det.shape,
                det.center,
                det.world_pos,
            )
            if registered:
                new_objects += 1

        return new_objects

    def _stable_in_window(self, window: List[List[Detection]]) -> List[Detection]:
        if not window:
            return []
        if len(window) == 1:
            return window[0]

        def key(det: Detection):
            return det.color, det.shape, det.center[0] // 20, det.center[1] // 20

        sets = [set(key(d) for d in frame) for frame in window]
        common = set.intersection(*sets) if sets else set()
        return [d for d in window[-1] if key(d) in common]


class DLBackend(PerceptionBackend):
    def __init__(self):
        self._required = {}
        self._counts = {}

    def load_config(self, mission_config: MissionConfig) -> None:
        self._required = mission_config.required_counts()
        self._counts = {k: 0 for k in self._required}

    def detect(self, bgr_image, depth=None, camera_info=None, robot_pose=None, mission_state=None) -> ScanResult:
        raise NotImplementedError('DLBackend not implemented. Use vision.backend: classical')

    def reset_counts(self) -> None:
        self._counts = {k: 0 for k in self._required}

    def get_counts(self) -> Dict[str, int]:
        return dict(self._counts)


def create_perception_backend(mission_config: MissionConfig) -> PerceptionBackend:
    backend_name = mission_config.vision.backend
    if backend_name == 'classical':
        backend = ClassicalBackend()
    elif backend_name == 'topic':
        from perception.topic import TopicVisionBackend
        backend = TopicVisionBackend()
    elif backend_name == 'dl':
        backend = DLBackend()
    else:
        raise ValueError('Unknown vision backend: {}'.format(backend_name))
    backend.load_config(mission_config)
    return backend
