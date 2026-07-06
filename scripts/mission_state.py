"""Central mission map/state — visited cells, targets, coverage progress."""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Set, Tuple


@dataclass
class TargetRecord:
    object_id: int
    color: str
    shape: str
    world_pos: Tuple[float, float, float]
    pixel_center: Tuple[int, int]
    stop_index: int


@dataclass
class MissionProgress:
    counts: Dict[str, int] = field(default_factory=dict)
    required: Dict[str, int] = field(default_factory=dict)
    mission_complete: bool = False
    coverage_ratio: float = 0.0
    visited_cells: int = 0
    free_cells: int = 0
    boundary_events: int = 0
    scan_stops: int = 0


class MissionState:
  """Stores exploration progress and registered target objects."""

  def __init__(self, required_counts: Dict[str, int], dedup_world_dist: float = 0.25):
    self.required = dict(required_counts)
    self.counts: Dict[str, int] = {k: 0 for k in required_counts}
    self.dedup_world_dist = dedup_world_dist

    self.visited_cells: Set[Tuple[int, int]] = set()
    self.boundary_events: int = 0
    self.scan_stops: int = 0
    self.target_records: List[TargetRecord] = []
    self._next_object_id = 1
    self._known_positions: Dict[str, List[Tuple[float, float, float]]] = {
        k: [] for k in required_counts
    }

    self.coverage_ratio: float = 0.0
    self.free_cells: int = 0

  def mark_visited(self, grid_x: int, grid_y: int) -> None:
    self.visited_cells.add((grid_x, grid_y))

  def record_boundary(self) -> None:
    self.boundary_events += 1

  def record_scan_stop(self) -> None:
    self.scan_stops += 1

  def update_coverage(self, visited: int, free: int) -> None:
    self.free_cells = free
    if free > 0:
      self.coverage_ratio = min(1.0, float(visited) / float(free))
    else:
      self.coverage_ratio = 0.0

  def is_mission_complete(self) -> bool:
    for key, needed in self.required.items():
      if self.counts.get(key, 0) < needed:
        return False
    return bool(self.required)

  def is_coverage_complete(self, threshold: float = 0.92) -> bool:
    return self.free_cells > 0 and self.coverage_ratio >= threshold

  def try_register_target(
      self,
      color: str,
      shape: str,
      pixel_center: Tuple[int, int],
      world_pos: Optional[Tuple[float, float, float]],
  ) -> bool:
    """Return True if this is a new target object (world or pixel dedup)."""
    key = '{}/{}'.format(color, shape)
    if key not in self.required:
      return False
    if self.counts.get(key, 0) >= self.required[key]:
      return False

    if world_pos is not None:
      if self._is_world_duplicate(key, world_pos):
        return False
      self._known_positions.setdefault(key, []).append(world_pos)
    else:
      if self._is_pixel_duplicate(key, pixel_center):
        return False

    record = TargetRecord(
        object_id=self._next_object_id,
        color=color,
        shape=shape,
        world_pos=world_pos or (0.0, 0.0, 0.0),
        pixel_center=pixel_center,
        stop_index=self.scan_stops,
    )
    self._next_object_id += 1
    self.target_records.append(record)
    self.counts[key] = self.counts.get(key, 0) + 1
    return True

  def _is_world_duplicate(self, key: str, pos: Tuple[float, float, float]) -> bool:
    for seen in self._known_positions.get(key, []):
      dx = pos[0] - seen[0]
      dy = pos[1] - seen[1]
      if math.hypot(dx, dy) <= self.dedup_world_dist:
        return True
    return False

  def _is_pixel_duplicate(self, key: str, center: Tuple[int, int]) -> bool:
    dedup_px = 40.0
    for rec in self.target_records:
      if '{}/{}'.format(rec.color, rec.shape) != key:
        continue
      if rec.world_pos != (0.0, 0.0, 0.0):
        continue
      dx = center[0] - rec.pixel_center[0]
      dy = center[1] - rec.pixel_center[1]
      if math.hypot(dx, dy) <= dedup_px:
        return True
    return False

  def progress(self) -> MissionProgress:
    return MissionProgress(
        counts=dict(self.counts),
        required=dict(self.required),
        mission_complete=self.is_mission_complete(),
        coverage_ratio=self.coverage_ratio,
        visited_cells=len(self.visited_cells),
        free_cells=self.free_cells,
        boundary_events=self.boundary_events,
        scan_stops=self.scan_stops,
    )

  def status_line(self) -> str:
    p = self.progress()
    return (
        'counts={} coverage={:.0%} visited={} boundaries={} scans={}'
    ).format(p.counts, p.coverage_ratio, p.visited_cells, p.boundary_events, p.scan_stops)
