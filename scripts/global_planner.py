"""Global search planning on OccupancyGrid — perimeter, frontier, coverage."""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

import rospy

from occupancy_grid import OccupancyGrid


@dataclass
class GlobalPlan:
  mode: str
  goal: Optional[Tuple[float, float]] = None
  waypoints: List[Tuple[float, float]] = field(default_factory=list)
  status: str = 'ok'
  debug: Dict[str, int] = field(default_factory=dict)
  message: str = ''


class GlobalPlanner:
  """Decide search mode and produce global goals / paths (A* via OccupancyGrid)."""

  MODES = ('perimeter', 'frontier', 'coverage')

  def __init__(
      self,
      grid: OccupancyGrid,
      initial_mode: str = 'frontier',
      perimeter_enabled: bool = False,
      perimeter_min_visited: int = 120,
      perimeter_yaw_deg: float = 340.0,
      perimeter_max_sec: float = 180.0,
      perimeter_min_coverage: float = 0.12,
      astar_max_candidates: int = 25,
  ):
    self.grid = grid
    self.initial_mode = initial_mode
    self.perimeter_enabled = perimeter_enabled
    self.perimeter_min_visited = perimeter_min_visited
    self.perimeter_yaw_deg = perimeter_yaw_deg
    self.perimeter_max_sec = perimeter_max_sec
    self.perimeter_min_coverage = perimeter_min_coverage
    self.astar_max_candidates = astar_max_candidates
    self._mode = self._resolve_initial_mode()
    self._yaw_travel_rad = 0.0
    self._last_yaw: Optional[float] = None
    self._started_at = rospy.Time.now()
    self._perimeter_complete = False

  def current_mode(self) -> str:
    return self._mode

  def _resolve_initial_mode(self) -> str:
    if not self.perimeter_enabled:
      return 'frontier'
    mode = self.initial_mode
    return mode if mode in self.MODES else 'frontier'

  def reset(self) -> None:
    self._mode = self._resolve_initial_mode()
    self._yaw_travel_rad = 0.0
    self._last_yaw = None
    self._started_at = rospy.Time.now()
    self._perimeter_complete = False

  @staticmethod
  def _normalize_yaw_delta(delta: float) -> float:
    while delta > math.pi:
      delta -= 2.0 * math.pi
    while delta < -math.pi:
      delta += 2.0 * math.pi
    return delta

  def note_pose(self, robot_yaw: float) -> None:
    if self._last_yaw is not None:
      delta = self._normalize_yaw_delta(robot_yaw - self._last_yaw)
      self._yaw_travel_rad += abs(delta)
    self._last_yaw = robot_yaw

  def _perimeter_elapsed_sec(self) -> float:
    return (rospy.Time.now() - self._started_at).to_sec()

  def tick_perimeter_progress(
      self,
      robot_x: float,
      robot_y: float,
      robot_yaw: float,
  ) -> bool:
    """Update perimeter metrics; return True when perimeter phase should end."""
    self.note_pose(robot_yaw)
    return self._check_perimeter_done(robot_x, robot_y)

  def update(
      self,
      robot_x: float,
      robot_y: float,
      robot_yaw: float,
  ) -> Dict[str, object]:
    """Called every MOVE tick during perimeter — track progress, detect completion."""
    done = self.tick_perimeter_progress(robot_x, robot_y, robot_yaw)
    visited, free, unknown = self.grid.count_states()
    return {
        'mode': self._mode,
        'perimeter_done': done,
        'yaw_travel_deg': math.degrees(self._yaw_travel_rad),
        'visited': visited,
        'free': free,
        'unknown': unknown,
        'elapsed_sec': self._perimeter_elapsed_sec(),
    }

  def _check_perimeter_done(self, robot_x: float, robot_y: float) -> bool:
    if self._perimeter_complete:
      return True
    visited, free, _ = self.grid.count_states()
    coverage = self.grid.coverage_ratio()
    yaw_deg = math.degrees(self._yaw_travel_rad)
    elapsed = self._perimeter_elapsed_sec()
    done = False
    reason = ''
    if visited >= self.perimeter_min_visited:
      done = True
      reason = 'visited={}'.format(visited)
    elif yaw_deg >= self.perimeter_yaw_deg:
      done = True
      reason = 'yaw={:.0f}deg'.format(yaw_deg)
    elif coverage >= self.perimeter_min_coverage and yaw_deg >= self.perimeter_yaw_deg * 0.65:
      done = True
      reason = 'cov={:.0%} yaw={:.0f}deg'.format(coverage, yaw_deg)
    elif elapsed >= self.perimeter_max_sec:
      done = True
      reason = 'timeout {:.0f}s'.format(elapsed)
    if done:
      self._perimeter_complete = True
      rospy.loginfo(
          'GlobalPlanner: perimeter complete ({}) visited=%d cov=%.0f%%',
          reason,
          visited,
          coverage * 100.0,
      )
    return done

  def plan(
      self,
      robot_x: float,
      robot_y: float,
      robot_yaw: float,
      exclude_regions: Optional[List[Tuple[float, float, float]]] = None,
      exclude_radius: float = 0.75,
      prefer_escape: bool = False,
      prefer_forward: bool = True,
  ) -> GlobalPlan:
    self.note_pose(robot_yaw)

    if self._mode == 'perimeter' and self.perimeter_enabled:
      if self._check_perimeter_done(robot_x, robot_y):
        self._mode = 'frontier'
        rospy.loginfo('GlobalPlanner: mode perimeter -> frontier')
      else:
        visited, free, unknown = self.grid.count_states()
        rospy.loginfo(
            'GlobalPlanner: mode=perimeter visited=%d free=%d unknown=%d '
            'yaw_travel=%.0fdeg elapsed=%.0fs',
            visited,
            free,
            unknown,
            math.degrees(self._yaw_travel_rad),
            self._perimeter_elapsed_sec(),
        )
        return GlobalPlan(mode='perimeter', status='ok', message='perimeter_mapping')

    if self._mode == 'frontier':
      goal, path, dbg = self.grid.nearest_frontier(
          robot_x,
          robot_y,
          robot_yaw=robot_yaw,
          exclude_regions=exclude_regions or [],
          exclude_radius=exclude_radius,
          prefer_escape=prefer_escape,
          max_astar_candidates=self.astar_max_candidates,
          prefer_forward=prefer_forward,
      )
      if goal is None or not path:
        return GlobalPlan(
            mode='frontier',
            status='no_goal',
            debug=dbg,
            message='no reachable frontier',
        )
      rospy.loginfo(
          'GlobalPlanner: mode=frontier goal=(%.2f,%.2f) path=%d wp frontiers=%d',
          goal[0],
          goal[1],
          len(path),
          dbg.get('frontiers', 0),
      )
      return GlobalPlan(
          mode='frontier',
          goal=goal,
          waypoints=path,
          status='ok',
          debug=dbg,
          message='frontier',
      )

    if self._mode == 'coverage':
      rospy.logwarn('GlobalPlanner: mode=coverage not implemented — fallback frontier')
      self._mode = 'frontier'
      return self.plan(
          robot_x,
          robot_y,
          robot_yaw,
          exclude_regions=exclude_regions,
          exclude_radius=exclude_radius,
          prefer_escape=prefer_escape,
          prefer_forward=prefer_forward,
      )

    return GlobalPlan(mode=self._mode, status='error', message='unknown mode')
