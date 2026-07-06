"""A* path planning on the local occupancy grid."""

from __future__ import annotations

import heapq
import math
from typing import List, Optional, Tuple

import numpy as np

from occupancy_grid import CellState, OccupancyGrid


class GridPlanner:
  _NEIGHBORS = (
      (1, 0), (-1, 0), (0, 1), (0, -1),
      (1, 1), (1, -1), (-1, 1), (-1, -1),
  )

  def __init__(
      self,
      unknown_cost: float = 2.0,
      inflation_cells: int = 1,
      waypoint_spacing_m: float = 0.20,
  ):
    self.unknown_cost = unknown_cost
    self.inflation_cells = inflation_cells
    self.waypoint_spacing_m = waypoint_spacing_m

  def plan(
      self,
      grid: OccupancyGrid,
      start_wx: float,
      start_wy: float,
      goal_wx: float,
      goal_wy: float,
  ) -> Optional[List[Tuple[float, float]]]:
    """Return world-frame waypoints from start to goal, or None if unreachable."""
    start = grid._to_grid(start_wx, start_wy)
    goal = grid._to_grid(goal_wx, goal_wy)
    if not grid.in_bounds(*start):
      return None

    blocked = self._build_blocked_mask(grid)
    if not grid.in_bounds(*goal) or blocked[goal[1], goal[0]]:
      goal = self._nearest_free(grid, blocked, goal)
      if goal is None:
        return None

    path_cells = self._astar(grid, blocked, start, goal)
    if not path_cells:
      return None

    waypoints = self._cells_to_waypoints(grid, path_cells)
    waypoints = self._simplify(grid, blocked, waypoints)
    if not waypoints:
      return None
    if self._dist(waypoints[-1], (goal_wx, goal_wy)) > grid.resolution * 1.5:
      waypoints.append((goal_wx, goal_wy))
    return waypoints

  @staticmethod
  def _dist(a: Tuple[float, float], b: Tuple[float, float]) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])

  def _build_blocked_mask(self, grid: OccupancyGrid) -> np.ndarray:
    occupied = grid.grid == CellState.OCCUPIED
    if self.inflation_cells <= 0:
      return occupied

    blocked = occupied.copy()
    dim = grid.dim
    radius = self.inflation_cells
    occ_idx = np.argwhere(occupied)
    for gy, gx in occ_idx:
      for dy in range(-radius, radius + 1):
        for dx in range(-radius, radius + 1):
          if dx * dx + dy * dy > radius * radius:
            continue
          nx, ny = gx + dx, gy + dy
          if 0 <= nx < dim and 0 <= ny < dim:
            blocked[ny, nx] = True
    return blocked

  def _cell_cost(self, grid: OccupancyGrid, blocked: np.ndarray, gx: int, gy: int) -> Optional[float]:
    if blocked[gy, gx]:
      return None
    state = grid.grid[gy, gx]
    base: Optional[float] = None
    if state in (CellState.FREE, CellState.VISITED):
      base = 1.0
    elif state == CellState.UNKNOWN:
      base = self.unknown_cost
    if base is None:
      return None
    return base * grid.collision_multiplier(gx, gy)

  def _nearest_free(
      self,
      grid: OccupancyGrid,
      blocked: np.ndarray,
      center: Tuple[int, int],
  ) -> Optional[Tuple[int, int]]:
    cx, cy = center
    for radius in range(1, 6):
      for dy in range(-radius, radius + 1):
        for dx in range(-radius, radius + 1):
          if max(abs(dx), abs(dy)) != radius:
            continue
          gx, gy = cx + dx, cy + dy
          if not grid.in_bounds(gx, gy):
            continue
          if self._cell_cost(grid, blocked, gx, gy) is not None:
            return gx, gy
    return None

  def _heuristic(self, ax: int, ay: int, bx: int, by: int) -> float:
    return math.hypot(ax - bx, ay - by)

  def _astar(
      self,
      grid: OccupancyGrid,
      blocked: np.ndarray,
      start: Tuple[int, int],
      goal: Tuple[int, int],
  ) -> List[Tuple[int, int]]:
    sx, sy = start
    gx, gy = goal
    if start == goal:
      return [start]

    open_heap: List[Tuple[float, float, int, int]] = []
    counter = 0
    heapq.heappush(open_heap, (self._heuristic(sx, sy, gx, gy), 0.0, counter, sx, sy))
    came_from: dict = {}
    g_score = {(sx, sy): 0.0}
    closed = set()

    while open_heap:
      _f, g, _ctr, cx, cy = heapq.heappop(open_heap)
      if (cx, cy) in closed:
        continue
      closed.add((cx, cy))
      if (cx, cy) == (gx, gy):
        return self._reconstruct(came_from, (cx, cy))

      for dx, dy in self._NEIGHBORS:
        nx, ny = cx + dx, cy + dy
        if not grid.in_bounds(nx, ny):
          continue
        step_cost = self._cell_cost(grid, blocked, nx, ny)
        if step_cost is None:
          continue
        move_cost = math.sqrt(2.0) if dx != 0 and dy != 0 else 1.0
        tentative = g + move_cost * step_cost
        key = (nx, ny)
        if tentative >= g_score.get(key, float('inf')):
          continue
        g_score[key] = tentative
        came_from[key] = (cx, cy)
        counter += 1
        f = tentative + self._heuristic(nx, ny, gx, gy)
        heapq.heappush(open_heap, (f, tentative, counter, nx, ny))
    return []

  @staticmethod
  def _reconstruct(came_from: dict, current: Tuple[int, int]) -> List[Tuple[int, int]]:
    path = [current]
    while current in came_from:
      current = came_from[current]
      path.append(current)
    path.reverse()
    return path

  def _cells_to_waypoints(
      self,
      grid: OccupancyGrid,
      cells: List[Tuple[int, int]],
  ) -> List[Tuple[float, float]]:
    if not cells:
      return []
    spacing = max(grid.resolution, self.waypoint_spacing_m)
    waypoints: List[Tuple[float, float]] = []
    last: Optional[Tuple[float, float]] = None
    for gx, gy in cells:
      wx, wy = grid._to_world(gx, gy)
      if last is None or self._dist(last, (wx, wy)) >= spacing:
        waypoints.append((wx, wy))
        last = (wx, wy)
    final = grid._to_world(cells[-1][0], cells[-1][1])
    if not waypoints or self._dist(waypoints[-1], final) > grid.resolution * 0.5:
      waypoints.append(final)
    return waypoints

  def _line_clear(
      self,
      grid: OccupancyGrid,
      blocked: np.ndarray,
      a: Tuple[float, float],
      b: Tuple[float, float],
  ) -> bool:
    dist = self._dist(a, b)
    if dist < 1e-6:
      return True
    step = grid.resolution * 0.5
    n = max(1, int(dist / step))
    for i in range(n + 1):
      t = i / float(n)
      px = a[0] + t * (b[0] - a[0])
      py = a[1] + t * (b[1] - a[1])
      gx, gy = grid._to_grid(px, py)
      if not grid.in_bounds(gx, gy):
        return False
      if blocked[gy, gx]:
        return False
    return True

  def _simplify(
      self,
      grid: OccupancyGrid,
      blocked: np.ndarray,
      waypoints: List[Tuple[float, float]],
  ) -> List[Tuple[float, float]]:
    if len(waypoints) <= 2:
      return waypoints
    simplified = [waypoints[0]]
    anchor = 0
    idx = 1
    while idx < len(waypoints):
      if idx == len(waypoints) - 1:
        simplified.append(waypoints[idx])
        break
      if self._line_clear(grid, blocked, waypoints[anchor], waypoints[idx + 1]):
        idx += 1
        continue
      simplified.append(waypoints[idx])
      anchor = idx
      idx += 1
    return simplified

  def path_length(
      self,
      grid: OccupancyGrid,
      start_wx: float,
      start_wy: float,
      goal_wx: float,
      goal_wy: float,
  ) -> Optional[float]:
    path = self.plan(grid, start_wx, start_wy, goal_wx, goal_wy)
    if not path:
      return None
    total = 0.0
    px, py = start_wx, start_wy
    for wx, wy in path:
      total += math.hypot(wx - px, wy - py)
      px, py = wx, wy
    return total
