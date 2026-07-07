"""Local occupancy grid from LiDAR — FREE / OCCUPIED / VISITED / UNKNOWN."""

from __future__ import annotations

import math
from enum import IntEnum
from typing import Callable, Dict, List, Optional, Tuple, TYPE_CHECKING

import numpy as np
from sensor_msgs.msg import LaserScan

if TYPE_CHECKING:
  from grid_planner import GridPlanner


class CellState(IntEnum):
  UNKNOWN = 0
  FREE = 1
  OCCUPIED = 2
  VISITED = 3


class OccupancyGrid:
  def __init__(
      self,
      resolution: float = 0.08,
      size_m: float = 8.0,
      origin_x: float = 0.0,
      origin_y: float = 0.0,
  ):
    self.resolution = resolution
    self.size_m = size_m
    self.origin_x = origin_x
    self.origin_y = origin_y
    self.dim = int(size_m / resolution)
    self.grid = np.full((self.dim, self.dim), CellState.UNKNOWN, dtype=np.int8)
    self.collision_cost = np.zeros((self.dim, self.dim), dtype=np.float32)
    self._planner: Optional['GridPlanner'] = None

  def _to_grid(self, wx: float, wy: float) -> Tuple[int, int]:
    gx = int((wx - self.origin_x) / self.resolution) + self.dim // 2
    gy = int((wy - self.origin_y) / self.resolution) + self.dim // 2
    return gx, gy

  def _to_world(self, gx: int, gy: int) -> Tuple[float, float]:
    wx = (gx - self.dim // 2) * self.resolution + self.origin_x
    wy = (gy - self.dim // 2) * self.resolution + self.origin_y
    return wx, wy

  def set_origin(self, origin_x: float, origin_y: float, reset: bool = False) -> None:
    """Move grid origin (typically to robot start pose). Optionally clear cells."""
    self.origin_x = origin_x
    self.origin_y = origin_y
    if reset:
      self.grid = np.full((self.dim, self.dim), CellState.UNKNOWN, dtype=np.int8)

  def robot_in_bounds(self, wx: float, wy: float) -> bool:
    gx, gy = self._to_grid(wx, wy)
    return self.in_bounds(gx, gy)

  def in_bounds(self, gx: int, gy: int) -> bool:
    return 0 <= gx < self.dim and 0 <= gy < self.dim

  def mark_robot(self, wx: float, wy: float) -> Tuple[int, int]:
    gx, gy = self._to_grid(wx, wy)
    if self.in_bounds(gx, gy):
      if self.grid[gy, gx] in (CellState.UNKNOWN, CellState.FREE):
        self.grid[gy, gx] = CellState.VISITED
    return gx, gy

  def update_scan(
      self,
      scan: LaserScan,
      robot_x: float,
      robot_y: float,
      robot_yaw: float,
      front_angle: float = math.pi,
  ) -> None:
    if not scan.ranges:
      return

    self.mark_robot(robot_x, robot_y)

    for idx, rng in enumerate(scan.ranges):
      if not (scan.range_min <= rng <= scan.range_max):
        continue
      beam = scan.angle_min + idx * scan.angle_increment
      angle = robot_yaw + (beam - front_angle)
      ox, oy = robot_x, robot_y
      ex = ox + rng * math.cos(angle)
      ey = oy + rng * math.sin(angle)

      steps = int(rng / self.resolution)
      for step in range(max(1, steps)):
        t = step / float(max(1, steps))
        px = ox + (ex - ox) * t
        py = oy + (ey - oy) * t
        gx, gy = self._to_grid(px, py)
        if not self.in_bounds(gx, gy):
          break
        if self.grid[gy, gx] == CellState.UNKNOWN:
          self.grid[gy, gx] = CellState.FREE

      egx, egy = self._to_grid(ex, ey)
      if self.in_bounds(egx, egy):
        self.grid[egy, egx] = CellState.OCCUPIED

  def count_states(self) -> Tuple[int, int, int]:
    visited = int(np.sum(self.grid == CellState.VISITED))
    free = int(np.sum((self.grid == CellState.FREE) | (self.grid == CellState.VISITED)))
    unknown = int(np.sum(self.grid == CellState.UNKNOWN))
    return visited, free, unknown

  def find_frontiers(self) -> List[Tuple[int, int]]:
    frontiers = []
    for gy in range(1, self.dim - 1):
      for gx in range(1, self.dim - 1):
        if self.grid[gy, gx] != CellState.FREE:
          continue
        if self.grid[gy, gx] == CellState.VISITED:
          continue
        has_unknown = False
        for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
          nx, ny = gx + dx, gy + dy
          if self.in_bounds(nx, ny) and self.grid[ny, nx] == CellState.UNKNOWN:
            has_unknown = True
            break
        if has_unknown:
          frontiers.append((gx, gy))
    return frontiers

  def attach_planner(self, planner: 'GridPlanner') -> None:
    self._planner = planner

  def record_collision(
      self,
      wx: float,
      wy: float,
      radius_m: float = 0.30,
      increment: float = 1.0,
      max_cost: float = 8.0,
  ) -> None:
    gx, gy = self._to_grid(wx, wy)
    r_cells = max(1, int(radius_m / self.resolution))
    for dy in range(-r_cells, r_cells + 1):
      for dx in range(-r_cells, r_cells + 1):
        if dx * dx + dy * dy > r_cells * r_cells:
          continue
        nx, ny = gx + dx, gy + dy
        if self.in_bounds(nx, ny):
          old = float(self.collision_cost[ny, nx])
          self.collision_cost[ny, nx] = min(max_cost, old + increment)

  def collision_multiplier(self, gx: int, gy: int) -> float:
    if not self.in_bounds(gx, gy):
      return 1.0
    return 1.0 + float(self.collision_cost[gy, gx])

  def plan_path(
      self,
      start_wx: float,
      start_wy: float,
      goal_wx: float,
      goal_wy: float,
  ) -> Optional[List[Tuple[float, float]]]:
    if self._planner is None:
      if self.path_is_clear(start_wx, start_wy, goal_wx, goal_wy):
        return [(goal_wx, goal_wy)]
      return None
    return self._planner.plan(self, start_wx, start_wy, goal_wx, goal_wy)

  def path_is_clear(
      self,
      x0: float,
      y0: float,
      x1: float,
      y1: float,
      step: float = 0.05,
  ) -> bool:
    """Sample line from robot to goal; reject if any cell is OCCUPIED."""
    dist = math.hypot(x1 - x0, y1 - y0)
    if dist < 1e-6:
      return True
    n = max(1, int(dist / step))
    for i in range(n + 1):
      t = i / float(n)
      px = x0 + t * (x1 - x0)
      py = y0 + t * (y1 - y0)
      gx, gy = self._to_grid(px, py)
      if not self.in_bounds(gx, gy):
        return False
      if self.grid[gy, gx] == CellState.OCCUPIED:
        return False
    return True

  @staticmethod
  def _min_exclude_dist(
      wx: float,
      wy: float,
      exclude: List[Tuple[float, float]],
  ) -> float:
    if not exclude:
      return float('inf')
    return min(math.hypot(wx - ex, wy - ey) for ex, ey in exclude)

  @staticmethod
  def _is_region_blocked(
      wx: float,
      wy: float,
      exclude_regions: List[Tuple[float, float, float]],
      default_radius: float,
  ) -> bool:
    for ex, ey, radius in exclude_regions:
      r = radius if radius > 0.0 else default_radius
      if math.hypot(wx - ex, wy - ey) < r:
        return True
    return False

  @staticmethod
  def _region_sep_dist(
      wx: float,
      wy: float,
      exclude_regions: List[Tuple[float, float, float]],
  ) -> float:
    if not exclude_regions:
      return float('inf')
    return min(
        max(0.0, math.hypot(wx - ex, wy - ey) - radius)
        for ex, ey, radius in exclude_regions
    )

  @staticmethod
  def _heading_penalty(
      robot_x: float,
      robot_y: float,
      robot_yaw: float,
      wx: float,
      wy: float,
  ) -> float:
    bearing = math.atan2(wy - robot_y, wx - robot_x)
    err = bearing - robot_yaw
    while err > math.pi:
      err -= 2.0 * math.pi
    while err < -math.pi:
      err += 2.0 * math.pi
    return abs(err)

  def nearest_frontier(
      self,
      robot_x: float,
      robot_y: float,
      robot_yaw: Optional[float] = None,
      exclude: Optional[List[Tuple[float, float]]] = None,
      exclude_regions: Optional[List[Tuple[float, float, float]]] = None,
      exclude_radius: float = 0.2,
      min_dist: float = 0.12,
      prefer_escape: bool = False,
      max_astar_candidates: int = 30,
      prefer_forward: bool = True,
      forward_max_deg: float = 110.0,
      eval_top_n: int = 10,
      narrow_penalty_weight: float = 0.40,
      failure_penalty_weight: float = 1.0,
      goal_soft_penalty: Optional[Callable[[float, float], float]] = None,
  ) -> Tuple[Optional[Tuple[float, float]], List[Tuple[float, float]], Dict[str, int]]:
    """Pick exploration goal with A* path; return (goal, waypoints, debug_stats).

    20260706：边界多候选评分选点；对排序后前 eval_top_n 个候选均做路径搜索，取总代价最低者。
    总代价 = 路径长度 + 狭窄代价 + 历史失败软惩罚（逃离时另计远离失败区奖励）。
    """
    exclude = exclude or []
    exclude_regions = exclude_regions or []
    debug: Dict[str, int] = {
        'frontiers': 0,
        'candidates_raw': 0,
        'candidates': 0,
        'blocked_skipped': 0,
        'min_dist_skipped': 0,
        'path_rejected': 0,
        'path_found': 0,
        'path_waypoints': 0,
        'astar_evaluated': 0,
    }
    frontiers = self.find_frontiers()
    debug['frontiers'] = len(frontiers)
    if not frontiers:
      unvisited_free = np.argwhere(self.grid == CellState.FREE)
      if unvisited_free.size == 0:
        return None, [], debug
      candidates = []
      for gy, gx in unvisited_free:
        wx, wy = self._to_world(int(gx), int(gy))
        candidates.append((wx, wy))
    else:
      candidates = [self._to_world(gx, gy) for gx, gy in frontiers]

    debug['candidates_raw'] = len(candidates)

    ranked: List[Tuple[float, float, float, float]] = []
    forward_limit = math.radians(forward_max_deg)
    for wx, wy in candidates:
      if self._is_region_blocked(wx, wy, exclude_regions, exclude_radius):
        debug['blocked_skipped'] += 1
        continue
      blocked = False
      for ex, ey in exclude:
        if math.hypot(wx - ex, wy - ey) < exclude_radius:
          debug['blocked_skipped'] += 1
          blocked = True
          break
      if blocked:
        continue
      d = math.hypot(wx - robot_x, wy - robot_y)
      if d < min_dist:
        debug['min_dist_skipped'] += 1
        continue
      if (
          prefer_forward
          and robot_yaw is not None
          and not prefer_escape
          and self._heading_penalty(robot_x, robot_y, robot_yaw, wx, wy) > forward_limit
      ):
        debug['min_dist_skipped'] += 1
        continue
      # 20260706：逃离选点时跳过前方扇区过滤，允许侧后方边界点
      if exclude_regions:
        sep = self._region_sep_dist(wx, wy, exclude_regions)
      else:
        sep = self._min_exclude_dist(wx, wy, exclude)
      heading_pen = (
          self._heading_penalty(robot_x, robot_y, robot_yaw, wx, wy)
          if robot_yaw is not None else 0.0
      )
      gx_i, gy_i = self._to_grid(wx, wy)
      coll_pen = 0.0
      if self.in_bounds(gx_i, gy_i):
        coll_pen = float(self.collision_cost[gy_i, gx_i]) * 0.15
      ranked.append((d + coll_pen, wx, wy, sep, heading_pen))
    if prefer_escape and (exclude_regions or exclude):
      ranked.sort(key=lambda item: (-item[3], item[0]))
      debug['escape_pick'] = 1
    elif robot_yaw is not None and prefer_forward:
      ranked.sort(key=lambda item: (item[4], item[0]))
    else:
      ranked.sort(key=lambda item: item[0])
    debug['candidates'] = len(ranked)

    # 20260706：评估多条候选取总代价最低（不再首条可达即停止）
    eval_limit = max(1, min(eval_top_n, max_astar_candidates, len(ranked)))
    best: Optional[Tuple[float, Tuple[float, float], List[Tuple[float, float]]]] = None
    best_total = float('inf')
    for d, wx, wy, sep, heading_pen in ranked[:eval_limit]:
      debug['astar_evaluated'] += 1
      path = self.plan_path(robot_x, robot_y, wx, wy)
      if path is None:
        debug['path_rejected'] += 1
        continue
      path_len = self._path_length(robot_x, robot_y, path)
      narrow_pen = self._path_narrow_penalty(path)
      fail_pen = 0.0
      if goal_soft_penalty is not None:
        fail_pen = max(0.0, float(goal_soft_penalty(wx, wy)))
      heading_cost = heading_pen * 0.20 if prefer_forward and not prefer_escape else 0.0
      escape_bonus = sep * 0.85 if prefer_escape and (exclude_regions or exclude) else 0.0
      total = (
          path_len
          + narrow_penalty_weight * narrow_pen
          + failure_penalty_weight * fail_pen
          + heading_cost
          - escape_bonus
      )
      if total < best_total:
        best_total = total
        best = (total, (wx, wy), path)
        debug['pick_path_len'] = int(path_len * 100)
        debug['pick_narrow'] = int(narrow_pen * 100)
        debug['pick_failure'] = int(fail_pen * 100)
        debug['pick_total'] = int(total * 100)

    if best is not None:
      _total, goal, path = best
      debug['path_found'] = 1
      debug['path_waypoints'] = len(path)
      return goal, path, debug
    return None, [], debug

  def _path_narrow_penalty(self, path: List[Tuple[float, float]]) -> float:
    """20260706：路径沿途占用格邻近度，估计边界目标狭窄代价。"""
    if not path:
      return 0.0
    penalty = 0.0
    for wx, wy in path:
      gx, gy = self._to_grid(wx, wy)
      occ_near = 0
      for dx in range(-2, 3):
        for dy in range(-2, 3):
          nx, ny = gx + dx, gy + dy
          if self.in_bounds(nx, ny) and self.grid[ny, nx] == CellState.OCCUPIED:
            occ_near += 1
      if occ_near >= 5:
        penalty += 0.50
      elif occ_near >= 3:
        penalty += 0.25
      elif occ_near >= 1:
        penalty += 0.08
    return penalty

  # --- 20260706：已禁用 — 首条可达即停的旧选点逻辑 ---
  # if not prefer_escape:
  #   break

  @staticmethod
  def _path_length(
      start_wx: float,
      start_wy: float,
      waypoints: List[Tuple[float, float]],
  ) -> float:
    total = 0.0
    px, py = start_wx, start_wy
    for wx, wy in waypoints:
      total += math.hypot(wx - px, wy - py)
      px, py = wx, wy
    return total

  def coverage_ratio(self) -> float:
    visited, free, _ = self.count_states()
    if free == 0:
      return 0.0
    return min(1.0, float(visited) / float(free))
