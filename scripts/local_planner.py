"""Local planning — unified cost, dynamic distance, Navigator contract."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Optional, Tuple

import rospy
from sensor_msgs.msg import LaserScan

from frontier_navigator import FrontierNavigator
from lidar_utils import clearance_profile, sector_min
from nav_feedback import GoalStatus
from occupancy_grid import CellState, OccupancyGrid
from pose_estimator import PoseEstimator


@dataclass
class ScoreBreakdown:
  clearance: float = 0.0
  alignment: float = 0.0
  unknown: float = 0.0
  progress: float = 0.0
  lateral: float = 0.0
  curvature: float = 0.0
  collision: float = 0.0
  stress: float = 0.0
  execution: float = 0.0
  total: float = 0.0


@dataclass
class _FailureRecord:
  x: float
  y: float
  angle_deg: float
  reason: str
  time: rospy.Time


@dataclass
class _Candidate:
  angle_deg: float
  score: float
  gx: float
  gy: float
  path: List[Tuple[float, float]]
  dist: float
  clearance: float
  unknown: float
  valid: bool
  breakdown: ScoreBreakdown


class LocalPlanner:
  def __init__(
      self,
      grid: OccupancyGrid,
      nav: FrontierNavigator,
      pose: PoseEstimator,
      boundary_dist: float = 0.30,
      front_angle: float = math.pi,
      nav_half_width: float = 0.14,
      wide_half_width: float = 0.22,
      plan_dist_min: float = 0.4,
      plan_dist_max: float = 3.0,
      plan_progress_dist_cap: float = 2.0,
      plan_clearance_tie_band: float = 0.06,
      plan_angles: Tuple[float, ...] = (0.0, 20.0, 35.0, 50.0, 70.0),
      stress_decay_tau: float = 0.8,
      cruise_speed: float = 0.18,
      creep_speed: float = 0.06,
      collision_cost_max: float = 8.0,
      score_clearance: float = 0.58,
      score_alignment: float = 0.10,
      score_explore: float = 0.22,
      score_progress: float = 0.06,
      score_curvature: float = 0.14,
      score_collision: float = 0.12,
      score_stress: float = 0.08,
      score_execution: float = 0.10,
      recent_collision_penalty: float = 0.25,
      recent_collision_penalty_deg: float = 20.0,
      recent_collision_penalty_sec: float = 3.0,
      failure_memory_penalty: float = 0.35,
      failure_memory_angle_deg: float = 25.0,
      failure_memory_radius_m: float = 0.45,
      failure_memory_sec: float = 45.0,
      failure_memory_max_entries: int = 8,
      wall_hug_zero_penalty: float = 0.25,
      wall_hug_clearance_margin: float = 0.12,
      wall_hug_angle_deg: float = 25.0,
      failure_memory_cluster_count: int = 3,
      failure_memory_cluster_radius_m: float = 0.60,
      failure_memory_cluster_penalty: float = 0.50,
      recovery_escape_angle_deg: float = 50.0,
      recovery_escape_center_max: float = 0.35,
      recovery_replan_rounds: int = 3,
      planner_debug: bool = True,
  ):
    self.grid = grid
    self.nav = nav
    self.pose = pose
    self.boundary_dist = boundary_dist
    self.front_angle = front_angle
    self.nav_half_width = nav_half_width
    self.wide_half_width = wide_half_width
    self.plan_dist_min = plan_dist_min
    self.plan_dist_max = plan_dist_max
    self.plan_progress_dist_cap = plan_progress_dist_cap
    self.plan_clearance_tie_band = plan_clearance_tie_band
    self.plan_angles = plan_angles
    self.stress_decay_tau = stress_decay_tau
    self.cruise_speed = cruise_speed
    self.creep_speed = creep_speed
    self.collision_cost_max = collision_cost_max
    self.score_clearance = score_clearance
    self.score_alignment = score_alignment
    self.score_explore = score_explore
    self.score_progress = score_progress
    self.score_curvature = score_curvature
    self.score_collision = score_collision
    self.score_stress = score_stress
    self.score_execution = score_execution
    self.recent_collision_penalty = recent_collision_penalty
    self.recent_collision_penalty_deg = recent_collision_penalty_deg
    self.recent_collision_penalty_sec = recent_collision_penalty_sec
    self.failure_memory_penalty = failure_memory_penalty
    self.failure_memory_angle_deg = failure_memory_angle_deg
    self.failure_memory_radius_m = failure_memory_radius_m
    self.failure_memory_sec = failure_memory_sec
    self.failure_memory_max_entries = failure_memory_max_entries
    self.wall_hug_zero_penalty = wall_hug_zero_penalty
    self.wall_hug_clearance_margin = wall_hug_clearance_margin
    self.wall_hug_angle_deg = wall_hug_angle_deg
    self.failure_memory_cluster_count = failure_memory_cluster_count
    self.failure_memory_cluster_radius_m = failure_memory_cluster_radius_m
    self.failure_memory_cluster_penalty = failure_memory_cluster_penalty
    self.recovery_escape_angle_deg = recovery_escape_angle_deg
    self.recovery_escape_center_max = recovery_escape_center_max
    self.recovery_replan_rounds = recovery_replan_rounds
    self.planner_debug = planner_debug
    self._last_replan = rospy.Time(0)
    self._stress_smoothed = 0.0
    self._last_stress_update = rospy.Time.now()
    self._debug_tick = 0
    self._last_boundary_angle_deg: Optional[float] = None
    self._last_boundary_time: Optional[rospy.Time] = None
    self._last_selected_angle_deg: Optional[float] = None
    self._penalty_pending_replan = False
    self._post_recovery_rounds_left = 0
    self._post_recovery_max_angle_deg = 35.0
    self._post_recovery_max_dist_m = 2.0
    self._failure_memory: List[_FailureRecord] = []
    self._passage_replan_rounds_left = 0
    self._passage_replan_max_angle_deg = 70.0

  def has_goal(self) -> bool:
    return self.nav.has_goal()

  def clear(self) -> None:
    self.nav.clear_goal()

  def last_selected_angle_deg(self) -> float:
    return self._last_selected_angle_deg if self._last_selected_angle_deg is not None else 0.0

  def record_failure(self, x: float, y: float, angle_deg: float, reason: str) -> None:
    """Failure Memory：记录恢复触发位姿与候选偏角，短期内相近方向降权。"""
    now = rospy.Time.now()
    self._failure_memory.append(
        _FailureRecord(x, y, angle_deg, reason, now),
    )
    while len(self._failure_memory) > self.failure_memory_max_entries:
      self._failure_memory.pop(0)
    rospy.loginfo(
        'LocalPlanner: FailureMemory 记录 (%.2f,%.2f) 偏角=%.0f° 原因=%s',
        x,
        y,
        angle_deg,
        reason,
    )

  def arm_passage_replan(
      self,
      max_angle_deg: float,
      max_dist_m: float,
      rounds: int = 2,
  ) -> None:
    """窄通道恢复后：放宽偏角上限，允许选缝隙方向大角度目标。"""
    self._passage_replan_rounds_left = rounds
    self._passage_replan_max_angle_deg = max_angle_deg
    self.arm_post_recovery_replan(max_angle_deg, max_dist_m, rounds=rounds)
    rospy.loginfo(
        'LocalPlanner: 缝隙恢复规划 |偏角|<=%.0f° d<=%.2fm ×%d轮',
        max_angle_deg,
        max_dist_m,
        rounds,
    )

  def arm_post_recovery_replan(
      self,
      max_angle_deg: float,
      max_dist_m: float,
      rounds: Optional[int] = None,
  ) -> None:
    """状态机在 Recovery 结束后调用；后续若干轮 replan_local 施加约束。"""
    self._post_recovery_rounds_left = rounds if rounds is not None else self.recovery_replan_rounds
    self._post_recovery_max_angle_deg = max_angle_deg
    self._post_recovery_max_dist_m = max_dist_m

  def _filter_post_recovery_candidates(
      self,
      valid: List[_Candidate],
      profile_center: float,
  ) -> List[_Candidate]:
    if self._post_recovery_rounds_left <= 0 or not valid:
      return valid
    max_angle = self._post_recovery_max_angle_deg
    if self._passage_replan_rounds_left > 0:
      max_angle = max(max_angle, self._passage_replan_max_angle_deg)
    max_dist = self._post_recovery_max_dist_m
    round_no = self.recovery_replan_rounds - self._post_recovery_rounds_left + 1
    filtered = [
        c for c in valid
        if abs(c.angle_deg) <= max_angle + 0.5 and c.dist <= max_dist + 0.05
    ]
    if (
        profile_center < self.recovery_escape_center_max
        and valid
    ):
      outside = [
          c for c in valid
          if abs(c.angle_deg) > max_angle + 0.5 or c.dist > max_dist + 0.05
      ]
      if outside:
        best_out = max(outside, key=lambda c: (c.clearance, c.score))
        best_in = max(filtered, key=lambda c: (c.clearance, c.score)) if filtered else None
        in_clear = best_in.clearance if best_in is not None else 0.0
        if (
            abs(best_out.angle_deg) >= self.recovery_escape_angle_deg - 0.5
            and best_out.clearance >= in_clear + 0.08
        ):
          if best_out not in filtered:
            filtered = list(filtered) + [best_out]
          rospy.loginfo(
              'LocalPlanner: 墙角破格候选 %.0f° clear=%.2f d=%.2f',
              best_out.angle_deg,
              best_out.clearance,
              best_out.dist,
          )
    if filtered:
      tag = '缝隙' if self._passage_replan_rounds_left > 0 else '恢复后'
      rospy.loginfo(
          'LocalPlanner: %s第%d轮 |偏角|<=%.0f° d<=%.2fm 候选 %d→%d',
          tag,
          round_no,
          max_angle,
          max_dist,
          len(valid),
          len(filtered),
      )
      return filtered
    rospy.logwarn(
        'LocalPlanner: 恢复后第%d轮约束无候选，放宽至全量',
        round_no,
    )
    return valid

  def note_boundary_hit(self) -> None:
    """记录最近一次边界触发时选中的候选偏角（度）。"""
    angle = self._last_selected_angle_deg if self._last_selected_angle_deg is not None else 0.0
    self._last_boundary_angle_deg = angle
    self._last_boundary_time = rospy.Time.now()
    self._penalty_pending_replan = True
    rospy.loginfo('LocalPlanner: 记录边界候选偏角 %.0f°', angle)

  def refresh_penalty_for_replan(self) -> None:
    """恢复后即将重规划：刷新计时，确保首次规划仍受方向惩罚。"""
    if self._last_boundary_angle_deg is None:
      return
    self._last_boundary_time = rospy.Time.now()
    self._penalty_pending_replan = True
    rospy.loginfo(
        'LocalPlanner: 恢复后刷新方向惩罚 偏角=%.0f°',
        self._last_boundary_angle_deg,
    )

  def note_boundary(self, angle_deg: float) -> None:
    """兼容旧接口；优先使用 note_boundary_hit。"""
    self._last_boundary_angle_deg = angle_deg
    self._last_boundary_time = rospy.Time.now()
    rospy.loginfo('LocalPlanner: 记录边界方向 %.0f°', angle_deg)

  def _recent_boundary_penalty(self, angle_deg: float) -> float:
    if self._last_boundary_time is None or self._last_boundary_angle_deg is None:
      return 0.0
    elapsed = (rospy.Time.now() - self._last_boundary_time).to_sec()
    if elapsed > self.recent_collision_penalty_sec:
      return 0.0
    diff = abs(angle_deg - self._last_boundary_angle_deg)
    if diff > 180.0:
      diff = 360.0 - diff
    if diff < self.recent_collision_penalty_deg:
      return self.recent_collision_penalty
    return 0.0

  def _failure_memory_cluster_penalty(self, rx: float, ry: float) -> float:
    """Hot-zone penalty when multiple recoveries cluster nearby."""
    now = rospy.Time.now()
    count = 0
    for rec in self._failure_memory:
      elapsed = (now - rec.time).to_sec()
      if elapsed > self.failure_memory_sec:
        continue
      if math.hypot(rx - rec.x, ry - rec.y) <= self.failure_memory_cluster_radius_m:
        count += 1
    if count >= self.failure_memory_cluster_count:
      return self.failure_memory_cluster_penalty
    return 0.0

  def _failure_memory_penalty(self, rx: float, ry: float, angle_deg: float) -> float:
    now = rospy.Time.now()
    total = self._failure_memory_cluster_penalty(rx, ry)
    for rec in self._failure_memory:
      elapsed = (now - rec.time).to_sec()
      if elapsed > self.failure_memory_sec:
        continue
      dist = math.hypot(rx - rec.x, ry - rec.y)
      if dist > self.failure_memory_radius_m:
        continue
      diff = abs(angle_deg - rec.angle_deg)
      if diff > 180.0:
        diff = 360.0 - diff
      zero_pair = abs(angle_deg) < 5.0 and abs(rec.angle_deg) < 5.0
      if diff >= self.failure_memory_angle_deg and not zero_pair:
        continue
      proximity = 1.0 - dist / max(0.01, self.failure_memory_radius_m)
      total += self.failure_memory_penalty * proximity
    return min(total, self.failure_memory_penalty * 2.0)

  def _wall_hug_penalty(self, angle_deg: float, center: float) -> float:
    if abs(angle_deg) > self.wall_hug_angle_deg + 0.5:
      return 0.0
    limit = self.boundary_dist + self.wall_hug_clearance_margin
    if center >= limit:
      return 0.0
    tightness = 1.0 - max(0.0, center - self.boundary_dist) / max(0.01, self.wall_hug_clearance_margin)
    angle_scale = 1.0 - abs(angle_deg) / max(1.0, self.wall_hug_angle_deg)
    return self.wall_hug_zero_penalty * tightness * max(0.35, angle_scale)

  def _distance_penalty(self, dist: float) -> float:
    if dist <= self.plan_progress_dist_cap:
      return 0.0
    span = max(0.01, self.plan_dist_max - self.plan_progress_dist_cap)
    return 0.12 * min(1.0, (dist - self.plan_progress_dist_cap) / span)

  def _pick_best_candidate(self, ranked: List[_Candidate]) -> Optional[_Candidate]:
    if not ranked:
      return None
    top = ranked[0]
    band = self.plan_clearance_tie_band
    tier = [c for c in ranked if c.clearance >= top.clearance - band]
    tier.sort(key=lambda c: (-c.clearance, -c.score))
    return tier[0]

  def _candidate_angles(self) -> List[float]:
    angles: List[float] = []
    for deg in self.plan_angles:
      if abs(deg) < 0.01:
        if 0.0 not in angles:
          angles.append(0.0)
        continue
      for sign in (1.0, -1.0):
        a = sign * deg
        if a not in angles:
          angles.append(a)
    return angles

  def _update_stress_smooth(self, stress_level: float) -> None:
    now = rospy.Time.now()
    dt = max(0.0, (now - self._last_stress_update).to_sec())
    self._last_stress_update = now
    target = stress_level / 5.0
    if self.stress_decay_tau <= 0.0:
      self._stress_smoothed = target
      return
    alpha = 1.0 - math.exp(-dt / self.stress_decay_tau)
    if target > self._stress_smoothed:
      self._stress_smoothed = target
    else:
      self._stress_smoothed += alpha * (target - self._stress_smoothed)

  def _dist_for_bearing(self, center: float, wide: float) -> float:
    finite = [v for v in (center, wide) if v < float('inf')]
    min_clear = min(finite) if finite else self.plan_dist_max
    return max(
        self.plan_dist_min,
        min(self.plan_dist_max, min_clear * 2.0),
    )

  def _lateral_open_bias(self, profile: dict, angle_deg: float) -> float:
    """Continuous bonus toward the more open side — no mode switch."""
    if abs(angle_deg) < 0.5:
      return 0.0
    left = float(profile.get('left', self.plan_dist_max))
    right = float(profile.get('right', self.plan_dist_max))
    denom = max(left, right, 0.25)
    asym = (left - right) / denom
    sign = 1.0 if angle_deg > 0 else -1.0
    return max(-0.12, min(0.12, 0.12 * asym * sign))

  def _final_score(
      self,
      angle_deg: float,
      center: float,
      wide: float,
      clearance: float,
      best_clearance: float,
      dist: float,
      explore: float,
      explore_here: float,
      curvature: float,
      coll_norm: float,
      exec_cost: float,
      lateral_bias: float,
  ) -> Tuple[float, ScoreBreakdown]:
    clear_norm = min(1.0, clearance / 2.0)
    rel_clear = clearance / max(best_clearance, 0.01)
    align = 1.0 - min(1.0, abs(angle_deg) / 70.0)
    unknown_gain = max(0.0, explore - explore_here)
    progress_dist = min(dist, self.plan_progress_dist_cap)
    progress = min(1.0, progress_dist / self.plan_progress_dist_cap)
    align_scale = min(1.0, rel_clear / 0.90)

    bd = ScoreBreakdown(
        clearance=self.score_clearance * clear_norm,
        alignment=self.score_alignment * align * align_scale,
        unknown=self.score_explore * unknown_gain,
        progress=self.score_progress * progress,
        lateral=lateral_bias,
        curvature=-self.score_curvature * curvature,
        collision=-self.score_collision * coll_norm,
        stress=-self.score_stress * self._stress_smoothed,
        execution=-self.score_execution * exec_cost,
    )
    bd.total = (
        bd.clearance + bd.alignment + bd.unknown + bd.progress + bd.lateral
        + bd.curvature + bd.collision + bd.stress + bd.execution
    )
    return bd.total, bd

  def _bearing_clearances(
      self,
      scan: LaserScan,
      angle_rad: float,
  ) -> Tuple[float, float]:
    center = sector_min(
        scan, self.front_angle + angle_rad, self.nav_half_width,
    )
    wide = sector_min(
        scan, self.front_angle + angle_rad, self.wide_half_width,
    )
    if center == float('inf'):
      center = self.plan_dist_max
    if wide == float('inf'):
      wide = self.plan_dist_max
    return center, wide

  def _collision_at(self, wx: float, wy: float) -> float:
    gx, gy = self.grid._to_grid(wx, wy)
    if not self.grid.in_bounds(gx, gy):
      return 0.0
    return float(self.grid.collision_cost[gy, gx])

  def _exploration_gain(self, wx: float, wy: float) -> float:
    gx, gy = self.grid._to_grid(wx, wy)
    if not self.grid.in_bounds(gx, gy):
      return 0.55
    state = int(self.grid.grid[gy, gx])
    if state == CellState.UNKNOWN:
      return 1.0
    if state == CellState.FREE:
      return 0.85
    if state == CellState.VISITED:
      return 0.35
    return 0.0

  def _trim_forward(
      self,
      path: List[Tuple[float, float]],
      rx: float,
      ry: float,
      ryaw: float,
  ) -> List[Tuple[float, float]]:
    trimmed = list(path)
    limit = math.radians(95.0)
    while len(trimmed) > 1:
      wx, wy = trimmed[0]
      bearing = math.atan2(wy - ry, wx - rx)
      err = bearing - ryaw
      while err > math.pi:
        err -= 2.0 * math.pi
      while err < -math.pi:
        err += 2.0 * math.pi
      if abs(err) <= limit:
        break
      trimmed.pop(0)
    return trimmed if trimmed else path[:1]

  def goal_valid(self, gx: float, gy: float, scan: LaserScan) -> bool:
    rx, ry, ryaw = self.pose.get_pose()
    bearing = math.atan2(gy - ry, gx - rx)
    angle_rad = bearing - ryaw
    while angle_rad > math.pi:
      angle_rad -= 2.0 * math.pi
    while angle_rad < -math.pi:
      angle_rad += 2.0 * math.pi
    center, _wide = self._bearing_clearances(scan, angle_rad)
    if center < self.boundary_dist + 0.02:
      return False
    if self._collision_at(gx, gy) >= 4.0:
      return False
    return True

  def _evaluate(
      self,
      scan: LaserScan,
      rx: float,
      ry: float,
      ryaw: float,
      angle_deg: float,
      exec_cost: float,
      profile: dict,
      explore_here: float,
      best_clearance: float,
  ) -> _Candidate:
    angle_rad = math.radians(angle_deg)
    heading = ryaw + angle_rad

    center, wide = self._bearing_clearances(scan, angle_rad)
    clearance = min(center, wide)
    dist = self._dist_for_bearing(center, wide)
    gx = rx + dist * math.cos(heading)
    gy = ry + dist * math.sin(heading)
    limit = self.boundary_dist + 0.02

    if center < limit:
      return _Candidate(
          angle_deg, 0.0, gx, gy, [], dist, clearance, 0.0, False, ScoreBreakdown(),
      )

    coll = self._collision_at(gx, gy)
    if coll >= 4.0:
      return _Candidate(
          angle_deg, 0.0, gx, gy, [], dist, clearance, 0.0, False, ScoreBreakdown(),
      )

    path = self.grid.plan_path(rx, ry, gx, gy)
    if not path:
      path = [(gx, gy)]

    explore = self._exploration_gain(gx, gy)
    curvature = (abs(angle_deg) / 70.0) ** 2
    coll_norm = min(1.0, coll / max(1.0, self.collision_cost_max))
    lateral_bias = self._lateral_open_bias(profile, angle_deg)

    score, breakdown = self._final_score(
        angle_deg,
        center,
        wide,
        clearance,
        best_clearance,
        dist,
        explore,
        explore_here,
        curvature,
        coll_norm,
        exec_cost,
        lateral_bias,
    )
    penalty = self._recent_boundary_penalty(angle_deg)
    fm_penalty = self._failure_memory_penalty(rx, ry, angle_deg)
    wall_penalty = self._wall_hug_penalty(angle_deg, center)
    dist_penalty = self._distance_penalty(dist)
    extra_penalty = penalty + fm_penalty + wall_penalty + dist_penalty
    if extra_penalty > 0.0:
      score -= extra_penalty
      breakdown.total = score

    path = self._trim_forward(path, rx, ry, ryaw)
    return _Candidate(
        angle_deg,
        score,
        gx,
        gy,
        path,
        dist,
        clearance,
        explore,
        True,
        breakdown,
    )

  def _angle_tag(self, angle_deg: float) -> str:
    if abs(angle_deg) < 0.5:
      return '0'
    return '{:+d}'.format(int(round(angle_deg)))

  def _log_candidates(self, ranked: List[_Candidate], selected: Optional[_Candidate]) -> None:
    if not self.planner_debug:
      return
    lines = [
        'LocalPlanner score breakdown (Clr=clearance Ali=alignment Unk=unknown):',
    ]
    for cand in ranked:
      tag = self._angle_tag(cand.angle_deg)
      if not cand.valid:
        lines.append('  {}° INVALID clear={:.2f}'.format(tag, cand.clearance))
        continue
      b = cand.breakdown
      penalty = self._recent_boundary_penalty(cand.angle_deg)
      rx, ry, _ = self.pose.get_pose()
      fm_penalty = self._failure_memory_penalty(rx, ry, cand.angle_deg)
      wall_penalty = self._wall_hug_penalty(cand.angle_deg, cand.clearance)
      dist_penalty = self._distance_penalty(cand.dist)
      extra = penalty + fm_penalty + wall_penalty + dist_penalty
      pen_tag = ''
      if extra > 0.0:
        parts = []
        if penalty > 0.0:
          parts.append('pen={:.2f}'.format(penalty))
        if fm_penalty > 0.0:
          parts.append('fm={:.2f}'.format(fm_penalty))
        if wall_penalty > 0.0:
          parts.append('wall={:.2f}'.format(wall_penalty))
        if dist_penalty > 0.0:
          parts.append('dist={:.2f}'.format(dist_penalty))
        pen_tag = ' ' + ' '.join(parts)
      lines.append(
          '  {}° TOTAL={:+.3f}  Clr={:+.3f} Ali={:+.3f} Unk={:+.3f} Pro={:+.3f} '
          'Lat={:+.3f} Cur={:+.3f} Col={:+.3f} Str={:+.3f} Exec={:+.3f}{}  '
          'd={:.2f} unk={:.2f}'.format(
              tag, b.total, b.clearance, b.alignment, b.unknown, b.progress,
              b.lateral, b.curvature, b.collision, b.stress, b.execution,
              pen_tag, cand.dist, cand.unknown,
          )
      )
    if selected is not None:
      tag = self._angle_tag(selected.angle_deg)
      lines.append(
          'Selected: {}° goal=({:.2f},{:.2f}) d={:.2f} TOTAL={:+.3f}'.format(
              tag, selected.gx, selected.gy, selected.dist, selected.breakdown.total,
          )
      )
    else:
      lines.append('Selected: none (all rejected)')
    rospy.loginfo('\n'.join(lines))

  def replan_local(self, scan: LaserScan, force: bool = False) -> bool:
    if self.nav.corridor_commit_active():
      return self.nav.has_goal()
    now = rospy.Time.now()
    if self._penalty_pending_replan and self._last_boundary_angle_deg is not None:
      rospy.loginfo(
          'LocalPlanner: 首次重规划应用方向惩罚 偏角=%.0f°',
          self._last_boundary_angle_deg,
      )
      self._penalty_pending_replan = False
    fb = self.nav.execution_feedback()
    exec_cost = fb.execution_cost if self.nav.has_goal() else 0.0
    self._update_stress_smooth(fb.metrics.get('stress_level', 0.0))

    profile = clearance_profile(
        scan,
        self.front_angle,
        self.nav_half_width,
        self.wide_half_width,
        self.boundary_dist,
    )

    rx, ry, _ryaw = self.pose.get_pose()
    explore_here = self._exploration_gain(rx, ry)
    angles = self._candidate_angles()

    preclear: List[Tuple[float, float]] = []
    for angle_deg in angles:
      center, wide = self._bearing_clearances(scan, math.radians(angle_deg))
      preclear.append((angle_deg, min(center, wide)))
    best_clearance = max((c for _a, c in preclear if c < float('inf')), default=self.plan_dist_max)

    rx, ry, ryaw = self.pose.get_pose()
    evaluated = [
        self._evaluate(
            scan, rx, ry, ryaw, angle_deg, exec_cost, profile, explore_here, best_clearance,
        )
        for angle_deg in angles
    ]
    valid = [c for c in evaluated if c.valid]
    valid.sort(key=lambda c: c.score, reverse=True)
    valid = self._filter_post_recovery_candidates(valid, float(profile.get('center', 99.0)))

    best = self._pick_best_candidate(valid)
    self._log_candidates(evaluated, best)

    if self._post_recovery_rounds_left > 0:
      self._post_recovery_rounds_left -= 1
    if self._passage_replan_rounds_left > 0:
      self._passage_replan_rounds_left -= 1

    if best is None:
      self._last_replan = now
      return False
    self._last_selected_angle_deg = best.angle_deg
    self.nav.set_path(best.path, (best.gx, best.gy))
    self._last_replan = now
    return True

  def tick(self, scan: LaserScan, move, cruise_speed: float) -> str:
    """跟目标导航；重规划仅由 FSM 在状态切换时触发（P0：不自动 force replan）。"""
    if self.nav.has_goal():
      status = self.nav.tick(scan, cruise_speed)
      fb = self.nav.execution_feedback()
      self._update_stress_smooth(fb.metrics.get('stress_level', 0.0))

      if self.planner_debug and self._tick_log_due():
        rospy.loginfo(fb.to_log_line())

      if fb.goal_status == GoalStatus.REACHED:
        return 'reached'

      if fb.goal_status == GoalStatus.FAILED:
        return 'blocked'

      if fb.goal_status == GoalStatus.ABORTED:
        return 'odom_fault'

      return status

    if not self.replan_local(scan, force=True):
      return 'no_candidate'
    return self.nav.tick(scan, cruise_speed)

  def _tick_log_due(self) -> bool:
    self._debug_tick += 1
    return self._debug_tick % 40 == 0
