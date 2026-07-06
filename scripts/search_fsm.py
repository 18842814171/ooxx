"""LiDAR-driven coverage search — map-aware FSM per 3.md."""

from __future__ import annotations

import enum
import math
import os
from typing import Callable, Dict, List, Optional, Tuple

import rospy
from sensor_msgs.msg import CameraInfo, Image, LaserScan

from boustrophedon_planner import BoustrophedonConfig, BoustrophedonPlanner, StripPhase
from config_loader import MissionConfig
from frontier_navigator import FrontierNavigator
from grid_planner import GridPlanner
from lidar_utils import (
    classify_boundary_type,
    clearance_profile,
    forward_path_clear,
    front_sector_clear,
    is_boundary,
    open_side,
)
from global_planner import GlobalPlanner
from local_planner import LocalPlanner
from mission_state import MissionState
from move_controller import MoveController
from occupancy_grid import OccupancyGrid
from perception.base import PerceptionBackend, ScanResult
from pose_estimator import PoseEstimator


FSM_BUILD_ID = '20260706-corridor-commit-lifecycle'


class SearchState(enum.Enum):
  GLOBAL_PLAN = 'GLOBAL_PLAN'
  EXPLORE = 'EXPLORE'
  STOP_SCAN = 'STOP_SCAN'
  UPDATE_MAP = 'UPDATE_MAP'
  PLAN_NEXT = 'PLAN_NEXT'
  MOVE_TO_GOAL = 'MOVE_TO_GOAL'
  BOUNDARY_MANEUVER = 'BOUNDARY_MANEUVER'
  WALL_FOLLOW = 'WALL_FOLLOW'
  RECOVERY = 'RECOVERY'
  RETURN_HOME = 'RETURN_HOME'
  DONE = 'DONE'


class RecoveryPhase(enum.Enum):
  PAUSE = 'pause'
  RECHECK = 'recheck'
  ACT = 'act'


class SearchFSM:
  def __init__(
      self,
      config: MissionConfig,
      move: MoveController,
      perception: PerceptionBackend,
      mission_state: MissionState,
      pose: PoseEstimator,
      get_scan: Callable[[], Optional[LaserScan]],
      get_image: Callable[[], Optional[object]],
      get_depth: Callable[[], Optional[object]] = lambda: None,
      get_camera_info: Callable[[], Optional[CameraInfo]] = lambda: None,
  ):
    self.config = config
    self.move = move
    self.perception = perception
    self.mission_state = mission_state
    self.pose = pose
    self.get_scan = get_scan
    self.get_image = get_image
    self.get_depth = get_depth
    self.get_camera_info = get_camera_info

    self.last_scan_result: Optional[ScanResult] = None
    self._last_twist_time = rospy.Time.now()
    self._start_pose: Optional[Tuple[float, float, float]] = None
    self._boundary_streak = 0
    self._explore_started_at = rospy.Time.now()
    self._explore_origin: Optional[Tuple[float, float]] = None
    self._explore_no_candidate_count = 0
    self._blocked_goals: List[Tuple[float, float, float]] = []
    self._max_blocked_goals = 16
    self._blocked_region_radius = 0.75
    self._blocked_region_radius_max = 1.2
    self._corner_blacklist_radius = 0.55
    self._consecutive_blocked = 0
    self._blocked_escalate_threshold = 3
    self._plan_settle_sec = 0.1
    self._prefer_escape_frontier = False
    self._last_recovery_turn_sign: Optional[float] = None
    self._recovery_escalation = 0
    self._recovery_attempts = 0
    self._max_recovery_attempts = 5
    self._recovery_rotate_step_deg = 30.0
    self._recovery_explore_active = False
    self._recovery_goal: Optional[Tuple[float, float]] = None
    self._recovery_reason: Optional[str] = None
    self._goal_failures: Dict[Tuple[float, float], Dict[str, int]] = {}
    self._goal_failure_blacklist_threshold = 2
    self._stall_start: Optional[rospy.Time] = None
    self._stall_x = 0.0
    self._stall_y = 0.0
    self._stall_last_yaw = 0.0
    self._stall_rotation_accum = 0.0
    self._stall_goal_dist = 0.0
    self._last_handler_state: Optional[SearchState] = None
    self._recovery_phase: Optional[RecoveryPhase] = None
    self._recovery_phase_started: Optional[rospy.Time] = None
    self._boundary_kind: Optional[str] = None
    self._boundary_hit_poses: List[Tuple[float, float]] = []
    self._boundary_escalation = 0
    self._boundary_max_level_retries = 0
    self._recovery_entry_center = float('inf')
    self._recovery_entry_x = 0.0
    self._recovery_entry_y = 0.0
    self._recovery_before_center = float('inf')
    self._recovery_before_x = 0.0
    self._recovery_before_y = 0.0
    self._nav_hold_streak = 0
    self._run_stats: Dict[str, float] = {
        'boundary': 0.0,
        'recovery': 0.0,
        'corner_stall': 0.0,
        'blocked_goal': 0.0,
        'speed_sum': 0.0,
        'speed_n': 0.0,
    }
    self._stats_interval_started = rospy.Time.now()
    self._bootstrap_logged = False
    self._bootstrap_started_at = rospy.Time.now()
    self._bootstrap_start_coverage = 0.0
    self._bootstrap_recovery_streak = 0
    self._bootstrap_exited = False
    self._resume_passage_mode = False
    self._recovery_perimeter_resume = False

    map_cfg = config.map
    self.grid = OccupancyGrid(
        resolution=map_cfg.resolution,
        size_m=map_cfg.size_m,
    )
    self.grid.attach_planner(
        GridPlanner(
            unknown_cost=map_cfg.astar_unknown_cost,
            inflation_cells=map_cfg.astar_inflation_cells,
            waypoint_spacing_m=map_cfg.astar_waypoint_spacing,
        )
    )
    gp_cfg = config.global_planner
    self.frontier_nav = FrontierNavigator(
        move=move,
        pose=pose,
        boundary_dist=config.search.boundary_dist,
        front_angle=config.search.front_angle,
        front_sector_half_width=config.search.front_sector_half_width,
        nav_block_half_width=config.search.nav_block_half_width,
        goal_tolerance=map_cfg.goal_tolerance,
        waypoint_tolerance=config.search.waypoint_tolerance,
        align_tolerance_deg=config.search.align_tolerance_deg,
        heading_kp=config.search.nav_heading_kp,
        boundary_confirm_frames=config.search.boundary_confirm_frames,
        creep_speed=config.search.creep_speed,
        align_timeout_sec=config.search.align_timeout_sec,
        align_arc_speed=config.search.align_arc_speed,
        forward_stop_margin=config.search.nav_forward_stop_margin,
        cruise_clear_margin=config.search.nav_cruise_clear_margin,
        wide_hold_margin=config.search.nav_wide_hold_margin,
        lateral_block_dist=config.search.nav_lateral_block_dist,
        robot_half_width=config.search.robot_half_width,
        robot_front_overhang=config.search.robot_front_overhang,
        passage_score_threshold=config.search.nav_passage_score_threshold,
        passage_max_ticks=config.search.nav_passage_max_ticks,
        passage_creep_factor=config.search.nav_passage_creep_factor,
        passage_auto_arm=config.search.nav_passage_auto_arm,
        passage_channel_auto_arm=config.search.nav_passage_channel_auto_arm,
        passage_wide_max=config.search.nav_passage_wide_max,
        open_align_center_min=config.search.nav_open_align_center_min,
        open_align_arc_speed=config.search.nav_open_align_arc_speed,
        wide_hold_override_center_min=config.search.nav_wide_hold_override_center_min,
        passage_side_max=config.search.nav_passage_side_max,
        passage_center_min_ratio=config.search.nav_passage_center_min_ratio,
        passage_channel_close_max=config.search.nav_passage_channel_close_max,
        passage_channel_open_min=config.search.nav_passage_channel_open_min,
        passage_channel_center_min_ratio=config.search.nav_passage_channel_center_min_ratio,
        passage_channel_heading_max_deg=config.search.nav_passage_channel_heading_max_deg,
        passage_channel_min_hold_ticks=config.search.nav_passage_channel_min_hold_ticks,
        passage_channel_lost_streak=config.search.nav_passage_channel_lost_streak,
        passage_channel_exit_yaw_hold_sec=config.search.nav_passage_channel_exit_yaw_hold_sec,
        corridor_commit_travel_m=config.search.nav_corridor_commit_travel_m,
        corridor_commit_cooldown_sec=config.search.nav_corridor_commit_cooldown_sec,
        corridor_commit_arm_confirm_frames=config.search.nav_corridor_commit_arm_confirm_frames,
        corridor_commit_min_hold_ticks=config.search.nav_corridor_commit_min_hold_ticks,
        corridor_commit_min_travel_m=config.search.nav_corridor_commit_min_travel_m,
        corridor_commit_abort_yaw_deg=config.search.nav_corridor_commit_abort_yaw_deg,
        corridor_commit_hard_abort_ratio=config.search.nav_corridor_commit_hard_abort_ratio,
        drive_heading_far_dist=config.search.nav_drive_heading_far_dist,
        drive_heading_far_deg=config.search.nav_drive_heading_far_deg,
        drive_heading_mid_dist=config.search.nav_drive_heading_mid_dist,
        drive_heading_mid_deg=config.search.nav_drive_heading_mid_deg,
        wall_target_dist=config.search.wall_target_dist,
        perimeter_wall_side=gp_cfg.perimeter_wall_side,
        perimeter_lost_wall_dist=gp_cfg.perimeter_lost_wall_dist,
        perimeter_found_wall_dist=gp_cfg.perimeter_found_wall_dist,
        perimeter_corner_stuck_ticks=gp_cfg.perimeter_corner_stuck_ticks,
    )
    self.global_planner = GlobalPlanner(
        grid=self.grid,
        initial_mode=gp_cfg.initial_mode,
        perimeter_enabled=gp_cfg.perimeter_enabled,
        perimeter_min_visited=gp_cfg.perimeter_min_visited,
        perimeter_yaw_deg=gp_cfg.perimeter_yaw_deg,
        perimeter_max_sec=gp_cfg.perimeter_max_sec,
        perimeter_min_coverage=gp_cfg.perimeter_min_coverage,
        astar_max_candidates=map_cfg.astar_max_candidates,
    )
    self.local_planner = LocalPlanner(
        grid=self.grid,
        nav=self.frontier_nav,
        pose=pose,
        boundary_dist=config.search.boundary_dist,
        front_angle=config.search.front_angle,
        nav_half_width=config.search.nav_block_half_width,
        wide_half_width=config.search.front_sector_half_width,
        plan_dist_min=config.search.local_plan_dist_min,
        plan_dist_max=config.search.local_plan_dist_max,
        plan_progress_dist_cap=config.search.local_plan_progress_dist_cap,
        plan_clearance_tie_band=config.search.local_plan_clearance_tie_band,
        plan_angles=config.search.local_plan_angles,
        stress_decay_tau=config.search.stress_decay_tau,
        cruise_speed=config.search.cruise_speed,
        creep_speed=config.search.creep_speed,
        collision_cost_max=config.search.collision_cost_max,
        score_clearance=config.search.local_plan_score_clearance,
        score_alignment=config.search.local_plan_score_alignment,
        score_explore=config.search.local_plan_score_explore,
        score_progress=config.search.local_plan_score_progress,
        score_curvature=config.search.local_plan_score_curvature,
        score_collision=config.search.local_plan_score_collision,
        score_stress=config.search.local_plan_score_stress,
        score_execution=config.search.local_plan_score_execution,
        recent_collision_penalty=config.search.recent_collision_penalty,
        recent_collision_penalty_deg=config.search.recent_collision_penalty_deg,
        recent_collision_penalty_sec=config.search.recent_collision_penalty_sec,
        failure_memory_penalty=config.search.failure_memory_penalty,
        failure_memory_angle_deg=config.search.failure_memory_angle_deg,
        failure_memory_radius_m=config.search.failure_memory_radius_m,
        failure_memory_sec=config.search.failure_memory_sec,
        failure_memory_max_entries=config.search.failure_memory_max_entries,
        wall_hug_zero_penalty=config.search.wall_hug_zero_penalty,
        wall_hug_clearance_margin=config.search.wall_hug_clearance_margin,
        wall_hug_angle_deg=config.search.wall_hug_angle_deg,
        failure_memory_cluster_count=config.search.failure_memory_cluster_count,
        failure_memory_cluster_radius_m=config.search.failure_memory_cluster_radius_m,
        failure_memory_cluster_penalty=config.search.failure_memory_cluster_penalty,
        recovery_escape_angle_deg=config.search.recovery_escape_angle_deg,
        recovery_escape_center_max=config.search.recovery_escape_center_max,
        recovery_replan_rounds=config.search.recovery_replan_rounds,
        planner_debug=config.search.planner_debug,
    )
    b_cfg = config.boustrophedon
    self.boustrophedon = BoustrophedonPlanner(
        move=move,
        config=BoustrophedonConfig(
            strip_width=b_cfg.strip_width,
            cruise_speed=config.search.cruise_speed,
            turn_speed=config.search.turn_speed,
            max_lanes=b_cfg.max_lanes,
        ),
    )

    self.state = self._initial_state(config)
    self._tick_count = 0
    rospy.loginfo(
        'FSM init: build=%s mode=%s initial_state=%s global_planner=%s',
        FSM_BUILD_ID,
        config.search.mode,
        self.state.value,
        self._use_global_planner(),
    )

  def _use_global_planner(self) -> bool:
    return (
        self.config.global_planner.enabled
        and self.mode == 'occupancy_grid'
    )

  def _initial_state(self, config: MissionConfig) -> SearchState:
    if config.search.mode == 'occupancy_grid':
      gp = config.global_planner
      if (
          gp.enabled
          and gp.perimeter_enabled
          and gp.initial_mode == 'perimeter'
      ):
        return SearchState.GLOBAL_PLAN
      return SearchState.PLAN_NEXT
    return SearchState.PLAN_NEXT

  def _goto_global_plan(self, reason: str) -> None:
    gp = self.config.global_planner
    if not gp.perimeter_enabled or gp.initial_mode != 'perimeter':
      self.frontier_nav.stop_perimeter_mode()
      self._set_state(SearchState.PLAN_NEXT, reason)
      return
    if self.global_planner.current_mode() != 'perimeter':
      self.frontier_nav.stop_perimeter_mode()
    self._set_state(SearchState.GLOBAL_PLAN, reason)

  def _apply_global_plan(self, plan, scan: LaserScan) -> bool:
    """Apply a GlobalPlan result. Return True if state was updated."""
    if plan.status == 'no_goal':
      rospy.logwarn('GlobalPlanner: no goal — RECOVERY')
      self._set_state(SearchState.RECOVERY, 'global_planner: no frontier')
      return True

    if plan.mode == 'perimeter':
      if not self.frontier_nav.perimeter_active():
        self.frontier_nav.start_perimeter_mode()
      self._set_state(SearchState.MOVE_TO_GOAL, 'global: perimeter')
      return True

    if plan.goal and plan.waypoints:
      self.frontier_nav.stop_perimeter_mode()
      if not self.frontier_nav.set_path(plan.waypoints, plan.goal):
        rospy.logwarn('GlobalPlanner: set_path rejected — PLAN_NEXT')
        self._set_state(SearchState.PLAN_NEXT, 'global: set_path rejected')
        return True
      wp_idx, wp_total = self.frontier_nav.path_progress()
      rospy.loginfo(
          'FSM GLOBAL_PLAN -> MOVE goal=(%.2f,%.2f) path=%d wp=%d/%d mode=%s',
          plan.goal[0],
          plan.goal[1],
          len(plan.waypoints),
          wp_idx,
          wp_total,
          plan.mode,
      )
      self._set_state(SearchState.MOVE_TO_GOAL, 'global: {}'.format(plan.mode))
      return True

    rospy.logwarn('GlobalPlanner: empty plan mode=%s', plan.mode)
    self._set_state(SearchState.RECOVERY, 'global_planner: empty plan')
    return True

  def _set_state(self, new_state: SearchState, reason: str) -> None:
    if new_state != self.state:
      rospy.loginfo(
          'FSM %s -> %s | %s',
          self.state.value,
          new_state.value,
          reason,
      )
    self.state = new_state

  @property
  def search(self):
    return self.config.search

  @property
  def mode(self) -> str:
    return self.search.mode

  def tick(self) -> SearchState:
    self._tick_count += 1
    state_in = self.state

    if self.state == SearchState.DONE:
      self.move.stop_robot(repeats=1)
      return self.state

    scan = self.get_scan()
    if scan is None:
      self.move.publish_stop_once()
      return self.state

    self._update_pose_from_motion()
    rx, ry, ryaw = self.pose.get_pose()
    self._ensure_start_pose()
    self.grid.update_scan(scan, rx, ry, ryaw, self.search.front_angle)
    gx, gy = self.grid.mark_robot(rx, ry)
    self.mission_state.mark_visited(gx, gy)
    visited, free, _ = self.grid.count_states()
    self.mission_state.update_coverage(visited, free)
    self._update_run_stats()

    handlers = {
        SearchState.GLOBAL_PLAN: lambda: self._tick_global_plan(scan),
        SearchState.EXPLORE: lambda: self._tick_explore(scan),
        SearchState.STOP_SCAN: self._tick_stop_scan,
        SearchState.UPDATE_MAP: self._tick_update_map,
        SearchState.PLAN_NEXT: self._tick_plan_next,
        SearchState.MOVE_TO_GOAL: lambda: self._tick_move_to_goal(scan),
        SearchState.BOUNDARY_MANEUVER: lambda: self._tick_boundary_maneuver(scan),
        SearchState.WALL_FOLLOW: lambda: self._tick_wall_follow(scan),
        SearchState.RECOVERY: lambda: self._tick_recovery(scan),
        SearchState.RETURN_HOME: lambda: self._tick_return_home(scan),
    }
    if (
        self.mode == 'occupancy_grid'
        and self.search.bootstrap_local_plan
        and not self._bootstrap_exited
        and not self._use_global_planner()
        and self.state in (SearchState.EXPLORE, SearchState.MOVE_TO_GOAL)
        and self._recovery_phase is None
        and not self._recovery_explore_active
    ):
      if self._maybe_exit_bootstrap():
        return self.state
    handlers[self.state]()

    if state_in in (SearchState.PLAN_NEXT, SearchState.MOVE_TO_GOAL, SearchState.EXPLORE, SearchState.GLOBAL_PLAN):
      if self.state == state_in and self._tick_count <= 5:
        rospy.loginfo('FSM tick #%d remain in %s', self._tick_count, self.state.value)

    if self.state not in (SearchState.DONE, SearchState.RETURN_HOME) and self._is_search_complete():
      if self._begin_return_home():
        pass
      else:
        self.state = SearchState.DONE
        rospy.loginfo('Search finished: %s', self.mission_state.status_line())

    self._maybe_log_run_stats(visited, free)
    return self.state

  def _update_pose_from_motion(self) -> None:
    now = rospy.Time.now()
    dt = (now - self._last_twist_time).to_sec()
    self._last_twist_time = now
    if dt <= 0 or dt > 0.5:
      return
    # Use actual published cmd_vel (incl. angular_z from FrontierNavigator),
    # not a hardcoded cruise_speed — otherwise yaw never integrates without /odom.
    self.pose.update_from_twist(self.move.get_last_twist(), dt)

  def _ensure_start_pose(self) -> None:
    if self._start_pose is not None:
      return
    self._start_pose = self.pose.get_pose()
    sx, sy, syaw = self._start_pose
    if self.mode == 'occupancy_grid':
      self.grid.set_origin(sx, sy, reset=True)
      rospy.loginfo(
          'Occupancy grid centered on start (%.2f, %.2f), size=%.1f m',
          sx, sy, self.grid.size_m,
      )
    rospy.loginfo(
        'Start pose recorded: (%.2f, %.2f, yaw=%.1f deg)',
        sx, sy, math.degrees(syaw),
    )

  def _normalize_angle(self, angle: float) -> float:
    while angle > math.pi:
      angle -= 2.0 * math.pi
    while angle < -math.pi:
      angle += 2.0 * math.pi
    return angle

  def _is_at_home_position(self) -> bool:
    if self._start_pose is None:
      return True
    sx, sy, _ = self._start_pose
    return self.pose.distance_to(sx, sy) <= self.config.map.goal_tolerance

  def _is_at_home_yaw(self) -> bool:
    if self._start_pose is None or not self.config.map.align_yaw_on_return:
      return True
    _, _, syaw = self._start_pose
    _, _, ryaw = self.pose.get_pose()
    return abs(self._normalize_angle(syaw - ryaw)) <= math.radians(12.0)

  def _begin_return_home(self) -> bool:
    """Enter RETURN_HOME if enabled; return True if navigating home."""
    if not self.config.map.return_home:
      return False
    self._ensure_start_pose()
    if self._start_pose is None:
      return False
    if self._is_at_home_position() and self._is_at_home_yaw():
      return False

    sx, sy, _ = self._start_pose
    self.frontier_nav.clear_goal()
    self._set_nav_target(sx, sy)
    self.state = SearchState.RETURN_HOME
    rospy.loginfo('Search phase complete — returning to start (%.2f, %.2f)', sx, sy)
    return True

  def _is_search_complete(self) -> bool:
    if self.mission_state.is_mission_complete():
      return True
    if self.mode == 'occupancy_grid' and self.mission_state.is_coverage_complete(
        self.config.map.coverage_complete_threshold
    ):
      return True
    if self.boustrophedon.is_done():
      return True
    if self.mission_state.boundary_events >= self.search.max_boundary_events:
      rospy.logwarn('Coverage budget reached.')
      return True
    return False

  def _is_boundary_confirmed(self, scan: LaserScan) -> bool:
    if is_boundary(
        scan,
        self.search.boundary_dist,
        self.search.front_angle,
        self.search.front_sector_half_width,
    ):
      self._boundary_streak += 1
    else:
      self._boundary_streak = 0
    return self._boundary_streak >= self.search.boundary_confirm_frames

  def _check_explore_timeout(self) -> bool:
    stall_sec = self.search.explore_stall_sec
    if stall_sec <= 0:
      return False
    elapsed = (rospy.Time.now() - self._explore_started_at).to_sec()
    if self._explore_origin is None:
      return elapsed >= stall_sec
    rx, ry, _ = self.pose.get_pose()
    ox, oy = self._explore_origin
    if math.hypot(rx - ox, ry - oy) >= 0.08:
      self._reset_explore_timer()
      return False
    return elapsed >= stall_sec

  def _reset_explore_timer(self) -> None:
    self._explore_started_at = rospy.Time.now()
    rx, ry, _ = self.pose.get_pose()
    self._explore_origin = (rx, ry)
    self._explore_no_candidate_count = 0

  def _enter_planner_deadlock_recovery(self, scan: LaserScan) -> None:
    """连续无有效候选：后退转向，恢复后交由规划器重选。"""
    threshold = self.search.explore_no_candidate_frames
    rospy.logwarn(
        '探索：连续 %d 帧无有效候选，进入恢复',
        threshold,
    )
    self._explore_no_candidate_count = 0
    self.local_planner.clear()
    kind = classify_boundary_type(
        scan,
        self.search.front_angle,
        self.search.nav_block_half_width,
        self.search.front_sector_half_width,
        self.search.boundary_dist,
    )
    self._start_boundary_recovery(scan, kind)
    self._recovery_reason = 'explore_deadlock'

  def _update_run_stats(self) -> None:
    twist = self.move.get_last_twist()
    self._run_stats['speed_sum'] += abs(twist.linear.x)
    self._run_stats['speed_n'] += 1.0

  def _maybe_log_run_stats(self, visited: int, free: int) -> None:
    interval = self.search.stats_interval_sec
    if interval <= 0:
      return
    elapsed = (rospy.Time.now() - self._stats_interval_started).to_sec()
    if elapsed < interval:
      return
    avg_speed = self._run_stats['speed_sum'] / max(1.0, self._run_stats['speed_n'])
    coverage = float(visited) / max(1.0, float(free))
    rospy.loginfo(
        '运行统计(%ds): 边界=%d 恢复=%d 角落停滞=%d 目标拉黑=%d 均速=%.2fm/s 覆盖率=%.0f%%',
        int(interval),
        int(self._run_stats['boundary']),
        int(self._run_stats['recovery']),
        int(self._run_stats['corner_stall']),
        int(self._run_stats['blocked_goal']),
        avg_speed,
        coverage * 100.0,
    )
    self._run_stats = {
        'boundary': 0.0,
        'recovery': 0.0,
        'corner_stall': 0.0,
        'blocked_goal': 0.0,
        'speed_sum': 0.0,
        'speed_n': 0.0,
    }
    self._stats_interval_started = rospy.Time.now()

  def _register_spatial_boundary(self, rx: float, ry: float) -> bool:
    radius = self.search.boundary_spatial_radius
    self._boundary_hit_poses.append((rx, ry))
    if len(self._boundary_hit_poses) > 24:
      self._boundary_hit_poses = self._boundary_hit_poses[-24:]
    count = sum(
        1 for hx, hy in self._boundary_hit_poses
        if math.hypot(rx - hx, ry - hy) < radius
    )
    if count >= self.search.boundary_spatial_count:
      self.grid.record_collision(
          rx, ry,
          radius_m=radius,
          increment=self.search.collision_cost_increment,
          max_cost=self.search.collision_cost_max,
      )
      self._add_blocked_region(rx, ry, radius=radius)
      self._boundary_hit_poses.clear()
      return True
    return False

  def _snapshot_recovery_entry(self, scan: LaserScan) -> None:
    """本轮 Recovery 入口基线；gain 相对此值，非每次 ACT 前。"""
    profile = clearance_profile(
        scan,
        self.search.front_angle,
        self.search.nav_block_half_width,
        self.search.front_sector_half_width,
        self.search.boundary_dist,
    )
    rx, ry, _ = self.pose.get_pose()
    self._recovery_entry_center = float(profile.get('center', 0.0))
    self._recovery_entry_x = rx
    self._recovery_entry_y = ry
    self._recovery_before_x = rx
    self._recovery_before_y = ry
    rospy.loginfo(
        'Recovery(%s): 入口基线 center=%.2f',
        self._recovery_reason or '?',
        self._recovery_entry_center,
    )

  def _snapshot_recovery_action_start(self) -> None:
    """单次 ACT 起点，仅用于日志位移参考。"""
    rx, ry, _ = self.pose.get_pose()
    self._recovery_before_x = rx
    self._recovery_before_y = ry

  def _recovery_success(self, scan: LaserScan) -> bool:
    profile = clearance_profile(
        scan,
        self.search.front_angle,
        self.search.nav_block_half_width,
        self.search.front_sector_half_width,
        self.search.boundary_dist,
    )
    center = float(profile.get('center', 0.0))
    rx, ry, _ = self.pose.get_pose()
    disp = math.hypot(rx - self._recovery_before_x, ry - self._recovery_before_y)
    center_gain = center - self._recovery_entry_center
    ok = center_gain >= self.search.recovery_success_center_gain
    if ok:
      rospy.loginfo(
          'Recovery(%s): 判定成功 level=%d center=%.2f entry=%.2f gain=%.2f disp=%.2fm',
          self._recovery_reason or '?',
          self._boundary_escalation,
          center,
          self._recovery_entry_center,
          center_gain,
          disp,
      )
    else:
      rospy.logwarn(
          'Recovery(%s): 效果不足 center=%.2f entry=%.2f gain=%.2f disp=%.2fm escalation=%d',
          self._recovery_reason or '?',
          center,
          self._recovery_entry_center,
          center_gain,
          disp,
          self._boundary_escalation,
      )
    return ok

  def _escalation_back_turn(self, level: int) -> Tuple[float, float]:
    cfg = self.search
    idx = min(level, len(cfg.recovery_backoff_levels) - 1)
    return cfg.recovery_backoff_levels[idx], cfg.recovery_turn_levels[idx]

  def _nav_hold_limit(self) -> int:
    return max(15, self.search.boundary_confirm_frames * 10)

  def _track_nav_hold(self, status: str) -> bool:
    """Navigator 报告 hold；累计超限后由 FSM 决策进 Recovery（非 Planner 重规划）。"""
    if self.frontier_nav.corridor_commit_active():
      self._nav_hold_streak = 0
      return False
    if status == 'hold':
      self._nav_hold_streak += 1
    else:
      self._nav_hold_streak = 0
    limit = self._nav_hold_limit()
    if self._nav_hold_streak >= limit:
      rospy.logwarn(
          'FSM: Navigator hold %d ticks (limit %d) — FSM enters boundary recovery',
          self._nav_hold_streak,
          limit,
      )
      self._nav_hold_streak = 0
      return True
    return False

  def _maybe_exit_bootstrap(self) -> bool:
    """Bootstrap 退出：一次性事件，统一入口调用。"""
    if self._bootstrap_exited or self._recovery_explore_active:
      return False
    exit_bootstrap, reason = self._should_exit_bootstrap()
    if not exit_bootstrap:
      return False
    self._bootstrap_exited = True
    rospy.logwarn('Bootstrap exit (%s) — switch to PLAN_NEXT', reason)
    self.local_planner.clear()
    self._bootstrap_recovery_streak = 0
    self._set_state(SearchState.PLAN_NEXT, 'bootstrap exit: {}'.format(reason))
    return True

  def _try_escape_or_replan(self, scan: LaserScan) -> None:
    """恢复完成：交还 FSM，直接重规划（P0：取消 escape_forward 直连 cmd_vel）。"""
    self._recovery_phase = None
    self._recovery_phase_started = None
    self._boundary_kind = None
    self._recovery_reason = None
    self.pose.reset_health_watch()
    self.local_planner.clear()
    self._complete_recovery_resume(scan)

  def _narrow_forward_clear(self, scan: LaserScan) -> bool:
    """与边界判定一致：仅窄扇区测距足够才视为前方畅通。"""
    return front_sector_clear(
        scan,
        self.search.front_angle,
        self.search.nav_block_half_width,
        self.search.boundary_dist,
    )

  def _forward_path_clear(self, scan: LaserScan) -> bool:
    return forward_path_clear(
        scan,
        self.search.front_angle,
        self.search.nav_block_half_width,
        self.search.front_sector_half_width,
        self.search.boundary_dist,
    )

  def _should_skip_boundary_action(self, scan: LaserScan, kind: str) -> bool:
    """Only a clear narrow passage may resume without a turn; walls always adjust."""
    if kind != 'narrow_passage':
      return False
    return self._forward_path_clear(scan)

  def _front_is_clear(self, scan: LaserScan) -> bool:
    return front_sector_clear(
        scan,
        self.search.front_angle,
        self.search.nav_block_half_width,
        self.search.boundary_dist,
    )

  def _on_boundary(self, scan: LaserScan) -> None:
    self._boundary_streak = 0
    self.move.publish_stop_once()
    self.mission_state.record_boundary()
    self._run_stats['boundary'] += 1.0

    kind = classify_boundary_type(
        scan,
        self.search.front_angle,
        self.search.nav_block_half_width,
        self.search.front_sector_half_width,
        self.search.boundary_dist,
    )
    rx, ry, ryaw = self.pose.get_pose()
    repeat = self._register_spatial_boundary(rx, ry)
    self.grid.record_collision(
        rx, ry,
        radius_m=self.search.boundary_spatial_radius * 0.5,
        increment=self.search.collision_cost_increment * 0.5,
        max_cost=self.search.collision_cost_max,
    )
    if repeat:
      kind = 'repeat'
      rospy.logwarn(
          '空间冷却：半径 %.2fm 内 %d 次，已记碰撞代价',
          self.search.boundary_spatial_radius,
          self.search.boundary_spatial_count,
      )

    rospy.loginfo(
        'Boundary hit (%d) type=%s',
        self.mission_state.boundary_events,
        kind,
    )
    goal = self.frontier_nav._final_goal or self.frontier_nav._goal
    self.local_planner.note_boundary_hit()

    if self.search.vision_at_boundary:
      self._recovery_phase = None
      self._set_state(SearchState.STOP_SCAN, 'boundary with vision scan')
      return

    profile = clearance_profile(
        scan,
        self.search.front_angle,
        self.search.nav_block_half_width,
        self.search.front_sector_half_width,
        self.search.boundary_dist,
    )
    obs = str(profile.get('obstacle', ''))
    center = float(profile.get('center', 99.0))
    pseudo_corner = (
        kind in ('repeat', 'side_wall', 'corner')
        or (obs == 'single_wall' and center < 0.32)
    )
    if pseudo_corner:
      self._blacklist_corner(goal, 'pseudo_corner')
      self._prefer_escape_frontier = True
      rospy.logwarn(
          'Pseudo-corner detected type=%s obs=%s center=%.2f — blacklist',
          kind,
          obs,
          center,
      )

    self._start_boundary_recovery(scan, kind)

  def _start_boundary_recovery(self, scan: LaserScan, kind: str) -> None:
    self._boundary_kind = kind
    self._recovery_reason = 'boundary_{}'.format(kind)
    self._recovery_phase = RecoveryPhase.PAUSE
    self._recovery_phase_started = rospy.Time.now()
    self._recovery_attempts = 0
    self._boundary_escalation = 0
    self._boundary_max_level_retries = 0
    self._nav_hold_streak = 0
    self._snapshot_recovery_entry(scan)
    rx, ry, _ = self.pose.get_pose()
    self.local_planner.record_failure(
        rx,
        ry,
        self.local_planner.last_selected_angle_deg(),
        kind,
    )
    visited, free, _ = self.grid.count_states()
    if (
        visited < self.config.map.bootstrap_visited
        and self.search.bootstrap_local_plan
        and not self._bootstrap_exited
    ):
      self._bootstrap_recovery_streak += 1
    self._run_stats['recovery'] += 1.0
    self._set_state(SearchState.RECOVERY, self._recovery_reason)

  def _execute_boundary_action(self, scan: LaserScan, kind: str, level: int) -> None:
    sign = self._pick_recovery_turn_sign(scan)
    cfg = self.search
    speed = cfg.cruise_speed * 0.5
    settle = 0.05
    back, turn_mag = self._escalation_back_turn(level)
    turn = sign * turn_mag

    self.move.stop_robot(repeats=2)
    if back > 0.0:
      self.move.move_distance_x(-back, speed, settle_sec=settle)
    self.move.rotate_angle(
        turn,
        cfg.turn_speed,
        get_yaw=self._get_yaw,
        settle_sec=settle,
    )
    rospy.loginfo(
        'Recovery(%s): unified level=%d back=%.2fm turn=%.0fdeg',
        self._recovery_reason,
        level,
        back,
        turn,
    )

  def _finish_recovery_resume(self, scan: LaserScan) -> None:
    self._boundary_escalation = 0
    self._try_escape_or_replan(scan)

  def _replan_after_recovery(self, scan: LaserScan, passage: bool = False) -> bool:
    """状态机决策：恢复后若干轮规划加约束，不强制 PLAN_NEXT。"""
    if passage:
      self.local_planner.arm_passage_replan(
          self.search.recovery_passage_replan_angle_deg,
          self.search.recovery_replan_max_dist_m,
          rounds=self.search.recovery_passage_replan_rounds,
      )
    else:
      self.local_planner.arm_post_recovery_replan(
          self.search.recovery_replan_max_angle_deg,
          self.search.recovery_replan_max_dist_m,
          rounds=self.search.recovery_replan_rounds,
      )
    return self.local_planner.replan_local(scan, force=True)

  def _should_exit_bootstrap(self) -> Tuple[bool, str]:
    """建图引导期退出判据：超时低覆盖、边界过多、visited 已满。"""
    if self._bootstrap_exited or not self.search.bootstrap_local_plan:
      return False, ''
    visited, free, _ = self.grid.count_states()
    bootstrap = self.config.map.bootstrap_visited
    if visited >= bootstrap:
      return False, ''
    elapsed = (rospy.Time.now() - self._bootstrap_started_at).to_sec()
    coverage = float(visited) / max(1.0, float(free))
    if self._bootstrap_start_coverage <= 0.0 and free > 0:
      self._bootstrap_start_coverage = coverage
    boundaries = self.mission_state.boundary_events
    if boundaries >= self.search.bootstrap_max_boundary:
      return True, 'boundary>={}'.format(self.search.bootstrap_max_boundary)
    if self._bootstrap_recovery_streak >= self.search.bootstrap_max_consecutive_recovery:
      return True, 'recovery_streak>={}'.format(
          self.search.bootstrap_max_consecutive_recovery,
      )
    if (
        elapsed >= self.search.bootstrap_max_sec
        and coverage - self._bootstrap_start_coverage
        < self.search.bootstrap_stall_coverage_delta
    ):
      return True, 'coverage_stall {:.0%}->{:.0%}'.format(
          self._bootstrap_start_coverage,
          coverage,
      )
    if (
        elapsed >= self.search.bootstrap_max_sec
        and coverage < self.search.bootstrap_min_coverage
    ):
      return True, 'timeout cov={:.0%}'.format(coverage)
    return False, ''

  def _complete_recovery_resume(self, scan: LaserScan) -> None:
    self._reset_explore_timer()
    self._explore_no_candidate_count = 0
    self.local_planner.refresh_penalty_for_replan()

    passage = self._resume_passage_mode
    self._resume_passage_mode = False

    if self._use_global_planner():
      if self._recovery_perimeter_resume:
        self._recovery_perimeter_resume = False
        if not self.frontier_nav.perimeter_active():
          self.frontier_nav.start_perimeter_mode()
        self.frontier_nav.reset_perimeter_controller()
        if passage:
          self.frontier_nav.arm_passage_mode()
        self._set_state(SearchState.MOVE_TO_GOAL, 'recovery done, resume perimeter')
        return
      gp = self.config.global_planner
      if gp.perimeter_enabled and gp.initial_mode == 'perimeter':
        self._goto_global_plan('recovery done, global replan')
      else:
        if passage:
          self._resume_passage_mode = True
        self._set_state(SearchState.PLAN_NEXT, 'recovery done, replan')
      return

    visited, _, _ = self.grid.count_states()
    bootstrap = self.config.map.bootstrap_visited
    if visited < bootstrap and self.search.bootstrap_local_plan and not self._bootstrap_exited:
      self._replan_after_recovery(scan, passage=passage)
      if passage:
        self.frontier_nav.arm_passage_mode()
      self._set_state(SearchState.EXPLORE, 'recovery done, resume explore')
      return
    if self.frontier_nav.has_goal() and self.local_planner.goal_valid(
        self.frontier_nav._final_goal[0],
        self.frontier_nav._final_goal[1],
        scan,
    ):
      if passage:
        self.frontier_nav.arm_passage_mode()
      self._set_state(SearchState.MOVE_TO_GOAL, 'recovery done, resume nav')
      return
    if self.search.bootstrap_local_plan and self._replan_after_recovery(
        scan,
        passage=passage,
    ):
      if passage:
        self.frontier_nav.arm_passage_mode()
      self._set_state(SearchState.EXPLORE, 'recovery done, local replan')
      return
    if visited < bootstrap and self.search.bootstrap_local_plan and not self._bootstrap_exited:
      self._set_state(SearchState.EXPLORE, 'recovery done, resume explore')
      return
    if passage:
      self._resume_passage_mode = True
    self._set_state(SearchState.PLAN_NEXT, 'recovery done, replan')

  def _tick_boundary_recovery(self, scan: LaserScan) -> None:
    reason = self._recovery_reason or 'boundary'
    phase = self._recovery_phase
    if phase is None:
      return

    if phase == RecoveryPhase.PAUSE:
      self.move.stop_robot(repeats=3)
      if self._recovery_phase_started is None:
        self._recovery_phase_started = rospy.Time.now()
      elapsed = (rospy.Time.now() - self._recovery_phase_started).to_sec()
      if elapsed < self.search.boundary_pause_sec:
        return
      self._recovery_phase = RecoveryPhase.RECHECK
      self._recovery_phase_started = rospy.Time.now()
      return

    if phase == RecoveryPhase.RECHECK:
      self.move.stop_robot(repeats=1)
      kind = self._boundary_kind or 'front_wall'
      if self._should_skip_boundary_action(scan, kind):
        rospy.loginfo('Recovery(%s): 通道可通行，继续', reason)
        if kind == 'narrow_passage':
          self._resume_passage_mode = True
        self._finish_recovery_resume(scan)
        return
      self._recovery_phase = RecoveryPhase.ACT
      return

    if phase == RecoveryPhase.ACT:
      kind = self._boundary_kind or 'front_wall'
      self._snapshot_recovery_action_start()
      self._execute_boundary_action(scan, kind, self._boundary_escalation)
      if self._recovery_success(scan):
        self._boundary_escalation = 0
        self._boundary_max_level_retries = 0
        self._finish_recovery_resume(scan)
        return
      at_max = (
          self._boundary_escalation >= self.search.recovery_max_escalation - 1
      )
      if at_max:
        self._boundary_max_level_retries += 1
        max_retries = self.search.recovery_max_level_retries
        if self._boundary_max_level_retries >= max_retries:
          rospy.logwarn(
              'Recovery(%s): level %d 重试 %d 次仍无 gain，强制结束恢复',
              reason,
              self._boundary_escalation,
              self._boundary_max_level_retries,
          )
          self._boundary_escalation = 0
          self._boundary_max_level_retries = 0
          self._finish_recovery_resume(scan)
          return
        rospy.logwarn(
            'Recovery(%s): level %d 未达 gain 阈值，继续同级恢复 (%d/%d)',
            reason,
            self._boundary_escalation,
            self._boundary_max_level_retries,
            max_retries,
        )
        self._recovery_phase = RecoveryPhase.PAUSE
        self._recovery_phase_started = rospy.Time.now()
        return
      self._boundary_escalation += 1
      rospy.logwarn(
          'Recovery(%s): 升级至 level %d',
          reason,
          self._boundary_escalation,
      )
      self._recovery_phase = RecoveryPhase.PAUSE
      self._recovery_phase_started = rospy.Time.now()
      return

  def _log_frontier_debug(self, dbg: Dict[str, int], goal_is_none: bool) -> None:
    level = rospy.logwarn if goal_is_none else rospy.loginfo
    msg = (
        f'Frontier pick: frontiers={dbg.get("frontiers", 0)} '
        f'candidates_raw={dbg.get("candidates_raw", 0)} '
        f'candidates={dbg.get("candidates", 0)} '
        f'blocked_skipped={dbg.get("blocked_skipped", 0)} '
        f'min_dist_skipped={dbg.get("min_dist_skipped", 0)} '
        f'path_rejected={dbg.get("path_rejected", 0)} '
        f'path_found={bool(dbg.get("path_found", 0))} '
        f'waypoints={dbg.get("path_waypoints", 0)} '
        f'astar_eval={dbg.get("astar_evaluated", 0)} '
        f'escape={bool(dbg.get("escape_pick", 0))} '
        f'blocked_goals={len(self._blocked_goals)}'
    )
    level(msg)


  def _tick_explore(self, scan: LaserScan) -> None:
    visited, _, _ = self.grid.count_states()
    if self.mode == 'occupancy_grid' and not self.frontier_nav.has_goal():
      if self._recovery_explore_active:
        if self._check_explore_timeout():
          rospy.loginfo('Recovery explore window done — replan')
          self._recovery_explore_active = False
          self._set_state(SearchState.PLAN_NEXT, 'recovery explore done')
          return
      elif visited >= self.config.map.bootstrap_visited:
        self._set_state(SearchState.PLAN_NEXT, 'occupancy_grid: no goal, replan')
        return

    if self._tick_count <= 5 or self._tick_count % 50 == 0:
      rx, ry, ryaw = self.pose.get_pose()
      rospy.loginfo(
          'FSM handler: EXPLORE pose=(%.2f, %.2f, yaw=%.1f deg) has_goal=%s',
          rx, ry, math.degrees(ryaw), self.frontier_nav.has_goal(),
      )
    if self._explore_origin is None:
      self._reset_explore_timer()

    if self._check_explore_timeout():
      rospy.logwarn('Explore stall — entering recovery.')
      self._enter_recovery('explore_stall')
      return

    if self._update_corner_stall(scan):
      rospy.logwarn(
          'Corner stall (EXPLORE): disp<3cm rot>60deg — blacklist & recovery',
      )
      failed_goal = self.frontier_nav._final_goal or self.frontier_nav._goal
      self.frontier_nav.clear_goal()
      self._blacklist_corner(failed_goal, 'corner_stall')
      self._run_stats['corner_stall'] += 1.0
      self._enter_recovery('corner_stall')
      return

    if self._is_boundary_confirmed(scan):
      if self.mode == 'boustrophedon':
        self.boustrophedon.on_boundary()
      self._on_boundary(scan)
      return

    if self.mode == 'occupancy_grid':
      status = self.local_planner.tick(scan, self.move, self.search.cruise_speed)
      if status == 'no_candidate':
        self._explore_no_candidate_count += 1
        if self._explore_no_candidate_count >= self.search.explore_no_candidate_frames:
          self._enter_planner_deadlock_recovery(scan)
        return
      self._explore_no_candidate_count = 0
      if status == 'passage':
        return
      if status == 'passage_done':
        self._reset_stall_watch()
        return
      if self._track_nav_hold(status):
        self._on_boundary(scan)
      elif status == 'blocked':
        self._on_boundary(scan)
      elif status == 'odom_fault':
        self._enter_recovery('odom_fault')
      elif status == 'reached':
        self._reset_explore_timer()
        self._bootstrap_recovery_streak = 0
      return

    if self.mode == 'boustrophedon':
      self.move.publish_twist(linear_x=self.boustrophedon.cruise_speed_signed())
    else:
      self.move.publish_twist(linear_x=self.search.cruise_speed)
      if hasattr(self.pose, 'note_cmd_vel'):
        from geometry_msgs.msg import Twist
        cmd = Twist()
        cmd.linear.x = self.search.cruise_speed
        self.pose.note_cmd_vel(cmd)

  def _tick_stop_scan(self) -> None:
    self.mission_state.record_scan_stop()
    self.move.stop_robot(repeats=3)
    rospy.sleep(0.25)

    image = self.get_image()
    depth = self.get_depth()
    camera_info = self.get_camera_info()
    robot_pose = self.pose.get_pose()

    self.last_scan_result = ScanResult(
        counts=dict(self.mission_state.counts),
        mission_complete=self.mission_state.is_mission_complete(),
    )
    for _ in range(5):
      result = self.perception.detect(
          image,
          depth=depth,
          camera_info=camera_info,
          robot_pose=robot_pose,
          mission_state=self.mission_state,
      )
      self.last_scan_result = result
      if result.new_objects > 0:
        break
      rospy.sleep(0.08)

    rospy.loginfo(
        'Vision: new=%d counts=%s',
        self.last_scan_result.new_objects,
        self.last_scan_result.counts,
    )

    self.state = SearchState.UPDATE_MAP

  def _tick_update_map(self) -> None:
    self.state = SearchState.PLAN_NEXT

  def _tick_global_plan(self, scan: LaserScan) -> None:
    """Global Planner: perimeter / frontier / coverage — then MOVE."""
    if self._is_boundary_confirmed(scan):
      self._on_boundary(scan)
      return

    rx, ry, ryaw = self.pose.get_pose()
    plan = self.global_planner.plan(
        rx,
        ry,
        ryaw,
        exclude_regions=self._blocked_goal_regions(),
        exclude_radius=self._effective_exclude_radius(),
        prefer_escape=self._prefer_escape_frontier,
        prefer_forward=not self._prefer_escape_frontier,
    )
    self._apply_global_plan(plan, scan)

  def _tick_plan_next(self) -> None:
    entering = self._last_handler_state != SearchState.PLAN_NEXT
    self._last_handler_state = SearchState.PLAN_NEXT
    if entering:
      rospy.loginfo('FSM handler: PLAN_NEXT (mode=%s tick=%d)', self.mode, self._tick_count)
    if self.mode == 'occupancy_grid':
      rx, ry, ryaw = self.pose.get_pose()
      gx, gy = self.grid._to_grid(rx, ry)
      in_bounds = self.grid.in_bounds(gx, gy)
      visited, free, unknown = self.grid.count_states()
      frontiers = self.grid.find_frontiers()
      if entering:
        rospy.loginfo(
            'PLAN_NEXT pose=(%.2f, %.2f, yaw=%.1f deg) grid=(%d,%d) in_bounds=%s '
            'map visited=%d free=%d unknown=%d frontiers=%d',
            rx, ry, math.degrees(ryaw), gx, gy, in_bounds,
            visited, free, unknown, len(frontiers),
        )
      bootstrap = self.config.map.bootstrap_visited
      if (
          visited < bootstrap
          and self.search.bootstrap_local_plan
          and not self._bootstrap_exited
      ):
        if not self._bootstrap_logged:
          rospy.loginfo(
              '建图引导：visited=%d，目标=%d，直走或局部绕行',
              visited, bootstrap,
          )
          self._bootstrap_logged = True
        self._set_state(SearchState.EXPLORE, 'map bootstrap')
        return
      goal, path, dbg = self.grid.nearest_frontier(
          rx, ry,
          robot_yaw=ryaw,
          exclude_regions=self._blocked_goal_regions(),
          exclude_radius=self._effective_exclude_radius(),
          prefer_escape=self._prefer_escape_frontier,
          max_astar_candidates=self.config.map.astar_max_candidates,
          prefer_forward=not self._prefer_escape_frontier,
      )
      if entering or goal is None:
        self._log_frontier_debug(dbg, goal_is_none=(goal is None))
      if goal is None:
        if (
            visited < bootstrap
            and self.search.bootstrap_local_plan
            and not self._bootstrap_exited
        ):
          rospy.logwarn(
              'PLAN_NEXT: map still building (visited=%d) — EXPLORE forward',
              visited,
          )
          self._set_state(SearchState.EXPLORE, 'map building')
          return
        rospy.logwarn(
            'PLAN_NEXT: no reachable frontier — RECOVERY (in_bounds=%s visited=%d)',
            in_bounds, visited,
        )
        self._set_state(SearchState.RECOVERY, 'no_frontier')
        return
      if not path:
        rospy.logwarn(
            'PLAN_NEXT: A* failed for frontier (%.2f, %.2f) — blacklist & replan',
            goal[0], goal[1],
        )
        self._add_blocked_region(goal[0], goal[1])
        return
      path = self._trim_path_forward(path, rx, ry, ryaw)
      self._recovery_explore_active = False
      self._recovery_goal = None
      self._reset_stall_watch()
      if not self.frontier_nav.set_path(path, goal):
        rospy.logwarn(
            'PLAN_NEXT: set_path rejected — stay in PLAN_NEXT (passage_deferred=%s)',
            self._resume_passage_mode,
        )
        return
      if entering:
        wp_idx, wp_total = self.frontier_nav.path_progress()
        rospy.loginfo(
            'PLAN_NEXT: frontier (%.2f, %.2f) path=%d wp (idx=%d)',
            goal[0], goal[1], wp_total, wp_idx,
        )
      if self._resume_passage_mode:
        self._resume_passage_mode = False
        self.frontier_nav.arm_passage_mode()
        rospy.loginfo('PLAN_NEXT: passage mode armed after replan')
      self._set_state(SearchState.MOVE_TO_GOAL, 'frontier goal set')
      return

    if self.mode == 'boustrophedon':
      if self.boustrophedon.phase.value == 'BOUNDARY_STOP':
        self.boustrophedon.step_boundary_stop()
        self.boustrophedon.step_shift_lane()
      self._set_state(SearchState.EXPLORE, 'boustrophedon lane continue')
      return

    self._set_state(SearchState.BOUNDARY_MANEUVER, 'non-grid plan next')

  def _blocked_goal_points(self) -> List[Tuple[float, float]]:
    return [(x, y) for x, y, _r in self._blocked_goals]

  def _blocked_goal_regions(self) -> List[Tuple[float, float, float]]:
    return list(self._blocked_goals)

  def _goal_key(self, gx: float, gy: float) -> Tuple[float, float]:
    return (round(gx, 2), round(gy, 2))

  def _record_goal_failure(self, gx: float, gy: float, reason: str) -> None:
    key = self._goal_key(gx, gy)
    if key not in self._goal_failures:
      self._goal_failures[key] = {}
    counts = self._goal_failures[key]
    counts[reason] = counts.get(reason, 0) + 1
    total = sum(counts.values())
    rospy.logwarn(
        'Goal failure (%.2f, %.2f) reason=%s count=%d total=%d',
        gx, gy, reason, counts[reason], total,
    )
    if total >= self._goal_failure_blacklist_threshold:
      self._add_blocked_region(gx, gy)
      self._goal_failures.pop(key, None)
      self._run_stats['blocked_goal'] += 1.0

  def _clear_goal_failures_for(self, gx: float, gy: float) -> None:
    self._goal_failures.pop(self._goal_key(gx, gy), None)

  def _reset_stall_watch(self) -> None:
    self._stall_start = None
    self._stall_rotation_accum = 0.0

  def _update_corner_stall(self, scan: LaserScan) -> bool:
    """True when robot spins in place near a wall without making progress."""
    rx, ry, ryaw = self.pose.get_pose()
    detail = self.frontier_nav.passage_detail()
    center = float(detail.get('center', float('inf')))
    goal = self.frontier_nav._final_goal or self.frontier_nav._goal
    now = rospy.Time.now()

    if self._stall_start is None:
      self._stall_start = now
      self._stall_x = rx
      self._stall_y = ry
      self._stall_last_yaw = ryaw
      self._stall_rotation_accum = 0.0
      self._stall_goal_dist = (
          self.pose.distance_to(goal[0], goal[1]) if goal is not None else 0.0
      )
      return False

    dyaw = abs(self._normalize_angle(ryaw - self._stall_last_yaw))
    self._stall_rotation_accum += dyaw
    self._stall_last_yaw = ryaw
    elapsed = (now - self._stall_start).to_sec()
    disp = math.hypot(rx - self._stall_x, ry - self._stall_y)

    if disp > 0.03:
      self._reset_stall_watch()
      return False

    if elapsed < 3.0:
      return False

    if goal is not None:
      dist_now = self.pose.distance_to(goal[0], goal[1])
      if abs(dist_now - self._stall_goal_dist) > 0.05:
        self._reset_stall_watch()
        return False

    if (
        disp < 0.03
        and self._stall_rotation_accum >= math.radians(60.0)
        and center < 0.35
    ):
      return True
    return False

  def _enter_perimeter_recovery(self, reason: str) -> None:
    self._recovery_reason = reason
    self._recovery_goal = None
    self._recovery_phase = None
    self._recovery_perimeter_resume = True
    self._prefer_escape_frontier = True
    self._nav_hold_streak = 0
    rx, ry, _ = self.pose.get_pose()
    self.local_planner.record_failure(
        rx,
        ry,
        self.local_planner.last_selected_angle_deg(),
        reason,
    )
    self._run_stats['recovery'] += 1.0
    self._set_state(SearchState.RECOVERY, reason)

  def _enter_recovery(self, reason: str) -> None:
    failed = self.frontier_nav._final_goal or self.frontier_nav._goal
    if failed is not None:
      self._record_goal_failure(failed[0], failed[1], reason)
      self._add_blocked_region(failed[0], failed[1], radius=self._corner_blacklist_radius)
    self.frontier_nav.clear_goal()
    self._recovery_reason = reason
    self._recovery_goal = None
    self._recovery_phase = None
    self._prefer_escape_frontier = True
    self._nav_hold_streak = 0
    rx, ry, _ = self.pose.get_pose()
    self.local_planner.record_failure(
        rx,
        ry,
        self.local_planner.last_selected_angle_deg(),
        reason,
    )
    self._run_stats['recovery'] += 1.0
    self._set_state(SearchState.RECOVERY, reason)

  def _trim_path_forward(
      self,
      path: List[Tuple[float, float]],
      rx: float,
      ry: float,
      ryaw: float,
      max_backward_deg: float = 95.0,
  ) -> List[Tuple[float, float]]:
    """Drop leading waypoints that lie behind the robot heading."""
    trimmed = list(path)
    limit = math.radians(max_backward_deg)
    while len(trimmed) > 1:
      wx, wy = trimmed[0]
      bearing = math.atan2(wy - ry, wx - rx)
      err = self._normalize_angle(bearing - ryaw)
      if abs(err) <= limit:
        break
      trimmed.pop(0)
    if not trimmed:
      ahead = 0.35
      trimmed = [(rx + ahead * math.cos(ryaw), ry + ahead * math.sin(ryaw))]
    return trimmed

  def _set_nav_target(self, wx: float, wy: float) -> None:
    rx, ry, ryaw = self.pose.get_pose()
    path = self.grid.plan_path(rx, ry, wx, wy)
    if path:
      path = self._trim_path_forward(path, rx, ry, ryaw)
      self.frontier_nav.set_path(path, (wx, wy))
    else:
      self.frontier_nav.set_goal(wx, wy)

  def _effective_exclude_radius(self) -> float:
    """Grow exclusion disk after repeated blocks in the same area."""
    bonus = 0.15 * max(0, self._consecutive_blocked - 1)
    return min(self._blocked_region_radius_max, self._blocked_region_radius + bonus)

  def _get_yaw(self) -> float:
    return self.pose.get_pose()[2]

  def _pick_recovery_turn_sign(self, scan: LaserScan) -> float:
    """Prefer the more open side; flip if the same side was tried last time."""
    profile = clearance_profile(
        scan,
        self.search.front_angle,
        self.search.nav_block_half_width,
        self.search.front_sector_half_width,
        self.search.boundary_dist,
    )
    left = float(profile.get('left', 99.0))
    right = float(profile.get('right', 99.0))
    tie_flip = None
    if abs(left - right) < 0.05 and self._last_recovery_turn_sign is not None:
      tie_flip = 'right' if self._last_recovery_turn_sign > 0 else 'left'
    side = open_side(scan, self.search.front_angle, tie_flip=tie_flip)
    sign = 1.0 if side == 'left' else -1.0
    if self._last_recovery_turn_sign is not None and sign == self._last_recovery_turn_sign:
      sign = -sign
      rospy.loginfo('Recovery: alternate turn side (last attempt used same side)')
    self._last_recovery_turn_sign = sign
    return sign

  def _recovery_turn_deg(self, scan: LaserScan, goal_ahead: bool = False) -> float:
    """Escalate turn angle: 15° → 30° → 45° (no 180° U-turn in passage)."""
    sign = self._pick_recovery_turn_sign(scan)
    level = min(2, self._recovery_escalation)
    detail = self.frontier_nav.passage_detail()
    score = float(detail.get('passage_score', 0.0))
    if goal_ahead or score > 0.55:
      magnitudes = (15.0, 25.0, 35.0)
    else:
      magnitudes = (15.0, 30.0, 45.0)
    return sign * magnitudes[level]

  def _add_blocked_region(self, wx: float, wy: float, radius: Optional[float] = None) -> None:
    """Exclude a disk around a failed goal so replan skips the same wall cluster."""
    effective_r = radius if radius is not None else self._effective_exclude_radius()
    self._blocked_goals.append((wx, wy, effective_r))
    if len(self._blocked_goals) > self._max_blocked_goals:
      self._blocked_goals.pop(0)
    rospy.logwarn(
        'Blocked region added center=(%.2f, %.2f) radius=%.2fm (regions=%d)',
        wx, wy, effective_r, len(self._blocked_goals),
    )

  def _blacklist_corner(self, goal: Optional[Tuple[float, float]], reason: str) -> None:
    """Record a corner pocket — blacklist both failed goal and current pose."""
    if goal is not None:
      self._record_goal_failure(goal[0], goal[1], reason)
      self._add_blocked_region(goal[0], goal[1], radius=self._blocked_region_radius)
    rx, ry, _ = self.pose.get_pose()
    self._add_blocked_region(rx, ry, radius=self._corner_blacklist_radius)
    self._prefer_escape_frontier = True

  def _run_strong_blocked_recovery(
      self,
      scan: LaserScan,
      gx: Optional[float] = None,
      gy: Optional[float] = None,
  ) -> None:
    """Leave a tight spot: back, small turn, forward — then replan."""
    speed = self.search.cruise_speed * 0.5
    turn = self.search.turn_speed
    goal_ahead = self._goal_ahead(gx, gy)
    turn_deg = self._recovery_turn_deg(scan, goal_ahead=goal_ahead)
    settle = 0.05
    get_yaw = self._get_yaw
    rospy.logwarn(
        'Strong blocked recovery: back 0.20m → rotate %.0f° → forward 0.35m (level=%d goal_ahead=%s)',
        turn_deg,
        min(2, self._recovery_escalation),
        goal_ahead,
    )
    self.move.stop_robot(repeats=3)
    self.move.move_distance_x(-0.20, speed, settle_sec=settle)
    self.move.rotate_angle(turn_deg, turn, get_yaw=get_yaw, settle_sec=settle)
    self.move.move_distance_x(0.35, speed, settle_sec=settle)
    self.move.stop_robot(repeats=1)
    rospy.sleep(self._plan_settle_sec)

  def _goal_ahead(self, gx: Optional[float], gy: Optional[float]) -> bool:
    if gx is None or gy is None:
      return False
    return abs(self.pose.angle_to(gx, gy)) <= math.radians(50.0)

  def _apply_passage_nudge(self, nudge_deg: float) -> None:
    goal = self.frontier_nav._final_goal or self.frontier_nav._goal
    if goal is None:
      return
    gx, gy = goal
    heading_err = self.pose.angle_to(gx, gy)
    sign = 1.0 if heading_err >= 0.0 else -1.0
    turn = sign * min(nudge_deg, abs(math.degrees(heading_err)))
    if abs(turn) < 2.0:
      turn = sign * nudge_deg
    rospy.loginfo(
        'Passage nudge: rotate %.1f deg toward goal (stress=%d)',
        turn,
        self.frontier_nav.stress_level(),
    )
    self.move.rotate_angle(
        turn,
        self.search.turn_speed,
        get_yaw=self._get_yaw,
        settle_sec=0.05,
    )
    self.frontier_nav.ack_nudge()

  def _handle_nav_blocked(
      self,
      gx: Optional[float],
      gy: Optional[float],
      scan: LaserScan,
  ) -> None:
    detail = self.frontier_nav.passage_detail()
    score = float(detail.get('passage_score', 0.0))
    obs = str(detail.get('obstacle', 'front_blocked'))
    self.frontier_nav.clear_goal()

    if gx is not None and gy is not None:
      self._record_goal_failure(gx, gy, obs)

    self._consecutive_blocked += 1
    self._recovery_escalation = min(2, self._recovery_escalation + 1)
    self._prefer_escape_frontier = True
    rospy.logwarn(
        'Nav blocked (%d/%d before strong recovery) obs=%s score=%.2f exclude_r=%.2fm escalation=%d',
        self._consecutive_blocked,
        self._blocked_escalate_threshold,
        obs,
        score,
        self._effective_exclude_radius(),
        self._recovery_escalation,
    )
    if self._consecutive_blocked >= self._blocked_escalate_threshold:
      self._consecutive_blocked = 0
      if gx is not None and gy is not None:
        self._add_blocked_region(gx, gy)
      self._run_strong_blocked_recovery(scan, gx, gy)
      self._set_state(SearchState.PLAN_NEXT, 'strong blocked recovery done')
      return
    speed = self.search.cruise_speed * 0.5
    settle = 0.05
    self.move.stop_robot(repeats=3)
    backoff = 0.12 if score > 0.45 else 0.18
    self.move.move_distance_x(-backoff, speed, settle_sec=settle)
    self.move.stop_robot(repeats=1)
    rospy.sleep(self._plan_settle_sec)
    if gx is not None and gy is not None:
      self._add_blocked_region(gx, gy, radius=self._corner_blacklist_radius)
    self._set_state(SearchState.PLAN_NEXT, 'nav blocked, backoff & replan')

  def _tick_move_to_goal(self, scan: LaserScan) -> None:
    if self._is_boundary_confirmed(scan):
      self._on_boundary(scan)
      return
    if self._update_corner_stall(scan):
      rospy.logwarn(
          'Corner stall: disp<3cm rot>60deg center<0.35 — blacklist & replan',
      )
      failed_goal = self.frontier_nav._final_goal or self.frontier_nav._goal
      self.frontier_nav.clear_goal()
      self._blacklist_corner(failed_goal, 'corner_stall')
      self._run_stats['corner_stall'] += 1.0
      self._recovery_escalation = min(2, self._recovery_escalation + 1)
      self._enter_recovery('corner_stall')
      return

    active_goal = self.frontier_nav._final_goal or self.frontier_nav._goal
    if self._use_global_planner() and self.frontier_nav.perimeter_active():
      rx, ry, ryaw = self.pose.get_pose()
      progress = self.global_planner.update(rx, ry, ryaw)
      if progress.get('perimeter_done'):
        rospy.loginfo(
            'FSM: perimeter done yaw=%.0fdeg visited=%d — replan frontier',
            float(progress.get('yaw_travel_deg', 0.0)),
            int(progress.get('visited', 0)),
        )
        self.frontier_nav.stop_perimeter_mode()
        plan = self.global_planner.plan(
            rx,
            ry,
            ryaw,
            exclude_regions=self._blocked_goal_regions(),
            exclude_radius=self._effective_exclude_radius(),
            prefer_escape=self._prefer_escape_frontier,
            prefer_forward=not self._prefer_escape_frontier,
        )
        self._apply_global_plan(plan, scan)
        return
      status = self.frontier_nav.tick_perimeter_mode(scan, self.search.cruise_speed)
    elif self.mode == 'occupancy_grid':
      if not self.frontier_nav.has_goal():
        rospy.logwarn('MOVE_TO_GOAL: no frontier goal — replan')
        self._set_state(SearchState.PLAN_NEXT, 'move without goal, replan')
        return
      status = self.frontier_nav.tick(scan, self.search.cruise_speed)
    elif self._use_global_planner() and self.frontier_nav.has_goal():
      status = self.frontier_nav.tick(scan, self.search.cruise_speed)
    else:
      status = self.local_planner.tick(scan, self.move, self.search.cruise_speed)
    if self._tick_count <= 10 or self._tick_count % 20 == 0:
      rx, ry, ryaw = self.pose.get_pose()
      detail = self.frontier_nav.passage_detail()
      wp_idx, wp_total = self.frontier_nav.path_progress()
      rospy.loginfo(
          'FSM handler: MOVE_TO_GOAL status=%s pose=(%.2f, %.2f, yaw=%.1f deg) '
          'wp=%d/%d obs=%s score=%.2f center=%.2f wide=%.2f stress=%d',
          status, rx, ry, math.degrees(ryaw),
          wp_idx, wp_total,
          detail.get('obstacle', '?'),
          float(detail.get('passage_score', 0.0)),
          float(detail.get('center', 0.0)),
          float(detail.get('wide', 0.0)),
          self.frontier_nav.stress_level(),
      )
    if status == 'nudge_5':
      self._apply_passage_nudge(5.0)
      return
    if status == 'nudge_10':
      self._apply_passage_nudge(10.0)
      return
    if status == 'passage':
      return
    if status == 'perimeter_recovery':
      self._enter_perimeter_recovery('perimeter_blocked')
      return
    if status == 'passage_done':
      self._reset_stall_watch()
      if self.frontier_nav.perimeter_active():
        self.frontier_nav.reset_perimeter_controller()
        return
      if self._use_global_planner() and not self.frontier_nav.perimeter_active():
        self._goto_global_plan('passage done, replan')
      return
    if self._track_nav_hold(status):
      self._on_boundary(scan)
    elif status == 'reached':
      self._consecutive_blocked = 0
      self._recovery_escalation = 0
      self._recovery_attempts = 0
      self._recovery_goal = None
      self._last_recovery_turn_sign = None
      self._prefer_escape_frontier = False
      self._reset_stall_watch()
      if active_goal is not None:
        self._clear_goal_failures_for(active_goal[0], active_goal[1])
      if self._use_global_planner():
        gp = self.config.global_planner
        if gp.perimeter_enabled and gp.initial_mode == 'perimeter':
          self._goto_global_plan('move done, replan')
        else:
          self._set_state(SearchState.PLAN_NEXT, 'frontier reached, replan')
      elif self.mode == 'occupancy_grid':
        self._set_state(SearchState.PLAN_NEXT, 'frontier reached, replan')
      else:
        self._set_state(SearchState.EXPLORE, 'goal reached')
      self._reset_explore_timer()
    elif status == 'blocked':
      gx, gy = active_goal or (None, None)
      self._handle_nav_blocked(gx, gy, scan)
    elif status == 'odom_fault':
      self._enter_recovery('odom_fault')

  def _tick_recovery(self, scan: LaserScan) -> None:
    """P0：Recovery 独占 cmd_vel；本状态不 tick Planner / Navigator。"""
    if self._recovery_phase is not None:
      self._tick_boundary_recovery(scan)
      return
    self._tick_generic_recovery(scan)

  def _tick_generic_recovery(self, scan: LaserScan) -> None:
    reason = self._recovery_reason or 'unknown'
    self._boundary_streak = 0
    self.move.publish_stop_brief()
    self.frontier_nav.clear_goal()

    self._recovery_attempts += 1
    if self._recovery_attempts >= self._max_recovery_attempts:
      rospy.logwarn(
          'Recovery(%s): %d attempts exhausted — clear blocked_goals (%d), EXPLORE',
          reason,
          self._max_recovery_attempts,
          len(self._blocked_goals),
      )
      self._blocked_goals.clear()
      self._recovery_attempts = 0
      self._recovery_goal = None
      self._recovery_reason = None
      self._recovery_explore_active = True
      self._reset_stall_watch()
      self._reset_explore_timer()
      self._set_state(SearchState.EXPLORE, 'recovery exhausted')
      return

    rotate_deg = self._recovery_rotate_step_deg * self._recovery_attempts
    sign = 1.0 if open_side(scan, self.search.front_angle) == 'left' else -1.0
    rotate_deg = sign * abs(rotate_deg)
    rospy.loginfo(
        'Recovery(%s): rotate %.0f deg (attempt %d/%d)',
        reason,
        rotate_deg,
        self._recovery_attempts,
        self._max_recovery_attempts,
    )
    self.move.rotate_angle(
        rotate_deg,
        self.search.turn_speed,
        get_yaw=self._get_yaw,
    )
    self.pose.reset_health_watch()
    self._reset_stall_watch()
    self._reset_explore_timer()
    self._recovery_goal = None
    self._recovery_reason = None

    visited, _, _ = self.grid.count_states()
    if (
        self.mode == 'occupancy_grid'
        and visited < self.config.map.bootstrap_visited
        and self.search.bootstrap_local_plan
        and not self._bootstrap_exited
    ):
      self._replan_after_recovery(scan)
      self._set_state(SearchState.EXPLORE, 'recovery rotate done')
      return
    if self.mode == 'occupancy_grid':
      self._set_state(SearchState.PLAN_NEXT, 'recovery rotate done')
    else:
      self._set_state(SearchState.BOUNDARY_MANEUVER, 'recovery maneuver')

  def _tick_return_home(self, scan: LaserScan) -> None:
    if self._start_pose is None:
      self.state = SearchState.DONE
      return

    sx, sy, syaw = self._start_pose
    rx, ry, ryaw = self.pose.get_pose()

    if self._is_at_home_position():
      if self.config.map.align_yaw_on_return:
        yaw_err = self._normalize_angle(syaw - ryaw)
        if abs(yaw_err) > math.radians(12.0):
          w = max(-0.6, min(0.6, 2.0 * yaw_err))
          self.move.publish_twist(angular_z=w)
          return
      self.move.stop_robot()
      self.state = SearchState.DONE
      rospy.loginfo(
          'Returned to start (%.2f, %.2f). Mission finished: %s',
          sx, sy, self.mission_state.status_line(),
      )
      return

    if not self.frontier_nav.has_goal():
      self._set_nav_target(sx, sy)

    status = self.frontier_nav.tick(scan, self.search.cruise_speed)
    if status == 'reached':
      self.frontier_nav.clear_goal()
    elif status == 'blocked':
      self.move.stop_robot()
      self.frontier_nav.clear_goal()
      self.move.rotate_angle(
          self.search.turn_angle_deg * 0.5,
          self.search.turn_speed,
          get_yaw=self._get_yaw,
      )

  def _tick_boundary_maneuver(self, scan: LaserScan) -> None:
    angle = self.search.turn_angle_deg
    speed = self.search.turn_speed
    mode = self.mode

    if mode == 'open_side':
      side = open_side(scan, self.search.front_angle)
      sign = 1.0 if side == 'left' else -1.0
      self.move.rotate_angle(sign * angle, speed, get_yaw=self._get_yaw)
    else:
      self.move.rotate_angle(angle, speed, get_yaw=self._get_yaw)

    if mode in ('left_hand_rule', 'open_side'):
      turn_left = mode != 'open_side' or open_side(scan, self.search.front_angle) == 'left'
      self.move.follow_wall(
          duration=self.search.wall_follow_duration,
          forward_speed=self.search.wall_follow_speed,
          turn_speed=speed * 0.15,
          turn_left=turn_left,
      )

    self.state = SearchState.EXPLORE
    self._reset_explore_timer()

  def _tick_wall_follow(self, scan: LaserScan) -> None:
    """Placeholder for closed-loop wall following (phase 2 on vehicle)."""
    if not getattr(self, '_wall_follow_warned', False):
      rospy.logwarn('WALL_FOLLOW not implemented yet — fallback to boundary maneuver.')
      self._wall_follow_warned = True
    self.state = SearchState.BOUNDARY_MANEUVER

  def status_string(self) -> str:
    extra = ''
    if self.mode == 'boustrophedon':
      extra = ' ' + self.boustrophedon.status()
    elif self.mode == 'occupancy_grid':
      extra = ' cov={:.0%}'.format(self.mission_state.coverage_ratio)
    return 'state={}{} | {}'.format(self.state.value, extra, self.mission_state.status_line())
