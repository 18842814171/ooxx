"""Navigate along A* waypoints — single control law + execution feedback."""

from __future__ import annotations

import math
from typing import Dict, List, Optional, Tuple

import rospy
from sensor_msgs.msg import LaserScan

from lidar_utils import clearance_profile
from move_controller import MoveController
from nav_feedback import GoalStatus, NavExecutionFeedback
from perimeter_controller import PerimeterController
from pose_estimator import PoseEstimator

NAV_BUILD_ID = '20260706-simple-avoid'


class FrontierNavigator:
  def __init__(
      self,
      move: MoveController,
      pose: PoseEstimator,
      boundary_dist: float = 0.35,
      front_angle: float = 3.14159,
      front_sector_half_width: float = 0.22,
      nav_block_half_width: float = 0.14,
      goal_tolerance: float = 0.03,
      waypoint_tolerance: float = 0.10,
      align_tolerance_deg: float = 18.0,
      heading_kp: float = 2.0,
      boundary_confirm_frames: int = 3,
      creep_speed: float = 0.06,
      align_timeout_sec: float = 1.5,
      align_arc_speed: float = 0.04,
      forward_stop_margin: float = 0.04,
      cruise_clear_margin: float = 0.08,
      cruise_resume_center: float = 0.45,
      wide_hold_margin: float = 0.08,
      lateral_block_dist: float = 0.25,
      robot_half_width: float = 0.18,
      robot_front_overhang: float = 0.08,
      passage_score_threshold: float = 0.45,
      passage_max_ticks: int = 80,
      passage_creep_factor: float = 0.90,
      passage_max_heading_corr: float = 0.12,
      passage_auto_arm: bool = True,
      passage_channel_auto_arm: bool = False,
      passage_wide_max: float = 0.48,
      passage_side_max: float = 0.65,
      passage_center_min_ratio: float = 0.80,
      passage_channel_close_max: float = 0.55,
      passage_channel_open_min: float = 0.75,
      passage_channel_center_min_ratio: float = 1.15,
      passage_channel_heading_max_deg: float = 35.0,
      passage_channel_min_hold_ticks: int = 15,
      passage_channel_lost_streak: int = 6,
      passage_channel_exit_yaw_hold_sec: float = 2.0,
      corridor_commit_travel_m: float = 0.8,
      corridor_commit_cooldown_sec: float = 2.0,
      corridor_commit_arm_confirm_frames: int = 10,
      corridor_commit_min_hold_ticks: int = 18,
      corridor_commit_min_travel_m: float = 0.30,
      corridor_commit_abort_yaw_deg: float = 45.0,
      corridor_commit_hard_abort_ratio: float = 0.55,
      corridor_centerline_delta: float = 0.08,
      wall_target_dist: float = 0.25,
      perimeter_wall_side: str = 'right',
      perimeter_lost_wall_dist: float = 0.80,
      perimeter_found_wall_dist: float = 0.65,
      perimeter_corner_stuck_ticks: int = 25,
      open_align_center_min: float = 0.65,
      open_align_arc_speed: float = 0.10,
      wide_hold_override_center_min: float = 0.70,
      drive_heading_far_dist: float = 1.5,
      drive_heading_far_deg: float = 32.0,
      drive_heading_mid_dist: float = 0.9,
      drive_heading_mid_deg: float = 28.0,
  ):
    self.move = move
    self.pose = pose
    self.boundary_dist = boundary_dist
    self.front_angle = front_angle
    self.front_sector_half_width = front_sector_half_width
    self.nav_block_half_width = nav_block_half_width
    self.goal_tolerance = goal_tolerance
    self.waypoint_tolerance = waypoint_tolerance
    self.align_tolerance = math.radians(align_tolerance_deg)
    self.drive_heading_tolerance = math.radians(22.0)
    self.spin_only_limit = math.radians(50.0)
    self.heading_kp = heading_kp
    self.boundary_confirm_frames = boundary_confirm_frames
    self.creep_speed = creep_speed
    self.align_timeout_sec = align_timeout_sec
    self.align_arc_speed = align_arc_speed
    self.forward_stop_margin = forward_stop_margin
    self.cruise_clear_margin = cruise_clear_margin
    self.cruise_resume_center = cruise_resume_center
    self.wide_hold_margin = wide_hold_margin
    self.lateral_block_dist = lateral_block_dist
    self.robot_half_width = robot_half_width
    self.robot_front_overhang = robot_front_overhang
    self.passage_score_threshold = passage_score_threshold
    self.passage_max_ticks = passage_max_ticks
    self.passage_creep_factor = passage_creep_factor
    self.passage_max_heading_corr = passage_max_heading_corr
    self.passage_auto_arm = passage_auto_arm
    self.passage_channel_auto_arm = passage_channel_auto_arm
    self.passage_wide_max = passage_wide_max
    self.passage_side_max = passage_side_max
    self.passage_center_min_ratio = passage_center_min_ratio
    self.passage_channel_close_max = passage_channel_close_max
    self.passage_channel_open_min = passage_channel_open_min
    self.passage_channel_center_min_ratio = passage_channel_center_min_ratio
    self.passage_channel_heading_max = math.radians(passage_channel_heading_max_deg)
    self.passage_channel_min_hold_ticks = passage_channel_min_hold_ticks
    self.passage_channel_lost_streak_frames = passage_channel_lost_streak
    self.passage_channel_exit_yaw_hold_sec = passage_channel_exit_yaw_hold_sec
    self.corridor_commit_travel_m = corridor_commit_travel_m
    self.corridor_commit_cooldown_sec = corridor_commit_cooldown_sec
    self.corridor_commit_arm_confirm_frames = corridor_commit_arm_confirm_frames
    self.corridor_commit_min_hold_ticks = corridor_commit_min_hold_ticks
    self.corridor_commit_min_travel_m = corridor_commit_min_travel_m
    self.corridor_commit_abort_yaw = math.radians(corridor_commit_abort_yaw_deg)
    self.corridor_commit_hard_abort_ratio = corridor_commit_hard_abort_ratio
    self.corridor_centerline_delta = corridor_centerline_delta
    self.wall_target_dist = wall_target_dist
    self.perimeter_wall_side = perimeter_wall_side
    self._perimeter_ctrl = PerimeterController(
        wall_side=perimeter_wall_side,
        wall_target_dist=wall_target_dist,
        lost_wall_dist=perimeter_lost_wall_dist,
        found_wall_dist=perimeter_found_wall_dist,
        creep_speed=creep_speed,
        forward_stop_dist=boundary_dist + forward_stop_margin + robot_front_overhang,
        lateral_block_dist=lateral_block_dist,
        robot_half_width=robot_half_width,
        corner_stuck_ticks=perimeter_corner_stuck_ticks,
    )
    self.open_align_center_min = open_align_center_min
    self.open_align_arc_speed = open_align_arc_speed
    self.wide_hold_override_center_min = wide_hold_override_center_min
    self.drive_heading_far_dist = drive_heading_far_dist
    self.drive_heading_far_deg = math.radians(drive_heading_far_deg)
    self.drive_heading_mid_dist = drive_heading_mid_dist
    self.drive_heading_mid_deg = math.radians(drive_heading_mid_deg)
    self.align_arc_max_w = 0.30
    self.align_max_w = 0.55
    self.drive_max_w = 0.18
    self._boundary_release_margin = 0.06
    self._goal: Optional[Tuple[float, float]] = None
    self._final_goal: Optional[Tuple[float, float]] = None
    self._path: List[Tuple[float, float]] = []
    self._path_index = 0
    self._center_close_streak = 0
    self._stress_level = 0
    self._last_profile: Dict[str, float] = {}
    self._align_started_at: Optional[rospy.Time] = None
    self._feedback = NavExecutionFeedback()
    self._passage_active = False
    self._passage_locked_yaw = 0.0
    self._passage_ticks = 0
    self._passage_channel_mode = False
    self._channel_lost_streak = 0
    self._channel_exit_yaw: Optional[float] = None
    self._channel_exit_until: Optional[rospy.Time] = None
    self._corridor_commit_active = False
    self._commit_entry_yaw = 0.0
    self._commit_open_side = 'none'
    self._commit_start_pose: Optional[Tuple[float, float]] = None
    self._commit_cooldown_until: Optional[rospy.Time] = None
    self._passage_arm_streak = 0
    self._passage_debug_ticks = 0
    self._perimeter_active = False

  def execution_feedback(self) -> NavExecutionFeedback:
    return self._feedback

  def set_goal(self, wx: float, wy: float) -> None:
    self.set_path([(wx, wy)], (wx, wy))

  def set_path(
      self,
      waypoints: List[Tuple[float, float]],
      final_goal: Tuple[float, float],
  ) -> bool:
    # if self._corridor_commit_active:
    #   rospy.logwarn(
    #       'Navigator: set_path skipped — corridor_commit_active',
    #   )
    #   return False
    self._perimeter_active = False
    self._path = list(waypoints) if waypoints else [final_goal]
    self._path_index = 0
    self._final_goal = final_goal
    self._goal = self._active_target()
    self.reset_passage_state()
    self._feedback = NavExecutionFeedback(goal_status=GoalStatus.RUNNING)
    return True

  def clear_goal(self) -> None:
    self._goal = None
    self._final_goal = None
    self._path = []
    self._path_index = 0
    self.reset_passage_state()
    self._feedback = NavExecutionFeedback()

  def has_goal(self) -> bool:
    return self._final_goal is not None

  def reset_passage_state(self) -> None:
    self._center_close_streak = 0
    self._stress_level = 0
    self._align_started_at = None
    self._passage_active = False
    self._passage_ticks = 0
    self._passage_channel_mode = False
    self._channel_lost_streak = 0
    self._channel_exit_yaw = None
    self._channel_exit_until = None
    self._corridor_commit_active = False
    self._commit_entry_yaw = 0.0
    self._commit_open_side = 'none'
    self._commit_start_pose = None
    self._passage_arm_streak = 0

  def start_perimeter_mode(self) -> None:
    self.clear_goal()
    self._perimeter_active = True
    self._perimeter_ctrl.reset()
    rospy.loginfo('Navigator: PerimeterMapping start wall_target=%.2f', self.wall_target_dist)

  def stop_perimeter_mode(self) -> None:
    self._perimeter_active = False

  def reset_perimeter_controller(self) -> None:
    self._perimeter_ctrl.reset()

  def perimeter_active(self) -> bool:
    return self._perimeter_active

  def corridor_commit_active(self) -> bool:
    # return self._corridor_commit_active
    return False

  def _centerline_err(self, left: float, right: float) -> float:
    return left - right

  def _apply_commit_centerline(
      self,
      profile: Dict[str, float],
      cmd_v: float,
  ) -> Tuple[float, float, float]:
    """Dead-zone lateral: shift away from the closer side (same sign as lateral_safety)."""
    left = float(profile.get('left', 99.0))
    right = float(profile.get('right', 99.0))
    err = self._centerline_err(left, right)
    delta = self.corridor_centerline_delta
    vy = self.creep_speed * 0.85
    strafe = 0.0
    if left < right - delta:
      strafe = -vy
    elif right < left - delta:
      strafe = vy

    if self._passage_channel_mode:
      if self._commit_open_side == 'left' and strafe < 0.0:
        strafe = 0.0
      elif self._commit_open_side == 'right' and strafe > 0.0:
        strafe = 0.0
      close = min(left, right)
      near_limit = self.passage_channel_close_max + self.robot_half_width * 0.25
      if close < near_limit:
        # 近侧过近：禁止横移（含朝物块侧），保留沿入口航向蠕行前进。
        strafe = 0.0

    return cmd_v, strafe, err

  def _begin_corridor_commit(self, locked_yaw: float, channel: bool) -> None:
    rx, ry, _ = self.pose.get_pose()
    left = float(self._last_profile.get('left', 99.0))
    right = float(self._last_profile.get('right', 99.0))
    self._corridor_commit_active = True
    self._commit_entry_yaw = locked_yaw
    self._commit_open_side = 'right' if right >= left else 'left'
    self._commit_start_pose = (rx, ry)
    err = self._centerline_err(left, right)
    tag = 'channel' if channel else 'gap'
    rospy.loginfo(
        'Navigator: CorridorCommit start entry_yaw=%.1f° open=%s pose=(%.2f,%.2f) '
        'L=%.2f R=%.2f err=%.2f strafe=0.000 %s',
        math.degrees(self._commit_entry_yaw),
        self._commit_open_side,
        rx,
        ry,
        left,
        right,
        err,
        tag,
    )

  def _commit_forward_dist(self) -> float:
    if self._commit_start_pose is None:
      return 0.0
    sx, sy = self._commit_start_pose
    rx, ry, _ = self.pose.get_pose()
    dx = rx - sx
    dy = ry - sy
    yaw = self._commit_entry_yaw
    return dx * math.cos(yaw) + dy * math.sin(yaw)

  def _release_corridor_commit(self, reason: str) -> str:
    dist = self._commit_forward_dist()
    open_side = self._commit_open_side
    entry_yaw = self._commit_entry_yaw
    ticks = self._passage_ticks
    was_channel = self._passage_channel_mode
    left = float(self._last_profile.get('left', 0.0))
    right = float(self._last_profile.get('right', 0.0))
    final_err = self._centerline_err(left, right)
    self._passage_active = False
    self._passage_channel_mode = False
    self._corridor_commit_active = False
    self._commit_start_pose = None
    self._commit_cooldown_until = (
        rospy.Time.now()
        + rospy.Duration(self.corridor_commit_cooldown_sec)
    )
    if was_channel:
      self._channel_exit_yaw = entry_yaw
      self._channel_exit_until = self._commit_cooldown_until
    rospy.loginfo(
        'Navigator: CorridorCommit done reason=%s dist=%.2f ticks=%d '
        'entry_yaw=%.1f° open=%s channel=%s final_L=%.2f final_R=%.2f final_err=%.2f',
        reason,
        dist,
        ticks,
        math.degrees(entry_yaw),
        open_side,
        was_channel,
        left,
        right,
        final_err,
    )
    return 'passage_done'

  def arm_passage_mode(
      self,
      locked_yaw: Optional[float] = None,
      channel: bool = False,
  ) -> None:
    """20260706：已禁用 — 缝隙与通道模式；统一走一般避障。"""
    # _, _, yaw = self.pose.get_pose()
    # self._passage_active = True
    # self._passage_channel_mode = channel
    # self._passage_locked_yaw = yaw if locked_yaw is None else locked_yaw
    # self._passage_ticks = 0
    # self._channel_lost_streak = 0
    # self._center_close_streak = 0
    # self._stress_level = 0
    # self._align_started_at = None
    # self._begin_corridor_commit(self._passage_locked_yaw, channel)
    pass

  def passage_mode_active(self) -> bool:
    return self._passage_active

  def tick_perimeter_mode(
      self,
      scan: LaserScan,
      cruise_speed: float,
  ) -> str:
    """Perimeter mapping via PerimeterController state machine."""
    profile = self._profile(scan)
    rx, ry, yaw = self.pose.get_pose()

    # if self._passage_active:
    #   return self._tick_passage_mode(scan, cruise_speed, profile, 0.0, 0.0)
    #
    # center = float(profile.get('center', 0.0))
    # if (
    #     self._is_block_wall_channel(profile)
    #     and center >= self.boundary_dist * self.passage_channel_center_min_ratio
    # ):
    #   self.arm_passage_mode(locked_yaw=yaw, channel=True)
    #   return self._tick_passage_mode(scan, cruise_speed, profile, 0.0, 0.0)

    cmd_v, w, strafe, sub_state, needs_recovery = self._perimeter_ctrl.tick(
        profile,
        yaw,
        rx,
        ry,
        cruise_speed,
    )
    self._publish_drive(cmd_v, w, linear_y=strafe)
    self._update_feedback('perimeter:{}'.format(sub_state), profile, 0.0)
    if needs_recovery:
      return 'perimeter_recovery'
    return 'perimeter:{}'.format(sub_state)

  def ack_nudge(self) -> None:
    self._center_close_streak = 0

  def passage_detail(self) -> Dict[str, float]:
    return dict(self._last_profile)

  def stress_level(self) -> int:
    return self._stress_level

  def path_progress(self) -> Tuple[int, int]:
    return self._path_index, len(self._path)

  def _active_target(self) -> Optional[Tuple[float, float]]:
    while self._path_index < len(self._path):
      wx, wy = self._path[self._path_index]
      if self.pose.distance_to(wx, wy) <= self.waypoint_tolerance:
        self._path_index += 1
        continue
      return wx, wy
    return self._final_goal

  def _advance_waypoints(self) -> None:
    self._goal = self._active_target()

  def _normalize_heading_error(self, heading_err: float) -> float:
    while heading_err > math.pi:
      heading_err -= 2.0 * math.pi
    while heading_err < -math.pi:
      heading_err += 2.0 * math.pi
    return heading_err

  def _clamp01(self, value: float) -> float:
    return max(0.0, min(1.0, value))

  def _linear_cost(self, value: float, good: float, bad: float) -> float:
    if value >= good:
      return 0.0
    if value <= bad:
      return 1.0
    return self._clamp01((good - value) / max(1e-6, good - bad))

  def _effective_stop_dist(self) -> float:
    return self.boundary_dist + self.forward_stop_margin + self.robot_front_overhang

  def _effective_lateral_dist(self) -> float:
    return self.lateral_block_dist + self.robot_half_width

  def _passage_abort_center(self) -> float:
    if self._passage_channel_mode:
      return self.boundary_dist * 0.75
    return self.boundary_dist + 0.02

  def _passage_abort_hard_center(self) -> float:
    return self.boundary_dist * self.corridor_commit_hard_abort_ratio

  def _corridor_in_commitment_phase(self, forward_dist: float) -> bool:
    """Early commit ticks / short travel — ignore soft center abort."""
    return (
        self._passage_ticks < self.corridor_commit_min_hold_ticks
        or forward_dist < self.corridor_commit_min_travel_m
    )

  def _should_abort_corridor_commit(
      self,
      center: float,
      forward_dist: float,
      heading_err: float,
  ) -> Optional[str]:
    if center < self._passage_abort_hard_center():
      return 'hard_center'
    if abs(heading_err) > self.corridor_commit_abort_yaw:
      return 'yaw'
    if self._passage_ticks >= self.passage_max_ticks:
      return 'timeout'
    if self._corridor_in_commitment_phase(forward_dist):
      return None
    if center < self._passage_abort_center():
      return 'center'
    return None

  def _passage_min_center(self) -> float:
    if self._passage_channel_mode:
      return self.boundary_dist * 0.90
    return self.boundary_dist * 0.85

  def _forward_stop_dist(self) -> float:
    return self._effective_stop_dist()

  def _cruise_clear_dist(self) -> float:
    return self.boundary_dist + self.cruise_clear_margin

  def _wide_hold_dist(self) -> float:
    return self.boundary_dist + self.wide_hold_margin

  def _drive_heading_tolerance_for(self, dist_goal: float) -> float:
    """远处目标放宽航向容差，减少开阔区原地对准。"""
    if dist_goal >= self.drive_heading_far_dist:
      return self.drive_heading_far_deg
    if dist_goal >= self.drive_heading_mid_dist:
      return self.drive_heading_mid_deg
    return self.drive_heading_tolerance

  def _is_block_wall_channel(self, profile: Dict[str, float]) -> bool:
    """一侧物块/墙近、另一侧开阔 — 第二个缝几何。"""
    left = float(profile.get('left', 99.0))
    right = float(profile.get('right', 99.0))
    center = float(profile.get('center', 0.0))
    center_min = self.boundary_dist * self.passage_channel_center_min_ratio
    if center < center_min:
      return False
    left_close = left < self.passage_channel_close_max
    right_close = right < self.passage_channel_close_max
    if left_close and not right_close and right >= self.passage_channel_open_min:
      return True
    if right_close and not left_close and left >= self.passage_channel_open_min:
      return True
    return False

  def _passage_arm_block_reason(
      self,
      profile: Dict[str, float],
      heading_err: float,
  ) -> Optional[str]:
    """Return None if passage may arm; otherwise human-readable block reason."""
    if self._perimeter_active:
      return 'perimeter_mode'
    if not self.passage_auto_arm:
      return 'auto_arm_disabled'
    if self._passage_active:
      return 'already_active'
    if (
        self._commit_cooldown_until is not None
        and rospy.Time.now() < self._commit_cooldown_until
    ):
      return 'commit_cooldown'

    score = float(profile.get('passage_score', 0.0))
    center = float(profile.get('center', 0.0))
    wide = float(profile.get('wide', 0.0))
    left = float(profile.get('left', 99.0))
    right = float(profile.get('right', 99.0))
    center_min = self.boundary_dist * self.passage_center_min_ratio
    channel = self._is_block_wall_channel(profile)

    if channel:
      if not self.passage_channel_auto_arm:
        return 'channel_auto_arm_disabled'
      if center > 1.0 or wide > 0.65:
        return 'open_area center={:.2f} wide={:.2f}'.format(center, wide)
      if center >= 0.85:
        return 'channel_ahead_clear center={:.2f}'.format(center)
      if center < 0.42:
        return 'channel_front_low center={:.2f}'.format(center)
      return None

    if score < self.passage_score_threshold:
      return 'score={:.2f}<{:.2f}'.format(score, self.passage_score_threshold)
    if wide >= self.passage_wide_max:
      return 'wide={:.2f}>={:.2f}'.format(wide, self.passage_wide_max)
    if center < center_min:
      return 'center={:.2f}<{:.2f}'.format(center, center_min)
    if min(left, right) > self.passage_side_max:
      return 'sides_open L={:.2f} R={:.2f}'.format(left, right)
    if abs(heading_err) > math.radians(85.0):
      return 'heading_err={:.0f}deg'.format(math.degrees(abs(heading_err)))
    return None

  def _maybe_log_passage_skip(
      self,
      reason: str,
      profile: Dict[str, float],
      heading_err: float,
  ) -> None:
    """Log near-miss passage checks (rate-limited)."""
    score = float(profile.get('passage_score', 0.0))
    wide = float(profile.get('wide', 0.0))
    if score < 0.25 and wide >= self.passage_wide_max + 0.06:
      return
    self._passage_debug_ticks += 1
    if self._passage_debug_ticks % 20 != 0:
      return
    center = float(profile.get('center', 0.0))
    left = float(profile.get('left', 99.0))
    right = float(profile.get('right', 99.0))
    rospy.loginfo(
        'Navigator: PassageMode not armed because %s '
        '(score=%.2f wide=%.2f center=%.2f L=%.2f R=%.2f head=%.0f°)',
        reason,
        score,
        wide,
        center,
        left,
        right,
        math.degrees(abs(heading_err)),
    )

  def _try_auto_arm_passage(
      self,
      profile: Dict[str, float],
      gx: float,
      gy: float,
      heading_err: float,
  ) -> bool:
    """正常导航阶段主动进入缝隙模式（不依赖 Recovery）。"""
    block = self._passage_arm_block_reason(profile, heading_err)
    if block is not None:
      self._passage_arm_streak = 0
      self._maybe_log_passage_skip(block, profile, heading_err)
      return False

    self._passage_arm_streak += 1
    if self._passage_arm_streak < self.corridor_commit_arm_confirm_frames:
      return False
    self._passage_arm_streak = 0

    score = float(profile.get('passage_score', 0.0))
    center = float(profile.get('center', 0.0))
    wide = float(profile.get('wide', 0.0))
    left = float(profile.get('left', 99.0))
    right = float(profile.get('right', 99.0))

    channel = self._is_block_wall_channel(profile)
    _, _, yaw = self.pose.get_pose()
    if channel:
      hold_yaw = self._channel_exit_yaw
      if (
          hold_yaw is not None
          and self._channel_exit_until is not None
          and rospy.Time.now() < self._channel_exit_until
      ):
        locked_yaw = hold_yaw
      else:
        locked_yaw = yaw
    else:
      locked_yaw = yaw
    self.arm_passage_mode(locked_yaw=locked_yaw, channel=channel)
    rospy.loginfo(
        'Navigator: PassageMode auto-arm score=%.2f wide=%.2f center=%.2f '
        'L=%.2f R=%.2f goal_yaw=%.1f° channel=%s',
        score,
        wide,
        center,
        left,
        right,
        math.degrees(locked_yaw),
        channel,
    )
    return True

  def _wide_blocks_forward(self, wide: float, center: float = 0.0) -> bool:
    if self._passage_active:
      return False
    if center >= self.wide_hold_override_center_min:
      return False
    return wide < self._wide_hold_dist()

  def _effective_path_clear(self, center: float, wide: float) -> float:
    """前方净空充足时，不因侧向 wide 偏低而限速。"""
    if center >= self.wide_hold_override_center_min:
      return center
    return min(center, wide)

  def _center_blocks_forward(self, center: float) -> bool:
    profile = self._last_profile
    if profile and self._is_block_wall_channel(profile):
      channel_min = self.boundary_dist * self.passage_channel_center_min_ratio
      return center < channel_min
    return center < self._forward_stop_dist()

  def _path_clearance(self, center: float, wide: float) -> float:
    return self._effective_path_clear(center, wide)

  def _drive_speed(self, center: float, wide: float, cruise_speed: float) -> float:
    if self._center_blocks_forward(center):
      return 0.0
    return self._speed_from_path_clear(
        self._effective_path_clear(center, wide),
        cruise_speed,
    )

  def _speed_from_path_clear(self, path_clear: float, cruise_speed: float) -> float:
    cruise_gate = self._cruise_clear_dist()
    stop_gate = self._forward_stop_dist()
    if path_clear < stop_gate:
      return 0.0
    if path_clear >= self.cruise_resume_center:
      return cruise_speed
    if path_clear >= cruise_gate:
      if path_clear > 0.38:
        return min(cruise_speed, 0.12)
      return self.creep_speed
    return self.creep_speed

  def _profile(self, scan: LaserScan) -> Dict[str, float]:
    profile = clearance_profile(
        scan,
        self.front_angle,
        self.nav_block_half_width,
        self.front_sector_half_width,
        self.boundary_dist,
    )
    self._last_profile = profile
    return profile

  def _apply_lateral_safety(
      self,
      profile: Dict[str, float],
      cmd_v: float,
  ) -> Tuple[float, float]:
    """侧向过近时降速并横移 — 一般障碍，不区分通道。"""
    # if self._passage_channel_mode:
    #   return self._apply_channel_lateral(profile, cmd_v)

    left = float(profile.get('left', 99.0))
    right = float(profile.get('right', 99.0))
    lateral_limit = self._effective_lateral_dist()
    strafe = 0.0
    strafe_gain = 0.65
    if left < lateral_limit:
      strafe -= self.creep_speed * strafe_gain
      cmd_v *= 0.65
    if right < lateral_limit:
      strafe += self.creep_speed * strafe_gain
      cmd_v *= 0.65
    return cmd_v, strafe

  def _apply_channel_lateral(
      self,
      profile: Dict[str, float],
      cmd_v: float,
  ) -> Tuple[float, float]:
    """块+墙通道：仅降速，禁止侧移顶物块。"""
    left = float(profile.get('left', 99.0))
    right = float(profile.get('right', 99.0))
    close = min(left, right)
    push_limit = self.passage_channel_close_max + self.robot_half_width * 0.35
    if close < push_limit:
      cmd_v *= max(0.30, close / max(0.05, push_limit))
    return cmd_v, 0.0

  def _publish_drive(self, cmd_v: float, w: float, linear_y: float = 0.0) -> None:
    self.move.publish_twist(linear_x=cmd_v, linear_y=linear_y, angular_z=w)
    if hasattr(self.pose, 'note_cmd_vel'):
      from geometry_msgs.msg import Twist
      cmd = Twist()
      cmd.linear.x = cmd_v
      cmd.linear.y = linear_y
      cmd.angular.z = w
      self.pose.note_cmd_vel(cmd)

  def _at_final_goal(self) -> bool:
    if self._final_goal is None:
      return True
    return self.pose.distance_to(self._final_goal[0], self._final_goal[1]) <= self.goal_tolerance

  def _update_feedback(
      self,
      tick_status: str,
      profile: Dict[str, float],
      heading_err: float,
  ) -> None:
    bd = self.boundary_dist
    center = float(profile.get('center', bd))
    wide = float(profile.get('wide', bd))
    left = float(profile.get('left', bd))
    right = float(profile.get('right', bd))

    center_c = self._linear_cost(center, bd + 0.08, bd * 0.55)
    wide_c = self._linear_cost(wide, bd + 0.06, bd * 0.50)
    head_c = self._clamp01(abs(heading_err) / math.radians(85.0))
    stress_c = self._stress_level / 5.0

    execution_cost = self._clamp01(
        0.38 * center_c + 0.27 * wide_c + 0.18 * head_c + 0.17 * stress_c,
    )

    reason = 'none'
    costs = (
        ('center', center_c),
        ('wide', wide_c),
        ('heading', head_c),
        ('stress', stress_c),
    )
    reason = max(costs, key=lambda item: item[1])[0]

    if tick_status == 'blocked':
      execution_cost = max(execution_cost, 0.92)
      reason = 'stress' if stress_c >= max(center_c, wide_c, head_c) else reason
    elif tick_status in ('nudge_5', 'nudge_10'):
      execution_cost = max(execution_cost, 0.75)
    elif tick_status == 'hold':
      reason = 'center'
    elif tick_status == 'align' and abs(heading_err) > self.drive_heading_tolerance:
      execution_cost = max(execution_cost, 0.45 + 0.35 * head_c)
      reason = 'heading'

    if tick_status == 'reached':
      goal_status = GoalStatus.REACHED
    elif tick_status == 'blocked':
      goal_status = GoalStatus.FAILED
    elif tick_status == 'odom_fault':
      goal_status = GoalStatus.ABORTED
    elif tick_status == 'idle':
      goal_status = GoalStatus.IDLE
    else:
      goal_status = GoalStatus.RUNNING

    self._feedback = NavExecutionFeedback(
        goal_status=goal_status,
        execution_cost=execution_cost,
        reason=reason,
        metrics={
            'center': center,
            'wide': wide,
            'left': left,
            'right': right,
            'stress_level': float(self._stress_level),
            'heading_err_deg': math.degrees(abs(heading_err)),
        },
    )

  def _simple_path_clear(self, center: float, wide: float) -> float:
    """一般障碍：取前方与宽扇区净空较小值，不区分通道/墙角。"""
    return min(center, wide)

  def _simple_drive_speed(
      self,
      path_clear: float,
      cruise_speed: float,
  ) -> float:
    """连续限速，避免状态切换时突然归零。"""
    stop_gate = self._forward_stop_dist()
    cruise_gate = self._cruise_clear_dist()
    if path_clear < stop_gate:
      ratio = max(0.0, (path_clear - self.boundary_dist * 0.45) / max(1e-6, stop_gate - self.boundary_dist * 0.45))
      return self.creep_speed * (0.35 + 0.65 * ratio)
    if path_clear >= self.cruise_resume_center:
      return cruise_speed
    if path_clear >= cruise_gate:
      t = (path_clear - cruise_gate) / max(1e-6, self.cruise_resume_center - cruise_gate)
      return self.creep_speed + t * (cruise_speed - self.creep_speed)
    t = (path_clear - stop_gate) / max(1e-6, cruise_gate - stop_gate)
    return self.creep_speed * (0.5 + 0.5 * t)

  def _handle_center_blocked(
      self,
      profile: Dict[str, float],
      heading_err: float,
  ) -> str:
    """一般避障：带速转向 + 侧移，连续帧确认后才上报 blocked。"""
    w_drive = max(
        -self.drive_max_w,
        min(self.drive_max_w, self.heading_kp * heading_err),
    )
    center = float(profile.get('center', 0.0))
    wide = float(profile.get('wide', 0.0))
    path_clear = self._simple_path_clear(center, wide)

    if self._center_close_streak >= self.boundary_confirm_frames * 4:
      self._center_close_streak = 0
      self.move.publish_stop_brief()
      self._update_feedback('blocked', profile, heading_err)
      return 'blocked'

    cmd_v = max(self.creep_speed * 0.55, self._simple_drive_speed(path_clear, self.creep_speed * 1.5))
    cmd_v, strafe = self._apply_lateral_safety(profile, cmd_v)
    self._publish_drive(cmd_v, w_drive * 0.85, linear_y=strafe)
    self._update_feedback('creep', profile, heading_err)
    return 'creep'

  # --- 20260706：已禁用 — 原多状态导航（缝隙、对准、停车、轻推）---
  # def _handle_center_blocked_legacy(...): ...
  # def _tick_passage_mode(...): 保留在下方供参考，不再调用

  def _tick_passage_mode(
      self,
      scan: LaserScan,
      cruise_speed: float,
      profile: Dict[str, float],
      gx: float,
      gy: float,
  ) -> str:
    """Corridor commit: fixed entry yaw until traverse distance or safety abort."""
    self._passage_ticks += 1
    center = float(profile.get('center', 0.0))
    _, _, yaw = self.pose.get_pose()
    heading_err = self._normalize_heading_error(self._commit_entry_yaw - yaw)
    forward_dist = self._commit_forward_dist()

    abort_reason = self._should_abort_corridor_commit(
        center,
        forward_dist,
        heading_err,
    )
    if abort_reason is not None:
      if abort_reason == 'center':
        rospy.logwarn(
            'Navigator: CorridorCommit abort center=%.2f dist=%.2f ticks=%d',
            center,
            forward_dist,
            self._passage_ticks,
        )
      else:
        rospy.logwarn(
            'Navigator: CorridorCommit abort %s center=%.2f dist=%.2f ticks=%d',
            abort_reason,
            center,
            forward_dist,
            self._passage_ticks,
        )
      self._release_corridor_commit(abort_reason)
      self._center_close_streak += 1
      return self._handle_center_blocked(profile, heading_err)

    if forward_dist >= self.corridor_commit_travel_m:
      return self._release_corridor_commit('traverse')

    w = max(
        -self.passage_max_heading_corr,
        min(self.passage_max_heading_corr, heading_err * 1.2),
    )
    cmd_v = self.creep_speed * self.passage_creep_factor
    if center < self._passage_min_center() and not self._passage_channel_mode:
      cmd_v *= 0.55
    cmd_v, strafe, err = self._apply_commit_centerline(profile, cmd_v)
    if self._passage_ticks == 1 or self._passage_ticks % 20 == 0:
      left = float(profile.get('left', 99.0))
      right = float(profile.get('right', 99.0))
      rospy.loginfo(
          'Navigator: CorridorCommit L=%.2f R=%.2f err=%.2f strafe=%.3f v=%.3f',
          left,
          right,
          err,
          strafe,
          cmd_v,
      )
    self._publish_drive(cmd_v, w, linear_y=strafe)
    self._update_feedback('passage', profile, heading_err)
    return 'passage'

  def tick(self, scan: LaserScan, cruise_speed: float) -> str:
    """20260706：统一一般避障；带速朝目标转向，按净空连续限速。"""
    if self._final_goal is None:
      self._feedback = NavExecutionFeedback()
      return 'idle'

    self._advance_waypoints()
    if self._goal is None:
      if self._at_final_goal():
        self.clear_goal()
        self.move.publish_stop_brief()
        self._feedback = NavExecutionFeedback(goal_status=GoalStatus.REACHED)
        return 'reached'
      self._goal = self._final_goal

    gx, gy = self._goal
    dist = self.pose.distance_to(gx, gy)
    at_waypoint = dist <= self.waypoint_tolerance
    at_final = self._at_final_goal()

    if at_final and (self._path_index >= len(self._path) or len(self._path) <= 1):
      self.clear_goal()
      self.move.publish_stop_brief()
      self._feedback = NavExecutionFeedback(goal_status=GoalStatus.REACHED)
      return 'reached'

    if at_waypoint and not at_final:
      self._advance_waypoints()
      if self._goal is None:
        self._goal = self._final_goal
      gx, gy = self._goal

    profile = self._profile(scan)
    center = float(profile['center'])
    wide = float(profile['wide'])

    if hasattr(self.pose, 'is_healthy') and not self.pose.is_healthy():
      self.move.publish_stop_brief()
      self._update_feedback('odom_fault', profile, 0.0)
      return 'odom_fault'

    heading_err = self._normalize_heading_error(self.pose.angle_to(gx, gy))
    dist_goal = self.pose.distance_to(gx, gy)
    path_clear = self._simple_path_clear(center, wide)
    stop_dist = self._forward_stop_dist()

    # 20260706：已禁用 — 缝隙、走廊、通道专用分支
    # if self._passage_active:
    #   return self._tick_passage_mode(scan, cruise_speed, profile, gx, gy)
    # if self._try_auto_arm_passage(profile, gx, gy, heading_err):
    #   return self._tick_passage_mode(scan, cruise_speed, profile, gx, gy)

    drive_tol = self._drive_heading_tolerance_for(dist_goal)
    w_drive = max(
        -self.drive_max_w,
        min(self.drive_max_w, self.heading_kp * heading_err),
    )

    if path_clear < stop_dist:
      self._center_close_streak += 1
      return self._handle_center_blocked(profile, heading_err)

    self._center_close_streak = max(0, self._center_close_streak - 1)
    self._stress_level = 0

    cmd_v = self._simple_drive_speed(path_clear, cruise_speed)
    cmd_v, strafe = self._apply_lateral_safety(profile, cmd_v)

    # 大偏角也带速转向，避免 align 原地停转
    min_arc_v = max(self.creep_speed, self.open_align_arc_speed * 0.85)
    if abs(heading_err) > drive_tol and cmd_v < min_arc_v:
      cmd_v = max(cmd_v, min_arc_v)

    self._publish_drive(cmd_v, w_drive, linear_y=strafe)
    if cmd_v >= cruise_speed * 0.55:
      self._update_feedback('drive', profile, heading_err)
      return 'drive'
    self._update_feedback('creep', profile, heading_err)
    return 'creep'

  # --- 20260706：已禁用 — 原 tick 多状态（对准、带速转向、停车、慢速、缝隙）---
  # def tick_legacy(self, scan, cruise_speed):
  #   ... 见 git 历史 ...
