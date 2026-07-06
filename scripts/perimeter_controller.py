"""Perimeter mapping controller — state machine for wall follow / lost wall / corner."""

from __future__ import annotations

import enum
import math
from typing import Dict, Tuple

import rospy


class PerimeterState(enum.Enum):
  SEARCH_WALL = 'search_wall'
  FOLLOW_WALL = 'follow_wall'
  LOST_WALL = 'lost_wall'
  TURN_CORNER = 'turn_corner'
  CORRIDOR = 'corridor'


class PerimeterController:
  """Follow arena boundary during global perimeter phase."""

  def __init__(
      self,
      wall_side: str = 'right',
      wall_target_dist: float = 0.25,
      lost_wall_dist: float = 0.80,
      found_wall_dist: float = 0.65,
      creep_speed: float = 0.06,
      forward_stop_dist: float = 0.42,
      lateral_block_dist: float = 0.25,
      robot_half_width: float = 0.18,
      corner_stuck_ticks: int = 25,
      corridor_center_min: float = 0.35,
  ):
    self.wall_side = wall_side
    self.wall_target_dist = wall_target_dist
    self.lost_wall_dist = lost_wall_dist
    self.found_wall_dist = found_wall_dist
    self.creep_speed = creep_speed
    self.forward_stop_dist = forward_stop_dist
    self.lateral_limit = lateral_block_dist + robot_half_width
    self.corner_stuck_ticks = corner_stuck_ticks
    self.corridor_center_min = corridor_center_min
    self._state = PerimeterState.SEARCH_WALL
    self._hold_yaw: float = 0.0
    self._corner_ticks = 0
    self._stuck_ticks = 0
    self._last_pose: Tuple[float, float] = (0.0, 0.0)
    self._log_ticks = 0

  def reset(self) -> None:
    self._state = PerimeterState.SEARCH_WALL
    self._hold_yaw = 0.0
    self._corner_ticks = 0
    self._stuck_ticks = 0
    self._last_pose = (0.0, 0.0)
    self._log_ticks = 0

  def state(self) -> PerimeterState:
    return self._state

  def _wall_dist(self, profile: Dict[str, float]) -> float:
    if self.wall_side == 'right':
      return float(profile.get('right', 99.0))
    return float(profile.get('left', 99.0))

  def _open_side_sign(self, profile: Dict[str, float]) -> float:
    left = float(profile.get('left', 99.0))
    right = float(profile.get('right', 99.0))
    if left >= right:
      return 1.0
    return -1.0

  def _speed_for_center(self, center: float, cruise_speed: float) -> float:
    if center > 1.2:
      return cruise_speed
    if center > 0.7:
      return 0.12
    return self.creep_speed

  def _lateral_strafe(self, profile: Dict[str, float], gain: float = 0.5) -> float:
    left = float(profile.get('left', 99.0))
    right = float(profile.get('right', 99.0))
    strafe = 0.0
    vy = self.creep_speed * gain
    if left < self.lateral_limit:
      strafe -= vy
    if right < self.lateral_limit:
      strafe += vy
    return strafe

  def _is_channel(self, profile: Dict[str, float]) -> bool:
    left = float(profile.get('left', 99.0))
    right = float(profile.get('right', 99.0))
    center = float(profile.get('center', 0.0))
    if center < self.corridor_center_min:
      return False
    close_max = 0.55
    open_min = 0.75
    left_close = left < close_max
    right_close = right < close_max
    if left_close and not right_close and right >= open_min:
      return True
    if right_close and not left_close and left >= open_min:
      return True
    return False

  def _note_motion(self, pose_x: float, pose_y: float) -> float:
    dx = pose_x - self._last_pose[0]
    dy = pose_y - self._last_pose[1]
    self._last_pose = (pose_x, pose_y)
    return math.hypot(dx, dy)

  def tick(
      self,
      profile: Dict[str, float],
      yaw: float,
      pose_x: float,
      pose_y: float,
      cruise_speed: float,
  ) -> Tuple[float, float, float, str, bool]:
    """Return cmd_v, w, strafe, state_name, needs_recovery."""
    center = float(profile.get('center', 0.0))
    wall_dist = self._wall_dist(profile)
    wall_found = wall_dist < self.found_wall_dist
    wall_lost = wall_dist > self.lost_wall_dist
    front_blocked = center < self.forward_stop_dist
    cmd_v = self._speed_for_center(center, cruise_speed)
    w = 0.0
    strafe = 0.0
    needs_recovery = False

    if self._is_channel(profile) and not front_blocked:
      self._state = PerimeterState.CORRIDOR
    elif self._state == PerimeterState.CORRIDOR and not self._is_channel(profile):
      self._state = PerimeterState.FOLLOW_WALL if wall_found else PerimeterState.LOST_WALL

    if self._state == PerimeterState.SEARCH_WALL:
      self._hold_yaw = yaw
      if wall_found:
        self._state = PerimeterState.FOLLOW_WALL
        wall_err = wall_dist - self.wall_target_dist
        w = max(-0.12, min(0.12, wall_err * 1.8))
        strafe = self._lateral_strafe(profile)
      else:
        cmd_v = max(cmd_v, self.creep_speed)
        w = 0.04 * self._open_side_sign(profile)

    elif self._state == PerimeterState.LOST_WALL:
      heading_err = self._normalize_angle(self._hold_yaw - yaw)
      w = max(-0.05, min(0.05, heading_err * 0.8))
      cmd_v = max(cmd_v, self.creep_speed * 0.9)
      if wall_found:
        self._state = PerimeterState.FOLLOW_WALL
        self._corner_ticks = 0
        wall_err = wall_dist - self.wall_target_dist
        w = max(-0.12, min(0.12, wall_err * 1.8))
        strafe = self._lateral_strafe(profile)

    elif self._state == PerimeterState.FOLLOW_WALL:
      if wall_lost:
        self._state = PerimeterState.LOST_WALL
        self._hold_yaw = yaw
        heading_err = self._normalize_angle(self._hold_yaw - yaw)
        w = max(-0.05, min(0.05, heading_err * 0.8))
        cmd_v = max(cmd_v, self.creep_speed * 0.9)
      elif front_blocked:
        self._state = PerimeterState.TURN_CORNER
        self._corner_ticks = 0
        cmd_v = 0.0
        strafe = 0.0
        w = 0.10 * self._open_side_sign(profile)
      else:
        wall_err = wall_dist - self.wall_target_dist
        w = max(-0.12, min(0.12, wall_err * 1.8))
        strafe = self._lateral_strafe(profile)

    elif self._state == PerimeterState.TURN_CORNER:
      cmd_v = 0.0
      strafe = 0.0
      w = 0.10 * self._open_side_sign(profile)
      self._corner_ticks += 1
      disp = self._note_motion(pose_x, pose_y)
      if disp < 0.02:
        self._stuck_ticks += 1
      else:
        self._stuck_ticks = 0
      if not front_blocked and wall_found:
        self._state = PerimeterState.FOLLOW_WALL
        self._corner_ticks = 0
        self._stuck_ticks = 0
        wall_err = wall_dist - self.wall_target_dist
        w = max(-0.12, min(0.12, wall_err * 1.8))
        strafe = self._lateral_strafe(profile)
        cmd_v = self._speed_for_center(center, cruise_speed)
      elif self._corner_ticks >= self.corner_stuck_ticks or self._stuck_ticks >= 15:
        needs_recovery = True
        self._corner_ticks = 0
        self._stuck_ticks = 0

    elif self._state == PerimeterState.CORRIDOR:
      self._hold_yaw = yaw
      cmd_v = self.creep_speed
      w = 0.0
      strafe = 0.0

    if front_blocked and cmd_v > 0.0:
      cmd_v = 0.0
    if cmd_v <= 0.0:
      strafe = 0.0

    self._log_ticks += 1
    if self._log_ticks % 20 == 0:
      rospy.loginfo(
          'PerimeterCtrl: state=%s wall=%s dist=%.2f center=%.2f v=%.3f w=%.3f',
          self._state.value,
          self.wall_side,
          wall_dist,
          center,
          cmd_v,
          w,
      )

    return cmd_v, w, strafe, self._state.value, needs_recovery

  @staticmethod
  def _normalize_angle(angle: float) -> float:
    while angle > math.pi:
      angle -= 2.0 * math.pi
    while angle < -math.pi:
      angle += 2.0 * math.pi
    return angle
