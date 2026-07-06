"""Robot pose from /odom with cmd_vel dead-reckoning fallback."""

from __future__ import annotations

import math
import threading
from typing import Optional, Tuple

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


class PoseEstimator:
  def __init__(self, odom_topic: str = '/odom', use_odom: bool = True):
    self._lock = threading.RLock()
    self._x = 0.0
    self._y = 0.0
    self._yaw = 0.0
    self._has_odom = False
    self._last_update = rospy.Time.now()
    self._use_odom = use_odom
    self._health_window = 2.5
    self._min_linear_cmd = 0.02
    self._min_angular_cmd = 0.05
    self._min_pos_change = 0.025
    self._min_yaw_change_drive = math.radians(3.0)
    self._min_yaw_change_spin = math.radians(8.0)
    self._watch_start: Optional[float] = None
    self._x_at_watch = 0.0
    self._y_at_watch = 0.0
    self._yaw_at_watch = 0.0
    self._last_linear_cmd = 0.0
    self._last_angular_cmd = 0.0
    self._watch_mode = 'idle'

    if use_odom:
      rospy.Subscriber(odom_topic, Odometry, self._odom_cb, queue_size=5)

  def _odom_cb(self, msg: Odometry) -> None:
    with self._lock:
      self._x = msg.pose.pose.position.x
      self._y = msg.pose.pose.position.y
      q = msg.pose.pose.orientation
      siny = 2.0 * (q.w * q.z + q.x * q.y)
      cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
      self._yaw = math.atan2(siny, cosy)
      self._has_odom = True
      self._last_update = msg.header.stamp

  def update_from_twist(self, twist: Twist, dt: float) -> None:
    """Dead reckoning when odom is unavailable."""
    if self._use_odom and self._has_odom:
      return
    with self._lock:
      vx = twist.linear.x
      vy = twist.linear.y
      wz = twist.angular.z
      cy = math.cos(self._yaw)
      sy = math.sin(self._yaw)
      self._x += (vx * cy - vy * sy) * dt
      self._y += (vx * sy + vy * cy) * dt
      self._yaw = self._normalize_angle(self._yaw + wz * dt)

  def get_pose(self) -> Tuple[float, float, float]:
    with self._lock:
      return self._x, self._y, self._normalize_angle(self._yaw)

  def has_odom(self) -> bool:
    with self._lock:
      return self._has_odom

  def _cmd_mode(self, linear: float, angular: float) -> str:
    if linear >= self._min_linear_cmd:
      return 'drive'
    if angular >= self._min_angular_cmd:
      return 'spin'
    return 'idle'

  def note_cmd_vel(self, twist: Twist) -> None:
    """Track commanded twist; health checks depend on drive vs spin."""
    with self._lock:
      linear = abs(twist.linear.x)
      angular = abs(twist.angular.z)
      mode = self._cmd_mode(linear, angular)
      if mode == 'idle':
        self._watch_start = None
        self._watch_mode = 'idle'
        self._last_linear_cmd = linear
        self._last_angular_cmd = angular
        return

      now = rospy.Time.now().to_sec()
      if self._watch_start is None or mode != self._watch_mode:
        self._watch_start = now
        self._watch_mode = mode
        self._x_at_watch = self._x
        self._y_at_watch = self._y
        self._yaw_at_watch = self._yaw

      self._last_linear_cmd = linear
      self._last_angular_cmd = angular

  def is_healthy(self) -> bool:
    """False when commanded motion is not reflected in odom (mode-aware)."""
    with self._lock:
      if not self._use_odom or not self._has_odom:
        return True
      if self._watch_start is None or self._watch_mode == 'idle':
        return True

      elapsed = rospy.Time.now().to_sec() - self._watch_start
      if elapsed < self._health_window:
        return True

      if self._watch_mode == 'drive':
        pos_delta = math.hypot(self._x - self._x_at_watch, self._y - self._y_at_watch)
        if self._last_linear_cmd >= self._min_linear_cmd:
          return pos_delta >= self._min_pos_change
        yaw_delta = abs(self._normalize_angle(self._yaw - self._yaw_at_watch))
        return yaw_delta >= self._min_yaw_change_drive

      if self._watch_mode == 'spin':
        yaw_delta = abs(self._normalize_angle(self._yaw - self._yaw_at_watch))
        return yaw_delta >= self._min_yaw_change_spin

      return True

  def reset_health_watch(self) -> None:
    with self._lock:
      self._watch_start = None
      self._watch_mode = 'idle'

  def _normalize_angle(self, angle: float) -> float:
    while angle > math.pi:
      angle -= 2.0 * math.pi
    while angle < -math.pi:
      angle += 2.0 * math.pi
    return angle

  def distance_to(self, gx: float, gy: float) -> float:
    x, y, _ = self.get_pose()
    return math.hypot(gx - x, gy - y)

  def angle_to(self, gx: float, gy: float) -> float:
    x, y, yaw = self.get_pose()
    bearing = math.atan2(gy - y, gx - x)
    return self._normalize_angle(bearing - yaw)
