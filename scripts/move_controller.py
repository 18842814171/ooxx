"""Chassis motion primitives — from competition_move_demo.py pattern."""

from __future__ import annotations

import math
import time
from typing import Callable, Optional

import rospy
from geometry_msgs.msg import Twist


class MoveController:
    def __init__(self, cmd_topic: str = '/controller/cmd_vel', rate_hz: int = 20):
        self.cmd_pub = rospy.Publisher(cmd_topic, Twist, queue_size=10)
        self.rate = rospy.Rate(rate_hz)
        self._last_twist = Twist()
        self._default_settle_sec = 0.08

    def get_last_twist(self) -> Twist:
        """Last cmd_vel published — used for dead reckoning when /odom is absent."""
        return self._last_twist

    def _record_and_publish(self, twist: Twist) -> None:
        self._last_twist = twist
        self.cmd_pub.publish(twist)

    def stop_robot(self, repeats: int = 1) -> None:
        """Publish zero velocity. Default once — safe inside 20Hz FSM tick."""
        twist = Twist()
        for _ in range(repeats):
            self._record_and_publish(twist)
            if repeats > 1:
                try:
                    self.rate.sleep()
                except rospy.ROSInterruptException:
                    break

    def publish_stop_once(self) -> None:
        """Alias for brief multi-frame stop — use in FSM tick handlers."""
        self.publish_stop_brief()

    def publish_stop_brief(self, repeats: int = 5) -> None:
        """Publish zero velocity several times in one tick to overcome control lag."""
        twist = Twist()
        for _ in range(repeats):
            self._record_and_publish(twist)

    def emergency_stop(self, repeats: int = 20) -> None:
        """Publish zero velocity; safe during Ctrl+C / rospy shutdown."""
        twist = Twist()
        for _ in range(repeats):
            self._record_and_publish(twist)
            time.sleep(0.05)

    def publish_twist(self, linear_x: float = 0.0, linear_y: float = 0.0, angular_z: float = 0.0) -> None:
        twist = Twist()
        twist.linear.x = linear_x
        twist.linear.y = linear_y
        twist.angular.z = angular_z
        self._record_and_publish(twist)

    def move_for(
        self,
        linear_x: float = 0.0,
        linear_y: float = 0.0,
        angular_z: float = 0.0,
        duration: float = 1.0,
        settle_sec: Optional[float] = None,
    ) -> None:
        start = rospy.Time.now()
        while not rospy.is_shutdown():
            if (rospy.Time.now() - start).to_sec() >= duration:
                break
            self.publish_twist(linear_x, linear_y, angular_z)
            self.rate.sleep()
        self.stop_robot(repeats=1)
        pause = self._default_settle_sec if settle_sec is None else settle_sec
        if pause > 0:
            rospy.sleep(pause)

    @staticmethod
    def _normalize_angle(angle: float) -> float:
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def rotate_angle(
        self,
        angle_deg: float,
        speed: float,
        get_yaw: Optional[Callable[[], float]] = None,
        settle_sec: Optional[float] = None,
    ) -> None:
        if angle_deg == 0:
            return
        if get_yaw is not None:
            self._rotate_angle_closed_loop(angle_deg, speed, get_yaw, settle_sec)
            return
        direction = 1.0 if angle_deg > 0 else -1.0
        duration = math.radians(abs(angle_deg)) / abs(speed)
        self.move_for(
            angular_z=direction * abs(speed),
            duration=duration,
            settle_sec=settle_sec,
        )

    def _rotate_angle_closed_loop(
        self,
        angle_deg: float,
        speed: float,
        get_yaw: Callable[[], float],
        settle_sec: Optional[float] = None,
    ) -> None:
        start_yaw = get_yaw()
        target = self._normalize_angle(start_yaw + math.radians(angle_deg))
        direction = 1.0 if angle_deg > 0 else -1.0
        w = direction * abs(speed)
        tol = math.radians(3.0)
        timeout = rospy.Time.now() + rospy.Duration(max(4.0, abs(angle_deg) / 30.0))
        while not rospy.is_shutdown():
            err = self._normalize_angle(target - get_yaw())
            if abs(err) <= tol:
                break
            if rospy.Time.now() > timeout:
                rospy.logwarn('rotate_angle closed-loop timeout (%.1f deg left)', math.degrees(err))
                break
            cmd_w = w if err * w > 0 else -w * 0.5
            self.publish_twist(angular_z=cmd_w)
            self.rate.sleep()
        self.stop_robot(repeats=1)
        pause = self._default_settle_sec if settle_sec is None else settle_sec
        if pause > 0:
            rospy.sleep(pause)

    def move_distance_x(
        self,
        distance: float,
        speed: float,
        settle_sec: Optional[float] = None,
    ) -> None:
        if distance == 0:
            return
        direction = 1.0 if distance > 0 else -1.0
        duration = abs(distance / speed)
        self.move_for(
            linear_x=direction * abs(speed),
            duration=duration,
            settle_sec=settle_sec,
        )

    def move_distance_y(self, distance: float, speed: float) -> None:
        if distance == 0:
            return
        direction = 1.0 if distance > 0 else -1.0
        duration = abs(distance / speed)
        self.move_for(linear_y=direction * abs(speed), duration=duration)

    def follow_wall(
        self,
        duration: float,
        forward_speed: float,
        turn_speed: float,
        turn_left: bool = True,
    ) -> None:
        angular = abs(turn_speed) if turn_left else -abs(turn_speed)
        self.move_for(linear_x=forward_speed, angular_z=angular, duration=duration)
