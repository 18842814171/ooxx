#!/usr/bin/env python3
# encoding: utf-8
"""Mission search node — ooxx_old chassis + search_navigation camera."""

from __future__ import print_function

import threading
from typing import Optional

import math
import sys
import os
_DIR = os.path.dirname(os.path.abspath(__file__))
if _DIR not in sys.path:
    sys.path.insert(0, _DIR)

import rospy
from sensor_msgs.msg import LaserScan

from config_loader import load_mission_config
from mission_state import MissionState
from move_controller import MoveController
from perception.classical import create_perception_backend
from pose_estimator import PoseEstimator
from search_fsm import SearchFSM, SearchState
from tracking.stub import create_tracker_backend


class MissionSearchNode:
    def __init__(self):
        rospy.init_node('ooxx')

        config_path = rospy.get_param('~mission_config')
        self.mission = load_mission_config(config_path)
        self.loop_hz = int(rospy.get_param('~loop_hz', 20))

        self._scan_lock = threading.RLock()
        self._latest_scan = None
        self._warned_no_scan = False

        self.mission_state = MissionState(
            self.mission.required_counts(),
            dedup_world_dist=self.mission.vision.dedup_world_dist,
        )

        cmd_topic = rospy.get_param('~cmd_vel_topic', '/controller/cmd_vel')
        self.move = MoveController(cmd_topic=cmd_topic, rate_hz=self.loop_hz)
        self.perception = create_perception_backend(self.mission)
        if hasattr(self.perception, 'set_mission_state'):
            self.perception.set_mission_state(self.mission_state)

        self.pose = PoseEstimator(
            odom_topic=self.mission.map.odom_topic,
            use_odom=self.mission.map.use_odom,
        )
        self.tracker = create_tracker_backend(self.mission.tracking.backend)

        self.fsm = SearchFSM(
            config=self.mission,
            move=self.move,
            perception=self.perception,
            mission_state=self.mission_state,
            pose=self.pose,
            get_scan=self.get_scan,
            get_image=lambda: None,
            get_depth=lambda: None,
            get_camera_info=lambda: None,
        )

        scan_topic = rospy.get_param('~scan_topic', '/scan')
        rospy.Subscriber(scan_topic, LaserScan, self._scan_callback, queue_size=1)

        self._shutdown_done = False
        self._status_interval = max(1, self.loop_hz // 2)
        self._tick_count = 0
        rospy.on_shutdown(self.shutdown)

        rospy.loginfo('Mission search | mode=%s | config=%s', self.mission.search.mode, config_path)
        rospy.loginfo('Targets: %s', self.mission.required_counts())
        rospy.loginfo(
            'Vision backend: %s (camera via target_detector node)',
            self.mission.vision.backend,
        )

    def _scan_callback(self, msg: LaserScan) -> None:
        with self._scan_lock:
            self._latest_scan = msg

    def get_scan(self):
        with self._scan_lock:
            return self._latest_scan

    def shutdown(self) -> None:
        if self._shutdown_done:
            return
        self._shutdown_done = True
        rospy.loginfo('收到退出信号 (Ctrl+C)，正在停车...')
        self.move.emergency_stop(repeats=20)

    def run(self) -> None:
        rate = rospy.Rate(self.loop_hz)
        cmd_topic = rospy.get_param('~cmd_vel_topic', '/controller/cmd_vel')
        while not rospy.is_shutdown():
            if self.get_scan() is None and not self._warned_no_scan:
                scan_topic = rospy.get_param('~scan_topic', '/scan')
                rospy.logwarn('未收到雷达数据，请确认 %s 有输出（可先 roslaunch lidar lidar_filter.launch）', scan_topic)
                self._warned_no_scan = True

            state = self.fsm.tick()
            self._tick_count += 1

            if self._tick_count % self._status_interval == 0:
                has_scan = self._latest_scan is not None
                has_odom = self.pose.has_odom()
                rx, ry, ryaw = self.pose.get_pose()
                rospy.loginfo(
                    'ooxx status: state=%s scan=%s odom=%s pose=(%.2f,%.2f,yaw=%.0f) cmd=%s',
                    state.value,
                    has_scan,
                    has_odom,
                    rx, ry, math.degrees(ryaw),
                    cmd_topic,
                )

            if state == SearchState.DONE:
                self.move.stop_robot()
                rospy.loginfo('Final: %s', self.mission_state.status_line())
                for rec in self.mission_state.target_records:
                    rospy.loginfo('  #%d %s/%s', rec.object_id, rec.color, rec.shape)
                break

            rate.sleep()


def main():
    node = None
    try:
        node = MissionSearchNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
    finally:
        if node is not None:
            node.shutdown()


if __name__ == '__main__':
    main()
