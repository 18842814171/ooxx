#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
目标检测器 — 对齐 search_navigation 相机处理方式：
  - Orbbec/Astra 输出 RGB，检测前转为 BGR
  - 回调只缓存图像，独立 run() 循环做检测
  - 发布 /target_current（ooxx 底盘读取）、/target_detected、/detection_image
"""

import math
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String


class TargetDetector:
    COLOR_LABELS = {
        'red': 'RED',
        'green': 'GREEN',
        'blue': 'BLUE',
        'None': 'None',
    }

    DRAW_COLORS_BGR = {
        'red': (0, 0, 255),
        'green': (0, 255, 0),
        'blue': (255, 0, 0),
        'black': (0, 0, 0),
    }

    COLOR_THRESHOLDS = {
        'red': {'min': [0, 150, 130], 'max': [255, 255, 255]},
        'green': {'min': [47, 0, 135], 'max': [255, 110, 255]},
        'blue': {'min': [0, 0, 0], 'max': [255, 136, 120]},
    }

    def __init__(self):
        rospy.init_node('target_detector', anonymous=False)

        self.image_width = rospy.get_param('~image_width', 320)
        self.image_height = rospy.get_param('~image_height', 240)
        self.min_area = rospy.get_param('~min_area', 200)
        self.confirm_frames = rospy.get_param('~confirm_frames', 3)
        self.detect_freq = rospy.get_param('~detect_freq', 10)
        self.show_debug = bool(rospy.get_param('~show_debug', False))

        self.rgb_image = None
        self.image_header = None
        self.color_history = []
        self.last_published_color = 'None'
        self._frame_count = 0
        self._warned_no_image = False
        self.bridge = CvBridge()

        self.detect_pub = rospy.Publisher('/target_detected', String, queue_size=10)
        self.current_pub = rospy.Publisher('/target_current', String, queue_size=1)
        self.image_pub = rospy.Publisher('/detection_image', Image, queue_size=10)

        camera_name = rospy.get_param('~depth_cam_name', 'astra_camera')
        self.topic_name = '/%s/rgb/image_raw' % camera_name
        rospy.Subscriber(self.topic_name, Image, self.image_callback, queue_size=1)

        self.rate = rospy.Rate(self.detect_freq)
        self._shutdown_done = False
        rospy.on_shutdown(self.shutdown)
        rospy.loginfo('目标检测器已启动 (RGB->BGR, 对齐 search_navigation)')
        rospy.loginfo('检测尺寸: %dx%d, 最小面积: %d', self.image_width, self.image_height, self.min_area)
        rospy.loginfo('订阅相机话题: %s', self.topic_name)
        if self.show_debug:
            rospy.loginfo('调试窗口已开启 (OpenCV imshow)')

    def image_callback(self, ros_image):
        if self._frame_count == 0:
            rospy.loginfo('收到第一帧图像 (%dx%d)', ros_image.width, ros_image.height)
        self._frame_count += 1
        self.rgb_image = np.ndarray(
            shape=(ros_image.height, ros_image.width, 3),
            dtype=np.uint8,
            buffer=ros_image.data,
        )
        self.image_header = ros_image.header

    @staticmethod
    def val_map(x, in_min, in_max, out_min, out_max):
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

    @staticmethod
    def get_area_max_contour(contours):
        contour_area_max = 0
        area_max_contour = None
        for c in contours:
            area = math.fabs(cv2.contourArea(c))
            if area > contour_area_max:
                contour_area_max = area
                if area > 50:
                    area_max_contour = c
        return area_max_contour, contour_area_max

    def detect_color(self, bgr_image):
        img_h, img_w = bgr_image.shape[:2]
        frame_resize = cv2.resize(
            bgr_image, (self.image_width, self.image_height), interpolation=cv2.INTER_NEAREST
        )
        frame_gb = cv2.GaussianBlur(frame_resize, (3, 3), 3)
        frame_lab = cv2.cvtColor(frame_gb, cv2.COLOR_BGR2LAB)

        max_area = 0
        color_area_max = None
        area_max_contour = None

        for color in ['red', 'green', 'blue']:
            th = self.COLOR_THRESHOLDS[color]
            mask = cv2.inRange(frame_lab, tuple(th['min']), tuple(th['max']))
            kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
            eroded = cv2.erode(mask, kernel)
            dilated = cv2.dilate(eroded, kernel)
            contours = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]
            contour_max, area = self.get_area_max_contour(contours)
            if contour_max is not None and area > max_area:
                max_area = area
                color_area_max = color
                area_max_contour = contour_max

        detect_color = 'None'
        if max_area > self.min_area and area_max_contour is not None:
            ((cx, cy), radius) = cv2.minEnclosingCircle(area_max_contour)
            cx = int(self.val_map(cx, 0, self.image_width, 0, img_w))
            cy = int(self.val_map(cy, 0, self.image_height, 0, img_h))
            r = int(self.val_map(radius, 0, self.image_width, 0, img_w))
            draw_bgr = self.DRAW_COLORS_BGR[color_area_max]
            cv2.circle(bgr_image, (cx, cy), r, draw_bgr, 2)
            label = self.COLOR_LABELS.get(color_area_max, '???')
            cv2.putText(
                bgr_image, label, (cx - 20, cy - r - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, draw_bgr, 2,
            )

            color_code = {'red': 1, 'green': 2, 'blue': 3}.get(color_area_max, 0)
            self.color_history.append(color_code)
            if len(self.color_history) >= self.confirm_frames:
                avg = int(round(np.mean(np.array(self.color_history))))
                rev = {1: 'red', 2: 'green', 3: 'blue'}
                detect_color = rev.get(avg, 'None')
                self.color_history = []
        else:
            self.color_history = []
            detect_color = 'None'

        draw_bgr = self.DRAW_COLORS_BGR.get(detect_color, (0, 0, 0))
        cv2.putText(
            bgr_image, 'Color: %s' % detect_color,
            (10, bgr_image.shape[0] - 10),
            cv2.FONT_HERSHEY_SIMPLEX, 0.65, draw_bgr, 2,
        )
        return bgr_image, detect_color

    def _publish_results(self, result_bgr, detected_color):
        self.current_pub.publish(detected_color)

        if self.image_header is not None:
            try:
                result_msg = self.bridge.cv2_to_imgmsg(result_bgr, 'bgr8')
                result_msg.header = self.image_header
                self.image_pub.publish(result_msg)
            except Exception as e:
                rospy.logerr('图像发布失败: %s', e)

        if detected_color != 'None' and detected_color != self.last_published_color:
            self.detect_pub.publish(detected_color)
            self.last_published_color = detected_color
            rospy.loginfo('检测到目标: %s', self.COLOR_LABELS.get(detected_color, detected_color))
        elif detected_color == 'None' and self.last_published_color != 'None':
            self.last_published_color = 'None'

    def shutdown(self):
        if self._shutdown_done:
            return
        self._shutdown_done = True
        rospy.loginfo('目标检测器退出，关闭调试窗口')
        cv2.destroyAllWindows()

    def run(self):
        rospy.loginfo('目标检测器运行中...')
        try:
            while not rospy.is_shutdown():
                if self.rgb_image is not None:
                    bgr = cv2.cvtColor(self.rgb_image, cv2.COLOR_RGB2BGR)
                    result_bgr, detected_color = self.detect_color(bgr.copy())
                    self._publish_results(result_bgr, detected_color)

                    if self.show_debug:
                        cv2.imshow('target_detector', result_bgr)
                        cv2.imshow('raw_camera', bgr)
                        key = cv2.waitKey(1)
                        if key == ord('q'):
                            rospy.loginfo('用户按 q 关闭调试窗口')
                            self.show_debug = False
                            cv2.destroyAllWindows()

                    self.rate.sleep()
                else:
                    if not self._warned_no_image:
                        rospy.logwarn('未收到图像，请确认相机已启动，话题: %s', self.topic_name)
                        self._warned_no_image = True
                    rospy.sleep(0.1)
        except rospy.ROSInterruptException:
            pass
        except Exception as e:
            rospy.logerr('目标检测器异常: %s', e)
        finally:
            self.shutdown()


if __name__ == '__main__':
    detector = TargetDetector()
    try:
        detector.run()
    finally:
        detector.shutdown()
