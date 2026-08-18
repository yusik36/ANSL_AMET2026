#!/usr/bin/env python3
"""Camera-based lane/track-boundary following.

Subscribes /image_raw, thresholds a bottom ROI in HSV to segment the track
surface markings, and estimates a steering correction from how far the
segmented blob's centroid is offset from the image's horizontal center.

Publishes:
  lane/steering  (Float64, rad)  -- only meaningful when lane/valid is true
  lane/valid     (Bool)          -- true if enough track-marking pixels were
                                     found in this frame to trust the offset

FIXME -- the single biggest unknown in this node: the competition track's
color/markings are NOT known as of this writing (2026-08-15). The course
spec + preliminary obstacle layout are only released 2026-08-18, and even
then the real (main-round) track differs from whatever gets revealed then
(see README "트랙 일반화" section). So this node deliberately does NOT
hardcode a color -- HSV_*_MIN/MAX below are placeholder defaults (tuned for
"a bright/light-colored line on a darker floor", the most common convention
for taped-down track boundaries) exposed as ROS2 parameters so they can be
retuned from the field without touching code. Use hsv_calibrate_node to read
off real values once you can point the camera at the actual track surface.

FIXME -- LANE_STEER_SIGN follows the same convention as physicar_nav's
AVOID_STEER_SIGN: assumes a positive centroid offset (blob center right of
image center) means the tracked line/lane is to the car's right and the car
should steer right (negative /steering, per REP-103/105 where CCW/left is
positive). Flip to -1.0 if the car steers away from the line instead of
toward centering it.
"""
import math

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float64
from cv_bridge import CvBridge, CvBridgeError

# ROI: fraction of image height, measured from the top, that we look at.
# Bottom of the frame = ground closest to the car, which is what matters for
# an immediate steering correction.
ROI_TOP_FRAC = 0.6
ROI_BOTTOM_FRAC = 1.0

# HSV thresholds. OpenCV hue range is 0-179. All six bounds are parameters so
# any marking colour can be targeted without code changes -- a low-saturation
# window (s_min=0, s_max~80, v_min high) tracks a white/grey line, a
# high-saturation window (s_min high) tracks a strongly coloured one.
#
# Defaults below are MEASURED off the simulator's "AMET 2026" track on
# 2026-08-18: its centre dashes are strong orange, sampled at H=18 S=247 V=227
# against a dark grey road surface. The window is padded generously around
# that so it survives lighting differences.
#
# FIXME: the main-round track is only revealed on race day and may not use
# orange dashes at all (see README "트랙 일반화"). Re-measure with
# hsv_calibrate_node on 2026-08-25 and override via launch/params rather than
# assuming these carry over.
HSV_H_MIN = 5
HSV_H_MAX = 35
HSV_S_MIN = 120         # high saturation = a strongly coloured marking
HSV_S_MAX = 255
HSV_V_MIN = 120
HSV_V_MAX = 255

MIN_VALID_AREA_PX = 300  # below this, treat the frame as "lane not found"
MAX_STEER_DEG = 20.0      # mirrors the real driver's hard clamp, belt-and-suspenders
LANE_STEER_SIGN = 1.0     # FIXME: flip to -1.0 if steering direction is backwards
MORPH_KERNEL = 5


class LaneFollowNode(Node):
    def __init__(self):
        super().__init__('lane_follow_node')

        self.declare_parameter('h_min', HSV_H_MIN)
        self.declare_parameter('h_max', HSV_H_MAX)
        self.declare_parameter('s_min', HSV_S_MIN)
        self.declare_parameter('s_max', HSV_S_MAX)
        self.declare_parameter('v_min', HSV_V_MIN)
        self.declare_parameter('v_max', HSV_V_MAX)
        self.declare_parameter('roi_top_frac', ROI_TOP_FRAC)
        self.declare_parameter('roi_bottom_frac', ROI_BOTTOM_FRAC)
        self.declare_parameter('min_valid_area_px', MIN_VALID_AREA_PX)
        self.declare_parameter('max_steer_deg', MAX_STEER_DEG)
        self.declare_parameter('lane_steer_sign', LANE_STEER_SIGN)

        self.h_min = self.get_parameter('h_min').value
        self.h_max = self.get_parameter('h_max').value
        self.s_min = self.get_parameter('s_min').value
        self.s_max = self.get_parameter('s_max').value
        self.v_min = self.get_parameter('v_min').value
        self.v_max = self.get_parameter('v_max').value
        self.roi_top_frac = self.get_parameter('roi_top_frac').value
        self.roi_bottom_frac = self.get_parameter('roi_bottom_frac').value
        self.min_valid_area_px = self.get_parameter('min_valid_area_px').value
        self.max_steer_rad = math.radians(self.get_parameter('max_steer_deg').value)
        self.lane_steer_sign = self.get_parameter('lane_steer_sign').value

        self.bridge = CvBridge()
        self.steering_pub = self.create_publisher(Float64, 'lane/steering', 10)
        self.valid_pub = self.create_publisher(Bool, 'lane/valid', 10)
        # Real Physicar's camera driver is expected to publish sensor QoS
        # (best-effort); a default-QoS subscriber would be DDS-incompatible
        # with that and silently receive nothing (2026-08-18).
        self.create_subscription(Image, 'image_raw', self.on_image, qos_profile_sensor_data)

        self.get_logger().info(
            f'lane_follow_node ready: HSV H[{self.h_min}-{self.h_max}] '
            f'S[{self.s_min}-{self.s_max}] V[{self.v_min}-{self.v_max}] -- '
            f'defaults measured off the simulator track, re-measure with '
            f'hsv_calibrate_node on the real venue'
        )

    def on_image(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as e:
            self.get_logger().warn(f'cv_bridge conversion failed, dropping frame: {e}')
            return

        h, w = frame.shape[:2]
        y0 = int(h * self.roi_top_frac)
        y1 = int(h * self.roi_bottom_frac)
        roi = frame[y0:y1, :]
        if roi.size == 0:
            self.publish(0.0, False)
            return

        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        lower = np.array([self.h_min, self.s_min, self.v_min], dtype=np.uint8)
        upper = np.array([self.h_max, self.s_max, self.v_max], dtype=np.uint8)
        mask = cv2.inRange(hsv, lower, upper)

        kernel = np.ones((MORPH_KERNEL, MORPH_KERNEL), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            self.publish(0.0, False)
            return

        largest = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(largest)
        if area < self.min_valid_area_px:
            self.publish(0.0, False)
            return

        moments = cv2.moments(largest)
        if moments['m00'] == 0:
            self.publish(0.0, False)
            return

        cx = moments['m10'] / moments['m00']
        roi_w = roi.shape[1]
        # -1.0 (centroid at left edge) .. +1.0 (centroid at right edge)
        offset = (cx - roi_w / 2.0) / (roi_w / 2.0)
        offset = max(-1.0, min(1.0, offset))

        steering = -offset * self.max_steer_rad * self.lane_steer_sign
        self.publish(steering, True)

    def publish(self, steering: float, valid: bool):
        self.steering_pub.publish(Float64(data=steering))
        self.valid_pub.publish(Bool(data=valid))


def main():
    rclpy.init()
    node = LaneFollowNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
