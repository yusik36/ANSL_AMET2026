#!/usr/bin/env python3
"""Traffic-light detection for the start signal.

Subscribes /image_raw, looks for a lit lamp, and publishes which colour it is.

Publishes:
  traffic/light_state  (String)  -- one of "NONE", "RED", "GREEN"
  traffic/valid         (Bool)    -- true if a frame was processed recently
                                     (i.e. the camera/this node is alive).
                                     False only means "stale camera data",
                                     NOT "no light currently visible" -- see
                                     below.

The contest fixture has TWO lamps: red on top, green below. There is no
amber (confirmed with the user 2026-08-18), so this node does not look for
one. A practice rig that cycles through an amber phase will read as "NONE"
during it, which the pipeline treats as "go" -- fine here, because the real
fixture never shows it.

Design decision worth calling out (this was a judgment call, not something
confirmed with contest staff): the competition only penalizes *starting on
a red light*, not sitting still at a green one, and most of the track has
no traffic light in view at all. So this node distinguishes two different
kinds of "don't know":
  - camera/node dead (traffic/valid=False) -> judgment_node must fail safe
    and stop, because we genuinely can't tell if it's red.
  - camera alive, frame processed, but no lamp found this frame
    (traffic/light_state="NONE") -> judgment_node treats this as "go"
    (matches state after a green light, and matches the common case of a
    straightaway with no light in view -- the only case that has to force a
    stop is a positively-identified red light).
That scoping choice lives in physicar_judgment/judgment_node.py, not here;
this node only ever reports what it currently sees.

WHY THIS LOOKS FOR LAMP-SHAPED BLOBS RATHER THAN COUNTING PIXELS
An earlier version classified by whichever colour owned the most pixels in
the whole frame. That structurally cannot work: a genuine lit signal
measured 142-460 px on the practice rig, while the green track mat covered
~4900 px and a tan wood door ~3000-3300 px. The real light loses that
contest every time, and no saturation threshold fixes it for good -- it
only moves which background happens to win. So instead of trusting a
threshold to starve the background, this node uses two facts about what a
lamp physically is:
  - it is mounted above the ground, so only the upper part of the frame is
    searched (the track mat is on the floor and never enters the ROI);
  - it is small and round, so candidate blobs are bounded in area and must
    be reasonably circular (a door panel or a mat edge is neither).
Saturation/value thresholds are still applied, but as one filter among
several rather than the only line of defence.

ASYMMETRIC CONFIRMATION
Starting on red costs 10 s -- the largest single penalty in the rules, and
roughly half a good lap. Starting a beat late costs only the fraction of a
second it takes to notice. The debounce is therefore deliberately lopsided:
a RED reading is published quickly (fail safe fast), while GREEN has to
persist for longer before the car is allowed to move. For the same reason,
if both a red and a green lamp somehow qualify in one frame, RED wins.

Temporal smoothing here is not a source of statefulness that would break
the stateless-restart requirement -- there is no persistent identity or
track-position state, just "was the last N frames' answer the same".
"""
import math
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String
from cv_bridge import CvBridge, CvBridgeError

# Lamp candidates are bounded on BOTH sides. The lower bound rejects specular
# speckle; the upper bound is what actually kills the false positives, since
# every background surface that fooled the old version was an order of
# magnitude too big (measured 2026-08-18 on the practice rig).
MIN_BLOB_AREA_PX = 60
MAX_BLOB_AREA_PX = 2000

# 4*pi*area/perimeter^2 -- 1.0 for a perfect circle. A lit lamp stays well
# above this even when partly clipped; panels and mat edges do not.
MIN_CIRCULARITY = 0.45

# Fraction of the frame height, from the top, that is searched. The fixture
# is mounted above the road surface, so anything on the ground is out of
# scope by construction.
ROI_BOTTOM_FRAC = 0.65

CONFIRM_FRAMES_STOP = 1   # RED: react immediately
CONFIRM_FRAMES_GO = 3     # GREEN: make it prove itself before we move
IMAGE_STALE_S = 1.0

# OpenCV hue is 0-179. Red wraps around 0/179, so it needs two ranges.
RED1_H = (0, 8)
RED2_H = (170, 179)
GREEN_H = (45, 85)
SAT_MIN = 150
VAL_MIN = 100


class TrafficLightNode(Node):
    def __init__(self):
        super().__init__('traffic_light_node')

        self.declare_parameter('min_blob_area_px', MIN_BLOB_AREA_PX)
        self.declare_parameter('max_blob_area_px', MAX_BLOB_AREA_PX)
        self.declare_parameter('min_circularity', MIN_CIRCULARITY)
        self.declare_parameter('roi_bottom_frac', ROI_BOTTOM_FRAC)
        self.declare_parameter('confirm_frames_stop', CONFIRM_FRAMES_STOP)
        self.declare_parameter('confirm_frames_go', CONFIRM_FRAMES_GO)
        self.declare_parameter('sat_min', SAT_MIN)
        self.declare_parameter('val_min', VAL_MIN)

        self.min_blob_area_px = self.get_parameter('min_blob_area_px').value
        self.max_blob_area_px = self.get_parameter('max_blob_area_px').value
        self.min_circularity = self.get_parameter('min_circularity').value
        self.roi_bottom_frac = self.get_parameter('roi_bottom_frac').value
        self.confirm_frames_stop = self.get_parameter('confirm_frames_stop').value
        self.confirm_frames_go = self.get_parameter('confirm_frames_go').value
        self.sat_min = self.get_parameter('sat_min').value
        self.val_min = self.get_parameter('val_min').value

        self.bridge = CvBridge()
        self.state_pub = self.create_publisher(String, 'traffic/light_state', 10)
        self.valid_pub = self.create_publisher(Bool, 'traffic/valid', 10)
        # See lane_follow_node.py: sensor QoS avoids a DDS reliability
        # mismatch against a best-effort camera publisher (2026-08-18).
        self.create_subscription(Image, 'image_raw', self.on_image, qos_profile_sensor_data)

        self.last_image_time = 0.0
        self.published_state = 'NONE'
        self.candidate_state = 'NONE'
        self.candidate_count = 0
        self.create_timer(1.0 / 10.0, self.check_staleness)

        self.get_logger().info(
            f'traffic_light_node ready: RED/GREEN only (no amber on the contest '
            f'fixture), lamp area {self.min_blob_area_px}-{self.max_blob_area_px} px, '
            f'circularity>{self.min_circularity}, top {self.roi_bottom_frac:.0%} of frame'
        )

    def check_staleness(self):
        # Runs independently of on_image so "no frames arriving at all"
        # (camera/node dead) is still detected even though this node is
        # otherwise purely reactive to incoming images.
        now = time.time()
        valid = (now - self.last_image_time) < IMAGE_STALE_S
        self.valid_pub.publish(Bool(data=valid))

    def best_lamp_area(self, mask) -> int:
        """Largest blob in `mask` that is shaped like a lamp, 0 if none is.

        Returns the area so the caller can compare candidates, but the shape
        and size gates are what actually keep background surfaces out.
        """
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        best = 0
        for c in contours:
            area = cv2.contourArea(c)
            if area < self.min_blob_area_px or area > self.max_blob_area_px:
                continue
            perim = cv2.arcLength(c, True)
            if perim <= 0:
                continue
            circularity = 4.0 * math.pi * area / (perim * perim)
            if circularity < self.min_circularity:
                continue
            best = max(best, int(area))
        return best

    def classify(self, hsv) -> str:
        h, s, v = hsv[:, :, 0], hsv[:, :, 1], hsv[:, :, 2]
        lit = ((s >= self.sat_min) & (v >= self.val_min)).astype(np.uint8) * 255

        red = (((h >= RED1_H[0]) & (h <= RED1_H[1]))
               | ((h >= RED2_H[0]) & (h <= RED2_H[1]))).astype(np.uint8) * 255
        green = ((h >= GREEN_H[0]) & (h <= GREEN_H[1])).astype(np.uint8) * 255

        red_area = self.best_lamp_area(cv2.bitwise_and(red, lit))
        green_area = self.best_lamp_area(cv2.bitwise_and(green, lit))

        # Only one lamp is ever lit on the real fixture, so seeing both means
        # something is wrong (reflection, a stray red object). Take the
        # cautious reading: a false RED costs a moment, a false GREEN costs 10 s.
        if red_area > 0:
            return 'RED'
        if green_area > 0:
            return 'GREEN'
        return 'NONE'

    def on_image(self, msg: Image):
        self.last_image_time = time.time()
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as e:
            self.get_logger().warn(f'cv_bridge conversion failed, dropping frame: {e}')
            return

        height = frame.shape[0]
        roi = frame[0:max(1, int(height * self.roi_bottom_frac)), :]
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        reading = self.classify(hsv)

        if reading == self.candidate_state:
            self.candidate_count += 1
        else:
            self.candidate_state = reading
            self.candidate_count = 1

        # Moving off a stop is the expensive mistake, so anything that would
        # release the car has to hold for longer than anything that stops it.
        needed = (self.confirm_frames_go if reading == 'GREEN'
                  else self.confirm_frames_stop)
        if self.candidate_count >= needed and reading != self.published_state:
            self.published_state = reading

        self.state_pub.publish(String(data=self.published_state))


def main():
    rclpy.init()
    node = TrafficLightNode()
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
