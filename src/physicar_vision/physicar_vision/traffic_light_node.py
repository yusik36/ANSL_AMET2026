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

The contest fixture is a small signal BOX standing on the ground, not an
overhead road signal. It has two lamps, red on top and green below -- no
amber (confirmed with the user 2026-08-18), so this node does not look for
one. A practice rig that cycles through an amber phase will read as "NONE"
during it, which the pipeline treats as "go" -- fine here, because the real
fixture never shows it.

It is placed ahead and to the RIGHT of the start position: roughly the four
o'clock direction in the camera frame, i.e. lower right, not up in the sky.
That placement is what the search window below is built around.

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
threshold to starve the background, this node uses two facts about what
this lamp physically is:
  - it sits in a known direction from the start line, so only a window
    around the four o'clock region is searched. Grass beside the track and
    anything on the left never enters the search at all;
  - it is a small compact lamp, so candidates are bounded in area at both
    ends and must be reasonably round (a square LED panel scores ~0.79 and
    still passes; a grass verge or a mat edge does not).
Saturation/value thresholds are still applied, but as one filter among
several rather than the only line of defence.

CALIBRATE THE SEARCH WINDOW BEFORE RACE DAY. The defaults cover a generous
lower-right region because the exact placement is only known approximately
("about four o'clock"). Run with debug_candidates:=true, look at where the
blobs are actually reported, and tighten roi_* to match. A window that
misses the lamp reads NONE, which the pipeline treats as GO -- exactly the
10 s mistake this node exists to prevent.

KNOWN WEAKNESS, worth a minute of checking on site: the green lamp sits low
and there may be grass behind it. If a same-hue background passes the
saturation gate, it merges with the lamp into one contour, and the combined
blob is then rejected for being too large -- a green light silently reading
as NONE. What keeps them apart is that the lamp emits and the background
only reflects: measured on the practice rig the lamp reached S 255 while no
background exceeded ~160. So point the camera at the lit green lamp with
debug_candidates on and confirm it is reported as its own blob of a
plausible size. If it is not, raise sat_min (or val_min) until it separates
-- do not widen max_blob_area_px to swallow the merged region.

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
# magnitude too big (measured 2026-08-18 on the practice rig). The upper
# bound is generous because the box stands close to the car and its lamp is
# correspondingly large in frame -- measure it and tighten.
MIN_BLOB_AREA_PX = 60
MAX_BLOB_AREA_PX = 6000

# 4*pi*area/perimeter^2 -- 1.0 for a perfect circle, ~0.79 for a square, so a
# square LED panel passes comfortably. Grass verges and panel edges do not.
MIN_CIRCULARITY = 0.45

# Search window as fractions of the frame, covering the four o'clock region
# where the signal box stands. Deliberately generous: the placement is only
# known approximately, and missing the lamp is the expensive failure.
# UNVERIFIED against a real frame -- calibrate with debug_candidates.
ROI_X_MIN_FRAC = 0.45
ROI_X_MAX_FRAC = 1.0
ROI_Y_MIN_FRAC = 0.25
ROI_Y_MAX_FRAC = 1.0

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
        self.declare_parameter('roi_x_min_frac', ROI_X_MIN_FRAC)
        self.declare_parameter('roi_x_max_frac', ROI_X_MAX_FRAC)
        self.declare_parameter('roi_y_min_frac', ROI_Y_MIN_FRAC)
        self.declare_parameter('roi_y_max_frac', ROI_Y_MAX_FRAC)
        self.declare_parameter('debug_candidates', False)
        self.declare_parameter('confirm_frames_stop', CONFIRM_FRAMES_STOP)
        self.declare_parameter('confirm_frames_go', CONFIRM_FRAMES_GO)
        self.declare_parameter('sat_min', SAT_MIN)
        self.declare_parameter('val_min', VAL_MIN)

        self.min_blob_area_px = self.get_parameter('min_blob_area_px').value
        self.max_blob_area_px = self.get_parameter('max_blob_area_px').value
        self.min_circularity = self.get_parameter('min_circularity').value
        self.roi_x_min_frac = self.get_parameter('roi_x_min_frac').value
        self.roi_x_max_frac = self.get_parameter('roi_x_max_frac').value
        self.roi_y_min_frac = self.get_parameter('roi_y_min_frac').value
        self.roi_y_max_frac = self.get_parameter('roi_y_max_frac').value
        self.debug_candidates = self.get_parameter('debug_candidates').value
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
            f'circularity>{self.min_circularity}, searching x '
            f'{self.roi_x_min_frac:.0%}-{self.roi_x_max_frac:.0%} y '
            f'{self.roi_y_min_frac:.0%}-{self.roi_y_max_frac:.0%} of frame '
            f'(the signal box stands at about four o\'clock)'
        )

    def check_staleness(self):
        # Runs independently of on_image so "no frames arriving at all"
        # (camera/node dead) is still detected even though this node is
        # otherwise purely reactive to incoming images.
        now = time.time()
        valid = (now - self.last_image_time) < IMAGE_STALE_S
        self.valid_pub.publish(Bool(data=valid))

    def best_lamp_area(self, mask, tag='') -> int:
        """Largest blob in `mask` that is shaped like a lamp, 0 if none is.

        Returns the area so the caller can compare candidates, but the shape
        and size gates are what actually keep background surfaces out.
        """
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        best = 0
        for c in contours:
            area = cv2.contourArea(c)
            perim = cv2.arcLength(c, True)
            circularity = (4.0 * math.pi * area / (perim * perim)) if perim > 0 else 0.0
            ok = (self.min_blob_area_px <= area <= self.max_blob_area_px
                  and circularity >= self.min_circularity)
            if self.debug_candidates and area >= self.min_blob_area_px:
                m = cv2.moments(c)
                cx = m['m10'] / m['m00'] if m['m00'] else 0.0
                cy = m['m01'] / m['m00'] if m['m00'] else 0.0
                self.get_logger().info(
                    '%s candidate area=%.0f circ=%.2f at roi(%.0f,%.0f) -> %s'
                    % (tag, area, circularity, cx, cy, 'ACCEPT' if ok else 'reject'))
            if ok:
                best = max(best, int(area))
        return best

    def crop(self, frame):
        """The four o'clock search window, in pixels."""
        h, w = frame.shape[:2]
        x0 = max(0, min(w - 1, int(w * self.roi_x_min_frac)))
        x1 = max(x0 + 1, min(w, int(w * self.roi_x_max_frac)))
        y0 = max(0, min(h - 1, int(h * self.roi_y_min_frac)))
        y1 = max(y0 + 1, min(h, int(h * self.roi_y_max_frac)))
        return frame[y0:y1, x0:x1]

    def classify(self, hsv) -> str:
        h, s, v = hsv[:, :, 0], hsv[:, :, 1], hsv[:, :, 2]
        lit = ((s >= self.sat_min) & (v >= self.val_min)).astype(np.uint8) * 255

        red = (((h >= RED1_H[0]) & (h <= RED1_H[1]))
               | ((h >= RED2_H[0]) & (h <= RED2_H[1]))).astype(np.uint8) * 255
        green = ((h >= GREEN_H[0]) & (h <= GREEN_H[1])).astype(np.uint8) * 255

        red_area = self.best_lamp_area(cv2.bitwise_and(red, lit), 'RED')
        green_area = self.best_lamp_area(cv2.bitwise_and(green, lit), 'GREEN')

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

        roi = self.crop(frame)
        if roi.size == 0:
            return
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
