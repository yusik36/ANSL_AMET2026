#!/usr/bin/env python3
"""Traffic-light color detection.

Subscribes /image_raw, looks for red/yellow/green blobs via HSV, and
publishes which one (if any) currently dominates the frame.

Publishes:
  traffic/light_state  (String)  -- one of "NONE", "RED", "YELLOW", "GREEN"
  traffic/valid         (Bool)    -- true if a frame was processed recently
                                     (i.e. the camera/this node is alive).
                                     False only means "stale camera data",
                                     NOT "no light currently visible" -- see
                                     below.

Unlike lane color, standard traffic-light red/yellow/green are effectively
venue-independent, so the HSV ranges below are reasonable real defaults
rather than pure placeholders -- still exposed as parameters for field
tuning against the actual light fixture/camera exposure.

Design decision worth calling out (this was a judgment call, not something
confirmed with contest staff): the competition only penalizes *starting on
a red light*, not sitting still at a green one, and most of the track has
no traffic light in view at all. So this node distinguishes two different
kinds of "don't know":
  - camera/node dead (traffic/valid=False) -> judgment_node must fail safe
    and stop, because we genuinely can't tell if it's red.
  - camera alive, frame processed, but no light-colored blob found this
    frame (traffic/light_state="NONE") -> judgment_node treats this as
    "go" (matches state after a green light, and matches the common case of
    a straightaway with no light in view -- the only case that has to force
    a stop is a positively-identified red light).
That scoping choice lives in physicar_judgment/judgment_node.py, not here;
this node only ever reports what it currently sees.

A short 2-frame confirmation counter is used before switching the published
state, to avoid single-frame flicker (specular reflections, motion blur)
causing a jerky stop/go/stop -- this is purely temporal smoothing of this
node's own noisy classification, not a source of statefulness that would
break the stateless-restart requirement (there is no persistent identity or
track-position state here, just "was the last N frames' answer the same").
"""
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String
from cv_bridge import CvBridge, CvBridgeError

MIN_BLOB_AREA_PX = 80
CONFIRM_FRAMES = 2
IMAGE_STALE_S = 1.0

# OpenCV hue is 0-179. Red wraps around 0/179, so it needs two ranges.
RED1_H = (0, 8)
RED2_H = (170, 179)
YELLOW_H = (18, 35)
GREEN_H = (45, 85)
# sat_min=100 let large low-saturation background surfaces (green track mat,
# tan wood door/paneling -- both ~S100-160 under this room's lighting)
# out-area the genuine light and win classify()'s argmax, confirmed
# 2026-08-18 on the practice rig with an actual lit red signal in frame
# (real light held S up to 255; background never exceeded ~160). Raising
# this to 150 keeps both false positives under min_blob_area_px while still
# clearing it for the real light by a comfortable margin (142px vs 80).
SAT_MIN = 150
VAL_MIN = 100


class TrafficLightNode(Node):
    def __init__(self):
        super().__init__('traffic_light_node')

        self.declare_parameter('min_blob_area_px', MIN_BLOB_AREA_PX)
        self.declare_parameter('confirm_frames', CONFIRM_FRAMES)
        self.declare_parameter('sat_min', SAT_MIN)
        self.declare_parameter('val_min', VAL_MIN)

        self.min_blob_area_px = self.get_parameter('min_blob_area_px').value
        self.confirm_frames = self.get_parameter('confirm_frames').value
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

        self.get_logger().info('traffic_light_node ready')

    def check_staleness(self):
        # Runs independently of on_image so "no frames arriving at all"
        # (camera/node dead) is still detected even though this node is
        # otherwise purely reactive to incoming images.
        now = time.time()
        valid = (now - self.last_image_time) < IMAGE_STALE_S
        self.valid_pub.publish(Bool(data=valid))

    def classify(self, hsv) -> str:
        s = hsv[:, :, 1]
        v = hsv[:, :, 2]
        sv_mask = (s >= self.sat_min) & (v >= self.val_min)

        h = hsv[:, :, 0]
        red_mask = (((h >= RED1_H[0]) & (h <= RED1_H[1])) | ((h >= RED2_H[0]) & (h <= RED2_H[1]))) & sv_mask
        yellow_mask = (h >= YELLOW_H[0]) & (h <= YELLOW_H[1]) & sv_mask
        green_mask = (h >= GREEN_H[0]) & (h <= GREEN_H[1]) & sv_mask

        areas = {
            'RED': int(np.count_nonzero(red_mask)),
            'YELLOW': int(np.count_nonzero(yellow_mask)),
            'GREEN': int(np.count_nonzero(green_mask)),
        }
        best_color, best_area = max(areas.items(), key=lambda kv: kv[1])
        if best_area < self.min_blob_area_px:
            return 'NONE'
        return best_color

    def on_image(self, msg: Image):
        self.last_image_time = time.time()
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as e:
            self.get_logger().warn(f'cv_bridge conversion failed, dropping frame: {e}')
            return

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        reading = self.classify(hsv)

        if reading == self.candidate_state:
            self.candidate_count += 1
        else:
            self.candidate_state = reading
            self.candidate_count = 1

        if self.candidate_count >= self.confirm_frames and reading != self.published_state:
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
