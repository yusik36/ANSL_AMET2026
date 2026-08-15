#!/usr/bin/env python3
"""Field calibration helper -- NOT part of the driving pipeline.

Point the camera at the track surface (for lane_follow_node) or at a lit
traffic-light fixture (for traffic_light_node) and run this node. Every
couple seconds it samples a center region of the current frame and logs the
observed HSV min/max/mean, plus a ready-to-paste suggested threshold range
with a small margin -- so you can retune the placeholder constants in
lane_follow_node.py / traffic_light_node.py (or their ROS2 parameters
directly) without a GUI/display, which the practice RPi and the contest
laptop may not conveniently have.

Usage:
    ros2 run physicar_vision hsv_calibrate_node --ros-args -p roi_frac:=0.15

`roi_frac` controls how large a square region (as a fraction of the shorter
image dimension) around the image center is sampled. Aim the camera so the
surface/color you care about fills that box.
"""
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

ROI_FRAC = 0.15
SAMPLE_PERIOD_S = 2.0
MARGIN = 10  # suggested range = observed min/max padded by this much


class HsvCalibrateNode(Node):
    def __init__(self):
        super().__init__('hsv_calibrate_node')
        self.declare_parameter('roi_frac', ROI_FRAC)
        self.roi_frac = self.get_parameter('roi_frac').value

        self.bridge = CvBridge()
        self.latest_frame = None
        self.create_subscription(Image, 'image_raw', self.on_image, 10)
        self.create_timer(SAMPLE_PERIOD_S, self.sample)
        self.get_logger().info(
            f'hsv_calibrate_node ready: sampling a center box '
            f'({self.roi_frac:.0%} of shorter side) every {SAMPLE_PERIOD_S}s'
        )

    def on_image(self, msg: Image):
        try:
            self.latest_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as e:
            self.get_logger().warn(f'cv_bridge conversion failed: {e}')

    def sample(self):
        frame = self.latest_frame
        if frame is None:
            self.get_logger().info('no image received yet')
            return

        h, w = frame.shape[:2]
        box = int(min(h, w) * self.roi_frac)
        cy, cx = h // 2, w // 2
        roi = frame[cy - box // 2: cy + box // 2, cx - box // 2: cx + box // 2]
        if roi.size == 0:
            return

        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        h_ch, s_ch, v_ch = hsv[:, :, 0], hsv[:, :, 1], hsv[:, :, 2]

        h_min, h_max, h_mean = int(h_ch.min()), int(h_ch.max()), float(h_ch.mean())
        s_min, s_max, s_mean = int(s_ch.min()), int(s_ch.max()), float(s_ch.mean())
        v_min, v_max, v_mean = int(v_ch.min()), int(v_ch.max()), float(v_ch.mean())

        sug_h_min = max(0, h_min - MARGIN)
        sug_h_max = min(179, h_max + MARGIN)
        sug_s_min = max(0, s_min - MARGIN)
        sug_v_min = max(0, v_min - MARGIN)

        self.get_logger().info(
            f'H[{h_min}-{h_max}] mean={h_mean:.0f}  '
            f'S[{s_min}-{s_max}] mean={s_mean:.0f}  '
            f'V[{v_min}-{v_max}] mean={v_mean:.0f}  '
            f'-- suggested: h_min={sug_h_min} h_max={sug_h_max} '
            f's_min~{sug_s_min} v_min~{sug_v_min} (pad the side that matters '
            f'for your threshold direction, e.g. lane uses s_max/v_min, '
            f'traffic light uses sat_min/val_min)'
        )


def main():
    rclpy.init()
    node = HsvCalibrateNode()
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
