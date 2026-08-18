#!/usr/bin/env python3
"""Measure what the camera can actually see, before any of it is relied on.

The planner design rests on two claims that had never been checked: that the
track can be segmented from the camera image, and that image rows can be
turned into ground distances accurately enough to steer by. Both are cheap to
test in the simulator, which knows exactly where the car and every cone are,
and expensive to be wrong about later.

What it does
  1. Reads ground truth: car pose from /sim/api/pose, object positions and
     sizes from /sim/api/objects.
  2. Projects every cone into the image using the URDF camera geometry, and
     prints where it should appear.
  3. Grabs a frame and reports where things actually are, so predicted and
     observed rows can be compared -- that difference is the IPM error.
  4. Reports the usable range: how many image rows separate 1 m from 3 m, and
     what one row of segmentation error costs in metres at each distance.
  5. Tries a few segmentation rules on the road surface and says which
     separates road from surroundings, rather than assuming a colour.

    python3 tools/YS_perception_probe.py
    python3 tools/YS_perception_probe.py --save-frame frame.png
"""
import argparse
import json
import math
import sys
import urllib.error
import urllib.request

import numpy as np

BASE = 'http://localhost/sim/api'

# Camera geometry, chained from physicar.urdf.xacro:
#   base_footprint -> base_link      z = wheel_radius            0.0375
#   base_link      -> camera_pan     xyz = (0.05,  0, 0.1)
#   camera_pan     -> camera_tilt    xyz = (0.025, 0, 0.013)
#   camera_tilt    -> camera_link    xyz = (0.030, 0, 0.014)
CAM_HEIGHT = 0.0375 + 0.1 + 0.013 + 0.014      # 0.1645 m above ground
CAM_FORWARD = 0.05 + 0.025 + 0.030             # 0.105 m ahead of the rear axle
CAM_TILT = 0.0                                 # tilt joint parks at zero

# Official calibration, given for 640x480; scaled to the published frame size.
CAL_W, CAL_H = 640.0, 480.0
FX, FY, CX, CY = 387.89, 387.19, 312.63, 229.36


def get_json(path, timeout=3.0):
    try:
        with urllib.request.urlopen(BASE + path, timeout=timeout) as r:
            return json.loads(r.read().decode('utf-8'))
    except (urllib.error.URLError, OSError, ValueError) as e:
        print('  ! %s failed: %s' % (path, e))
        return None


def intrinsics(w, h):
    sx, sy = w / CAL_W, h / CAL_H
    return FX * sx, FY * sy, CX * sx, CY * sy


def ground_to_image(forward, left, w, h):
    """Where a point on the ground at (forward, left) metres lands in frame.

    Optical frame is z forward, x right, y down, so a point on the ground
    sits at y = +CAM_HEIGHT below the axis when the camera is level.
    """
    fx, fy, cx, cy = intrinsics(w, h)
    z = forward - CAM_FORWARD
    if z <= 0.01:
        return None
    x = -left
    y = CAM_HEIGHT
    if CAM_TILT:
        c, s = math.cos(CAM_TILT), math.sin(CAM_TILT)
        y, z = c * y - s * z, s * y + c * z
        if z <= 0.01:
            return None
    return cx + fx * x / z, cy + fy * y / z


def image_row_to_ground(row, h, w):
    """Inverse of the above for the centre column: row -> forward distance."""
    _fx, fy, _cx, cy = intrinsics(w, h)
    d = row - cy
    if d <= 0:
        return float('inf')          # at or above the horizon
    return CAM_HEIGHT * fy / d + CAM_FORWARD


def to_car_frame(pose, x, y):
    """World point -> (forward, left) in the car's frame."""
    dx, dy = x - pose['x'], y - pose['y']
    yaw = pose['yaw']
    fwd = dx * math.cos(yaw) + dy * math.sin(yaw)
    left = -dx * math.sin(yaw) + dy * math.cos(yaw)
    return fwd, left


def grab_frame():
    """One camera frame as BGR, via ROS (ground truth topic, not a re-encode)."""
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image
    from cv_bridge import CvBridge

    holder = {}

    class Grab(Node):
        def __init__(self):
            super().__init__('YS_perception_probe')
            self.b = CvBridge()
            self.create_subscription(Image, '/camera/image_raw', self.cb,
                                     qos_profile_sensor_data)

        def cb(self, msg):
            if 'f' not in holder:
                holder['f'] = self.b.imgmsg_to_cv2(msg, 'bgr8')
                rclpy.shutdown()

    rclpy.init()
    n = Grab()
    try:
        rclpy.spin(n)
    except Exception:
        pass
    return holder.get('f')


def report_range(w, h):
    print('=== usable range (camera %.3f m up, level) ===' % CAM_HEIGHT)
    _fx, fy, _cx, cy = intrinsics(w, h)
    print('  frame %dx%d, fy %.1f, horizon row %.1f' % (w, h, fy, cy))
    print('  %-10s %-8s %s' % ('distance', 'row', 'one row of error costs'))
    for d in (0.4, 0.6, 1.0, 1.5, 2.0, 3.0, 4.0):
        row = cy + CAM_HEIGHT * fy / max(d - CAM_FORWARD, 1e-6)
        near = image_row_to_ground(row + 1, h, w)
        print('  %-10.1f %-8.1f %.3f m' % (d, row, abs(d - near)))
    r1 = cy + CAM_HEIGHT * fy / (1.0 - CAM_FORWARD)
    r3 = cy + CAM_HEIGHT * fy / (3.0 - CAM_FORWARD)
    print('  1 m to 3 m spans %.0f rows of %d' % (r1 - r3, h))


def report_objects(pose, objects, w, h):
    print()
    print('=== where ground truth says things are ===')
    print('  car at (%.2f, %.2f) yaw %.1f deg' % (pose['x'], pose['y'],
                                                  math.degrees(pose['yaw'])))
    print('  %-9s %-18s %-16s %s'
          % ('name', 'car frame (fwd,left)', 'predicted px', 'size'))
    for ob in objects:
        c = ob.get('current') or ob.get('origin') or {}
        s = ob.get('size') or {}
        if 'x' not in c:
            continue
        fwd, left = to_car_frame(pose, c['x'], c['y'])
        px = ground_to_image(fwd, left, w, h)
        where = 'behind' if fwd <= 0 else (
            '(%.0f, %.0f)' % px if px and 0 <= px[0] < w and 0 <= px[1] < h
            else 'out of frame')
        print('  %-9s (%6.2f, %6.2f)   %-16s %.2fx%.2fx%.2f'
              % (ob['name'], fwd, left, where,
                 s.get('x', 0), s.get('y', 0), s.get('z', 0)))


def report_segmentation(frame):
    """Which simple rule separates the road from everything else?"""
    import cv2
    h, w = frame.shape[:2]
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    H, S, V = hsv[:, :, 0].astype(int), hsv[:, :, 1].astype(int), hsv[:, :, 2].astype(int)

    print()
    print('=== segmentation ===')
    # A patch the car is definitely sitting on, and one well off to the side.
    road = (slice(int(h * 0.88), h), slice(int(w * 0.40), int(w * 0.60)))
    for name, sl in (('road ahead (bottom centre)', road),
                     ('far left edge', (slice(int(h * 0.60), int(h * 0.80)),
                                        slice(0, int(w * 0.10)))),
                     ('far right edge', (slice(int(h * 0.60), int(h * 0.80)),
                                         slice(int(w * 0.90), w)))):
        print('  %-28s H %3d+-%-3d  S %3d+-%-3d  V %3d+-%-3d'
              % (name, H[sl].mean(), H[sl].std(), S[sl].mean(), S[sl].std(),
                 V[sl].mean(), V[sl].std()))

    road_s, road_v = S[road].mean(), V[road].mean()
    print()
    print('  candidate rules, by how much of the frame each keeps:')
    rules = {
        'low saturation (S < %d)' % max(road_s + 40, 60): S < max(road_s + 40, 60),
        'dark (V < %d)' % max(road_v + 50, 90): V < max(road_v + 50, 90),
        'low sat AND dark': (S < max(road_s + 40, 60)) & (V < max(road_v + 50, 90)),
        'not green (H outside 35-90 or S low)': ~(((H > 35) & (H < 90)) & (S > 60)),
    }
    for name, mask in rules.items():
        bottom = mask[int(h * 0.55):]
        print('    %-40s %5.1f%% of lower frame' % (name, 100.0 * bottom.mean()))


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--save-frame', default='YS_probe_frame.png')
    args = ap.parse_args()

    pose = get_json('/pose')
    objs = get_json('/objects')
    if not pose:
        sys.exit('no pose -- is the simulator running?')

    print('grabbing a camera frame...')
    frame = grab_frame()
    if frame is None:
        sys.exit('no frame on /camera/image_raw')
    h, w = frame.shape[:2]
    print('frame %dx%d' % (w, h))

    report_range(w, h)
    if objs and objs.get('objects'):
        report_objects(pose, objs['objects'], w, h)
    report_segmentation(frame)

    import cv2
    cv2.imwrite(args.save_frame, frame)
    print()
    print('frame saved -> %s  (open it to check the predictions above)'
          % args.save_frame)


if __name__ == '__main__':
    main()
