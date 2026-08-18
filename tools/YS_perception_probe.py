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


_GRABBER = {}


def grab_frame(timeout=5.0):
    """One camera frame as BGR, from the ROS topic rather than a re-encode.

    The node is created once and kept, so the calibration sweep can grab a
    frame per pose -- rclpy does not survive being init/shutdown in a loop.
    """
    import time
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image
    from cv_bridge import CvBridge

    if 'node' not in _GRABBER:
        rclpy.init()
        node = Node('YS_perception_probe')
        bridge = CvBridge()
        latest = {}

        def cb(msg):
            latest['f'] = bridge.imgmsg_to_cv2(msg, 'bgr8')

        node.create_subscription(Image, '/camera/image_raw', cb,
                                 qos_profile_sensor_data)
        _GRABBER.update(node=node, latest=latest, rclpy=rclpy)

    node, latest = _GRABBER['node'], _GRABBER['latest']
    latest.pop('f', None)
    end = time.time() + timeout
    while time.time() < end and 'f' not in latest:
        rclpy.spin_once(node, timeout_sec=0.1)
    return latest.get('f')


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


def measure_ipm_error(frame, pose, objects, w, h):
    """Compare where each cone is predicted to appear against where it is.

    The prediction comes from the URDF geometry; the observation comes from
    the frame. Their difference is the number that says how far ahead the
    planner may trust what it sees. Cones are found by looking for a compact
    blob near the predicted column, so a wrong prediction shows up as a large
    residual rather than quietly matching itself.
    """
    import cv2
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    print()
    print('=== IPM check: predicted vs observed cone base ===')
    print('  %-8s %-8s %-11s %-11s %s'
          % ('cone', 'fwd [m]', 'pred (c,r)', 'seen (c,r)', 'error'))

    any_seen = False
    for ob in objects:
        if not ob['name'].startswith('cone'):
            continue
        c = ob.get('current') or ob.get('origin') or {}
        if 'x' not in c:
            continue
        fwd, left = to_car_frame(pose, c['x'], c['y'])
        pred = ground_to_image(fwd, left, w, h)
        if pred is None or not (0 <= pred[0] < w and 0 <= pred[1] < h):
            continue

        # Search a generous window around the prediction for the cone: it is
        # the saturated, non-road-hue thing standing on the ground.
        col0, col1 = int(max(0, pred[0] - 60)), int(min(w, pred[0] + 60))
        row0, row1 = int(max(0, pred[1] - 60)), int(min(h, pred[1] + 60))
        win = hsv[row0:row1, col0:col1]
        if win.size == 0:
            continue
        H, S, V = win[:, :, 0].astype(int), win[:, :, 1], win[:, :, 2]
        # Road measured around hue 100-110; anything strongly coloured and
        # away from that is a candidate.
        mask = ((S > 90) & (V > 60) & ((H < 90) | (H > 125))).astype(np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
        cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cnts = [c2 for c2 in cnts if cv2.contourArea(c2) > 20]
        if not cnts:
            print('  %-8s %-8.2f (%4.0f,%4.0f) %-11s not found'
                  % (ob['name'], fwd, pred[0], pred[1], '--'))
            continue

        big = max(cnts, key=cv2.contourArea)
        x, y, bw, bh = cv2.boundingRect(big)
        seen_col = col0 + x + bw / 2.0
        seen_row = row0 + y + bh          # base of the cone sits on the ground
        d_pred = image_row_to_ground(pred[1], h, w)
        d_seen = image_row_to_ground(seen_row, h, w)
        print('  %-8s %-8.2f (%4.0f,%4.0f) (%4.0f,%4.0f)  %+.0f px row -> '
              '%+.2f m range, %+.0f px col'
              % (ob['name'], fwd, pred[0], pred[1], seen_col, seen_row,
                 seen_row - pred[1], d_seen - d_pred, seen_col - pred[0]))
        any_seen = True

    if not any_seen:
        print('  no cone was both predicted in frame and found -- move the car '
              'so one is ahead of it, then re-run')


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


def post_json(path, payload, timeout=5.0):
    req = urllib.request.Request(
        BASE + path, data=json.dumps(payload).encode('utf-8'),
        headers={'Content-Type': 'application/json'}, method='POST')
    try:
        with urllib.request.urlopen(req, timeout=timeout) as r:
            return json.loads(r.read().decode('utf-8'))
    except (urllib.error.URLError, OSError, ValueError) as e:
        print('  ! POST %s failed: %s' % (path, e))
        return None


def calibrate_against_cone(cone, distances, w_h=None):
    """Park the car a measured distance behind a cone, facing it, and see
    which row the cone lands on. Repeating at several distances turns a
    single agreement into a curve, which is the difference between "the
    geometry happens to fit here" and "the geometry is right".
    """
    import time
    c = cone.get('current') or cone.get('origin')
    print()
    print('=== IPM calibration against %s at (%.2f, %.2f) ==='
          % (cone['name'], c['x'], c['y']))
    print('  %-8s %-10s %-10s %-10s %s'
          % ('set [m]', 'true [m]', 'pred row', 'seen row', 'range error'))

    rows = []
    for d in distances:
        # Approach along -x so the car faces +x toward the cone.
        pose = {'x': c['x'] - d, 'y': c['y'], 'yaw': 0.0}
        if post_json('/pose', pose) is None:
            print('  teleport unsupported -- skipping calibration')
            return
        time.sleep(1.2)
        actual = get_json('/pose')
        frame = grab_frame()
        if frame is None or actual is None:
            print('  %-8.2f  (no frame)' % d)
            continue
        h, w = frame.shape[:2]
        fwd, left = to_car_frame(actual, c['x'], c['y'])
        pred = ground_to_image(fwd, left, w, h)
        seen = find_cone_base(frame, pred, w, h)
        if pred is None or seen is None:
            print('  %-8.2f %-10.2f %-10s %-10s --'
                  % (d, fwd, '%.0f' % pred[1] if pred else '--', 'not found'))
            continue
        d_pred = image_row_to_ground(pred[1], h, w)
        d_seen = image_row_to_ground(seen[1], h, w)
        print('  %-8.2f %-10.2f %-10.0f %-10.0f %+.3f m'
              % (d, fwd, pred[1], seen[1], d_seen - d_pred))
        rows.append((fwd, pred[1], seen[1]))

    if len(rows) >= 2:
        errs = [abs(image_row_to_ground(s, 360, 480)
                    - image_row_to_ground(p, 360, 480)) for _f, p, s in rows]
        print('  range error: mean %.3f m, worst %.3f m' %
              (sum(errs) / len(errs), max(errs)))
        print('  -> trust the corridor out to roughly where the error stays '
              'under the track half-width (0.45 m)')


def find_cone_base(frame, pred, w, h, window=70):
    """Bottom-centre of the strongly coloured blob nearest the prediction."""
    import cv2
    if pred is None:
        return None
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    col0, col1 = int(max(0, pred[0] - window)), int(min(w, pred[0] + window))
    row0, row1 = int(max(0, pred[1] - window)), int(min(h, pred[1] + window))
    win = hsv[row0:row1, col0:col1]
    if win.size == 0:
        return None
    H, S, V = win[:, :, 0].astype(int), win[:, :, 1], win[:, :, 2]
    mask = ((S > 90) & (V > 60) & ((H < 90) | (H > 125))).astype(np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnts = [c for c in cnts if cv2.contourArea(c) > 20]
    if not cnts:
        return None
    x, y, bw, bh = cv2.boundingRect(max(cnts, key=cv2.contourArea))
    return col0 + x + bw / 2.0, row0 + y + bh


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--save-frame', default='YS_probe_frame.png')
    ap.add_argument('--calibrate', metavar='CONE',
                    help='park behind this cone at several distances and '
                         'measure the IPM error (moves the car)')
    ap.add_argument('--distances', default='0.8,1.2,1.6,2.0,2.5,3.0')
    args = ap.parse_args()

    if args.calibrate:
        objs = get_json('/objects') or {}
        match = [o for o in objs.get('objects', []) if o['name'] == args.calibrate]
        if not match:
            sys.exit('no object named %s' % args.calibrate)
        calibrate_against_cone(
            match[0], [float(x) for x in args.distances.split(',')])
        return

    objs = get_json('/objects')
    # Pose is read either side of the grab. If the car moved in between --
    # someone driving it, a reset -- then the predictions belong to a
    # different frame than the one measured, and the comparison is
    # meaningless. That happened on the first run and cost a whole pass.
    pose = get_json('/pose')
    if not pose:
        sys.exit('no pose -- is the simulator running?')

    print('grabbing a camera frame...')
    frame = grab_frame()
    if frame is None:
        sys.exit('no frame on /camera/image_raw')

    after = get_json('/pose') or {}
    moved = max(abs(after.get(k, pose[k]) - pose[k]) for k in ('x', 'y', 'yaw'))
    if moved > 0.01:
        print('!! the car moved %.3f during the grab -- predictions below do '
              'not match this frame. Re-run with the car held still.' % moved)
    h, w = frame.shape[:2]
    print('frame %dx%d' % (w, h))

    report_range(w, h)
    if objs and objs.get('objects'):
        report_objects(pose, objs['objects'], w, h)
        measure_ipm_error(frame, pose, objs['objects'], w, h)
    report_segmentation(frame)

    import cv2
    cv2.imwrite(args.save_frame, frame)
    print()
    print('frame saved -> %s  (open it to check the predictions above)'
          % args.save_frame)


if __name__ == '__main__':
    main()
