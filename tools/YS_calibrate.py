#!/usr/bin/env python3
"""Point the car down the track, run this, get the numbers the planner needs.

    ros2 run --prefix 'python3' ... no: just
    python3 tools/YS_calibrate.py
    python3 tools/YS_calibrate.py --save /tmp/frame.png --explain

At the venue nothing about the colours carries over. The floor is an
exhibition hall, not the simulator's grass; the boundary wall does not exist;
the light is whatever the hall has. What carries over is the SHAPE of the
rule -- the road is one cluster, the paint on it is another, everything else
is an obstacle -- so this tool re-measures the numbers and prints the launch
arguments that set them.

Three things it answers, in order of how badly they bite:

  1. Does the road separate from its surroundings at all, and IN WHICH
     CHANNEL? Hue is what the simulator needed, but a grey track on a grey
     floor separates in value or not at all, and finding that out on the
     morning of the race with a hue-only planner is too late. So all three
     channels are scored and the best one is named.

  2. What are the actual bounds? Taken from the patch of ground directly in
     front of the car, which is road by construction -- the car is standing
     on it -- rather than from a guess about what road looks like.

  3. Does the corridor scan then work? Printed row by row, because "the
     planner sees nothing" and "the planner sees a corridor 5 cm wide" are
     different problems and the summary number hides which one you have.
"""
import argparse
import os
import sys

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                '..', 'src', 'physicar_planner'))
from physicar_planner import corridor as C   # noqa: E402

CHANNELS = ('hue', 'saturation', 'value')


def grab(topic, timeout):
    """One frame off the camera. Sensor QoS: the publisher may be
    best-effort, and a reliable subscriber gets silence rather than an error."""
    rclpy.init()
    node = Node('ys_calibrate')
    bridge = CvBridge()
    got = {}

    def on_image(msg):
        if 'frame' not in got:
            got['frame'] = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    node.create_subscription(Image, topic, on_image, qos_profile_sensor_data)
    deadline = node.get_clock().now().nanoseconds + int(timeout * 1e9)
    while 'frame' not in got and node.get_clock().now().nanoseconds < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    rclpy.shutdown()
    return got.get('frame')


def road_patch(hsv):
    """The ground the car is standing on: bottom centre of the frame.

    Chosen by geometry rather than by colour, which is the point -- it is
    road because of where the car is, so it can be used to learn what road
    looks like without already knowing.
    """
    h, w = hsv.shape[:2]
    row = int(round(C.distance_to_row(0.45, w, h)))
    row = min(max(row, 0), h - 1)
    r0, r1 = max(0, row - 12), min(h, row + 12)
    c0, c1 = int(w * 0.42), int(w * 0.58)
    return hsv[r0:r1, c0:c1], (r0, r1, c0, c1)


def separation(patch, rest):
    """How well one channel tells the patch from everything else.

    Not a correlation or a t-statistic: the planner thresholds a channel, so
    the useful question is what fraction of the surroundings a band around
    the patch's own spread would wrongly admit. 0 means the band admits
    nothing else -- a clean cut. 1 means the channel is useless here.
    """
    out = {}
    for i, name in enumerate(CHANNELS):
        p = patch[:, :, i].astype(np.int16).ravel()
        r = rest[:, :, i].astype(np.int16).ravel()
        lo, hi = np.percentile(p, 2), np.percentile(p, 98)
        margin = max(4.0, 0.25 * (hi - lo))
        lo, hi = lo - margin, hi + margin
        leak = float(np.mean((r >= lo) & (r <= hi))) if r.size else 1.0
        out[name] = (leak, float(lo), float(hi))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--topic', default='/camera/image_raw')
    ap.add_argument('--timeout', type=float, default=10.0)
    ap.add_argument('--save', default='', help='write the frame here')
    ap.add_argument('--explain', action='store_true',
                    help='per-range corridor detail')
    a = ap.parse_args()

    frame = grab(a.topic, a.timeout)
    if frame is None:
        print('no frame on %s within %.0f s -- is the camera publishing?'
              % (a.topic, a.timeout))
        return 1
    if a.save:
        cv2.imwrite(a.save, frame)
        print('frame saved: %s' % a.save)

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    h, w = hsv.shape[:2]
    print('frame %dx%d' % (w, h))

    patch, (r0, r1, c0, c1) = road_patch(hsv)
    lower = hsv[int(h * 0.55):, :]
    print('\n=== what the car is standing on (rows %d-%d, cols %d-%d) ===' %
          (r0, r1, c0, c1))
    for i, name in enumerate(CHANNELS):
        ch = patch[:, :, i]
        print('  %-11s median %3d   2-98%%  %3d - %3d'
              % (name, int(np.median(ch)),
                 int(np.percentile(ch, 2)), int(np.percentile(ch, 98))))

    print('\n=== which channel separates road from surroundings ===')
    print('    leak = fraction of the rest of the frame a band around the')
    print('    road patch would wrongly admit. Lower is better.')
    sep = separation(patch, lower)
    for name, (leak, lo, hi) in sorted(sep.items(), key=lambda kv: kv[1][0]):
        print('  %-11s leak %5.1f%%   band %3.0f - %3.0f' % (name, leak * 100, lo, hi))
    best = min(sep.items(), key=lambda kv: kv[1][0])
    print('  -> best: %s (leak %.1f%%)' % (best[0], best[1][0] * 100))
    if best[1][0] > 0.35:
        print('  !! no channel cuts cleanly. The planner keys on hue, so if')
        print('     hue is not the winner here, the venue needs the rule')
        print('     changed, not the numbers -- do not just widen the band.')

    print('\n=== the rule as currently configured ===')
    blocked = C.blocked_mask(hsv)
    lower_blocked = blocked[int(h * 0.55):, :]
    print('  road hue %d-%d | paint S<=%d V>=%d | marks H %d-%d S>=%d'
          % (C.ROAD_H_MIN, C.ROAD_H_MAX, C.PAINT_S_MAX, C.PAINT_V_MIN,
             C.MARK_H_MIN, C.MARK_H_MAX, C.MARK_S_MIN))
    print('  lower frame: %.1f%% drivable, %.1f%% blocked'
          % ((1 - lower_blocked.mean()) * 100, lower_blocked.mean() * 100))
    print('  the patch under the car: %.1f%% drivable'
          % ((1 - C.blocked_mask(patch).mean()) * 100))

    hue_lo = int(np.percentile(patch[:, :, 0], 2))
    hue_hi = int(np.percentile(patch[:, :, 0], 98))
    pad = max(6, (hue_hi - hue_lo))
    print('\n=== suggested launch arguments ===')
    print('  ros2 launch physicar_bringup real_autonomy_launch.py \\')
    print('      road_h_min:=%d road_h_max:=%d' % (max(0, hue_lo - pad),
                                                   min(179, hue_hi + pad)))

    print('\n=== corridor scan ===')
    ranges = np.linspace(C.DEFAULT_MIN_RANGE, C.DEFAULT_MAX_RANGE,
                         C.DEFAULT_SAMPLES)
    corr = C.scan_corridor(blocked, w, h, ranges)
    if a.explain:
        print('   dist   row  ctr-col  centre    span     right    left')
        for d in ranges:
            row = int(round(C.distance_to_row(d, w, h)))
            ctr = int(round(C.lateral_to_column(0.0, d, w, h)))
            if row < 0 or row >= h or ctr < 0 or ctr >= w:
                print('  %5.2f  %4d  %6d   off-frame' % (d, row, ctr))
                continue
            hit = [s for s in corr if abs(s[0] - d) < 1e-6]
            state = 'BLOCKED' if blocked[row][ctr] else 'clear  '
            if not hit:
                print('  %5.2f  %4d  %6d   %s  dropped (span > %.2f m)'
                      % (d, row, ctr, state, C.MAX_PLAUSIBLE_SPAN))
            else:
                _d, r, l = hit[0]
                print('  %5.2f  %4d  %6d   %s  %5.2f m  %+6.2f  %+6.2f'
                      % (d, row, ctr, state, l - r, r, l))
    usable = [s for s in corr if s[2] > s[1]]
    print('  %d of %d ranges have a free span' % (len(usable), len(ranges)))

    target = C.pick_target(corr, C.lookahead_for(0.0, 0.35, 0.45))
    if target is None:
        print('  target: NONE -- the planner would stop the car here')
        return 2
    steer, radius = C.pure_pursuit(target)
    print('  target: %.2f m ahead, %+.3f m across -> %+.1f deg'
          % (target[0], target[1], np.degrees(steer)))
    return 0


if __name__ == '__main__':
    sys.exit(main())
