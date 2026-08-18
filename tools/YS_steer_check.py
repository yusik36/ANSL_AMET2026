#!/usr/bin/env python3
"""Measure which way the car actually turns, instead of assuming.

Publishes a known steering command with a little speed, watches the
ground-truth yaw, and reports the sign the vehicle really uses. It also
checks the lidar's mounting by asking which bearing the nearest returns come
from while a wall or cone sits ahead.

Both answers are supposed to be knowable from the source -- Physicar's
adapter computes angular.z = v*tan(steering)/wheelbase, so positive should
turn left, and the URDF mounts the lidar facing forward. But our practice
chassis is wired the other way on both counts, which is exactly the kind of
thing that is cheap to measure and expensive to be wrong about: a flipped
steering sign makes obstacle avoidance drive into the obstacle.

Run once per vehicle -- in the simulator now, and again on 2026-08-25 when
the real car arrives.

    python3 tools/YS_steer_check.py                 # steering + lidar
    python3 tools/YS_steer_check.py --steering-only
    python3 tools/YS_steer_check.py --angle 0.25 --speed 0.4

SAFETY: this drives the car. In the simulator that is free. On real hardware
give it clear space, and keep a hand on the power.
"""
import argparse
import json
import math
import subprocess
import sys
import time
import urllib.error
import urllib.request

DEFAULT_BASE = 'http://localhost/sim/api'


def get_json(url, timeout=1.0):
    try:
        with urllib.request.urlopen(url, timeout=timeout) as r:
            return json.loads(r.read().decode('utf-8'))
    except (urllib.error.URLError, OSError, ValueError):
        return None


def pub_once(topic, msg_type, payload):
    subprocess.run(['ros2', 'topic', 'pub', '--once', topic, msg_type, payload],
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                   timeout=15)


def drive(speed, steering):
    pub_once('/steering', 'std_msgs/msg/Float64', '{data: %f}' % steering)
    pub_once('/speed', 'std_msgs/msg/Float64', '{data: %f}' % speed)


def stop():
    pub_once('/speed', 'std_msgs/msg/Float64', '{data: 0.0}')
    pub_once('/steering', 'std_msgs/msg/Float64', '{data: 0.0}')


def wrap(a):
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


def yaw_of(base):
    p = get_json(base + '/pose')
    if not isinstance(p, dict) or 'yaw' not in p:
        return None
    return float(p['yaw'])


def run_steering(base, angle, speed, duration):
    """Drive with a fixed positive steering angle; report the yaw change."""
    print('--- resetting')
    try:
        req = urllib.request.Request(base + '/reset', data=b'{}',
                                     headers={'Content-Type': 'application/json'},
                                     method='POST')
        urllib.request.urlopen(req, timeout=5).read()
    except Exception:
        print('    (reset failed -- continuing from wherever the car is)')
    time.sleep(2.0)

    y0 = yaw_of(base)
    if y0 is None:
        sys.exit('no ground-truth pose from %s/pose' % base)

    print('--- driving  steering=+%.3f rad (+%.1f deg), speed=%.2f m/s, %.1fs'
          % (angle, math.degrees(angle), speed, duration))
    t_end = time.time() + duration
    # The driver expires a speed command after ~1 s without renewal, so it has
    # to be refreshed for the whole window rather than sent once.
    while time.time() < t_end:
        drive(speed, angle)
        time.sleep(0.25)
    stop()
    time.sleep(0.5)

    y1 = yaw_of(base)
    if y1 is None:
        sys.exit('lost pose while driving')
    dyaw = wrap(y1 - y0)

    print()
    print('    yaw %.3f -> %.3f rad   change %+.3f rad (%+.1f deg)'
          % (y0, y1, dyaw, math.degrees(dyaw)))
    if abs(dyaw) < math.radians(3.0):
        print('    INCONCLUSIVE: the car barely turned.')
        print('      Did it move at all? Raise --speed or --duration, and')
        print('      check nothing else is publishing /speed (a running stack')
        print('      will fight this script for the topic).')
        return None

    turned_left = dyaw > 0
    print('    positive /steering turned %s' % ('LEFT' if turned_left else 'RIGHT'))
    print()
    if turned_left:
        print('    => REP-103 convention, as Physicar is expected to use.')
        print('       avoid_steer_sign = +1.0   lane_steer_sign = +1.0')
    else:
        print('    => REVERSED from REP-103, as the practice chassis is.')
        print('       avoid_steer_sign = -1.0   lane_steer_sign = -1.0')
    print('       (both nodes assume positive steering means left, so they')
    print('        take the same sign -- one measurement settles both)')
    return turned_left


def lidar_bearings(base):
    """(bearing_deg, range_m) pairs from the robot's Web API.

    The exact response shape is not documented, so a few plausible ones are
    accepted and anything else is reported rather than guessed at.
    """
    data = get_json(base + '/lidar', timeout=3.0)
    if data is None:
        return None, 'no response from /lidar'

    # {"ranges": [...], "angle_min": r, "angle_increment": r}
    if isinstance(data, dict) and isinstance(data.get('ranges'), list):
        rs = data['ranges']
        a0 = float(data.get('angle_min', -math.pi))
        inc = data.get('angle_increment')
        inc = float(inc) if inc else (2 * math.pi / max(len(rs), 1))
        return [(math.degrees(wrap(a0 + i * inc)), r)
                for i, r in enumerate(rs)
                if isinstance(r, (int, float)) and math.isfinite(r)], None

    # {"points": [{"angle": deg_or_rad, "range": m}, ...]}
    if isinstance(data, dict) and isinstance(data.get('points'), list):
        out = []
        for p in data['points']:
            if not isinstance(p, dict):
                continue
            a, r = p.get('angle'), p.get('range', p.get('distance'))
            if isinstance(a, (int, float)) and isinstance(r, (int, float)):
                a = math.degrees(wrap(a)) if abs(a) <= math.pi + 1e-6 else a
                out.append((a, float(r)))
        return out, None

    # [[angle, range], ...]
    if isinstance(data, list) and data and isinstance(data[0], (list, tuple)):
        return [(math.degrees(wrap(a)) if abs(a) <= math.pi + 1e-6 else a, float(r))
                for a, r in data if isinstance(r, (int, float))], None

    # [r0, r1, ...] assumed to span a full turn
    if isinstance(data, list) and data and isinstance(data[0], (int, float)):
        n = len(data)
        return [(math.degrees(wrap(-math.pi + i * 2 * math.pi / n)), float(r))
                for i, r in enumerate(data) if math.isfinite(r)], None

    return None, 'unrecognised /lidar shape: %s' % (str(data)[:200],)


def run_lidar(base):
    """Report which scan bearing the nearest returns sit at.

    With something ahead of the car, the closest returns should cluster near
    0 deg. Clustering near 180 deg means the lidar is mounted backwards and
    front_offset_deg has to compensate.
    """
    print()
    print('--- lidar bearing check')
    print('    (put a cone or a wall clearly ahead of the car first)')
    pairs, err = lidar_bearings(base)
    if pairs is None:
        print('    %s' % err)
        return
    pairs = [(a, r) for a, r in pairs if r > 0.05]
    if not pairs:
        print('    scan carried no usable returns')
        return

    pairs.sort(key=lambda ar: ar[1])
    print('    nearest returns (bearing, range):')
    for a, r in pairs[:6]:
        print('      %+7.1f deg   %.2f m' % (a, r))

    near = [a for a, _ in pairs[:20]]
    forward = sum(1 for a in near if abs(a) < 45)
    rear = sum(1 for a in near if abs(a) > 135)
    print()
    print('    of the %d closest: %d ahead (|bearing|<45), %d behind (>135)'
          % (len(near), forward, rear))
    if rear > forward:
        print('    => scan 0 points BACKWARD: front_offset_deg = 180.0')
    elif forward > rear:
        print('    => scan 0 points FORWARD: front_offset_deg = 0.0')
    else:
        print('    INCONCLUSIVE: put something distinctly closer on one side')
        print('      of the car and run again.')


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--base', default=DEFAULT_BASE)
    ap.add_argument('--angle', type=float, default=0.30,
                    help='steering command in rad (max 0.35)')
    ap.add_argument('--speed', type=float, default=0.4, help='m/s')
    ap.add_argument('--duration', type=float, default=3.0, help='seconds')
    ap.add_argument('--steering-only', action='store_true')
    ap.add_argument('--lidar-only', action='store_true')
    args = ap.parse_args()

    if get_json(args.base + '/status') is None:
        sys.exit('cannot reach %s -- is the simulator running?' % args.base)

    try:
        if not args.lidar_only:
            run_steering(args.base, abs(args.angle), args.speed, args.duration)
        if not args.steering_only:
            run_lidar(args.base)
    finally:
        stop()
    print()
    print('done -- car stopped')


if __name__ == '__main__':
    main()
