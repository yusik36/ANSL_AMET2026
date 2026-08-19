#!/usr/bin/env python3
"""What the planner asked for and what the car did, on one time axis.

    python3 tools/YS_trace.py --seconds 8
    python3 tools/YS_trace.py --teleport 1.40,3.39,-1.5708 --seconds 8

The bench says whether a lap happened; the debug log says what the planner
decided. Neither answers the question you actually have after a crash, which
is whether the car did what it was told. Commanded speed and achieved speed
in adjacent columns settle it in one glance: same numbers means the planner
chose wrong, different numbers means the command never arrived.

Pose comes from the simulator's HTTP API rather than from odometry on
purpose -- odometry here is laser scan matching against a fence, which is
exactly the thing that will not exist at the practice venue, so a trace
built on it would measure the estimator as much as the car.
"""
import argparse
import json
import sys
import threading
import time
import urllib.request

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Bool, Float64


class Trace(Node):
    def __init__(self, api):
        super().__init__('ys_trace')
        self.api = api
        self.v = {}
        for topic, typ in (('plan/speed', Float64), ('plan/steering', Float64),
                           ('plan/valid', Bool), ('speed', Float64),
                           ('steering', Float64), ('traffic/valid', Bool)):
            self.create_subscription(
                typ, topic,
                lambda m, k=topic: self.v.__setitem__(k, m.data),
                qos_profile_sensor_data)

    def pose(self):
        try:
            with urllib.request.urlopen(self.api + '/pose', timeout=0.5) as r:
                return json.load(r)
        except Exception:
            return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--api', default='http://localhost/sim/api')
    ap.add_argument('--seconds', type=float, default=8.0)
    ap.add_argument('--rate', type=float, default=10.0)
    ap.add_argument('--teleport', default='',
                    help='x,y,yaw -- place the car, then trace from there')
    a = ap.parse_args()

    rclpy.init()
    node = Trace(a.api)
    spin = threading.Thread(
        target=lambda: rclpy.spin(node), daemon=True)
    spin.start()
    time.sleep(1.0)             # let the subscriptions match

    if a.teleport:
        x, y, yaw = (float(t) for t in a.teleport.split(','))
        body = json.dumps({'x': x, 'y': y, 'yaw': yaw}).encode()
        req = urllib.request.Request(a.api + '/pose', data=body,
                                     headers={'Content-Type': 'application/json'})
        with urllib.request.urlopen(req, timeout=2.0) as r:
            print('teleport: %s' % json.load(r).get('applied'))

    print('   t     x      y     yaw   | valid  plan_v  plan_st |  cmd_v'
          '  cmd_st | measured')
    prev, prev_t = None, None
    t0 = time.time()
    step = 1.0 / a.rate
    while time.time() - t0 < a.seconds:
        now = time.time()
        p = node.pose()
        meas = float('nan')
        if p and prev is not None and now > prev_t:
            meas = ((p['x'] - prev[0]) ** 2 + (p['y'] - prev[1]) ** 2) ** 0.5 \
                / (now - prev_t)
        if p:
            prev, prev_t = (p['x'], p['y']), now
        g = node.v.get
        print('%5.2f %6.3f %6.3f %6.3f | %-5s %6.2f %+7.3f | %6.2f %+7.3f | %6.2f'
              % (now - t0,
                 p['x'] if p else float('nan'), p['y'] if p else float('nan'),
                 p['yaw'] if p else float('nan'),
                 g('plan/valid'), g('plan/speed', float('nan')),
                 g('plan/steering', float('nan')),
                 g('speed', float('nan')), g('steering', float('nan')), meas))
        slack = step - (time.time() - now)
        if slack > 0:
            time.sleep(slack)

    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
