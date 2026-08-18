#!/usr/bin/env python3
"""Camera-driven corridor planner: one node from image to speed and steering.

Replaces the lane-following and obstacle-avoidance pair. Those split the job
into "follow the line" and "an obstacle has appeared, take over", which meant
control changed hands mid-corner and the lane node had to recover from
wherever avoidance left the car. Here an obstacle is just a place where the
drivable corridor is narrower, so there is no handover to get wrong.

Publishes:
  plan/speed      (Float64, m/s)   what the corridor allows right now
  plan/steering   (Float64, rad)   positive is left, matching Physicar
  plan/valid      (Bool)           false when there is nowhere to go, or the
                                   camera has gone quiet. judgment_node must
                                   fail safe on this, not average it away.

The planning itself lives in corridor.py with no ROS in it, so it can be --
and is -- checked against synthetic scenes and real frames without a
simulator running. This file is wiring: parameters, topics, staleness.

Parameters worth turning on race day, and nothing else:
  aggression        lateral acceleration budget, m/s^2. Not a measured
                    constant -- the knob. The simulator runs ODE's default
                    mu=1 on every wheel, worth about 9.8 m/s^2, which
                    flatters the real tyres, so the default here is
                    deliberately pessimistic.
  lookahead_gain    Ld = gain*v + base. ForzaETH's rule: as short as it can
                    be without the car weaving.
  lookahead_base
  max_range         how far the corridor is believed. Measured at 2.5 m in
                    the simulator; past 3 m the range error exceeds the
                    track's own half-width.
  block_h_min/max   the "not drivable" colour window. Measured, not guessed
  block_s_min       -- re-measure on the real venue with hsv_calibrate_node.
"""
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float64
from cv_bridge import CvBridge, CvBridgeError

from physicar_planner import corridor as C

IMAGE_STALE_S = 0.4
PUBLISH_RATE_HZ = 20.0


class PlannerNode(Node):
    def __init__(self):
        super().__init__('planner_node')

        self.declare_parameter('aggression', 4.0)
        self.declare_parameter('lookahead_gain', 0.35)
        self.declare_parameter('lookahead_base', 0.45)
        self.declare_parameter('min_range', C.DEFAULT_MIN_RANGE)
        self.declare_parameter('max_range', C.DEFAULT_MAX_RANGE)
        self.declare_parameter('samples', C.DEFAULT_SAMPLES)
        self.declare_parameter('max_lateral_slope', C.MAX_LATERAL_SLOPE)
        self.declare_parameter('block_h_min', C.BLOCK_H_MIN)
        self.declare_parameter('block_h_max', C.BLOCK_H_MAX)
        self.declare_parameter('block_s_min', C.BLOCK_S_MIN)
        self.declare_parameter('speed_cap', C.MAX_SPEED)
        self.declare_parameter('debug', False)

        g = lambda n: self.get_parameter(n).value          # noqa: E731
        self.aggression = g('aggression')
        self.gain = g('lookahead_gain')
        self.base = g('lookahead_base')
        self.min_range = g('min_range')
        self.max_range = g('max_range')
        self.samples = int(g('samples'))
        self.max_slope = g('max_lateral_slope')
        self.block = (int(g('block_h_min')), int(g('block_h_max')),
                      int(g('block_s_min')))
        self.speed_cap = g('speed_cap')
        self.debug = g('debug')

        self.bridge = CvBridge()
        self.speed_pub = self.create_publisher(Float64, 'plan/speed', 10)
        self.steer_pub = self.create_publisher(Float64, 'plan/steering', 10)
        self.valid_pub = self.create_publisher(Bool, 'plan/valid', 10)

        # Sensor QoS: the camera may publish best-effort, and a reliable
        # subscriber is DDS-incompatible with that -- it receives nothing and
        # says nothing about why.
        self.create_subscription(Image, 'image_raw', self.on_image,
                                 qos_profile_sensor_data)

        self.speed = 0.0
        self.steering = 0.0
        self.valid = False
        self.last_image_time = 0.0
        self.create_timer(1.0 / PUBLISH_RATE_HZ, self.on_tick)

        self.get_logger().info(
            'planner_node ready: aggression %.1f m/s^2, Ld = %.2f*v + %.2f, '
            'corridor %.2f-%.2f m, block H %d-%d S>=%d'
            % (self.aggression, self.gain, self.base, self.min_range,
               self.max_range, self.block[0], self.block[1], self.block[2]))

    def on_image(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as e:
            self.get_logger().warn('cv_bridge failed, dropping frame: %s' % e)
            return
        self.last_image_time = time.time()

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        speed, steer, dbg = C.plan(
            hsv, self.speed, self.aggression, self.gain, self.base,
            min_range=self.min_range, max_range=self.max_range,
            samples=self.samples, block=self.block, max_slope=self.max_slope)

        self.valid = dbg['target'] is not None
        self.speed = min(speed, self.speed_cap)
        self.steering = steer

        if self.debug:
            t = dbg['target']
            self.get_logger().info(
                'Ld %.2f  target %s  corner R %.2f  -> %.2f m/s %+.1f deg'
                % (dbg['lookahead'],
                   '(%.2f, %+.2f)' % t if t else 'none',
                   dbg.get('corner_radius', float('inf')),
                   self.speed, np.degrees(self.steering)))

    def on_tick(self):
        # Published on a timer rather than per frame so a dead camera shows
        # up as a stream of zeros, not as silence that something downstream
        # might read as "no news".
        fresh = (time.time() - self.last_image_time) < IMAGE_STALE_S
        if not fresh:
            self.speed, self.steering, self.valid = 0.0, 0.0, False
        self.speed_pub.publish(Float64(data=float(self.speed)))
        self.steer_pub.publish(Float64(data=float(self.steering)))
        self.valid_pub.publish(Bool(data=bool(self.valid)))


def main():
    rclpy.init()
    node = PlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.speed_pub.publish(Float64(data=0.0))
        except Exception:
            pass
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
