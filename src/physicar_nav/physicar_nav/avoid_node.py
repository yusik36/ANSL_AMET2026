#!/usr/bin/env python3
"""Reactive test node: drive straight at a slow constant speed, steer away
from whichever side has more clearance when an obstacle enters the front
cone, stop outright if something gets too close.

Publishes /speed (Float64, m/s) + /steering (Float64, rad) -- the same
interface physicar_driver_node expects. Does not know about the driver's
internal deadzone/clamp handling; it just publishes commands.

Two things likely need calibration on the real chassis before this behaves
correctly and are called out below with FIXME:
  - FRONT_OFFSET_DEG: assumes the lidar's zero-angle direction is the
    vehicle's forward direction. If the lidar is mounted rotated relative
    to the chassis, set this so (scan_angle + offset) = 0 means forward.
  - Steering sign: assumes positive /steering turns the same direction as
    positive scan angle (left, per REP-103/105 convention: CCW is
    positive). If the car turns the wrong way, flip AVOID_STEER_SIGN.
"""
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64

FORWARD_SPEED_MPS = 0.3      # confirmed above the ESC forward deadzone (2026-08-14 bench test)
AVOID_SPEED_MPS = 0.15
STOP_DISTANCE_M = 0.35
AVOID_DISTANCE_M = 0.7
FRONT_HALF_ANGLE_DEG = 30.0
FRONT_OFFSET_DEG = 0.0        # FIXME: calibrate against actual lidar mounting
AVOID_STEER_DEG = 15.0
AVOID_STEER_SIGN = 1.0        # FIXME: flip to -1.0 if avoidance turns the wrong way
PUBLISH_RATE_HZ = 20.0
SCAN_STALE_S = 0.5


def normalize_angle(a: float) -> float:
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


class ObstacleAvoidNode(Node):
    def __init__(self):
        super().__init__('obstacle_avoid_node')

        self.declare_parameter('forward_speed', FORWARD_SPEED_MPS)
        self.declare_parameter('avoid_speed', AVOID_SPEED_MPS)
        self.declare_parameter('stop_distance', STOP_DISTANCE_M)
        self.declare_parameter('avoid_distance', AVOID_DISTANCE_M)
        self.declare_parameter('front_half_angle_deg', FRONT_HALF_ANGLE_DEG)
        self.declare_parameter('front_offset_deg', FRONT_OFFSET_DEG)
        self.declare_parameter('avoid_steer_deg', AVOID_STEER_DEG)
        self.declare_parameter('avoid_steer_sign', AVOID_STEER_SIGN)

        self.forward_speed = self.get_parameter('forward_speed').value
        self.avoid_speed = self.get_parameter('avoid_speed').value
        self.stop_distance = self.get_parameter('stop_distance').value
        self.avoid_distance = self.get_parameter('avoid_distance').value
        self.front_half_angle = math.radians(self.get_parameter('front_half_angle_deg').value)
        self.front_offset = math.radians(self.get_parameter('front_offset_deg').value)
        self.avoid_steer_rad = math.radians(self.get_parameter('avoid_steer_deg').value)
        self.avoid_steer_sign = self.get_parameter('avoid_steer_sign').value

        self.speed_pub = self.create_publisher(Float64, 'speed', 10)
        self.steering_pub = self.create_publisher(Float64, 'steering', 10)

        self.last_scan = None
        self.last_scan_time = 0.0
        self.create_subscription(LaserScan, 'scan', self.on_scan, 10)
        self.create_timer(1.0 / PUBLISH_RATE_HZ, self.on_tick)

        self.get_logger().info(
            f'obstacle_avoid_node ready: forward={self.forward_speed} m/s, '
            f'avoid<{self.avoid_distance}m, stop<{self.stop_distance}m'
        )

    def on_scan(self, msg: LaserScan):
        self.last_scan = msg
        self.last_scan_time = self.get_clock().now().nanoseconds / 1e9

    def front_min_and_sides(self, scan: LaserScan):
        """Return (front_min, left_avg, right_avg), None entries if no valid points."""
        front_min = None
        left_vals = []
        right_vals = []

        angle = scan.angle_min
        for r in scan.ranges:
            a = normalize_angle(angle - self.front_offset)
            angle += scan.angle_increment

            if not math.isfinite(r) or r < scan.range_min or r > scan.range_max:
                continue
            if abs(a) > self.front_half_angle:
                continue

            if front_min is None or r < front_min:
                front_min = r
            if a >= 0.0:
                left_vals.append(r)
            else:
                right_vals.append(r)

        left_avg = sum(left_vals) / len(left_vals) if left_vals else None
        right_avg = sum(right_vals) / len(right_vals) if right_vals else None
        return front_min, left_avg, right_avg

    def on_tick(self):
        now = self.get_clock().now().nanoseconds / 1e9
        scan = self.last_scan
        if scan is None or (now - self.last_scan_time) > SCAN_STALE_S:
            self.publish(0.0, 0.0)
            return

        front_min, left_avg, right_avg = self.front_min_and_sides(scan)

        if front_min is None:
            # Nothing detected in the front cone within range: treat as clear.
            self.publish(self.forward_speed, 0.0)
            return

        if front_min < self.stop_distance:
            self.publish(0.0, 0.0)
            return

        if front_min < self.avoid_distance:
            # Steer toward whichever side has more room; unknown side defaults to 0 (no room info).
            left_room = left_avg if left_avg is not None else 0.0
            right_room = right_avg if right_avg is not None else 0.0
            steer = self.avoid_steer_rad * self.avoid_steer_sign
            if right_room > left_room:
                steer = -steer
            self.publish(self.avoid_speed, steer)
            return

        self.publish(self.forward_speed, 0.0)

    def publish(self, speed: float, steering: float):
        self.speed_pub.publish(Float64(data=speed))
        self.steering_pub.publish(Float64(data=steering))


def main():
    rclpy.init()
    node = ObstacleAvoidNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.publish(0.0, 0.0)
        except Exception:
            pass
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
