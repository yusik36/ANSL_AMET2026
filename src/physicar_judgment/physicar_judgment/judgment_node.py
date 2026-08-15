#!/usr/bin/env python3
"""Final arbitration: combines physicar_nav (obstacle avoidance),
physicar_vision (lane following + traffic light), and publishes the actual
/speed + /steering that physicar_driver_node forwards to the real hardware.

Priority (highest first), matching the design discussed for this pipeline:
  1. Obstacle avoidance / stop safety -- if obstacle/* itself is stale (the
     avoid_node isn't running or /scan died), we cannot trust ANY safety
     margin, so this unconditionally overrides everything else and stops.
     If obstacle/override_active is true (something is inside the avoid/stop
     zone), its steering wins over lane-following's suggestion.
  2. Traffic-light gate -- only ever zeroes *speed*, never fights steering,
     so the car is already pointed the right way the instant the light
     allows it to move again.
  3. Lane following -- supplies steering whenever it isn't overridden by (1)
     and its own /lane/valid is currently true. If lane tracking is lost,
     the last known steering is held briefly (lane_hold_s) rather than
     snapping to 0, then the car stops if it's still lost after that.

Inputs (subscribed):
  obstacle/speed_cap        Float64   -- see physicar_nav/avoid_node.py
  obstacle/steer_override   Float64
  obstacle/override_active  Bool
  lane/steering              Float64   -- see physicar_vision/lane_follow_node.py
  lane/valid                 Bool
  traffic/light_state        String    -- see physicar_vision/traffic_light_node.py
  traffic/valid               Bool

Output (published):
  speed      Float64  m/s   -- what physicar_driver_node subscribes to
  steering   Float64  rad

Parameters:
  require_lane_gate    (bool, default True)  -- when False, lane/valid is
      ignored and steering just falls back to 0 (straight) whenever obstacle
      avoidance isn't overriding. Used by physicar_nav/avoid_test_launch.py
      for a lidar-only bench test with no camera running.
  require_traffic_gate (bool, default True)  -- when False, traffic state is
      ignored entirely (always treated as "go"). Same bench-test use case.
  lane_hold_s           (float, default 0.5) -- grace period to keep steering
      toward the last known-good lane reading after lane/valid drops, before
      forcing a stop.
  input_stale_s          (float, default 0.5) -- how old an obstacle/lane/
      traffic message can be before it's treated as "that subsystem is
      down" rather than "that subsystem currently reports X".

This node is intentionally stateless with respect to the vehicle's position
on the track: the only memory it keeps (last-known steering, per-topic
timestamps) is short-lived and self-expiring, so behavior is correct
immediately after the car is physically picked up and placed anywhere on
the track (competition's Stateless requirement) -- it never assumes "we
started at point X".
"""
import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float64, String

MAX_SPEED_MPS = 3.0
MAX_STEERING_RAD = math.radians(20.0)

DEFAULT_LANE_HOLD_S = 0.5
DEFAULT_INPUT_STALE_S = 0.5
PUBLISH_RATE_HZ = 20.0


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


class JudgmentNode(Node):
    def __init__(self):
        super().__init__('judgment_node')

        self.declare_parameter('require_lane_gate', True)
        self.declare_parameter('require_traffic_gate', True)
        self.declare_parameter('lane_hold_s', DEFAULT_LANE_HOLD_S)
        self.declare_parameter('input_stale_s', DEFAULT_INPUT_STALE_S)

        self.require_lane_gate = self.get_parameter('require_lane_gate').value
        self.require_traffic_gate = self.get_parameter('require_traffic_gate').value
        self.lane_hold_s = self.get_parameter('lane_hold_s').value
        self.input_stale_s = self.get_parameter('input_stale_s').value

        # obstacle/*
        self.obstacle_speed_cap = 0.0
        self.obstacle_steer_override = 0.0
        self.obstacle_override_active = False
        self.obstacle_last_time = 0.0

        # lane/*
        self.lane_steering = 0.0
        self.lane_valid = False
        self.lane_last_time = 0.0
        self.lane_last_good_steering = 0.0
        self.lane_last_good_time = 0.0

        # traffic/*
        self.traffic_state = 'NONE'
        self.traffic_valid = False
        self.traffic_last_time = 0.0

        self.create_subscription(Float64, 'obstacle/speed_cap', self.on_obstacle_speed_cap, 10)
        self.create_subscription(Float64, 'obstacle/steer_override', self.on_obstacle_steer_override, 10)
        self.create_subscription(Bool, 'obstacle/override_active', self.on_obstacle_override_active, 10)
        self.create_subscription(Float64, 'lane/steering', self.on_lane_steering, 10)
        self.create_subscription(Bool, 'lane/valid', self.on_lane_valid, 10)
        self.create_subscription(String, 'traffic/light_state', self.on_traffic_state, 10)
        self.create_subscription(Bool, 'traffic/valid', self.on_traffic_valid, 10)

        self.speed_pub = self.create_publisher(Float64, 'speed', 10)
        self.steering_pub = self.create_publisher(Float64, 'steering', 10)

        self.create_timer(1.0 / PUBLISH_RATE_HZ, self.on_tick)

        self.get_logger().info(
            f'judgment_node ready: require_lane_gate={self.require_lane_gate} '
            f'require_traffic_gate={self.require_traffic_gate}'
        )

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    # -- obstacle/* --
    def on_obstacle_speed_cap(self, msg: Float64):
        self.obstacle_speed_cap = msg.data
        self.obstacle_last_time = self._now()

    def on_obstacle_steer_override(self, msg: Float64):
        self.obstacle_steer_override = msg.data
        self.obstacle_last_time = self._now()

    def on_obstacle_override_active(self, msg: Bool):
        self.obstacle_override_active = msg.data
        self.obstacle_last_time = self._now()

    # -- lane/* --
    def on_lane_steering(self, msg: Float64):
        self.lane_steering = msg.data
        self.lane_last_time = self._now()

    def on_lane_valid(self, msg: Bool):
        self.lane_valid = msg.data
        self.lane_last_time = self._now()
        if self.lane_valid:
            self.lane_last_good_steering = self.lane_steering
            self.lane_last_good_time = self._now()

    # -- traffic/* --
    def on_traffic_state(self, msg: String):
        self.traffic_state = msg.data
        self.traffic_last_time = self._now()

    def on_traffic_valid(self, msg: Bool):
        self.traffic_valid = msg.data
        self.traffic_last_time = self._now()

    def on_tick(self):
        now = self._now()

        obstacle_fresh = (now - self.obstacle_last_time) < self.input_stale_s
        if not obstacle_fresh:
            # Can't trust the safety layer at all -- stop unconditionally,
            # regardless of lane/traffic gate settings.
            self.publish(0.0, 0.0)
            return

        # 1. Steering: obstacle override wins outright; otherwise lane
        #    following, with a short hold if lane tracking just dropped.
        lane_fresh = (now - self.lane_last_time) < self.input_stale_s
        force_stop_lane_lost = False

        if self.obstacle_override_active:
            steering = self.obstacle_steer_override
        elif not self.require_lane_gate:
            steering = 0.0
        elif lane_fresh and self.lane_valid:
            steering = self.lane_steering
        elif (now - self.lane_last_good_time) < self.lane_hold_s:
            steering = self.lane_last_good_steering
        else:
            steering = 0.0
            force_stop_lane_lost = True

        # 2. Speed: obstacle's advisory cap is the base (already encodes the
        #    stop/avoid/forward tiers), reduced to 0 by either gate below.
        speed = self.obstacle_speed_cap

        if self.require_lane_gate and force_stop_lane_lost:
            speed = 0.0

        if self.require_traffic_gate:
            traffic_fresh = (now - self.traffic_last_time) < self.input_stale_s
            if not traffic_fresh or not self.traffic_valid or self.traffic_state == 'RED':
                speed = 0.0

        speed = clamp(speed, 0.0, MAX_SPEED_MPS)
        steering = clamp(steering, -MAX_STEERING_RAD, MAX_STEERING_RAD)
        self.publish(speed, steering)

    def publish(self, speed: float, steering: float):
        self.speed_pub.publish(Float64(data=speed))
        self.steering_pub.publish(Float64(data=steering))


def main():
    rclpy.init()
    node = JudgmentNode()
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
