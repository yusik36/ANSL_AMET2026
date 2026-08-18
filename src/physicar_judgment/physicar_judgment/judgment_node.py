#!/usr/bin/env python3
"""Final arbitration: gates the corridor planner with the traffic light and
publishes the /speed + /steering the driver forwards to the hardware.

This node used to merge a lane follower with a separate obstacle-avoidance
layer, deciding frame by frame which of them owned the steering. That split
is gone: physicar_planner reads the drivable corridor and an obstacle is
simply a place where it narrows, so there is one steering proposal and no
handover to arbitrate. What is left here is the part that still needs to sit
above the planner:

  1. Failsafe -- if plan/* has gone stale the car has no view of the track
     at all, so stop. This overrides everything below.
  2. Traffic-light gate -- only ever zeroes *speed*, never fights steering,
     so the car is already pointed the right way the instant the light
     allows it to move again.
  3. Otherwise pass the planner through, holding the last good steering
     briefly if it reports no target rather than snapping to straight.

Inputs (subscribed):
  plan/speed                 Float64   -- see physicar_planner/planner_node.py
  plan/steering              Float64
  plan/valid                 Bool
  traffic/light_state        String    -- see physicar_vision/traffic_light_node.py
  traffic/valid               Bool

Output (published):
  speed      Float64  m/s   -- what physicar_driver_node subscribes to
  steering   Float64  rad

Parameters:
  require_traffic_gate (bool, default True)  -- when False, traffic state is
      ignored entirely (always treated as "go"), for bench tests with no
      light in view.
  plan_hold_s           (float, default 0.5) -- grace period to keep steering
      toward the last known-good reading after plan/valid drops, before
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

DEFAULT_PLAN_HOLD_S = 0.5
DEFAULT_INPUT_STALE_S = 0.5
PUBLISH_RATE_HZ = 20.0


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


class JudgmentNode(Node):
    def __init__(self):
        super().__init__('judgment_node')

        self.declare_parameter('require_traffic_gate', True)
        self.declare_parameter('plan_hold_s', DEFAULT_PLAN_HOLD_S)
        self.declare_parameter('input_stale_s', DEFAULT_INPUT_STALE_S)

        self.require_traffic_gate = self.get_parameter('require_traffic_gate').value
        self.plan_hold_s = self.get_parameter('plan_hold_s').value
        self.input_stale_s = self.get_parameter('input_stale_s').value

        # plan/*
        self.plan_speed = 0.0
        self.plan_steering = 0.0
        self.plan_valid = False
        self.plan_last_time = 0.0
        self.plan_last_good_steering = 0.0
        self.plan_last_good_time = 0.0

        # traffic/*
        self.traffic_state = 'NONE'
        self.traffic_valid = False
        self.traffic_last_time = 0.0

        self.create_subscription(Float64, 'plan/speed', self.on_plan_speed, 10)
        self.create_subscription(Float64, 'plan/steering', self.on_plan_steering, 10)
        self.create_subscription(Bool, 'plan/valid', self.on_plan_valid, 10)
        self.create_subscription(String, 'traffic/light_state', self.on_traffic_state, 10)
        self.create_subscription(Bool, 'traffic/valid', self.on_traffic_valid, 10)

        self.speed_pub = self.create_publisher(Float64, 'speed', 10)
        self.steering_pub = self.create_publisher(Float64, 'steering', 10)

        self.create_timer(1.0 / PUBLISH_RATE_HZ, self.on_tick)

        self.get_logger().info(
            f'judgment_node ready: require_traffic_gate='
            f'{self.require_traffic_gate}, plan_hold={self.plan_hold_s}s'
        )

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    # -- plan/* --
    def on_plan_speed(self, msg: Float64):
        self.plan_speed = msg.data
        self.plan_last_time = self._now()

    def on_plan_steering(self, msg: Float64):
        self.plan_steering = msg.data
        self.plan_last_time = self._now()

    def on_plan_valid(self, msg: Bool):
        self.plan_valid = msg.data
        self.plan_last_time = self._now()
        if self.plan_valid:
            self.plan_last_good_steering = self.plan_steering
            self.plan_last_good_time = self._now()

    # -- traffic/* --
    def on_traffic_state(self, msg: String):
        self.traffic_state = msg.data
        self.traffic_last_time = self._now()

    def on_traffic_valid(self, msg: Bool):
        self.traffic_valid = msg.data
        self.traffic_last_time = self._now()

    def on_tick(self):
        now = self._now()

        plan_fresh = (now - self.plan_last_time) < self.input_stale_s
        if not plan_fresh:
            # The planner is the whole of perception now. If it has gone
            # quiet there is no view of the track at all, so stop rather
            # than coast on the last thing it said.
            self.publish(0.0, 0.0)
            return

        # 1. Steering comes from the planner. When it reports no target --
        #    view blocked, corridor closed -- the last good steering is held
        #    briefly rather than snapping to straight, because a momentary
        #    dropout mid-corner is common and straightening into one is not
        #    a recovery. If it stays lost, stop.
        force_stop_plan_lost = False
        if self.plan_valid:
            steering = self.plan_steering
        elif (now - self.plan_last_good_time) < self.plan_hold_s:
            steering = self.plan_last_good_steering
        else:
            steering = 0.0
            force_stop_plan_lost = True

        # 2. Speed likewise, zeroed by either gate below.
        speed = self.plan_speed

        if force_stop_plan_lost:
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
