#!/usr/bin/env python3
"""SUPERSEDED -- not started by any deployment launch. Kept as a fallback.

An obstacle is now a place where the drivable corridor is narrower, which
physicar_planner handles without a separate avoidance layer, so there is no
override to arbitrate and no control handed back mid-corner. Only
autonomy_launch.py (the practice chassis) still starts this node.

Note the parameters here carry PHYSICAR values -- front_offset_deg 0.0 and
avoid_steer_sign 1.0. The practice chassis has its lidar mounted backwards
and its servo wired inverted, so autonomy_launch.py overrides both to 180.0
and -1.0. Do not copy one platform's numbers to the other.

Reactive obstacle-avoidance node: watches /scan and decides how much the
front cone is worth overriding steering for.

Does NOT publish /speed or /steering directly anymore (it used to, in the
first cut of this node -- see git history). Now it publishes an advisory
speed cap + a steering override that only applies while something is close
enough to matter, so a judgment_node (physicar_judgment) can combine it with
lane-following and the traffic-light gate instead of this node unilaterally
owning the wheel:

  obstacle/speed_cap       (Float64, m/s)  -- max safe speed right now
                                               (0 in the stop zone, avoid_speed
                                               in the avoid zone, forward_speed
                                               when clear). Always valid.
  obstacle/steer_override  (Float64, rad)   -- steering to use *only* when
                                               override_active is true.
  obstacle/override_active (Bool)           -- true while front_min is inside
                                               avoid_distance (avoid or stop
                                               zone), meaning this node's
                                               steering should win over
                                               whatever physicar_vision's lane
                                               node suggested.

GEOMETRY DEFAULTS TARGET PHYSICAR, NOT THE PRACTICE CHASSIS (2026-08-18).
The defaults below describe the contest vehicle, because that is what the
code has to be right about on race day:
  - front_offset_deg = 0: physicar.urdf.xacro mounts the lidar with
    rpy="0 0 0", so scan angle 0 already points forward.
  - avoid_steer_sign = +1: positive /steering turns left. In the simulator
    cmd_vel_adapter_node.py publishes angular.z = v*tan(steering)/wheelbase,
    which is REP-103 positive-is-counter-clockwise; on the real car the
    driver's steering channel is left non-inverted.

The practice RC car is the odd one out -- its lidar is mounted backwards and
its servo wiring makes positive /steering turn right, both confirmed by
bench test on 2026-08-14. Those values now live in autonomy_launch.py (the
practice-platform launch) rather than here, so that running on the practice
car needs an override and running on Physicar does not. They were previously
the defaults, which meant the sim and the real vehicle both got a lidar
pointed at their own back bumper.

Verify rather than trust: tools/YS_steer_check.py drives a known steering
command and reports which way the car actually turned. Run it once in the
simulator, and again on 2026-08-25 when the real car arrives.
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float64

FORWARD_SPEED_MPS = 0.3      # confirmed above the ESC forward deadzone (2026-08-14 bench test)
AVOID_SPEED_MPS = 0.15
STOP_DISTANCE_M = 0.35
AVOID_DISTANCE_M = 0.7
FRONT_HALF_ANGLE_DEG = 30.0
FRONT_OFFSET_DEG = 0.0        # Physicar mounts the lidar facing forward: physicar.urdf.xacro gives lidar_joint rpy="0 0 0", so scan angle 0 is +x. See below for the practice chassis.
AVOID_STEER_DEG = 15.0
AVOID_STEER_SIGN = 1.0        # Positive /steering turns LEFT on Physicar (REP-103): cmd_vel_adapter_node.py computes angular.z = v*tan(steering)/L, and the real driver leaves its steering channel non-inverted. See below for the practice chassis.
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

        self.speed_cap_pub = self.create_publisher(Float64, 'obstacle/speed_cap', 10)
        self.steer_override_pub = self.create_publisher(Float64, 'obstacle/steer_override', 10)
        self.active_pub = self.create_publisher(Bool, 'obstacle/override_active', 10)

        self.last_scan = None
        self.last_scan_time = 0.0
        # Real Physicar publishes /scan best-effort (confirmed via official
        # physicar-ros docs, 2026-08-18); a default-QoS (reliable) subscriber
        # is INCOMPATIBLE with a best-effort publisher in DDS and silently
        # receives nothing, which made speed_cap stick at 0.0 forever (see
        # on_tick's "no scan" branch) -- i.e. the car would never move at all
        # on the real vehicle. qos_profile_sensor_data is safe to use even
        # against a reliable publisher (e.g. the practice sllidar_ros2 driver).
        self.create_subscription(LaserScan, 'scan', self.on_scan, qos_profile_sensor_data)
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
            # No/stale lidar data: don't advise a nonzero speed cap, and don't
            # claim steering authority either -- judgment_node treats a stale
            # obstacle/* input as "can't trust the safety layer" and stops
            # regardless of override_active, so this is belt-and-suspenders.
            self.publish(0.0, 0.0, False)
            return

        front_min, left_avg, right_avg = self.front_min_and_sides(scan)

        if front_min is None:
            # Nothing detected in the front cone within range: treat as clear.
            self.publish(self.forward_speed, 0.0, False)
            return

        if front_min < self.stop_distance:
            self.publish(0.0, 0.0, True)
            return

        if front_min < self.avoid_distance:
            # Steer toward whichever side has more room; unknown side defaults to 0 (no room info).
            left_room = left_avg if left_avg is not None else 0.0
            right_room = right_avg if right_avg is not None else 0.0
            steer = self.avoid_steer_rad * self.avoid_steer_sign
            if right_room > left_room:
                steer = -steer
            self.publish(self.avoid_speed, steer, True)
            return

        self.publish(self.forward_speed, 0.0, False)

    def publish(self, speed_cap: float, steer_override: float, override_active: bool):
        self.speed_cap_pub.publish(Float64(data=speed_cap))
        self.steer_override_pub.publish(Float64(data=steer_override))
        self.active_pub.publish(Bool(data=override_active))


def main():
    rclpy.init()
    node = ObstacleAvoidNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.publish(0.0, 0.0, True)
        except Exception:
            pass
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
