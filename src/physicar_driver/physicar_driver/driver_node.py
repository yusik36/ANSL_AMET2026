#!/usr/bin/env python3
"""Subscribes /speed (Float64, m/s) and /steering (Float64, rad) and forwards
them to the physicar_driver_fw Arduino firmware over serial, matching the
confirmed real-vehicle interface (2026-08-13, contest technical staff):
Ackermann /speed + /steering, not /cmd_vel Twist.

Safety:
- Clamps mirror the real vehicle's hardcoded limits: max speed 3.0 m/s,
  max steering +-20 deg.
- Reverse (negative speed) is not yet bench-tested on this platform, so it
  is clamped to 0 here until validated. Remove the clamp only after testing.
- If either /speed or /steering hasn't been updated within COMMAND_TIMEOUT_S,
  this node stops forwarding fresh values and sends neutral -- independent of
  (and in addition to) the firmware's own 1s watchdog.
"""
import math
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import serial

NEUTRAL_US = 1500

MAX_SPEED_MPS = 3.0
MAX_STEERING_RAD = math.radians(20.0)

# Must match the clamp range baked into physicar_driver_fw.ino.
STEER_MIN_US, STEER_MAX_US = 1300, 1700
ESC_MIN_US, ESC_MAX_US = 1350, 1750

COMMAND_TIMEOUT_S = 1.0
SEND_RATE_HZ = 20.0


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


class PhysicarDriverNode(Node):
    def __init__(self):
        super().__init__('physicar_driver_node')
        self.declare_parameter('port', '/dev/arduino')
        self.declare_parameter('baud', 115200)
        port = self.get_parameter('port').value
        baud = self.get_parameter('baud').value

        self.last_speed = 0.0
        self.last_steering = 0.0
        self.last_speed_time = 0.0
        self.last_steering_time = 0.0

        self.ser = serial.Serial(port, baud, timeout=3.0)
        self._wait_for_ready()

        self.create_subscription(Float64, 'speed', self.on_speed, 10)
        self.create_subscription(Float64, 'steering', self.on_steering, 10)
        self.create_timer(1.0 / SEND_RATE_HZ, self.send_command)

        self.get_logger().info(f'physicar_driver_node ready on {port} @ {baud}')

    def _wait_for_ready(self):
        deadline = time.time() + 5.0
        while time.time() < deadline:
            line = self.ser.readline().decode(errors='replace').strip()
            if line:
                self.get_logger().info(f'[fw boot] {line}')
            if line.startswith('READY'):
                self.ser.reset_input_buffer()
                return
        self.get_logger().warn('Did not see READY from firmware within 5s, continuing anyway')

    def on_speed(self, msg: Float64):
        self.last_speed = msg.data
        self.last_speed_time = time.time()

    def on_steering(self, msg: Float64):
        self.last_steering = msg.data
        self.last_steering_time = time.time()

    def send_command(self):
        now = time.time()
        speed_fresh = (now - self.last_speed_time) < COMMAND_TIMEOUT_S
        steering_fresh = (now - self.last_steering_time) < COMMAND_TIMEOUT_S

        if not (speed_fresh and steering_fresh):
            steer_us = NEUTRAL_US
            esc_us = NEUTRAL_US
        else:
            speed = clamp(self.last_speed, 0.0, MAX_SPEED_MPS)  # reverse untested: clamp to >=0
            steering = clamp(self.last_steering, -MAX_STEERING_RAD, MAX_STEERING_RAD)

            esc_us = int(round(NEUTRAL_US + (speed / MAX_SPEED_MPS) * (ESC_MAX_US - NEUTRAL_US)))
            steer_us = int(round(NEUTRAL_US + (steering / MAX_STEERING_RAD) * (STEER_MAX_US - NEUTRAL_US)))

            esc_us = clamp(esc_us, ESC_MIN_US, ESC_MAX_US)
            steer_us = clamp(steer_us, STEER_MIN_US, STEER_MAX_US)

        line = f'S{steer_us},T{esc_us}\n'
        self.get_logger().info(
            f'send: {line.strip()} speed_fresh={speed_fresh} steering_fresh={steering_fresh} '
            f'last_speed={self.last_speed} last_steering={self.last_steering}'
        )
        try:
            self.ser.write(line.encode())
        except serial.SerialException as e:
            self.get_logger().error(f'serial write failed: {e}')


def main():
    rclpy.init()
    node = PhysicarDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.ser.write(f'S{NEUTRAL_US},T{NEUTRAL_US}\n'.encode())
        except Exception:
            pass
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
