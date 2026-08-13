#!/usr/bin/env python3
import math
import struct
import serial
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField
from geometry_msgs.msg import Quaternion

FRAME_LEN = {0x14: 25, 0x2c: 49}  # header(3) + 8 unknown + 4*N floats + 2 trailer


def eul_to_qua(roll, pitch, yaw):
    a, b, c = roll / 2.0, pitch / 2.0, yaw / 2.0
    ca, cb, cc = math.cos(a), math.cos(b), math.cos(c)
    sa, sb, sc = math.sin(a), math.sin(b), math.sin(c)
    q = Quaternion()
    q.x = sa * cb * cc - ca * sb * sc
    q.y = ca * sb * cc + sa * cb * sc
    q.z = ca * cb * sc - sa * sb * cc
    q.w = ca * cb * cc + sa * sb * sc
    return q


class HfiImuNode(Node):
    def __init__(self):
        super().__init__('hfi_imu_node')
        self.declare_parameter('port', '/dev/imu')
        self.declare_parameter('baud', 921600)
        self.declare_parameter('frame_id', 'imu_link')
        port = self.get_parameter('port').value
        baud = self.get_parameter('baud').value
        self.frame_id = self.get_parameter('frame_id').value

        self.imu_pub = self.create_publisher(Imu, 'imu/data', 10)
        self.mag_pub = self.create_publisher(MagneticField, 'imu/mag', 10)
        self.ser = serial.Serial(port, baud, timeout=0.1)
        self.buf = bytearray()

        self.gyro = [0.0, 0.0, 0.0]
        self.accel = [0.0, 0.0, 0.0]
        self.mag = [0.0, 0.0, 0.0]

        self.timer = self.create_timer(0.002, self.poll)
        self.get_logger().info(f'HFI IMU node started on {port} @ {baud}')

    def poll(self):
        data = self.ser.read(512)
        if data:
            self.buf.extend(data)

        while True:
            idx = self.buf.find(b'\xaa\x55')
            if idx == -1:
                if len(self.buf) > 1:
                    del self.buf[:len(self.buf) - 1]
                return
            if idx > 0:
                del self.buf[:idx]

            if len(self.buf) < 3:
                return
            ftype = self.buf[2]
            flen = FRAME_LEN.get(ftype)
            if flen is None:
                del self.buf[:2]
                continue
            if len(self.buf) < flen:
                return

            frame = bytes(self.buf[:flen])
            del self.buf[:flen]
            self.handle_frame(ftype, frame, flen)

    def handle_frame(self, ftype, frame, flen):
        n_floats = (flen - 11 - 2) // 4
        floats = struct.unpack('<' + 'f' * n_floats, frame[11:11 + 4 * n_floats])

        if ftype == 0x2c:
            self.gyro = list(floats[0:3])
            self.accel = list(floats[3:6])
            self.mag = list(floats[6:9])
        elif ftype == 0x14:
            roll = floats[0] / 180.0 * math.pi
            pitch = floats[1] / -180.0 * math.pi
            yaw = floats[2] / -180.0 * math.pi
            self.publish(roll, pitch, yaw)

    def publish(self, roll, pitch, yaw):
        stamp = self.get_clock().now().to_msg()

        imu_msg = Imu()
        imu_msg.header.stamp = stamp
        imu_msg.header.frame_id = self.frame_id
        imu_msg.orientation = eul_to_qua(roll, pitch, yaw)
        imu_msg.angular_velocity.x = self.gyro[0]
        imu_msg.angular_velocity.y = self.gyro[1]
        imu_msg.angular_velocity.z = self.gyro[2]
        imu_msg.linear_acceleration.x = self.accel[0] * -9.8
        imu_msg.linear_acceleration.y = self.accel[1] * -9.8
        imu_msg.linear_acceleration.z = self.accel[2] * -9.8
        self.imu_pub.publish(imu_msg)

        mag_msg = MagneticField()
        mag_msg.header.stamp = stamp
        mag_msg.header.frame_id = self.frame_id
        mag_msg.magnetic_field.x = self.mag[0]
        mag_msg.magnetic_field.y = self.mag[1]
        mag_msg.magnetic_field.z = self.mag[2]
        self.mag_pub.publish(mag_msg)


def main():
    rclpy.init()
    node = HfiImuNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
