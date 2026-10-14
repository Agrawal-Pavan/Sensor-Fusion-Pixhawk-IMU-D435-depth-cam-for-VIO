#!/usr/bin/env python3
"""
pixhawk_imu_pub.py
Reads MAVLink IMU/Attitude messages from Pixhawk and publishes sensor_msgs/msg/Imu in ROS ENU/FLU frames.
"""

import math
import sys
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu

try:
    from pymavlink import mavutil
except ImportError:
    print("pymavlink is not installed! Run: pip install pymavlink", file=sys.stderr)
    sys.exit(1)


# Transformation matrices
# A maps NED world to ENU world: X_enu = Y_ned, Y_enu = X_ned, Z_enu = -Z_ned
A = np.array([
    [0.0, 1.0,  0.0],
    [1.0, 0.0,  0.0],
    [0.0, 0.0, -1.0]
], dtype=np.float64)

# B maps FRD body to FLU body: X_flu = X_frd, Y_flu = -Y_frd, Z_flu = -Z_frd
B = np.array([
    [1.0,  0.0,  0.0],
    [0.0, -1.0,  0.0],
    [0.0,  0.0, -1.0]
], dtype=np.float64)


def quat_to_rot_matrix(w, x, y, z):
    """Convert quaternion [w, x, y, z] to 3x3 rotation matrix."""
    return np.array([
        [1.0 - 2.0*(y*y + z*z), 2.0*(x*y - z*w),       2.0*(x*z + y*w)],
        [2.0*(x*y + z*w),       1.0 - 2.0*(x*x + z*z), 2.0*(y*z - x*w)],
        [2.0*(x*z - y*w),       2.0*(y*z + x*w),       1.0 - 2.0*(x*x + y*y)]
    ], dtype=np.float64)


def rot_matrix_to_quat(R):
    """Convert 3x3 rotation matrix to quaternion [w, x, y, z]."""
    tr = R[0, 0] + R[1, 1] + R[2, 2]
    if tr > 0.0:
        s = 0.5 / math.sqrt(tr + 1.0)
        w = 0.25 / s
        x = (R[2, 1] - R[1, 2]) * s
        y = (R[0, 2] - R[2, 0]) * s
        z = (R[1, 0] - R[0, 1]) * s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = 2.0 * math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = 2.0 * math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = 2.0 * math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    return w, x, y, z


class PixhawkImuPub(Node):
    def __init__(self):
        super().__init__('pixhawk_imu_pub')

        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baud', 57600)
        self.declare_parameter('frame_id', 'base_link')

        port = self.get_parameter('port').value
        baud = self.get_parameter('baud').value
        self.frame_id = self.get_parameter('frame_id').value

        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        self.pub = self.create_publisher(Imu, '/imu/data', qos)

        self.get_logger().info(f"Connecting to Pixhawk MAVLink on {port} @ {baud} baud...")
        try:
            self.mav = mavutil.mavlink_connection(port, baud=baud)
            self.mav.wait_heartbeat(timeout=5)
            self.get_logger().info("Connected to Pixhawk!")
        except Exception as e:
            self.get_logger().warn(f"Failed to connect to {port}: {e}. Retrying in loop...")
            self.mav = None

        self.timer = self.create_timer(0.01, self.read_mavlink)  # 100 Hz

    def read_mavlink(self):
        if not self.mav:
            return

        while True:
            msg = self.mav.recv_match(type=['ATTITUDE_QUATERNION', 'ATTITUDE', 'HIGHRES_IMU'], blocking=False)
            if msg is None:
                break

            msg_type = msg.get_type()
            now = self.get_clock().now().to_msg()

            if msg_type in ('ATTITUDE_QUATERNION', 'ATTITUDE'):
                imu_msg = Imu()
                imu_msg.header.stamp = now
                imu_msg.header.frame_id = self.frame_id

                if msg_type == 'ATTITUDE_QUATERNION':
                    # MAVLink quat: q1=w, q2=x, q3=y, q4=z in NED/FRD
                    w, x, y, z = msg.q1, msg.q2, msg.q3, msg.q4
                else:
                    # ATTITUDE gives roll, pitch, yaw (rad)
                    cr = math.cos(msg.roll * 0.5)
                    sr = math.sin(msg.roll * 0.5)
                    cp = math.cos(msg.pitch * 0.5)
                    sp = math.sin(msg.pitch * 0.5)
                    cy = math.cos(msg.yaw * 0.5)
                    sy = math.sin(msg.yaw * 0.5)
                    w = cr * cp * cy + sr * sp * sy
                    x = sr * cp * cy - cr * sp * sy
                    y = cr * sp * cy + sr * cp * sy
                    z = cr * cp * sy - sr * sp * cy

                # Convert rotation from NED/FRD to ENU/FLU: R_enu = A * R_ned * B
                R_ned = quat_to_rot_matrix(w, x, y, z)
                R_enu = A @ R_ned @ B
                qw, qx, qy, qz = rot_matrix_to_quat(R_enu)

                imu_msg.orientation.w = float(qw)
                imu_msg.orientation.x = float(qx)
                imu_msg.orientation.y = float(qy)
                imu_msg.orientation.z = float(qz)
                # Filtered attitude covariance (~0.01 rad^2)
                imu_msg.orientation_covariance[0] = 0.01
                imu_msg.orientation_covariance[4] = 0.01
                imu_msg.orientation_covariance[8] = 0.01

                # Angular velocity in body FLU (flip y and z from FRD)
                imu_msg.angular_velocity.x = float(msg.rollspeed)
                imu_msg.angular_velocity.y = float(-msg.pitchspeed)
                imu_msg.angular_velocity.z = float(-msg.yawspeed)
                imu_msg.angular_velocity_covariance[0] = 0.001
                imu_msg.angular_velocity_covariance[4] = 0.001
                imu_msg.angular_velocity_covariance[8] = 0.001

                # ATTITUDE carries no linear acceleration; mark as invalid for EKF
                imu_msg.linear_acceleration_covariance[0] = -1.0

                self.pub.publish(imu_msg)


def main(args=None):
    rclpy.init(args=args)
    node = PixhawkImuPub()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
