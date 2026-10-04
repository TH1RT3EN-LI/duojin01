"""Exercise the compiled serial driver against a PTY; never opens physical devices."""
import json
import math
import os
import signal
import struct
import subprocess
import time

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu


def frame(vx=1.0, gyro=100):
    data = bytearray([0x7b, 0])
    data.extend(struct.pack('>3h', int(vx * 1000), 0, 0))
    data.extend(struct.pack('>6h', 0, 0, 16384, 0, 0, gyro))
    data.extend(struct.pack('>h', 24000))
    checksum = 0
    for byte in data:
        checksum ^= byte
    return bytes(data + bytearray([checksum, 0x7d]))


def run():
    os.environ['ROS_DOMAIN_ID'] = '91'
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    master, slave = os.openpty()
    path = os.ttyname(slave)
    rclpy.init()
    node = Node('base_feedback_probe')
    odometry, inertial = [], []
    node.create_subscription(Odometry, '/odom', odometry.append, 100)
    node.create_subscription(Imu, '/imu', inertial.append, 100)
    logfile = '/tmp/base-feedback-probe.log'
    with open(logfile, 'w') as log:
        process = subprocess.Popen([
            'ros2', 'run', 'duojin01_base_driver', 'duojin01_base_driver_node', '--ros-args',
            '-p', f'usart_port_name:={path}', '-p', 'serial_baud_rate:=57600',
            '-p', 'loop_hz:=200', '-p', 'imu_yaw_rate_bias:=0.01',
            '-p', 'imu_yaw_rate_variance:=0.02',
            '-p', 'odom_twist_variances_moving:=[0.01, 0.02, 1e6, 1e6, 1e6, 0.03]'
        ], stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            # Wait for DDS discovery with no valid feedback, then send at 10 Hz.
            deadline = time.monotonic() + 10
            while node.count_publishers('/odom') == 0 and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.05)
            assert node.count_publishers('/odom'), f'No driver publisher; see {logfile}'
            for index in range(26):
                os.write(master, frame())
                until = time.monotonic() + 0.1
                while time.monotonic() < until:
                    rclpy.spin_once(node, timeout_sec=0.005)
            for _ in range(20):
                rclpy.spin_once(node, timeout_sec=0.01)
            assert process.poll() is None, f'Driver stopped; see {logfile}'
            before_bad_frame = len(odometry)
            bad = bytearray(frame(vx=4.0))
            bad[22] ^= 0xff
            os.write(master, bad)
            until = time.monotonic() + 0.1
            while time.monotonic() < until:
                rclpy.spin_once(node, timeout_sec=0.005)
            assert len(odometry) == before_bad_frame, 'Bad checksum produced odometry'
            assert len(odometry) >= 24, len(odometry)
            def stamp(message):
                return message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
            elapsed = stamp(odometry[-1]) - stamp(odometry[0])
            distance = odometry[-1].pose.pose.position.x - odometry[0].pose.pose.position.x
            assert abs(distance - elapsed) < 0.04, (elapsed, distance)
            assert abs(odometry[-1].twist.covariance[7] - 0.02) < 1e-9
            assert abs(odometry[-1].twist.covariance[35] - 0.03) < 1e-9
            assert inertial
            assert abs(inertial[-1].angular_velocity.z - (100 * 0.00026644 - 0.01)) < 1e-6
            assert inertial[-1].angular_velocity_covariance[8] == 0.02
            print(json.dumps({'feedback_hz': 10, 'timer_hz': 200, 'samples': len(odometry),
                              'elapsed_seconds': elapsed, 'distance_meters': distance,
                              'ratio': distance / elapsed, 'custom_baud': 57600,
                              'bad_checksum_ignored': True, 'calibration_parameters': 'passed'}, indent=2))
        finally:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()
            node.destroy_node()
            rclpy.shutdown()
            os.close(master)
            os.close(slave)


if __name__ == '__main__':
    run()
