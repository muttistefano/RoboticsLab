#!/usr/bin/env python3
"""Corrupt the simulator's too-perfect sensors (doc/control.md, exercise 7).

The Kalman filter exercise needs something to filter. The simulated wheel
odometry and IMU are essentially noise-free, so tuning an EKF against them
proves nothing. This node republishes both with configurable zero-mean
gaussian noise -- and, importantly, *says so*: the covariance fields of the
outgoing messages are inflated to match the injected noise, because a
sensor's covariance is its honesty, and the EKF weighs measurements by it.

Started automatically by `lampo_slam.launch.py use_ekf:=true noisy:=true`,
or by hand:

    ros2 run lampo_description sensor_noise.py --ros-args \
        -r __ns:=/r1_ -p use_sim_time:=true

Topics: odom -> odom_noisy, imu -> imu_noisy.
"""

from nav_msgs.msg import Odometry

import numpy as np

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from sensor_msgs.msg import Imu


class SensorNoise(Node):
    """Add gaussian noise to odom and imu; inflate covariances to match."""

    def __init__(self):
        super().__init__('sensor_noise')

        # Standard deviations. Live-tunable, so "turn the noise up until the
        # filter struggles" is a one-line experiment.
        self.declare_parameter('odom_vel_stddev', 0.05)    # m/s and rad/s
        self.declare_parameter('gyro_stddev', 0.02)        # rad/s
        self.declare_parameter('accel_stddev', 0.2)        # m/s^2

        self.rng = np.random.default_rng()

        self.odom_pub = self.create_publisher(Odometry, 'odom_noisy', 10)
        self.imu_pub = self.create_publisher(Imu, 'imu_noisy', 10)
        self.create_subscription(Odometry, 'odom', self.on_odom, 10)
        self.create_subscription(Imu, 'imu', self.on_imu, 10)

        self.get_logger().info('Republishing odom -> odom_noisy, '
                               'imu -> imu_noisy.')

    def noise(self, stddev):
        return float(self.rng.normal(0.0, stddev))

    def on_odom(self, msg: Odometry):
        s = self.get_parameter('odom_vel_stddev').value
        # The EKF is configured to fuse the odometry VELOCITIES (fusing the
        # pose too would double-count the same wheel ticks), so the velocity
        # is what gets corrupted.
        twist = msg.twist.twist
        twist.linear.x += self.noise(s)
        twist.linear.y += self.noise(s)
        twist.angular.z += self.noise(s)
        cov = list(msg.twist.covariance)
        for idx in (0, 7, 35):            # vx, vy, vyaw diagonal entries
            cov[idx] += s * s
        msg.twist.covariance = cov
        self.odom_pub.publish(msg)

    def on_imu(self, msg: Imu):
        gyro = self.get_parameter('gyro_stddev').value
        accel = self.get_parameter('accel_stddev').value
        msg.angular_velocity.z += self.noise(gyro)
        msg.linear_acceleration.x += self.noise(accel)
        msg.linear_acceleration.y += self.noise(accel)
        gyro_cov = list(msg.angular_velocity_covariance)
        gyro_cov[8] += gyro * gyro        # yaw-rate diagonal entry
        msg.angular_velocity_covariance = gyro_cov
        accel_cov = list(msg.linear_acceleration_covariance)
        for idx in (0, 4):                # ax, ay diagonal entries
            accel_cov[idx] += accel * accel
        msg.linear_acceleration_covariance = accel_cov
        self.imu_pub.publish(msg)


def main():
    rclpy.init()
    try:
        rclpy.spin(SensorNoise())
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()
