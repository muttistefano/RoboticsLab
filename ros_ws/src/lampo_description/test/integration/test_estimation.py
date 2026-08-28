"""The Kalman tuning lab does what doc/control.md promises.

lampo_slam.launch.py use_ekf:=true noisy:=true wires the EKF to deliberately
corrupted sensors while the simulator's clean odometry stays available as
ground truth. This drives the robot briefly and checks the noisy topics
exist, the filter runs, and its estimate stays near the truth with the
default noise -- the calibrated starting point the exercise degrades from.
"""

import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(__file__))

from geometry_msgs.msg import TwistStamped  # noqa: E402

from helpers import launch_file, NS, simulation, wait_for  # noqa: E402

from nav_msgs.msg import Odometry  # noqa: E402

import pytest  # noqa: E402

import rclpy  # noqa: E402


@pytest.mark.launch_test
def generate_test_description():
    """World, the base alone, and SLAM with the noisy EKF chain."""
    return simulation(
        launch_file('lampo_gz_mm.launch.py', mm='false'),
        launch_file('lampo_slam.launch.py', use_ekf='true', noisy='true'),
    )


class TestEstimation(unittest.TestCase):
    """Assertions against the Kalman tuning exercise."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('test_estimation', parameter_overrides=[
            rclpy.parameter.Parameter('use_sim_time', value=True)])

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def test_noisy_sensors_are_published(self):
        """sensor_noise.py republishes both corrupted topics."""
        for topic in ('odom_noisy', 'imu_noisy'):
            self.assertTrue(
                wait_for(lambda: self.node.count_publishers(
                    f'/{NS}/{topic}') > 0, timeout=90.0, node=self.node),
                f'{topic} never appeared')

    def test_the_filter_tracks_ground_truth(self):
        """odometry/filtered stays near the plugin's exact odometry.

        The EKF sees only the corrupted sensors; the plugin's clean odom is
        the answer sheet. With the default noise and the shipped covariances
        the estimate must stay within half a metre through a short drive.
        """
        truth, filtered = {}, {}

        def keep(store):
            def cb(msg):
                store['x'] = msg.pose.pose.position.x
                store['y'] = msg.pose.pose.position.y
            return cb

        subs = [
            self.node.create_subscription(
                Odometry, f'/{NS}/odom', keep(truth), 10),
            self.node.create_subscription(
                Odometry, f'/{NS}/odometry/filtered', keep(filtered), 10),
        ]
        for sub in subs:
            self.addCleanup(self.node.destroy_subscription, sub)

        self.assertTrue(
            wait_for(lambda: truth and filtered, timeout=90.0,
                     node=self.node),
            'odometry (truth or filtered) never arrived')

        # Drive forward for a few seconds of sim time.
        publisher = self.node.create_publisher(
            TwistStamped, f'/{NS}/cmd_vel_safe', 10)
        self.addCleanup(self.node.destroy_publisher, publisher)
        travelled = dict(truth)

        def driven_far_enough():
            msg = TwistStamped()
            msg.header.stamp = self.node.get_clock().now().to_msg()
            msg.twist.linear.x = 0.3
            publisher.publish(msg)
            return abs(truth['x'] - travelled['x']) > 1.0

        self.assertTrue(
            wait_for(driven_far_enough, timeout=60.0, node=self.node,
                     period=0.1),
            f'the robot never drove; truth: {truth}')

        error = ((truth['x'] - filtered['x']) ** 2
                 + (truth['y'] - filtered['y']) ** 2) ** 0.5
        self.assertLess(
            error, 0.5,
            f'filtered estimate too far from ground truth: {error:.2f} m '
            f'(truth {truth}, filtered {filtered})')
