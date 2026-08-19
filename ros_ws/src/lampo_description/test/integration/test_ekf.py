"""The robot_localization EKF fuses wheel odometry with the IMU.

`use_ekf:=true` is documented in the README but is off by default, which makes
it exactly the kind of option that quietly stops working. The EKF is the only
consumer of the IMU in this repo, and the only component that produces a pose
estimate the way real hardware does -- from noisy measurements rather than from
the simulator's ground truth.
"""

import os
import sys
import unittest

# launch_test executes this file directly, so the directory it lives in is not
# on the import path. Put it there before importing the shared helpers.
sys.path.insert(0, os.path.dirname(__file__))

from helpers import collect, launch_file, NS, simulation  # noqa: E402

from nav_msgs.msg import Odometry  # noqa: E402

import pytest  # noqa: E402

import rclpy  # noqa: E402

TIMEOUT = 120.0


@pytest.mark.launch_test
def generate_test_description():
    """World, robot, and the SLAM launch with the EKF switched on."""
    return simulation(
        launch_file('lampo_gz_mm.launch.py'),
        launch_file('lampo_slam.launch.py', use_ekf='true'),
    )


class TestEkf(unittest.TestCase):
    """Assertions against a running state estimator."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('test_ekf')

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def test_ekf_node_is_running(self):
        """use_ekf:=true actually starts the node."""
        names = [n for n, ns in self.node.get_node_names_and_namespaces()
                 if ns.strip('/') == NS.strip('/')]
        self.assertIn('ekf_filter_node', names,
                      'use_ekf:=true did not start robot_localization')

    def test_ekf_publishes_a_fused_estimate(self):
        """The filter produces output, in this robot's frames.

        Unprefixed frames here would mean the config was not substituted, which
        is what happens when a parameter file keys on a bare node name instead
        of the /** wildcard -- the node then runs on defaults and reports
        `odom` and `base_link` for every robot.
        """
        messages = collect(self.node, f'/{NS}/odometry/filtered', Odometry,
                           count=3, timeout=TIMEOUT)
        self.assertTrue(messages, 'the EKF never published an estimate')

        estimate = messages[-1]
        self.assertEqual(estimate.header.frame_id, f'{NS}odom')
        self.assertEqual(estimate.child_frame_id, f'{NS}base_footprint')

    def test_estimate_carries_a_covariance(self):
        """The output is a real filter estimate, not a passthrough.

        A zero covariance diagonal means the filter never incorporated a
        measurement -- usually because no input topic matched.
        """
        messages = collect(self.node, f'/{NS}/odometry/filtered', Odometry,
                           count=5, timeout=TIMEOUT)
        self.assertTrue(messages, 'the EKF never published an estimate')

        # Diagonal entries of the 6x6 pose covariance.
        diagonal = [messages[-1].pose.covariance[i * 6 + i] for i in range(6)]
        self.assertTrue(any(value > 0.0 for value in diagonal),
                        f'covariance diagonal is all zero: {diagonal}')
