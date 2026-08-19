"""Two robots share one simulation without colliding in the ROS graph.

This is block 4 of the demo, and the claim the README makes about namespacing:
the same launch file, a different `namespace:`, and everything -- topics,
frames, controllers -- is prefixed. It only holds if nothing anywhere is
hardcoded to r1_, which is exactly the kind of thing that rots silently.
"""

import os
import sys
import unittest

# launch_test executes this file directly, so the directory it lives in is not
# on the import path. Put it there before importing the shared helpers.
sys.path.insert(0, os.path.dirname(__file__))

from helpers import collect, launch_file, simulation, wait_for  # noqa: E402

from nav_msgs.msg import Odometry  # noqa: E402

import pytest  # noqa: E402

import rclpy  # noqa: E402

ROBOTS = ['r1_', 'r2_']
TIMEOUT = 120.0


@pytest.mark.launch_test
def generate_test_description():
    """One world, two robots, spawned apart from each other."""
    return simulation(
        launch_file('lampo_gz_mm.launch.py'),
        launch_file('lampo_gz_mm.launch.py', namespace='r2_', x=-2.0, y=1.0),
    )


class TestMultiRobot(unittest.TestCase):
    """Assertions against two simultaneously spawned robots."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('test_multirobot')

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def test_both_robots_publish_odometry(self):
        """Each robot reports its own pose, in its own frames."""
        for namespace in ROBOTS:
            with self.subTest(robot=namespace):
                messages = collect(self.node, f'/{namespace}/odom', Odometry,
                                   timeout=TIMEOUT)
                self.assertTrue(messages, f'/{namespace}/odom never published')
                self.assertEqual(messages[0].header.frame_id, f'{namespace}odom')
                self.assertEqual(messages[0].child_frame_id,
                                 f'{namespace}base_footprint')

    def test_neither_robot_leaks_into_the_global_namespace(self):
        """No robot topic escapes its namespace.

        An unprefixed /odom or /joint_states would mean the two robots are
        writing over each other, which looks like a physics bug rather than a
        naming one.
        """
        topics = {name for name, _ in self.node.get_topic_names_and_types()}
        for leaked in ('/odom', '/joint_states', '/lidar', '/imu', '/cmd_vel_safe'):
            self.assertNotIn(leaked, topics, f'{leaked} is not namespaced')

    def test_both_transform_trees_resolve(self):
        """Both robots' frames coexist in the merged global tree.

        Each robot relays its namespaced transforms onto the global topics, so
        one RViz shows both. That only works because every frame is prefixed.
        """
        import tf2_ros

        buffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(buffer, self.node)
        self.addCleanup(listener.unregister)

        for namespace in ROBOTS:
            for parent, child in ((f'{namespace}odom', f'{namespace}base_footprint'),
                                  (f'{namespace}base_footprint', f'{namespace}front_laser')):
                with self.subTest(transform=f'{parent} -> {child}'):
                    ok = wait_for(
                        lambda p=parent, c=child: buffer.can_transform(
                            p, c, rclpy.time.Time()),
                        timeout=TIMEOUT, node=self.node)
                    self.assertTrue(ok, f'no transform {parent} -> {child}')

    def test_each_robot_has_its_own_description(self):
        """The two robots are separate models, not one description reused.

        Each robot_state_publisher holds a URDF rendered with its own prefix.
        If the prefix were not threaded through, both would publish the same
        frame names and the transform tree would be a single tangled robot.
        """
        from rclpy.parameter_client import AsyncParameterClient

        descriptions = {}
        for namespace in ROBOTS:
            client = AsyncParameterClient(
                self.node, f'/{namespace}/robot_state_publisher')
            self.assertTrue(client.wait_for_services(timeout_sec=TIMEOUT),
                            f'{namespace}robot_state_publisher never appeared')
            future = client.get_parameters(['robot_description'])
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=30.0)
            self.assertIsNotNone(future.result(), f'{namespace}: no parameters')
            descriptions[namespace] = future.result().values[0].string_value

        for namespace in ROBOTS:
            other = next(n for n in ROBOTS if n != namespace)
            self.assertIn(f'{namespace}base_footprint', descriptions[namespace])
            self.assertNotIn(f'{other}base_footprint', descriptions[namespace],
                             f'{namespace} description contains {other} frames')
