"""slam_toolbox builds a map, and twist_mux decides who is driving.

Grouped into one launch because they share a robot and both belong to the same
story: a human drives, the mapper watches. Splitting them would mean starting
Gazebo twice for no extra coverage.
"""

import os
import sys
import unittest

# launch_test executes this file directly, so the directory it lives in is not
# on the import path. Put it there before importing the shared helpers.
sys.path.insert(0, os.path.dirname(__file__))

from geometry_msgs.msg import TwistStamped  # noqa: E402

from helpers import collect, launch_file, NS, simulation, wait_for  # noqa: E402

from nav_msgs.msg import OccupancyGrid  # noqa: E402

import pytest  # noqa: E402

import rclpy  # noqa: E402

STARTUP = 120.0


@pytest.mark.launch_test
def generate_test_description():
    """World, robot, SLAM, and the joystick arbitration stack.

    lampo_joy.launch.py starts joy_node too, which simply publishes nothing
    when no joystick is plugged in -- twist_mux is the part under test, and it
    does not care where its inputs come from.
    """
    return simulation(
        launch_file('lampo_gz_mm.launch.py'),
        launch_file('lampo_slam.launch.py'),
        launch_file('lampo_joy.launch.py'),
    )


class TestSlam(unittest.TestCase):
    """Assertions against a running mapper."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('test_slam')

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def test_map_is_published(self):
        """slam_toolbox produces an occupancy grid from the lidar."""
        grids = collect(self.node, f'/{NS}/map', OccupancyGrid,
                        count=1, timeout=STARTUP)
        self.assertTrue(grids, 'slam_toolbox never published a map')

        grid = grids[-1]
        self.assertGreater(grid.info.width, 0)
        self.assertGreater(grid.info.height, 0)

    def test_map_contains_observed_cells(self):
        """The map is not simply a blank grid of unknowns.

        A mapper that publishes an empty grid forever is the symptom of a scan
        arriving in a frame it cannot transform.
        """
        grids = collect(self.node, f'/{NS}/map', OccupancyGrid,
                        count=3, timeout=STARTUP)
        self.assertTrue(grids, 'no map published')
        known = sum(1 for cell in grids[-1].data if cell >= 0)
        self.assertGreater(known, 0, 'every cell in the map is unknown')

    def test_map_frame_is_global(self):
        """The map frame is shared; the robot frames are prefixed."""
        grids = collect(self.node, f'/{NS}/map', OccupancyGrid,
                        count=1, timeout=STARTUP)
        self.assertEqual(grids[-1].header.frame_id, 'map')

    def test_save_map_service_exists(self):
        """The documented save_map call has something to talk to."""
        from slam_toolbox.srv import SaveMap

        client = self.node.create_client(
            SaveMap, f'/{NS}/slam_toolbox/save_map')
        self.addCleanup(self.node.destroy_client, client)
        self.assertTrue(client.wait_for_service(timeout_sec=STARTUP),
                        'save_map service never appeared')

    def test_joystick_outranks_navigation_at_runtime(self):
        """twist_mux forwards the joystick while both inputs are publishing.

        The config test asserts the priorities; this asserts twist_mux is
        actually wired up and honours them.
        """
        joy = self.node.create_publisher(
            TwistStamped, f'/{NS}/cmd_vel_joy', 10)
        nav = self.node.create_publisher(
            TwistStamped, f'/{NS}/cmd_vel_nav', 10)
        self.addCleanup(self.node.destroy_publisher, joy)
        self.addCleanup(self.node.destroy_publisher, nav)

        received = []
        sub = self.node.create_subscription(
            TwistStamped, f'/{NS}/cmd_vel_safe',
            lambda m: received.append(m), 10)
        self.addCleanup(self.node.destroy_subscription, sub)

        self.assertTrue(
            wait_for(lambda: joy.get_subscription_count() > 0
                     and nav.get_subscription_count() > 0,
                     timeout=STARTUP, node=self.node),
            'twist_mux is not subscribed to both inputs')

        # Distinguishable speeds: the joystick wins, so 1.0 should come out.
        joy_cmd, nav_cmd = TwistStamped(), TwistStamped()
        joy_cmd.twist.linear.x = 1.0
        nav_cmd.twist.linear.x = -1.0

        def both_published():
            joy.publish(joy_cmd)
            nav.publish(nav_cmd)
            return len(received) >= 5

        self.assertTrue(wait_for(both_published, timeout=60.0, node=self.node),
                        'twist_mux published nothing on cmd_vel_safe')

        speeds = [m.twist.linear.x for m in received[-5:]]
        self.assertTrue(all(s > 0 for s in speeds),
                        f'navigation overrode the joystick: {speeds}')
