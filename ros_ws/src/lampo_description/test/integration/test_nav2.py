"""Nav2 comes up, agrees about time, and drives the robot to a goal.

The three things being proved here each correspond to a defect that was fixed
blind, without the packages installed to test it:

  * every server runs on sim time, not only amcl;
  * AMCL is seeded where the robot actually is;
  * the velocity chain ends at the topic the bridge forwards.
"""

import os
import sys
import unittest

# launch_test executes this file directly, so the directory it lives in is not
# on the import path. Put it there before importing the shared helpers.
sys.path.insert(0, os.path.dirname(__file__))

from geometry_msgs.msg import PoseStamped  # noqa: E402

from helpers import launch_file, NS, simulation, wait_for  # noqa: E402

from nav2_msgs.action import NavigateToPose  # noqa: E402

import pytest  # noqa: E402

import rclpy  # noqa: E402
from rclpy.action import ActionClient  # noqa: E402

from std_srvs.srv import Trigger  # noqa: E402

# A modest goal: 1.3 m straight ahead of the spawn pose at (-3.5, 2.2).
#
# Deliberately short. The point of this test is that the whole chain works --
# planner, controller, collision monitor, bridge, wheels -- not that Nav2 can
# cross a warehouse. Longer goals traverse aisles where the collision monitor
# repeatedly slows the robot to a crawl; on a machine whose control loop is
# already starved by software-rendered sensors, the progress checker then
# aborts a run that would have succeeded given more headroom. That made this
# test fail about two runs in three while proving nothing about the code.
# The demo in DEMO.md still sends a goal across the warehouse.
GOAL_X, GOAL_Y = -2.2, 2.2

STARTUP = 180.0


@pytest.mark.launch_test
def generate_test_description():
    """World, robot, and the navigation stack."""
    return simulation(
        launch_file('lampo_gz_mm.launch.py'),
        launch_file('lampo_nav_omni.launch.py'),
    )


class TestNav2(unittest.TestCase):
    """Assertions against a running navigation stack."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('test_nav2')

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def _lifecycle_manager_active(self, manager):
        client = self.node.create_client(Trigger, f'/{NS}/{manager}/is_active')
        self.addCleanup(self.node.destroy_client, client)
        if not client.wait_for_service(timeout_sec=STARTUP):
            return False

        def active():
            future = client.call_async(Trigger.Request())
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=10.0)
            return future.result() is not None and future.result().success

        return wait_for(active, timeout=STARTUP, node=self.node)

    def test_navigation_servers_reach_active(self):
        """The lifecycle manager brought the whole navigation group up."""
        self.assertTrue(
            self._lifecycle_manager_active('lifecycle_manager_navigation'),
            'lifecycle_manager_navigation never reported active')

    def test_localization_reaches_active(self):
        """AMCL and the map server came up too."""
        self.assertTrue(
            self._lifecycle_manager_active('lifecycle_manager_localization'),
            'lifecycle_manager_localization never reported active')

    def test_every_node_uses_sim_time(self):
        """No Nav2 node is left on the wall clock.

        use_sim_time was set on amcl alone. Every other server ran on wall
        time while TF was stamped in sim time, producing a permanent stream of
        "Lookup would require extrapolation into the future". The fix was a
        top-level /** wildcard; this asserts it reached every node.
        """
        from rclpy.parameter_client import AsyncParameterClient

        self.assertTrue(self._lifecycle_manager_active(
            'lifecycle_manager_navigation'), 'navigation never came up')

        # ros_gz_bridge is excluded on purpose: it relays messages and
        # preserves the gz header stamps rather than generating its own, and
        # making it wait on the very clock it is bridging can stall its
        # timers. See the comment in lampo_gz_mm.launch.py.
        ignored = ('test_', 'launch_', 'transform_', 'ros_gz_bridge', 'relay_')
        names = [n for n, ns in self.node.get_node_names_and_namespaces()
                 if ns.strip('/') == NS.strip('/') and not n.startswith(ignored)]
        self.assertTrue(names, 'no nodes found in the robot namespace')

        offenders = []
        for name in names:
            client = AsyncParameterClient(self.node, f'/{NS}/{name}')
            if not client.wait_for_services(timeout_sec=10.0):
                continue
            future = client.get_parameters(['use_sim_time'])
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=10.0)
            result = future.result()
            if result is None or not result.values:
                continue
            if not result.values[0].bool_value:
                offenders.append(name)

        self.assertFalse(offenders, f'these nodes run on the wall clock: {offenders}')

    def test_robot_navigates_to_a_goal(self):
        """A NavigateToPose goal is accepted and reached.

        This is the demo's block 3 in one assertion.
        """
        self.assertTrue(self._lifecycle_manager_active(
            'lifecycle_manager_navigation'), 'navigation never came up')

        client = ActionClient(self.node, NavigateToPose, f'/{NS}/navigate_to_pose')
        self.assertTrue(client.wait_for_server(timeout_sec=STARTUP),
                        'navigate_to_pose action server never appeared')

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.pose.position.x = GOAL_X
        goal.pose.pose.position.y = GOAL_Y
        goal.pose.pose.orientation.w = 1.0

        send = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send, timeout_sec=30.0)
        handle = send.result()
        self.assertIsNotNone(handle, 'goal was never acknowledged')
        self.assertTrue(handle.accepted, 'navigation rejected the goal')

        result = handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, result, timeout_sec=180.0)
        self.assertIsNotNone(result.result(), 'navigation never returned a result')

        from action_msgs.msg import GoalStatus
        self.assertEqual(result.result().status, GoalStatus.STATUS_SUCCEEDED,
                         f'navigation finished with status '
                         f'{result.result().status}, expected SUCCEEDED')

        # Arrival is asserted here rather than in its own test method:
        # unittest runs methods in alphabetical order, so a separate check
        # would have run before this one ever sent a goal.
        import tf2_ros

        buffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(buffer, self.node)
        self.addCleanup(listener.unregister)

        self.assertTrue(
            wait_for(lambda: buffer.can_transform('map', f'{NS}base_footprint',
                                                  rclpy.time.Time()),
                     timeout=60.0, node=self.node),
            'map -> base_footprint never resolved')

        position = buffer.lookup_transform(
            'map', f'{NS}base_footprint', rclpy.time.Time()).transform.translation
        distance = ((position.x - GOAL_X) ** 2 + (position.y - GOAL_Y) ** 2) ** 0.5
        self.assertLess(distance, 0.6, f'robot stopped {distance:.2f} m from the goal')
