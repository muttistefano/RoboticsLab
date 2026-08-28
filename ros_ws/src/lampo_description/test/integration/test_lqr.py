"""The LQR lab does what doc/control.md promises.

lqr_joint.py derives its gain from the Riccati equation instead of tuning:
this launches the raw-effort mode with the node, steps the reference, and
checks the derived controller actually lands the joint -- then changes a
cost weight through the parameter service, exactly as the exercise does,
and checks the re-derived gain still works.
"""

import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(__file__))

from controller_manager_msgs.srv import ListControllers  # noqa: E402

from helpers import launch_file, NS, simulation, wait_for  # noqa: E402

from launch_ros.actions import Node  # noqa: E402

import pytest  # noqa: E402

from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue  # noqa: E402
from rcl_interfaces.srv import SetParameters  # noqa: E402

import rclpy  # noqa: E402

from sensor_msgs.msg import JointState  # noqa: E402

from std_msgs.msg import Float64  # noqa: E402

JOINT = 'shoulder_lift_joint'            # the node's default LQR joint


@pytest.mark.launch_test
def generate_test_description():
    """World, the arm in raw-effort mode, and the LQR node."""
    return simulation(
        launch_file('lampo_gz_mm.launch.py', mm='true',
                    arm_controller='effort'),
        Node(package='lampo_description', executable='lqr_joint.py',
             namespace=NS, output='screen',
             parameters=[{'use_sim_time': True}]),
    )


class TestLqr(unittest.TestCase):
    """Assertions against the LQR exercise."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('test_lqr', parameter_overrides=[
            rclpy.parameter.Parameter('use_sim_time', value=True)])

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def converge(self, target, tolerance=0.05, timeout=90.0):
        """Publish `target` on the reference until the joint arrives."""
        publisher = self.node.create_publisher(
            Float64, f'/{NS}/lqr_joint/reference', 10)
        self.addCleanup(self.node.destroy_publisher, publisher)

        position = {}

        def remember(msg):
            position.update(dict(zip(msg.name, msg.position)))

        sub = self.node.create_subscription(
            JointState, f'/{NS}/joint_states', remember, 10)
        self.addCleanup(self.node.destroy_subscription, sub)

        def arrived():
            publisher.publish(Float64(data=target))
            return abs(position.get(f'{NS}{JOINT}', 1e3) - target) < tolerance

        self.assertTrue(
            wait_for(arrived, timeout=timeout, node=self.node),
            f'{JOINT} did not reach {target}; '
            f'at {position.get(f"{NS}{JOINT}")}')

    def wait_for_controllers(self):
        """Wait until the effort chain is fully up before commanding.

        Stepping the reference before the effort controller is active would
        make lqr_joint capture its hold pose mid-spawn.
        """
        lister = self.node.create_client(
            ListControllers, f'/{NS}/controller_manager/list_controllers')
        self.assertTrue(lister.wait_for_service(timeout_sec=90.0),
                        'controller_manager never appeared')
        self.addCleanup(self.node.destroy_client, lister)

        def ready():
            future = lister.call_async(ListControllers.Request())
            rclpy.spin_until_future_complete(self.node, future,
                                             timeout_sec=10.0)
            if future.result() is None:
                return False
            states = {c.name: c.state for c in future.result().controller}
            return states.get('forward_effort_controller') == 'active'

        self.assertTrue(wait_for(ready, timeout=90.0, node=self.node),
                        'forward_effort_controller never activated')

    def test_lqr_converges_on_a_step(self):
        """The derived gain lands the joint on a stepped reference."""
        self.wait_for_controllers()
        self.converge(-1.2)              # home pose is -1.57

    def test_weights_can_be_retuned_at_runtime(self):
        """Changing r re-solves the Riccati equation and the loop still works."""
        client = self.node.create_client(
            SetParameters, f'/{NS}/lqr_joint/set_parameters')
        self.assertTrue(client.wait_for_service(timeout_sec=90.0),
                        'lqr_joint parameter service never appeared')
        self.addCleanup(self.node.destroy_client, client)

        request = SetParameters.Request(parameters=[Parameter(
            name='r',
            value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE,
                                 double_value=0.001))])
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=10.0)
        self.assertIsNotNone(future.result(), 'set_parameters never replied')
        self.assertTrue(future.result().results[0].successful,
                        f'weight rejected: {future.result().results[0].reason}')

        # The stiffer controller still converges -- on a different target,
        # so this cannot pass on the previous test's leftovers.
        self.converge(-1.45)
