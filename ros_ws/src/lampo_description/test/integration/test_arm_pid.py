"""The PID lab does what doc/control.md promises.

The control exercises replace the forward position controller with a chained
pid_controller whose gains students retune at runtime. This launches that
configuration and checks the three promises the exercise rests on: the right
controllers come up in the right states, the loop actually converges on a step
reference, and `ros2 param set` retunes it without breaking the loop.
"""

import os
import sys
import unittest

# launch_test executes this file directly, so the directory it lives in is not
# on the import path. Put it there before importing the shared helpers.
sys.path.insert(0, os.path.dirname(__file__))

from control_msgs.msg import MultiDOFCommand  # noqa: E402

from controller_manager_msgs.srv import ListControllers  # noqa: E402

from helpers import launch_file, NS, simulation, wait_for  # noqa: E402

import pytest  # noqa: E402

from rcl_interfaces.msg import Parameter, ParameterValue  # noqa: E402
from rcl_interfaces.msg import ParameterType  # noqa: E402
from rcl_interfaces.srv import SetParameters  # noqa: E402

import rclpy  # noqa: E402

from sensor_msgs.msg import JointState  # noqa: E402

ARM_JOINTS = [
    'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
    'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint',
]

# A modest step away from the spawn pose (shoulder_lift and wrist_1 start at
# -1.57): every joint moves, none goes anywhere near a limit.
STEP_TARGET = [0.5, -1.2, 0.3, -1.4, 0.3, 0.3]


@pytest.mark.launch_test
def generate_test_description():
    """World plus the mobile manipulator in PID-lab mode."""
    return simulation(launch_file('lampo_gz_mm.launch.py', mm='true',
                                  arm_controller='pid'))


class TestArmPid(unittest.TestCase):
    """Assertions against the arm PID exercise configuration."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('test_arm_pid')

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def controller_states(self):
        client = self.node.create_client(
            ListControllers, f'/{NS}/controller_manager/list_controllers')
        self.assertTrue(client.wait_for_service(timeout_sec=90.0),
                        'controller_manager never appeared')
        self.addCleanup(self.node.destroy_client, client)

        def states():
            future = client.call_async(ListControllers.Request())
            rclpy.spin_until_future_complete(self.node, future,
                                             timeout_sec=10.0)
            if future.result() is None:
                return {}
            return {c.name: c.state for c in future.result().controller}
        return states

    def converge(self, target, tolerance=0.05, timeout=90.0):
        """Publish `target` as a step reference until every joint arrives."""
        publisher = self.node.create_publisher(
            MultiDOFCommand, f'/{NS}/arm_pid_controller/reference', 10)
        self.addCleanup(self.node.destroy_publisher, publisher)

        latest = {}

        def remember(msg):
            latest.update(dict(zip(msg.name, msg.position)))

        sub = self.node.create_subscription(
            JointState, f'/{NS}/joint_states', remember, 10)
        self.addCleanup(self.node.destroy_subscription, sub)

        reference = MultiDOFCommand(
            dof_names=[f'{NS}{j}' for j in ARM_JOINTS], values=target)

        def arrived():
            publisher.publish(reference)
            return all(
                abs(latest.get(f'{NS}{joint}', 1e3) - value) < tolerance
                for joint, value in zip(ARM_JOINTS, target))

        self.assertTrue(
            wait_for(arrived, timeout=timeout, node=self.node),
            f'arm did not reach {target}; '
            f'got {[latest.get(f"{NS}{j}") for j in ARM_JOINTS]}')

    def test_pid_mode_spawns_the_right_controllers(self):
        """The PID is active and the forward controller is the idle fallback.

        Both active would mean two controllers claiming command interfaces on
        the same joints; the forward controller missing would mean students
        cannot switch back with `ros2 control switch_controllers`.
        """
        states = self.controller_states()
        ok = wait_for(
            lambda: (states().get('arm_pid_controller') == 'active'
                     and states().get('joint_state_broadcaster') == 'active'
                     and states().get('forward_position_controller')
                     == 'inactive'),
            timeout=90.0, node=self.node)
        self.assertTrue(ok, f'unexpected controller states: {states()}')

    def test_arm_converges_on_a_step_reference(self):
        """The loop closes: a step on the reference topic moves the arm there.

        This is exercise 1 of doc/control.md, executed literally.
        """
        self.converge(STEP_TARGET)

    def test_gains_can_be_retuned_at_runtime(self):
        """`ros2 param set` on a gain is accepted and the loop keeps tracking.

        The whole exercise rests on live retuning: pid_controller re-reads its
        parameters every update cycle, so a gain change needs no restart.
        """
        client = self.node.create_client(
            SetParameters, f'/{NS}/arm_pid_controller/set_parameters')
        self.assertTrue(client.wait_for_service(timeout_sec=90.0),
                        'arm_pid_controller parameter service never appeared')
        self.addCleanup(self.node.destroy_client, client)

        request = SetParameters.Request(parameters=[Parameter(
            name=f'gains.{NS}shoulder_pan_joint.p',
            value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE,
                                 double_value=6.0))])
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=10.0)
        self.assertIsNotNone(future.result(), 'set_parameters never replied')
        self.assertTrue(future.result().results[0].successful,
                        f'gain rejected: {future.result().results[0].reason}')

        # The retuned loop still converges -- on a different pose, so this
        # cannot pass on the leftovers of the previous test.
        self.converge([-0.3, -1.4, 0.5, -1.2, -0.3, 0.5])
