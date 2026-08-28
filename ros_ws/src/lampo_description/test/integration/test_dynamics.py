"""The Zero-G and observer labs do what doc/control.md promises.

arm_controller:=effort is a raw torque passthrough; zero_g.py computes g(q)
with pinocchio and the arm floats. If the pinocchio model disagreed with the
URDF the simulator runs, the arm would drift or fall -- so the float test is
also a model-identity test. The observer runs beside it, estimating one
joint's velocity from position and torque alone.
"""

import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(__file__))

from controller_manager_msgs.srv import ListControllers  # noqa: E402

from helpers import launch_file, NS, simulation, wait_for  # noqa: E402

from launch_ros.actions import Node  # noqa: E402

import pytest  # noqa: E402

import rclpy  # noqa: E402

from sensor_msgs.msg import JointState  # noqa: E402

from std_msgs.msg import Float64MultiArray  # noqa: E402

ARM_JOINTS = [
    'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
    'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint',
]


def lab_node(executable):
    """One of the dynamics-lab nodes, exactly as the docs run it."""
    return Node(package='lampo_description', executable=executable,
                namespace=NS, output='screen',
                parameters=[{'use_sim_time': True}])


@pytest.mark.launch_test
def generate_test_description():
    """World, the arm in raw-effort mode, and the two lab nodes."""
    return simulation(
        launch_file('lampo_gz_mm.launch.py', mm='true',
                    arm_controller='effort'),
        lab_node('zero_g.py'),
        lab_node('joint_observer.py'),
    )


class TestDynamics(unittest.TestCase):
    """Assertions against the Zero-G and observer exercises."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('test_dynamics', parameter_overrides=[
            rclpy.parameter.Parameter('use_sim_time', value=True)])

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def joint_positions(self):
        positions, velocities = {}, {}

        def remember(msg):
            positions.update(dict(zip(msg.name, msg.position)))
            velocities.update(dict(zip(msg.name, msg.velocity)))

        sub = self.node.create_subscription(
            JointState, f'/{NS}/joint_states', remember, 10)
        self.addCleanup(self.node.destroy_subscription, sub)
        return positions, velocities

    def test_effort_mode_spawns_the_right_controllers(self):
        """The torque passthrough is active, the forward one is the fallback."""
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

        ok = wait_for(
            lambda: (states().get('forward_effort_controller') == 'active'
                     and states().get('joint_state_broadcaster') == 'active'
                     and states().get('forward_position_controller')
                     == 'inactive'),
            timeout=90.0, node=self.node)
        self.assertTrue(ok, f'unexpected controller states: {states()}')

    def test_the_arm_floats(self):
        """Gravity compensation holds the arm still without position control.

        This only works if pinocchio's g(q) matches the physics engine's
        gravity load -- both derive from the same URDF, and this assert is
        what proves it.
        """
        # Wait until zero_g.py is actually commanding (its model is loaded).
        self.assertTrue(
            wait_for(lambda: self.node.count_publishers(
                f'/{NS}/forward_effort_controller/commands') > 0,
                timeout=90.0, node=self.node),
            'zero_g.py never started commanding')

        positions, velocities = self.joint_positions()
        self.assertTrue(
            wait_for(lambda: all(f'{NS}{j}' in positions for j in ARM_JOINTS),
                     timeout=60.0, node=self.node),
            'joint_states never covered the arm')

        # Let the activation transient (the brief uncompensated sag) damp
        # out, then demand stillness.
        wait_for(lambda: False, timeout=5.0, node=self.node)
        start = {j: positions[f'{NS}{j}'] for j in ARM_JOINTS}
        wait_for(lambda: False, timeout=10.0, node=self.node)
        drift = {j: abs(positions[f'{NS}{j}'] - start[j]) for j in ARM_JOINTS}
        self.assertTrue(max(drift.values()) < 0.2,
                        f'the arm is not floating; drift: {drift}')
        # Stillness means still: a joint buzzing symmetrically around its
        # start position has near-zero drift and is very much not floating.
        # (The original fixed-Nm damping did exactly that to the wrist.)
        speed = {j: abs(velocities.get(f'{NS}{j}', 0.0)) for j in ARM_JOINTS}
        self.assertTrue(max(speed.values()) < 0.1,
                        f'the arm is oscillating; velocities: {speed}')

    def test_the_observer_estimate_matches_reality(self):
        """[q_meas, q_est, qd_true, qd_est]: the estimates track the truth.

        The observer reads position and commanded torque only; qd_true in
        the message exists purely so this test (and the plots in the docs)
        can grade it.
        """
        latest = []

        def remember(msg):
            latest[:] = msg.data

        sub = self.node.create_subscription(
            Float64MultiArray, f'/{NS}/joint_observer/estimate', remember, 10)
        self.addCleanup(self.node.destroy_subscription, sub)

        def tracking():
            return (len(latest) == 4
                    and abs(latest[0] - latest[1]) < 0.05
                    and abs(latest[2] - latest[3]) < 0.15)

        self.assertTrue(wait_for(tracking, timeout=90.0, node=self.node),
                        f'observer never converged; latest: {latest}')
