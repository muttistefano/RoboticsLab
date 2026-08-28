"""The Cartesian exercise does what doc/control.md promises.

arm_controller:=servo puts MoveIt Servo in front of the velocity controller:
Cartesian twists in, joint velocities out. This launches that configuration
and checks the promises the exercise rests on: the right controllers come up,
joint-jog unfolds the elbow out of the singular spawn pose (the arm spawns
with the elbow dead straight, where Servo rightly refuses Cartesian motion),
a twist command then actually moves the arm (which also proves the SRDF's
collision matrix does not read the arm's own mount as a permanent collision),
and the arm stops when the command stream stops.
"""

import os
import sys
import unittest

# launch_test executes this file directly, so the directory it lives in is not
# on the import path. Put it there before importing the shared helpers.
sys.path.insert(0, os.path.dirname(__file__))

from control_msgs.msg import JointJog  # noqa: E402

from controller_manager_msgs.srv import ListControllers  # noqa: E402

from geometry_msgs.msg import TwistStamped  # noqa: E402

from helpers import launch_file, NS, simulation, wait_for  # noqa: E402

from moveit_msgs.srv import ServoCommandType  # noqa: E402

import pytest  # noqa: E402

import rclpy  # noqa: E402

from sensor_msgs.msg import JointState  # noqa: E402

ARM_JOINTS = [
    'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
    'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint',
]


@pytest.mark.launch_test
def generate_test_description():
    """World plus the mobile manipulator in Cartesian-servo mode."""
    return simulation(launch_file('lampo_gz_mm.launch.py', mm='true',
                                  arm_controller='servo'))


class TestServo(unittest.TestCase):
    """Assertions against the Cartesian servo exercise configuration."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        # Sim time matters here: servo compares the stamp of every incoming
        # twist against its own (simulated) clock, so a wall-clock stamp
        # makes every command look hopelessly stale and nothing moves.
        cls.node = rclpy.create_node('test_servo', parameter_overrides=[
            rclpy.parameter.Parameter('use_sim_time', value=True)])

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def joint_states(self):
        """Subscribe to joint_states; returns dicts of latest pos and vel."""
        position, velocity = {}, {}

        def remember(msg):
            position.update(dict(zip(msg.name, msg.position)))
            velocity.update(dict(zip(msg.name, msg.velocity)))

        sub = self.node.create_subscription(
            JointState, f'/{NS}/joint_states', remember, 10)
        self.addCleanup(self.node.destroy_subscription, sub)
        return position, velocity

    def switch_command_mode(self, command_type, label):
        """Select what kind of command servo accepts (JOINT_JOG or TWIST)."""
        client = self.node.create_client(
            ServoCommandType, f'/{NS}/servo_node/switch_command_type')
        self.assertTrue(client.wait_for_service(timeout_sec=120.0),
                        'servo_node never appeared')
        self.addCleanup(self.node.destroy_client, client)

        request = ServoCommandType.Request(command_type=command_type)
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=10.0)
        self.assertIsNotNone(future.result(),
                             'switch_command_type never replied')
        self.assertTrue(future.result().success, f'{label} mode rejected')

    def test_servo_mode_spawns_the_right_controllers(self):
        """The velocity controller is active, the forward one is the fallback."""
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
            lambda: (states().get('forward_velocity_controller') == 'active'
                     and states().get('joint_state_broadcaster') == 'active'
                     and states().get('forward_position_controller')
                     == 'inactive'),
            timeout=90.0, node=self.node)
        self.assertTrue(ok, f'unexpected controller states: {states()}')

    def test_joint_jog_unfolds_the_elbow(self):
        """Joint jog bends the arm out of the DOUBLY singular spawn pose.

        The arm spawns with the elbow dead straight (a boundary singularity)
        AND wrist_2 at zero, where the axes of joints 4 and 6 are parallel
        (the wrist-alignment singularity). Servo rightly refuses Cartesian
        motion until BOTH are escaped -- bending only the elbow still leaves
        every twist throttled to a crawl. Joint-space jogging does not go
        through the Jacobian inverse, so it is the way out.
        """
        self.switch_command_mode(ServoCommandType.Request.JOINT_JOG,
                                 'JOINT_JOG')
        position, _ = self.joint_states()
        elbow = f'{NS}elbow_joint'
        wrist2 = f'{NS}wrist_2_joint'
        self.assertTrue(
            wait_for(lambda: elbow in position and wrist2 in position,
                     timeout=60.0, node=self.node),
            'joint_states never covered the arm')
        start = {elbow: position[elbow], wrist2: position[wrist2]}

        publisher = self.node.create_publisher(
            JointJog, f'/{NS}/servo_node/delta_joint_cmds', 10)
        self.addCleanup(self.node.destroy_publisher, publisher)

        def bend():
            msg = JointJog()
            msg.header.stamp = self.node.get_clock().now().to_msg()
            msg.joint_names = [elbow, wrist2]
            msg.velocities = [0.4, 0.3]
            publisher.publish(msg)
            return (position[elbow] - start[elbow] > 0.7
                    and position[wrist2] - start[wrist2] > 0.4)

        self.assertTrue(
            wait_for(bend, timeout=60.0, node=self.node, period=0.1),
            f'arm never unfolded: elbow {position[elbow] - start[elbow]:.3f},'
            f' wrist_2 {position[wrist2] - start[wrist2]:.3f} rad')

    def test_twist_command_moves_the_arm(self):
        """A Cartesian twist stream makes joints move.

        This is exercise 3 of doc/control.md, executed literally -- and it
        fails if Servo believes the arm is permanently in collision or at a
        singularity, so it also guards the SRDF.
        """
        self.switch_command_mode(ServoCommandType.Request.TWIST, 'TWIST')
        position, _ = self.joint_states()
        self.assertTrue(
            wait_for(lambda: all(f'{NS}{j}' in position for j in ARM_JOINTS),
                     timeout=60.0, node=self.node),
            'joint_states never covered the arm')
        start = {j: position[f'{NS}{j}'] for j in ARM_JOINTS}

        publisher = self.node.create_publisher(
            TwistStamped, f'/{NS}/servo_node/delta_twist_cmds', 10)
        self.addCleanup(self.node.destroy_publisher, publisher)

        def command():
            msg = TwistStamped()
            msg.header.stamp = self.node.get_clock().now().to_msg()
            msg.header.frame_id = f'{NS}tool0'
            msg.twist.linear.z = -0.1
            publisher.publish(msg)

        def moved():
            command()
            return any(abs(position[f'{NS}{j}'] - start[j]) > 0.05
                       for j in ARM_JOINTS)

        ok = wait_for(moved, timeout=60.0, node=self.node, period=0.1)
        deltas = {j: position[f'{NS}{j}'] - start[j] for j in ARM_JOINTS}
        self.assertTrue(ok, f'twist commands moved no arm joint: {deltas}')

    def test_twist_silence_halts_the_arm(self):
        """No stale-command runaway: silence on the topic stops the arm.

        Servo republishes zeros after incoming_command_timeout, so shortly
        after the last twist every arm joint velocity has to decay to zero.
        """
        _, velocity = self.joint_states()
        still = wait_for(
            lambda: all(abs(velocity.get(f'{NS}{j}', 1e3)) < 0.02
                        for j in ARM_JOINTS),
            timeout=30.0, node=self.node)
        speeds = {j: velocity.get(f'{NS}{j}') for j in ARM_JOINTS}
        self.assertTrue(still, f'arm still moving: {speeds}')
