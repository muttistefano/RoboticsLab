"""A spawned robot produces everything the rest of the stack depends on.

This is the test that would have caught most of what was wrong with this
package: sensors publishing into unresolvable frames, wheels with no
/joint_states publisher, controllers racing the model spawn, and an arm that
accepts commands and does not move.
"""

import os
import sys
import unittest

# launch_test executes this file directly, so the directory it lives in is not
# on the import path. Put it there before importing the shared helpers.
sys.path.insert(0, os.path.dirname(__file__))

from controller_manager_msgs.srv import ListControllers  # noqa: E402

from helpers import collect, launch_file, NS, simulation, wait_for  # noqa: E402

import pytest  # noqa: E402

import rclpy  # noqa: E402

from sensor_msgs.msg import Imu, JointState, LaserScan  # noqa: E402

from std_msgs.msg import Float64MultiArray  # noqa: E402

# The arm pose the README and DEMO use.
ARM_TARGET = [1.0, -1.2, 1.0, -1.4, -1.57, 0.0]

ARM_JOINTS = [
    'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
    'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint',
]


@pytest.mark.launch_test
def generate_test_description():
    """World plus the full mobile manipulator."""
    return simulation(launch_file('lampo_gz_mm.launch.py', mm='true'))


class TestRobot(unittest.TestCase):
    """Assertions against a spawned mobile manipulator."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('test_robot')

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def test_every_sensor_publishes(self):
        """The topics the demo introspects all exist and carry data."""
        for topic, msg_type in ((f'/{NS}/joint_states', JointState),
                                (f'/{NS}/lidar', LaserScan),
                                (f'/{NS}/imu', Imu)):
            with self.subTest(topic=topic):
                messages = collect(self.node, topic, msg_type, timeout=90.0)
                self.assertTrue(messages, f'{topic} never published')

    def test_wheel_joints_are_in_joint_states(self):
        """The wheels report their positions.

        joint_state_broadcaster only covers ros2_control joints -- the arm.
        Without the gz JointStatePublisher plugin nothing publishes the wheels,
        robot_state_publisher cannot emit base -> wheel transforms, and the
        robot renders in RViz with its wheels at the origin.
        """
        names = set()
        for msg in collect(self.node, f'/{NS}/joint_states', JointState,
                           count=10, timeout=90.0):
            names.update(msg.name)

        for wheel in ('front_left', 'front_right', 'rear_left', 'rear_right'):
            self.assertIn(f'{NS}{wheel}_wheel_joint', names,
                          f'{wheel} wheel is missing from /joint_states')

    def test_odometry_frames_are_namespaced(self):
        """Odometry arrives in this robot's frames, not global ones.

        Unprefixed frames are what makes a second robot impossible.
        """
        from nav_msgs.msg import Odometry

        messages = collect(self.node, f'/{NS}/odom', Odometry, timeout=90.0)
        self.assertTrue(messages, '/odom never published')
        self.assertEqual(messages[0].header.frame_id, f'{NS}odom')
        self.assertEqual(messages[0].child_frame_id, f'{NS}base_footprint')

    def test_transform_tree_resolves(self):
        """Odom, base_footprint and the camera optical frame all connect.

        A gap here is invisible until Nav2 refuses to plan.
        """
        import tf2_ros

        buffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(buffer, self.node)
        self.addCleanup(listener.unregister)

        for target in (f'{NS}base_link_sweepee', f'{NS}front_laser',
                       f'{NS}cam_optical_frame'):
            with self.subTest(frame=target):
                ok = wait_for(
                    lambda t=target: buffer.can_transform(
                        f'{NS}odom', t, rclpy.time.Time()),
                    timeout=60.0, node=self.node)
                self.assertTrue(ok, f'no transform {NS}odom -> {target}')

    def test_controllers_are_active(self):
        """Both controllers reached the active state.

        They are chained to the spawn with event handlers rather than a timer,
        so this also proves the ordering did not race.
        """
        client = self.node.create_client(
            ListControllers, f'/{NS}/controller_manager/list_controllers')
        self.assertTrue(client.wait_for_service(timeout_sec=90.0),
                        'controller_manager never appeared')

        def active():
            future = client.call_async(ListControllers.Request())
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=10.0)
            if future.result() is None:
                return {}
            return {c.name: c.state for c in future.result().controller}

        ok = wait_for(
            lambda: all(active().get(name) == 'active' for name in
                        ('joint_state_broadcaster', 'forward_position_controller')),
            timeout=90.0, node=self.node)
        self.assertTrue(ok, f'controllers not all active: {active()}')

    def test_arm_reaches_the_commanded_pose(self):
        """The demo's arm command actually moves the arm there.

        This is the ros2_control beat of the presentation. It publishes the
        exact command the run sheet uses and waits for the joints to arrive.
        """
        publisher = self.node.create_publisher(
            Float64MultiArray, f'/{NS}/forward_position_controller/commands', 10)
        self.addCleanup(self.node.destroy_publisher, publisher)

        self.assertTrue(
            wait_for(lambda: publisher.get_subscription_count() > 0,
                     timeout=90.0, node=self.node),
            'nothing is subscribed to the position controller')

        latest = {}

        def remember(msg):
            latest.update(dict(zip(msg.name, msg.position)))

        sub = self.node.create_subscription(
            JointState, f'/{NS}/joint_states', remember, 10)
        self.addCleanup(self.node.destroy_subscription, sub)

        def arrived():
            publisher.publish(Float64MultiArray(data=ARM_TARGET))
            return all(
                abs(latest.get(f'{NS}{joint}', 1e3) - target) < 0.05
                for joint, target in zip(ARM_JOINTS, ARM_TARGET))

        self.assertTrue(wait_for(arrived, timeout=60.0, node=self.node),
                        f'arm did not reach {ARM_TARGET}; '
                        f'got {[latest.get(f"{NS}{j}") for j in ARM_JOINTS]}')

    def test_gripper_is_passive_by_default(self):
        """No gripper controller is spawned, and the simulator is still alive.

        The Robotiq 4-bar linkage cannot be expressed in URDF, and with
        ros2_control attached the joint limiter aborts the whole simulator.
        Passive geometry is the deliberate trade.
        """
        client = self.node.create_client(
            ListControllers, f'/{NS}/controller_manager/list_controllers')
        self.assertTrue(client.wait_for_service(timeout_sec=90.0))

        future = client.call_async(ListControllers.Request())
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=10.0)
        names = [c.name for c in future.result().controller]
        self.assertNotIn('gripper_position_controller', names)

    def test_no_static_transform_relay(self):
        """/tf_static is published directly, with TRANSIENT_LOCAL durability.

        A topic_tools relay does not preserve that durability, so anything
        started later -- an RViz restart, or Nav2 in another terminal -- would
        never receive the static transforms.
        """
        from rclpy.qos import DurabilityPolicy

        info = self.node.get_publishers_info_by_topic('/tf_static')
        self.assertTrue(info, '/tf_static has no publisher')
        durabilities = {i.qos_profile.durability for i in info}
        self.assertIn(DurabilityPolicy.TRANSIENT_LOCAL, durabilities,
                      f'/tf_static is not TRANSIENT_LOCAL: {durabilities}')
