#!/usr/bin/env python3
"""Zero-G: gravity compensation for the arm (doc/control.md, exercise 4).

With arm_controller:=effort the arm gets raw torque and nothing else -- left
alone it falls. This node computes g(q), the torque each joint needs just to
hold its own weight, and commands exactly that: the arm floats, and a push in
Gazebo's Apply Force/Torque tool moves it as if it were weightless. This is
what cobot vendors sell as "freedrive", and it is one pinocchio call.

Run it after `lampo_gz_mm.launch.py mm:=true arm_controller:=effort`:

    ros2 run lampo_description zero_g.py --ros-args \
        -r __ns:=/r1_ -p use_sim_time:=true

The model comes from the robot_description topic -- the same URDF the
simulator runs -- so compensation and plant agree by construction.
"""

from lampo_description.arm_dynamics import ARM_JOINTS, ArmModel

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSProfile)

from sensor_msgs.msg import JointState

from std_msgs.msg import Float64MultiArray, String


class ZeroG(Node):
    """Publish g(q) as the effort command, 100 times a second."""

    def __init__(self):
        super().__init__('zero_g')

        # Viscous damping RATE [1/s]. In pure zero-G a push would coast
        # forever; real freedrive modes damp it so the arm glides and
        # settles. The torque is damping * M_jj(q) * velocity -- scaled by
        # each joint's apparent inertia -- so every joint decays at the same
        # rate. A fixed Nm.s/rad number instead is a trap: what gently damps
        # a 45 kg.m^2 shoulder violently destabilizes a 0.03 kg.m^2 wrist at
        # this node's 100 Hz. (Found the hard way; the wrist oscillated at
        # its velocity limit until the physics engine gave up.)
        self.declare_parameter('damping', 3.0)

        self.model = None
        self.positions = {}
        self.velocities = {}

        # robot_description is published once, latched (transient local);
        # the subscription must match that durability or it sees nothing.
        latched = QoSProfile(
            depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, 'robot_description',
                                 self.on_urdf, latched)
        self.create_subscription(JointState, 'joint_states',
                                 self.on_joints, 10)
        self.pub = self.create_publisher(
            Float64MultiArray, 'forward_effort_controller/commands', 10)

        self.create_timer(0.01, self.step)
        self.get_logger().info('Waiting for robot_description...')

    def on_urdf(self, msg):
        if self.model is None:
            self.model = ArmModel(msg.data)
            self.get_logger().info('Model loaded; the arm is now floating. '
                                   'Push it around (Apply Force/Torque).')

    def on_joints(self, msg):
        self.positions.update(dict(zip(msg.name, msg.position)))
        self.velocities.update(dict(zip(msg.name, msg.velocity)))

    def step(self):
        if self.model is None or not self.positions:
            return
        g = self.model.gravity(self.positions)
        inertia = self.model.inertia_diag(self.positions)
        d = self.get_parameter('damping').value
        self.pub.publish(Float64MultiArray(
            data=[g[name] - d * inertia[name] * self.velocities.get(name, 0.0)
                  for name in (self.model.full_name(j) for j in ARM_JOINTS)]))


def main():
    rclpy.init()
    try:
        rclpy.spin(ZeroG())
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()
