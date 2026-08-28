#!/usr/bin/env python3
"""A Luenberger observer for one arm joint (doc/control.md, exercise 5).

The simulator hands out joint velocity for free, but a real encoder gives
position only. This node estimates the velocity the honest way: from the
position measurement plus a model of the joint,

    q_ddot = (u - g(q)) / M(q)          # the same dynamics zero_g.py uses

with the classic observer correction  L * (q_measured - q_estimated).
The pole locations are a live parameter: fast poles trust the measurement
(and copy its noise), slow poles trust the model (and lag behind reality).
Feeling that tradeoff is the exercise.

Run it beside zero_g.py in effort mode:

    ros2 run lampo_description joint_observer.py --ros-args \
        -r __ns:=/r1_ -p use_sim_time:=true

and plot ~/estimate: [q_measured, q_estimated, qd_true, qd_estimated].
qd_true is the simulator's velocity, included purely as the answer sheet --
the observer never reads it.
"""

from lampo_description.arm_dynamics import (ARM_JOINTS, ArmModel,
                                            observer_gain)

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSProfile)

from sensor_msgs.msg import JointState

from std_msgs.msg import Float64MultiArray, String

DT = 0.01


class JointObserver(Node):
    """Estimate [q, qd] of one joint from its position and the model."""

    def __init__(self):
        super().__init__('joint_observer')

        # Which joint to observe (suffix; the namespace supplies the prefix)
        # and where to put the observer poles. Both live:
        #   ros2 param set /r1_/joint_observer poles "[-30.0, -40.0]"
        self.declare_parameter('joint', 'shoulder_lift_joint')
        self.declare_parameter('poles', [-8.0, -10.0])

        self.model = None
        self.positions = {}
        self.velocities = {}
        self.effort = 0.0
        self.x = None                     # the estimate, [q_hat, qd_hat]

        latched = QoSProfile(
            depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, 'robot_description',
                                 self.on_urdf, latched)
        self.create_subscription(JointState, 'joint_states',
                                 self.on_joints, 10)
        # The observer's input u is whatever is being commanded -- listen in
        # on the effort controller's command topic.
        self.create_subscription(Float64MultiArray,
                                 'forward_effort_controller/commands',
                                 self.on_command, 10)
        self.pub = self.create_publisher(Float64MultiArray, '~/estimate', 10)

        self.create_timer(DT, self.step)
        self.get_logger().info('Waiting for robot_description...')

    @property
    def joint(self):
        return self.get_parameter('joint').value

    def on_urdf(self, msg):
        if self.model is None:
            self.model = ArmModel(msg.data)
            self.get_logger().info(f'Observing {self.joint}.')

    def on_joints(self, msg):
        self.positions.update(dict(zip(msg.name, msg.position)))
        self.velocities.update(dict(zip(msg.name, msg.velocity)))

    def on_command(self, msg):
        index = ARM_JOINTS.index(self.joint)
        if len(msg.data) > index:
            self.effort = msg.data[index]

    def step(self):
        if self.model is None:
            return
        name = self.model.full_name(self.joint)
        if name not in self.positions:
            return
        q_meas = self.positions[name]

        if self.x is None:                # start on the measurement
            self.x = [q_meas, 0.0]

        # Model acceleration, evaluated at the measured configuration.
        g = self.model.gravity(self.positions)[name]
        inertia = self.model.inertia(self.positions, self.joint)
        accel = (self.effort - g) / inertia

        # x_hat' = A x_hat + B u + L (y - C x_hat), integrated with Euler.
        l1, l2 = observer_gain(self.get_parameter('poles').value)
        error = q_meas - self.x[0]
        self.x[0] += DT * (self.x[1] + l1 * error)
        self.x[1] += DT * (accel + l2 * error)

        self.pub.publish(Float64MultiArray(
            data=[q_meas, self.x[0],
                  self.velocities.get(name, 0.0), self.x[1]]))


def main():
    rclpy.init()
    try:
        rclpy.spin(JointObserver())
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()
