#!/usr/bin/env python3
"""LQR position control of one arm joint (doc/control.md, exercise 6).

Where the PID exercises tuned three knobs by feel, this controller is
*derived*: linearize the gravity-compensated joint into a double integrator,
write down a cost

    J = integral( q_pos * error^2  +  q_vel * error_dot^2  +  r * u^2 )

and let the Riccati equation hand back the unique optimal gain K. The knobs
left to tune are the honest ones -- how much you care about position error,
velocity, and actuator effort. They are live parameters; K is re-solved and
logged on every change:

    ros2 param set /r1_/lqr_joint r 0.001        # effort is cheap -> stiffer

The other five joints get gravity compensation plus a soft PD hold at their
startup pose, so the lab isolates a single clean second-order system.

Run it in effort mode (instead of zero_g.py, not beside it -- both write the
same command):

    ros2 run lampo_description lqr_joint.py --ros-args \
        -r __ns:=/r1_ -p use_sim_time:=true
    ros2 topic pub --once /r1_/lqr_joint/reference std_msgs/msg/Float64 \
        "{data: -1.2}"
"""

from lampo_description.arm_dynamics import ARM_JOINTS, ArmModel, lqr_gain

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSProfile)

from sensor_msgs.msg import JointState

from std_msgs.msg import Float64, Float64MultiArray, String


class LqrJoint(Node):
    """u = g(q) - K (x - x_ref) on one joint; gravity + PD hold on the rest."""

    def __init__(self):
        super().__init__('lqr_joint')

        self.declare_parameter('joint', 'shoulder_lift_joint')
        # The cost weights. Their RATIOS are what matters: scaling all three
        # together leaves K unchanged -- worth verifying as an exercise.
        self.declare_parameter('q_pos', 100.0)
        self.declare_parameter('q_vel', 10.0)
        self.declare_parameter('r', 0.01)
        # The soft hold on the non-LQR joints. RATES, not raw gains: the
        # torque is M_jj * (hold_p * error - hold_d * velocity), scaled by
        # each joint's apparent inertia, so the hold behaves identically on
        # a 45 kg.m^2 shoulder and a 0.03 kg.m^2 wrist. (hold_p is omega^2:
        # 25 -> a 5 rad/s hold; hold_d = 10 critically damps it.)
        self.declare_parameter('hold_p', 25.0)
        self.declare_parameter('hold_d', 10.0)

        self.model = None
        self.positions = {}
        self.velocities = {}
        self.home = None                  # where the held joints stay
        self.reference = None             # starts at the joint's own pose
        self.got_reference = False
        self.gains_key = None
        self.K = None

        latched = QoSProfile(
            depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, 'robot_description',
                                 self.on_urdf, latched)
        self.create_subscription(JointState, 'joint_states',
                                 self.on_joints, 10)
        self.create_subscription(Float64, '~/reference', self.on_reference,
                                 10)
        self.pub = self.create_publisher(
            Float64MultiArray, 'forward_effort_controller/commands', 10)

        self.create_timer(0.01, self.step)
        self.get_logger().info('Waiting for robot_description...')

    @property
    def joint(self):
        return self.get_parameter('joint').value

    def on_urdf(self, msg):
        if self.model is None:
            self.model = ArmModel(msg.data)

    def on_joints(self, msg):
        self.positions.update(dict(zip(msg.name, msg.position)))
        self.velocities.update(dict(zip(msg.name, msg.velocity)))

    def on_reference(self, msg):
        # The first external reference marks "the experiment starts now":
        # re-capture the hold pose, so anything that moved the arm since
        # startup (a fall, a re-pose through the forward controller) is
        # accepted as the new posture for the five held joints.
        if not self.got_reference:
            self.got_reference = True
            self.home = None
        self.reference = msg.data

    def refresh_gain(self):
        """Re-solve the Riccati equation when a weight changes."""
        weights = tuple(self.get_parameter(n).value
                        for n in ('q_pos', 'q_vel', 'r'))
        if weights == self.gains_key:
            return
        inertia = self.model.inertia(self.positions, self.joint)
        self.K = lqr_gain(inertia, *weights)
        self.gains_key = weights
        self.get_logger().info(
            f'q_pos={weights[0]:g} q_vel={weights[1]:g} r={weights[2]:g} '
            f'(M={inertia:.1f}) -> K = [{self.K[0]:.1f}, {self.K[1]:.1f}]')

    def step(self):
        if self.model is None:
            return
        names = {j: self.model.full_name(j) for j in ARM_JOINTS}
        if not all(names[j] in self.positions for j in ARM_JOINTS):
            return

        if self.home is None:
            self.home = {j: self.positions[names[j]] for j in ARM_JOINTS}
        if self.reference is None:
            self.reference = self.home[self.joint]
        self.refresh_gain()

        gravity = self.model.gravity(self.positions)
        inertia = self.model.inertia_diag(self.positions)
        hold_p = self.get_parameter('hold_p').value
        hold_d = self.get_parameter('hold_d').value

        efforts = []
        for j in ARM_JOINTS:
            name = names[j]
            u = gravity[name]
            if j == self.joint:
                u -= (self.K[0] * (self.positions[name] - self.reference)
                      + self.K[1] * self.velocities.get(name, 0.0))
            else:
                u += inertia[name] * (
                    hold_p * (self.home[j] - self.positions[name])
                    - hold_d * self.velocities.get(name, 0.0))
            limit = self.model.effort_limit(j)
            efforts.append(max(-limit, min(limit, u)))

        self.pub.publish(Float64MultiArray(data=efforts))


def main():
    rclpy.init()
    try:
        rclpy.spin(LqrJoint())
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()
