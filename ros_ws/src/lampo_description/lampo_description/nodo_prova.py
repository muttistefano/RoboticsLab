#!/usr/bin/env python3
"""Worked example: a closed-loop node that drives the robot to x = 0.

Deliberately small, and deliberately idiomatic. It shows the four things a
ROS 2 control node almost always needs:

  1. a subscription for state              (odometry)
  2. a publisher for commands              (velocity)
  3. a timer for the control loop          -- NOT a raw thread + time.sleep()
  4. sim-time awareness                    (use_sim_time)

Run it after spawning a robot:

    ros2 run lampo_description nodo_prova.py --ros-args \
        -r __ns:=/r1_ -p use_sim_time:=true

Because the node is launched into the robot's namespace, every topic name
below is *relative*: 'odom' becomes '/r1_/odom' automatically. Hardcoding
'/r1_/odom' would tie the node to one robot.
"""

import math

from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


class GoToOrigin(Node):
    """Proportional controller on the x axis."""

    def __init__(self):
        super().__init__('go_to_origin')

        # Tunable at runtime: ros2 param set /go_to_origin gain 0.5
        self.declare_parameter('gain', 0.3)
        self.declare_parameter('tolerance', 0.05)
        self.declare_parameter('max_speed', 0.4)

        self.odom = None

        # Relative names -- resolved against the node's namespace.
        self.cmd_pub = self.create_publisher(TwistStamped, 'cmd_vel_safe', 10)
        self.create_subscription(Odometry, 'odom', self.on_odom, 10)

        # The control loop. A timer is driven by the node's clock, so with
        # use_sim_time:=true it follows Gazebo's clock rather than wall time.
        self.create_timer(0.1, self.control_step)

        self.get_logger().info('Waiting for the first odometry message...')

    def on_odom(self, msg: Odometry):
        if self.odom is None:
            self.get_logger().info('First odometry message received.')
        self.odom = msg

    def control_step(self):
        if self.odom is None:
            return

        gain = self.get_parameter('gain').value
        tolerance = self.get_parameter('tolerance').value
        max_speed = self.get_parameter('max_speed').value

        error = -self.odom.pose.pose.position.x

        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'base_link'

        if math.fabs(error) > tolerance:
            # Proportional term, clamped so we never exceed the base's limits.
            cmd.twist.linear.x = max(-max_speed,
                                     min(max_speed, gain * error))
        else:
            # Inside the deadband: publish an explicit zero rather than simply
            # stopping. A robot that stops receiving commands keeps its last
            # velocity until the driver times out.
            cmd.twist.linear.x = 0.0

        self.cmd_pub.publish(cmd)


def main(args=None):
    # The context-manager form shuts rclpy down cleanly even on exceptions.
    try:
        with rclpy.init(args=args):
            rclpy.spin(GoToOrigin())
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()
