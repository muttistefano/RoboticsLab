"""Shared helpers for the simulator-in-the-loop tests.

These tests start a real Gazebo. Everything here exists to keep the individual
test files short enough to read as documentation of what the system is
supposed to do.
"""

import subprocess
import time

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

import launch_testing.actions

import rclpy

PKG = 'lampo_description'
NS = 'r1_'

# NOTE on VPNs: an active VPN can swallow the discovery multicast of both
# gz-transport and DDS. The cure (GZ_IP=127.0.0.1 and
# ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST) must be process-level environment
# -- launch snapshots its environment at startup, so setting os.environ here
# is already too late. CMakeLists.txt sets both on every simulator test; if
# you run a test file directly with launch_test, export them yourself.


def launch_file(name, **arguments):
    """Include one of this package's launch files with the given arguments."""
    path = [FindPackageShare(PKG), '/launch/', name]
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(path),
        launch_arguments=[(k, str(v)) for k, v in arguments.items()],
    )


def simulation(*extra, headless=True):
    """Build a LaunchDescription: the world, plus whatever else is passed.

    Always headless -- these run in CI and on machines without a display, and
    the GUI adds seconds of startup for nothing a test can assert on.
    """
    # The previous test's simulator may still be dying: gz sim rides out the
    # first SIGINT for seconds, and its server process ("gz sim server", no
    # world name in its command line) can outlive the wrapper entirely. A
    # leftover server silently captures this test's robot spawn -- the log
    # says "Another world of the same name is running" and every assertion
    # afterwards fails mysteriously. Clear the field before launching.
    subprocess.run(['pkill', '-9', '-f', 'gz sim'], check=False)
    time.sleep(1.0)
    return LaunchDescription([
        launch_file('lampo_sandbox.launch.py',
                    gui='false' if headless else 'true', rviz='false'),
        *extra,
        launch_testing.actions.ReadyToTest(),
    ])


def wait_for(predicate, timeout, node=None, period=0.25):
    """Spin until predicate() is true, or timeout seconds elapse.

    Returns whether it succeeded rather than raising, so the calling test can
    produce an assertion message that says what was actually being waited for.
    """
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if node is not None:
            rclpy.spin_once(node, timeout_sec=period)
        else:
            time.sleep(period)
        if predicate():
            return True
    return False


def wait_for_topic(node, topic, timeout=60.0):
    """Wait until `topic` has at least one publisher."""
    return wait_for(lambda: node.count_publishers(topic) > 0, timeout, node)


def collect(node, topic, msg_type, count=1, timeout=60.0):
    """Subscribe to a topic and return the first `count` messages received."""
    received = []
    sub = node.create_subscription(
        msg_type, topic, lambda m: received.append(m),
        rclpy.qos.QoSProfile(depth=10))
    try:
        wait_for(lambda: len(received) >= count, timeout, node)
        return received
    finally:
        node.destroy_subscription(sub)
