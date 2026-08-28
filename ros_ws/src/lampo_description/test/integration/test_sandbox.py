"""The world loads, and it loads completely.

The one failure this is really guarding against: the warehouse references nine
meshes by package:// URI, and Gazebo resolves those only if
GZ_SIM_RESOURCE_PATH is set. When it is not, the world still "loads" -- it
just comes up empty, with the errors buried in a wall of startup logging, and
the robot then drives through a warehouse that is not there.
"""

import os
import sys
import unittest

# launch_test executes this file directly, so the directory it lives in is not
# on the import path. Put it there before importing the shared helpers.
sys.path.insert(0, os.path.dirname(__file__))

from helpers import collect, simulation, wait_for_topic  # noqa: E402

import launch_testing  # noqa: E402

import pytest  # noqa: E402

import rclpy  # noqa: E402

from rosgraph_msgs.msg import Clock  # noqa: E402


@pytest.mark.launch_test
def generate_test_description():
    """Launch only the world."""
    return simulation()


class TestSandbox(unittest.TestCase):
    """Assertions against a running, robot-less simulation."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('test_sandbox')

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def test_clock_is_published(self):
        """/clock exists, so everything downstream can run on sim time.

        It is bridged by the sandbox rather than per robot: it belongs to the
        world, and N robots publishing it would race.
        """
        self.assertTrue(wait_for_topic(self.node, '/clock'),
                        'nothing is publishing /clock')

    def test_simulation_time_advances(self):
        """The physics engine is actually stepping, not merely alive."""
        stamps = collect(self.node, '/clock', Clock, count=20, timeout=30.0)
        self.assertGreater(len(stamps), 1, 'only one /clock message arrived')

        first, last = stamps[0].clock, stamps[-1].clock
        elapsed = (last.sec - first.sec) + (last.nanosec - first.nanosec) * 1e-9
        self.assertGreater(elapsed, 0.0, 'simulation time is not advancing')


@launch_testing.post_shutdown_test()
class TestWorldLoadedCleanly(unittest.TestCase):
    """Checks that can only run once the process output is complete."""

    def test_no_unresolved_meshes(self, proc_output):
        """Every package:// URI in the world resolved.

        This is the GZ_SIM_RESOURCE_PATH regression, and it is invisible at
        runtime: the world loads, it is just empty.
        """
        for line in proc_output:
            text = line.text.decode(errors='replace') if isinstance(
                line.text, bytes) else str(line.text)
            self.assertNotIn('could not be resolved', text,
                             f'unresolved resource: {text}')
