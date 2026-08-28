"""The control-lab math is right before a simulator ever runs.

The dynamics labs (doc/control.md, exercises 4-7) rest on a few pure
functions: the observer's pole placement, the LQR Riccati solution, and the
effort-controller wiring. Each has a failure mode that would otherwise only
surface as "the arm behaves strangely" -- the least debuggable symptom there
is. These tests pin the math and the wiring in milliseconds.
"""

import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).parents[1]))

from conftest import load_config  # noqa: E402

from lampo_description.arm_dynamics import (ARM_JOINTS, lqr_gain,  # noqa: E402
                                            observer_gain)

import numpy as np  # noqa: E402

import pytest  # noqa: E402


def test_observer_gain_places_the_poles():
    """L follows from matching coefficients -- checkable by hand.

    For A=[[0,1],[0,0]], C=[1,0]: det(sI-(A-LC)) = s^2 + l1 s + l2, and
    (s+8)(s+10) = s^2 + 18 s + 80.
    """
    l1, l2 = observer_gain([-8.0, -10.0])
    assert l1 == pytest.approx(18.0)
    assert l2 == pytest.approx(80.0)


def test_observer_rejects_unstable_poles():
    """A pole in the right half plane is a diverging estimator, not a lab."""
    with pytest.raises(ValueError):
        observer_gain([-8.0, 0.5])


def test_lqr_gain_stabilizes_the_joint():
    """The closed loop A - B K has strictly negative eigenvalues."""
    inertia = 45.0                       # the shoulder's apparent inertia
    K = lqr_gain(inertia, 100.0, 10.0, 0.01)
    A = np.array([[0.0, 1.0], [0.0, 0.0]])
    B = np.array([[0.0], [1.0 / inertia]])
    poles = np.linalg.eigvals(A - B @ K.reshape(1, 2))
    assert all(p.real < 0 for p in poles), f'unstable closed loop: {poles}'


def test_lqr_gain_depends_only_on_weight_ratios():
    """Scaling Q and R together leaves K unchanged.

    doc/control.md exercise 6 asks students to verify this; the suite holds
    the docs to it.
    """
    K1 = lqr_gain(45.0, 100.0, 10.0, 0.01)
    K2 = lqr_gain(45.0, 100.0 * 1e3, 10.0 * 1e3, 0.01 * 1e3)
    assert K1 == pytest.approx(K2, rel=1e-6)


def test_effort_controller_is_wired_for_the_arm():
    """The raw-torque passthrough exists and claims exactly the arm joints.

    zero_g.py, joint_observer.py and lqr_joint.py all assume the command
    vector is these six joints in this order; a reorder in the YAML would
    send the shoulder's torque to the wrist.
    """
    config = load_config('ur_controllers.yaml')['/**']
    types = config['controller_manager']['ros__parameters']
    assert types['forward_effort_controller']['type'] == \
        'effort_controllers/JointGroupEffortController'

    joints = config['forward_effort_controller']['ros__parameters']['joints']
    assert joints == [f'PREFIX_{j}' for j in ARM_JOINTS]
