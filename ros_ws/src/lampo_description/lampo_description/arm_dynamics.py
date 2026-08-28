"""The arm's dynamics, straight from the URDF.

Shared by the dynamics labs (zero_g.py, joint_observer.py, lqr_joint.py).
Pinocchio parses the same robot_description string the rest of the stack
uses, so the model here is -- by construction -- the robot in the simulator:
change a mass in the xacro and every lab follows.

Two quantities matter for the exercises:

  g(q)      the generalized gravity vector: the torque each joint must exert
            just to hold still. `computeGeneralizedGravity` evaluates it for
            the current configuration.
  M(q)      the joint-space inertia matrix (via the composite rigid body
            algorithm). The labs only use its diagonal, as the "apparent
            inertia" seen by one joint.

A subtlety worth knowing about: pinocchio stores unbounded (continuous)
joints -- the wheels -- as [cos, sin] pairs inside q, so q cannot be indexed
naively by joint number. Every write below goes through the joint's own
idx_q, and only for the six arm joints; wheels and gripper stay at neutral,
which is exactly where the passive gripper actually is.
"""

import numpy as np

import pinocchio as pin

# The arm joints, unprefixed; the robot carries a namespace prefix (r1_...)
# which is resolved by suffix matching so this module works for any robot.
ARM_JOINTS = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
              'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']


class ArmModel:
    """Gravity and inertia of the six arm joints, from a URDF string."""

    def __init__(self, urdf_xml):
        self.model = pin.buildModelFromXML(urdf_xml)
        self.data = self.model.createData()

        # Map each arm joint (matched by suffix, prefix-agnostic) to its
        # pinocchio joint id and its slots in q (positions) and v (torques).
        self.joint_ids = {}
        for short in ARM_JOINTS:
            matches = [i for i in range(1, self.model.njoints)
                       if str(self.model.names[i]).endswith(short)]
            if len(matches) != 1:
                raise ValueError(f'expected exactly one joint ending in '
                                 f'"{short}", found {len(matches)}')
            self.joint_ids[short] = matches[0]

    def full_name(self, short):
        """Return the prefixed joint name as it appears in joint_states."""
        return str(self.model.names[self.joint_ids[short]])

    def _q(self, positions):
        """Build the configuration vector from a {full_name: angle} dict."""
        q = pin.neutral(self.model)
        for short, jid in self.joint_ids.items():
            name = str(self.model.names[jid])
            if name in positions:
                q[self.model.joints[jid].idx_q] = positions[name]
        return q

    def gravity(self, positions):
        """g(q) for the six arm joints, as {full_name: torque}."""
        g = pin.computeGeneralizedGravity(self.model, self.data,
                                          self._q(positions))
        return {str(self.model.names[jid]): g[self.model.joints[jid].idx_v]
                for jid in self.joint_ids.values()}

    def inertia(self, positions, short):
        """Return the diagonal inertia M_jj(q) of one arm joint [kg m^2]."""
        M = pin.crba(self.model, self.data, self._q(positions))
        idx = self.model.joints[self.joint_ids[short]].idx_v
        return float(M[idx, idx])

    def inertia_diag(self, positions):
        """Return every arm joint's diagonal inertia, as {full_name: M_jj}."""
        M = pin.crba(self.model, self.data, self._q(positions))
        return {str(self.model.names[jid]):
                float(M[self.model.joints[jid].idx_v,
                        self.model.joints[jid].idx_v])
                for jid in self.joint_ids.values()}

    def effort_limit(self, short):
        """Return the joint's torque limit from the URDF [Nm]."""
        idx = self.model.joints[self.joint_ids[short]].idx_v
        return float(self.model.effortLimit[idx])


def observer_gain(poles):
    """Luenberger gain L for the 2-state chain x=[q, qd], y=q.

    A = [[0, 1], [0, 0]], C = [1, 0]. The observer error dynamics are
    det(sI - (A - LC)) = s^2 + l1 s + l2, so matching the desired
    (s - p1)(s - p2) = s^2 - (p1+p2) s + p1 p2 gives L in closed form --
    no numerics, just the algebra from the lecture.
    """
    p1, p2 = poles
    if p1 >= 0 or p2 >= 0:
        raise ValueError('observer poles must be strictly negative')
    return np.array([-(p1 + p2), p1 * p2])


def lqr_gain(inertia, q_pos, q_vel, r):
    """LQR gain K for one joint linearized as a double integrator.

    xdot = A x + B u with A = [[0, 1], [0, 0]], B = [[0], [1/M]];
    cost = integral( q_pos*e^2 + q_vel*ed^2 + r*u^2 ). Solves the
    continuous-time algebraic Riccati equation and returns K (shape (2,)).
    """
    from scipy.linalg import solve_continuous_are

    A = np.array([[0.0, 1.0], [0.0, 0.0]])
    B = np.array([[0.0], [1.0 / inertia]])
    Q = np.diag([q_pos, q_vel])
    R = np.array([[r]])
    P = solve_continuous_are(A, B, Q, R)
    return (np.linalg.solve(R, B.T @ P)).flatten()
