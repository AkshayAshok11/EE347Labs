"""EE 347 Lab 3 Part I, Group 9: forward kinematics and basic split IK.

Joint angles are encoder angles in radians. Cartesian poses are XYZ in mm
followed by roll, pitch and yaw in radians (R = Rz(yaw) Ry(pitch) Rx(roll)).
Run validate_ik.py to evaluate both solvers on the recorded Lab 2 targets.
"""

import numpy as np
from scipy.optimize import least_squares
from sympy import Matrix, Rational, cos, eye, lambdify, pi, sin, symbols


JOINT_SYMBOLS = symbols("q1:7", real=True)
q1, q2, q3, q4, q5, q6 = JOINT_SYMBOLS

TOOL_LENGTH_MM = Rational("65.5")

# Standard Lab 2 DH rows (a, alpha, d, theta), with the J2 offset included.
DH_PARAMETERS = (
    (0, -pi / 2, Rational("136.5"), q1),
    (100, 0, 0, q2 - pi / 2),
    (0, -pi / 2, 0, q3),
    (0, pi / 2, Rational("107.5"), q4),
    (0, -pi / 2, 0, q5),
    (0, 0, TOOL_LENGTH_MM, q6),
)

# Lab 3 Annex 1 limits, applied to encoder angles before the DH offset.
JOINT_LOWER_RAD = np.deg2rad([-165, -90, -180, -165, -115, -175])
JOINT_UPPER_RAD = np.deg2rad([165, 90, 65, 165, 115, 175])


def get_transformation_matrix(a, alpha, d, theta):
    """Return the standard DH transform from a joint frame to its parent."""
    c, s = cos(theta), sin(theta)
    ca, sa = cos(alpha), sin(alpha)
    return Matrix([
        [c, -s * ca, s * sa, a * c],
        [s, c * ca, -c * sa, a * s],
        [0, sa, ca, d],
        [0, 0, 0, 1],
    ])


T_BASE_FLANGE = eye(4)
for row in DH_PARAMETERS:
    T_BASE_FLANGE = T_BASE_FLANGE * get_transformation_matrix(*row)

# For all ten Lab 2 rows, R_fk.T @ R_reported is approximately Rz(pi).
# Postmultiply to express the same point in the robot's reported orientation.
T_FLANGE_REPORTED = Matrix.diag(-1, -1, 1, 1)
SYMBOLIC_FK = T_BASE_FLANGE * T_FLANGE_REPORTED
forward_kinematics_func = lambdify(JOINT_SYMBOLS, SYMBOLIC_FK, modules="numpy", cse=True)


def symbolic_forward_kinematics(q_values=JOINT_SYMBOLS):
    """Substitute six encoder angles into the symbolic base-to-reported FK."""
    values = tuple(q_values)
    if len(values) != len(JOINT_SYMBOLS):
        raise ValueError("Provide six encoder angles.")
    return SYMBOLIC_FK.subs(dict(zip(JOINT_SYMBOLS, values)), simultaneous=True)


def forward_kinematics(q_rad):
    """Evaluate the 4x4 base-to-reported transform at six encoder angles."""
    return forward_kinematics_func(*q_rad)


def transf_to_pose(T):
    """Extract XYZ and canonical roll, pitch, yaw from a transform."""
    horizontal = np.hypot(T[0, 0], T[1, 0])
    pitch = np.arctan2(-T[2, 0], horizontal)
    if horizontal < 1e-12:
        # At gimbal lock, set roll to zero and retain the observable rotation.
        roll = 0.0
        yaw = np.arctan2(-T[0, 1], T[1, 1])
    else:
        roll = np.arctan2(T[2, 1], T[2, 2])
        yaw = np.arctan2(T[1, 0], T[0, 0])
    return np.array([*T[:3, 3], roll, pitch, yaw])


def wrap_angles(angles_rad):
    """Map equivalent angles or angular differences to [-pi, pi)."""
    return (np.asarray(angles_rad) + np.pi) % (2 * np.pi) - np.pi


def position_error(q_position, target_xyz):
    """Compute the basic solver's XYZ error with J4 through J6 at zero."""
    current_xyz = forward_kinematics([*q_position, 0, 0, 0])[:3, 3]
    return current_xyz - target_xyz


def orientation_error(q_orientation, target_rpy):
    """Compute the basic solver's angular error with J1 through J3 at zero."""
    current_rpy = transf_to_pose(forward_kinematics([0, 0, 0, *q_orientation]))[3:]
    return wrap_angles(current_rpy - target_rpy)


def inverse_kinematics(target_pose, q_init, max_iterations=100, tolerance=1e-6):
    """Return the combined joint angles and both split solver results."""
    target_pose = np.asarray(target_pose, dtype=float)
    q_init = np.asarray(q_init, dtype=float)
    position = least_squares(
        position_error, q_init[:3], args=(target_pose[:3],),
        method="lm", max_nfev=max_iterations,
        ftol=tolerance, xtol=tolerance, gtol=tolerance,
    )
    orientation = least_squares(
        orientation_error, q_init[3:], args=(target_pose[3:],),
        method="lm", max_nfev=max_iterations,
        ftol=tolerance, xtol=tolerance, gtol=tolerance,
    )
    q_rad = wrap_angles(np.concatenate((position.x, orientation.x)))
    return q_rad, position, orientation
