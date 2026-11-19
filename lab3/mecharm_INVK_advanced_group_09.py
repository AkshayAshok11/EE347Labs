"""EE 347 Lab 3 Part I, Group 9: bounded pose-error inverse kinematics.

The input pose is [x, y, z, roll, pitch, yaw] in mm and radians. The solver
returns SciPy's result, whose x field contains six encoder angles in radians.
Run validate_ik.py for the two-solver comparison on Lab 2 data.
"""

import numpy as np
from scipy.optimize import least_squares

from mecharm_INVK_group_09 import (
    JOINT_LOWER_RAD,
    JOINT_UPPER_RAD,
    TOOL_LENGTH_MM,
    forward_kinematics,
    transf_to_pose,
    wrap_angles,
)


# For small rotations, angle times the nominal tool length gives a
# displacement scale in mm. Use this scale without fitting to Lab 2 data.
ORIENTATION_WEIGHT_MM_PER_RAD = float(TOOL_LENGTH_MM)
JOINT_LIMITS = (JOINT_LOWER_RAD, JOINT_UPPER_RAD)


def target_pose_error(q_rad, target_pose):
    """Return three position errors and three scaled, wrapped angle errors."""
    current_pose = transf_to_pose(forward_kinematics(q_rad))
    delta = current_pose - target_pose
    delta[3:] = ORIENTATION_WEIGHT_MM_PER_RAD * wrap_angles(delta[3:])
    return delta


def ik(target_pose, q_init, max_iter=1000, tolerance=1e-5, bounds=JOINT_LIMITS):
    """Fit all six joints at once under the Lab 3 encoder limits."""
    target_pose = np.asarray(target_pose, dtype=float)
    q_init = np.asarray(q_init, dtype=float)
    return least_squares(
        target_pose_error, q_init, args=(target_pose,), method="trf",
        bounds=bounds, max_nfev=max_iter,
        ftol=tolerance, xtol=tolerance, gtol=tolerance,
    )
