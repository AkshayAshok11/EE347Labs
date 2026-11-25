"""EE 347 Lab 3, Group 9: pose-error inverse kinematics and pick-and-place.

Self-contained: symbolic FK, numerical FK, transform-to-pose, pose error and the
bounded least-squares IK solver all live in this file.

Conventions
-----------
* Joint angles: encoder angles, radians inside the solver, degrees on the robot.
* Pose vector: [x, y, z, roll, pitch, yaw], mm and radians,
  R = Rz(yaw) @ Ry(pitch) @ Rx(roll). This matches get_coords() on the robot.
* "Flange point" = the point the robot reports with get_coords().
* "Jaw point"    = centre of the gripper jaws, TCP_OFFSET_MM beyond the flange
  along the tool z-axis. Waypoints are specified at the jaw point.
* Base frame: origin on the table under J1, z up. x/y as in get_coords().

Usage
-----
    python3 mecharm_INVK_advanced_group_09.py --dry-run   # IK only, no robot
    python3 mecharm_INVK_advanced_group_09.py             # run on the robot (Pi)
    python3 mecharm_INVK_advanced_group_09.py --auto      # no Enter between steps

EDIT THE "LAYOUT" SECTION BELOW BEFORE RUNNING ON THE ROBOT.
"""

import argparse
import csv
import time

import numpy as np
from scipy.optimize import least_squares
from sympy import Matrix, Rational, cos, eye, lambdify, pi, sin, symbols

# ============================================================================
# LAYOUT  (EDIT THESE: all values in mm, base frame, table surface at z = 0)
# ============================================================================
OBJECT_XY = (150.0, -60.0)     # centre of the object to pick up
OBJECT_HEIGHT = 20.0           # object height (marker lying flat ~ its diameter)
BLOCK_XY = (150.0, 60.0)       # centre of the top face of the wooden block
BLOCK_HEIGHT = 30.0            # height of the block's top face above the table

TCP_OFFSET_MM = 70.0           # flange point -> jaw centre (CHECK on your gripper)
APPROACH_MM = 60.0             # hover height above grasp / place height
PLACE_CLEARANCE_MM = 3.0       # release the object this far above the block
LIFT_TARGET_MM = 150.0         # object bottom must clear the table by this much
LIFT_MARGIN_MM = 25.0          # extra margin on top of LIFT_TARGET_MM
LIFT_TILT_DEG = -45.0          # tool tilt while lifting (negative = jaws outward);
                               # needed because a vertical tool cannot reach 150 mm
MIN_JAW_Z_MM = 8.0             # refuse any waypoint whose jaw point is lower
MAX_RADIUS_MM = 230.0          # refuse any waypoint farther than this from J1

# ============================================================================
# ROBOT / SOLVER SETTINGS
# ============================================================================
SPEED = 30                     # send_angles speed, 1-100
MOVE_TIME = 4.0                # seconds to wait for each move
GRIPPER_SPEED = 50
GRIPPER_WAIT = 1.5
HOME = [0, 0, 0, 0, 0, 0]      # neutral pose, degrees

IK_TOL_MM = 0.5                # a solution is accepted below this position residual
IK_TOL_DEG = 0.5               # ... and below this orientation residual
REFINE_ITERS = 2               # feedback corrections at pick/place waypoints
REFINE_THRESHOLD_MM = 2.0      # skip correction if already this close
REFINE_MAX_MM = 30.0           # abort if the robot is farther off than this
ARRIVE_TOL_DEG = 5.0           # warn if a joint ends this far from its command
MOVE_TIMEOUT = 40.0           # longest we wait for one move to finish (seconds)
LOG_FILE = "pick_place_log.csv"
BLIND_MODE = False             # set at run time if the robot cannot be read (no feedback)

# ============================================================================
# FORWARD KINEMATICS  (same model as the basic solver)
# ============================================================================
JOINT_SYMBOLS = symbols("q1:7", real=True)
q1, q2, q3, q4, q5, q6 = JOINT_SYMBOLS

TOOL_LENGTH_MM = Rational("65.5")

# Standard DH rows (a, alpha, d, theta); the -pi/2 J2 offset is inside theta.
DH_PARAMETERS = (
    (0, -pi / 2, Rational("136.5"), q1),
    (100, 0, 0, q2 - pi / 2),
    (0, -pi / 2, 0, q3),
    (0, pi / 2, Rational("107.5"), q4),
    (0, -pi / 2, 0, q5),
    (0, 0, TOOL_LENGTH_MM, q6),
)

# Joint limits from Lab 3 Annex 1 (encoder angles, before the DH offset).
JOINT_LOWER_RAD = np.deg2rad([-165, -90, -180, -165, -115, -175])
JOINT_UPPER_RAD = np.deg2rad([165, 90, 65, 165, 115, 175])
JOINT_LIMITS = (JOINT_LOWER_RAD, JOINT_UPPER_RAD)

# Scales orientation error (rad) into mm so it is comparable with position error.
ORIENTATION_WEIGHT_MM_PER_RAD = float(TOOL_LENGTH_MM)


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


# Chain the six joint transforms symbolically.
T_BASE_FLANGE = eye(4)
for _row in DH_PARAMETERS:
    T_BASE_FLANGE = T_BASE_FLANGE * get_transformation_matrix(*_row)

# Measured on the Lab 2 data: the robot's reported frame is the DH flange
# frame rotated by pi about z, so postmultiply by diag(-1, -1, 1, 1).
T_FLANGE_REPORTED = Matrix.diag(-1, -1, 1, 1)
SYMBOLIC_FK = T_BASE_FLANGE * T_FLANGE_REPORTED

# Convert the symbolic result into a fast numerical function of six angles.
forward_kinematics_func = lambdify(JOINT_SYMBOLS, SYMBOLIC_FK, modules="numpy", cse=True)


def symbolic_forward_kinematics(q_values=JOINT_SYMBOLS):
    """Substitute six joint values into the symbolic base-to-reported FK."""
    values = tuple(q_values)
    if len(values) != len(JOINT_SYMBOLS):
        raise ValueError("Provide six encoder angles.")
    return SYMBOLIC_FK.subs(dict(zip(JOINT_SYMBOLS, values)), simultaneous=True)


def forward_kinematics(q_rad):
    """Evaluate the 4x4 base-to-reported transform (angles in radians)."""
    return np.array(forward_kinematics_func(*q_rad), dtype=float)


def transf_to_pose(T):
    """Convert a 4x4 transform to [x, y, z, roll, pitch, yaw] (mm, rad)."""
    T = np.asarray(T, dtype=float)
    horizontal = np.hypot(T[0, 0], T[1, 0])
    pitch = np.arctan2(-T[2, 0], horizontal)
    if horizontal < 1e-12:
        # Gimbal lock: roll is not observable, so fix it at zero.
        roll = 0.0
        yaw = np.arctan2(-T[0, 1], T[1, 1])
    else:
        roll = np.arctan2(T[2, 1], T[2, 2])
        yaw = np.arctan2(T[1, 0], T[0, 0])
    return np.array([*T[:3, 3], roll, pitch, yaw])


def wrap_angles(angles_rad):
    """Map angles or angle differences into [-pi, pi)."""
    return (np.asarray(angles_rad) + np.pi) % (2 * np.pi) - np.pi


# ============================================================================
# INVERSE KINEMATICS  (pose-error, all six joints at once)
# ============================================================================
def target_pose_error(q_rad, target_pose):
    """6-vector residual: XYZ error (mm) and wrapped angle error scaled to mm.

    least_squares estimates the Jacobian of this function numerically (finite
    differences), so no analytic Jacobian is written anywhere in this file.
    """
    delta = transf_to_pose(forward_kinematics(q_rad)) - target_pose
    delta[3:] = ORIENTATION_WEIGHT_MM_PER_RAD * wrap_angles(delta[3:])
    return delta


def ik(target_pose, q_init, max_iter=1000, tolerance=1e-5, bounds=JOINT_LIMITS):
    """Solve for six joint angles under the joint limits (trust-region, 'trf')."""
    target_pose = np.asarray(target_pose, dtype=float)
    lo, hi = np.asarray(bounds[0]), np.asarray(bounds[1])
    # least_squares requires the start point strictly inside the bounds.
    q_init = np.clip(np.asarray(q_init, dtype=float), lo + 1e-6, hi - 1e-6)
    return least_squares(
        target_pose_error, q_init, args=(target_pose,), method="trf",
        bounds=bounds, max_nfev=max_iter,
        ftol=tolerance, xtol=tolerance, gtol=tolerance,
    )


def residuals(result):
    """Return (position error mm, orientation error deg) of an IK result."""
    r = result.fun
    return float(np.linalg.norm(r[:3])), float(np.degrees(np.linalg.norm(r[3:]) / ORIENTATION_WEIGHT_MM_PER_RAD))


def solve_waypoint(target_pose, q_seed, n_random=12):
    """IK with a warm start, then random restarts as a fallback.

    Among all converged solutions, return the one closest in joint space to
    q_seed, so consecutive waypoints do not flip the elbow or wrist.
    """
    rng = np.random.default_rng(0)
    seeds = [np.asarray(q_seed, dtype=float)]
    seeds += [np.deg2rad(rng.uniform(-60, 60, 6)) for _ in range(n_random)]
    good = []
    for i, seed in enumerate(seeds):
        result = ik(target_pose, seed)
        pos_mm, rot_deg = residuals(result)
        if pos_mm < IK_TOL_MM and rot_deg < IK_TOL_DEG:
            good.append(result)
        if i == 0 and good:
            break                      # warm start worked: keep the smooth path
    if not good:
        raise RuntimeError("IK did not converge for target pose %s" % np.round(target_pose, 3))
    return min(good, key=lambda r: np.linalg.norm(wrap_angles(r.x - q_seed)))


# ============================================================================
# WAYPOINT GEOMETRY
# ============================================================================
def tool_rotation(azimuth_rad, tilt_deg):
    """Rotation matrix for a gripper pointing down, tilted in the radial plane.

    tilt = 0 points straight down; negative tilt swings the jaws outward, away
    from the base. Yaw follows the azimuth so J6 stays near zero.
    """
    def rz(a):
        c, s = np.cos(a), np.sin(a)
        return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]])

    def ry(a):
        c, s = np.cos(a), np.sin(a)
        return np.array([[c, 0, s], [0, 1.0, 0], [-s, 0, c]])

    rx_pi = np.diag([1.0, -1.0, -1.0])           # Rx(pi): flips z to point down
    return rz(azimuth_rad) @ ry(np.deg2rad(tilt_deg)) @ rz(np.pi) @ rx_pi


def jaw_to_flange_pose(jaw_xyz, tilt_deg):
    """Convert a jaw-point position + tilt into the pose the IK must reach."""
    jaw_xyz = np.asarray(jaw_xyz, dtype=float)
    az = np.arctan2(jaw_xyz[1], jaw_xyz[0])
    R = tool_rotation(az, tilt_deg)
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = jaw_xyz - TCP_OFFSET_MM * R[:, 2]  # flange sits behind the jaws
    return transf_to_pose(T)


def build_waypoints():
    """Return the pick-and-place sequence as a list of waypoint dicts."""
    ox, oy = OBJECT_XY
    bx, by = BLOCK_XY
    grasp_z = OBJECT_HEIGHT / 2.0
    place_z = BLOCK_HEIGHT + OBJECT_HEIGHT / 2.0 + PLACE_CLEARANCE_MM
    lift_z = LIFT_TARGET_MM + OBJECT_HEIGHT / 2.0 + LIFT_MARGIN_MM
    return [
        dict(name="pick_hover", jaw=(ox, oy, grasp_z + APPROACH_MM), tilt=0, refine=False, after=None),
        dict(name="pick", jaw=(ox, oy, grasp_z), tilt=0, refine=True, after="close"),
        dict(name="pick_rise", jaw=(ox, oy, grasp_z + APPROACH_MM), tilt=0, refine=False, after=None),
        dict(name="lift", jaw=(ox, oy, lift_z), tilt=LIFT_TILT_DEG, refine=False, after=None),
        dict(name="place_hover", jaw=(bx, by, place_z + APPROACH_MM), tilt=0, refine=False, after=None),
        dict(name="place", jaw=(bx, by, place_z), tilt=0, refine=True, after="open"),
        dict(name="place_retreat", jaw=(bx, by, place_z + APPROACH_MM), tilt=0, refine=False, after=None),
    ]


def check_waypoint(wp):
    """Raise if a waypoint is outside the safety envelope set in the config."""
    x, y, z = wp["jaw"]
    if z < MIN_JAW_Z_MM:
        raise ValueError(f"{wp['name']}: jaw z={z:.1f} mm is below MIN_JAW_Z_MM")
    if np.hypot(x, y) > MAX_RADIUS_MM:
        raise ValueError(f"{wp['name']}: radius {np.hypot(x, y):.0f} mm exceeds MAX_RADIUS_MM")


def plan(waypoints):
    """Solve IK for every waypoint in order, warm-starting from the last one."""
    q_prev = np.zeros(6)
    for wp in waypoints:
        check_waypoint(wp)
        wp["pose"] = jaw_to_flange_pose(wp["jaw"], wp["tilt"])
        result = solve_waypoint(wp["pose"], q_prev)
        wp["q"] = result.x
        wp["ik_pos_mm"], wp["ik_rot_deg"] = residuals(result)
        q_prev = result.x
    return waypoints


def print_plan(waypoints):
    print(f"{'step':<14}{'jaw x,y,z (mm)':<26}{'tilt':>6}  joint angles (deg)               ik err mm/deg")
    for wp in waypoints:
        jaw = ", ".join(f"{v:6.1f}" for v in wp["jaw"])
        q = " ".join(f"{v:7.1f}" for v in np.degrees(wp["q"]))
        print(f"{wp['name']:<14}{jaw:<26}{wp['tilt']:>6.0f}  {q}   {wp['ik_pos_mm']:.3f}/{wp['ik_rot_deg']:.3f}")


# ============================================================================
# ROBOT I/O
# ============================================================================
def _valid(value):
    """True if the robot returned a list/tuple of six numbers."""
    return isinstance(value, (list, tuple)) and len(value) == 6


def read_state(mc, tries=30, delay=0.5):
    """Return (angles_deg, coords) where coords may be None if unreadable.

    The robot sometimes answers -1, None or [] (right after power-on or a
    move), so reads are retried. Joint angles are required; coordinates are
    only needed for the feedback correction, so a failed get_coords() returns
    None instead of stopping the run.
    """
    if BLIND_MODE:
        return None, None
    angles = coords = None
    for _ in range(tries):
        angles = mc.get_angles()
        if _valid(angles):
            break
        time.sleep(delay)
    else:
        raise RuntimeError(
            f"Could not read joint angles after {tries} tries (get_angles -> {angles!r}). "
            "Check power, the serial connection, and that no other program uses the port. "
            "Run diagnose_robot.py to see exactly what the robot returns.")
    for _ in range(tries // 4):
        time.sleep(0.05)                   # do not hit the serial port back-to-back
        coords = mc.get_coords()
        if _valid(coords):
            return np.array(angles, dtype=float), np.array(coords, dtype=float)
        time.sleep(delay)
    print(f"   WARNING: get_coords() returned {coords!r}; continuing without coordinates")
    return np.array(angles, dtype=float), None


def wait_for_motion(mc, min_wait=None):
    """Wait for the arm to stop moving before anything is read back.

    The robot answers -1 to every read while it is still moving, and a move
    from far away can take longer than a fixed wait. So: sleep a minimum time,
    then poll is_moving() until it reports 0 twice in a row (a -1 answer
    counts as "still moving"), then let the arm settle.
    """
    time.sleep(MOVE_TIME if min_wait is None else min_wait)
    if BLIND_MODE:
        return
    start, calm = time.time(), 0
    while time.time() - start < MOVE_TIMEOUT:
        try:
            moving = mc.is_moving()
        except Exception:
            moving = -1
        calm = calm + 1 if moving == 0 else 0
        if calm >= 2:
            break
        time.sleep(0.4)
    else:
        print("   WARNING: the arm still reports moving; reading anyway")
    time.sleep(0.5)


def go_home(mc, auto, tries=3):
    """Send the neutral pose and confirm the arm got there, retrying if not."""
    for attempt in range(1, tries + 1):
        mc.send_angles(HOME, SPEED)
        wait_for_motion(mc)
        if BLIND_MODE:
            return input("   (no feedback) Is the arm at the neutral pose? (y/n): ").strip().lower() == "y"
        try:
            angles, _ = read_state(mc)
        except RuntimeError as exc:
            print(f"   could not confirm the pose: {exc}")
            answer = input("   Is the arm at the neutral pose? (y/n): ").strip().lower()
            return answer == "y"
        worst = float(np.max(np.abs(angles - np.array(HOME))))
        if worst <= ARRIVE_TOL_DEG:
            print(f"   at neutral pose (worst joint {worst:.1f} deg from zero)")
            return True
        print(f"   neutral attempt {attempt}/{tries}: worst joint {worst:.1f} deg away, retrying")
        mc.power_on()                      # re-energise the servos in case they dropped out
        time.sleep(1)
    print("   WARNING: the arm did not reach the neutral pose. Check power and servos.")
    return False


def calibrate_gripper(mc):
    """Ask the operator which flag opens the gripper, then return (open, close)."""
    mc.set_gripper_state(0, GRIPPER_SPEED, 1)
    time.sleep(GRIPPER_WAIT)
    is_open = input("Sent gripper flag 0. Is the gripper OPEN? (y/n): ").strip().lower() == "y"
    return (0, 1) if is_open else (1, 0)


def pause(message, auto):
    """Wait for Enter before a motion unless running with --auto."""
    if not auto:
        input(f"\n>> {message}  [Enter to continue, Ctrl+C to stop] ")
    else:
        print(f"\n>> {message}")


def move_and_log(mc, wp, auto, writer, open_flag, close_flag):
    """Send one waypoint, refine it with encoder feedback, log, then grip/release."""
    q_deg = np.degrees(wp["q"])
    pause(f"Move to {wp['name']}: jaw {np.round(wp['jaw'], 1)} mm, tilt {wp['tilt']:.0f} deg", auto)
    mc.send_angles(q_deg.tolist(), SPEED)
    wait_for_motion(mc)

    angles, coords = read_state(mc)
    if angles is None:                     # blind mode: nothing can be read back
        angles = np.full(6, np.nan)
    target = wp["pose"].copy()
    refine_used = 0
    if wp["refine"] and coords is None:
        print("   skipping feedback correction (no coordinates from the robot)")
    elif wp["refine"]:
        # The FK model differs from the robot's own kinematics by ~10 mm, so
        # nudge the target by the observed XYZ error and re-solve (1-2 times).
        for _ in range(REFINE_ITERS):
            err = wp["pose"][:3] - coords[:3]
            if np.linalg.norm(err) < REFINE_THRESHOLD_MM:
                break
            if np.linalg.norm(err) > REFINE_MAX_MM:
                raise RuntimeError(f"{wp['name']}: robot is {np.linalg.norm(err):.0f} mm off target, aborting")
            target[:3] += err
            result = solve_waypoint(target, np.radians(angles))
            q_deg = np.degrees(result.x)
            print(f"   refine {refine_used + 1}: error {np.linalg.norm(err):.1f} mm, re-sending")
            mc.send_angles(q_deg.tolist(), SPEED)
            wait_for_motion(mc, MOVE_TIME / 2)
            angles, coords = read_state(mc)
            refine_used += 1
            if coords is None:
                break

    arrive_err = np.max(np.abs(angles - q_deg))
    if arrive_err > ARRIVE_TOL_DEG:
        print(f"   WARNING: a joint is {arrive_err:.1f} deg from its command")
    if coords is None:
        coords = np.full(6, np.nan)
        xyz_err = float("nan")
        print("   reported xyz unavailable")
    else:
        xyz_err = np.linalg.norm(coords[:3] - wp["pose"][:3])
        print(f"   reported xyz {np.round(coords[:3], 1)}, error vs target {xyz_err:.1f} mm")

    writer.writerow([wp["name"], *np.round(wp["jaw"], 2), wp["tilt"], *np.round(wp["pose"][:3], 2),
                     *np.round(np.degrees(wp["pose"][3:]), 2), *np.round(q_deg, 2),
                     round(wp["ik_pos_mm"], 4), round(wp["ik_rot_deg"], 4), refine_used,
                     *np.round(angles, 2), *np.round(coords, 2), round(xyz_err, 2)])

    if wp["after"] is not None:
        flag = close_flag if wp["after"] == "close" else open_flag
        pause(f"Gripper {wp['after']}", auto)
        mc.set_gripper_state(flag, GRIPPER_SPEED, 1)
        time.sleep(GRIPPER_WAIT)


LOG_HEADER = (["step", "jaw_x", "jaw_y", "jaw_z", "tilt_deg", "target_x", "target_y", "target_z",
               "target_rx", "target_ry", "target_rz"]
              + [f"cmd_j{i}" for i in range(1, 7)] + ["ik_pos_err_mm", "ik_rot_err_deg", "refine_iters"]
              + [f"actual_j{i}" for i in range(1, 7)]
              + ["actual_x", "actual_y", "actual_z", "actual_rx", "actual_ry", "actual_rz", "xyz_err_mm"])


def run_on_robot(waypoints, auto):
    """Connect, run the whole sequence, and return to the neutral pose."""
    from pymycobot import MechArm270, PI_PORT, PI_BAUD   # imported late: --dry-run needs no robot

    mc = MechArm270(PI_PORT, PI_BAUD)
    time.sleep(1)
    mc.power_on()
    time.sleep(2)                          # let the servos settle before the first command
    global BLIND_MODE
    try:
        read_state(mc, tries=8)
    except RuntimeError as exc:
        print(f"\n{exc}\n")
        if input("Continue WITHOUT reading the robot (no feedback correction, "
                 "no actual-vs-commanded log)? (y/n): ").strip().lower() != "y":
            raise SystemExit("Stopped. Fix the connection first (see diagnose_robot.py).")
        BLIND_MODE = True
    pause("Go to the neutral pose first", auto)
    if not go_home(mc, auto):
        input("Neutral pose not reached. Press Enter to continue anyway, Ctrl+C to stop. ")
    open_flag, close_flag = calibrate_gripper(mc)
    mc.set_gripper_state(open_flag, GRIPPER_SPEED, 1)
    time.sleep(GRIPPER_WAIT)

    with open(LOG_FILE, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(LOG_HEADER)
        try:
            for wp in waypoints:
                move_and_log(mc, wp, auto, writer, open_flag, close_flag)
        finally:
            # Always finish at neutral, even after an error or Ctrl+C.
            print("\nReturning to the neutral pose...")
            go_home(mc, True)
    print(f"Done. Log saved to {LOG_FILE}")


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--dry-run", action="store_true", help="solve and print the plan; do not connect")
    parser.add_argument("--auto", action="store_true", help="do not wait for Enter between moves")
    args = parser.parse_args()

    waypoints = plan(build_waypoints())       # all IK is solved before the arm moves
    print_plan(waypoints)
    if args.dry_run:
        return
    pause("Check the plan above and the LAYOUT section. Start the robot", False)
    run_on_robot(waypoints, args.auto)


if __name__ == "__main__":
    main()
