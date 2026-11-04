 --"""EE 347 Lab 2, Part IV: MechArm 270 Pi forward kinematics.

Install on the Pi: python3 -m pip install numpy sympy pymycobot

Examples:
  python3 mecharm_frd_K_group_xx.py
  python3 mecharm_frd_K_group_xx.py --angles 10 20 -30 0 15 0
  python3 mecharm_frd_K_group_xx.py --csv part3.csv
  python3 mecharm_frd_K_group_xx.py --csv part3.csv --robot

CSV input header (degrees and mm): j1..j6 (or q1..q6),x,y,z,rx,ry,rz
Each row must contain one stationary Part III measurement. No example
measurements are supplied or represented as experimental results.
Without --robot this script never connects to or moves the arm.

Model: standard DH, A_i = Rz(theta_i) Tz(d_i) Tx(a_i) Rx(alpha_i).
Geometry: d1=136.5, d4=107.5, d6=65.5 mm from handout Annex 1;
a2=100 mm and joint-axis geometry from the manufacturer's MechArm diagram:
https://docs.elephantrobotics.com/docs/mecharm-pi-en/2-serialproduct/2.6-mecharm_270/2.6.1.1%20%20Introduction%20of%20product%20parameters.html
The manufacturer diagram rounds d1,d4,d6 to 136,107,65 mm. These are
nominal dimensions, not measured calibration values for your individual arm.
Annex 2 is a different robot; its DH table must not be copied here.

AI assistance: ChatGPT helped implement and check this program. Cite its
use in the report as required by the handout, describing your actual use.
"""

import argparse
import csv
import time

import numpy as np
from sympy import Matrix, Rational, cos, eye, pi, sin, symbols


# Steps 4-5: symbols and DH rows, in the requested [a, alpha, d, theta] order.
q1, q2, q3, q4, q5, q6 = symbols("q1 q2 q3 q4 q5 q6", real=True)
joint_symbols = [q1, q2, q3, q4, q5, q6]
DH_table = Matrix([
    [0,   -pi / 2, Rational("136.5"), q1],
    [100,       0,                 0, q2],
    [0,   -pi / 2,                 0, q3],
    [0,    pi / 2, Rational("107.5"), q4],
    [0,   -pi / 2,                 0, q5],
    [0,         0, Rational("65.5"),  q6],
])

# Step 8: encoder zero differs from DH zero at joint 2 only.
offsets = [0, -pi / 2, 0, 0, 0, 0]

# Step 11: explicitly identify both the reference frame and the tool point.
# p_report = T_REPORT_BASE * T_BASE_FLANGE * T_FLANGE_TOOL * [0,0,0,1].
# Defaults compare BASE coordinates of the FLANGE. If Part III measured a
# world frame or a pen tip, replace these with the known rigid transforms.
# Do not infer a frame correction from a single point.
T_REPORT_BASE = np.eye(4)
T_FLANGE_TOOL = np.eye(4)


def get_transformation_matrix(a, alpha, d, theta):
    """Step 6: transform from frame i to frame i-1 using standard DH."""
    return Matrix([
        [cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha), a*cos(theta)],
        [sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta)],
        [0,           sin(alpha),            cos(alpha),            d],
        [0,           0,                     0,                     1],
    ])


# Step 7: T_06 = A_1 A_2 A_3 A_4 A_5 A_6.
T = eye(4)
for i in range(DH_table.rows):
    T = T * get_transformation_matrix(*list(DH_table.row(i)))

# Step 9: m=3 and n=3. The fourth column contains x, y, z.
end_effector_position = T[:3, 3]


def forward_kinematics(angles_deg):
    """Steps 8-10: substitute six encoder angles; return numeric T_06."""
    angles = np.asarray(angles_deg, dtype=float)
    if angles.shape != (6,) or not np.all(np.isfinite(angles)):
        raise ValueError("Provide exactly six finite joint angles in degrees.")
    joint_angles = np.radians(angles)
    subs_dict = {
        q: offset + angle
        for q, offset, angle in zip(joint_symbols, offsets, joint_angles)
    }
    return np.array(T.subs(subs_dict).evalf(), dtype=float)


def reported_position(angles_deg):
    """Position in the same frame and at the same point as the CSV data."""
    transform = T_REPORT_BASE @ forward_kinematics(angles_deg) @ T_FLANGE_TOOL
    return transform[:3, 3]


def read_samples(path):
    """Load actual Part III samples, preserving the recorded orientation."""
    coord_columns = ["x", "y", "z", "rx", "ry", "rz"]
    with open(path, newline="", encoding="utf-8-sig") as file:
        reader = csv.DictReader(file)
        # Part III's data.csv names the joints j1..j6; also accept q1..q6.
        prefix = "j" if "j1" in (reader.fieldnames or []) else "q"
        columns = [prefix + str(i) for i in range(1, 7)] + coord_columns
        if not set(columns).issubset(reader.fieldnames or []):
            raise ValueError("CSV must include: " + ",".join(columns))
        samples = []
        for line, row in enumerate(reader, start=2):
            values = np.array([float(row[name]) for name in columns])
            if not np.all(np.isfinite(values)):
                raise ValueError("Nonfinite measurement on CSV line %d" % line)
            samples.append((values[:6].tolist(), values[6:].tolist()))
    if not samples:
        raise ValueError("The CSV contains no measurements.")
    return samples


def validate(samples, output):
    """Step 11: save calculated/reported xyz, signed errors, and distance."""
    errors = []
    with open(output, "w", newline="") as file:
        writer = csv.writer(file)
        writer.writerow(["sample", "fk_x", "fk_y", "fk_z",
                         "reported_x", "reported_y", "reported_z",
                         "dx_mm", "dy_mm", "dz_mm", "error_mm"])
        for number, (angles, coords) in enumerate(samples, start=1):
            xyz = reported_position(angles)
            delta = xyz - np.array(coords[:3])
            error = float(np.linalg.norm(delta))
            errors.append(error)
            writer.writerow([number, *xyz, *coords[:3], *delta, error])
            print("Point %d: FK=%s; reported=%s; error=%.3f mm" %
                  (number, np.round(xyz, 3), coords[:3], error))
    print("Position RMSE: %.3f mm" % np.sqrt(np.mean(np.square(errors))))
    print("Maximum error: %.3f mm" % max(errors))
    print("Saved:", output)


def wait_for_position(robot, target, flag, timeout=30):
    """Wait for acknowledged completion; flag 0=angles, 1=coordinates."""
    time.sleep(0.5)
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if robot.is_in_position(target, flag) == 1:
            time.sleep(0.3)
            return
        time.sleep(0.2)
    # Don't abort the whole demo if one point isn't confirmed; report it and move on.
    print("Warning: robot did not confirm it reached", np.round(target, 1))


def robot_demo(samples, speed):
    """Steps 3, 11, 12: neutral, replay FK xyz, then return to neutral.

    Robot demo uses BASE/FLANGE coordinates and each sample's recorded
    rx,ry,rz. Configure the robot to that same convention before running.
    An interrupted/failed demo stops instead of commanding another motion.
    """
    if not (np.allclose(T_REPORT_BASE, np.eye(4)) and
            np.allclose(T_FLANGE_TOOL, np.eye(4))):
        raise ValueError("Robot demo requires base/flange samples and identity transforms.")
    from pymycobot import MechArm270

    robot = MechArm270("/dev/ttyAMA0", 1000000)
    time.sleep(1)
    home = [0, 0, 0, 0, 0, 0]
    try:
        robot.focus_all_servos()
        time.sleep(0.5)
        robot.send_angles(home, speed)
        wait_for_position(robot, home, 0)
        if samples is None:
            angles = robot.get_angles()
            time.sleep(0.1)
            coords = robot.get_coords()
            if not isinstance(angles, list) or len(angles) != 6:
                raise RuntimeError("Failed to read joint angles.")
            if not isinstance(coords, list) or len(coords) != 6:
                raise RuntimeError("Failed to read Cartesian coordinates.")
            samples = [(angles, coords)]
        for angles, coords in samples:
            x, y, z = forward_kinematics(angles)[:3, 3].tolist()
            # Use measured orientation, as permitted by Part IV step 11.
            target = [x, y, z, *coords[3:6]]
            print("Sending FK pose:", np.round(target, 3))
            robot.send_coords(target, speed, 0)
            wait_for_position(robot, target, 1)
            print("Robot reports:", robot.get_coords())
        robot.send_angles(home, speed)
        wait_for_position(robot, home, 0)
    except (Exception, KeyboardInterrupt):
        robot.stop()
        raise


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--angles", type=float, nargs=6, default=[0]*6)
    parser.add_argument("--csv", help="Part III CSV with the header above")
    parser.add_argument("--output", default="part4_validation.csv")
    parser.add_argument("--robot", action="store_true", help="Enable physical robot motion")
    parser.add_argument("--speed", type=int, default=20)
    args = parser.parse_args()
    if not 1 <= args.speed <= 100:
        parser.error("Speed must be between 1 and 100.")
    if args.robot and args.angles != [0]*6:
        parser.error("--angles is offline only; use --csv for robot playback.")
    samples = read_samples(args.csv) if args.csv else None
    if samples is not None:
        validate(samples, args.output)
    else:
        result = forward_kinematics(args.angles)
        print("Joint angles (degrees):", args.angles)
        print("T_06:\n", np.round(result, 6))
        print("Flange position [x,y,z] (mm):", np.round(result[:3, 3], 3))
    if args.robot:
        robot_demo(samples, args.speed)


if __name__ == "__main__":
    main()