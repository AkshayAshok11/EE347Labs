"""Run the EE 347 Lab 3 Part I solvers on Group 9's measured Lab 2 poses.

Requires NumPy, SciPy and SymPy. From the workspace root:
    python lab3/validate_ik.py
    python lab3/validate_ik.py --csv path/to/data.csv --output results.csv

By default, reads the ten recorded poses directly from the Lab 2 archive
and writes part1_validation.csv beside this script.
"""

import argparse
import csv
import io
from pathlib import Path
from zipfile import ZipFile

import numpy as np
from scipy.spatial.transform import Rotation

from mecharm_INVK_advanced_group_09 import ik
from mecharm_INVK_group_09 import (
    JOINT_LOWER_RAD,
    JOINT_UPPER_RAD,
    forward_kinematics,
    inverse_kinematics,
    transf_to_pose,
    wrap_angles,
)


LAB_DIR = Path(__file__).resolve().parent
LAB2_ARCHIVE = LAB_DIR.parent / "lab2" / "EE347_UW_Lab2_Group9.zip"
POSE_CSV = "Challenge_2_Drag_Teaching/data.csv"
POSE_COLUMNS = ["x", "y", "z", "rx", "ry", "rz"]
FIXED_SEED_RAD = np.full(6, 0.1)


def read_samples(file):
    """Read measured encoder angles and target poses in solver units."""
    reader = csv.DictReader(file)
    prefix = "j" if "j1" in (reader.fieldnames or []) else "q"
    joint_columns = [f"{prefix}{i}" for i in range(1, 7)]
    columns = joint_columns + POSE_COLUMNS
    if not set(columns).issubset(reader.fieldnames or []):
        raise ValueError("Expected CSV columns " + ", ".join(columns))
    samples = []
    for row in reader:
        try:
            values = np.array([float(row[column]) for column in columns])
        except (TypeError, ValueError) as exc:
            raise ValueError(f"Invalid measurement at CSV line {reader.line_num}") from exc
        if not np.all(np.isfinite(values)):
            raise ValueError(f"Nonfinite measurement at CSV line {reader.line_num}")
        q_recorded = np.deg2rad(values[:6])
        target_pose = np.concatenate((values[6:9], np.deg2rad(values[9:])))
        samples.append((q_recorded, target_pose))
    if not samples:
        raise ValueError("The CSV contains no measurements.")
    return samples


def assess_solution(sample_number, solver, q_rad, q_recorded, target_pose, results):
    """Measure the complete FK pose and preserve solver termination details."""
    transform = forward_kinematics(q_rad)
    pose = transf_to_pose(transform)
    xyz_delta = pose[:3] - target_pose[:3]
    target_rotation = Rotation.from_euler("xyz", target_pose[3:])
    achieved_rotation = Rotation.from_matrix(transform[:3, :3])
    rotation_error = (target_rotation.inv() * achieved_rotation).magnitude()
    within_limits = np.all((q_rad >= JOINT_LOWER_RAD) & (q_rad <= JOINT_UPPER_RAD))

    row = {
        "sample": sample_number,
        "solver": solver,
        "optimizer_success": all(result.success for result in results),
        "status": "/".join(str(result.status) for result in results),
        "message": " | ".join(result.message.strip() for result in results),
        "nfev": sum(result.nfev for result in results),
        "within_joint_limits": bool(within_limits),
        "position_error_mm": np.linalg.norm(xyz_delta),
        "rotation_error_deg": np.rad2deg(rotation_error),
    }
    joint_delta = np.rad2deg(wrap_angles(q_rad - q_recorded))
    for i, (recorded, solved, delta) in enumerate(
        zip(np.rad2deg(q_recorded), np.rad2deg(q_rad), joint_delta), start=1
    ):
        row[f"recorded_j{i}_deg"] = recorded
        row[f"solved_j{i}_deg"] = solved
        row[f"delta_j{i}_deg"] = delta

    target_display = np.concatenate((target_pose[:3], np.rad2deg(target_pose[3:])))
    pose_display = np.concatenate((pose[:3], np.rad2deg(pose[3:])))
    for name, unit, target, achieved in zip(
        POSE_COLUMNS, ("mm", "mm", "mm", "deg", "deg", "deg"),
        target_display, pose_display,
    ):
        row[f"target_{name}_{unit}"] = target
        row[f"achieved_{name}_{unit}"] = achieved
    return row


def compare_solvers(samples):
    rows = []
    for number, (q_recorded, target_pose) in enumerate(samples, start=1):
        q_basic, position, orientation = inverse_kinematics(target_pose, FIXED_SEED_RAD)
        advanced_fixed = ik(target_pose, FIXED_SEED_RAD)
        basic_seed = np.clip(q_basic, JOINT_LOWER_RAD, JOINT_UPPER_RAD)
        advanced_from_basic = ik(target_pose, basic_seed)
        solutions = (
            ("basic", q_basic, (position, orientation)),
            ("advanced_fixed", advanced_fixed.x, (advanced_fixed,)),
            ("advanced_from_basic", advanced_from_basic.x, (advanced_from_basic,)),
        )
        for name, q_rad, results in solutions:
            rows.append(assess_solution(number, name, q_rad, q_recorded, target_pose, results))
    return rows


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--csv", type=Path, help="Use a CSV instead of the Lab 2 archive")
    parser.add_argument("--output", type=Path, default=LAB_DIR / "part1_validation.csv")
    args = parser.parse_args()
    if args.csv:
        with args.csv.open(newline="", encoding="utf-8-sig") as file:
            samples = read_samples(file)
    else:
        with ZipFile(LAB2_ARCHIVE) as archive, archive.open(POSE_CSV) as raw_file:
            with io.TextIOWrapper(raw_file, encoding="utf-8-sig", newline="") as file:
                samples = read_samples(file)

    rows = compare_solvers(samples)
    with args.output.open("w", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(file, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)

    print(f"{'Solver':<22} {'XYZ RMSE (mm)':>14} {'Rotation RMSE (deg)':>20}"
          f" {'Within limits':>15} {'Solver success':>15}")
    for name in dict.fromkeys(row["solver"] for row in rows):
        cases = [row for row in rows if row["solver"] == name]
        xyz_rmse = np.sqrt(np.mean([row["position_error_mm"] ** 2 for row in cases]))
        rotation_rmse = np.sqrt(np.mean([row["rotation_error_deg"] ** 2 for row in cases]))
        feasible = sum(row["within_joint_limits"] for row in cases)
        success = sum(row["optimizer_success"] for row in cases)
        print(f"{name:<22} {xyz_rmse:14.3f} {rotation_rmse:20.3f}"
              f" {feasible:>12}/{len(cases):<2} {success:>12}/{len(cases):<2}")
    print(f"Saved {len(rows)} results to {args.output}")


if __name__ == "__main__":
    main()
