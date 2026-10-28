"""
Lab 2 - Part III: Drag Teaching with the MechArm 270 Pi
File: mecharm_control_group_xx.py   (rename xx to your group number)

What this program does
----------------------
1. record      : Releases all servos so you can move the arm by hand. Each time you
                 press Enter, it reads and displays the current joint angles and
                 end-effector coordinates and saves them to a CSV file (step 7-10).
2. play-angles : Re-locks the servos and replays every recorded point using
                 send_angles(), looping through all points N times (step 11).
3. play-coords : Same playback, but sends the recorded world coordinates with
                 send_coords() instead (step 12).
4. home        : Sends the arm back to the neutral (zero) position (step 15).

During playback, the pose the robot actually reached at every point is also logged
to a CSV, so you have data for the precision/accuracy analysis (steps 13-14).

Usage (run in a terminal on the robot's Raspberry Pi)
-----------------------------------------------------
    python mecharm_control_group_xx.py record
    python mecharm_control_group_xx.py play-angles --loops 3
    python mecharm_control_group_xx.py play-coords --loops 3
    python mecharm_control_group_xx.py home

Optional flags: --file points.csv  --speed 30  --dwell 1.5  --linear (play-coords only)
"""

import argparse
import csv
import sys
import time

# ---------------------------------------------------------------------------
# Imports - the class name differs between pymycobot versions, so try both.
# ---------------------------------------------------------------------------
try:
    from pymycobot import MechArm270 as Arm          # newer pymycobot versions
except ImportError:
    from pymycobot.mecharm import MechArm as Arm     # older pymycobot versions

try:
    from pymycobot import PI_PORT, PI_BAUD
except ImportError:
    # Values for the Raspberry Pi version of the mechArm 270
    PI_PORT, PI_BAUD = "/dev/ttyAMA0", 1000000

# ---------------------------------------------------------------------------
# Settings
# ---------------------------------------------------------------------------
HOME_ANGLES = [0, 0, 0, 0, 0, 0]
ANGLE_TOLERANCE_DEG = 2.0    # how close counts as "arrived" when sending angles
COORD_TOLERANCE_MM = 3.0     # how close counts as "arrived" when sending coords
MOVE_TIMEOUT_S = 10.0        # give up waiting for a move after this long

ANGLE_COLS = [f"j{i}" for i in range(1, 7)]
COORD_COLS = ["x", "y", "z", "rx", "ry", "rz"]


# ---------------------------------------------------------------------------
# Reading the robot state
# ---------------------------------------------------------------------------
def _valid(values):
    """pymycobot sometimes returns -1, None or an empty list when a read fails."""
    return isinstance(values, list) and len(values) == 6


def read_pose(mc, retries=15):
    """Return (angles, coords) for the current pose, retrying failed reads."""
    for _ in range(retries):
        angles = mc.get_angles()
        coords = mc.get_coords()
        if _valid(angles) and _valid(coords):
            return angles, coords
        time.sleep(0.1)
    raise RuntimeError("Could not read angles/coords from the robot. "
                       "Check the serial connection and that the arm is powered on.")


def display_pose(mc, label="Current pose"):
    """Custom function required by the lab: output and display the current
    end-effector coordinates and the corresponding joint angles."""
    angles, coords = read_pose(mc)
    print(f"{label}")
    print("  Joint angles (deg): " + ", ".join(f"J{i+1}={a:7.2f}" for i, a in enumerate(angles)))
    print("  End effector     : " + ", ".join(f"{n}={c:7.2f}" for n, c in zip(COORD_COLS, coords)))
    return angles, coords


# ---------------------------------------------------------------------------
# Moving the robot
# ---------------------------------------------------------------------------
def lock_servos(mc):
    """Re-enable (lock) all servos after drag teaching."""
    mc.power_on()
    time.sleep(1.0)


def wait_until_reached(mc, target, use_coords=False):
    """Poll the robot until it is within tolerance of the target, or time out.
    Returns True if it arrived, False on timeout."""
    start = time.time()
    while time.time() - start < MOVE_TIMEOUT_S:
        time.sleep(0.2)
        if use_coords:
            current = mc.get_coords()
            if _valid(current) and max(abs(c - t) for c, t in zip(current[:3], target[:3])) < COORD_TOLERANCE_MM:
                return True
        else:
            current = mc.get_angles()
            if _valid(current) and max(abs(c - t) for c, t in zip(current, target)) < ANGLE_TOLERANCE_DEG:
                return True
    return False


def go_home(mc, speed):
    print("Returning to home (zero) position...")
    mc.send_angles(HOME_ANGLES, speed)
    wait_until_reached(mc, HOME_ANGLES)
    display_pose(mc, "Home pose")


# ---------------------------------------------------------------------------
# CSV helpers
# ---------------------------------------------------------------------------
def load_points(path):
    with open(path, newline="") as f:
        rows = list(csv.DictReader(f))
    if not rows:
        sys.exit(f"No points found in {path}. Run 'record' first.")
    points = []
    for r in rows:
        points.append({
            "point": int(r["point"]),
            "type": r["type"],
            "angles": [float(r[c]) for c in ANGLE_COLS],
            "coords": [float(r[c]) for c in COORD_COLS],
        })
    return points


# ---------------------------------------------------------------------------
# Step 7-10: record points by drag teaching
# ---------------------------------------------------------------------------
def record(mc, path):
    mc.release_all_servos()
    time.sleep(0.5)
    print("Servos released - you can now move the arm by hand.\n"
          "Hold the arm steady at each point before pressing Enter.\n"
          "  Enter      -> record a MARK point (pen touching the paper)\n"
          "  h + Enter  -> record a HOVER point (pen just off the paper, used to\n"
          "                approach/leave a mark without dragging the pen)\n"
          "  q + Enter  -> finish and save\n")

    rows = []
    mark_count = 0
    while True:
        cmd = input(f"[{mark_count} marks recorded] Position the arm, then press Enter / h / q: ").strip().lower()
        if cmd == "q":
            break
        point_type = "hover" if cmd == "h" else "mark"
        angles, coords = display_pose(mc, f"Recorded {point_type} point #{len(rows) + 1}")
        rows.append([len(rows) + 1, point_type, *angles, *coords])
        if point_type == "mark":
            mark_count += 1
            print("  -> Now press the pen down by hand to make a reference dot and label it.\n")

    if mark_count < 10:
        print(f"Warning: only {mark_count} mark points recorded; the lab requires at least 10.")

    with open(path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["point", "type", *ANGLE_COLS, *COORD_COLS])
        writer.writerows(rows)
    print(f"Saved {len(rows)} points ({mark_count} marks) to {path}")


# ---------------------------------------------------------------------------
# Step 11 / 12: playback using angles or coordinates
# ---------------------------------------------------------------------------
def playback(mc, path, use_coords, loops, speed, dwell, linear):
    points = load_points(path)
    method = "coords" if use_coords else "angles"
    log_path = f"playback_{method}.csv"

    lock_servos(mc)
    print(f"Playing back {len(points)} points using {method}, {loops} loop(s), speed {speed}.")

    with open(log_path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["loop", "point", "type", "arrived",
                         *[f"target_{c}" for c in ANGLE_COLS + COORD_COLS],
                         *[f"actual_{c}" for c in ANGLE_COLS + COORD_COLS]])

        for loop in range(1, loops + 1):
            print(f"\n=== Loop {loop}/{loops} ===")
            for p in points:
                if use_coords:
                    mode = 1 if linear else 0          # 0 = angular, 1 = linear interpolation
                    mc.send_coords(p["coords"], speed, mode)
                    arrived = wait_until_reached(mc, p["coords"], use_coords=True)
                else:
                    mc.send_angles(p["angles"], speed)
                    arrived = wait_until_reached(mc, p["angles"])

                # Pause at marks so the pen leaves a clear dot; short pause at hovers.
                time.sleep(dwell if p["type"] == "mark" else 0.3)

                actual_angles, actual_coords = read_pose(mc)
                status = "ok" if arrived else "TIMEOUT (target may be unreachable)"
                print(f"  point {p['point']:>2} ({p['type']:5}) {status}")
                writer.writerow([loop, p["point"], p["type"], arrived,
                                 *p["angles"], *p["coords"], *actual_angles, *actual_coords])

    print(f"\nLogged reached poses to {log_path}")
    go_home(mc, speed)


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def main():
    parser = argparse.ArgumentParser(description="MechArm 270 Pi drag teaching (Lab 2, Part III)")
    parser.add_argument("command", choices=["record", "play-angles", "play-coords", "home"])
    parser.add_argument("--file", default="points.csv", help="CSV of recorded points")
    parser.add_argument("--loops", type=int, default=3, help="times to loop through all points")
    parser.add_argument("--speed", type=int, default=30, help="move speed, 1-100")
    parser.add_argument("--dwell", type=float, default=1.5, help="seconds to pause at each mark")
    parser.add_argument("--linear", action="store_true", help="play-coords: use linear interpolation")
    args = parser.parse_args()

    mc = Arm(PI_PORT, PI_BAUD)
    time.sleep(0.5)

    try:
        if args.command == "record":
            record(mc, args.file)
        elif args.command == "play-angles":
            playback(mc, args.file, False, args.loops, args.speed, args.dwell, args.linear)
        elif args.command == "play-coords":
            playback(mc, args.file, True, args.loops, args.speed, args.dwell, args.linear)
        elif args.command == "home":
            lock_servos(mc)
            go_home(mc, args.speed)
    except KeyboardInterrupt:
        print("\nStopped by user. Run 'home' to return the arm to zero.")


if __name__ == "__main__":
    main()