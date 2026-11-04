"""
EE 347 Lab 2 - Part III: Drag Teaching
Run on the robot's Raspberry Pi:   python3 mecharm_control_group_xx.py
"""

import csv
import time
from pymycobot import MechArm270, PI_PORT, PI_BAUD

# ---- Settings ----
NUM_POINTS = 10        # how many points to record (lab requires at least 10)
LOOPS = 3              # how many times to repeat the playback (lab requires at least 3)
SPEED = 30             # robot speed, 1-100
MOVE_TIME = 4          # seconds to wait for each move to finish
HOME = [0, 0, 0, 0, 0, 0]
CSV_FILE = "data.csv"
GRIPPER_TYPE = 1       # 1 = adaptive gripper (try 3 = parallel gripper if it doesn't respond)

# ---- Limits the robot accepts for send_angles / send_coords ----
# Read them from pymycobot itself so they match what playback will accept.
try:
    from pymycobot.robot_info import RobotLimit
    LIMITS = RobotLimit.robot_limit["MechArm270"]
except Exception:
    LIMITS = {                                  # same values, copied from pymycobot 4.0.7
        "angles_min": [-160, -75, -175, -155, -115, -180],
        "angles_max": [160, 120, 65, 155, 115, 180],
        "coords_min": [-272, -272, -86, -180, -180, -180],
        "coords_max": [272, 272, 408.9, 180, 180, 180],
    }

# ---- Connect to the robot (step 5) ----
mc = MechArm270(PI_PORT, PI_BAUD)
time.sleep(1)


def close_gripper():
    """Close the claw on the marker. Arguments: 1 = close, speed 50, gripper type."""
    mc.set_gripper_state(1, 50, GRIPPER_TYPE)
    time.sleep(2)                               # give the claw time to close


def get_position():
    """Task (step 7): print the current end-effector coordinates and joint angles."""
    while True:
        angles = mc.get_angles()      # [J1, J2, J3, J4, J5, J6] in degrees
        coords = mc.get_coords()      # [x, y, z, rx, ry, rz] in mm and degrees
        # A read occasionally fails and returns -1 or []; if so, just try again
        if isinstance(angles, list) and len(angles) == 6 and isinstance(coords, list) and len(coords) == 6:
            break
        time.sleep(0.1)
    print("  Joint angles:", angles)
    print("  Coordinates: ", coords)
    return angles, coords


def check_point(angles, coords):
    """Return a list of problems with this point. An empty list means the robot can play it back."""
    problems = []
    for i in range(6):
        low, high = LIMITS["angles_min"][i], LIMITS["angles_max"][i]
        if not low <= angles[i] <= high:
            problems.append(f"J{i + 1} is {angles[i]} deg (allowed {low} to {high})")
    names = ["x", "y", "z", "rx", "ry", "rz"]
    for i in range(6):
        low, high = LIMITS["coords_min"][i], LIMITS["coords_max"][i]
        if not low <= coords[i] <= high:
            problems.append(f"{names[i]} is {coords[i]} (allowed {low} to {high})")
    return problems


def record_points():
    """Steps 6-10: unlock the arm, move it by hand, and save each point to a CSV file."""
    mc.release_all_servos()
    close_gripper()                             # re-close in case releasing the servos loosened the claw
    print("Servos released - you can move the arm by hand.")

    points = []
    while len(points) < NUM_POINTS:
        input(f"Hold the pen on point {len(points) + 1}, then press Enter...")
        angles, coords = get_position()
        problems = check_point(angles, coords)
        if problems:                            # outside the limits -> don't save, ask again
            print("  NOT SAVED - the robot can't play this point back:")
            for p in problems:
                print("   -", p)
            print("  Move the arm to a less extreme position and try again.\n")
            continue
        points.append(angles + coords)          # one row: 6 angles followed by 6 coords
        print(f"  Point {len(points)} saved.\n")

    with open(CSV_FILE, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["j1", "j2", "j3", "j4", "j5", "j6", "x", "y", "z", "rx", "ry", "rz"])
        writer.writerows(points)
    print(f"Saved {len(points)} points to {CSV_FILE}")
    return points


def load_points():
    """Read previously recorded points back from the CSV file."""
    with open(CSV_FILE) as f:
        reader = csv.reader(f)
        next(reader)                            # skip the header row
        return [[float(value) for value in row] for row in reader]


def play_angles(points):
    """Step 11: send the recorded joint angles, looping through all points LOOPS times."""
    for loop in range(LOOPS):
        print(f"Angle playback - loop {loop + 1} of {LOOPS}")
        for i, p in enumerate(points):
            print(f"  Point {i + 1}")
            try:
                mc.send_angles(p[:6], SPEED)     # first 6 values = joint angles
            except Exception as error:           # the library rejects values outside the joint limits
                print(f"  Skipped point {i + 1}: {error}")
                continue
            time.sleep(MOVE_TIME)
            mc.send_angles(HOME, SPEED)          # lift away so the pen doesn't drag to the next point
            time.sleep(MOVE_TIME)


def play_coords(points):
    """Step 12: same as above, but send the recorded world coordinates instead."""
    for loop in range(LOOPS):
        print(f"Coordinate playback - loop {loop + 1} of {LOOPS}")
        for i, p in enumerate(points):
            print(f"  Point {i + 1}")
            try:
                mc.send_coords(p[6:], SPEED, 0)  # last 6 values = x, y, z, rx, ry, rz
            except Exception as error:           # the library rejects coordinates outside its limits
                print(f"  Skipped point {i + 1}: {error}")
                continue
            time.sleep(MOVE_TIME)
            mc.send_angles(HOME, SPEED)
            time.sleep(MOVE_TIME)


# ---- Main program ----
input("Place the marker between the claw's jaws, then press Enter to close the claw...")
close_gripper()

if input("Record new points? (y/n): ").strip().lower() == "y":
    points = record_points()
else:
    points = load_points()

mc.power_on()                                   # lock the servos again so the robot can move itself
time.sleep(1)
close_gripper()                                 # make sure the claw is still closed before playback

input("Press Enter to start playback using ANGLES...")
play_angles(points)

input("Put up fresh paper, then press Enter to start playback using COORDINATES...")
play_coords(points)

# Step 15: return to neutral (zero) position
mc.send_angles(HOME, SPEED)
time.sleep(MOVE_TIME)
print("Done - arm is back at the zero position.")