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

# ---- Connect to the robot (step 5) ----
mc = MechArm270(PI_PORT, PI_BAUD)
time.sleep(1)


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


def record_points():
    """Steps 6-10: unlock the arm, move it by hand, and save each point to a CSV file."""
    mc.release_all_servos()
    print("Servos released - you can move the arm by hand.")

    points = []
    for i in range(NUM_POINTS):
        input(f"Hold the pen on point {i + 1}, then press Enter...")
        angles, coords = get_position()
        points.append(angles + coords)          # one row: 6 angles followed by 6 coords

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
            mc.send_angles(p[:6], SPEED)         # first 6 values = joint angles
            time.sleep(MOVE_TIME)
            mc.send_angles(HOME, SPEED)          # lift away so the pen doesn't drag to the next point
            time.sleep(MOVE_TIME)


def play_coords(points):
    """Step 12: same as above, but send the recorded world coordinates instead."""
    for loop in range(LOOPS):
        print(f"Coordinate playback - loop {loop + 1} of {LOOPS}")
        for i, p in enumerate(points):
            print(f"  Point {i + 1}")
            mc.send_coords(p[6:], SPEED, 0)      # last 6 values = x, y, z, rx, ry, rz
            time.sleep(MOVE_TIME)
            mc.send_angles(HOME, SPEED)
            time.sleep(MOVE_TIME)


# ---- Main program ----
if input("Record new points? (y/n): ").strip().lower() == "y":
    points = record_points()
else:
    points = load_points()

mc.power_on()                                   # lock the servos again so the robot can move itself
time.sleep(1)

input("Press Enter to start playback using ANGLES...")
play_angles(points)

input("Put up fresh paper, then press Enter to start playback using COORDINATES...")
play_coords(points)

# Step 15: return to neutral (zero) position
mc.send_angles(HOME, SPEED)
time.sleep(MOVE_TIME)
print("Done - arm is back at the zero position.")