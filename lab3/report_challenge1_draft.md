# Lab 3 Report — Challenge 1 Draft (Group 9)

> Draft sections for the Challenge 1 part of the report. Paste them into the course report template.
> Items in [square brackets] are for you to fill in. All numbers come from `part1_validation.csv`,
> produced by `python3 lab3/validate_ik.py`.

---

## Challenge 1: Inverse Kinematics Algorithms

### 1.1 Objective

The goal of Challenge 1 was to write two numerical inverse kinematics (IK) solvers for the MechArm 270 Pi.
We validated both against the ten poses recorded in Lab 2 Challenge 2. Given a target end-effector pose
[x, y, z, roll, pitch, yaw], each solver returns the six joint angles that should place the end effector
at that pose. We checked the results in two ways:

- **Joint angles:** each solved angle against the encoder angle the robot reported for that pose in Lab 2.
- **Pose:** the solved angles passed back through forward kinematics (FK), compared with the target pose.

### 1.2 Kinematic Model

Both solvers share one forward kinematics model, built symbolically in SymPy from standard
Denavit–Hartenberg (DH) parameters. In this convention each joint's transform is
A_i = Rz(θ_i)·Tz(d_i)·Tx(a_i)·Rx(α_i).

**Table 1. DH parameters used in Lab 3 (lengths in mm).**

| Joint | a (mm) | α (rad) | d (mm) | θ |
|---|---|---|---|---|
| 1 | 0 | −π/2 | 136.5 | q1 |
| 2 | 100 | 0 | 0 | q2 − π/2 |
| 3 | **10** | −π/2 | 0 | q3 |
| 4 | 0 | π/2 | 107.5 | q4 |
| 5 | 0 | −π/2 | 0 | q5 |
| 6 | 0 | 0 | 65.5 | q6 |

The model has three adjustments:

- **Joint 2 offset (−π/2).** The encoder's zero for J2 differs from the DH zero by 90°, so the J2 row uses
  q2 − π/2.
- **Output frame.** For all ten Lab 2 samples, the robot's reported orientation differed from our DH flange
  frame by a 180° rotation about the tool z-axis. We therefore multiply the FK result by a fixed Rz(π), so
  that FK outputs poses in the same frame the robot reports.
- **Elbow offset a3 = 10 mm.** This changes our Lab 2 model. With the Lab 2 parameters (a3 = 0), FK
  applied to the recorded joint angles reproduced the recorded orientations exactly but missed the
  positions by 5.5 mm RMS (up to about 10 mm, mostly in z).
  - We fitted all five link lengths to the ten recorded poses by least squares. The fit returned a3 = 9.97 mm,
    with the other lengths within 0.5 mm of nominal.
  - Adding a3 = 10 mm reduced the FK position error to 0.5 mm RMS.
  - Without this offset, the advanced IK solver could match every pose only by bending J3 and J5 by a
    constant ≈5° from the recorded values, because atan(10/107.5) ≈ 5.3°.

**Units and angles.** Joint angles are in radians internally and in degrees in the tables. Positions are in
mm. Orientation is roll–pitch–yaw (rx, ry, rz) with R = Rz(yaw)·Ry(pitch)·Rx(roll), the convention the
robot uses for `get_coords()`. The symbolic FK is converted with `lambdify` into a fast numerical function
that the solvers call.

### 1.3 Basic IK Algorithm (`mecharm_INVK_group_09.py`)

Following the handout, the basic solver splits the problem into two independent three-joint subproblems.
Both use SciPy's `least_squares` with the Levenberg–Marquardt method and an initial guess of 0.1 rad for
every joint.

1. **Position subproblem:** solve for q1–q3 with q4 = q5 = q6 = 0. The residual is the FK position minus
   the target [x, y, z].
2. **Orientation subproblem:** solve for q4–q6 with q1 = q2 = q3 = 0. The residual is the FK
   roll/pitch/yaw minus the target orientation, wrapped to [−π, π).
3. **Combine:** join the two halves into one joint vector, [q1, q2, q3, q4, q5, q6].

### 1.4 Advanced (Pose-Error Least-Squares) IK Algorithm (`mecharm_INVK_advanced_group_09.py`)

The advanced solver solves for all six joints at once:

1. `transf_to_pose` converts the 4×4 FK matrix into [X, Y, Z, roll, pitch, yaw]. It uses the same RPY
   convention as the basic solver, and handles gimbal lock (pitch = ±90°) explicitly.
2. `target_pose_error` returns the six-element difference between the current pose and the target pose:
   - the position error is in mm;
   - the three angle errors are wrapped to [−π, π) and scaled by 65.5 mm/rad (the tool length), so that
     a rotation error and a position error of comparable physical size get comparable weight.
3. `ik` minimizes this error with `least_squares` using the Trust Region Reflective (`trf`) method. The
   Lab 3 Annex 1 joint limits are passed as bounds, so every solution the robot is asked to reach is
   physically reachable.

The solver starts from 0.1 rad for every joint, not 0. At all-zero joint angles the tool's pitch is exactly
90°. That is gimbal lock: roll and yaw are no longer uniquely defined, and the RPY error gives the optimizer
no useful direction. In our tests, starting exactly at zero stalled 3 of the 10 targets. Starting at 0.1 rad
solved all 10.

### 1.5 Validation Results

Every target pose came from Lab 2 Challenge 2. Table 2 shows the reference joint angles the robot reported
for those poses.

**Table 2. Recorded Lab 2 joint angles (degrees).**

| Sample | J1 | J2 | J3 | J4 | J5 | J6 |
|---|---|---|---|---|---|---|
| 1 | 25.8 | 24.5 | 9.2 | 5.4 | −40.5 | 87.8 |
| 2 | 7.1 | 8.7 | 9.5 | 3.2 | −19.1 | 86.4 |
| 3 | −8.6 | 16.3 | −1.2 | 2.0 | −17.1 | 86.5 |
| 4 | 31.2 | 29.7 | 51.8 | 7.2 | −28.5 | 88.8 |
| 5 | 4.7 | 41.7 | 37.4 | 7.5 | −29.6 | 82.8 |
| 6 | −20.0 | 32.2 | 38.0 | 15.4 | −12.2 | 71.5 |
| 7 | −20.1 | 24.4 | 17.8 | 17.6 | −39.2 | 1.2 |
| 8 | 5.5 | 66.9 | −59.9 | −98.1 | 96.0 | 174.8 |
| 9 | −15.4 | 51.1 | −60.0 | −25.7 | 79.6 | −27.5 |
| 10 | 2.0 | 41.2 | 5.8 | −0.4 | 41.7 | 100.7 |

#### Basic Algorithm

**Table 3. Basic IK: solved joint angles (degrees) and resulting pose error.**
The pose error is found by passing all six solved angles through FK and comparing with the target.

| Sample | J1 | J2 | J3 | J4 | J5 | J6 | Position error (mm) | Rotation error (°) |
|---|---|---|---|---|---|---|---|---|
| 1 | 24.7 | 19.7 | −0.5 | −72.9 | −23.2 | 166.6 | 26.34 | 31.16 |
| 2 | 6.7 | 7.6 | 3.6 | 98.1 | 6.1 | −8.6 | 6.98 | 13.05 |
| 3 | −8.8 | 15.4 | −6.4 | −101.9 | 9.4 | −169.8 | 10.74 | 12.56 |
| 4 | 29.3 | 27.0 | 42.9 | 17.9 | 57.1 | 59.8 | 62.64 | 75.09 |
| 5 | 2.9 | 39.1 | 28.3 | −0.9 | 49.5 | 86.4 | 54.81 | 67.52 |
| 6 | −21.6 | 31.7 | 34.0 | −15.2 | 61.8 | 111.4 | 67.32 | 68.79 |
| 7 | −23.8 | 19.9 | 8.2 | 98.2 | −31.4 | −82.8 | 35.47 | 36.73 |
| 8 | −12.7 | 21.9 | −2.8 | 81.3 | −89.5 | −11.2 | 92.21 | 22.96 |
| 9 | −23.2 | 24.1 | −1.4 | −30.3 | 79.0 | −9.5 | 83.29 | 32.33 |
| 10 | 1.9 | 32.7 | 28.0 | −0.2 | 88.8 | 98.4 | 91.62 | 60.71 |
| **RMSE** | | | | | | | **61.16** | **47.79** |

**Discussion.**

- **Each subproblem converged, but the combined answer is wrong.** Both optimizations reported success on
  all 10 targets. Even so, the combined solution misses the target by 61 mm and 48° RMS (as much as 92 mm
  and 75°). Each subproblem solves its own simplified version of the task; neither solves the real one.
- **J1 is the closest match.** Mean |Δ| was 2.1° for J1 (excluding sample 8), against 5.8° for J2, 14.8° for
  J3 and 40–50° for J4–J6. J1 mostly sets the arm's heading, which the wrist barely changes.
- **The wrist joints are effectively wrong.** Their mean |Δ| is 40–50° because they were solved as if the
  base, shoulder and elbow were all at 0°.
- **Error depends on how far the arm is from home.** The error was smallest (7–11 mm) for samples 2 and 3,
  where the elbow and wrist are close to their zero positions. It was largest (83–92 mm) for samples 8–10,
  where J2 or the wrist is strongly bent (for example J5 = 96° and 80° in samples 8 and 9).

#### Advanced Algorithm

**Table 4. Advanced IK (initial guess 0.1 rad): solved joint angles (degrees) and resulting pose error.**

| Sample | J1 | J2 | J3 | J4 | J5 | J6 | Position error (mm) | Rotation error (°) |
|---|---|---|---|---|---|---|---|---|
| 1 | 25.8 | 24.1 | 10.0 | 5.4 | −40.9 | 87.8 | 0.00 | 0.00 |
| 2 | 7.1 | 8.2 | 10.3 | 3.2 | −19.4 | 86.4 | 0.00 | 0.00 |
| 3 | −8.6 | 15.8 | −0.3 | 2.1 | −17.5 | 86.4 | 0.00 | 0.00 |
| 4 | 31.2 | 29.4 | 52.1 | 7.3 | −28.4 | 88.8 | 0.00 | 0.00 |
| 5 | 4.7 | 41.4 | 37.7 | 7.5 | −29.7 | 82.8 | 0.00 | 0.00 |
| 6 | −20.0 | 31.8 | 38.4 | 15.5 | −12.2 | 71.5 | 0.00 | 0.00 |
| 7 | −20.1 | 24.1 | 18.4 | 17.6 | −39.5 | 1.3 | 0.00 | 0.00 |
| 8 | 5.7 | 66.2 | −58.4 | **82.0** | **−96.2** | **−4.3** | 0.00 | 0.00 |
| 9 | −15.3 | 50.0 | −58.1 | −25.8 | 78.9 | −27.0 | 0.00 | 0.00 |
| 10 | 2.0 | 40.9 | 6.1 | −0.4 | 41.7 | 100.7 | 0.00 | 0.00 |
| **RMSE** | | | | | | | **0.000** | **0.000** |

**Discussion.**

- **Pose accuracy.** The advanced solver converged on all 10 targets in 8–14 function evaluations, with
  pose error below the reporting precision (< 0.001 mm and < 0.001°). Every solution is within the
  Annex 1 joint limits.
- **Agreement with the recorded joint angles.** For 9 of the 10 samples, every solved joint is within 1.9° of
  the recorded encoder angle. The mean |Δ| is 0.0–0.7° per joint.
- **Where the remaining 0.3–1.9° comes from.** It is mostly in J2 and J3. Our model uses nominal link
  lengths; the fit in Section 1.2 suggests the real values differ by up to 0.5 mm. In addition, the robot
  reports poses rounded to 0.1 mm and 0.01°.
- **Sample 8 is a second valid solution.** The solved wrist is (82.0, −96.2, −4.3)°, against the recorded
  (−98.1, 96.0, 174.8)°. This is the standard "wrist flip": rotating J4 and J6 by 180° and reversing the
  sign of J5 produces the same end-effector pose. The pose error is still zero, so the solver did not fail;
  it found a different valid solution. On the real arm, however, moving between the two would swing the
  wrist through about 180°. To get the solution closest to the current posture, start the solver from the
  robot's current joint angles, which is what Challenge 2 does.

#### Comparison of the Two Algorithms

**Table 5. Summary over the 10 Lab 2 targets.**

| Solver | Position RMSE (mm) | Rotation RMSE (°) | Mean abs. joint difference (°), J1–J6 (excl. sample 8) | Converged | Within joint limits |
|---|---|---|---|---|---|
| Basic (split) | 61.16 | 47.79 | 2.1, 5.8, 14.8, 45.8, 40.3, 50.5 | 10/10 | 10/10 |
| Advanced (initial guess 0.1 rad) | 0.000 | 0.000 | 0.0, 0.5, 0.7, 0.0, 0.3, 0.1 | 10/10 | 10/10 |
| Advanced (initial guess = basic answer) | 0.52 | 1.30 | — | 10/10 | 10/10 |

- **Overall.** The advanced solver is more accurate by more than four orders of magnitude in pose error.
  It also needs about 5× fewer function evaluations: 8–14, against 46–72 in total for the two basic
  subproblems.
- **Starting the advanced solver from the basic answer.** We also tried using the basic solver's output as
  the initial guess for the advanced solver. This worked for 8 of the 10 targets.
  - For samples 2 and 3 it stopped at 1.4 mm / 3.4° and 0.9 mm / 2.3°, with J4 pushed against its ±165°
    limit.
  - The basic answers for those samples had J4 ≈ ±100°, on the far side of the wrist's range. From there,
    the bounded optimizer runs into the limit before it can reach the true solution near J4 ≈ 2–3°.
  - This shows that the solver is local: it finds the nearest minimum, not necessarily the best one. A
    good initial guess matters as much as the solver itself.

**Figure [N].** [Screenshot of the summary printed by `python3 lab3/validate_ik.py`.]

### 1.6 Reflection Questions

**Q1. Why might the basic inverse kinematics fail to achieve exact joint angles?**

The basic solver optimizes two subproblems in which half the joints are frozen at 0. Each subproblem
converges, but it converges to a solution of the wrong problem:

- J1–J3 are chosen to put the tool tip on the target *assuming the wrist is straight*.
- J4–J6 are chosen to produce the target orientation *assuming the arm is at home*.

When the two halves are combined, the actual wrist angles move the tool tip, and the actual arm angles
rotate the wrist's frame. So the combined pose misses the target, and the joint angles differ from the
recorded ones, even though each optimizer reports success. The error grows the further the true posture is
from those frozen assumptions: 7 mm for sample 2, close to home, against 92 mm for sample 8, which is
strongly bent.

**Q2. What assumptions in the basic algorithm may cause this failure?**

The key assumption is that **position and orientation are decoupled**: position depends only on q1–q3, and
orientation depends only on q4–q6. Neither is true for this arm.

- **Position depends on the wrist.** The tool tip is 65.5 mm beyond the wrist center along the tool axis.
  J4–J6 swing it around the wrist center, moving the position by up to about 130 mm.
- **Orientation depends on the arm.** The end-effector rotation is R₀₆ = R₀₃(q1, q2, q3) · R₃₆(q4, q5, q6).
  Setting q1–q3 = 0 replaces R₀₃ with the identity, so the wrist solution is expressed in the wrong frame.

This split could be made correct with the standard "kinematic decoupling" method used for arms whose last
three axes meet at one point:

1. Solve q1–q3 for the *wrist-center* position (the target minus 65.5 mm along the target tool axis),
   instead of the tool tip.
2. Solve q4–q6 for R₃₆ = R₀₃ᵀ · R_target, using the q1–q3 just found.

Our basic solver does neither. Two further, smaller factors:

- The Levenberg–Marquardt method is unbounded, so the basic solver can return angles outside the joint
  limits.
- Each subproblem uses a single fixed initial guess, so it can land on a different (but valid) solution.

**Q3. What improvements does the advanced algorithm have over the basic algorithm?**

- It optimizes **all six joints together** against the full 6D pose error, so it accounts for how
  position and orientation depend on each other. Pose error drops from 61 mm / 48° to below 0.001 mm and
  0.001°.
- It **enforces the joint limits** (`trf` with bounds), so every solution can be sent to the robot.
- It **weights orientation error against position error** (65.5 mm/rad), so neither part dominates.
- It **wraps angle errors**, so that, for example, +179° and −179° count as 2° apart, not 358°.
- It converges faster: 8–14 function evaluations, against 46–72.

Its remaining limitations:

- It is a local optimizer: the solution depends on the initial guess (samples 2 and 3 from the basic
  answer, and the sample 8 wrist flip).
- Its RPY error stops working at pitch = ±90°, the gimbal-lock case we saw when starting from all zeros.

**Q4. Where is the Jacobian matrix implemented in the code for both algorithms?**

Neither script builds the Jacobian explicitly. It is computed inside SciPy's `least_squares`. Because we
pass no `jac` argument, `least_squares` estimates the Jacobian of the residual vector at every iteration by
finite differences: it nudges each joint angle slightly and measures how each error component changes. It
then uses that Jacobian to choose the next step: a Levenberg–Marquardt step in the basic solver, and a
trust-region step in the advanced solver.

- **Basic solver:** two separate **3×3** Jacobians.
  - The position Jacobian, ∂[x, y, z]/∂[q1, q2, q3], evaluated with q4–q6 = 0.
  - The orientation Jacobian, ∂[roll, pitch, yaw]/∂[q4, q5, q6], evaluated with q1–q3 = 0.
  - The off-diagonal blocks, ∂position/∂(q4–q6) and ∂orientation/∂(q1–q3), are never computed. Leaving
    them out is exactly what produces the decoupling error described in Q1 and Q2.
- **Advanced solver:** one full **6×6** Jacobian, ∂(pose error)/∂(q1–q6). It includes the coupling terms,
  which is why the advanced solver converges to the exact pose.

The Jacobian also explains the gimbal-lock behaviour. At pitch = ±90°, the roll and yaw rows of the
Jacobian depend on each other, so the Jacobian loses rank and the optimizer has no well-defined direction
to step in. We avoided this by never starting the solver at exactly zero.

---

### AI-Use Footnote (required by the handout)

[Generative AI tools were used in this lab. Describe your actual use, for example: "ChatGPT and Claude
(Anthropic) helped implement and debug the IK scripts, diagnose the missing a3 elbow offset by fitting the
DH link lengths to our Lab 2 data, and draft parts of this section. All results were generated by running
our code, and every group member reviewed and can explain the code and the analysis."]
