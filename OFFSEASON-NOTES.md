# Branch `2910-lessons` — what happened and what's left

**Written:** 2026-09-23
**Branched from:** `sotm-testing` @ `5cafb98` ("Champs edits") — the code the robot actually ran at Champs
**Size:** 18 commits, 10 files, +796 / −226 lines
**Pushed to:** `origin/2910-lessons`

> **None of this has ever run on a robot.** Every change compiles and every step was built
> before the next one started, but nothing here has been deployed, driven, or shot with.
> Treat the whole branch as untested until it has been on the practice field.

---

## 1. What this branch is

An offseason pass over the shooter/aiming code, taking patterns from four public codebases
and fixing a handful of real bugs found along the way. The robot is a WCP-CC: fixed
forward-firing 3-motor flywheel, PWM servo hood, 2-motor feeder, pivoting intake, no climber.
"Fixed shooter" is the important part — the drivetrain's heading *is* the aim axis, so every
borrowed aiming pattern had to come from another fixed-shooter robot to transfer cleanly.

The headline feature is a **shot-readiness gate**: the feeder now only runs when the robot is
actually in a position to make the shot, and a row of `Ready/*` lights on the dashboard says
which condition is blocking when it isn't.

## 2. Where the code came from

| Source | What it is | What we took |
|---|---|---|
| **2910 Jack in the Bot** — `FRCTeam2910/2026CompetitionRobot-Public` | Fixed shooter, top-tier | Readiness gate concept, `FieldCentricFacingAngle` aiming, operator RPM slider, known-spot odometry reset, tunable numbers |
| **1678 Citrus Circuits** — `frc1678/C2026-Public` | Fixed shooter, top-tier | Reject-lookahead guard, tilt gate, shared predicted-pose helper |
| **WCP-CC official** — `wcpllc/2026CompetitiveConcept` | Our exact hardware | Brownout threshold; confirmed our red-alliance aim conversion is correct |
| **6328 Mechanical Advantage ("Darwin")** — `Mechanical-Advantage/RobotCode2026Public` | Fixed shooter, top-tier | Drag-decayed lead, iterated lookahead, max-distance bound, measured flight-time data |

Checked and rejected as sources: **604** (turreted — aim code doesn't transfer), **254** (no
robot code release since 2018), **1690 / 3476 / 4414** (no public 2026 robot code), **971**
(C++/Bazel on their own infra, not transferable for one programmer).

## 3. What changed, step by step

| Step | What / why |
|---|---|
| 1 | **Bug fixes.** `aimAndPass` now requires the drivetrain (it was calling `setControl` every loop while the default drive command fought it). At-speed now requires motors 8, 9 *and* 10 — each runs its own PID, so one could lag while motor 8 read green. Removed the hood EMA (alpha was 1.0, a no-op). Regenerated the `RobotContainer` header from the actual code. |
| 2 | `Shot` became a `record`. |
| 4 | **Aiming rewritten.** Replaced the hand-rolled `tx * kAimP` loop with one shared `SwerveRequest.FieldCentricFacingAngle`. Phoenix runs the heading PID at 250 Hz instead of our 50 Hz. Added `facingFieldAngle()` to convert a blue-origin field angle into the operator-perspective frame Phoenix expects — without this, aiming is 180° wrong on red. |
| 3 | **Shot-readiness gate.** `isReadyToShoot()` + `gatedShoot()` on operator button 1. Old ungated `Shoot()` moved to button 11 as the override. `Ready/*` dashboard lights. `"Ignore Shot Gates"` toggle. |
| 5 | Flight-time lookup table replaced the constant `BALL_VELOCITY_MS = 8.0`. |
| 6 | Operator RPM % slider (Shuffleboard "Operator" tab). Applies to table shots only — not fixed shots, not pass shots. |
| 7 | `Landmarks.KnownSpot` (LEFT/RIGHT_TRENCH) + `shootFromKnownSpot()` on the hat. Resets odometry X/Y to a known field spot when vision can't see. |
| 8 | `TunableNumber` (our own, no AdvantageKit) for `kAimP` and the shooter velocity tolerance, gated behind `Constants.kTuningMode` (default false). |
| 9 | 0.05 s rising debouncers on the readiness gate and the at-speed check. |
| — | `aimAndWindUp` now looks up RPM/hood at the distance to the *virtual* target, not the real hub. Identical standing still. |
| — | 17 `TODO(tune)` notes added at every number needing on-robot tuning. |
| 10 | Shared `predictedTranslation()` helper feeds both the aim angle and the table distance. Drops the moving-shot lead when spinning fast and far off target, or when basically stopped. |
| 11 | **Reverted** — see §5. |
| 12 | **Tilt gate.** `Ready/Level` blocks shots when Pigeon pitch/roll exceed 5° (e.g. on a bump ramp, where the hood angle relative to the field is wrong). |
| 13 | roboRIO brownout threshold lowered to 6.1 V. |
| 14 | **Three corrections to the above** — see §4. |

## 4. The Step 14 corrections (important)

6328's code exposed three mistakes in work done earlier in the same session:

1. **Flight-time placeholders were ~40% too short.** The original guesses (0.6 s at 1.5 m)
   were far off two independent *measured* datasets: 1678 measured ~1.05 s at 2.0 m, 6328
   measured 1.017 s at 1.63 m rising to 1.25 s at 4.875 m. Reseeded to 1.00 / 1.12 / 1.22 s.
   **Still placeholders — our shooter is different. Measure ours.**
2. **The moving-shot lead over-aimed by ~20%.** A ball does not carry the robot's full
   velocity for its whole flight; drag bleeds it off. Added `kLeadDragPerSec` (0.375, 6328's
   value; set to 0 to disable) and the lookup now iterates 3× to convergence instead of once.
3. **The readiness gate had no maximum distance.** Past the table's last point (140 in) the
   interpolator silently clamps to the last entry, so every light read green while the RPM and
   hood were a guess. Added `Ready/InRange`.

## 5. Tried and reverted / deliberately not adopted

- **1678's pre-shot feeder back-off** (commit `012ed2c`, reverted in `7185246`). Reverses a
  roller during spin-up so a ball resting on the flywheel doesn't drag RPM. **We have no
  secondary roller between the feeder and the flywheel**, so reversing the fuel feed would
  just push fuel back down the feeder. Don't re-add this without that hardware.
- **WCP's `PeakReverseVoltage = 0`** on the shooter motors — declined.
- **AdvantageKit, Choreo, REVLib, grapplefrc** — no new vendordeps, by rule. Patterns only.
- **Vision architecture** — frozen. MegaTag2 + std-dev fusion in `LimelightSubsys` untouched.
- **Distance table values, PathPlanner NamedCommand names, CAN IDs, `TunerConstants`** — untouched.
- 1678's CANrange hopper tracking, motorized hood homing, 1085-line `Superstructure`;
  6328's `energy/` battery estimator and `salesman/` fuel-pickup planner; all sim/LED/
  visualization code. Out of scope or needs hardware we don't have.

## 6. Behaviors that will surprise you

Read this section before the first practice match.

- **Button 1 only feeds while an aim command is held.** `Ready/Heading` requires the
  facing-angle request to be the one actively driving (so a stale PID error can't read as
  "aimed"). Right bumper or Y satisfies it. Manual wind-ups (buttons 5/7/9), the known-spot
  hat shots, and `windUp75` do **not** — with those, button 1 will never fire. Use **button
  11** (ungated) or `"Ignore Shot Gates"` for any non-aimed shot.
- **The new max-distance gate may block shots that worked at Champs.** We have never measured
  the robot's actual maximum range. If you were shooting from past 140 in and the table was
  quietly clamping, that now reads `Ready/InRange` false. The gate is telling you something
  true, but "a guess that scores" is still a shot — override it and extend the table rather
  than giving up the range.
- **Effective shooting window is 59 in – 140 in.** `kMinimumShotDistanceMeters` (1.5 m = 59 in)
  is stricter than the table's closest point (47 in), so the two closest table entries are
  unreachable through the gate. Reconcile this when tuning; 1.2 m may be the right minimum.
- **Aim commands interrupt each other and do not resume.** Pressing Y while holding right
  bumper cancels the bumper's aim; releasing Y does not bring it back (`whileTrue` only
  schedules on the press). Re-press to resume.
- **Known-spot shots fight vision.** The hat resets odometry X/Y, but `updateVision()` keeps
  fusing MegaTag2 every loop and will pull the pose back. Use it with `"Vision Enabled"` off,
  or treat it strictly as a "vision sees nothing" fallback.
- **The RPM % slider applies in autonomous too.** Leave it at +20% after a match and the next
  auto shoots 20% hot. Put "zero the slider" on the pre-match checklist.

## 7. Open items — identified but NOT done

- **`"Shooter At Speed"` reads green while idling.** The at-speed check only requires
  `targetRPM > 0`, and idle sets 3000 RPM, so once idle settles the light is green with no
  wind-up commanded. It can't fire on its own (the heading gate blocks), but it's a misleading
  light and makes `Ready/ALL` one gate closer than it should be. Fix: track an `isIdling` flag
  in `ShooterSubsys` and return false while idling. **Small, worth doing.**
- **Target-rate feedforward on the heading controller.** 2910, 1678, and 6328 *all* feed the
  target's angular rate forward; we're P-only. This is the fix for heading lag while strafing
  around the hub. 3-for-3 across elite fixed shooters — strongest single remaining improvement.
- **Hub shift timing.** Both 6328 (`HubShiftUtil`) and 1678 (`ShiftUtil`) shift their firing
  windows by ball flight time so fuel lands inside an open window. We have zero match-time
  handling. Whether this matters is a strategy question for the drive team, not a code question.
- **Idle RPM is hot.** We idle at 3000 of 3350 (~90%); 6328 idles at ~45% of shot speed.
  Real battery cost across a match.
- **Shooter `kV` is 0.1; the physics value is 12/100 = 0.12** (WCP's number). Our `kI` of 0.5
  covers the gap. If RPM settle between balls feels slow, try `kV` 0.12 with a lower `kI`.
- **WCP items not adopted** (offered, not chosen): vector deadband + direction-preserving
  joystick curve (our squared-per-axis curve weakens diagonals); heading hold after rotation
  input stops; degree-based intake pivot with current-sense homing (would replace the
  drift-prone raw-rotation targets `-14.5` / `-0.144`).

## 8. Tuning checklist

17 `TODO(tune)` comments are in the code — each says what the number does, what too-high and
too-low look like, and how to measure it. Find them with:

```
grep -rn "TODO(tune)" src/main/java
```

They live in `Constants.java`, `RobotCommands.java`, `Landmarks.java`, `ShooterSubsys.java`,
and `HoodSubsys.java`. The big ones: heading `kAimP`, the flight-time table, the three shot
gates, `kShotTableMaxInches`, the debounce times, and the trench-spot offset.

## 9. First day on the field

1. **Verify the aim direction on both alliances.** Hold right bumper standing still and confirm
   the robot turns *toward* the hub — on red as well as blue. This single test validates both
   the shot direction and the red-alliance perspective conversion from Step 4.
2. **Check the Pigeon reads ~0 pitch/roll on flat carpet** (`Robot Pitch (deg)` /
   `Robot Roll (deg)`). If not, mount-calibrate in Tuner X before trusting `Ready/Level`.
3. **Confirm stationary shots still work** at several table distances with the slider at 0.
4. **Find the real maximum range.** Park at 140 in, confirm it scores, then back up in ~10 in
   steps, nudging the slider until each scores and noting `Target RPM`. It stops working when
   the hood maxes out (0.77) or the flywheel can't reach the RPM. Add table entries for every
   distance that scored, then raise `kShotTableMaxInches`.
5. **Film flight times while you're already parked** at those known distances — slow-mo 240 fps,
   count frames from ball-exit to hub-entry. Same drive gets you both measurements.
6. **Drive onto a ramp** with button 1 held and confirm `Ready/Level` goes red.
7. Only after all that: raise `kScoringSpeedToleranceMps` and try shooting on the move.

## 10. Building

`JAVA_HOME` points at `~/wpilib/2026/jdk`, which doesn't exist on this machine. Build with:

```
JAVA_HOME=/Library/Java/JavaVirtualMachines/temurin-21.jdk/Contents/Home ./gradlew build
```

Most Java files in this repo use **CRLF** line endings. Edit them in place; a whole-file
rewrite that converts to LF makes every line show as changed (this already cost one cleanup
commit, `10cd7d6`).
