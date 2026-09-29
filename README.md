# FRC 6925 Rebuilt Robot Code

Java command-based robot project for Team 6925's 2026 "Rebuilt" robot. The robot uses a CTRE Phoenix 6 swerve drivetrain, PathPlanner autonomous routines, a Limelight 3 AprilTag pose-estimation pipeline, a three-motor shooter, servo hood, feeder, and pivoting intake.

## Robot Features From First Principles

Each robot feature is built out of the same basic loop: read sensors or controller input, decide what state the robot should move toward, command motors/servos, and publish telemetry so humans can see what happened.

- Command-based robot structure. Files: `Main.java`, `Robot.java`, `RobotContainer.java`, `RobotCommands.java`.
  WPILib calls `Main`, `Main` starts `Robot`, and `Robot` runs the `CommandScheduler` every 20 ms. Subsystems own hardware. Commands describe actions. `RobotContainer` wires buttons and autonomous event names to commands.

- Swerve drivetrain. Files: `CommandSwerveDrivetrain.java`, `TunerConstants.java`, `tuner-project.json`.
  A swerve robot has four wheel modules. Each module can spin its wheel and rotate its wheel angle. To move in any direction, the drivetrain math converts desired robot motion into four wheel speeds and four wheel angles. `TunerConstants` defines the real hardware IDs, wheel locations, gear ratios, offsets, and gains. `CommandSwerveDrivetrain` wraps CTRE's drivetrain class so commands can control it.

- Field-centric driving. Files: `RobotContainer.java`, `CommandSwerveDrivetrain.java`.
  Joystick forward should mean "field forward," not "wherever the robot nose points." The drivetrain uses the gyro heading to rotate driver input from field coordinates into robot coordinates. This lets the driver think about the field instead of the robot's current angle.

- Joystick shaping and slew limiting. File: `RobotContainer.java`.
  Raw joysticks are noisy and twitchy. The code squares translation inputs for finer low-speed control, uses an `x^1.5` curve for rotation, applies deadbands near zero, and uses `SlewRateLimiter` so acceleration ramps in smoothly while stops happen quickly.

- Driver speed modes. Files: `RobotContainer.java`, `CommandSwerveDrivetrain.java`.
  The drivetrain normally drives at 75 percent of theoretical max speed. Buttons can toggle full speed, toggle 20 percent speed, or temporarily lower speed during other actions. This changes a multiplier before the final velocity command is sent.

- Brake and point-wheels-forward modes. Files: `RobotContainer.java`, `CommandSwerveDrivetrain.java`.
  Brake mode turns modules into a resisting stance. Point-wheels-forward commands all modules to face 0 degrees, which is useful for setup checks and quick alignment.

- SysId drivetrain characterization. Files: `RobotContainer.java`, `CommandSwerveDrivetrain.java`, `Telemetry.java`.
  SysId sends controlled voltage/rate tests to the drivetrain and logs the response. Those logs help tune feedforward and PID constants. This project has translation, steer, and rotation characterization routines, though the controller bindings currently select the default translation routine.

- Autonomous path following. Files: `RobotContainer.java`, `CommandSwerveDrivetrain.java`, `src/main/deploy/pathplanner/settings.json`, `src/main/deploy/pathplanner/autos/*`, `src/main/deploy/pathplanner/paths/*`.
  PathPlanner stores paths and autonomous routines as deploy files. `AutoBuilder` reads robot geometry/settings and turns a desired path into chassis speeds. Named commands let a path trigger robot actions like shooting or intake while driving.

- Vision pose estimation. Files: `LimelightSubsys.java`, `LimelightHelpers.java`, `RobotContainer.java`, `Robot.java`.
  The Limelight sees AprilTags and estimates where the robot is on the field. The drivetrain already estimates pose from wheels and gyro; vision measurements correct that estimate over time. `Robot.robotPeriodic()` updates vision outside autonomous, and disabled mode can reset pose from vision before a match or test.

- Disabled-mode vision seeding. Files: `Robot.java`, `RobotContainer.java`, `LimelightSubsys.java`.
  While disabled, the robot can safely trust a good AprilTag reading to set its starting pose. The code rejects vision jumps over 1 meter unless the current pose is near the origin, which helps avoid one bad camera frame corrupting the robot's position.

- Field landmarks and hub target selection. Files: `Landmarks.java`, `RobotCommands.java`.
  Auto-aim and distance shots need a target point. `Landmarks` stores blue and red hub positions in inches, publishes them to SmartDashboard for tuning, and returns the correct target based on Driver Station alliance.

- Distance-based shooting. Files: `RobotCommands.java`, `Constants.java`, `Landmarks.java`, `ShooterSubsys.java`, `HoodSubsys.java`.
  A ball needs different speed and angle depending on distance. The code measures distance from the robot pose to the hub, looks up/interpolates a `Shot` containing shooter RPM and hood position, then commands the shooter and hood.

- Shoot-while-moving compensation. Files: `RobotCommands.java`, `CommandSwerveDrivetrain.java`.
  A moving robot launches the ball with robot velocity added to ball velocity. The code estimates ball flight time and creates a "virtual target" offset by robot motion so the robot aims where the target will effectively be during flight.

- Hub auto-aim. Files: `RobotCommands.java`, `RobotContainer.java`, `Landmarks.java`.
  The robot calculates the angle from its current pose to the hub, subtracts its current heading, normalizes that heading error, and turns at a rate proportional to the error. The driver can still translate while the command controls rotation.

- Pass-shot auto-aim. Files: `RobotCommands.java`, `LimelightHelpers.java`, `Constants.java`.
  The robot checks whether the Limelight sees one of the trench AprilTags. If it does, it offsets the tag's `tx` angle inward and rotates toward that corrected angle while spinning the shooter to pass RPM and moving the hood to pass position.

- Shooter subsystem. Files: `ShooterSubsys.java`, `CTREConfigs.java`, `Constants.java`, `RobotCommands.java`.
  Three TalonFX motors spin flywheels. Velocity control means the code asks for RPM, the motor controller measures actual speed, and CTRE's closed loop adjusts voltage to hold the target. The shooter has a default idle command that holds 3000 RPM unless idle is toggled off.

- Hood subsystem. Files: `HoodSubsys.java`, `Constants.java`, `RobotCommands.java`.
  Two PWM servos set the launch angle. Code clamps requests to the allowed servo range, writes the same value to both servos, estimates whether the hood has reached position, and reports readiness.

- Feeder subsystem. Files: `FeederSubsys.java`, `CTREConfigs.java`, `RobotCommands.java`.
  The feeder moves balls from storage into the shooter. It controls the main feeder and the fuel-feed motor together using named speed presets: off, slow feed, fast feed, turbo feed, and reverse.

- Intake subsystem. Files: `IntakeSubsys.java`, `CTREConfigs.java`, `RobotCommands.java`.
  The intake has one roller motor to pull balls in and one rotator motor to deploy/retract the arm. The rotator stores a target position and has a default command that keeps holding that target. Other commands can deploy, retract, jog slowly, creep by small position steps, or oscillate to shake balls loose.

- Shoot command with intake motion. Files: `RobotCommands.java`, `FeederSubsys.java`, `IntakeSubsys.java`.
  Shooting is not only "turn on feeder." The command feeds balls while gradually raising the intake rotator toward a retracted position and adding a shake motion. On release it stops feeding and stores the last deployed position so another command can redeploy.

- Limelight exposure tuning. Files: `RobotCommands.java`, `LimelightHelpers.java`.
  The exposure-tune command steps camera exposure upward until a tag is detected continuously for a short stable time. It writes current and final exposure values to SmartDashboard.

- Telemetry and logging. Files: `Telemetry.java`, `Robot.java`, `ShooterSubsys.java`, `HoodSubsys.java`, `IntakeSubsys.java`, `LimelightSubsys.java`.
  Telemetry is how the robot explains itself. This project publishes drivetrain pose/speeds/module states to NetworkTables and CTRE SignalLogger, while subsystems publish useful SmartDashboard values such as shooter RPM, hood readiness, intake position, selected auto, and robot speed.

## Requirements

- WPILib 2026.
- Java 17.
- GradleRIO 2026.2.1.
- CTRE Phoenix 6 vendordep 26.1.1.
- PathPlannerLib vendordep 2026.1.2.
- A roboRIO/Systemcore configured for Team 6925.
- A CANivore bus named `CANivore`.
- Limelight named `limelight` on NetworkTables.

## Common Commands

```sh
./gradlew build
./gradlew test
./gradlew simulateJava
./gradlew deploy
```

On Windows, use `gradlew.bat` instead of `./gradlew`.

## Controls With Source Files

### Driver Xbox Controller, Port 0

- Left stick: field-centric translation. Defined in `RobotContainer.java`; executed through `CommandSwerveDrivetrain.java`.
- Right stick X: rotation. Defined in `RobotContainer.java`; executed through `CommandSwerveDrivetrain.java`.
- A: swerve brake while held. Binding in `RobotContainer.java`; brake request from CTRE swerve API through `CommandSwerveDrivetrain.java`.
- B: toggle full speed vs default speed. Binding in `RobotContainer.java`; speed multiplier method in `CommandSwerveDrivetrain.java`.
- Left trigger: point wheels forward for 0.5 seconds. Binding in `RobotContainer.java`; point-wheels request through `CommandSwerveDrivetrain.java`.
- Right trigger: toggle 20 percent speed mode. Binding in `RobotContainer.java`; speed multiplier method in `CommandSwerveDrivetrain.java`.
- Left bumper: reseed field-centric heading. Binding in `RobotContainer.java`; drivetrain method inherited through `CommandSwerveDrivetrain.java`.
- Right bumper: auto-aim at hub and wind up shooter using distance-based settings while held. Binding in `RobotContainer.java`; command in `RobotCommands.java`; target point from `Landmarks.java`; shooter/hood actions in `ShooterSubsys.java` and `HoodSubsys.java`.
- Y: auto-aim pass and wind up for pass shot while held. Binding in `RobotContainer.java`; command in `RobotCommands.java`; Limelight readings through `LimelightHelpers.java`.
- POV up: wind up for 75.125 inch shot while held. Binding in `RobotContainer.java`; command in `RobotCommands.java`; constants in `Constants.java`.
- POV down: slow manual intake rotator movement. Binding in `RobotContainer.java`; command in `IntakeSubsys.java`.
- POV left/right: creep intake rotator backward/forward. Binding in `RobotContainer.java`; command in `IntakeSubsys.java`.
- Back + Y/X: SysId dynamic forward/reverse. Binding in `RobotContainer.java`; routines in `CommandSwerveDrivetrain.java`; logs through `Telemetry.java` and CTRE SignalLogger.
- Start + Y/X: SysId quasistatic forward/reverse. Binding in `RobotContainer.java`; routines in `CommandSwerveDrivetrain.java`; logs through `Telemetry.java` and CTRE SignalLogger.

### Operator 3D Joystick, Port 1

- Button 1: shoot/feed while held; redeploy intake on release. Binding in `RobotContainer.java`; commands in `RobotCommands.java`; hardware actions in `FeederSubsys.java` and `IntakeSubsys.java`.
- Button 2: run intake with oscillation while held; also halves default speed while held. Binding in `RobotContainer.java`; intake command in `IntakeSubsys.java`; speed multiplier in `CommandSwerveDrivetrain.java`.
- Button 3: toggle shooter idle on/off. Binding in `RobotContainer.java`; command in `RobotCommands.java`; state in `ShooterSubsys.java`.
- Button 4: retract intake slowly to the stored retracted position. Binding in `RobotContainer.java`; command in `IntakeSubsys.java`.
- Button 5: manual distance-based wind-up while held. Binding in `RobotContainer.java`; command in `RobotCommands.java`; shot table constants in `Constants.java`.
- Button 6: deploy intake slowly. Binding in `RobotContainer.java`; command in `IntakeSubsys.java`.
- Button 7: close hub wind-up while held. Binding in `RobotContainer.java`; command in `RobotCommands.java`; constants in `Constants.java`.
- Button 8: pass-shot wind-up while held. Binding in `RobotContainer.java`; command in `RobotCommands.java`; constants in `Constants.java`.
- Button 9: close-range wind-up while held. Binding in `RobotContainer.java`; command in `RobotCommands.java`; constants in `Constants.java`.
- Button 10: point swerve wheels forward while held. Binding in `RobotContainer.java`; request executed through `CommandSwerveDrivetrain.java`.
- Button 11: test wind-up while held. Binding in `RobotContainer.java`; command in `RobotCommands.java`; constants in `Constants.java`.
- Button 12: retract intake with oscillation while held. Binding in `RobotContainer.java`; command in `IntakeSubsys.java`.
- POV down: reverse intake and feeder while held. Binding in `RobotContainer.java`; command in `RobotCommands.java`; hardware actions in `IntakeSubsys.java` and `FeederSubsys.java`.
- POV left: auto-tune Limelight exposure. Binding in `RobotContainer.java`; command in `RobotCommands.java`; NetworkTables helper in `LimelightHelpers.java`.

## Autonomous

Autonomous routines are built with PathPlanner. `RobotContainer` builds a chooser with `M-S` as the default and registers these named commands:

- `shoot`: feed balls while raising/oscillating the intake.
- `autoShoot`: timed auto shot for 3 seconds.
- `StopFeed`: stop feeder motors.
- `windUp`: fixed teleop-style wind-up.
- `windUpOnce`: set fixed shooter/hood values once.
- `autoWindUpClose`: close shot wind-up with wait-for-speed timeout.
- `autoWindUpCloser`: closest shot wind-up with wait-for-speed timeout.
- `AdjustedWindUp`: continuously apply distance-based shooter/hood settings.
- `AdjustedShootWhileMoving`: distance-adjusted shooting while moving.
- `AdjustedWindUpOnce`: set distance-based shooter/hood values once and wait for speed.
- `autoAimAndWindUp`: rotate toward the hub, set shot values, and finish when aimed/ready or timed out.
- `IntakeMid`: set intake roller to mid speed.
- `IntakeFast`: set intake roller to fast speed.
- `StopIntake`: stop intake roller.
- `intakeDeploy`: move intake rotator to `-14.5` motor rotations.
- `waitForDeploy`: wait until intake rotator is near its target, with timeout.
- `intakeBounce`: currently a no-op.
- `jolt`, `ClimbUp`, `ClimbDown`, `climbDown`, `StopClimber`, `hopperDeploy`, `VisionUpdate`: currently no-ops kept so older autos still load.
- `hoodReset`: set hood position to 0.

## Wireless Testing

For current VH-109 radio testing, the preferred at-home setup is two VH-109 radios:

1. Configure the robot radio at `http://radio.local/` in Robot Radio Mode.
2. Configure the second VH-109 in Access Point Mode.
3. Use the same team number, suffix, 6 GHz WPA/SAE key, and 2.4 GHz WPA/SAE key on both radios.
4. Mount/power the robot radio on the robot and connect it to the roboRIO/Systemcore Ethernet.
5. Put the access-point radio high with clear line of sight.
6. Connect the Driver Station laptop by Ethernet to the access-point radio.
7. Set the Driver Station team number to 6925 and leave the laptop network adapter on DHCP.
8. In Driver Station Diagnostics, confirm Robot Radio and Robot are green, then enable in Teleop for a short controls test.

If you only have one VH-109, standalone 2.4 GHz mode can host a direct network by turning on DIP switch 3 and connecting to the configured 2.4 GHz network. Treat that as a fallback only: the WPILib/Vivid docs warn that 2.4 GHz standalone testing can perform poorly, especially in noisy school/shop environments.

Useful addresses for Team 6925 use `10.TE.AM.x`, so `TE.AM` is `69.25`:

- Robot radio: `10.69.25.1`.
- roboRIO/Systemcore: usually `10.69.25.2`.
- Access point radio at home: `10.69.25.4`.
- Driver Station: DHCP address in the `10.69.25.x` range.

## Potential Bugs And Risks

- `RobotContainer` has a large overview comment that is stale in several places. It references a climber subsystem and older constants that are not present in the current code.
- Several PathPlanner named commands related to climber/hopper behavior are registered as no-ops. Autos containing those events will run, but the physical action will not happen.
- `LimelightSubsys` alliance tag filtering is commented out, so the robot accepts any valid tag instead of limiting to the intended alliance tag IDs.
- `Robot.robotPeriodic()` calls `getAutonomousCommand()` every loop only to publish its name. Depending on chooser behavior this is probably harmless, but it is extra work and may obscure whether a command was selected once or repeatedly queried.
- `ShooterSubsys` only checks motor 8 for `Shooter At Speed`; motors 9 and 10 are displayed but do not gate readiness.
- `HoodSubsys` starts `lastUpdateTime` at 0 seconds, so the first periodic update can compute a very large elapsed time and jump the estimated current position directly to the target.
- `HoodSubsys` sets `kSmoothingAlpha` to 1.0, which disables the intended smoothing despite the smoothing comments.
- `RobotCommands.aimAndPass()` creates a drivetrain command but only requires shooter and hood, not drivetrain. That can conflict with the drivetrain default command trying to control the drive at the same time.
- `RobotCommands.Shoot()` directly commands the intake rotator through `setRotatorOscillate()` but does not update `rotatorTargetPosition` until release. The default hold command may fight or snap if command scheduling changes.
- `RobotCommands.manualWindUp()` writes `Manual Distance (in)` when the command factory is called, not when the command runs. It is fine after startup, but surprising during tests.

## File Reference

### Root Files

- `README.md`: this project overview, feature list, controls, wireless test notes, risks, and file map.
- `WPILib-License.md`: WPILib BSD license text for generated/base robot code.
- `build.gradle`: GradleRIO Java project configuration, dependencies, deploy target, simulation GUI/Driver Station setup, fat-jar packaging, and Java compile options.
- `settings.gradle`: Gradle plugin repository configuration, including local WPILib 2026 Maven lookup.
- `gradlew`: Unix/macOS Gradle wrapper launcher.
- `gradlew.bat`: Windows Gradle wrapper launcher.
- `gradle/wrapper/gradle-wrapper.jar`: Gradle wrapper binary used by `gradlew`.
- `gradle/wrapper/gradle-wrapper.properties`: Gradle wrapper distribution settings.
- `tuner-project.json`: CTRE Tuner X source project for the swerve module hardware layout and offsets.

### Java Source

- `src/main/java/frc/robot/Main.java`: robot program entry point; starts `Robot`.
- `src/main/java/frc/robot/Robot.java`: WPILib `TimedRobot` lifecycle; runs the command scheduler, schedules/cancels autonomous, updates vision outside autonomous, seeds pose while disabled, and publishes selected auto and speed.
- `src/main/java/frc/robot/RobotContainer.java`: constructs subsystems, configures all driver/operator bindings, registers PathPlanner named commands, creates the autonomous chooser, and provides vision update/pose-seeding helpers.
- `src/main/java/frc/robot/RobotCommands.java`: central factory for composed robot commands, including shooter wind-up, feed/shoot, intake commands, reverse, auto-aim, pass aim, distance interpolation, moving shots, and Limelight exposure tuning.
- `src/main/java/frc/robot/Constants.java`: robot-wide constants for controller port, shooter RPMs, hood positions, interpolation table points, aim gains, lookahead time, hub offset, and pass aim offset.
- `src/main/java/frc/robot/CTREConfigs.java`: CTRE TalonFX configuration objects for shooter, intake, intake rotator, fuel feed, and feeder motors.
- `src/main/java/frc/robot/Landmarks.java`: field landmark helper; publishes default blue/red hub positions to SmartDashboard and returns the alliance-specific target position.
- `src/main/java/frc/robot/LimelightHelpers.java`: Limelight-provided helper library for NetworkTables access, pose conversion, target data parsing, MegaTag pose estimates, camera settings, LED/stream/pipeline control, snapshots, rewind capture, and USB port forwarding.
- `src/main/java/frc/robot/Telemetry.java`: publishes swerve pose, speeds, module states/targets/positions, odometry timing, Field2d pose data, and CTRE SignalLogger records.
- `src/main/java/frc/robot/generated/TunerConstants.java`: CTRE Tuner X generated swerve constants, CAN IDs, module offsets, gearing, wheel radius, gains, current limits, Pigeon configuration, and drivetrain factory.
- `src/main/java/frc/robot/subsystems/CommandSwerveDrivetrain.java`: command-based wrapper around Phoenix 6 `SwerveDrivetrain`; configures PathPlanner AutoBuilder, applies operator perspective, supports SysId, simulation updates, vision timestamp conversion, speed multipliers, and request commands.
- `src/main/java/frc/robot/subsystems/ShooterSubsys.java`: controls three TalonFX shooter motors in velocity mode, default idle RPM behavior, right-motor-only testing, stop, at-speed check, and shooter dashboard telemetry.
- `src/main/java/frc/robot/subsystems/FeederSubsys.java`: controls main feeder CAN 51 and fuel-feed CAN 11 together with enum-defined speeds.
- `src/main/java/frc/robot/subsystems/IntakeSubsys.java`: controls intake roller CAN 45 and intake rotator CAN 50, including rotator hold, oscillation, bounce, slow/manual/creep rotation, absolute/relative positioning, deploy waits, and intake telemetry.
- `src/main/java/frc/robot/subsystems/HoodSubsys.java`: controls left/right hood servos on PWM 0/1, clamps hood position, estimates travel time, reports target/current position through Sendable, and publishes at-position telemetry.
- `src/main/java/frc/robot/subsystems/LimelightSubsys.java`: configures Limelight camera pose and target offset, sends robot orientation for MegaTag2, filters measurements by tag count/area, computes vision standard deviations, and publishes estimated pose.
- `src/main/java/frc/lib/util/CommandX3DController.java`: command-based wrapper for the operator 3D joystick axes, slider, and trigger button.

### Deploy Files

- `src/main/deploy/example.txt`: WPILib placeholder explaining that files in `deploy` are copied to the roboRIO deploy directory.
- `src/main/deploy/pathplanner/settings.json`: PathPlanner robot geometry, mass, gearing, wheel, current limit, velocity, acceleration, and module-location settings.
- `src/main/deploy/pathplanner/navgrid.json`: PathPlanner navigation grid/obstacle map for the 2026 Rebuilt field.

### PathPlanner Autos

- `src/main/deploy/pathplanner/autos/LeftBump-NZ.auto`: autonomous routine using left-bump and neutral-zone path/event sequencing.
- `src/main/deploy/pathplanner/autos/RightBump-RightBump.auto`: autonomous routine moving from right bump toward another right-bump scoring/intake sequence.
- `src/main/deploy/pathplanner/autos/NoMove-Closer.auto`: no-drive close auto that winds up closer and shoots.
- `src/main/deploy/pathplanner/autos/RT-HP-MidHub.auto`: right-trench/human-player to mid-hub autonomous routine.
- `src/main/deploy/pathplanner/autos/Test.auto`: test autonomous routine for validating PathPlanner behavior.
- `src/main/deploy/pathplanner/autos/RightBump-NZ.auto`: right-bump to neutral-zone autonomous routine.
- `src/main/deploy/pathplanner/autos/RightBump-MiddleHub.auto`: right-bump to middle-hub autonomous routine.
- `src/main/deploy/pathplanner/autos/RT-MH-DS.auto`: right-trench/mid-hub/depot-side autonomous routine.
- `src/main/deploy/pathplanner/autos/RT-HP-RightBump.auto`: right-trench/human-player to right-bump autonomous routine.
- `src/main/deploy/pathplanner/autos/HELL YEAH!!! Offset R HM ends RB.auto`: experimental offset routine ending near right bump.
- `src/main/deploy/pathplanner/autos/Turn test.auto`: turn-only autonomous test.
- `src/main/deploy/pathplanner/autos/M-S.auto`: middle-to-shoot default autonomous.
- `src/main/deploy/pathplanner/autos/M-S-D-S.auto`: middle-shoot, depot, shoot autonomous routine.

### PathPlanner Paths

- `src/main/deploy/pathplanner/paths/RB-S.path`: right-bump to shoot path.
- `src/main/deploy/pathplanner/paths/New New Path.path`: experimental generated path.
- `src/main/deploy/pathplanner/paths/New New New Path.path`: experimental generated path.
- `src/main/deploy/pathplanner/paths/LB-LB.path`: left-bump to left-bump path.
- `src/main/deploy/pathplanner/paths/Depot Intake Pass.path`: depot intake/pass path.
- `src/main/deploy/pathplanner/paths/RT-HP.path`: right-trench to human-player path.
- `src/main/deploy/pathplanner/paths/HP-RB.path`: human-player to right-bump path.
- `src/main/deploy/pathplanner/paths/Basic Drive-and-Shoot from right corner.path`: simple right-corner drive-and-shoot test path.
- `src/main/deploy/pathplanner/paths/HP-MH.path`: human-player to middle-hub path.
- `src/main/deploy/pathplanner/paths/RB-RB.path`: right-bump to right-bump path.
- `src/main/deploy/pathplanner/paths/New Path.path`: experimental generated path.
- `src/main/deploy/pathplanner/paths/RW-NZI-Hub.path`: right-wall to neutral-zone-intake to hub path.
- `src/main/deploy/pathplanner/paths/Bump Approach Right.path`: approach path for the right bump.
- `src/main/deploy/pathplanner/paths/RB2-RB.path`: alternate right-bump to right-bump path.
- `src/main/deploy/pathplanner/paths/Test drive.path`: test driving path.
- `src/main/deploy/pathplanner/paths/DI.path`: depot-intake path.
- `src/main/deploy/pathplanner/paths/post-depot to climb.path`: post-depot path retained from older climb workflow.
- `src/main/deploy/pathplanner/paths/LB-S.path`: left-bump to shoot path.
- `src/main/deploy/pathplanner/paths/LW to Shoot.path`: left-wall to shoot path.
- `src/main/deploy/pathplanner/paths/LW Shoot to Depot Approach.path`: left-wall shooting position to depot approach path.
- `src/main/deploy/pathplanner/paths/Depot to Hub.path`: depot to hub path.
- `src/main/deploy/pathplanner/paths/RB-HP.path`: right-bump to human-player path.
- `src/main/deploy/pathplanner/paths/Intake Deploy Jolt Center.path`: center jolt/deploy path retained for old intake deployment behavior.
- `src/main/deploy/pathplanner/paths/Intake Deploy Jolt.path`: jolt/deploy path retained for old intake deployment behavior.
- `src/main/deploy/pathplanner/paths/NZ from LB.path`: neutral-zone path from left bump.
- `src/main/deploy/pathplanner/paths/LH-CL1.path`: left-hub to climb-level-1 named path retained from older workflow.
- `src/main/deploy/pathplanner/paths/LB-NZI-RB-S.path`: left-bump to neutral-zone-intake to right-bump to shoot path.
- `src/main/deploy/pathplanner/paths/Basic Shoot from infront target.path`: simple in-front-of-target shooting path.
- `src/main/deploy/pathplanner/paths/R-NZ-SA.path`: right-side neutral-zone scoring/approach path.
- `src/main/deploy/pathplanner/paths/LW-NZI-Hub.path`: left-wall to neutral-zone-intake to hub path.
- `src/main/deploy/pathplanner/paths/Hub Shoot Backup.path`: backup path after hub shot.
- `src/main/deploy/pathplanner/paths/MH-D.path`: middle-hub to depot path.
- `src/main/deploy/pathplanner/paths/Turn test.path`: turn-only path for rotation testing.
- `src/main/deploy/pathplanner/paths/Basic drive and shoot from left.path`: simple left-side drive-and-shoot test path.
- `src/main/deploy/pathplanner/paths/NZ from RB.path`: neutral-zone path from right bump.
- `src/main/deploy/pathplanner/paths/RW to Shoot.path`: right-wall to shoot path.

### Vendor Dependencies

- `vendordeps/Phoenix6-26.1.1.json`: CTRE Phoenix 6 Java/JNI vendordep for TalonFX, CANcoder, Pigeon2, CANivore, swerve, simulation, and CTRE logging APIs.
- `vendordeps/PathplannerLib-2026.1.2.json`: PathPlannerLib vendordep for autonomous path following, AutoBuilder, named commands, GUI robot config, and holonomic control.
- `vendordeps/WPILibNewCommands.json`: WPILib command-based library vendordep.

### Field Files

- `edu/wpi/first/fields/2026-rebuilt.json`: custom 2026 Rebuilt field definition used by tooling, including field image name, field corners, size, and units.
