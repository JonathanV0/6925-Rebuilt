// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/*
 * =========================================================================
 *                     FRC TEAM 6925 - ROBOT OVERVIEW
 * =========================================================================
 * Keep this block in sync with the code below it. If you change a binding,
 * a constant, or a subsystem, update the matching line here.
 *
 * DRIVETRAIN (CommandSwerveDrivetrain)
 *   - Swerve drive using CTRE Tuner X-generated constants (TunerConstants)
 *   - Field-centric control via Xbox controller (port 0)
 *   - Left stick = translation (squared input + 1.5/s slew accel, instant decel)
 *   - Right stick X = rotation (x^1.5 curve), max 1.5 rot/s
 *   - Default speed multiplier is 75% (kDefaultSpeedMulti)
 *
 * DRIVER CONTROLS (Xbox, port 0)
 *   Left trigger  = Snap wheels to 0° for 0.5 s (press)
 *   Right trigger = Toggle 1/5 speed (press)
 *   Left bumper   = Reseed field-centric heading (gyro reset)
 *   Right bumper  = aimAndWindUp: face the hub (shoot-on-the-move virtual
 *                   target) + distance-table RPM/hood, driver keeps translation
 *   A             = Brake (X-lock wheels) while held
 *   B             = Toggle full speed (100%)
 *   Y             = aimAndPass: face 15° inward from a trench tag + pass RPM/hood
 *   POV up        = windUp75 (75" table point — hold)
 *   POV down      = Intake slow rotate (hold)
 *   POV left/right= Intake creep rotate ±1 (hold)
 *   Back/Start + X/Y = SysId dynamic/quasistatic routines
 *
 * SHOOTER (ShooterSubsys) — 3 TalonFX motors on CANivore
 *   - CAN 8  = right motor (Clockwise_Positive)
 *   - CAN 9  = middle motor (CounterClockwise_Positive)
 *   - CAN 10 = left motor (CounterClockwise_Positive)
 *   - Each motor runs its OWN VelocityVoltage PID (kP=0.59, kI=0.5, kV=0.1)
 *   - Current limits: 90A stator / 70A supply (all three)
 *   - Neutral mode: Coast (flywheel spins down naturally)
 *   - Idle: default command holds kIdleRPM (3000) when idle is on (button 3 toggles)
 *   - Fixed shots: kFixedShotRPM = 3350;  Pass: kPassShotRPM = 5650
 *   - Distance-adjusted shots use the 47"–140" interpolation table in
 *     RobotCommands (Constants.ShooterConstants kRPMAtXXin / kHoodAtXXin, +150 RPM)
 *   - Table RPMs are scaled by the "RPM Percent Adder" slider on the Shuffleboard
 *     "Operator" tab (-10..+50 %, default 0). Fixed and pass shots are NOT scaled.
 *   - "Shooter At Speed" = ALL THREE motors within kVelocityToleranceRPM (300)
 *     of target; per-motor booleans are also on SmartDashboard
 *
 * HOOD (HoodSubsys) — 2 servos
 *   - PWM 0 = left servo,  PWM 1 = right servo
 *   - Position range: 0.01 (low) to 0.77 (high), tolerance 0.01
 *   - "Hood At Position" models servo travel at 20 mm/s over a 100 mm stroke
 *   - Adjusts shot angle; paired with shooter RPM via the distance table
 *
 * FEEDER (FeederSubsys) — 2 TalonFX motors on CANivore
 *   - CAN 51 = main feeder motor (feeds balls into shooter) — 40A/40A
 *   - CAN 11 = fuel feed motor (on the shooter, pushes balls to flywheels) — 50A/40A
 *   - Both controlled together via FeederSpeed enum (feeder / fuelFeed):
 *       OFF        →  0.0 /  0.0
 *       FEED_SLOW  → -0.3 /  0.1
 *       FEED_FAST  → -0.8 /  0.8
 *       FEED_TURBO → -1.0 /  1.0
 *       REVERSE    →  0.5 / -0.4
 *       PRESHOT_REVERSE → 0.0 / -0.2  (fuel feed backs ball off flywheel while not ready)
 *   - Neutral mode: Coast
 *   - Feeder is a SEPARATE subsystem from shooter so both run at once
 *
 * INTAKE (IntakeSubsys) — 2 TalonFX motors on CANivore
 *   - CAN 45 = intake roller (80A/80A, Coast)
 *   - CAN 50 = intake rotator (80A/80A, Brake — holds position when idle)
 *   - Rotator uses PositionVoltage PID with three gain slots:
 *       Slot0 gentle (kP=1.0, kV ff), Slot1 snappy (kP=2, kD=0.5),
 *       Slot2 medium (kP=1.5, kD=0.25, kV ff)
 *   - Default command holds the last target position
 *   - IntakeSpeed values are NEGATIVE (motor spins inward to grab balls):
 *       SLOW -0.1, MID -0.25, FAST -0.7, TURBO -1.0, REVERSE +0.25
 *
 * CLIMBER — REMOVED
 *   - No climber motor on the robot. Climber-related PathPlanner named
 *     commands are registered as Commands.none() so old autos still load.
 *
 * LIMELIGHT (LimelightSubsys) — ENABLED
 *   - MegaTag2 pose estimation with alliance-based tag ID filtering
 *   - Pose + per-frame standard deviations fused into the drivetrain's
 *     Kalman filter every loop outside autonomous (RobotContainer.updateVision)
 *   - While disabled, seeds the pose from vision, rejecting jumps > 1 m
 *   - Camera: 1.46" behind center, 25.39" high, 20.37° above horizontal
 *   - "Vision Enabled" SmartDashboard boolean turns fusion off for testing
 *
 * SHOT READINESS GATE (RobotCommands.isReadyToShoot)
 *   - Button 1 only feeds while ALL of: shooter at speed (3 motors), hood at
 *     position, heading error < kScoringHeadingToleranceDeg (4°) while an aim
 *     command is active, robot speed < kScoringSpeedToleranceMps (0.15 m/s),
 *     distance to hub >= kMinimumShotDistanceMeters (1.5 m)
 *   - Each gate is shown on SmartDashboard as Ready/AtSpeed, Ready/HoodAtPos,
 *     Ready/Heading, Ready/Speed, Ready/Distance, and Ready/ALL
 *   - Ready/ALL and "Shooter At Speed" are rising-edge debounced 0.05 s so one
 *     noisy sensor frame can't start the feeder
 *   - "Ignore Shot Gates" (SmartDashboard boolean) bypasses the gate entirely
 *
 * OPERATOR CONTROLS (X3D Joystick, port 1)
 *   Button 1  = Gated Shoot (feeder FEED_FAST only while ready, PRESHOT_REVERSE while
 *               not ready, + intake bounce — hold) + 1/5 drive speed.
 *               On release: intake redeploys to last deployed position
 *   Button 2  = Intake with Oscillate (TURBO — hold) + 37.5% drive speed
 *   Button 3  = Toggle shooter idle on/off (press)
 *   Button 4  = Retract Intake (slow to -0.144 rot, 0.2 duty — press)
 *   Button 5  = Manual Wind Up from "Manual Distance (in)" on SmartDashboard (hold)
 *   Button 6  = Deploy Intake (slow to -14.0 rot, 0.3 duty — press)
 *   Button 7  = Wind Up Closer (3350 RPM, hood 0.0 — hold, in front of hub)
 *   Button 8  = Wind Up Pass (5650 RPM, hood 0.7 — hold)
 *   Button 9  = Wind Up Close (3350 RPM, hood 0.3 — hold)
 *   Button 10 = Snap Wheels to 0° (hold)
 *   Button 11 = Manual Shoot (ungated override — feeds immediately, same intake
 *               bounce / redeploy / 1/5 drive speed as button 1)
 *   Button 12 = Retract with Oscillate (FAST — hold)
 *   Hat Down  = Reverse All (eject jammed ball — intake + feeder backward)
 *   Hat Left  = Auto-tune Limelight exposure (press)
 *   Hat Up    = Shoot from RIGHT_TRENCH known spot (re-seeds odometry X/Y — hold)
 *   Hat Right = Shoot from LEFT_TRENCH known spot (re-seeds odometry X/Y — hold)
 *               (Landmarks.KnownSpot: trench tag + 0.5 m toward hub, driver's L/R)
 *
 * AUTONOMOUS
 *   - Uses PathPlanner with NamedCommands for event markers; default auto "M-S"
 *   - Shooting: shoot, autoShoot, StopFeed, windUp, windUpOnce,
 *     autoWindUpClose, autoWindUpCloser, AdjustedWindUp,
 *     AdjustedShootWhileMoving, AdjustedWindUpOnce, autoAimAndWindUp, hoodReset
 *   - Intake: IntakeMid, IntakeFast, StopIntake, intakeDeploy, waitForDeploy
 *   - No-ops kept for old autos: intakeBounce, jolt, ClimbUp, ClimbDown,
 *     climbDown, StopClimber, hopperDeploy, VisionUpdate
 *   - Vision fusion is paused during autonomous (Robot.robotPeriodic)
 *   - Shoot-while-moving and aimAndWindUp lead the target by the ball flight time
 *     from RobotCommands.distanceToFlightTimeSec (1.5–4.5 m table, placeholder
 *     values); outside that range they fall back to kLookAheadSeconds = 0.25 s
 *   - Team note: robot is too tall for the trench — cross via the bump ramps
 *
 * LIVE TUNING (frc.robot.util.TunableNumber)
 *   - Constants.kTuningMode = true exposes "/Tuning/Aim/kAimP" and
 *     "/Tuning/Shooter/VelocityToleranceRPM" on NetworkTables for live edits.
 *     With it false (competition default) they are plain constants.
 *
 * MOTOR CAN IDs (all on the "CANivore" bus)
 *   8  = Shooter right
 *   9  = Shooter middle
 *   10 = Shooter left
 *   11 = Fuel feed (on shooter, controlled by FeederSubsys)
 *   45 = Intake roller
 *   50 = Intake rotator
 *   51 = Feeder
 *   (Swerve drive/steer motors and CANcoders are defined in TunerConstants)
 *
 * =========================================================================
 */

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.lib.util.CommandX3DController;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.FeederSubsys;
import frc.robot.subsystems.IntakeSubsys;
import frc.robot.subsystems.HoodSubsys;
import frc.robot.subsystems.LimelightSubsys;
import edu.wpi.first.math.filter.SlewRateLimiter;
import frc.robot.subsystems.ShooterSubsys;
import frc.robot.util.OperatorDashboard;

public class RobotContainer {
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(1.5).in(RadiansPerSecond); // 1.5 rotations per second max angular velocity

    // Slew rate limiters: 1.5/sec accel, 100/sec decel (instant stop)
    private final SlewRateLimiter xLimiter = new SlewRateLimiter(1.5, -100, 0);
    private final SlewRateLimiter yLimiter = new SlewRateLimiter(1.5, -100, 0);

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.05).withRotationalDeadband(MaxAngularRate * 0.075) // 30% translation, 15% rotation deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);
    private final SendableChooser<Command> autoChooser;

    private final CommandXboxController joystick = new CommandXboxController(0);
    private final CommandX3DController operator = new CommandX3DController(1);


    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    // Subsystems from Re-made branch
    private final ShooterSubsys shooter = new ShooterSubsys();
    private final IntakeSubsys intake = new IntakeSubsys();
    private final FeederSubsys feeder = new FeederSubsys();
    private final HoodSubsys hood = new HoodSubsys();
    private final LimelightSubsys limelight = new LimelightSubsys("limelight", () -> drivetrain.getState().Pose);
    private final OperatorDashboard operatorDashboard = new OperatorDashboard();

    public RobotContainer() {
        // Dependency injection: RobotCommands gets its collaborators handed in rather than
        // creating them, so there's exactly one instance of each and no hidden globals.
        RobotCommands.init(shooter, feeder, hood, intake, drivetrain, limelight, operatorDashboard);
        SmartDashboard.putBoolean("Vision Enabled", true);
        configureBindings();
        // Register named commands for PathPlanner event markers
        // ── Shooting ──────────────────────────────────────────────────────────
        NamedCommands.registerCommand("shoot", RobotCommands.Shoot());
        NamedCommands.registerCommand("autoShoot", RobotCommands.autoShoot(3));
        NamedCommands.registerCommand("StopFeed", RobotCommands.stopFeed());
        // Fixed shot (no vision): set RPM/hood to hardcoded values
        NamedCommands.registerCommand("windUp", RobotCommands.windUp());
        NamedCommands.registerCommand("windUpOnce", RobotCommands.windUpOnce());
        // Auto wind-up commands: set RPM/hood, wait until at speed (max 2s), then finish
        NamedCommands.registerCommand("autoWindUpClose", RobotCommands.autoWindUpClose());
        NamedCommands.registerCommand("autoWindUpCloser", RobotCommands.autoWindUpCloser());
        // Distance-adjusted shot: interpolates RPM/hood from odometry distance
        // For the vision auto variant, pair this with accurate pose correction
        NamedCommands.registerCommand("AdjustedWindUp", RobotCommands.adjustedWindUp());
        // Moving shot: adjusts RPM/hood continuously + feeds; use as deadline alongside a path
        NamedCommands.registerCommand("AdjustedShootWhileMoving", RobotCommands.adjustedShootWhileMoving());
        // Static shot wind-up: snaps to distance-based RPM/hood then waits for spinup
        NamedCommands.registerCommand("AdjustedWindUpOnce", RobotCommands.adjustedWindUpOnce());
        // Auto-aim for autonomous: rotates robot to face hub + sets RPM/hood, finishes when ready
        NamedCommands.registerCommand("autoAimAndWindUp", RobotCommands.autoAimAndWindUp());
        // ── Intake ────────────────────────────────────────────────────────────
        NamedCommands.registerCommand("IntakeMid", RobotCommands.intakeMid());
        NamedCommands.registerCommand("IntakeFast", RobotCommands.intakeFast());
        NamedCommands.registerCommand("StopIntake", RobotCommands.stopIntake());
        NamedCommands.registerCommand("intakeDeploy", intake.goToPositionCommand(-14.5));
        NamedCommands.registerCommand("waitForDeploy", intake.waitForDeployCommand());
        NamedCommands.registerCommand("intakeBounce", Commands.none()); // bounce is now built into autoShoot
        // Vision updates now run automatically in robotPeriodic() — no named command needed
        // ── Climber commands (motor removed — register as no-ops so PathPlanner autos don't error)
        NamedCommands.registerCommand("jolt", Commands.none());
        NamedCommands.registerCommand("ClimbUp", Commands.none());
        NamedCommands.registerCommand("ClimbDown", Commands.none());
        NamedCommands.registerCommand("climbDown", Commands.none());
        NamedCommands.registerCommand("StopClimber", Commands.none());
        NamedCommands.registerCommand("hopperDeploy", Commands.none());
        NamedCommands.registerCommand("VisionUpdate", Commands.none());
        NamedCommands.registerCommand("hoodReset", Commands.runOnce(() -> hood.setPosition(0)));

        autoChooser = AutoBuilder.buildAutoChooser("M-S");
        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() -> {
                // Squared input + slew rate limiting for smooth, precise control
                double leftY = joystick.getLeftY();
                double leftX = joystick.getLeftX();
                double rightX = joystick.getRightX();

                // If translation joystick is within deadband, stop instantly (no slew ramp-down)
                boolean translationDead = Math.abs(leftY) < 0.05 && Math.abs(leftX) < 0.05;
                boolean rotationDead = Math.abs(rightX) < 0.075;

                if (translationDead) {
                    xLimiter.reset(0);
                    yLimiter.reset(0);
                }

                double squaredY = translationDead ? 0 : -Math.copySign(leftY * leftY, leftY);
                double squaredX = translationDead ? 0 : -Math.copySign(leftX * leftX, leftX);
                double sqrtRot = -Math.copySign(Math.pow(Math.abs(rightX), 1.5), rightX); // x^1.5 curve for rotation
                double slewedY = translationDead ? 0 : xLimiter.calculate(squaredY);
                double slewedX = translationDead ? 0 : yLimiter.calculate(squaredX);
                return drive.withVelocityX(slewedY * MaxSpeed * drivetrain.getCurrentSpeedMulti())
                    .withVelocityY(slewedX * MaxSpeed * drivetrain.getCurrentSpeedMulti())
                    .withRotationalRate(sqrtRot * MaxAngularRate);
            })
        );

        // Snap wheels to 0 for 0.5s then resume driving (steer motors hold in Brake mode)
        joystick.leftTrigger().onTrue(
            drivetrain.applyRequest(() -> point.withModuleDirection(new Rotation2d(0)))
                .withTimeout(0.5));
        // Toggle 1/5th speed with right trigger (press once to toggle)
        joystick.rightTrigger().onTrue(drivetrain.toggleSpeedMulti(1.0 / 5.0));
        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        // Toggle full speed with B button (default is 75%)
        joystick.b().onTrue(drivetrain.toggleSpeedMulti(1.0));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // Reset the field-centric heading on left bumper press.
        joystick.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        // D-pad up/down = manual intake up/down
        joystick.povUp().whileTrue(RobotCommands.windUp75()); // 75.125" wind up
        joystick.povDown().whileTrue(intake.slowRotateCommand(.025));
        // D-pad left/right = precise intake rotation at 1 RPM motor
        joystick.povLeft().whileTrue(intake.creepRotateCommand(-1));
        joystick.povRight().whileTrue(intake.creepRotateCommand(1));

        // Hold right bumper to auto-aim at target + spin up shooter (distance-based RPM)
        // When "Shooter At Speed" turns green, operator pulls trigger to fire instantly
        joystick.rightBumper().whileTrue(
            RobotCommands.aimAndWindUp(
                () -> -joystick.getLeftY() * MaxSpeed,
                () -> -joystick.getLeftX() * MaxSpeed
            )
        );

        // Hold Y to auto-aim pass: aims 15° inward from trench AprilTag + spins up pass RPM
        joystick.y().whileTrue(
            RobotCommands.aimAndPass(
                () -> -joystick.getLeftY() * MaxSpeed,
                () -> -joystick.getLeftX() * MaxSpeed,
                MaxSpeed
            )
        );

        // ===== Operator X3D Joystick =====
        // Button 1 = gated shoot: feeder only runs while RobotCommands.isReadyToShoot()
        operator.button(1).whileTrue(RobotCommands.gatedShoot());
        operator.button(1).onFalse(RobotCommands.redeployAfterShoot());
        // Button 11 = manual (ungated) shoot — the override if a gate sensor is lying.
        // Mirrors button 1's redeploy + slow-drive side effects so it feels identical.
        operator.button(11).whileTrue(RobotCommands.Shoot());
        operator.button(11).onFalse(RobotCommands.redeployAfterShoot());
        operator.button(11).whileTrue(drivetrain.holdSpeedMulti(1.0 / 5.0));
        operator.button(5).whileTrue(RobotCommands.manualWindUp());
        operator.button(9).whileTrue(RobotCommands.windUpClose()); // Close-range shot
        operator.button(2).whileTrue(intake.intakeWithOscillateCommand(IntakeSubsys.IntakeSpeed.INTAKE_TURBO));
        operator.button(2).whileTrue(drivetrain.holdSpeedMulti(0.75 / 2.0));
        operator.button(1).whileTrue(drivetrain.holdSpeedMulti(1.0 / 5.0));
        operator.button(12).whileTrue(intake.retractWithOscillateCommand(IntakeSubsys.IntakeSpeed.INTAKE_FAST));
        // Removed: was conflicting with Shoot() on intakeSubsys — Shoot() already bounces the intake
        // operator.button(1).whileTrue(intake.retractWithGentleOscillateCommand(IntakeSubsys.IntakeSpeed.INTAKE_FAST));
        operator.button(3).onTrue(RobotCommands.toggleShooterIdle()); // Toggle shooter idle on/off
        operator.button(7).whileTrue(RobotCommands.windUpCloser());//infront hub shot
        operator.button(6).onTrue(intake.goToPositionSlowCommand(-14.0, 0.3)); // Deploy intake
        operator.button(4).onTrue(intake.goToPositionSlowCommand(-0.14423828125, 0.2)); // Retract intake
        operator.button(10).whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(0)))); // Snap wheels forward
        operator.pov(180).whileTrue(RobotCommands.reverseAll()); // Hat down = eject jammed ball
        operator.pov(270).onTrue(RobotCommands.autoTuneExposure()); // Hat left = auto-tune LL exposure
        // Hat up/right = known-spot shots: re-seed odometry to a trench spot, then table wind-up
        operator.pov(0).whileTrue(RobotCommands.shootFromKnownSpot(Landmarks.KnownSpot.RIGHT_TRENCH));
        operator.pov(90).whileTrue(RobotCommands.shootFromKnownSpot(Landmarks.KnownSpot.LEFT_TRENCH));
        operator.button(8).whileTrue(RobotCommands.windUpPass());
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    // Maximum distance (meters) a vision update can jump from current pose before we reject it.
    // Prevents a single bad Limelight frame from corrupting the auto start position.
    private static final double kMaxVisionJumpMeters = 1.0;

    /**
     * Runs a single vision update cycle — reads the Limelight, and if a valid
     * measurement is available, feeds it into the drivetrain's Kalman filter.
     * Called from robotPeriodic() so it runs every cycle in all modes.
     */
    public void updateVision() {
        if (limelight == null) return;
        if (!SmartDashboard.getBoolean("Vision Enabled", true)) return;
        limelight.getMeasurement().ifPresent(measurement -> {
            drivetrain.addVisionMeasurement(
                measurement.poseEstimate.pose,
                measurement.poseEstimate.timestampSeconds,
                measurement.standardDeviations
            );
        });
    }

    /**
     * Seeds the drivetrain pose from Limelight vision while disabled.
     * Rejects measurements that jump more than 1 meter from the current estimate
     * to protect against bad frames corrupting the auto start position.
     */
    public void seedPoseFromVision() {
        if (limelight == null) return;
        final Pose2d currentPose = drivetrain.getState().Pose;
        limelight.getMeasurement().ifPresent(measurement -> {
            final double jump = currentPose.getTranslation()
                .getDistance(measurement.poseEstimate.pose.getTranslation());
            if (jump < kMaxVisionJumpMeters || currentPose.getTranslation().getNorm() < 0.01) {
                // Accept if jump is small, OR if current pose is near origin (uninitialized)
                drivetrain.resetPose(measurement.poseEstimate.pose);
            }
        });
    }
}
