// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/*
 * =========================================================================
 *                     FRC TEAM 6925 - ROBOT OVERVIEW
 * =========================================================================
 *
 * DRIVETRAIN (CommandSwerveDrivetrain)
 *   - Swerve drive using CTRE TunerX-generated constants
 *   - Field-centric control via Xbox controller (port 0)
 *   - Left trigger toggles half-speed mode (0.5x multiplier)
 *   - Left bumper reseeds field-centric heading (gyro reset)
 *   - Right bumper = auto-aim at target + distance-based shooter wind-up
 *
 * SHOOTER (ShooterSubsys) — 3 TalonFX motors
 *   - CAN 8  = leader motor (inverted — Clockwise_Positive)
 *   - CAN 9  = follower (opposed to leader)
 *   - CAN 10 = follower (opposed to leader)
 *   - Uses VelocityVoltage PID control (kP=0.5, kI=2.0, kV=0.12)
 *   - Current limits: 120A stator / 70A supply
 *   - Neutral mode: Coast (flywheel spins down naturally)
 *   - Default RPM: 3350 for fixed shots
 *   - Distance-adjusted RPM uses interpolation table in RobotCommands:
 *       47"   → 3350 RPM,  hood 0.1
 *       84"   → 3350 RPM,  hood 0.37
 *       120"  → 3350 RPM,  hood 0.45
 *       165"  → 3650 RPM,  hood 0.48  (WCP CC extended range)
 *   - "Shooter At Speed" = within 100 RPM of target
 *
 * HOOD (HoodSubsys) — 2 servos
 *   - PWM 0 = left servo,  PWM 1 = right servo
 *   - Position range: 0.01 (low) to 0.77 (high)
 *   - Adjusts shot angle; paired with shooter RPM via distance table
 *
 * FEEDER (FeederSubsys) — 2 TalonFX motors
 *   - CAN 51 = main feeder motor (feeds balls into shooter)
 *   - CAN 11 = fuel feed motor (on the shooter, pushes balls to flywheels)
 *   - Both controlled together via FeederSpeed enum:
 *       OFF        → 0.0  / 0.0
 *       FEED_SLOW  → -0.3 / 0.1
 *       FEED_FAST  → -0.5 / 0.5
 *       REVERSE    → 0.3  / -0.1
 *   - Current limits: 50A stator / 40A supply (both motors)
 *   - Neutral mode: Coast
 *   - IMPORTANT: Feeder is a SEPARATE subsystem from shooter so both
 *     can run simultaneously (button 1 = feed, button 2 = flywheels)
 *
 * INTAKE (IntakeSubsys) — 2 TalonFX motors
 *   - CAN 45 = intake roller (picks up balls from ground)
 *   - CAN 50 = intake rotator (pivots intake arm up/down)
 *   - Roller and rotator are controlled INDEPENDENTLY:
 *       Button 11 = intake with oscillate (fast)
 *       Button 12 = retract with oscillate (fast)
 *       Rotator uses PositionVoltage PID (kP=10) for angle control
 *   - Roller: 60A/60A, Coast mode
 *   - Rotator: 60A/60A, Brake mode (holds position when idle)
 *   - IntakeSpeed values are NEGATIVE (motor spins inward to grab balls)
 *
 * CLIMBER (ClimberSubsys) — 1 TalonFX motor
 *   - CAN 12 = climber motor
 *   - CLIMB_UP = 0.5 duty cycle, CLIMB_DOWN = -0.5
 *   - Current limits: 60A/60A, Brake mode
 *   - Used in auto L1 climb sequences and the "jolt" intake deploy
 *
 * LIMELIGHT (LimelightSubsys) — ENABLED
 *   - Limelight 3 camera for AprilTag vision
 *   - Uses MegaTag2 pose estimation with alliance-based tag filtering
 *   - Feeds pose estimates into drivetrain's Kalman filter
 *   - Camera: 1.46" behind center, 25.39" high, 20° above horizontal
 *
 * OPERATOR CONTROLS (X3D Joystick, port 1)
 *   Button 1  = Shoot (runs feeder motors — hold to feed balls)
 *   Button 2  = Intake with Oscillate (fast — hold)
 *   Button 3  = Climber Down (hold)
 *   Button 4  = Retract Intake (rotate to 580°)
 *   Button 5  = Climber Up (hold)
 *   Button 6  = Deploy Intake (rotate to -585°)
 *   Button 7  = Wind Up Closer (3350 RPM, hood 0.0 — hold)
 *   Button 8  = Hopper Release (climber up/down sequence — press once)
 *   Button 9  = Wind Up Close (3350 RPM, hood 0.3 — hold)
 *   Button 10 = Snap Wheels to 0° (hold)
 *   Button 11 = Wind Up Test (3350 RPM, hood 0.45 — hold)
 *   Button 12 = Retract with Oscillate (fast — hold)
 *   Hat Down  = Reverse All (eject jammed ball — intake + feeder backward)
 *
 * AUTONOMOUS
 *   - Uses PathPlanner with NamedCommands for event markers
 *   - Key named commands: shoot, StopFeed, windUp, windUpOnce,
 *     AdjustedWindUp, AdjustedShootWhileMoving, AdjustedWindUpOnce,
 *     IntakeMid, IntakeFast, StopIntake, ClimbUp, ClimbDown,
 *     StopClimber, jolt
 *   - "jolt" = raises climber, drives forward, brakes hard to deploy
 *     intake mechanically, then lowers climber back down
 *   - Robot is TOO TALL for trench — must use bump ramps to cross field
 *   - Shoot-while-moving uses predicted position (0.25s lookahead)
 *
 * MOTOR CAN IDs
 *   8  = Shooter leader (inverted)
 *   9  = Shooter follower
 *   10 = Shooter follower
 *   11 = Fuel feed (on shooter, controlled by FeederSubsys)
 *   12 = Climber
 *   45 = Intake roller
 *   50 = Intake rotator
 *   51 = Feeder
 *   (Swerve drive motors are defined in TunerConstants)
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

public class RobotContainer {
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(1.5).in(RadiansPerSecond); // 1.5 rotations per second max angular velocity

    // Slew rate limiters: 1.5/sec accel, 100/sec decel (instant stop)
    private final SlewRateLimiter xLimiter = new SlewRateLimiter(1.5, -100, 0);
    private final SlewRateLimiter yLimiter = new SlewRateLimiter(1.5, -100, 0);

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.125).withRotationalDeadband(MaxAngularRate * 0.075) // 30% translation, 15% rotation deadband
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

    public RobotContainer() {
        RobotCommands.init(shooter, feeder, hood, intake, drivetrain, limelight);
        configureBindings();
        // Register named commands for PathPlanner event markers
        // ── Shooting ──────────────────────────────────────────────────────────
        NamedCommands.registerCommand("shoot", RobotCommands.Shoot());
        NamedCommands.registerCommand("autoShoot", RobotCommands.autoShoot(4));
        NamedCommands.registerCommand("StopFeed", RobotCommands.stopFeed());
        // Fixed shot (no vision): set RPM/hood to hardcoded values
        NamedCommands.registerCommand("windUp", RobotCommands.windUp());
        NamedCommands.registerCommand("windUpOnce", RobotCommands.windUpOnce());
        // Auto wind-up commands: set RPM/hood, wait until at speed (max 2s), then finish
        NamedCommands.registerCommand("autoWindUp", RobotCommands.autoWindUp());
        NamedCommands.registerCommand("autoWindUpClose", RobotCommands.autoWindUpClose());
        NamedCommands.registerCommand("autoWindUpCloser", RobotCommands.autoWindUpCloser());
        // Distance-adjusted shot: interpolates RPM/hood from odometry distance
        // For the vision auto variant, pair this with accurate pose correction
        NamedCommands.registerCommand("AdjustedWindUp", RobotCommands.adjustedWindUp());
        // Moving shot: adjusts RPM/hood continuously + feeds; use as deadline alongside a path
        NamedCommands.registerCommand("AdjustedShootWhileMoving", RobotCommands.adjustedShootWhileMoving());
        // Static shot wind-up: snaps to distance-based RPM/hood then waits for spinup
        NamedCommands.registerCommand("AdjustedWindUpOnce", RobotCommands.adjustedWindUpOnce());
        // ── Intake ────────────────────────────────────────────────────────────
        NamedCommands.registerCommand("IntakeMid", RobotCommands.intakeMid());
        NamedCommands.registerCommand("IntakeFast", RobotCommands.intakeFast());
        NamedCommands.registerCommand("StopIntake", RobotCommands.stopIntake());
        NamedCommands.registerCommand("intakeDeploy", intake.goToPositionCommand(-14.5));
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
                double squaredY = -Math.copySign(leftY * leftY, leftY); // squared curve for translation
                double squaredX = -Math.copySign(leftX * leftX, leftX);
                double sqrtRot = -Math.copySign(Math.pow(Math.abs(rightX), 1.5), rightX); // x^1.5 curve for rotation
                double slewedY = xLimiter.calculate(squaredY);
                double slewedX = yLimiter.calculate(squaredX);
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
        joystick.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        ));

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
                () -> -joystick.getLeftX() * MaxSpeed,
                MaxSpeed
            )
        );

        // ===== Operator X3D Joystick =====
        operator.button(1).whileTrue(RobotCommands.Shoot());
       // operator.button(15).whileTrue(RobotCommands.windUp());
        operator.button(11).whileTrue(RobotCommands.windUpTest());
        operator.button(9).whileTrue(RobotCommands.windUpClose()); // Close-range shot
        operator.button(2).whileTrue(intake.intakeWithOscillateCommand(IntakeSubsys.IntakeSpeed.INTAKE_FAST));
        operator.button(2).whileTrue(drivetrain.holdSpeedMulti(1.0 / 2.0));
        operator.button(1).whileTrue(drivetrain.holdSpeedMulti(1.0 / 5.0));
        operator.button(12).whileTrue(intake.retractWithOscillateCommand(IntakeSubsys.IntakeSpeed.INTAKE_FAST));
        // Removed: was conflicting with Shoot() on intakeSubsys — Shoot() already bounces the intake
        // operator.button(1).whileTrue(intake.retractWithGentleOscillateCommand(IntakeSubsys.IntakeSpeed.INTAKE_FAST));
        operator.button(7).whileTrue(RobotCommands.windUpCloser());//infront hub shot
        operator.button(6).onTrue(intake.goToPositionSlowCommand(-14.20, 0.3)); // Deploy intake
        operator.button(4).onTrue(intake.goToPositionSlowCommand(-0.14423828125, 0.2)); // Retract intake
        operator.button(10).whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(0)))); // Snap wheels forward
        operator.pov(180).whileTrue(RobotCommands.reverseAll()); // Hat down = eject jammed ball
        operator.pov(270).onTrue(RobotCommands.autoTuneExposure()); // Hat left = auto-tune LL exposure
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
