// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/*
 * ╔══════════════════════════════════════════════════════════════════════════╗
 * ║                    FRC TEAM 6925 — ROBOT OVERVIEW                      ║
 * ╠══════════════════════════════════════════════════════════════════════════╣
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
 *   - Default RPM: 4500 for fixed shots
 *   - Distance-adjusted RPM uses interpolation table in RobotCommands:
 *       52"  → 2800 RPM,  hood 0.19
 *       114" → 3275 RPM,  hood 0.40
 *       165" → 3650 RPM,  hood 0.48
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
 *       Button 11 = run roller at mid speed (-0.25)
 *       Button 12 = stop roller
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
 * LIMELIGHT (LimelightSubsys) — currently DISABLED (set to null)
 *   - Limelight 2 camera for AprilTag vision
 *   - Uses MegaTag1 pose estimation with alliance-based tag filtering
 *   - Feeds pose estimates into drivetrain's Kalman filter
 *   - Camera: 1.46" behind center, 25.39" high, 20° above horizontal
 *   - Needs radio fixed before re-enabling
 *
 * OPERATOR CONTROLS (X3D Joystick, port 1)
 *   Button 1  = Shoot (runs feeder motors — hold to feed balls)
 *   Button 2  = Wind Up (spins flywheels to 4500 RPM + sets hood — hold)
 *   Button 3  = Reverse All (eject jammed ball — intake + feeder backward)
 *   Button 7  = Test right motor only at 4500 RPM (diagnostic)
 *   Button 9  = Stop Feed
 *   Button 10 = Adjusted Wind Up (distance-based RPM/hood — hold)
 *   Button 11 = Intake Mid speed
 *   Button 12 = Stop Intake
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
 * ╚══════════════════════════════════════════════════════════════════════════╝
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
import frc.robot.subsystems.ClimberSubsys;
import frc.robot.subsystems.ClimberSubsys.ClimberSpeed;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.FeederSubsys;
import frc.robot.subsystems.IntakeSubsys;
import frc.robot.subsystems.HoodSubsys;
import frc.robot.subsystems.LimelightSubsys;
import frc.robot.subsystems.ShooterSubsys;

public class RobotContainer {
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(1.5).in(RadiansPerSecond); // 1.5 rotations per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
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
    private final ClimberSubsys climber = new ClimberSubsys();
    private final HoodSubsys hood = new HoodSubsys();
    // TODO: Re-enable when Limelight is connected and radio is configured
    private final LimelightSubsys limelight = null; // new LimelightSubsys("limelight");

    public RobotContainer() {
        RobotCommands.init(shooter, feeder, hood, intake, drivetrain, climber);
        configureBindings();
        // Register named commands for PathPlanner event markers
        // ── Shooting ──────────────────────────────────────────────────────────
        NamedCommands.registerCommand("shoot", RobotCommands.Shoot());
        NamedCommands.registerCommand("StopFeed", RobotCommands.stopFeed());
        // Fixed shot (no vision): set RPM/hood to hardcoded values
        NamedCommands.registerCommand("windUp", RobotCommands.windUp());
        NamedCommands.registerCommand("windUpOnce", RobotCommands.windUpOnce());
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
        // Vision updates now run automatically in robotPeriodic() — no named command needed
        // ── Climber (for L1 auto climb) ───────────────────────────────────────
        NamedCommands.registerCommand("jolt", RobotCommands.jolt());
        NamedCommands.registerCommand("ClimbUp", climber.setSpeedCommand(ClimberSpeed.CLIMB_UP));
        NamedCommands.registerCommand("ClimbDown", climber.setSpeedCommand(ClimberSpeed.CLIMB_DOWN));
        NamedCommands.registerCommand("StopClimber", climber.setSpeedCommand(ClimberSpeed.OFF));

        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed * drivetrain.getCurrentSpeedMulti()) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed * drivetrain.getCurrentSpeedMulti()) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        // Toggle half speed with left trigger
        joystick.leftTrigger().onTrue(drivetrain.toggleSpeedMulti(0.5));

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
        operator.button(2).whileTrue(RobotCommands.windUp());
        operator.button(9).onTrue(RobotCommands.stopFeed());
        operator.button(10).whileTrue(RobotCommands.adjustedWindUp());
        operator.button(11).whileTrue(RobotCommands.intakeMid());
        operator.button(12).onTrue(RobotCommands.stopIntake());
        operator.button(3).whileTrue(RobotCommands.reverseAll()); // Eject jammed ball
        operator.button(7).whileTrue(Commands.runEnd(
            () -> shooter.setRightMotorOnly(4500),
            () -> { shooter.stopShooter(); shooter.reEnableFollowers(); },
            shooter
        )); // Test right motor only

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
        final Pose2d currentPose = drivetrain.getState().Pose;
        limelight.getMeasurement(currentPose).ifPresent(measurement -> {
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
        limelight.getMeasurement(currentPose).ifPresent(measurement -> {
            final double jump = currentPose.getTranslation()
                .getDistance(measurement.poseEstimate.pose.getTranslation());
            if (jump < kMaxVisionJumpMeters || currentPose.getTranslation().getNorm() < 0.01) {
                // Accept if jump is small, OR if current pose is near origin (uninitialized)
                drivetrain.resetPose(measurement.poseEstimate.pose);
            }
        });
    }
}
