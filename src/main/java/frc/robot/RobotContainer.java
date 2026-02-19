// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.lib.util.CommandX3DController;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.ClimberSubsys;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.FeederSubsys;
import frc.robot.subsystems.IntakeSubsys;
import frc.robot.subsystems.HoodSubsys;
import frc.robot.subsystems.LimelightSubsys;
import frc.robot.subsystems.ShooterSubsys;

public class RobotContainer {
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

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
    // private final LimelightSubsys limelight = new LimelightSubsys("limelight");

    public RobotContainer() {
        RobotCommands.init(shooter, feeder, hood, intake, /* limelight, */ drivetrain);
        configureBindings();
        // Register named commands for PathPlanner event markers
        // ── Shooting ──────────────────────────────────────────────────────────
        NamedCommands.registerCommand("Shoot", RobotCommands.Shoot());
        NamedCommands.registerCommand("StopFeed", RobotCommands.stopFeed());
        // Fixed shot (no vision): set RPM/hood to hardcoded values
        NamedCommands.registerCommand("WindUp", RobotCommands.windUp());
        // Distance-adjusted shot: interpolates RPM/hood from odometry distance
        // For the vision auto variant, pair this with accurate pose correction
        NamedCommands.registerCommand("AdjustedWindUp", RobotCommands.adjustedWindUp());
        // ── Intake ────────────────────────────────────────────────────────────
        NamedCommands.registerCommand("IntakeMid", RobotCommands.intakeMid());
        NamedCommands.registerCommand("StopIntake", RobotCommands.stopIntake());
        // ── Vision (enable when limelight is re-enabled) ──────────────────────
        // NamedCommands.registerCommand("VisionUpdate", RobotCommands.updateVision());

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

        // Hold right bumper to auto-aim at target while still driving
        joystick.rightBumper().whileTrue(
            RobotCommands.aimAtTarget(
                () -> -joystick.getLeftY() * MaxSpeed,
                () -> -joystick.getLeftX() * MaxSpeed,
                MaxSpeed
            )
        );

        // ===== Operator X3D Joystick =====
        // TODO: Remap these buttons once joystick layout is decided
        operator.button(1).whileTrue(RobotCommands.Shoot());
        operator.button(2).whileTrue(RobotCommands.windUp());
        operator.button(0).onTrue(RobotCommands.stopFeed());
        operator.button(0).whileTrue(RobotCommands.adjustedWindUp());
        operator.button(0).whileTrue(RobotCommands.intakeMid());
        operator.button(0).onTrue(RobotCommands.stopIntake());

        // // Limelight vision updates run continuously as default command
        // limelight.setDefaultCommand(RobotCommands.updateVision());

    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
