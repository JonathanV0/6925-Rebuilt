package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ClimberSubsys;
import frc.robot.subsystems.ClimberSubsys.ClimberSpeed;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.FeederSubsys;
import frc.robot.subsystems.FeederSubsys.FeederSpeed;
import frc.robot.subsystems.HoodSubsys;
import frc.robot.subsystems.IntakeSubsys;
import frc.robot.subsystems.IntakeSubsys.IntakeSpeed;
import frc.robot.subsystems.ShooterSubsys;

public final class RobotCommands {
    private static ShooterSubsys shooterSubsys;
    private static FeederSubsys feederSubsys;
    private static HoodSubsys hoodSubsys;
    private static IntakeSubsys intakeSubsys;
    private static CommandSwerveDrivetrain drivetrain;
    private static ClimberSubsys climberSubsys;

    // Distance-to-shot lookup table (team should calibrate these values)
    private static final InterpolatingTreeMap<Distance, Shot> distanceToShotMap = new InterpolatingTreeMap<>(
        (startValue, endValue, q) ->
            InverseInterpolator.forDouble()
                .inverseInterpolate(startValue.in(Meters), endValue.in(Meters), q.in(Meters)),
        (startValue, endValue, t) ->
            new Shot(
                Interpolator.forDouble().interpolate(startValue.shooterRPM, endValue.shooterRPM, t),
                Interpolator.forDouble().interpolate(startValue.hoodPosition, endValue.hoodPosition, t)
            )
    );

    static {
        // TODO: Calibrate these values on the real robot
        distanceToShotMap.put(Inches.of(52.0), new Shot(2800, 0.19));
        distanceToShotMap.put(Inches.of(114.4), new Shot(3275, 0.40));
        distanceToShotMap.put(Inches.of(165.5), new Shot(3650, 0.48));
    }

    public static void init(
        ShooterSubsys shooter,
        FeederSubsys feeder,
        HoodSubsys hood,
        IntakeSubsys intake,
        CommandSwerveDrivetrain drive,
        ClimberSubsys climber
    ) {
        RobotCommands.shooterSubsys = shooter;
        RobotCommands.feederSubsys = feeder;
        RobotCommands.hoodSubsys = hood;
        RobotCommands.intakeSubsys = intake;
        RobotCommands.drivetrain = drive;
        RobotCommands.climberSubsys = climber;
    }

    // ========== Fixed Shot Commands ==========

    /** Sets RPM/hood once and finishes — for use in auto sequences */
    public static Command windUpOnce() {
        return Commands.runOnce(() -> {
            shooterSubsys.setVelocityRPM(4500);
            hoodSubsys.setPosition(0.4);
        }, shooterSubsys, hoodSubsys);
    }

    /** Holds RPM/hood while button is held, coasts on release — for teleop */
    public static Command windUp() {
        return Commands.runEnd(
            () -> {
                shooterSubsys.setVelocityRPM(4500);
                hoodSubsys.setPosition(0.4);
            },
            () -> shooterSubsys.stopShooter(),
            shooterSubsys, hoodSubsys
        );
    }

    /** Wind up + feed: spins flywheels AND runs both feeders while held, stops everything on release */
    public static Command windUpAndShoot() {
        return Commands.runEnd(
            () -> {
                shooterSubsys.setVelocityRPM(4500);
                hoodSubsys.setPosition(0.4);
                feederSubsys.setSpeed(FeederSpeed.FEED_FAST);
            },
            () -> {
                shooterSubsys.stopShooter();
                feederSubsys.setSpeed(FeederSpeed.OFF);
            },
            shooterSubsys, hoodSubsys, feederSubsys
        );
    }

    public static Command Shoot() {
        return Commands.runEnd(
            () -> feederSubsys.setSpeed(FeederSpeed.FEED_FAST),
            () -> feederSubsys.setSpeed(FeederSpeed.OFF),
            feederSubsys
        );
    }

    public static Command stopFeed() {
        return feederSubsys.setSpeedCommand(FeederSpeed.OFF);
    }

    // ========== Intake Commands ==========

    public static Command intakeMid() {
        return intakeSubsys.setSpeedCommand(IntakeSpeed.INTAKE_MID);
    }

    public static Command intakeFast() {
        return intakeSubsys.setSpeedCommand(IntakeSpeed.INTAKE_FAST);
    }

    public static Command stopIntake() {
        return intakeSubsys.setSpeedCommand(IntakeSpeed.OFF);
    }

    public static Command reverseAll() {
        return new ParallelCommandGroup(
            intakeSubsys.setSpeedCommand(IntakeSpeed.REVERSE),
            feederSubsys.setSpeedCommand(FeederSpeed.REVERSE)
        );
    }

    // ========== Teleop Aim + Wind-Up Combo ==========

    /**
     * One-button teleop shot prep: auto-aims at the target while the driver drives,
     * AND continuously adjusts shooter RPM/hood based on distance.
     * Hold this, then pull the trigger (Shoot) when "Shooter At Speed" is green.
     * The robot is already aimed and spun up — zero wait time on the shot.
     */
    public static Command aimAndWindUp(DoubleSupplier velocityX, DoubleSupplier velocityY, double maxSpeed) {
        final SwerveRequest.FieldCentricFacingAngle aimDrive = new SwerveRequest.FieldCentricFacingAngle()
            .withDeadband(maxSpeed * 0.1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

        aimDrive.HeadingController.setPID(8, 0, 0);
        aimDrive.HeadingController.enableContinuousInput(-Math.PI, Math.PI);

        return Commands.parallel(
            // Aim at target while driving
            drivetrain.applyRequest(() -> {
                final Translation2d robotPos = drivetrain.getState().Pose.getTranslation();
                final Translation2d targetPos = Landmarks.targetPosition();
                final Rotation2d angleToTarget = targetPos.minus(robotPos).getAngle();
                return aimDrive
                    .withVelocityX(velocityX.getAsDouble())
                    .withVelocityY(velocityY.getAsDouble())
                    .withTargetDirection(angleToTarget);
            }),
            // Continuously adjust RPM and hood based on predicted distance
            Commands.run(() -> {
                final Distance distance = getPredictedDistanceToTarget();
                final Shot shot = distanceToShotMap.get(distance);
                shooterSubsys.setVelocityRPM(shot.shooterRPM);
                hoodSubsys.setPosition(shot.hoodPosition);
            }, shooterSubsys, hoodSubsys)
        );
    }

    // ========== Range-Adjusted Shot Commands ==========

    // How far ahead (seconds) to predict robot position for shot calculations.
    // Accounts for shooter spinup + ball flight time.
    private static final double kLookAheadSeconds = 0.25;

    private static Distance getDistanceToTarget() {
        final Translation2d robotPosition = drivetrain.getState().Pose.getTranslation();
        final Translation2d targetPosition = Landmarks.targetPosition();
        return Meters.of(robotPosition.getDistance(targetPosition));
    }

    /**
     * Predicts where the robot will be in kLookAheadSeconds based on current velocity,
     * then returns the distance from that future position to the target.
     * More accurate than current-position distance when shooting while moving.
     */
    private static Distance getPredictedDistanceToTarget() {
        final Pose2d currentPose = drivetrain.getState().Pose;
        final ChassisSpeeds fieldSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(
            drivetrain.getState().Speeds, currentPose.getRotation());
        // Predict future position: current + velocity * time
        final Translation2d futurePosition = currentPose.getTranslation().plus(
            new Translation2d(
                fieldSpeeds.vxMetersPerSecond * kLookAheadSeconds,
                fieldSpeeds.vyMetersPerSecond * kLookAheadSeconds
            )
        );
        final Translation2d targetPosition = Landmarks.targetPosition();
        return Meters.of(futurePosition.getDistance(targetPosition));
    }

    public static Command adjustedWindUp() {
        return Commands.run(() -> {
            final Distance distance = getPredictedDistanceToTarget();
            final Shot shot = distanceToShotMap.get(distance);
            shooterSubsys.setVelocityRPM(shot.shooterRPM);
            hoodSubsys.setPosition(shot.hoodPosition);
            SmartDashboard.putNumber("Distance to Target (inches)", distance.in(Inches));
            SmartDashboard.putNumber("Target RPM", shot.shooterRPM);
            SmartDashboard.putNumber("Target Hood Position", shot.hoodPosition);
        }, shooterSubsys, hoodSubsys);
    }


    // ========== Moving Shot Commands (for backing-up auto) ==========

    /**
     * Winds up the shooter using the distance interpolation table, waits until at speed,
     * then continues adjusting RPM/hood AND runs both feeders simultaneously.
     * Designed for use inside a PathPlanner deadline group alongside a drive path —
     * the path ending cancels this command; call StopFeed after.
     */
    public static Command adjustedShootWhileMoving() {
        return Commands.sequence(
            // Phase 1: spin up to predicted-distance RPM, wait until at speed (max 2s to prevent deadlock)
            Commands.run(() -> {
                final Distance distance = getPredictedDistanceToTarget();
                final Shot shot = distanceToShotMap.get(distance);
                shooterSubsys.setVelocityRPM(shot.shooterRPM);
                hoodSubsys.setPosition(shot.hoodPosition);
            }, shooterSubsys, hoodSubsys)
            .until(shooterSubsys::isVelocityWithinTolerance)
            .withTimeout(2.0),
            // Phase 2: maintain RPM/hood AND run both feeders to shoot while still moving
            Commands.run(() -> {
                final Distance distance = getPredictedDistanceToTarget();
                final Shot shot = distanceToShotMap.get(distance);
                shooterSubsys.setVelocityRPM(shot.shooterRPM);
                hoodSubsys.setPosition(shot.hoodPosition);
                feederSubsys.setSpeed(FeederSpeed.FEED_FAST);
            }, shooterSubsys, hoodSubsys, feederSubsys)
        );
    }

    /**
     * Snaps RPM and hood to distance-table values once from current robot position,
     * then blocks until the shooter reaches target RPM (±100 RPM).
     * Times out after 2 seconds to prevent auto deadlock on CAN dropout or brownout.
     * Use in sequential autos before calling shoot().
     */
    public static Command adjustedWindUpOnce() {
        return Commands.runOnce(() -> {
            final Distance distance = getDistanceToTarget();
            final Shot shot = distanceToShotMap.get(distance);
            shooterSubsys.setVelocityRPM(shot.shooterRPM);
            hoodSubsys.setPosition(shot.hoodPosition);
        }, shooterSubsys, hoodSubsys)
        .andThen(Commands.waitUntil(shooterSubsys::isVelocityWithinTolerance).withTimeout(2.0));
    }

    // ========== Intake Jolt ==========

    /**
     * Raises the climbers, drives forward briefly, then brakes hard to jolt the intake open,
     * then lowers the climbers back down. Use at the start of an auto to deploy the intake.
     */
    public static Command jolt() {
        final SwerveRequest.RobotCentric joltDrive = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
        final SwerveRequest.SwerveDriveBrake hardBrake = new SwerveRequest.SwerveDriveBrake();

        return Commands.sequence(
            climberSubsys.setSpeedCommand(ClimberSpeed.CLIMB_UP),
            Commands.waitSeconds(0.5),
            climberSubsys.setSpeedCommand(ClimberSpeed.OFF),
            intakeSubsys.rotateRotatorCommand(90), // 90° CCW
            drivetrain.applyRequest(() -> joltDrive.withVelocityX(2.5)).withTimeout(0.35),
            drivetrain.applyRequest(() -> hardBrake).withTimeout(0.15),
            climberSubsys.setSpeedCommand(ClimberSpeed.CLIMB_DOWN),
            Commands.waitSeconds(0.4),
            climberSubsys.setSpeedCommand(ClimberSpeed.OFF)
        );
    }

    // ========== Shot Data ==========

    public static class Shot {
        public final double shooterRPM;
        public final double hoodPosition;

        public Shot(double shooterRPM, double hoodPosition) {
            this.shooterRPM = shooterRPM;
            this.hoodPosition = hoodPosition;
        }
    }
}
