package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.FeederSubsys;
import frc.robot.subsystems.FeederSubsys.FeederSpeed;
import frc.robot.subsystems.HoodSubsys;
// import frc.robot.subsystems.LimelightSubsys;
import frc.robot.subsystems.IntakeSubsys;
import frc.robot.subsystems.IntakeSubsys.IntakeSpeed;
import frc.robot.subsystems.ShooterSubsys;
import frc.robot.subsystems.ShooterSubsys.FuelFeedSpeed;

public final class RobotCommands {
    private static ShooterSubsys shooterSubsys;
    private static FeederSubsys feederSubsys;
    private static HoodSubsys hoodSubsys;
    private static IntakeSubsys intakeSubsys;
    // private static LimelightSubsys limelightSubsys;
    private static CommandSwerveDrivetrain drivetrain;

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
        // LimelightSubsys limelight,
        CommandSwerveDrivetrain drive
    ) {
        RobotCommands.shooterSubsys = shooter;
        RobotCommands.feederSubsys = feeder;
        RobotCommands.hoodSubsys = hood;
        RobotCommands.intakeSubsys = intake;
        // RobotCommands.limelightSubsys = limelight;
        RobotCommands.drivetrain = drive;
    }

    // ========== Fixed Shot Commands ==========

    public static Command windUp() {
        return new SequentialCommandGroup(
            shooterSubsys.setVelocityRPMCommand(3000),
            hoodSubsys.positionCommand(0.4)
        );
    }

    public static Command Shoot() {
        return new ParallelCommandGroup(
            feederSubsys.setSpeedCommand(FeederSpeed.FEED_FAST),
            shooterSubsys.setFeedSpeedCommand(FuelFeedSpeed.FEED_FAST)
        );
    }

    public static Command stopFeed() {
        return new ParallelCommandGroup(
            feederSubsys.setSpeedCommand(FeederSpeed.OFF),
            shooterSubsys.setFeedSpeedCommand(FuelFeedSpeed.OFF)
        );
    }

    // ========== Intake Commands ==========

    public static Command intakeSlow() {
        return intakeSubsys.setSpeedCommand(IntakeSpeed.INTAKE_SLOW);
    }

    public static Command intakeMid() {
        return intakeSubsys.setSpeedCommand(IntakeSpeed.INTAKE_MID);
    }

    public static Command stopIntake() {
        return intakeSubsys.setSpeedCommand(IntakeSpeed.OFF);
    }

    public static Command setHood(double position) {
        return hoodSubsys.positionCommand(position);
    }

    // // ========== Vision Commands ==========

    // public static Command updateVision() {
    //     return Commands.run(() -> {
    //         final Pose2d currentPose = drivetrain.getState().Pose;
    //         limelightSubsys.getMeasurement(currentPose).ifPresent(measurement -> {
    //             drivetrain.addVisionMeasurement(
    //                 measurement.poseEstimate.pose,
    //                 measurement.poseEstimate.timestampSeconds,
    //                 measurement.standardDeviations
    //             );
    //         });
    //     }, limelightSubsys);
    // }

    // ========== Aim at Target ==========

    public static Command aimAtTarget(DoubleSupplier velocityX, DoubleSupplier velocityY, double maxSpeed) {
        final SwerveRequest.FieldCentricFacingAngle aimDrive = new SwerveRequest.FieldCentricFacingAngle()
            .withDeadband(maxSpeed * 0.1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

        // PID controls how the robot rotates to face the target:
        // P=8: rotational speed scales with heading error (tune on robot)
        // I=0: no integral correction needed for heading
        // D=0: start without dampening, increase if robot oscillates
        aimDrive.HeadingController.setPID(8, 0, 0);
        aimDrive.HeadingController.enableContinuousInput(-Math.PI, Math.PI);

        return drivetrain.applyRequest(() -> {
            final Translation2d robotPos = drivetrain.getState().Pose.getTranslation();
            final Translation2d targetPos = Landmarks.targetPosition();
            final Rotation2d angleToTarget = targetPos.minus(robotPos).getAngle();
            return aimDrive
                .withVelocityX(velocityX.getAsDouble())
                .withVelocityY(velocityY.getAsDouble())
                .withTargetDirection(angleToTarget);
        });
    }

    // ========== Range-Adjusted Shot Commands ==========

    private static Distance getDistanceToTarget() {
        final Translation2d robotPosition = drivetrain.getState().Pose.getTranslation();
        final Translation2d targetPosition = Landmarks.targetPosition();
        return Meters.of(robotPosition.getDistance(targetPosition));
    }

    public static Command adjustedWindUp() {
        return Commands.run(() -> {
            final Distance distance = getDistanceToTarget();
            final Shot shot = distanceToShotMap.get(distance);
            shooterSubsys.setVelocityRPM(shot.shooterRPM);
            hoodSubsys.setPosition(shot.hoodPosition);
            SmartDashboard.putNumber("Distance to Target (inches)", distance.in(Inches));
            SmartDashboard.putNumber("Target RPM", shot.shooterRPM);
            SmartDashboard.putNumber("Target Hood Position", shot.hoodPosition);
        }, shooterSubsys, hoodSubsys);
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
