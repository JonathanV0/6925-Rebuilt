package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static frc.robot.Constants.ShooterConstants.*;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.ClimberSubsys;
import frc.robot.subsystems.ClimberSubsys.ClimberSpeed;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.FeederSubsys;
import frc.robot.subsystems.FeederSubsys.FeederSpeed;
import frc.robot.subsystems.HoodSubsys;
import frc.robot.subsystems.IntakeSubsys;
import frc.robot.subsystems.IntakeSubsys.IntakeSpeed;
import frc.robot.subsystems.LimelightSubsys;
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
        // Team-calibrated data points
        distanceToShotMap.put(Inches.of(47.0), new Shot(kRPMAt47in, kHoodAt47in));
        distanceToShotMap.put(Inches.of(84.0), new Shot(kRPMAt84in, kHoodAt84in));
        distanceToShotMap.put(Inches.of(120.0), new Shot(kRPMAt120in, kHoodAt120in));
        // WCP CC extended range
        distanceToShotMap.put(Inches.of(165.5), new Shot(kRPMAt165in, kHoodAt165in));
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
            shooterSubsys.setVelocityRPM(kFixedShotRPM);
            hoodSubsys.setPosition(kDefaultHoodPosition);
        }, shooterSubsys, hoodSubsys);
    }

    /** Auto wind up (default): delays 0.25s to avoid brownout at auto start,
     *  sets RPM/hood, waits until at speed (max 2s), then finishes. */
    public static Command autoWindUp() {
        return Commands.waitSeconds(0.25).andThen(
            Commands.runOnce(() -> {
                shooterSubsys.setVelocityRPM(kFixedShotRPM);
                hoodSubsys.setPosition(kDefaultHoodPosition);
            }, shooterSubsys, hoodSubsys),
            Commands.waitUntil(shooterSubsys::isVelocityWithinTolerance).withTimeout(2.0)
        );
    }

    /** Auto wind up (close): delays 0.25s, sets RPM/hood close, waits until at speed (max 2s). */
    public static Command autoWindUpClose() {
        return Commands.waitSeconds(0.25).andThen(
            Commands.runOnce(() -> {
                shooterSubsys.setVelocityRPM(kFixedShotRPM);
                hoodSubsys.setPosition(kCloseHoodPosition);
            }, shooterSubsys, hoodSubsys),
            Commands.waitUntil(shooterSubsys::isVelocityWithinTolerance).withTimeout(2.0)
        );
    }

    /** Auto wind up (closer/hub): delays 0.25s, sets RPM/hood closer, waits until at speed (max 2s). */
    public static Command autoWindUpCloser() {
        return Commands.waitSeconds(0.25).andThen(
            Commands.runOnce(() -> {
                shooterSubsys.setVelocityRPM(kFixedShotRPM);
                hoodSubsys.setPosition(kCloserHoodPosition);
            }, shooterSubsys, hoodSubsys),
            Commands.waitUntil(shooterSubsys::isVelocityWithinTolerance).withTimeout(2.0)
        );
    }

    /** Holds RPM/hood while button is held, coasts on release — for teleop */
    public static Command windUp() {
        return Commands.runEnd(
            () -> {
                shooterSubsys.setVelocityRPM(kFixedShotRPM);
                hoodSubsys.setPosition(kDefaultHoodPosition);
            },
            () -> shooterSubsys.stopShooter(),
            shooterSubsys, hoodSubsys
        );
    }

    /** Close-range wind up: same RPM, lower hood — for teleop */
    public static Command windUpClose() {
        return Commands.runEnd(
            () -> {
                shooterSubsys.setVelocityRPM(kFixedShotRPM);
                hoodSubsys.setPosition(kCloseHoodPosition);
            },
            () -> shooterSubsys.stopShooter(),
            shooterSubsys, hoodSubsys
        );
    }

     public static Command windUpCloser() {
        return Commands.runEnd(
            () -> {
                shooterSubsys.setVelocityRPM(kFixedShotRPM);
                hoodSubsys.setPosition(kCloserHoodPosition);
            },
            () -> shooterSubsys.stopShooter(),
            shooterSubsys, hoodSubsys
        );
    }

    public static Command windUpTest() {
        return Commands.runEnd(
            () -> {
                shooterSubsys.setVelocityRPM(kFixedShotRPM);
                hoodSubsys.setPosition(kTestHoodPosition);
            },
            () -> shooterSubsys.stopShooter(),
            shooterSubsys, hoodSubsys
        );
    }

    /** Wind up + feed: spins flywheels AND runs both feeders while held, stops everything on release */
    public static Command windUpAndShoot() {
        return Commands.runEnd(
            () -> {
                shooterSubsys.setVelocityRPM(kFixedShotRPM);
                hoodSubsys.setPosition(kDefaultHoodPosition);
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
            () -> {
                if (shooterSubsys.isReadyToFeed()) {
                    feederSubsys.setSpeed(FeederSpeed.FEED_FAST);
                }
            },
            () -> feederSubsys.setSpeed(FeederSpeed.OFF),
            feederSubsys
        );
    }

    /** Timed auto shoot: waits for shooter to be ready, runs feeders for the given duration, then stops. */
    public static Command autoShoot(double seconds) {
        return Commands.sequence(
            Commands.waitUntil(shooterSubsys::isReadyToFeed).withTimeout(1.0),
            Commands.run(() -> feederSubsys.setSpeed(FeederSpeed.FEED_FAST), feederSubsys)
                .withTimeout(seconds),
            Commands.runOnce(() -> feederSubsys.setSpeed(FeederSpeed.OFF), feederSubsys)
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
        return Commands.runEnd(
            () -> {
                intakeSubsys.setSpeed(IntakeSpeed.REVERSE);
                feederSubsys.setSpeed(FeederSpeed.REVERSE);
                shooterSubsys.setVelocityRPM(-500);
            },
            () -> {
                intakeSubsys.setSpeed(IntakeSpeed.OFF);
                feederSubsys.setSpeed(FeederSpeed.OFF);
                shooterSubsys.setVelocityRPM(0);
            },
            intakeSubsys, feederSubsys, shooterSubsys
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
        final SwerveRequest.FieldCentric aimDrive = new SwerveRequest.FieldCentric()
            .withDeadband(maxSpeed * 0.1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

        // Only aim at hub tags (2, 5, 10)
        final int[] hubTagIDs = {2, 5, 10, 26, 18, 21};

        // Single run loop: aim, adjust RPM/hood, and drive all update together each cycle
        return Commands.runEnd(() -> {
            LimelightHelpers.SetFiducialIDFiltersOverride("limelight", hubTagIDs);
            // 1. Aim at target using Limelight tx
            final double tx = LimelightHelpers.getTV("limelight")
                ? LimelightHelpers.getTX("limelight") + kAimOffsetDegrees : 0.0;
            drivetrain.setControl(aimDrive
                .withVelocityX(velocityX.getAsDouble())
                .withVelocityY(velocityY.getAsDouble())
                .withRotationalRate(-tx * kAimP));

            // 2. Adjust RPM and hood using Limelight distance (ty), fallback to odometry
            final Distance distance;
            if (LimelightHelpers.getTV("limelight")) {
                final double ty = LimelightHelpers.getTY("limelight");
                final double heightDiff = LimelightSubsys.kTargetHeightInches - LimelightSubsys.kCameraHeightInches;
                final double angleRad = Math.toRadians(LimelightSubsys.kCameraMountAngleDegrees + ty);
                final double distInches = heightDiff / Math.tan(angleRad) + kHubCenterOffsetInches;
                distance = Inches.of(distInches);
            } else {
                distance = getPredictedDistanceToTarget();
            }
            final Shot shot = distanceToShotMap.get(distance);
            shooterSubsys.setVelocityRPM(shot.shooterRPM);
            hoodSubsys.setPosition(shot.hoodPosition);
            SmartDashboard.putNumber("Auto Distance (inches)", distance.in(Inches));
        }, () -> {
            // On release: coast shooter and reset hood
            shooterSubsys.stopShooter();
            hoodSubsys.setPosition(0);
        }, drivetrain, shooterSubsys, hoodSubsys);
    }

    // ========== Range-Adjusted Shot Commands ==========

    // How far ahead (seconds) to predict robot position for shot calculations.
    // Accounts for shooter spinup + ball flight time.

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
     * Delays 0.25s to avoid brownout, snaps RPM and hood to distance-table values
     * from current robot position, then blocks until the shooter reaches target RPM (±100 RPM).
     * Times out after 2 seconds to prevent auto deadlock on CAN dropout or brownout.
     * Use in sequential autos before calling shoot().
     */
    public static Command adjustedWindUpOnce() {
        return Commands.waitSeconds(0.25).andThen(
            Commands.runOnce(() -> {
                final Distance distance = getDistanceToTarget();
                final Shot shot = distanceToShotMap.get(distance);
                shooterSubsys.setVelocityRPM(shot.shooterRPM);
                hoodSubsys.setPosition(shot.hoodPosition);
            }, shooterSubsys, hoodSubsys),
            Commands.waitUntil(shooterSubsys::isVelocityWithinTolerance).withTimeout(2.0)
        );
    }

    // ========== Hopper Release ==========

    /** Raises the climber briefly then lowers it back down to release the hopper. */
    public static Command hopperRelease() {
        return Commands.sequence(
            climberSubsys.setSpeedCommand(ClimberSpeed.CLIMB_UP),
            Commands.waitSeconds(0.5),
            climberSubsys.setSpeedCommand(ClimberSpeed.CLIMB_DOWN),
            Commands.waitSeconds(0.4),
            climberSubsys.setSpeedCommand(ClimberSpeed.OFF)
        );
    }

    // ========== Intake Jolt ==========

    /** Deploys the hopper then the intake. Replaces the old drive-and-brake jolt. */
    public static Command jolt() {
        return Commands.sequence(
            hopperRelease(),
            intakeSubsys.goToPositionCommand(-14.5)
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
