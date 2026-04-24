package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static frc.robot.Constants.ShooterConstants.*;

import java.util.Optional;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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
    private static LimelightSubsys limelightSubsys;

    private static final AprilTagFieldLayout aprilTagFieldLayout =
        AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

    private static final double kAimOffsetDegrees = 0.0;
    private static final double BALL_VELOCITY_MS = 8.0;

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
        distanceToShotMap.put(Inches.of(47.0), new Shot(kFixedShotRPM + 150, kHoodAt47in));
        distanceToShotMap.put(Inches.of(50.0), new Shot(kRPMAt50in + 150, kHoodAt50in));
        distanceToShotMap.put(Inches.of(75.125), new Shot(kRPMAt75in + 150, kHoodAt75in));
        distanceToShotMap.put(Inches.of(84.0), new Shot(kFixedShotRPM + 150, kHoodAt84in));
        distanceToShotMap.put(Inches.of(92.0), new Shot(kRPMAt92in + 150, kHoodAt92in));
        distanceToShotMap.put(Inches.of(100.0), new Shot(kRPMAt100in + 150, kHoodAt100in));
        distanceToShotMap.put(Inches.of(110.0), new Shot(kRPMAt110in + 150, kHoodAt110in));
    }

    public static void init(
        ShooterSubsys shooter,
        FeederSubsys feeder,
        HoodSubsys hood,
        IntakeSubsys intake,
        CommandSwerveDrivetrain drive,
        LimelightSubsys limelight
    ) {
        RobotCommands.shooterSubsys = shooter;
        RobotCommands.feederSubsys = feeder;
        RobotCommands.hoodSubsys = hood;
        RobotCommands.intakeSubsys = intake;
        RobotCommands.drivetrain = drive;
        RobotCommands.limelightSubsys = limelight;
    }

    // ========== Fixed Shot Commands ==========

    /** Sets RPM/hood once and finishes — for use in auto sequences */
    public static Command windUpOnce() {
        return Commands.runOnce(() -> {
            shooterSubsys.setVelocityRPM(kFixedShotRPM);
            hoodSubsys.setPosition(kDefaultHoodPosition);
        }, shooterSubsys, hoodSubsys);
    }

    /** Auto wind up (default): sets RPM/hood, waits until at speed (max 2s), then finishes. */
    public static Command autoWindUp() {
        return Commands.runOnce(() -> {
            shooterSubsys.setVelocityRPM(kFixedShotRPM);
            hoodSubsys.setPosition(kDefaultHoodPosition);
        }, shooterSubsys, hoodSubsys)
        .andThen(Commands.waitUntil(shooterSubsys::isVelocityWithinTolerance).withTimeout(2.0));
    }

    /** Auto wind up (close): sets RPM/hood close, waits until at speed (max 2s), then finishes. */
    public static Command autoWindUpClose() {
        return Commands.runOnce(() -> {
            shooterSubsys.setVelocityRPM(kFixedShotRPM);
            hoodSubsys.setPosition(kCloseHoodPosition);
        }, shooterSubsys, hoodSubsys)
        .andThen(Commands.waitUntil(shooterSubsys::isVelocityWithinTolerance).withTimeout(2.0));
    }

    /** Auto wind up (closer/hub): sets RPM/hood closer, waits until at speed (max 2s), then finishes. */
    public static Command autoWindUpCloser() {
        return Commands.runOnce(() -> {
            shooterSubsys.setVelocityRPM(kFixedShotRPM);
            hoodSubsys.setPosition(kCloserHoodPosition);
        }, shooterSubsys, hoodSubsys)
        .andThen(Commands.waitUntil(shooterSubsys::isVelocityWithinTolerance).withTimeout(2.0));
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
    public static Command windUpPass() {
        return Commands.runEnd(
            () -> {
                shooterSubsys.setVelocityRPM(kPassShotRPM);
                hoodSubsys.setPosition(kPassHoodPosition);
            },
            () -> shooterSubsys.stopShooter(),
            shooterSubsys, hoodSubsys
        );
    }

    public static Command windUp110() {
        return Commands.runEnd(
            () -> {
                shooterSubsys.setVelocityRPM(kRPMAt110in);
                hoodSubsys.setPosition(kHoodAt110in);
            },
            () -> shooterSubsys.stopShooter(),
            shooterSubsys, hoodSubsys
        );
    }

    public static Command windUp75() {
        return Commands.runEnd(
            () -> {
                shooterSubsys.setVelocityRPM(kRPMAt75in);
                hoodSubsys.setPosition(kHoodAt75in);
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
        final double oscillationMotorRotations = (60.0 / 360.0) * 8.0;
        final double period = 0.8;
        final double[] state = {0, 0}; // [startTime, deployedPosition]
        return Commands.runEnd(
            () -> {
                feederSubsys.setSpeed(FeederSpeed.FEED_FAST);
                intakeSubsys.setSpeed(IntakeSpeed.INTAKE_FAST);
                if (state[0] == 0) {
                    state[0] = Timer.getFPGATimestamp();
                    state[1] = intakeSubsys.getRotatorPosition();
                }
                double elapsed = Timer.getFPGATimestamp() - state[0];
                boolean goUp = ((int)(elapsed / (period / 2.0)) % 2 == 0);
                double target = goUp ? state[1] + oscillationMotorRotations : state[1];
                intakeSubsys.setRotatorOscillate(target);
            },
            () -> {
                feederSubsys.setSpeed(FeederSpeed.OFF);
                intakeSubsys.setSpeed(IntakeSpeed.OFF);
                state[0] = 0;
            },
            feederSubsys, intakeSubsys
        );
    }

    /** Timed auto shoot: bounces intake + feeds for the given duration, then stops and redeploys intake. */
    public static Command autoShoot(double seconds) {
        final double oscillationMotorRotations = (60.0 / 360.0) * 8.0;
        final double period = 0.8;
        final double[] state = {0, 0}; // [startTime, deployedPosition]
        return Commands.runEnd(
            () -> {
                feederSubsys.setSpeed(FeederSpeed.FEED_FAST);
                intakeSubsys.setSpeed(IntakeSpeed.INTAKE_FAST);
                if (state[0] == 0) {
                    state[0] = Timer.getFPGATimestamp();
                    state[1] = intakeSubsys.getRotatorPosition();
                }
                double elapsed = Timer.getFPGATimestamp() - state[0];
                boolean goUp = ((int)(elapsed / (period / 2.0)) % 2 == 0);
                double target = goUp ? state[1] + oscillationMotorRotations : state[1];
                intakeSubsys.setRotatorOscillate(target);
            },
            () -> {
                feederSubsys.setSpeed(FeederSpeed.OFF);
                intakeSubsys.setSpeed(IntakeSpeed.OFF);
                intakeSubsys.setRotatorTarget(-14.5);
                shooterSubsys.stopShooter();
                state[0] = 0;
            },
            feederSubsys, intakeSubsys
        ).withTimeout(seconds);
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
            },
            () -> {
                intakeSubsys.setSpeed(IntakeSpeed.OFF);
                feederSubsys.setSpeed(FeederSpeed.OFF);
            },
            intakeSubsys, feederSubsys
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

        return Commands.runEnd(() -> {
                // Get robot pose from MT2-fused odometry
                final Pose2d robotPose = drivetrain.getState().Pose;
                final Translation2d robotPos = robotPose.getTranslation();

                // Get the detected tag's field position
                final int tagID = (int) LimelightHelpers.getFiducialID("limelight");
                SmartDashboard.putNumber("Tracked Tag ID", tagID);
                final Optional<Pose3d> tagPose = aprilTagFieldLayout.getTagPose(tagID);

                final Distance distance;
                double tx = 0.0;

                if (tagPose.isPresent()) {
                    final Translation2d tagPosition = tagPose.get().toPose2d().getTranslation();

                    // Compute hub center: 23.5" behind tag face along robot-to-tag direction
                    final Translation2d robotToTag = tagPosition.minus(robotPos);
                    final double tagDistMeters = robotToTag.getNorm();
                    final Translation2d unitDir = new Translation2d(
                        robotToTag.getX() / tagDistMeters,
                        robotToTag.getY() / tagDistMeters);
                    final Translation2d hubCenter = tagPosition.plus(
                        unitDir.times(kHubCenterOffsetInches * 0.0254));

                    final double distInches = tagDistMeters / 0.0254 + kHubCenterOffsetInches;
                    distance = Inches.of(distInches);

                    // Aim: field-frame angle from robot to tag
                    final double headingRad = robotPose.getRotation().getRadians();

                    // Shoot-on-the-move velocity compensation
                    final ChassisSpeeds fieldSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(
                        drivetrain.getState().Speeds, robotPose.getRotation());
                    final double distMeters = distInches * 0.0254;
                    final double flightTime = distMeters / BALL_VELOCITY_MS;
                    // Virtual target = hub center minus robot velocity * flight time
                    final Translation2d virtualTarget = hubCenter.minus(
                        new Translation2d(
                            fieldSpeeds.vxMetersPerSecond * flightTime,
                            fieldSpeeds.vyMetersPerSecond * flightTime));
                    final Translation2d robotToVirtual = virtualTarget.minus(robotPos);
                    final double virtualFieldAngle = Math.atan2(robotToVirtual.getY(), robotToVirtual.getX());

                    // tx = angle error in robot-relative degrees
                    tx = Math.toDegrees(virtualFieldAngle - headingRad) + kAimOffsetDegrees;
                    // Normalize to [-180, 180]
                    tx = Math.IEEEremainder(tx, 360.0);

                    SmartDashboard.putNumber("Flight Time", flightTime);
                } else {
                    // No valid tag ID — fall back to pose-only distance, no aim correction
                    distance = Meters.of(robotPos.getDistance(Landmarks.targetPosition()));
                }

                drivetrain.setControl(aimDrive
                    .withVelocityX(velocityX.getAsDouble())
                    .withVelocityY(velocityY.getAsDouble())
                    .withRotationalRate(tx * kAimP));

                final Shot shot = distanceToShotMap.get(distance);
                shooterSubsys.setVelocityRPM(shot.shooterRPM);
                hoodSubsys.setPosition(shot.hoodPosition);
                SmartDashboard.putNumber("Auto Distance (inches)", distance.in(Inches));
                SmartDashboard.putNumber("Corrected TX (deg)", tx);
                SmartDashboard.putNumber("Aim Rotation Rate", tx * kAimP);
            },
            () -> shooterSubsys.stopShooter(),
            drivetrain, shooterSubsys, hoodSubsys)
        ;
    }

    // ========== Auto-Aim Full-Field Pass ==========

    /** Trench AprilTag IDs used for pass aiming */
    private static final int[] kTrenchTagIDs = {7, 12, 23, 28};

    /**
     * Auto-aim pass: rotates the robot to face 15° inward from a trench AprilTag
     * toward field center, while spinning up to pass RPM/hood.
     * Driver retains full translation control. If no trench tag is visible,
     * just spins up without auto-rotation.
     */
    public static Command aimAndPass(DoubleSupplier velocityX, DoubleSupplier velocityY, double maxSpeed) {
        final SwerveRequest.FieldCentric passDrive = new SwerveRequest.FieldCentric()
            .withDeadband(maxSpeed * 0.1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

        return Commands.runEnd(() -> {
                shooterSubsys.setVelocityRPM(kPassShotRPM);
                hoodSubsys.setPosition(kPassHoodPosition);

                final int tagID = (int) LimelightHelpers.getFiducialID("limelight");
                final boolean isTrenchTag = tagID == 7 || tagID == 12 || tagID == 23 || tagID == 28;

                double rotationRate = 0.0;
                if (LimelightHelpers.getTV("limelight") && isTrenchTag) {
                    final double rawTx = LimelightHelpers.getTX("limelight");
                    // Tags 12, 28 → offset left (-15°); Tags 7, 23 → offset right (+15°)
                    final double offset = (tagID == 12 || tagID == 28)
                        ? -kPassAimOffsetDegrees
                        :  kPassAimOffsetDegrees;
                    final double correctedTx = rawTx + offset;
                    rotationRate = -correctedTx * kAimP;
                }

                drivetrain.setControl(passDrive
                    .withVelocityX(velocityX.getAsDouble())
                    .withVelocityY(velocityY.getAsDouble())
                    .withRotationalRate(rotationRate));
            },
            () -> shooterSubsys.stopShooter(),
            shooterSubsys, hoodSubsys
        );
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

    // ========== Auto Exposure Tuning ==========

    /**
     * Slowly sweeps Limelight exposure from low to high until an AprilTag is
     * detected continuously for 0.1 seconds. Publishes the current exposure
     * to SmartDashboard so you can see where it lands.
     *
     * Exposure range: 10 µs to 10000 µs, stepping by 50 µs every 100 ms.
     */
    public static Command autoTuneExposure() {
        final double[] exposure = {10.0};       // current exposure in µs
        final double[] tagSeenSince = {-1.0};   // timestamp when tag was first continuously seen
        final double kStep = 50.0;              // µs per step
        final double kMaxExposure = 10000.0;    // max exposure µs
        final double kStableTime = 0.35;        // seconds of continuous detection to accept

        return Commands.run(() -> {
            // Set exposure: sensor_set takes [autoExposure, exposure_us, autoGain, gain]
            // autoExposure=0 means manual
            LimelightHelpers.setLimelightNTDoubleArray("limelight", "sensor_set",
                new double[]{0, exposure[0], 1, 20});

            SmartDashboard.putNumber("LL Auto-Tune Exposure (us)", exposure[0]);

            if (LimelightHelpers.getTV("limelight")) {
                if (tagSeenSince[0] < 0) {
                    tagSeenSince[0] = Timer.getFPGATimestamp();
                }
            } else {
                tagSeenSince[0] = -1.0;
            }

            // If tag not yet stable, keep increasing exposure
            if (tagSeenSince[0] < 0 || Timer.getFPGATimestamp() - tagSeenSince[0] < kStableTime) {
                exposure[0] = Math.min(exposure[0] + kStep, kMaxExposure);
            }
            // Otherwise: tag is stable — stop incrementing (command keeps running to hold the value)
        }).until(() ->
            // Finish when tag has been stable for kStableTime
            tagSeenSince[0] > 0 && Timer.getFPGATimestamp() - tagSeenSince[0] >= kStableTime
        ).finallyDo(() ->
            SmartDashboard.putNumber("LL Tuned Exposure (us)", exposure[0])
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
