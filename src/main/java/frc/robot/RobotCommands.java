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

    private static final double kAimOffsetDegrees = 0.0;

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
        distanceToShotMap.put(Inches.of(75.125), new Shot(kRPMAt75in + 150, kHoodAt75in));
        distanceToShotMap.put(Inches.of(84.0), new Shot(kFixedShotRPM + 150, kHoodAt84in));
        distanceToShotMap.put(Inches.of(92.0), new Shot(kRPMAt92in + 150, kHoodAt92in));
        distanceToShotMap.put(Inches.of(100.0), new Shot(kRPMAt100in + 150, kHoodAt100in));
    }

    public static void init(
        ShooterSubsys shooter,
        FeederSubsys feeder,
        HoodSubsys hood,
        IntakeSubsys intake,
        CommandSwerveDrivetrain drive
    ) {
        RobotCommands.shooterSubsys = shooter;
        RobotCommands.feederSubsys = feeder;
        RobotCommands.hoodSubsys = hood;
        RobotCommands.intakeSubsys = intake;
        RobotCommands.drivetrain = drive;
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

    // TODO: 139.5" was measured from limelight to hub center, not robot center to hub center — remeasure
    public static Command windUp139() {
        return Commands.runEnd(
            () -> {
                shooterSubsys.setVelocityRPM(kRPMAt139in);
                hoodSubsys.setPosition(kHoodAt139in);
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

        // Hold last known values so flicker frames don't cause jumps
        final double[] lastTx = {0.0};
        final Distance[] lastDistance = {null};  // null = no reading yet, accept first one unconditionally
        final int[] outlierCount = {0};  // consecutive cycles the reading was rejected

        return Commands.runEnd(() -> {
                final int tagID = (int) LimelightHelpers.getFiducialID("limelight");
                SmartDashboard.putNumber("Tracked Tag ID", tagID);

                // Compute distance and aim angle to hub center
                final Distance distance;
                double tx;
                if (LimelightHelpers.getTV("limelight")) {
                    final double rawTx = LimelightHelpers.getTX("limelight");
                    final double ty = LimelightHelpers.getTY("limelight");
                    final double heightDiff = LimelightSubsys.kTargetHeightInches - LimelightSubsys.kCameraHeightInches;
                    final double angleRad = Math.toRadians(LimelightSubsys.kCameraMountAngleDegrees + ty);
                    final double cameraToTagInches = heightDiff / Math.tan(angleRad);
                    final double distInches = cameraToTagInches + kHubCenterOffsetInches;
                    final Distance rawDistance = Inches.of(distInches);

                    // Accept first reading unconditionally; after that, reject outlier spikes
                    // unless they persist for 3+ cycles (then it's real movement)
                    if (lastDistance[0] == null) {
                        distance = rawDistance;
                        lastDistance[0] = distance;
                    } else if (Math.abs(distInches - lastDistance[0].in(Inches)) > 20.0) {
                        outlierCount[0]++;
                        if (outlierCount[0] >= 3) {
                            // Sustained new distance — accept it
                            distance = rawDistance;
                            lastDistance[0] = distance;
                            outlierCount[0] = 0;
                        } else {
                            distance = lastDistance[0];
                        }
                    } else {
                        distance = rawDistance;
                        lastDistance[0] = distance;
                        outlierCount[0] = 0;
                    }

                    // Compute aim angle to hub center (behind tag + lateral offset)
                    final double rawTxRad = Math.toRadians(rawTx);
                    double lateralInches = 0.0;
                    switch (tagID) {
                        case 8: case 24:           // Offset-RIGHT tags — hub center is LEFT
                            lateralInches = -8.0;
                            break;
                        case 9: case 11:           // Offset-LEFT tags — hub center is RIGHT
                        case 25: case 27:
                            lateralInches = 8.0;
                            break;
                    }
                    final double hubLateral = cameraToTagInches * Math.sin(rawTxRad) + lateralInches;
                    final double hubForward = cameraToTagInches * Math.cos(rawTxRad) + kHubCenterOffsetInches;
                    tx = Math.toDegrees(Math.atan2(hubLateral, hubForward)) + kAimOffsetDegrees;

                    // Update last-known aim angle
                    lastTx[0] = tx;
                } else {
                    // No target — hold last known aim and distance, or use odometry if no reading yet
                    tx = lastTx[0];
                    distance = lastDistance[0] != null ? lastDistance[0] : getPredictedDistanceToTarget();
                }
                drivetrain.setControl(aimDrive
                    .withVelocityX(velocityX.getAsDouble())
                    .withVelocityY(velocityY.getAsDouble())
                    .withRotationalRate(-tx * kAimP));

                final Shot shot = distanceToShotMap.get(distance);
                shooterSubsys.setVelocityRPM(shot.shooterRPM);
                hoodSubsys.setPosition(shot.hoodPosition);
                SmartDashboard.putNumber("Auto Distance (inches)", distance.in(Inches));
                SmartDashboard.putNumber("Corrected TX (deg)", tx);
                SmartDashboard.putBoolean("LL TV (code)", LimelightHelpers.getTV("limelight"));
                SmartDashboard.putNumber("LL TX (code)", LimelightHelpers.getTX("limelight"));
                SmartDashboard.putNumber("Aim Rotation Rate", -tx * kAimP);
            },
            () -> shooterSubsys.stopShooter(),
            shooterSubsys, hoodSubsys)
        ;
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
