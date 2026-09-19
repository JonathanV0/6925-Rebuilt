package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static frc.robot.Constants.ShooterConstants.*;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
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
import frc.robot.util.OperatorDashboard;

public final class RobotCommands {
    private static ShooterSubsys shooterSubsys;
    private static FeederSubsys feederSubsys;
    private static HoodSubsys hoodSubsys;
    private static IntakeSubsys intakeSubsys;
    private static CommandSwerveDrivetrain drivetrain;
    private static LimelightSubsys limelightSubsys;
    private static OperatorDashboard operatorDashboard;

    private static double lastDeployedPosition = 0.0;

    // kRising: "ready" must hold for 50 ms before we believe it, but drops instantly.
    // One noisy RPM frame shouldn't be enough to fire the feeder.
    // TODO(tune): ready debounce time (seconds). Longer = fewer false starts from a single
    //   good frame, but adds that much delay to every shot. 0.05 s = 2-3 loops. Raise to
    //   0.1 if "Ready/ALL" flickers green/red while otherwise settled; don't go past ~0.2.
    private static final Debouncer readyDebouncer = new Debouncer(0.05, Debouncer.DebounceType.kRising);

    // TODO(tune): kAimOffsetDegrees. A constant aim bias added to every hub shot. Use it if
    //   the robot reliably misses to the same side when "Aim Heading Error (deg)" reads 0
    //   (shooter not centered on the robot, wheels spinning the ball sideways, etc.).
    //   Positive = rotate counter-clockwise (left). Try +/-1 deg steps.
    private static final double kAimOffsetDegrees = 0.0;

    // Ball time-of-flight vs. distance to hub (key: meters, value: seconds). A real ball
    // slows down in the air, so a lookup beats the old constant-velocity guess.
    // TODO(tune): flight-time table — these are PLACEHOLDERS, not measurements.
    //   What it is: how long the ball is in the air from leaving the wheels to entering the
    //   hub, at each distance. It sets how far ahead of the hub we aim while moving.
    //   How: put the robot at a known distance (read "Auto Distance (inches)"), film a shot
    //   with a phone in slow-mo (240 fps), count frames from ball-exit to hub-entry, divide
    //   by the frame rate. Repeat at ~1.5, 2.5, 3.5, 4.5 m and replace the points below.
    //   If unmeasured, expect the moving-shot lead to be wrong; stationary shots are unaffected.
    private static final InterpolatingDoubleTreeMap distanceToFlightTimeSec = new InterpolatingDoubleTreeMap();
    private static final double kFlightTableMinMeters = 1.5;
    private static final double kFlightTableMaxMeters = 4.5;
    static {
        distanceToFlightTimeSec.put(1.5, 0.6);
        distanceToFlightTimeSec.put(3.0, 0.9);
        distanceToFlightTimeSec.put(4.5, 1.2);
    }

    /**
     * Distance-table lookup with the operator's live RPM % trim applied. Every table-based
     * shot goes through here so the trim can't be forgotten at one call site. Fixed shots
     * (kFixedShotRPM) and pass shots deliberately bypass it.
     */
    private static Shot lookupShot(Distance distance) {
        final Shot raw = distanceToShotMap.get(distance);
        return new Shot(raw.shooterRPM() * operatorDashboard.getRPMMultiplier(), raw.hoodPosition());
    }

    /** Flight time from the table, or kLookAheadSeconds if we're outside the measured range. */
    private static double flightTimeSeconds(double distanceMeters) {
        if (distanceMeters < kFlightTableMinMeters || distanceMeters > kFlightTableMaxMeters) {
            return kLookAheadSeconds;
        }
        return distanceToFlightTimeSec.get(distanceMeters);
    }

    // ---- Look-ahead guard (from 1678's MotionCompensatedShootingPlanner.updateRejectLookAhead) ----
    // TODO(tune): reject-lookahead thresholds. While the robot is still whipping around to
    //   acquire the hub, its velocity is mostly rotation noise, and leading the target off
    //   that makes the aim wander. So the lead is dropped when spinning fast AND far off
    //   target, or when basically stationary (nothing to lead). 1678's values; raise the
    //   yaw-rate number if "Aim/Reject LookAhead" flickers while driving straight at speed.
    private static final double kRejectLookAheadYawRateRadPerSec = Math.toRadians(300.0);
    private static final double kRejectLookAheadHeadingErrorRad = Math.toRadians(50.0);
    private static final double kStaticShotSpeedMps = 0.09;

    /**
     * Where the robot will be one ball-flight-time from now (heading ignored). Both the aim
     * angle and the RPM/hood distance are measured from THIS point, so the one look-ahead
     * rule lives here instead of being duplicated per command. Moving the robot forward by
     * v*t is the same vector math as moving the target back by v*t (the old "virtual target").
     * Standing still it returns the current position, so stationary shots are unchanged.
     */
    private static Translation2d predictedTranslation() {
        final Pose2d pose = drivetrain.getState().Pose;
        final Translation2d robotPos = pose.getTranslation();
        final Translation2d hub = Landmarks.targetPosition();
        final ChassisSpeeds fieldSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(
            drivetrain.getState().Speeds, pose.getRotation());

        final double speedMps = Math.hypot(fieldSpeeds.vxMetersPerSecond, fieldSpeeds.vyMetersPerSecond);
        // Geometric heading error (not the PID's) so this works even when no aim command is active
        final double headingErrorRad = Math.abs(MathUtil.angleModulus(
            hub.minus(robotPos).getAngle().getRadians() - pose.getRotation().getRadians()));
        final boolean yawUnstable = Math.abs(fieldSpeeds.omegaRadiansPerSecond) >= kRejectLookAheadYawRateRadPerSec;
        final boolean rejectLookAhead =
            (yawUnstable && headingErrorRad > kRejectLookAheadHeadingErrorRad) || speedMps < kStaticShotSpeedMps;
        SmartDashboard.putBoolean("Aim/Reject LookAhead", rejectLookAhead);
        if (rejectLookAhead) {
            return robotPos;
        }

        final double flightTime = flightTimeSeconds(robotPos.getDistance(hub));
        return robotPos.plus(new Translation2d(
            fieldSpeeds.vxMetersPerSecond * flightTime,
            fieldSpeeds.vyMetersPerSecond * flightTime));
    }

    // Distance-to-shot lookup table (team should calibrate these values)
    private static final InterpolatingTreeMap<Distance, Shot> distanceToShotMap = new InterpolatingTreeMap<>(
        (startValue, endValue, q) ->
            InverseInterpolator.forDouble()
                .inverseInterpolate(startValue.in(Meters), endValue.in(Meters), q.in(Meters)),
        (startValue, endValue, t) ->
            new Shot(
                Interpolator.forDouble().interpolate(startValue.shooterRPM(), endValue.shooterRPM(), t),
                Interpolator.forDouble().interpolate(startValue.hoodPosition(), endValue.hoodPosition(), t)
            )
    );

    static {
        distanceToShotMap.put(Inches.of(47.0), new Shot(kFixedShotRPM + 150, kHoodAt47in));
        distanceToShotMap.put(Inches.of(50.0), new Shot(kRPMAt50in + 150, kHoodAt50in));
        distanceToShotMap.put(Inches.of(65.0), new Shot(kRPMAt65in + 150, kHoodAt65in));
        distanceToShotMap.put(Inches.of(75.125), new Shot(kRPMAt75in + 150, kHoodAt75in));                                                                                                                                                                                                                              
        distanceToShotMap.put(Inches.of(85.0), new Shot(kRPMAt85in + 150, kHoodAt85in));
        distanceToShotMap.put(Inches.of(92.0), new Shot(kRPMAt92in + 150, kHoodAt92in));
        distanceToShotMap.put(Inches.of(100.0), new Shot(kRPMAt100in + 150, kHoodAt100in));
        distanceToShotMap.put(Inches.of(110.0), new Shot(kRPMAt110in + 150, kHoodAt110in));
        distanceToShotMap.put(Inches.of(120.0), new Shot(kRPMAt120in + 150, kHoodAt120in));
        distanceToShotMap.put(Inches.of(130.0), new Shot(kRPMAt130in + 150, kHoodAt130in));
        distanceToShotMap.put(Inches.of(140.0), new Shot(kRPMAt140in + 150, kHoodAt140in));
    }

    public static void init(
        ShooterSubsys shooter,
        FeederSubsys feeder,
        HoodSubsys hood,
        IntakeSubsys intake,
        CommandSwerveDrivetrain drive,
        LimelightSubsys limelight,
        OperatorDashboard dashboard
    ) {
        RobotCommands.shooterSubsys = shooter;
        RobotCommands.feederSubsys = feeder;
        RobotCommands.hoodSubsys = hood;
        RobotCommands.intakeSubsys = intake;
        RobotCommands.drivetrain = drive;
        RobotCommands.limelightSubsys = limelight;
        RobotCommands.operatorDashboard = dashboard;
        // Published here so the toggle exists on the dashboard before anyone needs it
        SmartDashboard.putBoolean("Ignore Shot Gates", false);
    }

    // ========== Shot Readiness Gate ==========

    /**
     * True only when every condition for a good shot holds (modeled on 2910's
     * isReadyToScore). Also publishes each sub-condition so the operator can see WHICH
     * gate is blocking. Call every loop; gatedShoot() feeds only while this is true.
     * The "Ignore Shot Gates" dashboard toggle bypasses everything for when a sensor
     * is lying mid-match and the operator would rather trust their eyes.
     */
    public static boolean isReadyToShoot() {
        final Pose2d pose = drivetrain.getState().Pose;
        final ChassisSpeeds speeds = drivetrain.getState().Speeds;
        // Speed magnitude is the same in the robot or field frame, so no conversion needed
        final double speedMps = Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
        final double distanceMeters = pose.getTranslation().getDistance(Landmarks.targetPosition());

        final boolean atSpeed    = shooterSubsys.isVelocityWithinTolerance();
        final boolean hoodAtPos  = hoodSubsys.isPositionWithinTolerance();
        final boolean atHeading  = drivetrain.isAtHeading(Math.toRadians(kScoringHeadingToleranceDeg));
        final boolean slowEnough = speedMps < kScoringSpeedToleranceMps;
        final boolean farEnough  = distanceMeters >= kMinimumShotDistanceMeters;
        final boolean all = readyDebouncer.calculate(
            atSpeed && hoodAtPos && atHeading && slowEnough && farEnough);

        SmartDashboard.putBoolean("Ready/AtSpeed", atSpeed);
        SmartDashboard.putBoolean("Ready/HoodAtPos", hoodAtPos);
        SmartDashboard.putBoolean("Ready/Heading", atHeading);
        SmartDashboard.putBoolean("Ready/Speed", slowEnough);
        SmartDashboard.putBoolean("Ready/Distance", farEnough);
        SmartDashboard.putBoolean("Ready/ALL", all);

        return all || SmartDashboard.getBoolean("Ignore Shot Gates", false);
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
            hoodSubsys.setPosition(kHoodAt47in);
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

    /** Manual wind-up: reads "Manual Distance (in)" from SmartDashboard and sets RPM/hood
     *  from the interpolation table. Use to test specific distance points without vision. */
    public static Command manualWindUp() {
        SmartDashboard.putNumber("Manual Distance (in)", 75.0);
        return Commands.runEnd(
            () -> {
                final Distance distance = Inches.of(SmartDashboard.getNumber("Manual Distance (in)", 75.0));
                final Shot shot = lookupShot(distance);
                shooterSubsys.setVelocityRPM(shot.shooterRPM());
                hoodSubsys.setPosition(shot.hoodPosition());
                SmartDashboard.putNumber("Manual Shot RPM", shot.shooterRPM());
                SmartDashboard.putNumber("Manual Shot Hood", shot.hoodPosition());
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

    /** Manual shoot: feeds immediately while held, no readiness checks. Operator override. */
    public static Command Shoot() {
        return shootWithFeedGate(() -> true);
    }

    /**
     * Gated shoot: identical to Shoot() (feeder + intake bounce while held) except the
     * feeder only runs while isReadyToShoot() is true. Lets the operator hold the trigger
     * early and have the ball leave the instant the robot is actually ready.
     */
    public static Command gatedShoot() {
        return shootWithFeedGate(RobotCommands::isReadyToShoot);
    }

    /**
     * Shared body for Shoot()/gatedShoot(). feedNow is a BooleanSupplier: a tiny function
     * we call every loop to ask "should the feeder run right now?". That keeps one copy of
     * the intake-bounce logic instead of two that could drift apart.
     */
    private static Command shootWithFeedGate(BooleanSupplier feedNow) {
        final double retractedPosition = 0.0; // fully retracted motor position (rotations)
        final double raiseDuration = 5.0;     // seconds to fully retract from deployed
        final double shakeAmount = (30.0 / 360.0) * 8.0;
        final double shakePeriod = 0.8;
        final double[] state = {Double.NaN, 0}; // [deployedPosition, startTime]
        return Commands.runEnd(
            () -> {
                feederSubsys.setSpeed(feedNow.getAsBoolean() ? FeederSpeed.FEED_FAST : FeederSpeed.OFF);
                if (Double.isNaN(state[0])) {
                    state[0] = intakeSubsys.getRotatorPosition();
                    state[1] = Timer.getFPGATimestamp();
                    lastDeployedPosition = state[0];
                }
                double elapsed = Timer.getFPGATimestamp() - state[1];
                double progress = Math.min(elapsed / raiseDuration, 1.0);
                double baseTarget = state[0] + progress * (retractedPosition - state[0]);
                boolean goUp = ((int)(elapsed / (shakePeriod / 2.0)) % 2 == 0);
                intakeSubsys.setRotatorOscillate(goUp ? baseTarget + shakeAmount : baseTarget);
            },
            () -> {
                feederSubsys.setSpeed(FeederSpeed.OFF);
                intakeSubsys.setSpeed(IntakeSpeed.OFF);
                state[0] = Double.NaN;
            },
            feederSubsys, intakeSubsys
        );
    }

    /** Instantly commands the intake back to its deployed position.
     *  Bind with operator.button(1).onFalse() to run automatically after Shoot() ends. */
    public static Command redeployAfterShoot() {
        return Commands.runOnce(
            () -> intakeSubsys.setRotatorTarget(lastDeployedPosition),
            intakeSubsys
        );
    }

    /** Timed auto shoot: raises intake + feeds for the given duration, then stops and redeploys intake. */
    public static Command autoShoot(double seconds) {
        final double raiseAmount = (180.0 / 360.0) * 8.0;
        final double[] deployedPosition = {Double.NaN};
        return Commands.runEnd(
            () -> {
                feederSubsys.setSpeed(FeederSpeed.FEED_FAST);
                intakeSubsys.setSpeed(IntakeSpeed.INTAKE_FAST);
                if (Double.isNaN(deployedPosition[0])) {
                    deployedPosition[0] = intakeSubsys.getRotatorPosition();
                }
                intakeSubsys.setRotatorGentle(deployedPosition[0] + raiseAmount);
            },
            () -> {
                feederSubsys.setSpeed(FeederSpeed.OFF);
                intakeSubsys.setSpeed(IntakeSpeed.OFF);
                intakeSubsys.setRotatorTarget(-14.5);
                shooterSubsys.stopShooter();
                deployedPosition[0] = Double.NaN;
            },
            feederSubsys, intakeSubsys
        ).withTimeout(seconds);
    }

    public static Command toggleShooterIdle() {
        return Commands.runOnce(() -> shooterSubsys.toggleIdle());
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
    public static Command aimAndWindUp(DoubleSupplier velocityX, DoubleSupplier velocityY) {
        return Commands.runEnd(() -> {
                // Get robot pose from MT2-fused odometry
                final Pose2d robotPose = drivetrain.getState().Pose;
                final Translation2d robotPos = robotPose.getTranslation();
                final Translation2d hubCenter = Landmarks.targetPosition();

                // Distance from robot to hub center (odometry-based, no TY/vision)
                final double distMeters = robotPos.getDistance(hubCenter);
                final Distance distance = Meters.of(distMeters);

                // Shoot-on-the-move: aim and range from where we'll be when the ball lands.
                // predictedTranslation() applies the flight-time lead and the reject guard.
                final Translation2d predictedToHub = hubCenter.minus(predictedTranslation());
                // getAngle() is atan2(y, x) wrapped in a Rotation2d, so no manual normalizing
                final Rotation2d aimFieldAngle = predictedToHub.getAngle()
                    .plus(Rotation2d.fromDegrees(kAimOffsetDegrees));

                // The drivetrain's heading PID closes the loop; we only hand it the goal.
                drivetrain.setControl(drivetrain.facingFieldAngle(aimFieldAngle)
                    .withVelocityX(velocityX.getAsDouble())
                    .withVelocityY(velocityY.getAsDouble()));

                // RPM/hood come from the predicted distance: that's the path the ball actually
                // flies while moving. Standing still it equals the plain hub distance.
                final Distance aimDistance = Meters.of(predictedToHub.getNorm());
                final Shot shot = lookupShot(aimDistance);
                shooterSubsys.setVelocityRPM(shot.shooterRPM());
                hoodSubsys.setPosition(shot.hoodPosition());
                SmartDashboard.putNumber("Auto Distance (inches)", distance.in(Inches));
                SmartDashboard.putNumber("Aim Distance (inches)", aimDistance.in(Inches));
                SmartDashboard.putNumber("Aim Heading Error (deg)", Math.toDegrees(drivetrain.getHeadingErrorRadians()));
                SmartDashboard.putNumber("Flight Time", flightTimeSeconds(distMeters));
                SmartDashboard.putNumber("Robot X (in)", robotPos.getX() / 0.0254);
                SmartDashboard.putNumber("Robot Y (in)", robotPos.getY() / 0.0254);
                SmartDashboard.putNumber("Robot Heading (deg)", robotPose.getRotation().getDegrees());
                SmartDashboard.putNumber("Target X (in)", hubCenter.getX() / 0.0254);
                SmartDashboard.putNumber("Target Y (in)", hubCenter.getY() / 0.0254);
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

                if (LimelightHelpers.getTV("limelight") && isTrenchTag) {
                    final double rawTx = LimelightHelpers.getTX("limelight");
                    // Tags 12, 28 → offset left (-15°); Tags 7, 23 → offset right (+15°)
                    final double offset = (tagID == 12 || tagID == 28)
                        ? -kPassAimOffsetDegrees
                        :  kPassAimOffsetDegrees;
                    final double correctedTx = rawTx + offset;
                    // Limelight tx is positive when the tag is to the RIGHT of the crosshair, and a
                    // right turn is a NEGATIVE (clockwise) heading change in WPILib, so subtract.
                    // Re-evaluated every loop so the goal tracks the live tx like the old P loop did.
                    final Rotation2d targetHeading = drivetrain.getState().Pose.getRotation()
                        .minus(Rotation2d.fromDegrees(correctedTx));
                    drivetrain.setControl(drivetrain.facingFieldAngle(targetHeading)
                        .withVelocityX(velocityX.getAsDouble())
                        .withVelocityY(velocityY.getAsDouble()));
                    SmartDashboard.putNumber("Pass Corrected TX (deg)", correctedTx);
                } else {
                    // No trench tag: same as before — translate freely, no rotation command
                    drivetrain.setControl(passDrive
                        .withVelocityX(velocityX.getAsDouble())
                        .withVelocityY(velocityY.getAsDouble())
                        .withRotationalRate(0.0));
                }
            },
            () -> shooterSubsys.stopShooter(),
            // drivetrain must be a requirement: this command calls setControl() every loop, and
            // without it the default drive command keeps running and fights over the swerve.
            drivetrain, shooterSubsys, hoodSubsys
        );
    }

    // ========== Range-Adjusted Shot Commands ==========

    private static Distance getDistanceToTarget() {
        final Translation2d robotPosition = drivetrain.getState().Pose.getTranslation();
        final Translation2d targetPosition = Landmarks.targetPosition();
        return Meters.of(robotPosition.getDistance(targetPosition));
    }

    /**
     * Predicts where the robot will be one ball-flight-time from now based on current
     * velocity, then returns the distance from that future position to the target.
     * More accurate than current-position distance when shooting while moving.
     */
    private static Distance getPredictedDistanceToTarget() {
        return Meters.of(predictedTranslation().getDistance(Landmarks.targetPosition()));
    }

    public static Command adjustedWindUp() {
        return Commands.run(() -> {
            final Distance distance = getPredictedDistanceToTarget();
            final Shot shot = lookupShot(distance);
            shooterSubsys.setVelocityRPM(shot.shooterRPM());
            hoodSubsys.setPosition(shot.hoodPosition());
            SmartDashboard.putNumber("Distance to Target (inches)", distance.in(Inches));
            SmartDashboard.putNumber("Target RPM", shot.shooterRPM());
            SmartDashboard.putNumber("Target Hood Position", shot.hoodPosition());
        }, shooterSubsys, hoodSubsys);
    }


    /**
     * Fallback shot when vision can't see: the operator parks the robot at a spot they
     * know by feel, this snaps odometry's X/Y to that spot (heading is kept — the gyro
     * is still trusted), then runs the normal distance-table wind-up from there.
     * The pose reset is a runOnce with no drivetrain requirement so it doesn't interrupt
     * an aim command the driver may be holding at the same time.
     */
    public static Command shootFromKnownSpot(Landmarks.KnownSpot spot) {
        return Commands.runOnce(() -> drivetrain.resetTranslation(spot.translation()))
            .andThen(adjustedWindUp());
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
                final Shot shot = lookupShot(distance);
                shooterSubsys.setVelocityRPM(shot.shooterRPM());
                hoodSubsys.setPosition(shot.hoodPosition());
            }, shooterSubsys, hoodSubsys)
            .until(shooterSubsys::isVelocityWithinTolerance)
            .withTimeout(2.0),
            // Phase 2: maintain RPM/hood AND run both feeders to shoot while still moving
            Commands.run(() -> {
                final Distance distance = getPredictedDistanceToTarget();
                final Shot shot = lookupShot(distance);
                shooterSubsys.setVelocityRPM(shot.shooterRPM());
                hoodSubsys.setPosition(shot.hoodPosition());
                feederSubsys.setSpeed(FeederSpeed.FEED_FAST);
            }, shooterSubsys, hoodSubsys, feederSubsys)
        );
    }

    /**
     * Snaps RPM and hood to distance-table values once from current robot position,
     * then blocks until all three shooter motors reach target RPM (within ShooterSubsys.kVelocityToleranceRPM).
     * Times out after 2 seconds to prevent auto deadlock on CAN dropout or brownout.
     * Use in sequential autos before calling shoot().
     */
    public static Command adjustedWindUpOnce() {
        return Commands.runOnce(() -> {
            final Distance distance = getDistanceToTarget();
            final Shot shot = lookupShot(distance);
            shooterSubsys.setVelocityRPM(shot.shooterRPM());
            hoodSubsys.setPosition(shot.hoodPosition());
        }, shooterSubsys, hoodSubsys)
        .andThen(Commands.waitUntil(shooterSubsys::isVelocityWithinTolerance).withTimeout(2.0));
    }

    // ========== Autonomous Auto-Aim ==========

    /**
     * Autonomous auto-aim: rotates the robot to face the hub AND sets
     * RPM/hood from the distance interpolation table.
     * Finishes when heading error < 2° AND shooter is at target RPM.
     * Times out at 3 seconds to prevent auto deadlock.
     *
     * Use as a PathPlanner event marker at any shoot point in an auto routine.
     * Follow this with the "shoot" named command to fire.
     */
    public static Command autoAimAndWindUp() {
        // TODO(tune): auto aim tolerance (degrees). Tighter than teleop's 4 deg because auto
        //   has time to settle. If autos time out here (3 s) before firing, loosen to 3-4.
        final double kHeadingToleranceRad = Math.toRadians(2.0);

        return Commands.run(() -> {
                final Pose2d robotPose = drivetrain.getState().Pose;
                final Translation2d robotPos = robotPose.getTranslation();
                final Translation2d hubCenter = Landmarks.targetPosition();

                // Distance for RPM/hood interpolation
                final double distMeters = robotPos.getDistance(hubCenter);
                final Distance distance = Meters.of(distMeters);

                // Field angle from robot to hub; the drivetrain's heading PID does the rest
                final Rotation2d targetFieldAngle = hubCenter.minus(robotPos).getAngle()
                    .plus(Rotation2d.fromDegrees(kAimOffsetDegrees));

                // Rotate toward target, no translation (robot holds position while aiming)
                drivetrain.setControl(drivetrain.facingFieldAngle(targetFieldAngle)
                    .withVelocityX(0)
                    .withVelocityY(0));

                // Set RPM and hood from distance table
                final Shot shot = lookupShot(distance);
                shooterSubsys.setVelocityRPM(shot.shooterRPM());
                hoodSubsys.setPosition(shot.hoodPosition());

                SmartDashboard.putNumber("Auto Aim Heading Error (deg)", Math.toDegrees(drivetrain.getHeadingErrorRadians()));
                SmartDashboard.putNumber("Auto Aim Distance (in)", distance.in(Inches));
            }, drivetrain, shooterSubsys, hoodSubsys)
            // isAtHeading() reads the error the heading PID computed on its last run
            .until(() -> drivetrain.isAtHeading(kHeadingToleranceRad)
                      && shooterSubsys.isVelocityWithinTolerance())
            .withTimeout(3.0);
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

    /**
     * A record is a compact immutable data class: Java writes the constructor, the
     * accessors shooterRPM()/hoodPosition(), equals/hashCode and toString for us.
     * Shots never change after creation, so a record is the right fit.
     */
    public record Shot(double shooterRPM, double hoodPosition) {}
}
