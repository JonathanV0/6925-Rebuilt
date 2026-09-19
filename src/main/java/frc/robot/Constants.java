// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.util.TunableNumber;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  // When true, TunableNumbers read live from the "/Tuning" NetworkTable. Leave false for
  // competition so a leftover dashboard value can't silently change robot behavior.
  public static final boolean kTuningMode = false;

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class ShooterConstants {
    // Fixed-shot flywheel RPM (used by basic windUp variants)
    public static final double kFixedShotRPM = 3350;
    public static final double kPassShotRPM = 5650;

    // Hood positions for fixed-shot commands
    public static final double kDefaultHoodPosition = 0.5;
    public static final double kCloseHoodPosition = 0.3;
    public static final double kCloserHoodPosition = 0.0;
    public static final double kPassHoodPosition = 0.7;

    // Distance-to-shot interpolation table entries (team-calibrated)
    public static final double kHoodAt47in = 0.00;
    public static final double kRPMAt50in = 3000;
    public static final double kHoodAt50in = 0.0;
    public static final double kHoodAt84in = 0.15;
    public static final double kRPMAt85in = 3300;
    public static final double kHoodAt85in = 0.1;
    public static final double kRPMAt65in = 3200;
    public static final double kHoodAt65in = 0.015;
    public static final double kRPMAt75in = 3250;
    public static final double kHoodAt75in = 0.075;
    public static final double kRPMAt92in = 3350;
    public static final double kHoodAt92in = 0.2;
    public static final double kRPMAt100in = 3350;
    public static final double kHoodAt100in = 0.2;
    public static final double kRPMAt110in = 3450;
    public static final double kHoodAt110in = 0.25;
    public static final double kRPMAt120in = 3400;
    public static final double kHoodAt120in = 0.3;
    public static final double kRPMAt130in = 3450;
    public static final double kHoodAt130in = 0.5;
    public static final double kRPMAt140in = 3500;
    public static final double kHoodAt140in = 0.4;

    // Aim heading gain, radians/sec per DEGREE of heading error (the drivetrain rescales
    // it to per-radian for the heading PID). Tunable live when kTuningMode is on.
    // TODO(tune): kAimP. What it is: how hard the robot turns per degree of aim error.
    //   Too low  -> slow to swing onto the hub, "Ready/Heading" takes forever to go green.
    //   Too high -> overshoots and wobbles back and forth around the target.
    //   How: hold right bumper standing still, watch "Aim Heading Error (deg)"; raise until
    //   it snaps to ~0 with one small overshoot, then back off ~20%. 0.1 deg-based = 5.7/rad;
    //   2910 runs P=5.0 with D=0.15 (rad-based) — add D in applyHeadingP() if it oscillates.
    public static final TunableNumber kAimP = new TunableNumber("Aim/kAimP", 0.1);
    public static final double kAimD = 0.01;

    // Fallback look-ahead (seconds) when the distance is outside the measured
    // flight-time table in RobotCommands (distanceToFlightTimeSec)
    // TODO(tune): kLookAheadSeconds. Only used when closer than 1.5 m or farther than 4.5 m
    //   from the hub. Should roughly equal the ball's flight time at those extremes; once
    //   the flight-time table is measured, set this to the table's nearest end value.
    public static final double kLookAheadSeconds = 0.25;

    // Horizontal offset from tag face to hub center (inches)
    public static final double kHubCenterOffsetInches = 23.5;

    // Pass aim offset: degrees inward from trench AprilTag toward field center
    public static final double kPassAimOffsetDegrees = 15.0;

    // Shot-readiness gates (see RobotCommands.isReadyToShoot). Modeled on 2910's
    // isReadyToScore: a ball fed while any of these is false is a likely miss.
    // 2910's 2026 values are identical (4 deg, 0.15 m/s, 1.5 m) — a sane starting point.
    //
    // TODO(tune): kScoringHeadingToleranceDeg. How far off-target (degrees) we still fire.
    //   The hub opening is wide, so this is forgiving up close and strict far away. Start
    //   at 4; if far shots clip the rim left/right, lower it; if the feeder hesitates while
    //   the robot is visibly on target, raise it. Watch "Ready/Heading" on the dashboard.
    public static final double kScoringHeadingToleranceDeg = 4.0;
    // TODO(tune): kScoringSpeedToleranceMps. Max robot speed (m/s) at which we fire.
    //   The distance table was calibrated standing still, so movement adds error the table
    //   doesn't know about. 0.15 m/s is "basically stopped". Raise it only if you've shot
    //   while creeping and the balls still go in; "Ready/Speed" shows this gate.
    public static final double kScoringSpeedToleranceMps = 0.15;
    // Closer than this the hood geometry can't loft the ball into the hub
    // TODO(tune): kMinimumShotDistanceMeters. Park at the closest spot that still scores
    //   (bumper to hub wall) and read "Auto Distance (inches)"; set this a bit below that.
    //   1.5 m = 59 in; the table's closest point is 47 in, so 1.2 m may be right for us.
    public static final double kMinimumShotDistanceMeters = 1.5;
    // Max pitch/roll (degrees) at which we still fire (1678's ShotVerifier.isTilted). On a
    // bump ramp the hood angle relative to the field is wrong, so the shot would be too.
    // TODO(tune): kMaxShotTiltDeg. First check the Pigeon's pitch/roll read ~0 on flat
    //   carpet ("Robot Pitch (deg)" / "Robot Roll (deg)"); if not, mount-calibrate it in
    //   Tuner X. Then confirm 5 deg blocks shots on the ramp but not on field seams.
    public static final double kMaxShotTiltDeg = 5.0;
  }
}
