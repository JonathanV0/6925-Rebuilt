// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class ShooterConstants {
    // Fixed-shot flywheel RPM (used by basic windUp variants)
    public static final double kFixedShotRPM = 3350;
    public static final double kPassShotRPM = 3500;

    // Hood positions for fixed-shot commands
    public static final double kDefaultHoodPosition = 0.5;
    public static final double kCloseHoodPosition = 0.3;
    public static final double kTestHoodPosition = 0.45;
    public static final double kCloserHoodPosition = 0.0;
    public static final double kPassHoodPosition = .75;

    // Distance-to-shot interpolation table entries (team-calibrated)
    public static final double kRPMAt47in = 3350;
    public static final double kHoodAt47in = 0.1;
    public static final double kRPMAt84in = 3350;
    public static final double kHoodAt84in = 0.37;
    public static final double kRPMAt120in = 3350;
    public static final double kHoodAt120in = 0.45;
    public static final double kRPMAt75in = 3350;
    public static final double kHoodAt75in = 0.15;
    public static final double kRPMAt139in = 3350;
    public static final double kHoodAt139in = 0.6;

    // Extended range data points (from WCP CC Big Dumper)
    public static final double kRPMAt52in = 2800;
    public static final double kHoodAt52in = 0.19;
    public static final double kRPMAt114in = 3275;
    public static final double kHoodAt114in = 0.40;
    public static final double kRPMAt165in = 3650;
    public static final double kHoodAt165in = 0.48;

    // Limelight aim PD gains (radians/sec per degree of tx error / change)
    public static final double kAimP = 0.1;
    public static final double kAimD = 0.01;

    // Aim offset in degrees - positive shifts aim to the right, negative shifts left
    // Tune this if the robot consistently aims too far left or right
    public static final double kAimOffsetDegrees = -11.0;

    // How far ahead (seconds) to predict robot position for shot calculations
    public static final double kLookAheadSeconds = 0.25;

    // Horizontal offset from tag face to hub center (inches)
    public static final double kHubCenterOffsetInches = 23.5;
  }
}
