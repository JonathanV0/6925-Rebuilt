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
    // Fixed-shot flywheel RPM (used by all windUp variants and the shot table)
    public static final double kFixedShotRPM = 3350;

    // Hood positions for fixed-shot commands
    public static final double kDefaultHoodPosition = 0.5;
    public static final double kCloseHoodPosition = 0.3;
    public static final double kTestHoodPosition = 0.45;
    public static final double kCloserHoodPosition = 0.0;

    // Distance-to-shot interpolation table entries (hood positions)
    // Increase these values to make the shot go further (higher arc)
    public static final double kHoodAt47in = 0.1;
    public static final double kHoodAt84in = 0.37;
    public static final double kHoodAt120in = 0.55;

    // Limelight aim PD gains (radians/sec per degree of tx error / change)
    public static final double kAimP = 0.08;
    public static final double kAimD = 0.004;

    // Aim offset in degrees - positive shifts aim to the right, negative shifts left
    // Tune this if the robot consistently aims too far left or right
    public static final double kAimOffsetDegrees = 0.0;

    // How far ahead (seconds) to predict robot position for shot calculations
    public static final double kLookAheadSeconds = 0.25;

    // Horizontal offset from tag face to hub center (inches)
    public static final double kHubCenterOffsetInches = 23.5;
  }
}
