// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.CTREConfigs;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class ShooterSubsys extends SubsystemBase {
  /** Creates a new ShooterSubsys. */
  private final TalonFX fuelShoot = new TalonFX(8, "CANivore");
  private final TalonFX fuelShoot0 = new TalonFX(9, "CANivore");
  private final TalonFX fuelShoot1 = new TalonFX(10, "CANivore");

  private final TalonFX fuelFeed = new TalonFX(11, "CANivore");


  public ShooterSubsys() {

    fuelShoot.getConfigurator().apply(CTREConfigs.SHOOTER_CONFIG);
    fuelShoot0.getConfigurator().apply(CTREConfigs.SHOOTER_CONFIG);
    fuelShoot1.getConfigurator().apply(CTREConfigs.SHOOTER_CONFIG);

    fuelShoot0.setControl(new Follower(fuelShoot.getDeviceID(), MotorAlignmentValue.Aligned));
    fuelShoot1.setControl(new Follower(fuelShoot.getDeviceID(), MotorAlignmentValue.Aligned));
  }

  // ========== AI GENERATED - PLEASE DOUBLE CHECK ==========

  // Velocity control request for PID
  private final VelocityVoltage m_velocityRequest = new VelocityVoltage(0);

  // Velocity PID control - Use this for precise RPM control in competition
  // Only command the leader (fuelShoot). Motors 9 and 10 are followers and will match automatically.
  public void setVelocityRPM(double rpm) {
    targetRPM = rpm;
    double rps = rpm / 60.0; // Convert RPM to rotations per second
    fuelShoot.setControl(m_velocityRequest.withVelocity(rps));
  }

  // Commands for button binding
  public Command setVelocityRPMCommand(double rpm) {
    return Commands.runOnce(() -> setVelocityRPM(rpm), this);
  }

  private static final double kVelocityToleranceRPM = 100.0;
  private double targetRPM = 0;

  public double getVelocityRPM() {
    return fuelShoot.getVelocity().getValueAsDouble() * 60.0;
  }

  public boolean isVelocityWithinTolerance() {
    // Require a positive target — prevents false-positive when shooter is idle (0 RPM = "at speed")
    return targetRPM > 0 && Math.abs(getVelocityRPM() - targetRPM) < kVelocityToleranceRPM;
  }
  // ========== END AI GENERATED CODE ==========

  public void setSpeed(FuelFeedSpeed speed) {
    fuelFeed.set(speed.value);
  }

  public Command setFeedSpeedCommand(FuelFeedSpeed speed) {
    return Commands.runOnce(() -> setSpeed(speed), this);
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Shooter At Speed", isVelocityWithinTolerance());
    SmartDashboard.putNumber("Shooter RPM", getVelocityRPM());
    SmartDashboard.putNumber("Shooter Target RPM", targetRPM);
  }
  public enum FuelFeedSpeed {
    OFF(0),
    FEED_SLOW(.1),
    FEED_FAST(.8),
    REVERSE(-.1);

    public final double value;
    FuelFeedSpeed(double value) {
      this.value = value;
    }
  }
}
