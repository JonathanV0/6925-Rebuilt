// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.VelocityVoltage;
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



  private static final double kIdleRPM = 3000;
  private boolean idleEnabled = true;

  public ShooterSubsys() {
    fuelShoot.getConfigurator().apply(CTREConfigs.SHOOTER_CONFIG);
    fuelShoot0.getConfigurator().apply(CTREConfigs.SHOOTER_CONFIG_9);
    fuelShoot1.getConfigurator().apply(CTREConfigs.SHOOTER_CONFIG_10);
    setDefaultCommand(Commands.run(() -> {
      if (idleEnabled) setVelocityRPM(kIdleRPM);
      else stopShooter();
    }, this));
  }

  public void toggleIdle() {
    idleEnabled = !idleEnabled;
  }

  // Each motor gets its own velocity PID so they independently hold RPM
  private final VelocityVoltage m_velocityRequest = new VelocityVoltage(0);
  private final VelocityVoltage m_velocityRequest0 = new VelocityVoltage(0);
  private final VelocityVoltage m_velocityRequest1 = new VelocityVoltage(0);

  public void setVelocityRPM(double rpm) {
    targetRPM = rpm;
    double rps = rpm / 60.0;
    fuelShoot.setControl(m_velocityRequest.withVelocity(rps));
    fuelShoot0.setControl(m_velocityRequest0.withVelocity(rps));
    fuelShoot1.setControl(m_velocityRequest1.withVelocity(rps));
  }

  /** Spins only the right (leader) motor at the given RPM. Others are stopped. */
  public void setRightMotorOnly(double rpm) {
    fuelShoot0.stopMotor();
    fuelShoot1.stopMotor();
    double rps = rpm / 60.0;
    fuelShoot.setControl(m_velocityRequest.withVelocity(rps));
  }

  /** Neutralizes the shooter motors — they will coast to a stop. */
  public void stopShooter() {
    targetRPM = 0;
    fuelShoot.stopMotor();
    fuelShoot0.stopMotor();
    fuelShoot1.stopMotor();
  }

  // Commands for button binding
  public Command setVelocityRPMCommand(double rpm) {
    return Commands.runOnce(() -> setVelocityRPM(rpm), this);
  }

  private static final double kVelocityToleranceRPM = 300.0;

  private double targetRPM = 0;

  public double getVelocityRPM() {
    return fuelShoot.getVelocity().getValueAsDouble() * 60.0;
  }

  public double getMotor9RPM() {
    return fuelShoot0.getVelocity().getValueAsDouble() * 60.0;
  }

  public double getMotor10RPM() {
    return fuelShoot1.getVelocity().getValueAsDouble() * 60.0;
  }

  private boolean isMotorAtSpeed(double motorRPM) {
    // Require a positive target — prevents false-positive when shooter is idle (0 RPM = "at speed")
    return targetRPM > 0 && Math.abs(motorRPM - targetRPM) < kVelocityToleranceRPM;
  }

  public boolean isMotor8AtSpeed()  { return isMotorAtSpeed(getVelocityRPM()); }
  public boolean isMotor9AtSpeed()  { return isMotorAtSpeed(getMotor9RPM()); }
  public boolean isMotor10AtSpeed() { return isMotorAtSpeed(getMotor10RPM()); }

  public boolean isVelocityWithinTolerance() {
    // Each motor runs its own velocity PID, so one can lag while motor 8 reads "at speed".
    // A ball touching a slow wheel gets a bad shot, so all three must be ready.
    return isMotor8AtSpeed() && isMotor9AtSpeed() && isMotor10AtSpeed();
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Shooter At Speed", isVelocityWithinTolerance());
    SmartDashboard.putBoolean("Shooter At Speed Motor 8", isMotor8AtSpeed());
    SmartDashboard.putBoolean("Shooter At Speed Motor 9", isMotor9AtSpeed());
    SmartDashboard.putBoolean("Shooter At Speed Motor 10", isMotor10AtSpeed());
    SmartDashboard.putBoolean("Shooter Idle On", idleEnabled);
    SmartDashboard.putNumber("Shooter RPM", getVelocityRPM());
    SmartDashboard.putNumber("Shooter RPM Motor 9", getMotor9RPM());
    SmartDashboard.putNumber("Shooter RPM Motor 10", getMotor10RPM());
    SmartDashboard.putNumber("Shooter Target RPM", targetRPM);
  }

  
}
