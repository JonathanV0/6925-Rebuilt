// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.CTREConfigs;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class ShooterSubsys extends SubsystemBase {
  /** Creates a new ShooterSubsys. */
  private final TalonFX fuelShoot = new TalonFX(8, "");
  private final TalonFX fuelShoot0 = new TalonFX(9, "");
  private final TalonFX fuelShoot1 = new TalonFX(10, "");



  public ShooterSubsys() {

    fuelShoot.getConfigurator().apply(CTREConfigs.SHOOTER_CONFIG);
    fuelShoot0.getConfigurator().apply(CTREConfigs.SHOOTER_CONFIG_9);
    fuelShoot1.getConfigurator().apply(CTREConfigs.SHOOTER_CONFIG_10);
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

  private static final double kVelocityToleranceRPM = 100.0;
  private static final double kJamRPMDropThreshold = 500.0; // RPM drop that indicates a jam
  private static final double kJamReverseRPM = -500.0;      // RPM to reverse at when clearing jam
  private static final double kJamReverseDuration = 0.3;     // seconds to reverse
  private static final double kJamCooldown = 1.0;            // seconds before checking for jams again

  private double targetRPM = 0;
  private boolean isClearing = false;
  private double clearStartTime = 0;
  private double lastClearTime = 0;
  private double savedTargetRPM = 0;

  public double getVelocityRPM() {
    return fuelShoot.getVelocity().getValueAsDouble() * 60.0;
  }

  public boolean isVelocityWithinTolerance() {
    // Require a positive target — prevents false-positive when shooter is idle (0 RPM = "at speed")
    return targetRPM > 0 && Math.abs(getVelocityRPM() - targetRPM) < kVelocityToleranceRPM;
  }
  // ========== END AI GENERATED CODE ==========

  @Override
  public void periodic() {
    double now = Timer.getFPGATimestamp();
    double currentRPM = getVelocityRPM();

    if (isClearing) {
      // Currently reversing to clear jam — check if done
      if (now - clearStartTime >= kJamReverseDuration) {
        isClearing = false;
        lastClearTime = now;
        setVelocityRPM(savedTargetRPM); // resume original RPM
      }
    } else if (targetRPM > 500 && (now - lastClearTime) > kJamCooldown) {
      // Check for jam: shooter should be spinning but RPM dropped significantly
      if (currentRPM < targetRPM - kJamRPMDropThreshold) {
        isClearing = true;
        clearStartTime = now;
        savedTargetRPM = targetRPM;
        setVelocityRPM(kJamReverseRPM);
        SmartDashboard.putBoolean("Shooter Jammed", true);
      }
    }

    if (!isClearing) {
      SmartDashboard.putBoolean("Shooter Jammed", false);
    }

    SmartDashboard.putBoolean("Shooter At Speed", isVelocityWithinTolerance());
    SmartDashboard.putNumber("Shooter RPM", currentRPM);
    SmartDashboard.putNumber("Shooter Target RPM", targetRPM);
  }
}
