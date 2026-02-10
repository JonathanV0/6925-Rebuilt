// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.CTREConfigs;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class ShooterSubsys extends SubsystemBase {
  /** Creates a new ShooterSubsys. */
  private final TalonFX fuelShoot = new TalonFX(0, "CANivore");
  private final TalonFX fuelShoot0 = new TalonFX(0, "CANivore");
  private final TalonFX fuelShoot1 = new TalonFX(0, "CANivore");

  private final TalonFX fuelFeed0 = new TalonFX(0, "CANivore");
  private final TalonFX fuelFeed1 = new TalonFX(0, "CANivore");

  
  public ShooterSubsys() {

    fuelShoot.getConfigurator().apply(CTREConfigs.SHOOTER_CONFIG);
    fuelShoot0.getConfigurator().apply(CTREConfigs.SHOOTER_CONFIG);
    fuelShoot1.getConfigurator().apply(CTREConfigs.SHOOTER_CONFIG);
    // TODO: Configure fuelShoot0 and fuelShoot1 as followers once Phoenix 6 libraries are properly loaded
  }

  // ========== AI GENERATED - PLEASE DOUBLE CHECK ==========

  // Velocity control request for PID
  private final VelocityVoltage m_velocityRequest = new VelocityVoltage(0);

  // Velocity PID control - Use this for precise RPM control in competition
  public void setVelocityRPM(double rpm) {
    double rps = rpm / 60.0; // Convert RPM to rotations per second
    fuelShoot.setControl(m_velocityRequest.withVelocity(rps));
    fuelShoot0.setControl(m_velocityRequest.withVelocity(rps));
    fuelShoot1.setControl(m_velocityRequest.withVelocity(rps));
  }

  public void setVelocityRPS(double rps) {
    fuelShoot.setControl(m_velocityRequest.withVelocity(rps));
    fuelShoot0.setControl(m_velocityRequest.withVelocity(rps));
    fuelShoot1.setControl(m_velocityRequest.withVelocity(rps));
  }

  // Commands for button binding
  public Command setVelocityRPMCommand(double rpm) {
    return Commands.runOnce(() -> setVelocityRPM(rpm), this);
  }

  public void stopShooter() {
    fuelShoot.set(0);
    fuelShoot0.set(0);
    fuelShoot1.set(0);
  }
  // ========== END AI GENERATED CODE ==========

  // Simple percent output control - Good for initial testing
  public void setSpeed(FuelShootSpeed speed) {
    fuelShoot.set(speed.value);
    fuelShoot0.set(speed.value);
    fuelShoot1.set(speed.value);
  }

  public void setSpeed(FuelFeedSpeed speed) {
    fuelFeed0.set(speed.value);
    fuelFeed1.set(speed.value);
  }

  public Command setSpeedCommand(FuelShootSpeed speed) {
    return Commands.runOnce(() -> fuelShoot.set(speed.value), this);
  }

  @Override
  public void periodic() {}
  public enum FuelShootSpeed {
    OFF(0.0),
    SHOOT_MAX(1.0),
    SHOOT_HALF(0.5);

    public final double value;
    FuelShootSpeed(double value) {
      this.value = value;
      }
  }
  public enum FuelFeedSpeed {
    OFF(0),
    FEED_SLOW(.1),
    FEED_FAST(.4);

    public final double value;
    FuelFeedSpeed(double value) {
      this.value = value;
    }
  }
}
