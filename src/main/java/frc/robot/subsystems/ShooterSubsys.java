// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class ShooterSubsys extends SubsystemBase {
  /** Creates a new ShooterSubsys. */
  private final TalonFX fuelShoot0 = new TalonFX(0);
  private final TalonFX fuelShoot1 = new TalonFX(0);
  private final TalonFX fuelShoot2 = new TalonFX(0);

  private final TalonFX fuelFeed0 = new TalonFX(0);
  private final TalonFX fuelFeed1 = new TalonFX(0);

  public ShooterSubsys() {

  }

  public void setSpeed(FuelShootSpeed speed) {
    fuelShoot0.set(speed.value);
    fuelShoot1.set(speed.value);
    fuelShoot2.set(speed.value);
  }

  public void setSpeed(FuelFeedSpeed speed) {
    fuelFeed0.set(speed.value);
    fuelFeed1.set(speed.value);
  }

  public Command setSpeedCommand(FuelShootSpeed speed) {
    return Commands.runOnce(() -> fuelShoot0.set(speed.value), this);
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
