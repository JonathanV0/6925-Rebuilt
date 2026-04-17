// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.CTREConfigs;

public class FeederSubsys extends SubsystemBase {
  private final TalonFX feeder0 = new TalonFX(51, "CANivore");
  private final TalonFX fuelFeed = new TalonFX(11, "CANivore");

  public FeederSubsys() {
    feeder0.getConfigurator().apply(CTREConfigs.FEEDER_CONFIG);
    fuelFeed.getConfigurator().apply(CTREConfigs.FUEL_FEED_CONFIG);
  }

  public void setSpeed(FeederSpeed speed) {
    feeder0.set(speed.value);
    fuelFeed.set(speed.fuelFeedValue);
  }

  public Command setSpeedCommand(FeederSpeed speed) {
    return Commands.runOnce(() -> setSpeed(speed), this);
  }

  @Override
  public void periodic() {
    
  }

  public enum FeederSpeed {
    OFF(0.0, 0.0),
    FEED_SLOW(-0.3, 0.1),
    FEED_FAST(-0.6, 0.6),
    REVERSE(0.3, -0.1);

    public final double value;
    public final double fuelFeedValue;
    FeederSpeed(double value, double fuelFeedValue) {
      this.value = value;
      this.fuelFeedValue = fuelFeedValue;
    }
  }
}
