// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.CTREConfigs;

public class FeederSubsys extends SubsystemBase {
  private final TalonFX feeder0 = new TalonFX(0, "CANivore");
  private final TalonFX feeder1 = new TalonFX(0, "CANivore");

  public FeederSubsys() {
    feeder0.getConfigurator().apply(CTREConfigs.FEEDER_CONFIG);
    feeder1.getConfigurator().apply(CTREConfigs.FEEDER_CONFIG);
  }

  public void setSpeed(FeederSpeed speed) {
    feeder0.set(speed.value);
    feeder1.set(speed.value);
  }

  public Command setSpeedCommand(FeederSpeed speed) {
    return Commands.runOnce(() -> setSpeed(speed), this);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public enum FeederSpeed {
    OFF(0.0),
    FEED_SLOW(0.3),
    FEED_FAST(0.7);

    public final double value;
    FeederSpeed(double value) {
      this.value = value;
    }
  }
}
