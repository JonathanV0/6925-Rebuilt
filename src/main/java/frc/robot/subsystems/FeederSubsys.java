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
    FEED_FAST(-0.8, 0.8),
    FEED_TURBO(-1.0, 1.0),
    REVERSE(0.5, -0.4),
    // Pre-shot back-off (from 1678): while the flywheel spins up, gently pull the ball at the
    // wheel away from it so it doesn't drag RPM down or get flicked out short. Only the
    // fuel-feed roller reverses; the main feeder stays off so balls don't retreat to the hopper.
    // TODO(tune): bench-check the ball backs off ~1 cm and STAYS in the feeder. If it falls
    //   back out, reduce toward -0.1; if the flywheel still rubs the ball, raise toward -0.3.
    PRESHOT_REVERSE(0.0, -0.2);

    public final double value;
    public final double fuelFeedValue;
    FeederSpeed(double value, double fuelFeedValue) {
      this.value = value;
      this.fuelFeedValue = fuelFeedValue;
    }
  }
}
