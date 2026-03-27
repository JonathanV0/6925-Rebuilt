// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.CTREConfigs;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class ClimberSubsys extends SubsystemBase {
  /** Creates a new ClimberSubsys. */
  private static final double kMaxDownPosition = 0.5;  // max negative (CLIMB_DOWN direction)
  private static final double kMaxUpPosition = 0.5;    // max positive (CLIMB_UP / button 5 direction)
  private final TalonFX climber = new TalonFX(12, "");

  public ClimberSubsys() {
    climber.getConfigurator().apply(CTREConfigs.CLIMBER_CONFIG);
    climber.setPosition(0);
  }

  public void setSpeed(ClimberSpeed speed) {
    double position = climber.getPosition().getValueAsDouble();
    if (speed == ClimberSpeed.CLIMB_UP && position >= kMaxUpPosition) {
      climber.set(0);
    } else if (speed == ClimberSpeed.CLIMB_DOWN && position <= -kMaxDownPosition) {
      climber.set(0);
    } else {
      climber.set(speed.value);
    }
  }

  public Command setSpeedCommand(ClimberSpeed speed) {
    return Commands.runOnce(() -> setSpeed(speed), this);
  }

  /** Runs the climber at the given speed while held, stops on release. */
  public Command holdSpeedCommand(ClimberSpeed speed) {
    return Commands.runEnd(
      () -> setSpeed(speed),
      () -> climber.set(0),
      this
    );
  }

  @Override
  public void periodic() {
    double position = climber.getPosition().getValueAsDouble();
    // Hard safety: if the climber is past either limit and still moving that direction, kill it
    if (position >= kMaxUpPosition && climber.get() > 0) {
      climber.set(0);
    } else if (position <= -kMaxDownPosition && climber.get() < 0) {
      climber.set(0);
    }
    SmartDashboard.putNumber("Climber Position", position);
  }

  public enum ClimberSpeed {
    OFF(0.0),
    CLIMB_UP(0.5),
    CLIMB_DOWN(-0.5);

    public final double value;
    ClimberSpeed(double value) {
      this.value = value;
    }
  }
}
