// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.CTREConfigs;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class IntakeSubsys extends SubsystemBase {
  /** Creates a new Intake. */
  private final TalonFX intake = new TalonFX(0, "CANivore");

  public IntakeSubsys() {
    intake.getConfigurator().apply(CTREConfigs.INTAKE_CONFIG);
  }

  public Command setSpeedCommand(IntakeSpeed speed) {
    return Commands.runOnce(() -> setSpeed(speed), this);
  }

  public void setSpeed(IntakeSpeed speed) {
    intake.set(speed.value);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public enum IntakeSpeed {
    OFF(0),
    INTAKE_SLOW(.1),
    INTAKE_MID(.25);

    public final double value;
    IntakeSpeed(double value) {
      this.value = value;
    }
  }
}
