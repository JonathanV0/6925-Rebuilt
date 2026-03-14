// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.CTREConfigs;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class IntakeSubsys extends SubsystemBase {
  /** Creates a new Intake. */
  private final TalonFX intake = new TalonFX(45, "");
  private final TalonFX intakeRotator = new TalonFX(50, "");
  private final PositionVoltage rotatorPositionRequest = new PositionVoltage(0);

  public IntakeSubsys() {
    intake.getConfigurator().apply(CTREConfigs.INTAKE_CONFIG);
    intakeRotator.getConfigurator().apply(CTREConfigs.INTAKE_ROTATOR_CONFIG);
  }

  public Command setSpeedCommand(IntakeSpeed speed) {
    return Commands.runOnce(() -> setSpeed(speed), this);
  }

  public void setSpeed(IntakeSpeed speed) {
    intake.set(speed.value);
  }

  /** Rotates the intake rotator CCW by the given degrees from its current position. */
  public Command rotateRotatorCommand(double degrees) {
    return Commands.runOnce(() -> {
      double target = intakeRotator.getPosition().getValueAsDouble() + (degrees / 360.0);
      intakeRotator.setControl(rotatorPositionRequest.withPosition(target));
    }, this);
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Intake Running", intake.get() != 0);
  }

  public enum IntakeSpeed {
    OFF(0),
    INTAKE_SLOW(-.1),
    INTAKE_MID(-.25),
    INTAKE_FAST(-.5),
    REVERSE(.25);

    public final double value;
    IntakeSpeed(double value) {
      this.value = value;
    }
  }
}
