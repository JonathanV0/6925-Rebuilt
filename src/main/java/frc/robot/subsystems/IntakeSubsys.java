// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.CTREConfigs;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class IntakeSubsys extends SubsystemBase {
  /** Creates a new Intake. */
  private final TalonFX intake = new TalonFX(45, "");
  private final TalonFX intakeRotator = new TalonFX(50, "");
  private final PositionVoltage rotatorPositionRequest = new PositionVoltage(0); // Slot0 = gentle
  private final PositionVoltage rotatorOscillateRequest = new PositionVoltage(0).withSlot(1); // Slot1 = snappy

  private double rotatorTargetPosition;

  public IntakeSubsys() {
    intake.getConfigurator().apply(CTREConfigs.INTAKE_CONFIG);
    intakeRotator.getConfigurator().apply(CTREConfigs.INTAKE_ROTATOR_CONFIG);
    rotatorTargetPosition = intakeRotator.getPosition().getValueAsDouble();
    setDefaultCommand(holdPositionCommand());
  }

  /** Default command: continuously drives the rotator to the stored target position. */
  private Command holdPositionCommand() {
    return this.run(() -> {
      intakeRotator.setControl(rotatorPositionRequest.withPosition(rotatorTargetPosition));
    });
  }

  public Command setSpeedCommand(IntakeSpeed speed) {
    return Commands.runOnce(() -> setSpeed(speed), this);
  }

  public void setSpeed(IntakeSpeed speed) {
    intake.set(speed.value);
  }

  /** Runs the intake roller and oscillates the rotator to dislodge balls. Hold to run.
   *  The rotator bounces between the deployed position and slightly above it. */
  public Command intakeWithOscillateCommand(IntakeSpeed speed) {
    // Oscillation range: 5° of output above deployed position
    final double oscillationMotorRotations = (7.0 / 360.0) * 8.0;
    final double[] state = {0, 0}; // [startTime, deployedPosition]
    return this.runEnd(
      () -> {
        intake.set(speed.value);
        if (state[0] == 0) {
          state[0] = Timer.getFPGATimestamp();
          state[1] = intakeRotator.getPosition().getValueAsDouble(); // capture deployed position
        }
        double elapsed = Timer.getFPGATimestamp() - state[0];
        // Alternate between deployed position and slightly above every 0.3s
        boolean goUp = ((int)(elapsed / 0.3) % 2 == 0);
        double target = goUp ? state[1] + oscillationMotorRotations : state[1];
        intakeRotator.setControl(rotatorOscillateRequest.withPosition(target));
      },
      () -> {
        intake.set(0);
        // Return to deployed position
        intakeRotator.setControl(rotatorOscillateRequest.withPosition(state[1]));
        state[0] = 0;
      }
    );
  }

  /** Oscillates the rotator from deployed position upward by 60°, running roller to push balls in.
   *  Hold to run. On release, returns to deployed position. */
  public Command retractWithOscillateCommand(IntakeSpeed speed) {
    final double oscillationMotorRotations = (60.0 / 360.0) * 8.0;
    final double[] state = {0, 0}; // [startTime, deployedPosition]
    return this.runEnd(
      () -> {
        intake.set(speed.value);
        if (state[0] == 0) {
          state[0] = Timer.getFPGATimestamp();
          state[1] = intakeRotator.getPosition().getValueAsDouble(); // capture deployed position
        }
        double elapsed = Timer.getFPGATimestamp() - state[0];
        // Alternate between deployed position and 60° above every 0.3s
        boolean goUp = ((int)(elapsed / 0.3) % 2 == 0);
        double target = goUp ? state[1] + oscillationMotorRotations : state[1];
        intakeRotator.setControl(rotatorOscillateRequest.withPosition(target));
      },
      () -> {
        intake.set(0);
        // Return to deployed position
        intakeRotator.setControl(rotatorOscillateRequest.withPosition(state[1]));
        state[0] = 0;
      }
    );
  }

  /** Runs the intake rotator at a slow duty cycle while held, stops on release.
   *  Updates rotatorTargetPosition so the hold command doesn't snap back. */
  public Command slowRotateCommand(double speed) {
    return Commands.runEnd(
      () -> intakeRotator.set(speed),
      () -> {
        intakeRotator.set(0);
        rotatorTargetPosition = intakeRotator.getPosition().getValueAsDouble();
      },
      this
    );
  }

  /** Rotates the intake rotator CCW by the given degrees from its current position. */
  public Command rotateRotatorCommand(double degrees) {
    return Commands.runOnce(() -> {
      rotatorTargetPosition = intakeRotator.getPosition().getValueAsDouble() + (degrees / 360.0) * 8.0;
    }, this);
  }

  /** Sets target and actively drives the rotator for the given duration. For use in auto. */
  public Command deployIntakeCommand(double degrees, double seconds) {
    return Commands.sequence(
      rotateRotatorCommand(degrees),
      this.run(() -> {
        intakeRotator.setControl(rotatorPositionRequest.withPosition(rotatorTargetPosition));
      }).withTimeout(seconds)
    );
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
