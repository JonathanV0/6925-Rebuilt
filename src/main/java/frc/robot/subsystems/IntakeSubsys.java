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
    // Oscillation range: 15° of output above deployed position
    final double oscillationMotorRotations = (11.5 / 360.0) * 8.0;
    final double[] state = {0, 0}; // [startTime, deployedPosition]
    return this.runEnd(
      () -> {
        intake.set(speed.value);
        if (state[0] == 0) {
          state[0] = Timer.getFPGATimestamp();
          state[1] = intakeRotator.getPosition().getValueAsDouble(); // capture deployed position
        }
        double elapsed = Timer.getFPGATimestamp() - state[0];
        // Alternate between deployed position and slightly above every 0.10s
        boolean goUp = ((int)(elapsed / 0.10) % 2 == 0);
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
    final double oscillationMotorRotations = (90.0 / 360.0) * 8.0;
    final double[] state = {0, 0}; // [startTime, deployedPosition]
    return this.runEnd(
      () -> {
        intake.set(speed.value);
        if (state[0] == 0) {
          state[0] = Timer.getFPGATimestamp();
          state[1] = intakeRotator.getPosition().getValueAsDouble(); // capture deployed position
        }
        double elapsed = Timer.getFPGATimestamp() - state[0];
        // Alternate between deployed position and 90° above every 0.3s
        boolean goUp = ((int)(elapsed / 0.3) % 2 == 0);
        double target = goUp ? state[1] + oscillationMotorRotations : state[1];
        intakeRotator.setControl(rotatorOscillateRequest.withPosition(target));
      },
      () -> {
        intake.set(0);
        intakeRotator.setControl(rotatorOscillateRequest.withPosition(state[1]));
        state[0] = 0;
      }
    );
  }

  /** Gentle oscillation for use during shooting — slower period, wider sweep, softer PID */
  public Command retractWithGentleOscillateCommand(IntakeSpeed speed) {
    final double oscillationMotorRotations = (160.0 / 360.0) * 8.0;
    final double[] state = {0, 0}; // [startTime, deployedPosition]
    return this.runEnd(
      () -> {
        intake.set(speed.value);
        if (state[0] == 0) {
          state[0] = Timer.getFPGATimestamp();
          state[1] = intakeRotator.getPosition().getValueAsDouble();
        }
        double elapsed = Timer.getFPGATimestamp() - state[0];
        // Alternate between deployed position and 160° above every 0.75s
        boolean goUp = ((int)(elapsed / 0.75) % 2 == 0);
        double target = goUp ? state[1] + oscillationMotorRotations : state[1];
        intakeRotator.setControl(rotatorPositionRequest.withPosition(target));
      },
      () -> {
        intake.set(0);
        intakeRotator.setControl(rotatorPositionRequest.withPosition(state[1]));
        state[0] = 0;
      }
    );
  }

  /** Auto-only: bounces the intake a set number of times at 0.5s intervals, then stops.
   *  Like spamming button 12 repeatedly. */
  public Command autoBounceCommand(int bounces) {
    final double oscillationMotorRotations = (60.0 / 360.0) * 8.0;
    final double period = 0.8; // seconds per bounce cycle
    final double totalTime = bounces * period;
    final double[] state = {0, 0}; // [startTime, deployedPosition]
    return this.run(() -> {
        intake.set(IntakeSpeed.INTAKE_FAST.value);
        if (state[0] == 0) {
          state[0] = Timer.getFPGATimestamp();
          state[1] = intakeRotator.getPosition().getValueAsDouble();
        }
        double elapsed = Timer.getFPGATimestamp() - state[0];
        boolean goUp = ((int)(elapsed / (period / 2.0)) % 2 == 0);
        double target = goUp ? state[1] + oscillationMotorRotations : state[1];
        intakeRotator.setControl(rotatorOscillateRequest.withPosition(target));
      })
      .withTimeout(totalTime)
      .finallyDo(() -> {
        intake.set(0);
        intakeRotator.setControl(rotatorOscillateRequest.withPosition(state[1]));
        state[0] = 0;
      });
  }

  /** Runs the intake rotator at a slow duty cycle while held, stops on release.
   *  Updates rotatorTargetPosition so the hold command doesn't snap back. */
  public Command slowRotateCommand(double speed) {
    return Commands.runEnd(
      () -> intakeRotator.set(speed),
      () -> {
        rotatorTargetPosition = intakeRotator.getPosition().getValueAsDouble();
        intakeRotator.setControl(rotatorPositionRequest.withPosition(rotatorTargetPosition));
      },
      this
    );
  }

  /** Creeps the rotator at ~1 RPM motor using position steps. Hold to run, holds position on release. */
  public Command creepRotateCommand(double direction) {
    // 30 RPM motor. At 50Hz loop, each cycle moves 30/(60*50) = 1/100 rotations
    final double stepPerCycle = (1.0 / 100.0) * Math.signum(direction);
    return Commands.runEnd(
      () -> {
        rotatorTargetPosition += stepPerCycle;
        intakeRotator.setControl(rotatorPositionRequest.withPosition(rotatorTargetPosition));
      },
      () -> {
        rotatorTargetPosition = intakeRotator.getPosition().getValueAsDouble();
        intakeRotator.setControl(rotatorPositionRequest.withPosition(rotatorTargetPosition));
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

  /** Moves the intake rotator to an absolute motor position (in rotations) instantly. */
  public Command goToPositionCommand(double motorRotations) {
    return Commands.runOnce(() -> {
      rotatorTargetPosition = motorRotations;
      intakeRotator.setControl(rotatorPositionRequest.withPosition(rotatorTargetPosition));
    }, this);
  }

  /** Drives the rotator toward an absolute position at a fixed duty cycle, then holds with PID.
   *  Finishes automatically when it reaches the target. Single press to activate. */
  public Command goToPositionSlowCommand(double motorRotations, double speed) {
    return this.run(() -> {
        double current = intakeRotator.getPosition().getValueAsDouble();
        if (Math.abs(current - motorRotations) < 0.5) {
          // Close enough — switch to PID hold
          rotatorTargetPosition = motorRotations;
          intakeRotator.setControl(rotatorPositionRequest.withPosition(rotatorTargetPosition));
        } else {
          // Drive toward target at fixed speed
          double direction = (motorRotations > current) ? 1.0 : -1.0;
          intakeRotator.set(speed * direction);
        }
      })
      .until(() -> Math.abs(intakeRotator.getPosition().getValueAsDouble() - motorRotations) < 0.5)
      .finallyDo(() -> {
        rotatorTargetPosition = motorRotations;
        intakeRotator.setControl(rotatorPositionRequest.withPosition(rotatorTargetPosition));
      });
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
    SmartDashboard.putNumber("Intake Rotator Position", intakeRotator.getPosition().getValueAsDouble());
  }

  public enum IntakeSpeed {
    OFF(0),
    INTAKE_SLOW(-.1),
    INTAKE_MID(-.25),
    INTAKE_FAST(-.7),
    REVERSE(.25);

    public final double value;
    IntakeSpeed(double value) {
      this.value = value;
    }
  }
}
