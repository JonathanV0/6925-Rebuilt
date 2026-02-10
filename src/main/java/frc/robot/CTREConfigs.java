// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;

/** Add your docs here. */
public class CTREConfigs {

    public static final TalonFXConfiguration SHOOTER_CONFIG = new TalonFXConfiguration();
    public static final TalonFXConfiguration INTAKE_CONFIG = new TalonFXConfiguration();
    public static final TalonFXConfiguration FEEDER_CONFIG = new TalonFXConfiguration();
    public static final TalonFXConfiguration CLIMBER_CONFIG  = new TalonFXConfiguration();

    static {
        //Shooter config
    SHOOTER_CONFIG.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    SHOOTER_CONFIG.CurrentLimits.StatorCurrentLimitEnable = true;
    SHOOTER_CONFIG.CurrentLimits.StatorCurrentLimit = 60;

    // Velocity PID for shooter flywheels
    SHOOTER_CONFIG.Slot0.kP = 0.1;   // Proportional gain (tune this)
    SHOOTER_CONFIG.Slot0.kI = 0.0;   // Integral gain
    SHOOTER_CONFIG.Slot0.kD = 0.0;   // Derivative gain
    SHOOTER_CONFIG.Slot0.kV = 0.12;  // Velocity feedforward (tune this)
    SHOOTER_CONFIG.Slot0.kS = 0.25;  // Static friction feedforward

    //Intake config
    INTAKE_CONFIG.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    INTAKE_CONFIG.CurrentLimits.StatorCurrentLimitEnable = true;
    INTAKE_CONFIG.CurrentLimits.StatorCurrentLimit = 60;

    //Feeder config
    FEEDER_CONFIG.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    FEEDER_CONFIG.CurrentLimits.StatorCurrentLimitEnable = true;
    FEEDER_CONFIG.CurrentLimits.StatorCurrentLimit = 60;
    }
}
