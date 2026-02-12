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

    public CTREConfigs() {

    //Shooter config
    SHOOTER_CONFIG.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    SHOOTER_CONFIG.CurrentLimits.StatorCurrentLimitEnable = true;
    SHOOTER_CONFIG.CurrentLimits.StatorCurrentLimit = 60;
    SHOOTER_CONFIG.CurrentLimits.SupplyCurrentLimitEnable = true;
    SHOOTER_CONFIG.CurrentLimits.SupplyCurrentLimit = 60;

    //Intake config
    INTAKE_CONFIG.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    INTAKE_CONFIG.CurrentLimits.StatorCurrentLimitEnable = true;
    INTAKE_CONFIG.CurrentLimits.StatorCurrentLimit = 60;
    INTAKE_CONFIG.CurrentLimits.SupplyCurrentLimitEnable = true;
    INTAKE_CONFIG.CurrentLimits.SupplyCurrentLimit = 60;

    //Feeder config
    FEEDER_CONFIG.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    FEEDER_CONFIG.CurrentLimits.StatorCurrentLimitEnable = true;
    FEEDER_CONFIG.CurrentLimits.StatorCurrentLimit = 60;
    FEEDER_CONFIG.CurrentLimits.SupplyCurrentLimitEnable = true;
    FEEDER_CONFIG.CurrentLimits.SupplyCurrentLimit = 60;

    //Climber config
    CLIMBER_CONFIG.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    CLIMBER_CONFIG.CurrentLimits.StatorCurrentLimitEnable = true;
    CLIMBER_CONFIG.CurrentLimits.StatorCurrentLimit = 60;
    CLIMBER_CONFIG.CurrentLimits.SupplyCurrentLimitEnable = true;
    CLIMBER_CONFIG.CurrentLimits.SupplyCurrentLimit = 60;

    }
}
