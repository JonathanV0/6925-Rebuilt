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
    public static final TalonFXConfiguration INTAKE_ROTATOR_CONFIG = new TalonFXConfiguration();
    public static final TalonFXConfiguration FUEL_FEED_CONFIG = new TalonFXConfiguration();
    public static final TalonFXConfiguration FEEDER_CONFIG = new TalonFXConfiguration();
    public static final TalonFXConfiguration CLIMBER_CONFIG  = new TalonFXConfiguration();

    static {

    //Shooter config
    SHOOTER_CONFIG.MotorOutput.NeutralMode = NeutralModeValue.Coast; // Coast for flywheel — spins down naturally, less heat, faster recovery
    SHOOTER_CONFIG.CurrentLimits.StatorCurrentLimitEnable = true;
    SHOOTER_CONFIG.CurrentLimits.StatorCurrentLimit = 120;
    SHOOTER_CONFIG.CurrentLimits.SupplyCurrentLimitEnable = true;
    SHOOTER_CONFIG.CurrentLimits.SupplyCurrentLimit = 70;
    // Velocity PID — aggressive WCP CC values
    SHOOTER_CONFIG.Slot0.kP = 0.5;
    SHOOTER_CONFIG.Slot0.kI = 2.0;
    SHOOTER_CONFIG.Slot0.kV = 0.12;

    //Intake config
    INTAKE_CONFIG.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    INTAKE_CONFIG.CurrentLimits.StatorCurrentLimitEnable = true;
    INTAKE_CONFIG.CurrentLimits.StatorCurrentLimit = 60;
    INTAKE_CONFIG.CurrentLimits.SupplyCurrentLimitEnable = true;
    INTAKE_CONFIG.CurrentLimits.SupplyCurrentLimit = 60;
    //Intake rotator config
    INTAKE_ROTATOR_CONFIG.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    INTAKE_ROTATOR_CONFIG.CurrentLimits.StatorCurrentLimitEnable = true;
    INTAKE_ROTATOR_CONFIG.CurrentLimits.StatorCurrentLimit = 60;
    INTAKE_ROTATOR_CONFIG.CurrentLimits.SupplyCurrentLimitEnable = true;
    INTAKE_ROTATOR_CONFIG.CurrentLimits.SupplyCurrentLimit = 60;
    // Position PID — soft values to avoid slamming
    INTAKE_ROTATOR_CONFIG.Slot0.kP = 10;
    INTAKE_ROTATOR_CONFIG.Slot0.kV = 12.0 / (6000.0 / 60.0 / 50.0);  // 12V / max pivot RPS (6000 RPM / 50:1 reduction)

    //Fuel feed config (shooter subsystem motor 11)
    FUEL_FEED_CONFIG.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    FUEL_FEED_CONFIG.CurrentLimits.StatorCurrentLimitEnable = true;
    FUEL_FEED_CONFIG.CurrentLimits.StatorCurrentLimit = 50;
    FUEL_FEED_CONFIG.CurrentLimits.SupplyCurrentLimitEnable = true;
    FUEL_FEED_CONFIG.CurrentLimits.SupplyCurrentLimit = 40;

    //Feeder config
    FEEDER_CONFIG.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    FEEDER_CONFIG.CurrentLimits.StatorCurrentLimitEnable = true;
    FEEDER_CONFIG.CurrentLimits.StatorCurrentLimit = 50;
    FEEDER_CONFIG.CurrentLimits.SupplyCurrentLimitEnable = true;
    FEEDER_CONFIG.CurrentLimits.SupplyCurrentLimit = 40;

    //Climber config
    CLIMBER_CONFIG.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    CLIMBER_CONFIG.CurrentLimits.StatorCurrentLimitEnable = true;
    CLIMBER_CONFIG.CurrentLimits.StatorCurrentLimit = 60;
    CLIMBER_CONFIG.CurrentLimits.SupplyCurrentLimitEnable = true;
    CLIMBER_CONFIG.CurrentLimits.SupplyCurrentLimit = 60;

    }
}
