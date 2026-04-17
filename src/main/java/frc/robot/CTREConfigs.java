// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

/** Add your docs here. */
public class CTREConfigs {

    public static final TalonFXConfiguration SHOOTER_CONFIG = new TalonFXConfiguration();       // Motor 8 (right)
    public static final TalonFXConfiguration SHOOTER_CONFIG_9 = new TalonFXConfiguration();    // Motor 9 (middle)
    public static final TalonFXConfiguration SHOOTER_CONFIG_10 = new TalonFXConfiguration();   // Motor 10 (left)
    public static final TalonFXConfiguration INTAKE_CONFIG = new TalonFXConfiguration();
    public static final TalonFXConfiguration INTAKE_ROTATOR_CONFIG = new TalonFXConfiguration();
    public static final TalonFXConfiguration FUEL_FEED_CONFIG = new TalonFXConfiguration();
    public static final TalonFXConfiguration FEEDER_CONFIG = new TalonFXConfiguration();


    static {

    //Shooter config
    SHOOTER_CONFIG.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    SHOOTER_CONFIG.MotorOutput.Inverted = InvertedValue.Clockwise_Positive; // Motor 8 spins opposite from 9/10
    SHOOTER_CONFIG.CurrentLimits.StatorCurrentLimitEnable = true;
    SHOOTER_CONFIG.CurrentLimits.StatorCurrentLimit = 90;
    SHOOTER_CONFIG.CurrentLimits.SupplyCurrentLimitEnable = true;
    SHOOTER_CONFIG.CurrentLimits.SupplyCurrentLimit = 70;
    // Velocity PID — Motor 8 (right)
    SHOOTER_CONFIG.Slot0.kP = 0.77;
    SHOOTER_CONFIG.Slot0.kI = 1.0;
    SHOOTER_CONFIG.Slot0.kV = 0.1;

    // Velocity PID — Motor 9 (middle)
    SHOOTER_CONFIG_9.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    SHOOTER_CONFIG_9.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; // Motors 9/10 opposite from 8
    SHOOTER_CONFIG_9.CurrentLimits.StatorCurrentLimitEnable = true;
    SHOOTER_CONFIG_9.CurrentLimits.StatorCurrentLimit = 90;
    SHOOTER_CONFIG_9.CurrentLimits.SupplyCurrentLimitEnable = true;
    SHOOTER_CONFIG_9.CurrentLimits.SupplyCurrentLimit = 70;
    SHOOTER_CONFIG_9.Slot0.kP = 0.77;
    SHOOTER_CONFIG_9.Slot0.kI = 1.0;
    SHOOTER_CONFIG_9.Slot0.kV = 0.1;

    // Velocity PID — Motor 10 (left)
    SHOOTER_CONFIG_10.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    SHOOTER_CONFIG_10.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; // Motors 9/10 opposite from 8
    SHOOTER_CONFIG_10.CurrentLimits.StatorCurrentLimitEnable = true;
    SHOOTER_CONFIG_10.CurrentLimits.StatorCurrentLimit = 90;
    SHOOTER_CONFIG_10.CurrentLimits.SupplyCurrentLimitEnable = true;
    SHOOTER_CONFIG_10.CurrentLimits.SupplyCurrentLimit = 70;
    SHOOTER_CONFIG_10.Slot0.kP = 0.77;
    SHOOTER_CONFIG_10.Slot0.kI = 1.0;
    SHOOTER_CONFIG_10.Slot0.kV = 0.1;

    //Intake config
    INTAKE_CONFIG.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    INTAKE_CONFIG.CurrentLimits.StatorCurrentLimitEnable = true;
    INTAKE_CONFIG.CurrentLimits.StatorCurrentLimit = 80;
    INTAKE_CONFIG.CurrentLimits.SupplyCurrentLimitEnable = true;
    INTAKE_CONFIG.CurrentLimits.SupplyCurrentLimit = 80;
    //Intake rotator config
    INTAKE_ROTATOR_CONFIG.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    INTAKE_ROTATOR_CONFIG.CurrentLimits.StatorCurrentLimitEnable = true;
    INTAKE_ROTATOR_CONFIG.CurrentLimits.StatorCurrentLimit =80;
    INTAKE_ROTATOR_CONFIG.CurrentLimits.SupplyCurrentLimitEnable = true;
    INTAKE_ROTATOR_CONFIG.CurrentLimits.SupplyCurrentLimit = 80;
    // Slot0: deploy/retract — gentle
    INTAKE_ROTATOR_CONFIG.Slot0.kP = 1.0;
    INTAKE_ROTATOR_CONFIG.Slot0.kV = 12.0 / (6000.0 / 60.0 / 8.0);  // 12V / max pivot RPS (6000 RPM / 8:1 reduction)
    // Slot1: oscillation
    INTAKE_ROTATOR_CONFIG.Slot1.kP = 2;
    INTAKE_ROTATOR_CONFIG.Slot1.kD = 0.5;

    //Fuel feed config (shooter subsystem motor 11)
    FUEL_FEED_CONFIG.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    FUEL_FEED_CONFIG.CurrentLimits.StatorCurrentLimitEnable = true;
    FUEL_FEED_CONFIG.CurrentLimits.StatorCurrentLimit = 50;
    FUEL_FEED_CONFIG.CurrentLimits.SupplyCurrentLimitEnable = true;
    FUEL_FEED_CONFIG.CurrentLimits.SupplyCurrentLimit = 40;

    //Feeder config
    FEEDER_CONFIG.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    FEEDER_CONFIG.CurrentLimits.StatorCurrentLimitEnable = true;
    FEEDER_CONFIG.CurrentLimits.StatorCurrentLimit = 40;
    FEEDER_CONFIG.CurrentLimits.SupplyCurrentLimitEnable = true;
    FEEDER_CONFIG.CurrentLimits.SupplyCurrentLimit = 40;

    }
}
