// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;

/** Add your docs here. */
public class Configs {

    public final TalonFXConfiguration SHOOTER_CONFIG = new TalonFXConfiguration();
    public final TalonFXConfiguration INTAKE_CONFIG = new TalonFXConfiguration();
    public final TalonFXConfiguration FEEDER_CONFIG = new TalonFXConfiguration();
    public final TalonFXConfiguration CLIMBER_CONFIG  = new TalonFXConfiguration();

    public Configs() {

    //Shooter config
    SHOOTER_CONFIG.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    SHOOTER_CONFIG.CurrentLimits.StatorCurrentLimitEnable = true;
    SHOOTER_CONFIG.CurrentLimits.StatorCurrentLimit = 60;

    //Intake config
    }
}
