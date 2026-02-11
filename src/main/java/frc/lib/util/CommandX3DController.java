// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.util;


import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class CommandX3DController extends CommandGenericHID {
    public CommandX3DController(int port) {
        super(port);
    }

    public Trigger trigger() {
        return button(1);
    }

    public double getPitch() {
        return getHID().getRawAxis(1);
    }

    public double getRoll() {
        return getHID().getRawAxis(0);
    }

    public double getYaw() {
        return getHID().getRawAxis(2);
    }

    public double getSlider() {
        return -getHID().getRawAxis(3);
    }
}
