package frc.robot.util;

import java.util.Map;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;

/**
 * Operator-facing live trims, modeled on 2910's OperatorDashboard.
 * The RPM trim exists because a match-day flywheel (worn wheels, cold balls, low battery)
 * rarely matches the day the table was calibrated; a percent knob fixes that without redeploying.
 */
public class OperatorDashboard {
    private final ShuffleboardTab tab;
    private final SimpleWidget rpmPercentAdder;

    public OperatorDashboard() {
        tab = Shuffleboard.getTab("Operator");
        rpmPercentAdder = tab
            .add("RPM Percent Adder", 0.0)
            .withSize(5, 3)
            .withPosition(1, 1)
            .withWidget(BuiltInWidgets.kNumberSlider)
            .withProperties(Map.of("min", -10.0, "max", 50.0));
    }

    /** 1.0 at the slider's default; e.g. +5 on the slider returns 1.05. */
    public double getRPMMultiplier() {
        return 1.0 + rpmPercentAdder.getEntry().getDouble(0.0) / 100.0;
    }
}
