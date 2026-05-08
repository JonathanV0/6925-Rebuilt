package frc.robot;

import static edu.wpi.first.units.Units.Inches;

import java.util.Optional;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Landmarks {

    // Default hub positions (inches). Shown on SmartDashboard for live tuning.
    private static final double kBlueHubX = 182.105;
    private static final double kBlueHubY = 158.845;
    private static final double kRedHubX  = 469.115;
    private static final double kRedHubY  = 158.845;

    /** Call once in robotInit() to push the default values to SmartDashboard. */
    public static void initDashboard() {
        SmartDashboard.putNumber("Hub/Blue X (in)", kBlueHubX);
        SmartDashboard.putNumber("Hub/Blue Y (in)", kBlueHubY);
        SmartDashboard.putNumber("Hub/Red X (in)",  kRedHubX);
        SmartDashboard.putNumber("Hub/Red Y (in)",  kRedHubY);
    }

    public static Translation2d targetPosition() {
        final Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == Alliance.Blue) {
            return new Translation2d(
                Inches.of(SmartDashboard.getNumber("Hub/Blue X (in)", kBlueHubX)),
                Inches.of(SmartDashboard.getNumber("Hub/Blue Y (in)", kBlueHubY)));
        }
        return new Translation2d(
            Inches.of(SmartDashboard.getNumber("Hub/Red X (in)", kRedHubX)),
            Inches.of(SmartDashboard.getNumber("Hub/Red Y (in)", kRedHubY)));
    }
}
