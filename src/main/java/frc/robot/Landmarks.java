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

    /** Single place for the alliance decision so every landmark flips the same way. Unknown = red. */
    private static boolean isBlueAlliance() {
        final Optional<Alliance> alliance = DriverStation.getAlliance();
        return alliance.isPresent() && alliance.get() == Alliance.Blue;
    }

    public static Translation2d targetPosition() {
        if (isBlueAlliance()) {
            return new Translation2d(
                Inches.of(SmartDashboard.getNumber("Hub/Blue X (in)", kBlueHubX)),
                Inches.of(SmartDashboard.getNumber("Hub/Blue Y (in)", kBlueHubY)));
        }
        return new Translation2d(
            Inches.of(SmartDashboard.getNumber("Hub/Red X (in)", kRedHubX)),
            Inches.of(SmartDashboard.getNumber("Hub/Red Y (in)", kRedHubY)));
    }

    // How far from the trench tag (toward the hub) the robot bumper sits when shooting there
    // TODO(tune): kTrenchSpotOffsetMeters. Should be the distance from the tag's wall to the
    //   robot CENTER when parked in the trench spot: ~ half the robot length + bumper +
    //   any gap. Measure with a tape once, on the practice field. Also confirm the tag
    //   positions in KnownSpot below match the real field (welded vs. AndyMark layout).
    private static final double kTrenchSpotOffsetMeters = 0.5;

    /**
     * Field spots the robot can be parked at by feel, used to re-seed odometry when
     * vision is unavailable (2910's "scoring from known translation"). LEFT/RIGHT are
     * from the DRIVER's point of view, which is why the blue and red tags swap sides.
     * Each enum entry carries its own data (an enum with fields), so adding a spot is one line.
     */
    public enum KnownSpot {
        // TODO: verify on practice field. Tag positions from WPILib's 2026-rebuilt-welded layout.
        LEFT_TRENCH (new Translation2d(4.5882, 7.4248),   // blue tag 23 (driver's left, +y wall)
                     new Translation2d(11.9529, 0.6445)), // red  tag 7  (driver's left, -y wall)
        RIGHT_TRENCH(new Translation2d(4.5882, 0.6445),   // blue tag 28
                     new Translation2d(11.9529, 7.4248)); // red  tag 12

        private final Translation2d blueTag;
        private final Translation2d redTag;

        KnownSpot(Translation2d blueTag, Translation2d redTag) {
            this.blueTag = blueTag;
            this.redTag = redTag;
        }

        /** Where the robot is when parked at this spot: the tag, nudged toward the hub. */
        public Translation2d translation() {
            final Translation2d tag = isBlueAlliance() ? blueTag : redTag;
            final Translation2d hub = targetPosition();
            // interpolate(end, t) walks t of the way from tag to hub; t = 0.5 m / total distance
            return tag.interpolate(hub, kTrenchSpotOffsetMeters / tag.getDistance(hub));
        }
    }
}
