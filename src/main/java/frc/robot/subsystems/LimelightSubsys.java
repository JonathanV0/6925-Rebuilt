package frc.robot.subsystems;

import java.util.Optional;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;

public class LimelightSubsys extends SubsystemBase {
    // AprilTag target height above ground
    public static final double kTargetHeightInches = 44.25;
    // All hub AprilTag IDs per alliance (from 2026-rebuilt-welded.json field layout)
    private static final int[] kBlueTagIDs = {18, 19, 20, 21, 24, 25, 26, 27};
    private static final int[] kRedTagIDs  = {2, 3, 4, 5, 8, 9, 10, 11};
    // Camera mounting height above ground
    public static final double kCameraHeightInches = 25.39; // 0.548 + 0.046 + 0.0508 m = 0.6448 m
    // Camera mounting angle: 110 deg from face down = 20 deg above horizontal
    public static final double kCameraMountAngleDegrees = 20.0;

    // Reject measurements where the avg tag area is below this threshold (% of image)
    // Tags that are too small in the frame produce unreliable pose estimates on LL2
    private static final double kMinTagAreaPercent = 0.5;

    private final String name;
    private final NetworkTable telemetryTable;
    private final StructPublisher<Pose2d> posePublisher;

    public LimelightSubsys(String name) {
        this.name = name;
        this.telemetryTable = NetworkTableInstance.getDefault().getTable("SmartDashboard/" + name);
        this.posePublisher = telemetryTable.getStructTopic("Estimated Robot Pose", Pose2d.struct).publish();

        // Tag filter is set each cycle in getMeasurement() based on alliance
    }

    public Optional<Measurement> getMeasurement(Pose2d currentRobotPose) {
        // Update tag filter based on current alliance
        final Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == Alliance.Blue) {
            LimelightHelpers.SetFiducialIDFiltersOverride(name, kBlueTagIDs);
        } else {
            LimelightHelpers.SetFiducialIDFiltersOverride(name, kRedTagIDs);
        }

        LimelightHelpers.SetRobotOrientation(name, currentRobotPose.getRotation().getDegrees(), 0, 0, 0, 0, 0);

        // MegaTag1 works on all Limelight versions (including Limelight 2)
        final PoseEstimate poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(name);
        if (poseEstimate == null || poseEstimate.tagCount == 0 || poseEstimate.avgTagArea < kMinTagAreaPercent) {
            return Optional.empty();
        }

        // Scale trust with distance: close tags are trusted more, far tags less.
        // Quadratic falloff: at 1m → 0.05, at 3m → 0.45, at 5m → 1.25
        final double distance = poseEstimate.avgTagDist;
        final double xyStdDev = 0.05 * distance * distance;
        // Never trust MegaTag1 heading (only accurate with multi-tag, which LL2 doesn't support well)
        final Matrix<N3, N1> standardDeviations = VecBuilder.fill(xyStdDev, xyStdDev, 9999.0);

        posePublisher.set(poseEstimate.pose);

        return Optional.of(new Measurement(poseEstimate, standardDeviations));
    }

    public double getTYDegrees() {
        return LimelightHelpers.getTY(name);
    }

    public static class Measurement {
        public final PoseEstimate poseEstimate;
        public final Matrix<N3, N1> standardDeviations;

        public Measurement(PoseEstimate poseEstimate, Matrix<N3, N1> standardDeviations) {
            this.poseEstimate = poseEstimate;
            this.standardDeviations = standardDeviations;
        }
    }
}
