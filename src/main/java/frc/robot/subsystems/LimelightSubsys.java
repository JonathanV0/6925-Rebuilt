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
    // All AprilTag IDs per alliance side for full-field localization
    // Hub: 8 tags, Trench: 4 tags, Outpost: 2 tags, Tower wall: 2 tags = 16 per alliance
    private static final int[] kBlueTagIDs = {17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32};
    private static final int[] kRedTagIDs  = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16};
    // Camera mounting position relative to robot center
    public static final double kCameraForwardInches = -1.46;  // 1.46" behind center
    public static final double kCameraSideInches = 0.0;       // centered left-right
    public static final double kCameraHeightInches = 25.39;   // 0.548 + 0.046 + 0.0508 m = 0.6448 m
    // Camera mounting angle: 110 deg from face down = 20 deg above horizontal
    public static final double kCameraMountAngleDegrees = 20.0;

    // LL3 has better resolution — can reliably see tags from further away
    private static final double kMinTagAreaPercent = 0.1;

    private final String name;
    private final NetworkTable telemetryTable;
    private final StructPublisher<Pose2d> posePublisher;

    public LimelightSubsys(String name) {
        this.name = name;
        this.telemetryTable = NetworkTableInstance.getDefault().getTable("SmartDashboard/" + name);
        this.posePublisher = telemetryTable.getStructTopic("Estimated Robot Pose", Pose2d.struct).publish();
    }

    public Optional<Measurement> getMeasurement(Pose2d currentRobotPose) {
        // Update tag filter based on current alliance
        final Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == Alliance.Blue) {
            LimelightHelpers.SetFiducialIDFiltersOverride(name, kBlueTagIDs);
        } else {
            LimelightHelpers.SetFiducialIDFiltersOverride(name, kRedTagIDs);
        }

        // Feed gyro heading to LL3 — MegaTag2 uses this for accurate 3D multi-tag solving
        LimelightHelpers.SetRobotOrientation(name, currentRobotPose.getRotation().getDegrees(), 0, 0, 0, 0, 0);

        // MegaTag2: LL3 fuses multiple tags in 3D using the gyro heading for much better accuracy
        final PoseEstimate poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);
        if (poseEstimate == null || poseEstimate.tagCount == 0 || poseEstimate.avgTagArea < kMinTagAreaPercent) {
            return Optional.empty();
        }

        final double distance = poseEstimate.avgTagDist;
        final Matrix<N3, N1> standardDeviations;

        if (poseEstimate.tagCount >= 2) {
            // Multi-tag MegaTag2: very high confidence — tight XY, trust heading
            final double xyStdDev = 0.02 * distance;
            standardDeviations = VecBuilder.fill(xyStdDev, xyStdDev, 0.1);
        } else {
            // Single tag: trust XY with quadratic distance falloff, ignore heading
            final double xyStdDev = 0.05 * distance * distance;
            standardDeviations = VecBuilder.fill(xyStdDev, xyStdDev, 9999.0);
        }

        posePublisher.set(poseEstimate.pose);

        return Optional.of(new Measurement(poseEstimate, standardDeviations));
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
