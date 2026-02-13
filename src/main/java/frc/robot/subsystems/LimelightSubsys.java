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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;

public class LimelightSubsys extends SubsystemBase {
    // AprilTag target height above ground (tags 9 and 10)
    public static final double kTargetHeightInches = 44.25;

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

        // Only track AprilTags 9 and 10 (our shooting side of the target)
        LimelightHelpers.SetFiducialIDFiltersOverride(name, new int[]{9, 10});
    }

    public Optional<Measurement> getMeasurement(Pose2d currentRobotPose) {
        LimelightHelpers.SetRobotOrientation(name, currentRobotPose.getRotation().getDegrees(), 0, 0, 0, 0, 0);

        // MegaTag1 works on all Limelight versions (including Limelight 2)
        final PoseEstimate poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(name);
        if (poseEstimate == null || poseEstimate.tagCount == 0 || poseEstimate.avgTagArea < kMinTagAreaPercent) {
            return Optional.empty();
        }

        final Matrix<N3, N1> standardDeviations = VecBuilder.fill(0.1, 0.1, 10.0);

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
