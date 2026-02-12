package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Vision subsystem for AprilVision 3.2 AprilTag detection and pose estimation.
 *
 * AprilVision 3.2 - Custom FRC Vision System.
 *
 * This subsystem manages multiple vision cameras and provides
 * pose estimates for robot localization. It uses the
 * MULTI_TAG_PNP_ON_COPROCESSOR strategy for best accuracy.
 *
 * Camera names must match those configured in the AprilVision dashboard
 * (accessed at http://coprocessor-ip:5801).
 */
public class VisionSubsystem extends SubsystemBase {

    // Vision cameras - names must match dashboard config
    private final PhotonCamera frontCamera;
    private final PhotonCamera leftCamera;
    private final PhotonCamera rightCamera;

    // Pose estimators for each camera
    private final PhotonPoseEstimator frontPoseEstimator;
    private final PhotonPoseEstimator leftPoseEstimator;
    private final PhotonPoseEstimator rightPoseEstimator;

    // Camera mounting positions relative to robot center (meters, radians)
    // Adjust these Transform3d values to match your robot's camera mounting
    private static final Transform3d FRONT_ROBOT_TO_CAM = new Transform3d(
        new Translation3d(0.25, 0.0, 0.50),    // 25cm forward, 50cm up
        new Rotation3d(0, 0, 0)                  // facing forward
    );

    private static final Transform3d LEFT_ROBOT_TO_CAM = new Transform3d(
        new Translation3d(0.0, 0.20, 0.50),     // 20cm left, 50cm up
        new Rotation3d(0, 0, Math.toRadians(90))  // facing left
    );

    private static final Transform3d RIGHT_ROBOT_TO_CAM = new Transform3d(
        new Translation3d(0.0, -0.20, 0.50),     // 20cm right, 50cm up
        new Rotation3d(0, 0, Math.toRadians(-90)) // facing right
    );

    // Standard deviation scaling factors
    private static final double SINGLE_TAG_STD_DEV_X = 0.5;   // meters
    private static final double SINGLE_TAG_STD_DEV_Y = 0.5;   // meters
    private static final double SINGLE_TAG_STD_DEV_THETA = 0.9; // radians
    private static final double MULTI_TAG_STD_DEV_X = 0.2;
    private static final double MULTI_TAG_STD_DEV_Y = 0.2;
    private static final double MULTI_TAG_STD_DEV_THETA = 0.5;

    // State
    private Optional<EstimatedRobotPose> latestFrontPose = Optional.empty();
    private Optional<EstimatedRobotPose> latestLeftPose = Optional.empty();
    private Optional<EstimatedRobotPose> latestRightPose = Optional.empty();

    /**
     * Creates a new VisionSubsystem.
     * Camera names must match AprilVision dashboard configuration.
     */
    public VisionSubsystem() {
        // Initialize vision cameras
        frontCamera = new PhotonCamera("front");
        leftCamera = new PhotonCamera("left");
        rightCamera = new PhotonCamera("right");

        // Load the AprilTag field layout for the current FRC season
        AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

        // Create pose estimators
        // MULTI_TAG_PNP_ON_COPROCESSOR: computes multi-tag pose
        // on the coprocessor for best performance. Falls back to LOWEST_AMBIGUITY
        // when only one tag is visible.
        frontPoseEstimator = new PhotonPoseEstimator(
            fieldLayout,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            FRONT_ROBOT_TO_CAM
        );
        frontPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

        leftPoseEstimator = new PhotonPoseEstimator(
            fieldLayout,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            LEFT_ROBOT_TO_CAM
        );
        leftPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

        rightPoseEstimator = new PhotonPoseEstimator(
            fieldLayout,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            RIGHT_ROBOT_TO_CAM
        );
        rightPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    }

    @Override
    public void periodic() {
        // Update pose estimates from each camera
        latestFrontPose = frontPoseEstimator.update(frontCamera.getLatestResult());
        latestLeftPose = leftPoseEstimator.update(leftCamera.getLatestResult());
        latestRightPose = rightPoseEstimator.update(rightCamera.getLatestResult());

        updateTelemetry();
    }

    // ========================================================================
    // Pose Estimation Methods
    // ========================================================================

    /**
     * Gets the latest estimated robot pose from the front camera.
     * @return Estimated pose with timestamp, or empty if no valid estimate
     */
    public Optional<EstimatedRobotPose> getFrontCameraPose() {
        return latestFrontPose;
    }

    /**
     * Gets the latest estimated robot pose from the left camera.
     */
    public Optional<EstimatedRobotPose> getLeftCameraPose() {
        return latestLeftPose;
    }

    /**
     * Gets the latest estimated robot pose from the right camera.
     */
    public Optional<EstimatedRobotPose> getRightCameraPose() {
        return latestRightPose;
    }

    /**
     * Calculates standard deviations for a pose estimate.
     * Uses distance to tags and number of tags to scale trust.
     *
     * @param estimatedPose The pose estimate to calculate std devs for
     * @return Matrix of standard deviations [x, y, theta]
     */
    public Matrix<N3, N1> getEstimationStdDevs(EstimatedRobotPose estimatedPose) {
        int numTags = estimatedPose.targetsUsed.size();
        double avgDist = 0;

        for (PhotonTrackedTarget target : estimatedPose.targetsUsed) {
            var tagPose = frontPoseEstimator.getFieldTags().getTagPose(target.getFiducialId());
            if (tagPose.isPresent()) {
                avgDist += tagPose.get().toPose2d()
                    .getTranslation()
                    .getDistance(estimatedPose.estimatedPose.toPose2d().getTranslation());
            }
        }

        if (numTags == 0) {
            return VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        }

        avgDist /= numTags;

        // Scale standard deviations based on distance and tag count
        if (numTags > 1) {
            // Multi-tag: more trustworthy, scale with distance
            return VecBuilder.fill(
                MULTI_TAG_STD_DEV_X * (1 + avgDist * avgDist / 30),
                MULTI_TAG_STD_DEV_Y * (1 + avgDist * avgDist / 30),
                MULTI_TAG_STD_DEV_THETA * (1 + avgDist * avgDist / 20)
            );
        } else {
            // Single tag: less trustworthy, scale more aggressively
            return VecBuilder.fill(
                SINGLE_TAG_STD_DEV_X * (1 + avgDist * avgDist / 15),
                SINGLE_TAG_STD_DEV_Y * (1 + avgDist * avgDist / 15),
                SINGLE_TAG_STD_DEV_THETA * (1 + avgDist * avgDist / 10)
            );
        }
    }

    // ========================================================================
    // Detection Query Methods
    // ========================================================================

    /**
     * Gets the latest pipeline result from the front camera.
     */
    public PhotonPipelineResult getLatestFrontResult() {
        return frontCamera.getLatestResult();
    }

    /**
     * Checks if any camera currently sees an AprilTag.
     */
    public boolean hasTargets() {
        return frontCamera.getLatestResult().hasTargets() ||
               leftCamera.getLatestResult().hasTargets() ||
               rightCamera.getLatestResult().hasTargets();
    }

    /**
     * Gets the total number of detected AprilTags across all cameras.
     */
    public int getTotalTagCount() {
        int count = 0;
        count += frontCamera.getLatestResult().getTargets().size();
        count += leftCamera.getLatestResult().getTargets().size();
        count += rightCamera.getLatestResult().getTargets().size();
        return count;
    }

    /**
     * Gets the best target from the front camera.
     * Useful for alignment commands.
     *
     * @return Best target or empty if no targets visible
     */
    public Optional<PhotonTrackedTarget> getBestFrontTarget() {
        var result = frontCamera.getLatestResult();
        if (result.hasTargets()) {
            return Optional.of(result.getBestTarget());
        }
        return Optional.empty();
    }

    /**
     * Checks if a specific tag ID is visible from any camera.
     *
     * @param tagId The AprilTag ID to search for
     * @return true if the tag is visible
     */
    public boolean isTagVisible(int tagId) {
        return isTagVisibleFromCamera(frontCamera, tagId) ||
               isTagVisibleFromCamera(leftCamera, tagId) ||
               isTagVisibleFromCamera(rightCamera, tagId);
    }

    private boolean isTagVisibleFromCamera(PhotonCamera camera, int tagId) {
        var result = camera.getLatestResult();
        if (!result.hasTargets()) return false;
        for (var target : result.getTargets()) {
            if (target.getFiducialId() == tagId) return true;
        }
        return false;
    }

    // ========================================================================
    // Camera Status Methods
    // ========================================================================

    /**
     * Checks if a specific camera is connected.
     */
    public boolean isCameraConnected(String name) {
        switch (name) {
            case "front": return frontCamera.isConnected();
            case "left":  return leftCamera.isConnected();
            case "right": return rightCamera.isConnected();
            default: return false;
        }
    }

    /**
     * Gets the number of connected cameras.
     */
    public int getConnectedCameraCount() {
        int count = 0;
        if (frontCamera.isConnected()) count++;
        if (leftCamera.isConnected()) count++;
        if (rightCamera.isConnected()) count++;
        return count;
    }

    /**
     * Checks if the vision system is operational (at least one camera connected).
     */
    public boolean isOperational() {
        return getConnectedCameraCount() > 0;
    }

    // ========================================================================
    // Telemetry
    // ========================================================================

    private void updateTelemetry() {
        // Camera connection status
        SmartDashboard.putBoolean("Vision/FrontConnected", frontCamera.isConnected());
        SmartDashboard.putBoolean("Vision/LeftConnected", leftCamera.isConnected());
        SmartDashboard.putBoolean("Vision/RightConnected", rightCamera.isConnected());
        SmartDashboard.putBoolean("Vision/HasTargets", hasTargets());
        SmartDashboard.putNumber("Vision/TotalTags", getTotalTagCount());

        // Front camera latency
        var frontResult = frontCamera.getLatestResult();
        SmartDashboard.putNumber("Vision/FrontLatencyMs", frontResult.getLatencyMillis());
        SmartDashboard.putNumber("Vision/FrontTargets", frontResult.getTargets().size());

        // Pose availability
        SmartDashboard.putBoolean("Vision/FrontPoseValid", latestFrontPose.isPresent());
        SmartDashboard.putBoolean("Vision/LeftPoseValid", latestLeftPose.isPresent());
        SmartDashboard.putBoolean("Vision/RightPoseValid", latestRightPose.isPresent());
    }
}
