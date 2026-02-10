package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Vision subsystem for FRC AprilTag Vision Coprocessor integration.
 *
 * This subsystem handles:
 * - Pose estimation fusion with robot odometry
 * - Auto-alignment to AprilTags
 * - Vision status monitoring
 *
 * NetworkTables Topics:
 * - /FRCVision/fused/pose          [x, y, theta] robot pose
 * - /FRCVision/fused/std_devs      [x, y, theta] standard deviations
 * - /FRCVision/fused/valid         boolean validity flag
 * - /FRCVision/auto_align/*        auto-alignment data
 */
public class VisionSubsystem extends SubsystemBase {

    // NetworkTables
    private final NetworkTableInstance ntInstance;
    private final NetworkTable visionTable;

    // Fused pose subscribers
    private final DoubleArraySubscriber poseSub;
    private final DoubleArraySubscriber stdDevsSub;
    private final BooleanSubscriber validSub;
    private final DoubleSubscriber timestampSub;
    private final DoubleSubscriber latencyMsSub;      // For odometry sync
    private final DoubleSubscriber confidenceSub;
    private final IntegerSubscriber tagCountSub;
    private final IntegerSubscriber heartbeatSub;     // Connection check

    // Auto-align publishers (robot -> vision)
    private final IntegerPublisher targetTagIdPub;
    private final DoubleArrayPublisher targetOffsetPub;

    // Auto-align subscribers (vision -> robot)
    private final BooleanSubscriber alignTargetVisibleSub;
    private final DoubleArraySubscriber alignTargetPoseSub;
    private final DoubleArraySubscriber alignRobotPoseSub;
    private final DoubleArraySubscriber alignErrorSub;
    private final DoubleSubscriber alignDistanceSub;
    private final BooleanSubscriber alignReadySub;
    private final BooleanSubscriber alignHasTargetSub;

    // Status subscribers
    private final DoubleSubscriber uptimeSub;
    private final BooleanSubscriber cam0ConnectedSub;
    private final BooleanSubscriber cam1ConnectedSub;
    private final BooleanSubscriber cam2ConnectedSub;

    // Odometry publishers (robot -> vision coprocessor for innovation gating)
    private final DoubleArrayPublisher rioOdomPosePub;
    private final DoublePublisher rioOdomAngularVelPub;

    // Pose estimator reference (provided by drive subsystem)
    private SwerveDrivePoseEstimator poseEstimator;

    // State tracking
    private boolean lastValidState = false;
    private long lastHeartbeat = 0;
    private int updateCount = 0;

    // Configuration
    private static final double MAX_POSE_AMBIGUITY = 0.2; // Max acceptable pose uncertainty
    private static final double STALE_DATA_THRESHOLD = 0.5; // Seconds before data is stale

    /**
     * Creates a new VisionSubsystem.
     */
    public VisionSubsystem() {
        ntInstance = NetworkTableInstance.getDefault();
        visionTable = ntInstance.getTable("FRCVision");

        // Fused pose data
        NetworkTable fusedTable = visionTable.getSubTable("fused");
        poseSub = fusedTable.getDoubleArrayTopic("pose").subscribe(new double[]{});
        stdDevsSub = fusedTable.getDoubleArrayTopic("std_devs").subscribe(new double[]{0.1, 0.1, 0.1});
        validSub = fusedTable.getBooleanTopic("valid").subscribe(false);
        timestampSub = fusedTable.getDoubleTopic("timestamp").subscribe(0.0);
        latencyMsSub = fusedTable.getDoubleTopic("latency_ms").subscribe(0.0);
        confidenceSub = fusedTable.getDoubleTopic("confidence").subscribe(0.0);
        tagCountSub = fusedTable.getIntegerTopic("tag_count").subscribe(0);
        heartbeatSub = fusedTable.getIntegerTopic("heartbeat").subscribe(0);

        // Auto-align topics
        NetworkTable alignTable = visionTable.getSubTable("auto_align");

        // Publishers - robot tells vision what to align to
        targetTagIdPub = alignTable.getIntegerTopic("target_tag_id").publish();
        targetOffsetPub = alignTable.getDoubleArrayTopic("target_offset").publish();

        // Subscribers - vision tells robot alignment status
        alignTargetVisibleSub = alignTable.getBooleanTopic("target_visible").subscribe(false);
        alignTargetPoseSub = alignTable.getDoubleArrayTopic("target_pose").subscribe(new double[]{});
        alignRobotPoseSub = alignTable.getDoubleArrayTopic("robot_pose").subscribe(new double[]{});
        alignErrorSub = alignTable.getDoubleArrayTopic("error").subscribe(new double[]{});
        alignDistanceSub = alignTable.getDoubleTopic("distance_m").subscribe(0.0);
        alignReadySub = alignTable.getBooleanTopic("ready").subscribe(false);
        alignHasTargetSub = alignTable.getBooleanTopic("has_target").subscribe(false);

        // Status
        NetworkTable statusTable = visionTable.getSubTable("status");
        uptimeSub = statusTable.getDoubleTopic("uptime").subscribe(0.0);
        cam0ConnectedSub = statusTable.getBooleanTopic("cam0_connected").subscribe(false);
        cam1ConnectedSub = statusTable.getBooleanTopic("cam1_connected").subscribe(false);
        cam2ConnectedSub = statusTable.getBooleanTopic("cam2_connected").subscribe(false);

        // Odometry publishers - send robot's odometry to coprocessor so it can
        // reject vision measurements that disagree too much (innovation gating).
        // This prevents false positive tag detections from corrupting the pose.
        NetworkTable odomTable = visionTable.getSubTable("rio_odometry");
        rioOdomPosePub = odomTable.getDoubleArrayTopic("pose").publish();
        rioOdomAngularVelPub = odomTable.getDoubleTopic("angular_velocity").publish();

        // Initialize alignment to no target
        targetTagIdPub.set(-1);
        targetOffsetPub.set(new double[]{0.5, 0.0});
    }

    /**
     * Sets the pose estimator for vision fusion.
     * Call this from RobotContainer after creating the drive subsystem.
     */
    public void setPoseEstimator(SwerveDrivePoseEstimator estimator) {
        this.poseEstimator = estimator;
    }

    @Override
    public void periodic() {
        publishOdometry();
        updatePoseEstimator();
        updateTelemetry();
    }

    /**
     * Publishes robot odometry to the coprocessor for innovation gating.
     * The coprocessor uses this to reject vision measurements that disagree
     * too much with where the robot thinks it is (e.g., false tag detections).
     */
    private void publishOdometry() {
        if (poseEstimator == null) return;

        Pose2d currentPose = poseEstimator.getEstimatedPosition();
        rioOdomPosePub.set(new double[]{
            currentPose.getX(),
            currentPose.getY(),
            currentPose.getRotation().getRadians()
        });

        // Angular velocity should come from the gyro. If you have access to
        // the drive subsystem's gyro rate, publish it here. For now, we use 0.0
        // which disables the angular velocity rejection on the coprocessor.
        // TODO: Replace with actual gyro rate from your drive subsystem:
        //   rioOdomAngularVelPub.set(drive.getGyroRate());
        rioOdomAngularVelPub.set(0.0);
    }

    /**
     * Updates the pose estimator with vision measurements.
     * Uses latency compensation for accurate odometry fusion.
     */
    private void updatePoseEstimator() {
        if (poseEstimator == null) return;

        boolean valid = validSub.get();
        double[] pose = poseSub.get();
        double[] stdDevs = stdDevsSub.get();
        double latencyMs = latencyMsSub.get();
        double confidence = confidenceSub.get();
        long heartbeat = heartbeatSub.get();

        // Check data validity
        if (!valid || pose.length < 3 || stdDevs.length < 3) {
            return;
        }

        // Check for stale data using heartbeat (if heartbeat hasn't changed, data is stale)
        if (heartbeat <= lastHeartbeat) {
            return;
        }
        lastHeartbeat = heartbeat;

        // Check confidence threshold
        if (confidence < 0.3) {
            return;
        }

        // Calculate the FPGA timestamp when the image was captured
        // Current FPGA time minus the processing latency
        double captureTime = Timer.getFPGATimestamp() - (latencyMs / 1000.0);

        // Don't use measurements from the future or too far in the past
        if (captureTime < 0 || latencyMs > 500) {
            return;
        }

        // Create pose from vision data
        Pose2d visionPose = new Pose2d(
            pose[0],
            pose[1],
            new Rotation2d(pose[2])
        );

        // The coprocessor computes distance-based standard deviations using
        // the pinhole camera model: distance = (tag_size * focal_length) / pixel_size.
        // These std_devs scale with distance^2 and 1/sqrt(tag_count), so they
        // naturally weight close multi-tag readings higher than far single-tag ones.
        // Just pass them through to the pose estimator - the coprocessor already
        // did the hard work of computing proper values.
        var scaledStdDevs = VecBuilder.fill(
            stdDevs[0],
            stdDevs[1],
            stdDevs[2]
        );

        // Add vision measurement to pose estimator with latency-compensated timestamp
        poseEstimator.addVisionMeasurement(
            visionPose,
            captureTime,
            scaledStdDevs
        );

        updateCount++;
    }

    /**
     * Updates SmartDashboard telemetry.
     */
    private void updateTelemetry() {
        SmartDashboard.putBoolean("Vision/Valid", validSub.get());
        SmartDashboard.putNumber("Vision/Confidence", confidenceSub.get());
        SmartDashboard.putNumber("Vision/TagCount", tagCountSub.get());
        SmartDashboard.putNumber("Vision/LatencyMs", latencyMsSub.get());
        SmartDashboard.putNumber("Vision/UpdateCount", updateCount);
        SmartDashboard.putNumber("Vision/Uptime", uptimeSub.get());
        SmartDashboard.putNumber("Vision/Heartbeat", heartbeatSub.get());

        // Camera status
        SmartDashboard.putBoolean("Vision/Cam0", cam0ConnectedSub.get());
        SmartDashboard.putBoolean("Vision/Cam1", cam1ConnectedSub.get());
        SmartDashboard.putBoolean("Vision/Cam2", cam2ConnectedSub.get());

        // Alignment status
        SmartDashboard.putBoolean("Vision/AlignReady", alignReadySub.get());
        SmartDashboard.putNumber("Vision/AlignDistance", alignDistanceSub.get());
    }

    // ========================================================================
    // Vision Pose Methods
    // ========================================================================

    /**
     * Gets the current vision pose if valid.
     * @return Vision pose or null if invalid
     */
    public Pose2d getVisionPose() {
        if (!validSub.get()) return null;

        double[] pose = poseSub.get();
        if (pose.length < 3) return null;

        return new Pose2d(pose[0], pose[1], new Rotation2d(pose[2]));
    }

    /**
     * Checks if vision has a valid pose.
     */
    public boolean hasValidPose() {
        return validSub.get();
    }

    /**
     * Gets the vision confidence (0.0-1.0).
     */
    public double getConfidence() {
        return confidenceSub.get();
    }

    /**
     * Gets the number of visible AprilTags.
     */
    public int getTagCount() {
        return (int) tagCountSub.get();
    }

    // ========================================================================
    // Auto-Align Methods
    // ========================================================================

    /**
     * Starts alignment to a specific AprilTag.
     *
     * @param tagId AprilTag ID to align to
     * @param distanceMeters Distance from tag to stop at
     * @param angleRadians Approach angle offset (0 = perpendicular)
     */
    public void startAlignment(int tagId, double distanceMeters, double angleRadians) {
        targetTagIdPub.set(tagId);
        targetOffsetPub.set(new double[]{distanceMeters, angleRadians});

        System.out.println("[Vision] Starting alignment to tag " + tagId +
                          " at " + distanceMeters + "m");
    }

    /**
     * Starts alignment to a tag at default distance (0.5m, perpendicular).
     */
    public void startAlignment(int tagId) {
        startAlignment(tagId, 0.5, 0.0);
    }

    /**
     * Stops alignment.
     */
    public void stopAlignment() {
        targetTagIdPub.set(-1);
        System.out.println("[Vision] Alignment stopped");
    }

    /**
     * Checks if the target tag is currently visible.
     */
    public boolean isTargetVisible() {
        return alignTargetVisibleSub.get();
    }

    /**
     * Checks if the robot is aligned to the target (within tolerance).
     */
    public boolean isAligned() {
        return alignReadySub.get();
    }

    /**
     * Checks if an alignment target is set.
     */
    public boolean hasAlignmentTarget() {
        return alignHasTargetSub.get();
    }

    /**
     * Gets the alignment error [x, y, theta].
     * Error is (target - current), so positive means robot needs to move in positive direction.
     *
     * @return Error array [x_meters, y_meters, theta_radians] or null if no target
     */
    public double[] getAlignmentError() {
        if (!alignHasTargetSub.get()) return null;

        double[] error = alignErrorSub.get();
        if (error.length < 3) return null;

        return error;
    }

    /**
     * Gets the X alignment error (field-relative).
     */
    public double getAlignmentErrorX() {
        double[] error = getAlignmentError();
        return error != null ? error[0] : 0.0;
    }

    /**
     * Gets the Y alignment error (field-relative).
     */
    public double getAlignmentErrorY() {
        double[] error = getAlignmentError();
        return error != null ? error[1] : 0.0;
    }

    /**
     * Gets the rotation alignment error (radians).
     */
    public double getAlignmentErrorTheta() {
        double[] error = getAlignmentError();
        return error != null ? error[2] : 0.0;
    }

    /**
     * Gets the distance to the alignment target (meters).
     */
    public double getAlignmentDistance() {
        return alignDistanceSub.get();
    }

    /**
     * Gets the target pose for alignment.
     */
    public Pose2d getAlignmentTargetPose() {
        double[] pose = alignTargetPoseSub.get();
        if (pose.length < 3) return null;
        return new Pose2d(pose[0], pose[1], new Rotation2d(pose[2]));
    }

    // ========================================================================
    // Status Methods
    // ========================================================================

    /**
     * Gets the vision system uptime in seconds.
     */
    public double getUptime() {
        return uptimeSub.get();
    }

    /**
     * Checks if a specific camera is connected.
     */
    public boolean isCameraConnected(int cameraId) {
        switch (cameraId) {
            case 0: return cam0ConnectedSub.get();
            case 1: return cam1ConnectedSub.get();
            case 2: return cam2ConnectedSub.get();
            default: return false;
        }
    }

    /**
     * Gets the number of connected cameras.
     */
    public int getConnectedCameraCount() {
        int count = 0;
        if (cam0ConnectedSub.get()) count++;
        if (cam1ConnectedSub.get()) count++;
        if (cam2ConnectedSub.get()) count++;
        return count;
    }

    /**
     * Checks if the vision system is fully operational.
     */
    public boolean isOperational() {
        return getConnectedCameraCount() > 0 && uptimeSub.get() > 0;
    }
}
