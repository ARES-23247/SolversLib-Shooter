package org.firstinspires.ftc.teamcode.command.subsystems.vision;

import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.util.vision.LimelightCamera;
import org.firstinspires.ftc.teamcode.command.subsystems.drive.Drive;
import org.firstinspires.ftc.teamcode.Constants.VisionFusionMode;

import com.pedropathing.geometry.Pose;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.teamcode.Constants.*;

/**
 * Advanced multi-camera vision subsystem with fusion, calibration, and health monitoring.
 *
 * <p>This subsystem supports multiple Limelight3A cameras with three fusion modes:</p>
 * <ul>
 *   <li><b>SELECT_BEST:</b> Use single best camera (most stable)</li>
 *   <li><b>WEIGHTED_AVERAGE:</b> Fuse all cameras with confidence weighting</li>
 *   <li><b>CONSENSUS:</b> Require cameras to agree, reject outliers</li>
 * </ul>
 *
 * <h3>Features:</h3>
 * <ul>
 *   <li><b>Multi-Camera Fusion:</b> Combines measurements from multiple cameras</li>
 *   <li><b>Camera Calibration:</b> Per-camera mount offsets and noise parameters</li>
 *   <li><b>Health Monitoring:</b> Detects and excludes failed cameras</li>
 *   <li><b>Adaptive Priority:</b> Boosts priority for cameras facing movement direction</li>
 *   <li><b>Tag ID Filtering:</b> Prefer specific tags (e.g., your alliance's tags)</li>
 * </ul>
 *
 * <h3>Fusion Modes:</h3>
 *
 * <h4>SELECT_BEST (Default):</h4>
 * <p>Scores each camera and uses only the winner. Most stable, easiest to debug.</p>
 * <pre>{@code score = (tagCount × priority) / (distance + 1)}</pre>
 *
 * <h4>WEIGHTED_AVERAGE:</h4>
 * <p>Fuses all cameras with confidence-based weighting. More accurate but potentially less stable.</p>
 * <pre>{@code
 * weight_i = (tagCount_i × priority_i) / (distance_i + 1)
 * pose_fused = Σ(weight_i × pose_i) / Σ(weight_i)
 * }</pre>
 *
 * <h4>CONSENSUS:</h4>
 * <p>Requires cameras to agree within threshold. Rejects outliers automatically.</p>
 * <pre>{@code
 * avg_pose = average(all measurements)
 * reject cameras where |camera_pose - avg_pose| > threshold
 * pose_fused = average(remaining cameras)
 * }</pre>
 *
 * <h3>Configuration:</h3>
 * <p>Configure in {@link org.firstinspires.ftc.teamcode.Constants}:</p>
 * <pre>{@code
 * // Choose fusion mode
 * VISION_FUSION_MODE = VisionFusionMode.WEIGHTED_AVERAGE;
 *
 * // Add cameras with calibration
 * LIMELIGHT_NAMES = {"limelightFront", "limelightRear"};
 * LIMELIGHT_PRIORITIES = {1.0, 0.5};
 * LIMELIGHT_MOUNT_POSITIONS = {{6.0, 0.0}, {-6.0, 0.0}};  // Front+6in, Rear-6in
 * LIMELIGHT_ORIENTATION_OFFSETS = {0.0, 0.0};  // Aligned
 * LIMELIGHT_POSITION_NOISE = {0.8, 1.2};  // Front more accurate
 * LIMELIGHT_HEADING_NOISE = {0.02, 0.05};
 *
 * // Tag preferences
 * VISION_PREFERRED_TAG_IDS = {1, 2, 3};  // Use blue alliance tags
 *
 * // Health monitoring
 * VISION_CAMERA_HEALTH_TIMEOUT = 5.0;
 * }</pre>
 *
 * @see org.firstinspires.ftc.teamcode.util.LimelightCamera
 * @see org.firstinspires.ftc.teamcode.util.SensorFusionLocalizer
 */
public class Vision extends SubsystemBase {

    /**
     * Reference to the robot singleton.
     */
    private final Robot robot = Robot.getInstance();

    /**
     * Tracks whether ANY camera currently sees a tag.
     */
    private boolean isAnyTagVisible = false;

    /**
     * The camera(s) selected for pose correction.
     */
    private List<LimelightCamera> activeCameras = new ArrayList<>();

    /**
     * Loop counter for vision update throttling (update vision every N loops for performance).
     */
    private int visionLoopCounter = 0;

    /**
     * Constructs the Vision subsystem.
     */
    public Vision() {
        // Initialization if needed
    }

    /**
     * Periodic update method called every loop iteration.
     *
     * <p>Performs multi-camera pose correction:</p>
     * <ol>
     *   <li>Update all cameras with latest detections</li>
     *   <li>Apply adaptive priority based on robot heading/movement</li>
     *   <li>Select fusion mode and compute pose correction</li>
     *   <li>Apply sensor fusion correction</li>
     *   <li>Update telemetry with all camera statuses</li>
     * </ol>
     */
    @Override
    public void periodic() {
        if (robot.limelightCameras == null || robot.limelightCameras.length == 0) {
            return;  // No cameras configured
        }

        // Vision update throttling for performance (update every N loops)
        visionLoopCounter++;
        if (visionLoopCounter < VISION_UPDATE_FREQUENCY) {
            return;  // Skip this vision update
        }
        visionLoopCounter = 0;

        // Step 1: Update all cameras with calibration data
        isAnyTagVisible = false;
        for (int i = 0; i < robot.limelightCameras.length; i++) {
            LimelightCamera camera = robot.limelightCameras[i];

            // Get calibration data for this camera
            double mountX = (i < LIMELIGHT_MOUNT_POSITIONS.length) ?
                LIMELIGHT_MOUNT_POSITIONS[i][0] : 0.0;
            double mountY = (i < LIMELIGHT_MOUNT_POSITIONS.length) ?
                LIMELIGHT_MOUNT_POSITIONS[i][1] : 0.0;
            double orientationOffset = (i < LIMELIGHT_ORIENTATION_OFFSETS.length) ?
                LIMELIGHT_ORIENTATION_OFFSETS[i] : 0.0;

            // Create calibrated camera wrapper
            LimelightCamera calibratedCam = new LimelightCamera(
                camera.getLimelight(),
                camera.getName(),
                LIMELIGHT_PRIORITIES[i],
                mountX, mountY, orientationOffset,
                (i < LIMELIGHT_POSITION_NOISE.length) ? LIMELIGHT_POSITION_NOISE[i] : 1.0,
                (i < LIMELIGHT_HEADING_NOISE.length) ? LIMELIGHT_HEADING_NOISE[i] : 0.03
            );

            // Update the camera
            calibratedCam.update(
                VISION_MAX_TAG_DISTANCE,
                VISION_MIN_TAGS,
                VISION_CAMERA_HEALTH_TIMEOUT,
                VISION_PREFERRED_TAG_IDS
            );

            // Update the original camera's state
            camera = calibratedCam;  // Replace with calibrated version
            robot.limelightCameras[i] = camera;

            if (camera.hasValidDetection()) {
                isAnyTagVisible = true;
            }
        }

        // Step 2: Apply adaptive priority (optional)
        if (VISION_ADAPTIVE_PRIORITY && robot.drive != null) {
            applyAdaptivePriority();
        }

        // Step 3: Select fusion mode and compute pose correction
        double[] fusedPose = selectPoseByFusionMode();

        // Step 4: Apply sensor fusion correction with adaptive noise
        if (fusedPose != null && robot.drive != null && robot.drive.getSensorFusion() != null) {
            // Check if any active camera has floating robot detection
            boolean hasFloatingReading = checkAnyCameraFloating();

            if (hasFloatingReading) {
                // Use increased noise for floating readings
                double positionNoise = calculateEffectivePositionNoise();
                double headingNoise = calculateEffectiveHeadingNoise();

                robot.drive.getSensorFusion().correctWithVision(
                    fusedPose[0],
                    fusedPose[1],
                    fusedPose[2],
                    positionNoise,
                    headingNoise
                );
            } else {
                // Normal vision correction with default noise
                robot.drive.getSensorFusion().correctWithVision(
                    fusedPose[0],
                    fusedPose[1],
                    fusedPose[2]
                );
            }
        }

        // Step 5: Telemetry
        updateTelemetry();
    }

    /**
     * Applies adaptive priority based on robot movement direction.
     *
     * <p>Boosts priority for cameras facing the direction of movement.</p>
     */
    private void applyAdaptivePriority() {
        // Get robot velocities
        Pose currentPose = robot.drive.getSensorFusion().getEstimatedPose();
        double heading = currentPose.getHeading();

        double forwardSpeed = robot.drive.getTeleOpSpeeds().vxMetersPerSecond;
        double lateralSpeed = robot.drive.getTeleOpSpeeds().vyMetersPerSecond;

        // Determine movement direction
        double movementAngle = Math.atan2(lateralSpeed, forwardSpeed);
        double relativeAngle = movementAngle - heading;

        // Normalize to -PI to PI
        while (relativeAngle > Math.PI) relativeAngle -= 2 * Math.PI;
        while (relativeAngle < -Math.PI) relativeAngle += 2 * Math.PI;

        // Apply priority multipliers based on camera name
        for (LimelightCamera camera : robot.limelightCameras) {
            String name = camera.getName().toLowerCase();
            double multiplier = 1.0;

            // Front cameras (driving forward)
            if (name.contains("front") && Math.abs(relativeAngle) < Math.PI / 4) {
                multiplier = 2.0;
            }
            // Rear cameras (driving backward)
            else if (name.contains("rear") && Math.abs(relativeAngle) > 3 * Math.PI / 4) {
                multiplier = 2.0;
            }
            // Side cameras (strafing)
            else if (name.contains("left") && relativeAngle > Math.PI / 4 && relativeAngle < 3 * Math.PI / 4) {
                multiplier = 1.5;
            }
            else if (name.contains("right") && relativeAngle < -Math.PI / 4 && relativeAngle > -3 * Math.PI / 4) {
                multiplier = 1.5;
            }

            camera.setPriorityMultiplier(multiplier);
        }
    }

    /**
     * Selects pose correction based on configured fusion mode.
     *
     * @return array of [x, y, heading] in inches/radians, or null if no valid detections
     */
    private double[] selectPoseByFusionMode() {
        List<LimelightCamera> healthyCameras = new ArrayList<>();
        for (LimelightCamera camera : robot.limelightCameras) {
            if (camera.hasValidDetection() && camera.isHealthy()) {
                healthyCameras.add(camera);
            }
        }

        if (healthyCameras.isEmpty()) {
            activeCameras.clear();
            return null;
        }

        switch (VISION_FUSION_MODE) {
            case SELECT_BEST:
                return selectBestCamera(healthyCameras);

            case WEIGHTED_AVERAGE:
                return weightedAverageFusion(healthyCameras);

            case CONSENSUS:
                return consensusFusion(healthyCameras);

            default:
                return selectBestCamera(healthyCameras);
        }
    }

    /**
     * SELECT_BEST mode: Choose single best camera.
     *
     * @param healthyCameras list of healthy cameras with valid detections
     * @return pose from best camera
     */
    private double[] selectBestCamera(List<LimelightCamera> healthyCameras) {
        LimelightCamera bestCamera = null;
        double bestScore = -1.0;

        for (LimelightCamera camera : healthyCameras) {
            // Score = (tagCount × priority) / (distance + 1)
            double score = (camera.getVisibleTagCount() * camera.getPriority()) /
                          (camera.getDetectionDistance() + 1.0);

            if (score > bestScore) {
                bestScore = score;
                bestCamera = camera;
            }
        }

        activeCameras.clear();
        if (bestCamera != null) {
            activeCameras.add(bestCamera);
        }

        return bestCamera != null ?
            new double[]{bestCamera.getPoseX(), bestCamera.getPoseY(), bestCamera.getPoseHeading()} :
            null;
    }

    /**
     * WEIGHTED_AVERAGE mode: Fuse all cameras with confidence weighting.
     *
     * @param healthyCameras list of healthy cameras with valid detections
     * @return fused pose from all cameras
     */
    private double[] weightedAverageFusion(List<LimelightCamera> healthyCameras) {
        double sumWeights = 0.0;
        double sumX = 0.0;
        double sumY = 0.0;
        double sumHeading = 0.0;

        // Calculate weights and weighted sum
        for (LimelightCamera camera : healthyCameras) {
            double weight = camera.getConfidenceWeight();

            sumWeights += weight;
            sumX += weight * camera.getPoseX();
            sumY += weight * camera.getPoseY();
            sumHeading += weight * camera.getPoseHeading();
        }

        if (sumWeights == 0.0) {
            return null;
        }

        // Normalize by total weight
        double fusedX = sumX / sumWeights;
        double fusedY = sumY / sumWeights;
        double fusedHeading = sumHeading / sumWeights;

        // Normalize heading to -PI to PI
        while (fusedHeading > Math.PI) fusedHeading -= 2 * Math.PI;
        while (fusedHeading < -Math.PI) fusedHeading += 2 * Math.PI;

        activeCameras = healthyCameras;
        return new double[]{fusedX, fusedY, fusedHeading};
    }

    /**
     * CONSENSUS mode: Require cameras to agree, reject outliers.
     *
     * @param healthyCameras list of healthy cameras with valid detections
     * @return consensus pose (with outliers rejected)
     */
    private double[] consensusFusion(List<LimelightCamera> healthyCameras) {
        if (healthyCameras.size() < 2) {
            // Need at least 2 cameras for consensus
            return selectBestCamera(healthyCameras);
        }

        // Calculate average pose
        double avgX = 0.0, avgY = 0.0, avgHeading = 0.0;
        for (LimelightCamera camera : healthyCameras) {
            avgX += camera.getPoseX();
            avgY += camera.getPoseY();
            avgHeading += camera.getPoseHeading();
        }
        avgX /= healthyCameras.size();
        avgY /= healthyCameras.size();
        avgHeading /= healthyCameras.size();

        // Normalize heading
        while (avgHeading > Math.PI) avgHeading -= 2 * Math.PI;
        while (avgHeading < -Math.PI) avgHeading += 2 * Math.PI;

        // Find cameras within consensus threshold
        List<LimelightCamera> consensusCameras = new ArrayList<>();
        for (LimelightCamera camera : healthyCameras) {
            double diffX = camera.getPoseX() - avgX;
            double diffY = camera.getPoseY() - avgY;
            double diffHeading = camera.getPoseHeading() - avgHeading;

            // Normalize heading difference
            while (diffHeading > Math.PI) diffHeading -= 2 * Math.PI;
            while (diffHeading < -Math.PI) diffHeading += 2 * Math.PI;

            double distanceDiff = Math.hypot(diffX, diffY);

            // Check if within consensus threshold
            if (distanceDiff <= VISION_CONSENSUS_MAX_DIFF &&
                Math.abs(diffHeading) <= VISION_CONSENSUS_MAX_DIFF) {
                consensusCameras.add(camera);
            }
        }

        // Warn if outliers detected
        if (consensusCameras.size() < healthyCameras.size() &&
            robot.telemetry != null) {
            System.out.println(String.format(
                "Vision: Rejected %d outlier cameras",
                healthyCameras.size() - consensusCameras.size()
            ));
        }

        if (consensusCameras.isEmpty()) {
            // No consensus, fall back to best camera
            return selectBestCamera(healthyCameras);
        }

        // Average the consensus cameras
        double fusedX = 0.0, fusedY = 0.0, fusedHeading = 0.0;
        for (LimelightCamera camera : consensusCameras) {
            fusedX += camera.getPoseX();
            fusedY += camera.getPoseY();
            fusedHeading += camera.getPoseHeading();
        }
        fusedX /= consensusCameras.size();
        fusedY /= consensusCameras.size();
        fusedHeading /= consensusCameras.size();

        // Normalize heading
        while (fusedHeading > Math.PI) fusedHeading -= 2 * Math.PI;
        while (fusedHeading < -Math.PI) fusedHeading += 2 * Math.PI;

        activeCameras = consensusCameras;
        return new double[]{fusedX, fusedY, fusedHeading};
    }

    /**
     * Updates telemetry with all camera statuses.
     */
    private void updateTelemetry() {
        if (robot.telemetry == null) return;

        // Overall status
        robot.telemetry.addData("Vision Fusion Mode", VISION_FUSION_MODE);
        robot.telemetry.addData("Vision Any Tag Visible", isAnyTagVisible);
        robot.telemetry.addData("Vision Active Cameras", activeCameras.size());

        // Per-camera status
        for (LimelightCamera camera : robot.limelightCameras) {
            robot.telemetry.addData("Vision " + camera.getName(),
                camera.getStatusString());

            // Health monitoring
            robot.telemetry.addData("Vision " + camera.getName() + " Time Since Last",
                String.format("%.1fs", camera.getTimeSinceLastDetection()));

            // Filtering telemetry
            if (camera.isPoseFloating()) {
                robot.telemetry.addData("Vision " + camera.getName() + " Floating", "YES");
            }
            if (camera.getBoundaryRejectionCount() > 0) {
                robot.telemetry.addData("Vision " + camera.getName() + " Boundary Rejects", String.valueOf(camera.getBoundaryRejectionCount()));
            }
        }

        // Active camera(s) info
        if (!activeCameras.isEmpty()) {
            StringBuilder activeNames = new StringBuilder("Active: ");
            for (LimelightCamera cam : activeCameras) {
                activeNames.append(cam.getName()).append(", ");
            }
            // Remove trailing comma
            if (activeNames.length() > 2) {
                activeNames.setLength(activeNames.length() - 2);
            }
            robot.telemetry.addData("Vision Active Camera(s)", activeNames.toString());
        }
    }

    /**
     * Checks if ANY camera currently sees a tag.
     *
     * @return true if at least one camera has a valid detection
     */
    public boolean isTagVisible() {
        return isAnyTagVisible;
    }

    /**
     * Gets the list of currently active cameras.
     *
     * @return list of active cameras (empty if no detections)
     */
    public List<LimelightCamera> getActiveCameras() {
        return activeCameras;
    }

    /**
     * Gets the number of configured cameras.
     *
     * @return number of Limelight cameras
     */
    public int getCameraCount() {
        return robot.limelightCameras != null ? robot.limelightCameras.length : 0;
    }

    /**
     * Gets the number of healthy cameras.
     *
     * @return number of cameras with recent detections
     */
    public int getHealthyCameraCount() {
        if (robot.limelightCameras == null) return 0;

        int count = 0;
        for (LimelightCamera camera : robot.limelightCameras) {
            if (camera.isHealthy()) {
                count++;
            }
        }
        return count;
    }

    /**
     * Checks if any active camera detected a "floating" robot reading.
     *
     * <p>Floating robot occurs when Z-axis indicates unrealistic height,
     * often due to seeing elevated AprilTags or bad perspective.</p>
     *
     * @return true if any active camera has floating pose, false otherwise
     */
    private boolean checkAnyCameraFloating() {
        for (LimelightCamera camera : activeCameras) {
            if (camera.isPoseFloating()) {
                return true;
            }
        }
        return false;
    }

    /**
     * Calculates the effective position noise from active cameras.
     *
     * <p>If any camera has floating detection, returns increased noise.
     * Otherwise returns the default vision noise.</p>
     *
     * @return effective position noise (inches)
     */
    private double calculateEffectivePositionNoise() {
        // Check if any active camera has floating reading
        for (LimelightCamera camera : activeCameras) {
            if (camera.isPoseFloating()) {
                return camera.getEffectivePositionNoise();
            }
        }
        // No floating readings, use default
        return org.firstinspires.ftc.teamcode.Constants.FUSION_VISION_NOISE_POSITION;
    }

    /**
     * Calculates the effective heading noise from active cameras.
     *
     * <p>If any camera has floating detection, returns increased noise.
     * Otherwise returns the default vision noise.</p>
     *
     * @return effective heading noise (radians)
     */
    private double calculateEffectiveHeadingNoise() {
        // Check if any active camera has floating reading
        for (LimelightCamera camera : activeCameras) {
            if (camera.isPoseFloating()) {
                return camera.getEffectiveHeadingNoise();
            }
        }
        // No floating readings, use default
        return org.firstinspires.ftc.teamcode.Constants.FUSION_VISION_NOISE_HEADING;
    }
}
