package org.firstinspires.ftc.teamcode.util.vision;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.external.navigation.Pose3D;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * Advanced wrapper class for a Limelight3A camera with calibration, health monitoring, and multi-camera fusion support.
 *
 * <p>This class encapsulates:</p>
 * <ul>
 *   <li>The Limelight3A hardware instance</li>
 *   <li>Camera name, priority, and calibration data</li>
 *   <li>Health monitoring (detects failed/disconnected cameras)</li>
 *   <li>Current detection status with tag ID tracking</li>
 *   <li>Last measured pose (raw and calibrated)</li>
 * </ul>
 *
 * <h3>Camera Calibration:</h3>
 * <p>Each camera can have:</p>
 * <ul>
 *   <li><b>Mount Position:</b> Offset from robot center (for multi-camera fusion)</li>
 *   <li><b>Orientation Offset:</b> If camera isn't perfectly aligned</li>
 *   <li><b>Position Noise:</b> How much to trust this camera's X/Y measurements</li>
 *   <li><b>Heading Noise:</b> How much to trust this camera's heading</li>
 * </ul>
 *
 * <h3>Health Monitoring:</h3>
 * <p>Tracks how long since last valid detection. Camera marked unhealthy if timeout exceeded.</p>
 *
 * <h3>Tag ID Filtering:</h3>
 * <p>Can prefer specific tag IDs (e.g., use only tags on your alliance's side).</p>
 */
public class LimelightCamera {

    private final Limelight3A limelight;
    private final String name;
    private final double basePriority;

    // Calibration data
    private final double mountX;  // Mount position X offset (inches)
    private final double mountY;  // Mount position Y offset (inches)
    private final double orientationOffset;  // Orientation offset (radians)
    private final double positionNoise;  // Position measurement noise (inches)
    private final double headingNoise;  // Heading measurement noise (radians)

    // Current detection status
    private boolean hasValidDetection = false;
    private boolean isHealthy = true;  // Health monitoring
    private int visibleTagCount = 0;
    private int detectedTagID = -1;  // Which tag ID was detected
    private double detectionDistance = 0.0;
    private double priority = 0.0;

    // Timing for health monitoring
    private final ElapsedTime timeSinceLastDetection = new ElapsedTime();

    // Last pose measurement (inches and radians)
    private double poseX = 0.0;
    private double poseY = 0.0;
    private double poseHeading = 0.0;

    /**
     * Creates a new Limelight camera wrapper with calibration.
     *
     * @param limelight the Limelight3A hardware instance
     * @param name the camera name (for telemetry and logging)
     * @param basePriority the base priority (0.0 to 1.0, higher = preferred)
     */
    public LimelightCamera(Limelight3A limelight, String name, double basePriority) {
        this(limelight, name, basePriority, 0.0, 0.0, 0.0, 1.0, 0.03);
    }

    /**
     * Creates a new Limelight camera wrapper with full calibration.
     *
     * @param limelight the Limelight3A hardware instance
     * @param name the camera name
     * @param basePriority the base priority
     * @param mountX X offset of camera from robot center (inches)
     * @param mountY Y offset of camera from robot center (inches)
     * @param orientationOffset orientation offset (radians)
     * @param positionNoise position measurement noise std dev (inches)
     * @param headingNoise heading measurement noise std dev (radians)
     */
    public LimelightCamera(Limelight3A limelight, String name, double basePriority,
                           double mountX, double mountY, double orientationOffset,
                           double positionNoise, double headingNoise) {
        this.limelight = limelight;
        this.name = name;
        this.basePriority = basePriority;
        this.mountX = mountX;
        this.mountY = mountY;
        this.orientationOffset = orientationOffset;
        this.positionNoise = positionNoise;
        this.headingNoise = headingNoise;
        this.timeSinceLastDetection.reset();
    }

    /**
     * Polls the camera for new detections and updates internal state.
     *
     * <p>This method:</p>
     * <ol>
     *   <li>Gets latest result from Limelight</li>
     *   <li>Checks validity and filters by distance/tag count</li>
     *   <li>Finds preferred tag ID (if configured)</li>
     *   <li>Applies calibration (mount offset, orientation offset)</li>
     *   <li>Updates health status</li>
     * </ol>
     *
     * @param maxDistance maximum tag distance to trust (inches)
     * @param minTags minimum number of tags required
     * @param healthTimeout seconds before marking camera unhealthy
     * @param preferredTagIDs tag IDs to prefer (empty = no preference)
     */
    public void update(double maxDistance, int minTags, double healthTimeout,
                      int[] preferredTagIDs) {
        LLResult result = limelight.getLatestResult();

        hasValidDetection = false;
        visibleTagCount = 0;
        detectedTagID = -1;

        if (result != null && result.isValid()) {
            if (result.getFiducialResults() != null && result.getFiducialResults().size() >= minTags) {

                // Find preferred tag ID (if configured)
                Pose3D botpose = findBestTagPose(result, preferredTagIDs);
                if (botpose != null) {
                    // Convert from meters to inches
                    double rawX = botpose.getPosition().x * 39.3701;
                    double rawY = botpose.getPosition().y * 39.3701;
                    double rawHeading = botpose.getOrientation().getYaw(AngleUnit.RADIANS);

                    // Apply calibration (mount offset + orientation offset)
                    poseX = rawX + mountX;
                    poseY = rawY + mountY;
                    poseHeading = rawHeading + orientationOffset;

                    // Normalize heading
                    while (poseHeading > Math.PI) poseHeading -= 2 * Math.PI;
                    while (poseHeading < -Math.PI) poseHeading += 2 * Math.PI;

                    // Calculate distance from origin
                    detectionDistance = Math.hypot(poseX, poseY);

                    // Only consider valid if within max distance
                    if (detectionDistance <= maxDistance) {
                        hasValidDetection = true;
                        visibleTagCount = result.getFiducialResults().size();

                        // Get tag ID from the first fiducial result
                        if (!result.getFiducialResults().isEmpty()) {
                            detectedTagID = result.getFiducialResults().get(0).getId();
                        }

                        // Reset health timer on valid detection
                        timeSinceLastDetection.reset();
                        isHealthy = true;
                    }
                }
            }
        }

        // Check health timeout
        if (timeSinceLastDetection.seconds() > healthTimeout) {
            isHealthy = false;
        }

        // Reset priority to base (will be multiplied by adaptive logic)
        priority = basePriority;
    }

    /**
     * Finds the best tag pose from the Limelight result.
     *
     * <p>If preferred tag IDs are configured, chooses the closest preferred tag.
     * Otherwise, uses the default botpose (Limelight's best estimate).</p>
     *
     * @param result the Limelight result
     * @param preferredTagIDs tag IDs to prefer (empty = no preference)
     * @return the best tag pose, or null if none found
     */
    private Pose3D findBestTagPose(LLResult result, int[] preferredTagIDs) {
        if (preferredTagIDs == null || preferredTagIDs.length == 0) {
            // No preference, use Limelight's default botpose
            return result.getBotpose();
        }

        // Find closest preferred tag
        Pose3D bestPose = null;
        double bestDistance = Double.MAX_VALUE;

        for (var fiducial : result.getFiducialResults()) {
            int tagID = fiducial.getId();

            // Check if this is a preferred tag
            for (int preferredID : preferredTagIDs) {
                if (tagID == preferredID) {
                    Pose3D tagPose = fiducial.getRobotPose();
                    if (tagPose != null) {
                        double distance = Math.hypot(
                            tagPose.getPosition().x * 39.3701,
                            tagPose.getPosition().y * 39.3701
                        );

                        if (distance < bestDistance) {
                            bestDistance = distance;
                            bestPose = tagPose;
                        }
                    }
                    break;
                }
            }
        }

        return bestPose != null ? bestPose : result.getBotpose();
    }

    /**
     * Simple update method (backward compatible).
     */
    public void update(double maxDistance, int minTags) {
        update(maxDistance, minTags, Double.MAX_VALUE, new int[0]);
    }

    /**
     * Sets the priority multiplier for adaptive camera selection.
     *
     * @param multiplier priority multiplier (1.0 = normal, 2.0 = double priority)
     */
    public void setPriorityMultiplier(double multiplier) {
        this.priority = basePriority * multiplier;
    }

    /**
     * Gets the confidence weight for multi-camera fusion.
     *
     * <p>Weight = priority / (distance + 1) × tagCount</p>
     *
     * @return confidence weight (higher = better)
     */
    public double getConfidenceWeight() {
        if (!hasValidDetection || !isHealthy) {
            return 0.0;
        }
        double distanceWeight = 1.0 / (detectionDistance + 1.0);
        double tagWeight = visibleTagCount;
        return priority * tagWeight * distanceWeight;
    }

    /**
     * Checks if this camera currently has a valid tag detection.
     *
     * @return true if tags are visible and within max distance
     */
    public boolean hasValidDetection() {
        return hasValidDetection;
    }

    /**
     * Checks if this camera is healthy (has recent detections).
     *
     * @return true if camera has had valid detections recently
     */
    public boolean isHealthy() {
        return isHealthy;
    }

    /**
     * Gets the time since last valid detection (seconds).
     *
     * @return seconds since last detection
     */
    public double getTimeSinceLastDetection() {
        return timeSinceLastDetection.seconds();
    }

    /**
     * Gets the number of visible tags.
     *
     * @return number of AprilTags currently visible
     */
    public int getVisibleTagCount() {
        return visibleTagCount;
    }

    /**
     * Gets the ID of the detected tag.
     *
     * @return tag ID (-1 if no tag detected)
     */
    public int getDetectedTagID() {
        return detectedTagID;
    }

    /**
     * Gets the distance to the detected tags (inches).
     *
     * @return distance from robot to tags
     */
    public double getDetectionDistance() {
        return detectionDistance;
    }

    /**
     * Gets the current priority (including adaptive multipliers).
     *
     * @return priority value (higher = preferred)
     */
    public double getPriority() {
        return priority;
    }

    /**
     * Gets the camera's position noise (for sensor fusion).
     *
     * @return position noise std dev (inches)
     */
    public double getPositionNoise() {
        return positionNoise;
    }

    /**
     * Gets the camera's heading noise (for sensor fusion).
     *
     * @return heading noise std dev (radians)
     */
    public double getHeadingNoise() {
        return headingNoise;
    }

    /**
     * Gets the camera name.
     *
     * @return camera name
     */
    public String getName() {
        return name;
    }

    /**
     * Gets the Limelight3A hardware instance.
     *
     * @return the limelight
     */
    public Limelight3A getLimelight() {
        return limelight;
    }

    // ===== Pose Getters =====

    /**
     * Gets the measured X position (inches, calibrated).
     *
     * @return X position in inches
     */
    public double getPoseX() {
        return poseX;
    }

    /**
     * Gets the measured Y position (inches, calibrated).
     *
     * @return Y position in inches
     */
    public double getPoseY() {
        return poseY;
    }

    /**
     * Gets the measured heading (radians, calibrated).
     *
     * @return heading in radians
     */
    public double getPoseHeading() {
        return poseHeading;
    }

    /**
     * Gets a formatted string describing the current detection status.
     *
     * @return status string (e.g., "limelightFront: 2 tags @ 48in [Tag #1]")
     */
    public String getStatusString() {
        if (hasValidDetection) {
            return String.format("%s: %d tags @ %.0fin [Tag #%d]%s",
                name, visibleTagCount, detectionDistance, detectedTagID,
                isHealthy ? "" : " [UNHEALTHY]");
        } else {
            return String.format("%s: No tags [%s]",
                name, isHealthy ? "OK" : "UNHEALTHY");
        }
    }
}
