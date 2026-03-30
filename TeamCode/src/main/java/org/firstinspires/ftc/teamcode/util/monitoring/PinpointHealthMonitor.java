package org.firstinspires.ftc.teamcode.util.monitoring;

/**
 * Health monitor for detecting odometry sensor drift.
 *
 * <p>This class monitors odometry sensors (like GoBilda Pinpoint) for signs of drift
 * or malfunction. It compares the sensor readings against a trusted reference (like
 * vision localization) and detects significant deviations.</p>
 *
 * <h3>Drift Detection:</h3>
 * <p>Drift is detected when:</p>
 * <ol>
 *   <li>Sensor position deviates from reference by more than threshold</li>
 *   <li>Deviation persists for multiple consecutive readings</li>
 *   <li>Sensor velocity is inconsistent with commanded velocity</li>
 * </ol>
 *
 * <h3>Usage:</h3>
 * <pre>{@code
 * // Create health monitor
 * PinpointHealthMonitor monitor = new PinpointHealthMonitor();
 *
 * // In control loop:
 * double pinpointX = pinpoint.getX();
 * double pinpointY = pinpoint.getY();
 * double visionX = limelight.getBotpose().getX();
 * double visionY = limelight.getBotpose().getY();
 *
 * if (monitor.isHealthy(pinpointX, pinpointY, visionX, visionY)) {
 *     // Pinpoint is healthy, use it
 *     fusion.correctWithPinpoint(pinpointX, pinpointY, heading);
 * } else {
 *     // Pinpoint is drifting, ignore it
 *     // Only use swerve odometry + vision
 * }
 * }}</pre>
 *
 * @see SensorFusionLocalizer
 */
public class PinpointHealthMonitor {

    /**
     * Maximum allowed position deviation before sensor is considered unhealthy (inches).
     * If Pinpoint deviates from vision by more than this, it's flagged as drifted.
     */
    private final double positionThreshold;

    /**
     * Number of consecutive bad readings before sensor is marked as unhealthy.
     * Prevents false positives from temporary vision noise.
     */
    private final int consecutiveBadReadingsThreshold;

    /**
     * Current count of consecutive bad readings.
     */
    private int badReadingCount = 0;

    /**
     * Whether the sensor is currently healthy.
     */
    private boolean isHealthy = true;

    /**
     * Timestamp of the last health state change.
     */
    private long lastStateChangeTime = System.nanoTime();

    /**
     * Creates a health monitor with default thresholds.
     *
     * <p>Defaults: 6-inch threshold, 5 consecutive bad readings</p>
     */
    public PinpointHealthMonitor() {
        this(6.0, 5);
    }

    /**
     * Creates a health monitor with custom thresholds.
     *
     * @param positionThreshold maximum position deviation (inches)
     * @param consecutiveBadReadingsThreshold number of bad readings before marking unhealthy
     */
    public PinpointHealthMonitor(double positionThreshold, int consecutiveBadReadingsThreshold) {
        this.positionThreshold = positionThreshold;
        this.consecutiveBadReadingsThreshold = consecutiveBadReadingsThreshold;
    }

    /**
     * Checks if Pinpoint is healthy by comparing against reference position.
     *
     * <p>This method compares the Pinpoint reading against a trusted reference (like vision).
     * If the deviation exceeds the threshold for multiple consecutive readings, the sensor
     * is marked as unhealthy.</p>
     *
     * @param pinpointX Pinpoint X position (inches)
     * @param pinpointY Pinpoint Y position (inches)
     * @param referenceX Reference X position (inches, typically from vision or EKF)
     * @param referenceY Reference Y position (inches, typically from vision or EKF)
     * @return true if Pinpoint is healthy, false if it's drifted
     */
    public boolean isHealthy(double pinpointX, double pinpointY, double referenceX, double referenceY) {
        // Calculate deviation from reference
        double deviation = Math.hypot(pinpointX - referenceX, pinpointY - referenceY);

        boolean isReadingBad = deviation > positionThreshold;

        if (isReadingBad) {
            badReadingCount++;

            // Mark as unhealthy if threshold exceeded
            if (badReadingCount >= consecutiveBadReadingsThreshold) {
                if (isHealthy) {
                    // State change: healthy -> unhealthy
                    isHealthy = false;
                    lastStateChangeTime = System.nanoTime();
                }
            }
        } else {
            // Reading is good, reset counter
            badReadingCount = 0;

            if (!isHealthy) {
                // State change: unhealthy -> healthy
                isHealthy = true;
                lastStateChangeTime = System.nanoTime();
            }
        }

        return isHealthy;
    }

    /**
     * Checks if the sensor is currently healthy (without updating state).
     *
     * @return true if sensor is healthy, false if drifted
     */
    public boolean isHealthy() {
        return isHealthy;
    }

    /**
     * Gets the current bad reading count.
     *
     * @return number of consecutive bad readings
     */
    public int getBadReadingCount() {
        return badReadingCount;
    }

    /**
     * Gets the time since last health state change (seconds).
     *
     * @return time in seconds since health state last changed
     */
    public double getTimeSinceStateChange() {
        return (System.nanoTime() - lastStateChangeTime) / 1e9;
    }

    /**
     * Resets the health monitor to healthy state.
     *
     * <p>Use this after re-calibrating Pinpoint or fixing the issue.</p>
     */
    public void reset() {
        badReadingCount = 0;
        isHealthy = true;
        lastStateChangeTime = System.nanoTime();
    }

    /**
     * Checks if it's safe to reset Pinpoint to vision pose.
     *
     * <p>Safe reset conditions:</p>
     * <ul>
     *   <li>Robot is stationary (low velocity)</li>
     *   <li>Vision data is available (tag visible)</li>
     *   <li>Vision position uncertainty is low (confident measurement)</li>
     * </ul>
     *
     * <p>This prevents resetting Pinpoint while the robot is moving, which could
     * cause sudden pose jumps and disrupt odometry.</p>
     *
     * @param robotVelocity current robot velocity magnitude (inches/second)
     * @param visionUncertainty vision position uncertainty (inches)
     * @param visionTagVisible whether at least one AprilTag is visible
     * @param maxStationaryVelocity maximum velocity to be considered "stationary" (inches/sec)
     * @param maxVisionUncertainty maximum vision uncertainty for safe reset (inches)
     * @return true if safe to reset Pinpoint, false otherwise
     */
    public boolean isSafeToReset(double robotVelocity, double visionUncertainty,
                                  boolean visionTagVisible, double maxStationaryVelocity,
                                  double maxVisionUncertainty) {
        boolean isStationary = robotVelocity < maxStationaryVelocity;
        boolean hasGoodVision = visionTagVisible && visionUncertainty < maxVisionUncertainty;

        return isStationary && hasGoodVision;
    }
}
