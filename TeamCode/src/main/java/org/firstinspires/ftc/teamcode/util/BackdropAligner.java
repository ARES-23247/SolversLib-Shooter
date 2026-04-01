package org.firstinspires.ftc.teamcode.util;

import org.firstinspires.ftc.teamcode.hardware.SRSHub;

/**
 * Backdrop Alignment System using VL53L5CX sensor.
 *
 * <p>This class uses the VL53L5CX multi-zone ToF sensor to measure distance
 * to the backdrop and align the robot perpendicular to it for scoring.</p>
 *
 * <h3>Features:</h3>
 * <ul>
 *   <li>Measure distance to backdrop</li>
 *   <li>Detect if robot is parallel to backdrop</li>
 *   <li>Calculate alignment adjustment needed</li>
 *   <li>Detect when at optimal scoring distance</li>
 * </ul>
 *
 * <h3>Mounting:</h3>
 * <p>Mount the VL53L5CX at the front of the robot, centered with the scoring
 * mechanism, facing forward (horizontal, not angled).</p>
 *
 * <h3>Usage Example:</h3>
 * <pre>{@code
 * // In autonomous
 * BackdropAligner aligner = new BackdropAligner(srsHub);
 * aligner.initialize();
 *
 * while (!aligner.isAtScoringDistance()) {
 *     aligner.update();
 *     double adjustment = aligner.getAlignmentAdjustment();
 *     drive.drive(0, 0, adjustment * 0.01);  // Rotate to align
 * }
 * }</pre>
 *
 * @see VL53L5CX
 */
public class BackdropAligner {

    /**
     * Target scoring distance from backdrop (mm).
     */
    private static final int TARGET_DISTANCE_MM = 100;  // 10cm

    /**
     * Distance tolerance for "at scoring position" (mm).
     */
    private static final int DISTANCE_TOLERANCE_MM = 20;  // 2cm

    /**
     * Angle tolerance for "aligned with backdrop" (degrees).
     */
    private static final double ANGLE_TOLERANCE_DEG = 3.0;

    /**
     * Maximum distance to consider backdrop detected (mm).
     */
    private static final int MAX_BACKDROP_DISTANCE_MM = 1000;  // 1 meter

    private final VL53L5CX sensor;
    private boolean initialized = false;
    private VL53L5CX.DistanceFrame lastFrame;

    /**
     * Creates a backdrop aligner.
     *
     * @param srsHub SRS Hub for I2C communication
     */
    public BackdropAligner(SRSHub srsHub) {
        // Use 8×8 resolution for detailed alignment data
        this.sensor = new VL53L5CX(srsHub);
        try {
            sensor.setResolution(VL53L5CX.Resolution.RES_8X8);
        } catch (Exception e) {
            System.err.println("Failed to set resolution: " + e.getMessage());
        }
    }

    /**
     * Initializes the backdrop aligner.
     *
     * @return true if initialization succeeded
     */
    public boolean initialize() {
        initialized = sensor.initialize();
        if (initialized) {
            try {
                sensor.startRanging();
            } catch (Exception e) {
                System.err.println("Failed to start ranging: " + e.getMessage());
                initialized = false;
            }
        }
        return initialized;
    }

    /**
     * Updates the aligner with latest sensor data.
     *
     * <p>Call this once per loop iteration.</p>
     */
    public void update() {
        if (!initialized) return;

        try {
            VL53L5CX.DistanceFrame frame = sensor.readDistanceFrame(50);
            if (frame != null) {
                lastFrame = frame;
            }
        } catch (Exception e) {
            System.err.println("Failed to read distance: " + e.getMessage());
        }
    }

    /**
     * Checks if the backdrop is detected.
     *
     * @return true if an object is detected within MAX_BACKDROP_DISTANCE_MM
     */
    public boolean isBackdropDetected() {
        if (lastFrame == null) return false;

        int minDist = lastFrame.getMinDistanceMm();
        return minDist > 0 && minDist < MAX_BACKDROP_DISTANCE_MM;
    }

    /**
     * Gets the distance to the backdrop (center zone).
     *
     * @return Distance in millimeters, or -1 if not detected
     */
    public int getBackdropDistanceMm() {
        if (lastFrame == null) return -1;

        VL53L5CX.ZoneDistance center = lastFrame.getCenterZone();
        return center.isValid() ? center.distanceMm : -1;
    }

    /**
     * Gets the distance to the backdrop in inches.
     *
     * @return Distance in inches, or -1 if not detected
     */
    public double getBackdropDistanceInches() {
        int mm = getBackdropDistanceMm();
        return mm > 0 ? mm / 25.4 : -1;
    }

    /**
     * Checks if the robot is at the optimal scoring distance.
     *
     * @return true if within DISTANCE_TOLERANCE_MM of TARGET_DISTANCE_MM
     */
    public boolean isAtScoringDistance() {
        int distance = getBackdropDistanceMm();
        if (distance < 0) return false;

        int error = Math.abs(distance - TARGET_DISTANCE_MM);
        return error < DISTANCE_TOLERANCE_MM;
    }

    /**
     * Checks if the robot is parallel to the backdrop.
     *
     * <p>Compares left and right zones to detect angular misalignment.</p>
     *
     * @return true if aligned within ANGLE_TOLERANCE_DEG
     */
    public boolean isParallelToBackdrop() {
        if (!isBackdropDetected()) return false;

        VL53L5CX.ZoneDistance[][] zones = lastFrame.zones;
        int center = zones.length / 2;

        // Compare left half vs right half
        double leftDist = 0, rightDist = 0;
        int leftCount = 0, rightCount = 0;

        for (int row = 0; row < zones.length; row++) {
            for (int col = 0; col < zones.length; col++) {
                VL53L5CX.ZoneDistance zone = zones[row][col];
                if (zone.isValid() && zone.distanceMm < MAX_BACKDROP_DISTANCE_MM) {
                    if (col < center) {
                        leftDist += zone.distanceMm;
                        leftCount++;
                    } else {
                        rightDist += zone.distanceMm;
                        rightCount++;
                    }
                }
            }
        }

        if (leftCount == 0 || rightCount == 0) return true;

        double leftAvg = leftDist / leftCount;
        double rightAvg = rightDist / rightCount;

        // Calculate angle from distance difference
        double distanceDiff = Math.abs(leftAvg - rightAvg);
        double angleRad = Math.atan2(distanceDiff, 150);  // Assume 15cm sensor spread
        double angleDeg = Math.toDegrees(angleRad);

        return angleDeg < ANGLE_TOLERANCE_DEG;
    }

    /**
     * Calculates the alignment adjustment needed.
     *
     * <p>Positive = turn clockwise, Negative = turn counterclockwise</p>
     *
     * @return Adjustment value (-1.0 to 1.0), or 0 if aligned/not detected
     */
    public double getAlignmentAdjustment() {
        if (!isBackdropDetected()) return 0;
        if (isParallelToBackdrop()) return 0;

        VL53L5CX.ZoneDistance[][] zones = lastFrame.zones;
        int center = zones.length / 2;

        double leftDist = 0, rightDist = 0;
        int leftCount = 0, rightCount = 0;

        for (int row = 0; row < zones.length; row++) {
            for (int col = 0; col < zones.length; col++) {
                VL53L5CX.ZoneDistance zone = zones[row][col];
                if (zone.isValid() && zone.distanceMm < MAX_BACKDROP_DISTANCE_MM) {
                    if (col < center) {
                        leftDist += zone.distanceMm;
                        leftCount++;
                    } else {
                        rightDist += zone.distanceMm;
                        rightCount++;
                    }
                }
            }
        }

        if (leftCount == 0 || rightCount == 0) return 0;

        double leftAvg = leftDist / leftCount;
        double rightAvg = rightDist / rightCount;

        // Calculate angle
        double distanceDiff = rightAvg - leftAvg;
        double angleRad = Math.atan2(distanceDiff, 150);
        double angleDeg = Math.toDegrees(angleRad);

        // Normalize to -1 to 1 range
        double normalized = Math.max(-1, Math.min(1, angleDeg / 10.0));

        return -normalized;  // Invert so positive = clockwise
    }

    /**
     * Gets the distance error from target scoring position.
     *
     * @return Error in millimeters (positive = too far, negative = too close)
     */
    public int getDistanceError() {
        int distance = getBackdropDistanceMm();
        if (distance < 0) return 0;

        return distance - TARGET_DISTANCE_MM;
    }

    /**
     * Gets a human-readable status message.
     *
     * @return Status string describing current state
     */
    public String getStatusMessage() {
        if (!isBackdropDetected()) {
            return "No backdrop detected";
        }

        if (isAtScoringDistance() && isParallelToBackdrop()) {
            return String.format("Aligned at %.1fin", getBackdropDistanceInches());
        }

        if (!isAtScoringDistance()) {
            int error = getDistanceError();
            String direction = error > 0 ? "forward" : "backward";
            return String.format("Move %s %.1fin", direction, Math.abs(error) / 25.4);
        }

        if (!isParallelToBackdrop()) {
            double adjustment = getAlignmentAdjustment();
            String direction = adjustment > 0 ? "right" : "left";
            return String.format("Turn %s", direction);
        }

        return "Unknown";
    }

    /**
     * Gets the last complete distance frame.
     *
     * @return Last frame, or null if no data available
     */
    public VL53L5CX.DistanceFrame getLastFrame() {
        return lastFrame;
    }

    /**
     * Checks if the aligner is initialized.
     */
    public boolean isInitialized() {
        return initialized;
    }

    /**
     * Closes the aligner and releases resources.
     */
    public void close() {
        sensor.close();
        initialized = false;
    }
}
