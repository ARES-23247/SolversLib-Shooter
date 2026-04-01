package org.firstinspires.ftc.teamcode.util.vision;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * Utility for calibrating Limelight3A camera position and orientation.
 *
 * <p>This class helps you determine:</p>
 * <ul>
 *   <li><b>Camera Mount Position:</b> Where is the camera relative to robot center?</li>
 *   <li><b>Camera Orientation Offset:</b> Is the camera rotated? Pointing straight?</li>
 *   <li><b>Camera Accuracy:</b> How accurate are the measurements?</li>
 * </ul>
 *
 * <h3>Calibration Process:</h3>
 * <ol>
 *   <li>Place robot at known position relative to AprilTag</li>
 *   <li>Read what Limelight reports</li>
 *   <li>Compare expected vs actual to calculate offset</li>
 *   <li>Update Constants.java with calibration values</li>
 * </ol>
 *
 * <h3>Calibration Scenarios:</h3>
 *
 * <h4>1. Mount Position Calibration</h4>
 * <p><b>Setup:</b> Place robot center 24 inches directly facing AprilTag #1</p>
 * <p><b>Expected:</b> Limelight reports (24, 0, 0) - 24 inches forward, 0 lateral, 0 heading</p>
 * <p><b>If camera reports:</b> (30, 0, 0)</p>
 * <p><b>Offset:</b> Camera is mounted 6 inches forward of robot center</p>
 * <pre>{@code
 * LIMELIGHT_MOUNT_POSITIONS[0] = {6.0, 0.0};  // 6 inches forward
 * }</pre>
 *
 * <h4>2. Orientation Offset Calibration</h4>
 * <p><b>Setup:</b> Same as above (24 inches facing tag)</p>
 * <p><b>Expected:</b> Heading = 0 radians (pointing at tag)</p>
 * <p><b>If camera reports:</b> Heading = 0.1 radians (~5.7°)</p>
 * <p><b>Offset:</b> Camera is rotated 5.7° clockwise</p>
 * <pre>{@code
 * LIMELIGHT_ORIENTATION_OFFSETS[0] = -0.1;  // Rotate counter-clockwise to fix
 * }</pre>
 *
 * <h4>3. Dual-Camera Calibration</h4>
 * <p>Calibrate multiple cameras by placing robot at different positions.</p>
 *
 * <h3>Usage Example:</h3>
 * <pre>{@code
 * // In your calibration OpMode:
 * LimelightCalibrator calibrator = new LimelightCalibrator(limelight, telemetry);
 *
 * // Scenario 1: Robot center at 24 inches from tag #1, facing it
 * CalibrationResult result = calibrator.calibrateMountPosition(
 *     24.0,  // Expected X distance (inches)
 *     0.0,    // Expected Y distance (inches)
 *     0.0     // Expected heading (radians)
 * );
 *
 * if (result.success) {
 *     System.out.println("Camera is mounted at:");
 *     System.out.println(String.format("  X offset: %.2f inches", result.calibratedX));
 *     System.out.println(String.format("  Y offset: %.2f inches", result.calibratedY));
 * }
 * }</pre>
 *
 * @see com.qualcomm.hardware.limelightvision.Limelight3A
 */
public class LimelightCalibrator {

    private final Limelight3A limelight;
    private final Telemetry telemetry;

    /**
     * Result of a calibration operation.
     */
    public static class CalibrationResult {
        /** Whether calibration was successful */
        public boolean success;

        /** Expected position (what we placed the robot at) */
        public double expectedX;
        public double expectedY;
        public double expectedHeading;

        /** Measured position (what Limelight reported) */
        public double measuredX;
        public double measuredY;
        public double measuredHeading;

        /** Calibration offsets (to add to Constants) */
        public double calibratedX;
        public double calibratedY;
        public double calibratedHeading;

        /** Tag ID that was detected */
        public int tagID;

        /** Number of tags visible */
        public int tagCount;

        /** Distance to tag (inches) */
        public double distance;

        public CalibrationResult() {
            success = false;
        }

        /**
         * Creates a successful calibration result.
         */
        public CalibrationResult(double expectedX, double expectedY, double expectedHeading,
                                double measuredX, double measuredY, double measuredHeading,
                                int tagID, int tagCount, double distance) {
            this.success = true;
            this.expectedX = expectedX;
            this.expectedY = expectedY;
            this.expectedHeading = expectedHeading;
            this.measuredX = measuredX;
            this.measuredY = measuredY;
            this.measuredHeading = measuredHeading;
            this.tagID = tagID;
            this.tagCount = tagCount;
            this.distance = distance;

            // Calculate offsets
            // Mount position = measured - expected (where camera is vs robot center)
            this.calibratedX = measuredX - expectedX;
            this.calibratedY = measuredY - expectedY;

            // Orientation offset = measured - expected (how much camera is rotated)
            this.calibratedHeading = measuredHeading - expectedHeading;

            // Normalize to -PI to PI
            while (this.calibratedHeading > Math.PI) this.calibratedHeading -= 2 * Math.PI;
            while (this.calibratedHeading < -Math.PI) this.calibratedHeading += 2 * Math.PI;
        }

        /**
         * Gets a formatted string for Constants.java.
         */
        public String getConstantsFormat() {
            return String.format(
                "  {%s: %.2f, %s: %.2f},  // %s\n",
                "Calibrated X", calibratedX,
                "Calibrated Y", calibratedY,
                getCalibrationSummary()
            );
        }

        /**
         * Gets a brief summary of the calibration.
         */
        public String getCalibrationSummary() {
            if (!success) {
                return "Calibration failed";
            }

            double xOffsetInches = calibratedX;
            double yOffsetInches = calibratedY;
            double headingDegrees = Math.toDegrees(calibratedHeading);

            StringBuilder summary = new StringBuilder();

            // Position offset
            double offsetMagnitude = Math.hypot(xOffsetInches, yOffsetInches);
            if (offsetMagnitude > 0.5) {
                String direction = getOffsetDirection(xOffsetInches, yOffsetInches);
                summary.append(String.format("Mounted %.1f\" %s", offsetMagnitude, direction));
            } else {
                summary.append("Centered on robot");
            }

            // Orientation offset
            if (Math.abs(headingDegrees) > 2.0) {
                summary.append(String.format(", rotated %.1f° %s",
                    Math.abs(headingDegrees),
                    headingDegrees > 0 ? "CCW" : "CW"
                ));
            } else {
                summary.append(", aligned");
            }

            return summary.toString();
        }

        /**
         * Gets direction string for offset.
         */
        private String getOffsetDirection(double x, double y) {
            String primary = Math.abs(x) > Math.abs(y) ?
                (x > 0 ? "forward" : "backward") :
                (y > 0 ? "left" : "right");

            String secondary = Math.abs(x) > Math.abs(y) ?
                (y > 0 ? "slightly left" : "slightly right") :
                (x > 0 ? "slightly forward" : "slightly backward");

            return primary + "-" + secondary;
        }

        /**
         * Prints detailed calibration report to telemetry.
         */
        public void printReport() {
            if (!success) {
                System.out.println("Calibration FAILED");
                return;
            }

            System.out.println("===========================================");
            System.out.println("   LIMELIGHT CALIBRATION RESULTS");
            System.out.println("===========================================");
            System.out.println("");
            System.out.println("EXPECTED POSITION (Robot Placement):");
            System.out.println(String.format("  X: %.2f inches", expectedX));
            System.out.println(String.format("  Y: %.2f inches", expectedY));
            System.out.println(String.format("  Heading: %.1f°", Math.toDegrees(expectedHeading)));
            System.out.println("");
            System.out.println("MEASURED POSITION (Limelight):");
            System.out.println(String.format("  X: %.2f inches", measuredX));
            System.out.println(String.format("  Y: %.2f inches", measuredY));
            System.out.println(String.format("  Heading: %.1f°", Math.toDegrees(measuredHeading)));
            System.out.println("");
            System.out.println("CALIBRATED OFFSETS:");
            System.out.println(String.format("  X Offset: %.2f inches", calibratedX));
            System.out.println(String.format("  Y Offset: %.2f inches", calibratedY));
            System.out.println(String.format("  Heading Offset: %.2f° (%.4f rad)",
                Math.toDegrees(calibratedHeading), calibratedHeading));
            System.out.println("");
            System.out.println("SUMMARY: " + getCalibrationSummary());
            System.out.println("");
            System.out.println("DETECTION DETAILS:");
            System.out.println(String.format("  Tag ID: %d", tagID));
            System.out.println(String.format("  Tags Visible: %d", tagCount));
            System.out.println(String.format("  Distance: %.1f inches", distance));
            System.out.println("");
            System.out.println("UPDATE Constants.java WITH:");
            System.out.println(String.format("  LIMELIGHT_MOUNT_POSITIONS[0] = {%.2f, %.2f};",
                calibratedX, calibratedY));
            System.out.println(String.format("  LIMELIGHT_ORIENTATION_OFFSETS[0] = %.4f;  // %.1f°",
                calibratedHeading, Math.toDegrees(calibratedHeading)));
            System.out.println("===========================================");
        }
    }

    /**
     * Creates a new Limelight calibrator.
     *
     * @param limelight the Limelight camera to calibrate
     * @param telemetry telemetry instance for output
     */
    public LimelightCalibrator(Limelight3A limelight, Telemetry telemetry) {
        this.limelight = limelight;
        this.telemetry = telemetry;
    }

    /**
     * Calibrates camera mount position and orientation.
     *
     * <h3>Setup Instructions:</h3>
     * <ol>
     *   <li>Place robot center at exact position relative to AprilTag</li>
     *   <li>Face the AprilTag directly (robot heading = 0° relative to tag)</li>
     *   <li>Run this method</li>
     * </ol>
     *
     * <h3>Recommended Calibration Position:</h3>
     * <ul>
     *   <li>Distance: 24-36 inches from tag (close = more accurate)</li>
     *   <li>Heading: 0° (directly facing tag)</li>
     *   <li>Lateral: 0 inches (directly in front of tag)</li>
     * </ul>
     *
     * @param expectedX expected X distance from tag (inches, forward+)
     * @param expectedY expected Y distance from tag (inches, left+)
     * @param expectedHeading expected heading relative to tag (radians, 0 = facing tag)
     * @return calibration result with offsets
     */
    public CalibrationResult calibrateMountPosition(double expectedX, double expectedY, double expectedHeading) {
        System.out.println("===========================================");
        System.out.println("   LIMELIGHT CALIBRATION STARTING");
        System.out.println("===========================================");
        System.out.println("");
        System.out.println("SETUP INSTRUCTIONS:");
        System.out.println("1. Place robot center at exact position:");
        System.out.println(String.format("   X: %.1f inches (forward+)", expectedX));
        System.out.println(String.format("   Y: %.1f inches (left+)", expectedY));
        System.out.println(String.format("   Heading: %.1f° (facing tag)", Math.toDegrees(expectedHeading)));
        System.out.println("");
        System.out.println("2. Robot must be stationary");
        System.out.println("");
        System.out.println("Reading camera values...");

        // Read from Limelight
        com.qualcomm.hardware.limelightvision.LLResult result = limelight.getLatestResult();
        CalibrationResult calibrationResult = new CalibrationResult();

        if (result != null && result.isValid()) {
            if (result.getFiducialResults() != null && !result.getFiducialResults().isEmpty()) {
                Pose3D botpose = result.getBotpose();

                if (botpose != null) {
                    // Convert from meters to inches
                    double measuredX = botpose.getPosition().x * 39.3701;
                    double measuredY = botpose.getPosition().y * 39.3701;
                    double measuredHeading = botpose.getOrientation().getYaw(AngleUnit.RADIANS);

                    int tagID = result.getFiducialResults().get(0).getFiducialId();
                    int tagCount = result.getFiducialResults().size();
                    double distance = Math.hypot(measuredX, measuredY);

                    System.out.println(String.format("Measured: X=%.1f\", Y=%.1f\", Heading=%.1f°",
                        measuredX, measuredY, Math.toDegrees(measuredHeading)));
                    System.out.println(String.format("Tag #%d visible, %d total tags, %.1f\" away",
                        tagID, tagCount, distance));

                    calibrationResult = new CalibrationResult(
                        expectedX, expectedY, expectedHeading,
                        measuredX, measuredY, measuredHeading,
                        tagID, tagCount, distance
                    );

                    calibrationResult.printReport();
                    return calibrationResult;
                }
            }
        }

        System.out.println("");
        System.out.println("CALIBRATION FAILED!");
        System.out.println("No AprilTag detected. Check:");
        System.out.println("  - Camera is connected and polling");
        System.out.println("  - AprilTag is in view");
        System.out.println("  - Pipeline is set to AprilTag (pipeline 0)");
        System.out.println("===========================================");

        return calibrationResult;
    }

    /**
     * Verifies camera accuracy by measuring known position.
     *
     * <p>Place robot at known position and run this to check accuracy.</p>
     *
     * @param expectedX expected X (inches)
     * @param expectedY expected Y (inches)
     * @param expectedHeading expected heading (radians)
     * @param tolerance allowable error (inches for position, radians for heading)
     * @return true if measurement is within tolerance
     */
    public boolean verifyAccuracy(double expectedX, double expectedY, double expectedHeading,
                                 double positionTolerance, double headingTolerance) {
        System.out.println("===========================================");
        System.out.println("   LIMELIGHT ACCURACY VERIFICATION");
        System.out.println("===========================================");
        System.out.println("");

        com.qualcomm.hardware.limelightvision.LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid() &&
            result.getFiducialResults() != null && !result.getFiducialResults().isEmpty()) {

            Pose3D botpose = result.getBotpose();
            if (botpose != null) {
                double measuredX = botpose.getPosition().x * 39.3701;
                double measuredY = botpose.getPosition().y * 39.3701;
                double measuredHeading = botpose.getOrientation().getYaw(AngleUnit.RADIANS);

                double errorX = Math.abs(measuredX - expectedX);
                double errorY = Math.abs(measuredY - expectedY);
                double errorHeading = Math.abs(measuredHeading - expectedHeading);

                // Normalize heading error
                while (errorHeading > Math.PI) errorHeading -= 2 * Math.PI;
                while (errorHeading < -Math.PI) errorHeading += 2 * Math.PI;

                System.out.println(String.format("Expected: X=%.1f\", Y=%.1f\", H=%.1f°",
                    expectedX, expectedY, Math.toDegrees(expectedHeading)));
                System.out.println(String.format("Measured: X=%.1f\", Y=%.1f\", H=%.1f°",
                    measuredX, measuredY, Math.toDegrees(measuredHeading)));
                System.out.println("");
                System.out.println(String.format("Errors: X=%.2f\", Y=%.2f\", H=%.2f°",
                    errorX, errorY, Math.toDegrees(errorHeading)));
                System.out.println("");
                System.out.println(String.format("Tolerances: X=%.2f\", Y=%.2f\", H=%.2f°",
                    positionTolerance, positionTolerance, Math.toDegrees(headingTolerance)));
                System.out.println("");

                boolean positionOK = (errorX <= positionTolerance && errorY <= positionTolerance);
                boolean headingOK = (errorHeading <= headingTolerance);

                if (positionOK && headingOK) {
                    System.out.println("✓ ACCURACY VERIFIED - Within tolerance!");
                    return true;
                } else {
                    System.out.println("✗ ACCURACY FAILED - Outside tolerance!");
                    if (!positionOK) System.out.println("  Position error too large");
                    if (!headingOK) System.out.println("  Heading error too large");
                    return false;
                }
            }
        }

        System.out.println("✗ VERIFICATION FAILED - No tag detected");
        System.out.println("===========================================");
        return false;
    }

    /**
     * Scans for visible tags and reports their positions.
     *
     * <p>Useful for checking which tags the camera can see.</p>
     */
    public void scanForTags() {
        System.out.println("===========================================");
        System.out.println("   LIMELIGHT TAG SCAN");
        System.out.println("===========================================");
        System.out.println("");

        com.qualcomm.hardware.limelightvision.LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            if (result.getFiducialResults() != null && !result.getFiducialResults().isEmpty()) {
                System.out.println(String.format("Tags visible: %d", result.getFiducialResults().size()));
                System.out.println("");

                for (com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult fiducial : result.getFiducialResults()) {
                    int id = fiducial.getFiducialId();
                    Pose3D pose = fiducial.getRobotPoseTargetSpace();

                    if (pose != null) {
                        double x = pose.getPosition().x * 39.3701;
                        double y = pose.getPosition().y * 39.3701;
                        double heading = pose.getOrientation().getYaw(AngleUnit.RADIANS);

                        System.out.println(String.format("Tag #%d: X=%.1f\", Y=%.1f\", H=%.1f°",
                            id, x, y, Math.toDegrees(heading)));
                    }
                }
            } else {
                System.out.println("No tags visible");
            }
        } else {
            System.out.println("Limelight not returning valid results");
        }

        System.out.println("===========================================");
    }

    /**
     * Telemetry interface for compatibility.
     */
    public interface Telemetry {
        Log log();

        interface Log {
            void add(String message);
        }
    }
}
