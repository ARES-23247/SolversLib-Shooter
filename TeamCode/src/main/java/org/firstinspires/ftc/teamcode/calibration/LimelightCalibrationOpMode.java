package org.firstinspires.ftc.teamcode.calibration;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.globals.Robot;
import org.firstinspires.ftc.teamcode.util.vision.LimelightCalibrator;
import org.firstinspires.ftc.teamcode.util.vision.LimelightCamera;

/**
 * OpMode for calibrating Limelight3A camera position and orientation.
 *
 * <h3>Calibration Process:</h3>
 * <ol>
 *   <li>Place robot at known position relative to AprilTag (e.g., 24 inches facing tag #1)</li>
 *   <li>Press A to run calibration</li>
 *   <li>Read the calibrated offsets from telemetry</li>
 *   <li>Update Constants.java with the calibration values</li>
 *   <li>Press B to verify accuracy (optional)</li>
 * </ol>
 *
 * <h3>Recommended Calibration Position:</h3>
 * <ul>
 *   <li>Distance: 24-36 inches from tag (close = more accurate)</li>
 *   <li>Heading: 0° (directly facing tag)</li>
 *   <li>Lateral: 0 inches (directly in front of tag)</li>
 * </ul>
 *
 * <h3>Updating Constants.java:</h3>
 * <p>After calibration, update the camera index in Constants.java:</p>
 * <pre>{@code
 * LIMELIGHT_MOUNT_POSITIONS[0] = {calibratedX, calibratedY};
 * LIMELIGHT_ORIENTATION_OFFSETS[0] = calibratedHeading;
 * }</pre>
 *
 * @see org.firstinspires.ftc.teamcode.util.LimelightCalibrator
 */
@TeleOp(name = "Limelight Calibration", group = "Calibration")
public class LimelightCalibrationOpMode extends LinearOpMode {

    private Robot robot;
    private LimelightCalibrator calibrator;

    // Calibration parameters (adjustable via gamepad)
    private double expectedX = 24.0;  // Default: 24 inches from tag
    private double expectedY = 0.0;   // Default: centered laterally
    private double expectedHeading = 0.0;  // Default: facing tag

    // Verification tolerances
    private double positionTolerance = 2.0;  // inches
    private double headingTolerance = Math.toRadians(5.0);  // 5 degrees

    // Which camera to calibrate (for multi-camera setups)
    private int cameraIndex = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap, telemetry);

        // Initialize calibrator with first camera (or selected camera)
        if (robot.limelightCameras != null && robot.limelightCameras.length > 0) {
            calibrator = new LimelightCalibrator(
                robot.limelightCameras[cameraIndex].getLimelight(),
                new TelemetryAdapter()
            );
        } else {
            telemetry.log().add("ERROR: No Limelight cameras found!");
            telemetry.log().add("Check Robot.java initialization");
            telemetry.update();
            return;
        }

        // Show calibration instructions
        showInstructions();

        waitForStart();

        telemetry.log().add("Calibration OpMode started");
        telemetry.update();

        while (opModeIsActive()) {
            // Handle calibration controls
            handleCalibrationControls();

            // Update telemetry
            updateStatus();

            sleep(50);  // 20Hz update rate
        }

        telemetry.log().add("Calibration OpMode ended");
        telemetry.update();
    }

    /**
     * Displays initial calibration instructions.
     */
    private void showInstructions() {
        telemetry.log().add("===========================================");
        telemetry.log().add("   LIMELIGHT CALIBRATION");
        telemetry.log().add("===========================================");
        telemetry.log().add("");
        telemetry.log().add("SETUP:");
        telemetry.log().add("1. Place robot at known position:");
        telemetry.log().add("   - " + String.format("%.1f", expectedX) + " inches from tag (X)");
        telemetry.log().add("   - " + String.format("%.1f", expectedY) + " inches lateral (Y)");
        telemetry.log().add("   - " + String.format("%.1f°", Math.toDegrees(expectedHeading)) + " heading");
        telemetry.log().add("");
        telemetry.log().add("2. Robot must be stationary");
        telemetry.log().add("");
        telemetry.log().add("CONTROLS:");
        telemetry.log().add("A: Run calibration");
        telemetry.log().add("B: Verify accuracy");
        telemetry.log().add("X: Scan for visible tags");
        telemetry.log().add("Y: Adjust expected X (+/- with D-pad up/down)");
        telemetry.log().add("Left Bumper: Previous camera");
        telemetry.log().add("Right Bumper: Next camera");
        telemetry.log().add("");
        telemetry.log().add("===========================================");
        telemetry.update();
    }

    /**
     * Handles gamepad controls for calibration.
     */
    private void handleCalibrationControls() {
        // Adjust expected X distance
        if (gamepad1.dpad_up) {
            expectedX += 1.0;
            telemetry.log().add(String.format("Expected X: %.1f inches", expectedX));
        } else if (gamepad1.dpad_down) {
            expectedX = Math.max(6.0, expectedX - 1.0);  // Minimum 6 inches
            telemetry.log().add(String.format("Expected X: %.1f inches", expectedX));
        }

        // Adjust expected Y position
        if (gamepad1.dpad_left) {
            expectedY -= 1.0;
            telemetry.log().add(String.format("Expected Y: %.1f inches", expectedY));
        } else if (gamepad1.dpad_right) {
            expectedY += 1.0;
            telemetry.log().add(String.format("Expected Y: %.1f inches", expectedY));
        }

        // Select camera (for multi-camera setups)
        if (gamepad1.left_bumper) {
            cameraIndex = Math.max(0, cameraIndex - 1);
            if (robot.limelightCameras.length > cameraIndex) {
                calibrator = new LimelightCalibrator(
                    robot.limelightCameras[cameraIndex].getLimelight(),
                    new TelemetryAdapter()
                );
                telemetry.log().add("Selected camera: " + robot.limelightCameras[cameraIndex].getName());
            }
        } else if (gamepad1.right_bumper) {
            cameraIndex = Math.min(robot.limelightCameras.length - 1, cameraIndex + 1);
            if (robot.limelightCameras.length > cameraIndex) {
                calibrator = new LimelightCalibrator(
                    robot.limelightCameras[cameraIndex].getLimelight(),
                    new TelemetryAdapter()
                );
                telemetry.log().add("Selected camera: " + robot.limelightCameras[cameraIndex].getName());
            }
        }

        // Run calibration
        if (gamepad1.a && !gamepad1.touchpad) {  // Prevent repeated triggers
            telemetry.log().add("Running calibration...");
            telemetry.update();

            LimelightCalibrator.CalibrationResult result = calibrator.calibrateMountPosition(
                expectedX,
                expectedY,
                expectedHeading
            );

            if (result.success) {
                telemetry.log().add("");
                telemetry.log().add("CALIBRATION SUCCESSFUL!");
                telemetry.log().add("");
                telemetry.log().add("Update Constants.java:");
                telemetry.log().add(String.format("LIMELIGHT_MOUNT_POSITIONS[%d] = {%.2f, %.2f};",
                    cameraIndex, result.calibratedX, result.calibratedY));
                telemetry.log().add(String.format("LIMELIGHT_ORIENTATION_OFFSETS[%d] = %.4f;  // %.1f°",
                    cameraIndex, result.calibratedHeading, Math.toDegrees(result.calibratedHeading)));
            } else {
                telemetry.log().add("");
                telemetry.log().add("CALIBRATION FAILED!");
                telemetry.log().add("Check that AprilTag is visible");
            }
        }

        // Verify accuracy
        if (gamepad1.b) {
            telemetry.log().add("Verifying accuracy...");
            telemetry.update();

            boolean passed = calibrator.verifyAccuracy(
                expectedX,
                expectedY,
                expectedHeading,
                positionTolerance,
                headingTolerance
            );

            if (passed) {
                telemetry.log().add("✓ Accuracy verified within tolerance");
            } else {
                telemetry.log().add("✗ Accuracy outside tolerance");
            }
        }

        // Scan for tags
        if (gamepad1.x) {
            calibrator.scanForTags();
        }
    }

    /**
     * Updates status telemetry.
     */
    private void updateStatus() {
        telemetry.addData("Camera", robot.limelightCameras[cameraIndex].getName());
        telemetry.addData("Expected Position",
            String.format("(%.1f, %.1f) inches, %.1f°",
                expectedX, expectedY, Math.toDegrees(expectedHeading)));
        telemetry.addData("Camera Index", "%d / %d",
            cameraIndex + 1, robot.limelightCameras.length);
        telemetry.update();
    }

    /**
     * Telemetry adapter for LimelightCalibrator.
     */
    private class TelemetryAdapter implements LimelightCalibrator.Telemetry {
        @Override
        public Log log() {
            return new Log() {
                @Override
                public void add(String message) {
                    telemetry.log().add(message);
                }
            };
        }
    }
}
