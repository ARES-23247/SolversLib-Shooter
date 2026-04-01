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
        robot = Robot.getInstance();
robot.init(hardwareMap);

        // Initialize calibrator with first camera (or selected camera)
        if (robot.limelightCameras != null && robot.limelightCameras.length > 0) {
            calibrator = new LimelightCalibrator(
                robot.limelightCameras[cameraIndex].getLimelight(),
                new TelemetryAdapter()
            );
        } else {
            System.out.println("ERROR: No Limelight cameras found!");
            System.out.println("Check Robot.java initialization");
            telemetry.update();
            return;
        }

        // Show calibration instructions
        showInstructions();

        waitForStart();

        System.out.println("Calibration OpMode started");
        telemetry.update();

        while (opModeIsActive()) {
            // Handle calibration controls
            handleCalibrationControls();

            // Update telemetry
            updateStatus();

            sleep(50);  // 20Hz update rate
        }

        System.out.println("Calibration OpMode ended");
        telemetry.update();
    }

    /**
     * Displays initial calibration instructions.
     */
    private void showInstructions() {
        System.out.println("===========================================");
        System.out.println("   LIMELIGHT CALIBRATION");
        System.out.println("===========================================");
        System.out.println("");
        System.out.println("SETUP:");
        System.out.println("1. Place robot at known position:");
        System.out.println("   - " + String.format("%.1f", expectedX) + " inches from tag (X)");
        System.out.println("   - " + String.format("%.1f", expectedY) + " inches lateral (Y)");
        System.out.println("   - " + String.format("%.1f°", Math.toDegrees(expectedHeading)) + " heading");
        System.out.println("");
        System.out.println("2. Robot must be stationary");
        System.out.println("");
        System.out.println("CONTROLS:");
        System.out.println("A: Run calibration");
        System.out.println("B: Verify accuracy");
        System.out.println("X: Scan for visible tags");
        System.out.println("Y: Adjust expected X (+/- with D-pad up/down)");
        System.out.println("Left Bumper: Previous camera");
        System.out.println("Right Bumper: Next camera");
        System.out.println("");
        System.out.println("===========================================");
        telemetry.update();
    }

    /**
     * Handles gamepad controls for calibration.
     */
    private void handleCalibrationControls() {
        // Adjust expected X distance
        if (gamepad1.dpad_up) {
            expectedX += 1.0;
            System.out.println(String.format("Expected X: %.1f inches", expectedX));
        } else if (gamepad1.dpad_down) {
            expectedX = Math.max(6.0, expectedX - 1.0);  // Minimum 6 inches
            System.out.println(String.format("Expected X: %.1f inches", expectedX));
        }

        // Adjust expected Y position
        if (gamepad1.dpad_left) {
            expectedY -= 1.0;
            System.out.println(String.format("Expected Y: %.1f inches", expectedY));
        } else if (gamepad1.dpad_right) {
            expectedY += 1.0;
            System.out.println(String.format("Expected Y: %.1f inches", expectedY));
        }

        // Select camera (for multi-camera setups)
        if (gamepad1.left_bumper) {
            cameraIndex = Math.max(0, cameraIndex - 1);
            if (robot.limelightCameras.length > cameraIndex) {
                calibrator = new LimelightCalibrator(
                    robot.limelightCameras[cameraIndex].getLimelight(),
                    new TelemetryAdapter()
                );
                System.out.println("Selected camera: " + robot.limelightCameras[cameraIndex].getName());
            }
        } else if (gamepad1.right_bumper) {
            cameraIndex = Math.min(robot.limelightCameras.length - 1, cameraIndex + 1);
            if (robot.limelightCameras.length > cameraIndex) {
                calibrator = new LimelightCalibrator(
                    robot.limelightCameras[cameraIndex].getLimelight(),
                    new TelemetryAdapter()
                );
                System.out.println("Selected camera: " + robot.limelightCameras[cameraIndex].getName());
            }
        }

        // Run calibration
        if (gamepad1.a && !gamepad1.touchpad) {  // Prevent repeated triggers
            System.out.println("Running calibration...");
            telemetry.update();

            LimelightCalibrator.CalibrationResult result = calibrator.calibrateMountPosition(
                expectedX,
                expectedY,
                expectedHeading
            );

            if (result.success) {
                System.out.println("");
                System.out.println("CALIBRATION SUCCESSFUL!");
                System.out.println("");
                System.out.println("Update Constants.java:");
                System.out.println(String.format("LIMELIGHT_MOUNT_POSITIONS[%d] = {%.2f, %.2f};",
                    cameraIndex, result.calibratedX, result.calibratedY));
                System.out.println(String.format("LIMELIGHT_ORIENTATION_OFFSETS[%d] = %.4f;  // %.1f°",
                    cameraIndex, result.calibratedHeading, Math.toDegrees(result.calibratedHeading)));
            } else {
                System.out.println("");
                System.out.println("CALIBRATION FAILED!");
                System.out.println("Check that AprilTag is visible");
            }
        }

        // Verify accuracy
        if (gamepad1.b) {
            System.out.println("Verifying accuracy...");
            telemetry.update();

            boolean passed = calibrator.verifyAccuracy(
                expectedX,
                expectedY,
                expectedHeading,
                positionTolerance,
                headingTolerance
            );

            if (passed) {
                System.out.println("✓ Accuracy verified within tolerance");
            } else {
                System.out.println("✗ Accuracy outside tolerance");
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
                    System.out.println(message);
                }
            };
        }
    }
}
