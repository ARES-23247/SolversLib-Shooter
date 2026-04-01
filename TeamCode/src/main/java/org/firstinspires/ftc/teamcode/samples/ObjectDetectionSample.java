package org.firstinspires.ftc.teamcode.samples;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.util.TelemetryData;

import org.firstinspires.ftc.teamcode.globals.Robot;

/**
 * Limelight3A Object Detection Sample OpMode.
 *
 * <p>This OpMode demonstrates:</p>
 * <ul>
 *   <li>Detecting colored game pieces (yellow pixels, etc.)</li>
 *   <li>Auto-alignment to detected objects</li>
 *   <li>Distance estimation</li>
 *   <li>Multiple pipeline switching</li>
 * </ul>
 *
 * <h3>Setup:</h3>
 * <ol>
 *   <li>Configure Limelight pipeline 1 for color detection (HSV tuning)</li>
 *   <li>Point camera at game piece</li>
 *   <li>Adjust HSV sliders in web interface until game piece is highlighted</li>
 * </ol>
 *
 * <h3>Controls:</h3>
 * <ul>
 *   <li>Gamepad A: Toggle auto-align</li>
 *   <li>Gamepad B: Switch to AprilTag pipeline</li>
 *   <li>Gamepad X: Switch to color detection pipeline</li>
 *   <li>Gamepad Y: Print calibration data</li>
 * </ul>
 *
 * <h3>Calibration:</h3>
 * <p>To calibrate distance estimation:</p>
 * <ol>
 *   <li>Place game piece at known distances (12", 24", 36", 48")</li>
 *   <li>Press Y to log ty values</li>
 *   <li>Plot distance vs. ty in Excel/Google Sheets</li>
 *   <li>Fit formula: distance = A / tan(ty + B)</li>
 * </ol>
 */
@TeleOp(name = "Limelight Detection Test", group = "Test")
public class ObjectDetectionSample extends LinearOpMode {

    private Robot robot;
    private boolean autoAlign = false;
    private int currentPipeline = 1;  // 1 = color detection

    // Camera calibration constants (adjust these!)
    private static final double CAMERA_HEIGHT = 8.0;      // inches from floor
    private static final double CAMERA_ANGLE = 20.0;       // degrees (downward)
    private static final double TARGET_HEIGHT = 2.0;       // inches (game piece)
    private static final double ALIGNMENT_GAIN = 0.02;    // P-controller gain
    private static final double MAX_STRAFE = 0.3;         // max strafe speed

    @Override
    public void runOpMode() throws InterruptedException {
        TelemetryData logger = new TelemetryData(super.telemetry);

        logger.addData("Status", "Initializing Limelight Detection...");
        logger.update();

        robot = Robot.getInstance();
        robot.init(hardwareMap);
        robot.telemetry = logger;

        // Set Limelight to color detection pipeline (pipeline 1)
        robot.limelight.pipelineSwitch(1);
        robot.limelight.start();
        currentPipeline = 1;

        logger.addData("Status", "Ready!");
        logger.addData("Pipeline", "Color Detection");
        logger.addData("", "");
        logger.addData("Instructions", "Point camera at game piece");
        logger.addData("Controls", "A: Align | B: AprilTag | X: Color | Y: Calibrate");
        logger.update();

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {
            // Handle gamepad input
            if (gamepad1.a) {
                autoAlign = !autoAlign;
                Thread.sleep(200);  // Debounce
            }

            if (gamepad1.b && currentPipeline != 0) {
                robot.limelight.pipelineSwitch(0);
                currentPipeline = 0;
                Thread.sleep(200);
            }

            if (gamepad1.x && currentPipeline != 1) {
                robot.limelight.pipelineSwitch(1);
                currentPipeline = 1;
                Thread.sleep(200);
            }

            if (gamepad1.y) {
                printCalibrationData();
                Thread.sleep(500);
            }

            // Read detection data
            boolean detected = isTargetDetected();
            double tx = getHorizontalOffset();
            double ty = getVerticalOffset();
            double ta = getTargetArea();

            // Auto-align mode
            if (autoAlign && detected) {
                // Auto-strafe to center on target
                double strafe = -tx * ALIGNMENT_GAIN;
                strafe = Math.max(-MAX_STRAFE, Math.min(MAX_STRAFE, strafe));

                robot.drive.setTeleOpDrive(0, strafe, 0, true);
                logger.addData("Mode", "AUTO-ALIGN");
            } else {
                // Manual drive
                double forward = -gamepad1.left_stick_y;
                double lateral = gamepad1.left_stick_x;
                double turn = gamepad1.right_stick_x;

                robot.drive.setTeleOpDrive(forward, lateral, turn, true);
                logger.addData("Mode", autoAlign ? "SEARCHING" : "MANUAL");
            }

            // Telemetry
            logger.addData("Pipeline", currentPipeline == 0 ? "AprilTag" : "Color");
            logger.addData("Detected", detected ? "YES" : "NO");

            if (detected) {
                logger.addData("Horizontal Offset", String.format("%.2f°", tx));
                logger.addData("Vertical Offset", String.format("%.2f°", ty));
                logger.addData("Target Area", String.format("%.2f%%", ta));
                logger.addData("Estimated Distance", String.format("%.1f in", estimateDistance()));

                // Alignment status
                if (Math.abs(tx) < 1.0) {
                    logger.addData("Alignment", "✓ ALIGNED");
                } else {
                    String direction = tx > 0 ? "← LEFT" : "→ RIGHT";
                    logger.addData("Alignment", direction);
                }

                // Distance guidance
                if (ta > 15.0) {
                    logger.addData("Action", "BACK UP - TOO CLOSE");
                } else if (ta < 2.0) {
                    logger.addData("Action", "MOVE FORWARD - TOO FAR");
                } else {
                    logger.addData("Action", "GOOD DISTANCE");
                }
            }

            logger.addData("","");
            logger.addData("Controls", "A: Align | B: AprilTag | X: Color | Y: Calibrate");

            logger.update();

            robot.updateLoop(logger);
        }

        // Cleanup
        robot.limelight.pipelineSwitch(0);  // Back to AprilTag
    }

    /**
     * Checks if a target is detected.
     */
    private boolean isTargetDetected() {
        com.qualcomm.hardware.limelightvision.LLResult result = robot.limelight.getLatestResult();
        return result != null && result.isValid();
    }

    /**
     * Gets horizontal offset to target.
     * @return Degrees (negative = left of center, positive = right of center)
     */
    private double getHorizontalOffset() {
        com.qualcomm.hardware.limelightvision.LLResult result = robot.limelight.getLatestResult();
        return result != null ? result.getTx() : 0.0;
    }

    /**
     * Gets vertical offset to target.
     * @return Degrees (negative = above center, positive = below center)
     */
    private double getVerticalOffset() {
        com.qualcomm.hardware.limelightvision.LLResult result = robot.limelight.getLatestResult();
        return result != null ? result.getTy() : 0.0;
    }

    /**
     * Gets target area (0-100%).
     * Larger values = closer target.
     */
    private double getTargetArea() {
        com.qualcomm.hardware.limelightvision.LLResult result = robot.limelight.getLatestResult();
        return result != null ? result.getTa() : 0.0;
    }

    /**
     * Estimates distance to target using trigonometry.
     *
     * Formula: distance = (targetHeight - cameraHeight) / tan(ty + cameraAngle)
     *
     * Note: This requires calibration for your camera mounting position!
     */
    private double estimateDistance() {
        double ty = getVerticalOffset();

        // Convert to radians
        double tyRadians = Math.toRadians(ty + CAMERA_ANGLE);

        // Calculate distance using trigonometry
        double distance = (TARGET_HEIGHT - CAMERA_HEIGHT) / Math.tan(tyRadians);

        return distance;
    }

    /**
     * Prints calibration data for distance estimation.
     *
     * Use this to calibrate your camera:
     * 1. Place game piece at known distance (e.g., 24 inches)
     * 2. Press Y to log the ty value
     * 3. Repeat for multiple distances
     * 4. Plot distance vs. ty in Excel
     * 5. Fit curve to find the formula
     */
    private void printCalibrationData() {
        boolean detected = isTargetDetected();

        System.out.println("=== Limelight Calibration Data ===");
        System.out.println("Detected: " + (detected ? "YES" : "NO"));

        if (detected) {
            System.out.println(String.format("tx (horizontal): %.2f°", getHorizontalOffset()));
            System.out.println(String.format("ty (vertical): %.2f°", getVerticalOffset()));
            System.out.println(String.format("ta (area): %.2f%%", getTargetArea()));
            System.out.println(String.format("Estimated distance: %.1f in", estimateDistance()));
            System.out.println("");
            System.out.println("Record this data at known distances!");
            System.out.println("Example: Place object at 24in, press Y");
            System.out.println("Then fit formula: distance = A / tan(ty + B)");
        }

        System.out.println("====================================");
    }
}
