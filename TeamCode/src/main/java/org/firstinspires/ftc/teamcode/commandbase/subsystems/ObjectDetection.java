package org.firstinspires.ftc.teamcode.commandbase.subsystems;

import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.globals.Robot;

/**
 * Object Detection Subsystem using Limelight3A.
 *
 * <p>This subsystem provides easy access to Limelight's color detection
 * capabilities for game piece detection and alignment.</p>
 *
 * <h3>Features:</h3>
 * <ul>
 *   <li>Detect colored game pieces (HSV filtering)</li>
 *   <li>Auto-alignment to targets</li>
 *   <li>Distance estimation</li>
 *   <li>Multiple pipeline support</li>
 * </ul>
 *
 * <h3>Usage:</h3>
 * <pre>{@code
 * // In Robot.java
 * public ObjectDetection objectDetection;
 *
 * public void init(HardwareMap hwMap) {
 *     objectDetection = new ObjectDetection();
 *     objectDetection.setPipeline(ObjectDetection.Pipeline.COLOR);
 * }
 *
 * // In TeleOp
 * if (objectDetection.hasTarget()) {
 *     double adjustment = objectDetection.getAlignmentAdjustment();
 *     drive.strafe(adjustment);
 * }
 * }</pre>
 *
 * @see org.firstinspires.ftc.teamcode.globals.Robot
 */
public class ObjectDetection extends SubsystemBase {

    /**
     * Detection pipeline types.
     */
    public enum Pipeline {
        /** AprilTag detection (pose estimation) */
        APRILTAG(0),
        /** Color detection (game pieces) */
        COLOR(1),
        /** Custom pipeline 2 */
        CUSTOM_2(2),
        /** Custom pipeline 3 */
        CUSTOM_3(3);

        final int index;
        Pipeline(int index) {
            this.index = index;
        }
    }

    private final Robot robot;
    private Pipeline currentPipeline = Pipeline.COLOR;
    private boolean lastDetected = false;
    private double lastTx = 0;
    private double lastTy = 0;
    private double lastTa = 0;

    // Alignment parameters
    private static final double ALIGNMENT_GAIN = 0.02;
    private static final double ALIGNMENT_TOLERANCE = 1.0;  // degrees
    private static final double MAX_STRAFE = 0.3;

    // Distance calibration (adjust for your setup!)
    private static final double CAMERA_HEIGHT = 8.0;      // inches
    private static final double CAMERA_ANGLE = 20.0;       // degrees
    private static final double TARGET_HEIGHT = 2.0;       // inches

    /**
     * Creates the object detection subsystem.
     */
    public ObjectDetection() {
        this.robot = Robot.getInstance();
    }

    /**
     * Sets the detection pipeline.
     *
     * @param pipeline Pipeline to use
     */
    public void setPipeline(Pipeline pipeline) {
        if (currentPipeline != pipeline) {
            robot.limelight.pipelineSwitch(pipeline.index);
            currentPipeline = pipeline;

            // Small delay for pipeline to switch
            try {
                Thread.sleep(50);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }
    }

    /**
     * Gets the current pipeline.
     */
    public Pipeline getPipeline() {
        return currentPipeline;
    }

    /**
     * Periodic update method called every loop iteration.
     *
     * <p>Updates detection data from Limelight.</p>
     */
    @Override
    public void periodic() {
        com.qualcomm.hardware.limelightvision.LLResult result = robot.limelight.getLatestResult();

        if (result != null) {
            lastDetected = result.isValid();
            lastTx = result.getTx();
            lastTy = result.getTy();
            lastTa = result.getTa();
        } else {
            lastDetected = false;
        }
    }

    /**
     * Checks if a target is currently detected.
     */
    public boolean hasTarget() {
        return lastDetected;
    }

    /**
     * Gets horizontal offset to target.
     *
     * @return Degrees (negative = left of center, positive = right of center)
     */
    public double getHorizontalOffset() {
        return lastTx;
    }

    /**
     * Gets vertical offset to target.
     *
     * @return Degrees (negative = above center, positive = below center)
     */
    public double getVerticalOffset() {
        return lastTy;
    }

    /**
     * Gets target area (0-100%).
     *
     * <p>Used for rough distance estimation.</p>
     *
     * @return Target area percentage
     */
    public double getTargetArea() {
        return lastTa;
    }

    /**
     * Estimates distance to target.
     *
     * <p><b>Note:</b> This requires calibration for your camera mounting position!</p>
     *
     * @return Estimated distance in inches
     */
    public double getEstimatedDistance() {
        if (!lastDetected) return -1;

        double tyRadians = Math.toRadians(lastTy + CAMERA_ANGLE);
        double distance = (TARGET_HEIGHT - CAMERA_HEIGHT) / Math.tan(tyRadians);

        return distance;
    }

    /**
     * Checks if the robot is aligned with the target.
     *
     * @return true if horizontal offset is within tolerance
     */
    public boolean isAligned() {
        return Math.abs(lastTx) < ALIGNMENT_TOLERANCE;
    }

    /**
     * Gets strafe adjustment to center on target.
     *
     * <p>Positive = strafe right, Negative = strafe left</p>
     *
     * @return Adjustment value (-1 to 1)
     */
    public double getAlignmentAdjustment() {
        if (!lastDetected) return 0;

        double adjustment = -lastTx * ALIGNMENT_GAIN;
        return Math.max(-MAX_STRAFE, Math.min(MAX_STRAFE, adjustment));
    }

    /**
     * Gets forward speed to approach target.
     *
     * <p>Slows down as it gets closer based on target area.</p>
     *
     * @param maxSpeed Maximum forward speed
     * @return Forward speed (0 to maxSpeed)
     */
    public double getApproachSpeed(double maxSpeed) {
        if (!lastDetected) return 0;

        // Target area: 0% (far) to 20% (close)
        // Slow down when area is large
        if (lastTa > 15.0) {
            return 0;  // Too close, stop!
        } else if (lastTa > 10.0) {
            return maxSpeed * 0.3;  // Getting close, slow down
        } else if (lastTa > 5.0) {
            return maxSpeed * 0.6;  // Medium distance
        } else {
            return maxSpeed;  // Far away, full speed
        }
    }

    /**
     * Checks if target is at optimal scoring distance.
     *
     * @param minArea Minimum target area (%)
     * @param maxArea Maximum target area (%)
     * @return true if target area is in range
     */
    public boolean isAtDistance(double minArea, double maxArea) {
        return lastDetected && lastTa >= minArea && lastTa <= maxArea;
    }

    /**
     * Gets the target direction for driver feedback.
     *
     * @return Direction string
     */
    public String getDirectionString() {
        if (!lastDetected) return "NO TARGET";

        if (Math.abs(lastTx) < ALIGNMENT_TOLERANCE) {
            return "✓ ALIGNED";
        } else if (lastTx > 0) {
            return "→ TURN LEFT";
        } else {
            return "← TURN RIGHT";
        }
    }

    /**
     * Gets human-readable status.
     */
    public String getStatusString() {
        if (!lastDetected) {
            return "SEARCHING...";
        }

        return String.format("DETECTED - %.1fin, %s", getEstimatedDistance(), getDirectionString());
    }
}
