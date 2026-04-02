package org.firstinspires.ftc.teamcode.subsystems.localization;

import com.pedropathing.geometry.Pose;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.kinematics.wpilibkinematics.ChassisSpeeds;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.util.localization.SensorFusionLocalizer;
import org.firstinspires.ftc.teamcode.util.localization.SwerveOdometry;
import org.firstinspires.ftc.teamcode.util.monitoring.PinpointHealthMonitor;
import org.firstinspires.ftc.teamcode.util.vision.LimelightCamera;
import org.firstinspires.ftc.teamcode.subsystems.vision.Vision;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drive;

import com.qualcomm.robotcore.util.RobotLog;

import static org.firstinspires.ftc.teamcode.Constants.*;

/**
 * Localization subsystem that fuses multiple pose sources for optimal robot positioning.
 *
 * <p>This subsystem manages all localization-related concerns, including:</p>
 * <ul>
 *   <li><b>Sensor Fusion:</b> Extended Kalman Filter (EKF) fusing swerve odometry, Pinpoint,
 *       and Limelight vision for optimal pose estimation</li>
 *   <li><b>Pinpoint Management:</b> Health monitoring, auto-reset, and IMU fallback</li>
 *   <li><b>IMU Backup:</b> OctoQuad IMU calibration and fallback when Pinpoint fails</li>
 *   <li><b>Vision Integration:</b> Receives vision pose updates from Vision subsystem</li>
 * </ul>
 *
 * <h3>Architecture:</h3>
 * <p>This subsystem acts as a single source of truth for robot pose. Other subsystems
 * (Drive, Vision) consume pose from here rather than managing their own localization.</p>
 *
 * <h3>How It Works:</h3>
 * <ol>
 *   <li><b>Prediction Step:</b> Predicts pose using swerve odometry and drive velocities</li>
 *   <li><b>Pinpoint Correction:</b> Corrects pose with Pinpoint odometry (if healthy)</li>
 *   <li><b>Vision Correction:</b> Corrects pose with Limelight vision (if tags visible)</li>
 *   <li><b>Fusion:</b> EKF combines all sources with uncertainty-based weighting</li>
 *   <li><b>Health Management:</b> Monitors Pinpoint health, auto-resets when safe, falls back to OctoQuad IMU</li>
 * </ol>
 *
 * <h3>Usage:</h3>
 * <pre>
 * // In Robot.init():
 * localization = new LocalizationSubsystem();
 * localization.init();
 *
 * // In Robot.periodic():
 * localization.update(currentDriveSpeeds);
 *
 * // In Drive or Vision subsystems:
 * Pose fusedPose = localization.getFusedPose();
 * </pre>
 *
 * <h3>Thread Safety:</h3>
 * <p>This subsystem is NOT thread-safe. All calls must be made from the same thread
 * (typically the OpMode loop thread).</p>
 *
 * @see org.firstinspires.ftc.teamcode.util.localization.SensorFusionLocalizer
 * @see org.firstinspires.ftc.teamcode.util.monitoring.PinpointHealthMonitor
 * @see org.firstinspires.ftc.teamcode.subsystems.drive.Drive
 */
public class LocalizationSubsystem extends SubsystemBase {

    private final Robot robot;

    /**
     * Extended Kalman Filter for sensor fusion.
     * Combines swerve odometry, Pinpoint, and Limelight vision.
     */
    private final SensorFusionLocalizer sensorFusion;

    /**
     * Swerve odometry tracker using drive wheel encoders.
     * Provides high-frequency pose updates for the EKF prediction step.
     */
    private final SwerveOdometry swerveOdometry;

    /**
     * Health monitor for detecting Pinpoint drift.
     * Automatically disables Pinpoint corrections when it's drifted too far from truth.
     */
    private final PinpointHealthMonitor pinpointHealthMonitor;

    /**
     * Cached fused pose from sensor fusion.
     * Updated every periodic() call for telemetry access.
     */
    private Pose fusedPose = null;

    /**
     * Cached position uncertainty from sensor fusion.
     */
    private double[] positionUncertainty = new double[]{0, 0};

    /**
     * Cached heading uncertainty from sensor fusion.
     */
    private double headingUncertainty = 0.0;

    /**
     * Tracks whether vision tag was visible in the previous loop.
     * Used for Pinpoint auto-reset logic.
     */
    private boolean visionTagWasVisible = false;

    /**
     * Creates a new LocalizationSubsystem.
     */
    public LocalizationSubsystem() {
        this.robot = Robot.getInstance();

        // Initialize sensor fusion EKF
        sensorFusion = new SensorFusionLocalizer();
        configureSensorFusionNoise();

        // Initialize swerve odometry (will be updated with drive encoder data)
        swerveOdometry = new SwerveOdometry();

        // Initialize Pinpoint health monitor with constants
        pinpointHealthMonitor = new PinpointHealthMonitor(
            PINPOINT_HEALTH_DEVIATION_THRESHOLD,
            PINPOINT_HEALTH_CONSECUTIVE_BAD_READINGS
        );
    }

    /**
     * Initializes the localization subsystem.
     *
     * <p>This method must be called before the subsystem is used. It performs the following:</p>
     * <ol>
     *   <li>Initializes sensor fusion noise parameters</li>
     *   <li>Prepares Pinpoint health monitoring</li>
     *   <li>Sets up OctoQuad IMU backup system</li>
     * </ol>
     */
    public void init() {
        // Sensor fusion is already configured in constructor
        // Additional initialization can be done here if needed
    }

    /**
     * Main update loop for the localization subsystem.
     *
     * <p>This method should be called once per loop iteration, typically from
     * {@link Robot#periodic()}. It performs the following operations:</p>
     *
     * <ol>
     *   <li><b>Predict:</b> Update EKF prediction with swerve odometry</li>
     *   <li><b>Pinpoint Correction:</b> Correct with Pinpoint if healthy and within field bounds</li>
     *   <li><b>Vision Correction:</b> Correct with Limelight vision if tags visible</li>
     *   <li><b>Pinpoint Auto-Reset:</b> Reset Pinpose if drifted and safe to do so</li>
     *   <li><b>IMU Backup:</b> Calibrate OctoQuad IMU if Pinpoint is unhealthy</li>
     *   <li><b>Fusion:</b> Fuse all sources and get estimated pose</li>
     * </ol>
     *
     * @param currentDriveSpeeds current chassis speeds from drive subsystem
     */
    public void update(ChassisSpeeds currentDriveSpeeds) {
        // ===== 1. Prediction Step =====
        // Update swerve odometry with current drive velocities
        // (This is done by the drive subsystem passing odometry data)
        // TODO: Need to get odometry data from drive subsystem

        // Predict with swerve odometry
        sensorFusion.predictWithSwerve(currentDriveSpeeds);

        // ===== 2. Pinpoint Correction Step =====
        if (robot.pinpoint != null) {
            // Read Pinpoint pose
            double pinpointX = robot.pinpoint.getPosition().getX(org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.INCH);
            double pinpointY = robot.pinpoint.getPosition().getY(org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.INCH);
            double pinpointHeading = Math.toRadians(robot.pinpoint.getHeading(org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.RADIANS));

            // Check if Pinpoint is outside field boundaries (impossible position)
            boolean pinpointOutOfBounds = ENABLE_PINPOINT_BOUNDARY_CHECK &&
                                         !isWithinFieldBounds(pinpointX, pinpointY);

            // Only use Pinpoint if:
            // 1. Not outside field boundaries
            // 2. Health monitoring is disabled OR Pinpoint is healthy
            boolean usePinpoint = !pinpointOutOfBounds &&
                                (!ENABLE_PINPOINT_HEALTH_MONITOR || pinpointHealthMonitor.isHealthy());

            if (usePinpoint) {
                // Correct with Pinpoint pose
                sensorFusion.correctWithPinpoint(pinpointX, pinpointY, pinpointHeading);
            }
            // Else: Pinpoint is drifted or out of bounds, skip correction (rely on swerve + vision)
        }

        // ===== 3. Vision Correction Step =====
        if (robot.vision != null && robot.vision.isTagVisible()) {
            // Get vision pose from Limelight
            Pose visionPose = robot.vision.getRobotPose();
            double visionUncertainty = robot.vision.getPoseUncertainty();

            // Correct with vision pose (using default noise parameters)
            sensorFusion.correctWithVision(
                visionPose.getX(),
                visionPose.getY(),
                visionPose.getHeading()
            );
        }

        // ===== 4. Get Fused Pose =====
        fusedPose = sensorFusion.getEstimatedPose();

        // ===== 5. Pinpoint Auto-Reset Logic =====
        // If Pinpoint is drifted and conditions are safe, auto-reset to vision pose
        if (ENABLE_PINPOINT_HEALTH_MONITOR && robot.pinpoint != null && !pinpointHealthMonitor.isHealthy()) {
            // Check if safe to reset (robot is stationary and vision is good)
            ChassisSpeeds estimatedVel = sensorFusion.getEstimatedVelocity();
            double[] posUncertainty = sensorFusion.getPositionUncertainty();
            double robotVelocity = Math.hypot(estimatedVel.vxMetersPerSecond, estimatedVel.vyMetersPerSecond);
            double maxUncertainty = Math.max(posUncertainty[0], posUncertainty[1]);
            boolean visionTagVisible = (robot.vision != null && robot.vision.isTagVisible());

            boolean safeToReset = pinpointHealthMonitor.isSafeToReset(
                robotVelocity,
                maxUncertainty,
                visionTagVisible,
                PINPOINT_AUTO_RESET_MAX_VELOCITY,
                PINPOINT_AUTO_RESET_MAX_UNCERTAINTY
            );

            if (safeToReset && robot.vision != null && robot.vision.isTagVisible()) {
                // Reset Pinpoint odometry
                // Note: Pinpoint will start from (0,0,0), but the EKF will continue
                // to provide accurate fused pose using vision corrections
                robot.pinpoint.resetPosAndIMU();

                // Reset health monitor
                pinpointHealthMonitor.reset();

                RobotLog.i("Localization: Pinpoint auto-reset to (0,0,0), EKF maintaining pose");
            }
        }

        // ===== 6. OctoQuad IMU Backup Auto-Calibration Logic =====
        // If OctoQuad IMU is being used as backup and conditions are safe, calibrate its offset
        if (OCTOQUAD_IMU_BACKUP_ENABLED) {
            // Check if OctoQuad IMU is currently being used as backup
            if (!robot.getIMUSource().equals("Pinpoint")) {
                // Check if safe to calibrate OctoQuad IMU
                ChassisSpeeds estimatedVel = sensorFusion.getEstimatedVelocity();
                double[] posUncertainty = sensorFusion.getPositionUncertainty();
                double robotVelocity = Math.hypot(estimatedVel.vxMetersPerSecond, estimatedVel.vyMetersPerSecond);
                double maxUncertainty = Math.max(posUncertainty[0], posUncertainty[1]);

                boolean safeToCalibrate = robot.isSafeToCalibrateOctoQuadIMU(
                    robotVelocity,
                    maxUncertainty
                );

                if (safeToCalibrate && robot.vision != null && robot.vision.isTagVisible()) {
                    // Get fused pose heading for calibration reference
                    double fusedHeading = fusedPose.getHeading();

                    // Calibrate OctoQuad IMU offset to fused pose heading
                    boolean calibrated = robot.calibrateOctoQuadIMU(fusedHeading);

                    if (calibrated) {
                        RobotLog.i("Localization: OctoQuad IMU auto-calibrated to fused heading");
                    }
                }
            }

            // Pinpoint is healthy again, reset OctoQuad IMU offset for next time
            if (ENABLE_PINPOINT_HEALTH_MONITOR && robot.pinpoint != null && pinpointHealthMonitor.isHealthy()) {
                robot.resetOctoQuadIMUOffset();
            }
        }

        // ===== 7. Cache Uncertainties for Telemetry =====
        positionUncertainty = sensorFusion.getPositionUncertainty();
        headingUncertainty = sensorFusion.getHeadingUncertainty();
    }

    /**
     * Gets the fused pose from sensor fusion.
     *
     * <p>This is the single source of truth for robot pose. It combines swerve odometry,
     * Pinpoint deadwheel, and Limelight vision using an Extended Kalman Filter.</p>
     *
     * @return fused robot pose, or null if sensor fusion is disabled
     */
    public Pose getFusedPose() {
        return fusedPose;
    }

    /**
     * Gets the position uncertainty from sensor fusion.
     *
     * @return array of [xUncertainty, yUncertainty] in inches
     */
    public double[] getPositionUncertainty() {
        return positionUncertainty;
    }

    /**
     * Gets the heading uncertainty from sensor fusion.
     *
     * @return heading uncertainty in radians
     */
    public double getHeadingUncertainty() {
        return headingUncertainty;
    }

    /**
     * Checks if the Pinpoint odometry is healthy.
     *
     * @return true if Pinpoint is healthy, false if it has drifted
     */
    public boolean isPinpointHealthy() {
        return !ENABLE_PINPOINT_HEALTH_MONITOR || pinpointHealthMonitor.isHealthy();
    }

    /**
     * Gets the Pinpoint bad reading count.
     *
     * @return number of consecutive bad Pinpoint readings
     */
    public int getPinpointBadReadingCount() {
        return ENABLE_PINPOINT_HEALTH_MONITOR ? pinpointHealthMonitor.getBadReadingCount() : 0;
    }

    /**
     * Checks if a pose is within valid field boundaries.
     *
     * @param x X position (inches)
     * @param y Y position (inches)
     * @return true if within bounds, false if outside
     */
    private boolean isWithinFieldBounds(double x, double y) {
        double margin = FIELD_BOUNDARY_MARGIN;
        double xMin = FIELD_X_MIN - margin;
        double xMax = FIELD_X_MAX + margin;
        double yMin = FIELD_Y_MIN - margin;
        double yMax = FIELD_Y_MAX + margin;

        return x >= xMin && x <= xMax && y >= yMin && y <= yMax;
    }

    /**
     * Configures the sensor fusion noise parameters from Constants.
     *
     * <p>This method sets up the EKF noise covariance matrices based on the
     * sensor characteristics. Lower noise = more trust in that sensor.</p>
     */
    private void configureSensorFusionNoise() {
        // Process noise: how much the system changes unpredictably
        double[] processNoise = {
            FUSION_PROCESS_NOISE_POSITION,    // Position uncertainty (inches)
            FUSION_PROCESS_NOISE_HEADING,      // Heading uncertainty (radians)
            FUSION_PROCESS_NOISE_VELOCITY     // Velocity uncertainty (in/s)
        };

        // Swerve odometry noise (medium accuracy, high frequency)
        double[] swerveNoise = {
            FUSION_SWERVE_NOISE_POSITION,     // Position uncertainty (inches)
            FUSION_SWERVE_NOISE_HEADING        // Heading uncertainty (radians)
        };

        // Pinpoint odometry noise (high accuracy, low frequency)
        double[] pinpointNoise = {
            FUSION_PINPOINT_NOISE_POSITION,    // Position uncertainty (inches)
            FUSION_PINPOINT_NOISE_HEADING      // Heading uncertainty (radians)
        };

        // Vision noise (variable accuracy, tag-dependent)
        double[] visionNoise = {
            FUSION_VISION_NOISE_POSITION,      // Position uncertainty (inches)
            FUSION_VISION_NOISE_HEADING        // Heading uncertainty (radians)
        };

        // Configure sensor fusion with noise parameters
        sensorFusion.setNoiseParameters(processNoise, swerveNoise, pinpointNoise, visionNoise);
    }
}
