package org.firstinspires.ftc.teamcode.dashboard;

import com.bylazar.configurables.PanelsConfigurables;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.util.vision.LimelightCamera;

import java.util.ArrayList;
import java.util.List;

/**
 * Panels Dashboard with Capture & Replay for ARES Robot.
 *
 * <p>This class provides comprehensive telemetry visualization with Panels Capture & Replay support:</p>
 * <ul>
 *   <li><b>Field Overlay:</b> Robot position, heading, module orientations, trajectory history</li>
 *   <li><b>Live Graphs:</b> Velocities, currents, loop time, PID errors</li>
 *   <li><b>Capture & Replay:</b> Full recording for post-match analysis with timeline scrubbing</li>
 *   <li><b>Multi-Tab Display:</b> Organized data for easy viewing</li>
 * </ul>
 *
 * <h3>Capture & Replay Workflow:</h3>
 * <ol>
 *   <li><b>During Match:</b> Panels records all telemetry at high frequency (~50Hz)</li>
 *   <li><b>Post-Match:</b> Download capture files from Control Hub</li>
 *   <li><b>Replay Mode:</b> Scrub timeline, view synchronized graphs/telemetry</li>
 *   <li><b>Slow Motion:</b> Analyze swerve maneuvers at 0.25x speed</li>
 * </ol>
 *
 * <h3>Usage:</h3>
 * <pre>
 * // In Drive.periodic():
 * PanelsDashboard.getInstance().update();
 *
 * // In OpMode initialization:
 * PanelsDashboard.getInstance().initialize();
 * </pre>
 */
@Configurable
public class PanelsDashboard {

    private static PanelsDashboard instance;

    private final Robot robot;
    private final TelemetryManager telemetry;

    // ===== Robot State =====
    private Pose robotPose;
    private Pose fusedPose;
    private Pose swerveOdometry;
    private Pose pinpointPose;

    // ===== Vision State =====
    private boolean visionTagVisible;
    private int visionActiveCameras;
    private String activeCameraNames;

    // ===== Drive Inputs =====
    private double inputForward;
    private double inputLateral;
    private double inputTurn;
    private String inputMode;

    // ===== Module States =====
    private double[] moduleAngles = new double[4];      // Radians
    private double[] moduleTargetAngles = new double[4]; // Radians
    private double[] moduleVelocities = new double[4];   // In/s
    private double[] moduleTargetVelocities = new double[4]; // In/s

    // ===== Performance Metrics =====
    private double loopTime;
    private double loopFrequency;
    private double batteryVoltage;
    private double currentDraw;
    private boolean currentLimitingActive;
    private String encoderType;

    // ===== Sensor Fusion Data =====
    private double[] posUncertainty = new double[2];  // X, Y uncertainty (inches)
    private double headingUncertainty;                // Heading uncertainty (radians)

    // ===== IMU & Health Status =====
    private String imuSource;
    private double octoquadIMUOffset;
    private boolean pinpointHealthy;
    private int pinpointBadCount;

    // ===== Camera Status =====
    private String camera1Status;
    private String camera2Status;
    private double camera1Distance;
    private double camera2Distance;

    // ===== Trajectory History (for field overlay) =====
    private static final int TRAJECTORY_HISTORY_SIZE = 500;
    private final List<Pose> trajectoryHistory = new ArrayList<>();
    private final List<Double> trajectoryTimestamps = new ArrayList<>();

    private PanelsDashboard() {
        this.robot = Robot.getInstance();
        this.telemetry = PanelsTelemetry.INSTANCE.getTelemetry();
    }

    public static synchronized PanelsDashboard getInstance() {
        if (instance == null) {
            instance = new PanelsDashboard();
        }
        return instance;
    }

    /**
     * Initialize the dashboard for Capture & Replay.
     */
    public void initialize() {
        // Register with Panels configurables for capture/replay
        PanelsConfigurables.INSTANCE.refreshClass(this);
    }

    /**
     * Update all dashboard data for live viewing and capture.
     * Call this method once per loop iteration.
     */
    public void update() {
        double currentTime = System.currentTimeMillis() / 1000.0;

        // ===== 1. Robot Pose & Localization =====
        telemetry.addData("Robot X (in)", robotPose != null ? String.format("%.2f", robotPose.getX()) : "0.00");
        telemetry.addData("Robot Y (in)", robotPose != null ? String.format("%.2f", robotPose.getY()) : "0.00");
        telemetry.addData("Robot Heading (°)", robotPose != null ? String.format("%.1f", Math.toDegrees(robotPose.getHeading())) : "0.0");
        telemetry.addData("Robot Time (s)", String.format("%.3f", currentTime));

        // Fused pose (EKF output)
        if (fusedPose != null) {
            telemetry.addData("Fusion X (in)", String.format("%.2f", fusedPose.getX()));
            telemetry.addData("Fusion Y (in)", String.format("%.2f", fusedPose.getY()));
            telemetry.addData("Fusion Heading (°)", String.format("%.1f", Math.toDegrees(fusedPose.getHeading())));
        }

        // Localization sources (for replay debugging)
        if (swerveOdometry != null) {
            telemetry.addData("Swerve Odometry X", String.format("%.2f", swerveOdometry.getX()));
            telemetry.addData("Swerve Odometry Y", String.format("%.2f", swerveOdometry.getY()));
            telemetry.addData("Swerve Odometry Heading (°)", String.format("%.1f", Math.toDegrees(swerveOdometry.getHeading())));
        }

        if (pinpointPose != null) {
            telemetry.addData("Pinpoint X", String.format("%.2f", pinpointPose.getX()));
            telemetry.addData("Pinpoint Y", String.format("%.2f", pinpointPose.getY()));
            telemetry.addData("Pinpoint Heading (°)", String.format("%.1f", Math.toDegrees(pinpointPose.getHeading())));
        }

        // Trajectory history (for graphing in replay)
        if (robotPose != null) {
            trajectoryHistory.add(new Pose(robotPose.getX(), robotPose.getY(), robotPose.getHeading()));
            trajectoryTimestamps.add(currentTime);
            if (trajectoryHistory.size() > TRAJECTORY_HISTORY_SIZE) {
                trajectoryHistory.remove(0);
                trajectoryTimestamps.remove(0);
            }
        }

        telemetry.addData("Trajectory Points", trajectoryHistory.size());

        // Uncertainty (for EKF tuning during replay)
        telemetry.addData("Position Uncertainty X (in)", String.format("%.3f", posUncertainty[0]));
        telemetry.addData("Position Uncertainty Y (in)", String.format("%.3f", posUncertainty[1]));
        telemetry.addData("Heading Uncertainty (°)", String.format("%.3f", Math.toDegrees(headingUncertainty)));

        // ===== 2. Vision Status =====
        telemetry.addData("Vision Tag Visible", visionTagVisible ? "YES" : "NO");
        telemetry.addData("Vision Active Cameras", visionActiveCameras);
        telemetry.addData("Vision Active Names", activeCameraNames);

        // ===== 3. Drive Inputs =====
        telemetry.addData("Input Forward (m/s)", String.format("%.3f", inputForward));
        telemetry.addData("Input Lateral (m/s)", String.format("%.3f", inputLateral));
        telemetry.addData("Input Turn (rad/s)", String.format("%.3f", inputTurn));
        telemetry.addData("Input Mode", inputMode);

        // ===== 4. Module States (Detailed - for replay analysis) =====
        String[] moduleNames = {"FR", "FL", "BL", "BR"};
        for (int i = 0; i < 4; i++) {
            telemetry.addData(moduleNames[i] + " Angle (°)", String.format("%.1f", Math.toDegrees(moduleAngles[i])));
            telemetry.addData(moduleNames[i] + " Target Angle (°)", String.format("%.1f", Math.toDegrees(moduleTargetAngles[i])));
            telemetry.addData(moduleNames[i] + " Angle Error (°)", String.format("%.2f", Math.toDegrees(moduleTargetAngles[i] - moduleAngles[i])));
            telemetry.addData(moduleNames[i] + " Velocity (in/s)", String.format("%.2f", moduleVelocities[i]));
            telemetry.addData(moduleNames[i] + " Target Vel (in/s)", String.format("%.2f", moduleTargetVelocities[i]));
            telemetry.addData(moduleNames[i] + " Vel Error (in/s)", String.format("%.2f", moduleTargetVelocities[i] - moduleVelocities[i]));
        }

        // ===== 5. Performance Metrics (for replay profiling) =====
        telemetry.addData("Loop Time (ms)", String.format("%.2f", loopTime * 1000));
        telemetry.addData("Loop Frequency (Hz)", String.format("%.1f", loopFrequency));
        telemetry.addData("Battery Voltage (V)", String.format("%.2f", batteryVoltage));
        telemetry.addData("Current Draw (A)", String.format("%.2f", currentDraw));
        telemetry.addData("Current Limiting Active", currentLimitingActive ? "YES" : "NO");
        telemetry.addData("Encoder Type", encoderType);

        // ===== 6. IMU & Sensor Status =====
        telemetry.addData("IMU Source", imuSource);
        telemetry.addData("OctoQuad IMU Offset (°)", String.format("%.2f", Math.toDegrees(octoquadIMUOffset)));
        telemetry.addData("Pinpoint Healthy", pinpointHealthy ? "YES" : "DRIFTED");
        telemetry.addData("Pinpoint Bad Count", pinpointBadCount);

        // ===== 7. Camera Status =====
        telemetry.addData("Cam 1 Status", camera1Status);
        telemetry.addData("Cam 1 Distance (in)", String.format("%.1f", camera1Distance));

        if (robot.limelightCameras != null && robot.limelightCameras.length > 1) {
            telemetry.addData("Cam 2 Status", camera2Status);
            telemetry.addData("Cam 2 Distance (in)", String.format("%.1f", camera2Distance));
        }
    }

    // ===== Robot Pose & Localization Setters =====

    public void setRobotPose(Pose pose) {
        this.robotPose = pose;
    }

    public void setFusedPose(Pose pose) {
        this.fusedPose = pose;
    }

    public void setSwerveOdometry(Pose pose) {
        this.swerveOdometry = pose;
    }

    public void setPinpointPose(Pose pose) {
        this.pinpointPose = pose;
    }

    public void setLocalizationUncertainty(double[] posUncertainty, double headingUncertainty) {
        this.posUncertainty = posUncertainty;
        this.headingUncertainty = headingUncertainty;
    }

    // ===== Vision Setters =====

    public void setVisionStatus(boolean tagVisible, int activeCameras, String activeNames) {
        this.visionTagVisible = tagVisible;
        this.visionActiveCameras = activeCameras;
        this.activeCameraNames = activeNames;
    }

    // ===== Drive Input Setters =====

    public void setInputs(double forward, double lateral, double turn, String mode) {
        this.inputForward = forward;
        this.inputLateral = lateral;
        this.inputTurn = turn;
        this.inputMode = mode;
    }

    // ===== Module State Setters =====

    public void setModuleStates(double[] angles, double[] targetAngles, double[] velocities, double[] targetVelocities) {
        this.moduleAngles = angles;
        this.moduleTargetAngles = targetAngles;
        this.moduleVelocities = velocities;
        this.moduleTargetVelocities = targetVelocities;
    }

    // ===== Performance Setters =====

    public void setPerformance(double loopTimeSeconds, double voltage, double current, boolean limiting) {
        this.loopTime = loopTimeSeconds;
        this.loopFrequency = 1.0 / Math.max(loopTimeSeconds, 0.001);
        this.batteryVoltage = voltage;
        this.currentDraw = current;
        this.currentLimitingActive = limiting;
    }

    public void setHardwareStatus(String encoderType) {
        this.encoderType = encoderType;
    }

    // ===== Sensor Setters =====

    public void setIMUStatus(String source, double offset) {
        this.imuSource = source;
        this.octoquadIMUOffset = offset;
    }

    public void setPinpointStatus(boolean healthy, int badCount) {
        this.pinpointHealthy = healthy;
        this.pinpointBadCount = badCount;
    }

    // ===== Camera Setters =====

    public void setCameraStatus(int cameraIndex, String status, double targetDistance) {
        if (cameraIndex == 0) {
            this.camera1Status = status;
            this.camera1Distance = targetDistance;
        } else if (cameraIndex == 1) {
            this.camera2Status = status;
            this.camera2Distance = targetDistance;
        }
    }

    /**
     * Clear trajectory history (call at match start).
     */
    public void clearTrajectory() {
        trajectoryHistory.clear();
        trajectoryTimestamps.clear();
    }

    /**
     * Get trajectory history for external visualization (optional).
     */
    public List<Pose> getTrajectoryHistory() {
        return new ArrayList<>(trajectoryHistory);
    }

    /**
     * Get trajectory timestamps for external visualization (optional).
     */
    public List<Double> getTrajectoryTimestamps() {
        return new ArrayList<>(trajectoryTimestamps);
    }
}
