package org.firstinspires.ftc.teamcode.dashboard;

import com.bylazar.configurables.PanelsConfigurables;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.field.PanelsField;
import com.bylazar.field.Canvas;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.util.vision.LimelightCamera;
import org.firstinspires.ftc.teamcode.Constants;

/**
 * Panels Dashboard Configuration for ARES Robot.
 *
 * <p>This class configures the Panels web dashboard with organized tabs and fields
 * for displaying all robot telemetry data during competition.</p>
 *
 * <h3>Dashboard Structure (3 Tabs):</h3>
 * <ul>
 *   <li><b>Robot:</b> Pose, Vision, Limelight camera feeds, Localization</li>
 *   <li><b>Drive:</b> Inputs, Performance, Hardware, Power</li>
 *   <li><b>Sensors:</b> IMU backup, Pinpoint health, Camera details</li>
 * </ul>
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

    // ===== Tab 1: Robot =====
    private Pose robotPose;
    private boolean visionTagVisible;
    private int visionActiveCameras;
    private String activeCameraNames;

    // ===== Tab 2: Drive =====
    private double inputForward;
    private double inputLateral;
    private double inputTurn;
    private String inputMode;
    private double loopTime;
    private double loopFrequency;
    private double batteryVoltage;
    private double currentDraw;
    private boolean currentLimitingActive;
    private String encoderType;

    // ===== Tab 3: Sensors =====
    private String imuSource;
    private double octoquadIMUOffset;
    private boolean pinpointHealthy;
    private int pinpointBadCount;
    private String camera1Status;
    private String camera2Status;
    private double camera1Distance;
    private double camera2Distance;

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
     * Initialize the dashboard.
     */
    public void initialize() {
        // Register with Panels configurables
        PanelsConfigurables.INSTANCE.refreshClass(this);
    }

    /**
     * Update all dashboard data.
     * Call this method once per loop iteration.
     */
    public void update() {
        // ===== Tab 1: Robot =====
        telemetry.addTab("Robot");

        // Robot Pose
        telemetry.addData("Pose X (in)", robotPose != null ? String.format("%.2f", robotPose.getX()) : "0.00");
        telemetry.addData("Pose Y (in)", robotPose != null ? String.format("%.2f", robotPose.getY()) : "0.00");
        telemetry.addData("Pose Heading (°)", robotPose != null ? String.format("%.1f", Math.toDegrees(robotPose.getHeading())) : "0.0");

        // Vision Status
        telemetry.addData("Tag Visible", visionTagVisible ? "YES ✓" : "NO");
        telemetry.addData("Active Cameras", visionActiveCameras);

        // Limelight Camera URLs (for direct viewing)
        if (robot.limelightCameras != null && robot.limelightCameras.length > 0) {
            StringBuilder camUrls = new StringBuilder();
            for (int i = 0; i < robot.limelightCameras.length; i++) {
                String camName = robot.limelightCameras[i].getName();
                String url = "http://" + robot.limelightCameras[i].getLimelight().getIpAddress() + ":5800";
                camUrls.append(camName).append(": ").append(url);
                if (i < robot.limelightCameras.length - 1) camUrls.append(" | ");
            }
            telemetry.addData("Camera Streams", camUrls.toString());
        }

        // ===== Tab 2: Drive =====
        telemetry.addTab("Drive");

        // Inputs
        telemetry.addData("Input Fwd", String.format("%.3f", inputForward));
        telemetry.addData("Input Lat", String.format("%.3f", inputLateral));
        telemetry.addData("Input Turn", String.format("%.3f", inputTurn));
        telemetry.addData("Mode", inputMode);

        // Performance
        telemetry.addData("Loop Time", String.format("%.1f ms", loopTime * 1000));
        telemetry.addData("Frequency", String.format("%.1f Hz", loopFrequency));

        // Hardware
        telemetry.addData("Battery", String.format("%.2f V", batteryVoltage));
        telemetry.addData("Current", String.format("%.2f A", currentDraw));
        telemetry.addData("Limiting", currentLimitingActive ? "ACTIVE" : "OFF");
        telemetry.addData("Encoders", encoderType);

        // ===== Tab 3: Sensors =====
        telemetry.addTab("Sensors");

        // IMU Backup Status
        telemetry.addData("IMU Source", imuSource);
        telemetry.addData("OctoQuad Offset", String.format("%.2f°", Math.toDegrees(octoquadIMUOffset)));

        // Pinpoint Health
        telemetry.addData("Pinpoint Healthy", pinpointHealthy ? "YES ✓" : "DRIFTED");
        telemetry.addData("Pinpoint Bad Count", pinpointBadCount);

        // Limelight Camera Details
        if (robot.limelightCameras != null && robot.limelightCameras.length > 0) {
            telemetry.addData("Cam 1 Status", camera1Status);
            telemetry.addData("Cam 1 Target Dist", String.format("%.1f in", camera1Distance));

            if (robot.limelightCameras.length > 1) {
                telemetry.addData("Cam 2 Status", camera2Status);
                telemetry.addData("Cam 2 Target Dist", String.format("%.1f in", camera2Distance));
            }
        }
    }

    // ===== Tab 1: Robot Setters =====

    public void setRobotPose(Pose pose) {
        this.robotPose = pose;
    }

    public void setVisionStatus(boolean tagVisible, int activeCameras, String activeNames) {
        this.visionTagVisible = tagVisible;
        this.visionActiveCameras = activeCameras;
        this.activeCameraNames = activeNames;
    }

    // ===== Tab 2: Drive Setters =====

    public void setInputs(double forward, double lateral, double turn, String mode) {
        this.inputForward = forward;
        this.inputLateral = lateral;
        this.inputTurn = turn;
        this.inputMode = mode;
    }

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

    // ===== Tab 3: Sensors Setters =====

    public void setIMUStatus(String source, double offset) {
        this.imuSource = source;
        this.octoquadIMUOffset = offset;
    }

    public void setPinpointStatus(boolean healthy, int badCount) {
        this.pinpointHealthy = healthy;
        this.pinpointBadCount = badCount;
    }

    public void setCameraStatus(int cameraIndex, String status, double targetDistance) {
        if (cameraIndex == 0) {
            this.camera1Status = status;
            this.camera1Distance = targetDistance;
        } else if (cameraIndex == 1) {
            this.camera2Status = status;
            this.camera2Distance = targetDistance;
        }
    }
}
