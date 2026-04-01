package org.firstinspires.ftc.teamcode.dashboard;

import com.bylazar.configurables.PanelsConfigurables;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.field.PanelsField;
import com.bylazar.field.Canvas;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.Robot;

/**
 * Panels Dashboard Configuration for ARES Robot.
 *
 * <p>This class configures the Panels web dashboard with organized tabs and fields
 * for displaying all robot telemetry data during competition.</p>
 *
 * <h3>Dashboard Structure:</h3>
 * <ul>
 *   <li><b>Robot Pose:</b> Fused pose, uncertainties, field view</li>
 *   <li><b>Vision:</b> Tag visibility, camera status, fusion mode</li>
 *   <li><b>Drive:</b> Motor inputs, encoder types, hardware status</li>
 *   <li><b>IMU Backup:</b> Pinpoint health, OctoQuad backup status</li>
 *   <li><b>Performance:</b> Loop timing, battery voltage, current draw</li>
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

    // Dashboard fields
    private Pose robotPose;
    private double loopTime;
    private double batteryVoltage;
    private double currentDraw;
    private boolean currentLimitingActive;
    private String encoderType;
    private double inputForward;
    private double inputLateral;
    private double inputTurn;
    private String inputMode;
    private String imuSource;
    private double octoquadIMUOffset;
    private boolean pinpointHealthy;
    private int pinpointBadCount;
    private boolean visionTagVisible;
    private int visionActiveCameras;
    private int visionHealthyCameras;

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
        // Update all telemetry fields
        telemetry.addData("Robot Pose X", robotPose != null ? robotPose.getX() : 0.0);
        telemetry.addData("Robot Pose Y", robotPose != null ? robotPose.getY() : 0.0);
        telemetry.addData("Robot Pose Heading", robotPose != null ? Math.toDegrees(robotPose.getHeading()) : 0.0);
        telemetry.addData("Loop Time (ms)", loopTime * 1000);
        telemetry.addData("Battery Voltage (V)", batteryVoltage);
        telemetry.addData("Current Draw (A)", currentDraw);
        telemetry.addData("Current Limiting", currentLimitingActive ? "ACTIVE" : "OFF");
        telemetry.addData("Encoder Type", encoderType);
        telemetry.addData("Input Forward", inputForward);
        telemetry.addData("Input Lateral", inputLateral);
        telemetry.addData("Input Turn", inputTurn);
        telemetry.addData("Input Mode", inputMode);
        telemetry.addData("IMU Source", imuSource);
        telemetry.addData("OctoQuad IMU Offset", Math.toDegrees(octoquadIMUOffset));
        telemetry.addData("Pinpoint Healthy", pinpointHealthy ? "YES" : "NO");
        telemetry.addData("Pinpoint Bad Count", pinpointBadCount);
        telemetry.addData("Vision Tag Visible", visionTagVisible ? "YES" : "NO");
        telemetry.addData("Vision Active Cameras", visionActiveCameras);
        telemetry.addData("Vision Healthy Cameras", visionHealthyCameras);
    }

    // Setter methods for updating dashboard data

    public void setRobotPose(Pose pose) {
        this.robotPose = pose;
    }

    public void setLoopTime(double seconds) {
        this.loopTime = seconds;
    }

    public void setBatteryVoltage(double voltage) {
        this.batteryVoltage = voltage;
    }

    public void setCurrentDraw(double amps) {
        this.currentDraw = amps;
    }

    public void setCurrentLimitingActive(boolean active) {
        this.currentLimitingActive = active;
    }

    public void setEncoderType(String type) {
        this.encoderType = type;
    }

    public void setInputs(double forward, double lateral, double turn, String mode) {
        this.inputForward = forward;
        this.inputLateral = lateral;
        this.inputTurn = turn;
        this.inputMode = mode;
    }

    public void setIMUStatus(String source, double offset) {
        this.imuSource = source;
        this.octoquadIMUOffset = offset;
    }

    public void setPinpointStatus(boolean healthy, int badCount) {
        this.pinpointHealthy = healthy;
        this.pinpointBadCount = badCount;
    }

    public void setVisionStatus(boolean tagVisible, int activeCameras, int healthyCameras) {
        this.visionTagVisible = tagVisible;
        this.visionActiveCameras = activeCameras;
        this.visionHealthyCameras = healthyCameras;
    }
}
