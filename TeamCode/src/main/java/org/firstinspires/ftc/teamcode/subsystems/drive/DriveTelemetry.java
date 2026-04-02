package org.firstinspires.ftc.teamcode.subsystems.drive;

import com.pedropathing.geometry.Pose;
import com.seattlesolvers.solverslib.kinematics.wpilibkinematics.ChassisSpeeds;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.dashboard.PanelsDashboard;
import org.firstinspires.ftc.teamcode.util.telemetry.TelemetryLogger;
import org.firstinspires.ftc.teamcode.util.vision.LimelightCamera;

import static org.firstinspires.ftc.teamcode.Constants.*;

/**
 * Telemetry handler for the Drive subsystem.
 *
 * <p>This class separates all telemetry concerns (dashboard display and CSV logging)
 * from the drive control logic. It collects data from the Drive subsystem and
 * formats it for Panels dashboard and CSV logging.</p>
 *
 * <h3>Responsibilities:</h3>
 * <ul>
 *   <li><b>Panels Dashboard:</b> Update field overlay, graphs, and telemetry values</li>
 *   <li><b>CSV Logging:</b> Record all drive data for post-match analysis</li>
 *   <li><b>Data Collection:</b> Gather pose, inputs, module states, performance metrics</li>
 * </ul>
 *
 * <h3>Usage:</h3>
 * <pre>
 * // In Robot.init():
 * DriveTelemetry driveTelemetry = new DriveTelemetry(robot.drive);
 *
 * // In Robot.periodic():
 * if (ENABLE_DASHBOARD_OVERLAY) {
 *     driveTelemetry.update();
 * }
 * </pre>
 *
 * <h3>Benefits:</h3>
 * <ul>
 *   <li><b>Single Responsibility:</b> Drive.java only handles drive control</li>
 *   <li><b>Easy to Disable:</b> Just don't instantiate in performance mode</li>
 *   <li><b>Clean Code:</b> Drive.java is ~400 lines instead of ~800</li>
 *   <li><b>Reusable Pattern:</b> Apply to Vision, Elevator, etc.</li>
 * </ul>
 *
 * @see org.firstinspires.ftc.teamcode.subsystems.drive.Drive
 * @see org.firstinspires.ftc.teamcode.util.telemetry.TelemetryLogger
 */
public class DriveTelemetry {

    private final Drive drive;
    private final Robot robot;
    private final PanelsDashboard dashboard;
    private final TelemetryLogger csvLogger;

    // Cached data to avoid redundant getter calls
    private Pose lastPose;
    private ChassisSpeeds lastInputs;
    private Double lastLoopTime;
    private Double lastBatteryVoltage;
    private Double lastCurrentDraw;
    private Boolean lastCurrentLimiting;

    /**
     * Creates a new DriveTelemetry handler.
     *
     * @param drive the Drive subsystem to collect telemetry from
     */
    public DriveTelemetry(Drive drive) {
        this.drive = drive;
        this.robot = Robot.getInstance();
        this.dashboard = PanelsDashboard.getInstance();

        // Create CSV logger
        // Update every loop (minUpdateIntervalMs = 0) for high-resolution data
        this.csvLogger = new TelemetryLogger(
            "drive",           // filename
            ENABLE_CSV_LOGGING, // CSV enabled
            false,             // Dashboard disabled (handled separately)
            0                  // Update every loop
        );
    }

    /**
     * Updates all telemetry (dashboard and CSV).
     *
     * <p>This method should be called once per loop iteration, typically from
     * {@link Robot#periodic()}.</p>
     */
    public void update() {
        // Collect data from drive subsystem
        collectTelemetryData();

        // Update Panels Dashboard
        if (ENABLE_DASHBOARD_OVERLAY) {
            updateDashboard();
        }

        // Update CSV Logger
        if (ENABLE_CSV_LOGGING) {
            updateCsvLogger();
        }
    }

    /**
     * Collects all telemetry data from the Drive subsystem.
     * Caches the data to avoid redundant getter calls.
     */
    private void collectTelemetryData() {
        lastPose = drive.getPose();
        lastInputs = drive.getLastInputs();
        lastLoopTime = drive.getLastLoopTime();
        lastBatteryVoltage = robot.getBatteryVoltage();
        lastCurrentDraw = drive.getCurrentDraw();
        lastCurrentLimiting = drive.isCurrentLimitingActive();
    }

    /**
     * Updates Panels Dashboard with drive telemetry.
     */
    private void updateDashboard() {
        // ===== Robot Pose & Localization =====
        dashboard.setRobotPose(lastPose);
        dashboard.setFusedPose(drive.getFusedPose());

        // Pinpoint pose (if available)
        if (robot.pinpoint != null) {
            Pose pinpointPose = new Pose(
                robot.pinpoint.getPosition().getX(org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.INCH),
                robot.pinpoint.getPosition().getY(org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.INCH),
                robot.pinpoint.getHeading(org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.RADIANS)
            );
            dashboard.setPinpointPose(pinpointPose);
        }

        // Localization uncertainty
        if (ENABLE_SENSOR_FUSION) {
            double[] posUncertainty = drive.getPositionUncertainty();
            double headingUncertainty = drive.getHeadingUncertainty();
            dashboard.setLocalizationUncertainty(posUncertainty, headingUncertainty);
        }

        // ===== Vision Status =====
        if (robot.vision != null) {
            StringBuilder activeNames = new StringBuilder();
            if (robot.vision.isTagVisible() && !robot.vision.getActiveCameras().isEmpty()) {
                for (LimelightCamera cam : robot.vision.getActiveCameras()) {
                    if (activeNames.length() > 0) activeNames.append(", ");
                    activeNames.append(cam.getName());
                }
            }
            dashboard.setVisionStatus(
                robot.vision.isTagVisible(),
                robot.vision.getActiveCameras().size(),
                activeNames.toString()
            );
        }

        // ===== Drive Inputs =====
        String mode = drive.isAutonomousMode() ? "Auto" : "TeleOp";
        dashboard.setInputs(
            lastInputs.vxMetersPerSecond,
            lastInputs.vyMetersPerSecond,
            lastInputs.omegaRadiansPerSecond,
            mode
        );

        // ===== Module States =====
        double[] moduleAngles = new double[4];
        double[] moduleTargetAngles = new double[4];
        double[] moduleVelocities = new double[4];
        double[] moduleTargetVelocities = new double[4];

        org.firstinspires.ftc.teamcode.subsystems.drive.OctoSwerveModuleV2[] modules = drive.getModules();
        for (int i = 0; i < 4; i++) {
            moduleAngles[i] = modules[i].getModuleHeadingRadians();
            moduleTargetAngles[i] = modules[i].getTargetAngleRadians();
            moduleVelocities[i] = modules[i].getCurrentVelocityInchesPerSec();
            moduleTargetVelocities[i] = modules[i].getTargetMagnitude();
        }

        dashboard.setModuleStates(moduleAngles, moduleTargetAngles, moduleVelocities, moduleTargetVelocities);

        // ===== Performance Metrics =====
        dashboard.setPerformance(
            lastLoopTime,
            lastBatteryVoltage,
            lastCurrentDraw,
            lastCurrentLimiting
        );

        dashboard.setHardwareStatus(drive.getEncoderType());

        // ===== IMU & Sensor Status =====
        if (OCTOQUAD_IMU_BACKUP_ENABLED) {
            dashboard.setIMUStatus(
                robot.getIMUSource(),
                robot.getOctoQuadIMUOffset()
            );

            if (ENABLE_PINPOINT_HEALTH_MONITOR && robot.pinpoint != null) {
                dashboard.setPinpointStatus(
                    drive.isPinpointHealthy(),
                    drive.getPinpointBadReadingCount()
                );
            }
        }

        // ===== Camera Status =====
        if (robot.limelightCameras != null) {
            for (int i = 0; i < robot.limelightCameras.length; i++) {
                LimelightCamera cam = robot.limelightCameras[i];
                dashboard.setCameraStatus(
                    i,
                    cam.hasValidDetection() ? "TRACKING" : "SEARCHING",
                    cam.getDetectionDistance()
                );
            }
        }

        // Update dashboard (field overlay + graphs)
        dashboard.update();
    }

    /**
     * Updates CSV logger with drive telemetry.
     */
    private void updateCsvLogger() {
        // ===== Robot Pose =====
        csvLogger.addData("Robot X (Inches)", lastPose.getX());
        csvLogger.addData("Robot Y (Inches)", lastPose.getY());
        csvLogger.addData("Robot Heading (Rad)", lastPose.getHeading());

        // ===== Controller Inputs =====
        csvLogger.addData("Input Forward", lastInputs.vxMetersPerSecond);
        csvLogger.addData("Input Lateral", lastInputs.vyMetersPerSecond);
        csvLogger.addData("Input Turn", lastInputs.omegaRadiansPerSecond);
        csvLogger.addData("Input Mode", drive.isAutonomousMode() ? "Auto" : "TeleOp");

        // ===== Sensor Fusion Data =====
        Pose fusedPose = drive.getFusedPose();
        if (fusedPose != null) {
            csvLogger.addData("Fusion X (Inches)", fusedPose.getX());
            csvLogger.addData("Fusion Y (Inches)", fusedPose.getY());
            csvLogger.addData("Fusion Heading (Rad)", fusedPose.getHeading());

            double[] posUncertainty = drive.getPositionUncertainty();
            double headingUncertainty = drive.getHeadingUncertainty();
            csvLogger.addData("Fusion X Uncertainty", posUncertainty[0]);
            csvLogger.addData("Fusion Y Uncertainty", posUncertainty[1]);
            csvLogger.addData("Fusion Heading Uncertainty", headingUncertainty);
        }

        // ===== Pinpoint Odometry =====
        if (robot.pinpoint != null) {
            csvLogger.addData("Pinpoint X", robot.pinpoint.getPosition().getX(org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.INCH));
            csvLogger.addData("Pinpoint Y", robot.pinpoint.getPosition().getY(org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.INCH));
            csvLogger.addData("Pinpoint Heading", robot.pinpoint.getHeading(org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.RADIANS));
        }

        // ===== Vision Data =====
        if (robot.vision != null) {
            csvLogger.addData("Vision Tag Visible", robot.vision.isTagVisible() ? 1 : 0);
            csvLogger.addData("Vision Active Cameras", robot.vision.getActiveCameras().size());
            csvLogger.addData("Vision Healthy Cameras", robot.vision.getHealthyCameraCount());
        }

        // ===== Power & Performance =====
        csvLogger.addData("Battery Voltage (V)", lastBatteryVoltage);
        csvLogger.addData("Loop Time (ms)", lastLoopTime * 1000);
        csvLogger.addData("Current Limiting Active", lastCurrentLimiting ? 1 : 0);
        csvLogger.addData("Swerve Encoder Type", drive.getEncoderType());

        // ===== IMU Status =====
        csvLogger.addData("IMU Source", robot.getIMUSource());
        csvLogger.addData("OctoQuad IMU Offset (rad)", robot.getOctoQuadIMUOffset());

        // ===== Pinpoint Health =====
        if (ENABLE_PINPOINT_HEALTH_MONITOR && robot.pinpoint != null) {
            csvLogger.addData("Pinpoint Healthy", drive.isPinpointHealthy() ? 1 : 0);
            csvLogger.addData("Pinpoint Bad Count", drive.getPinpointBadReadingCount());
        }

        // ===== SRS Hub Current Data =====
        if (SRS_HUB_ENABLED) {
            csvLogger.addData("Current Draw (A)", lastCurrentDraw);
            csvLogger.addData("SRS Hub Status", robot.srsHub != null && robot.srsHub.disconnected() ? "DISCONNECTED" : "OK");
        }

        csvLogger.update();
    }

    /**
     * Clears trajectory history in the dashboard.
     *
     * <p>Call this at the start of autonomous or teleop to begin a fresh trajectory.</p>
     */
    public void clearTrajectory() {
        dashboard.clearTrajectory();
    }

    /**
     * Closes the telemetry logger.
     *
     * <p>Should be called when the OpMode stops.</p>
     */
    public void close() {
        csvLogger.close();
    }
}
