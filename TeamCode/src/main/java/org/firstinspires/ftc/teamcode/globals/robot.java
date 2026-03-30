package org.firstinspires.ftc.teamcode.globals;

import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.Drive;
import com.seattlesolvers.solverslib.hardware.motors.CRServoEx;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.util.TelemetryData;
import org.firstinspires.ftc.teamcode.util.DataLogger;

import org.firstinspires.ftc.teamcode.hardware.SRSHub;
import org.firstinspires.ftc.teamcode.util.drivers.OctoQuadFWv3;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.Drive;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.Vision;

import java.io.File;
import java.io.IOException;

import dev.nullftc.profiler.Profiler;
import dev.nullftc.profiler.entry.BasicProfilerEntryFactory;
import dev.nullftc.profiler.exporter.CSVProfilerExporter;

/**
 * Main robot singleton class that manages all hardware, subsystems, and the robot's update loop.
 *
 * <p>This class serves as the central hub for the entire robot codebase. It implements the singleton
 * pattern to ensure there is only one instance of the robot throughout the program's lifecycle.
 * The robot is responsible for:</p>
 *
 * <ul>
 *   <li>Initializing all hardware components (motors, servos, sensors)</li>
 *   <li>Configuring performance optimizations (PhotonCore bulk caching)</li>
 *   <li>Creating and managing subsystem instances</li>
 *   <li>Running the main update loop that executes the command scheduler</li>
 *   <li>Managing telemetry, logging, and profiling</li>
 * </ul>
 *
 * <h3>Performance Optimizations:</h3>
 * <ul>
 *   <li><b>PhotonCore Bulk Caching:</b> Manual mode with 8 parallel commands for reduced I2C latency</li>
 *   <li><b>Parallel Servo Updates:</b> All servos update simultaneously for ~3-6ms faster loop times</li>
 *   <li><b>REV Hub IMU Disabled:</b> Reduces I2C bus traffic - using GoBilda Pinpoint IMU instead</li>
 *   <li><b>Voltage Sensor Caching:</b> Updates every 250ms to reduce polling overhead</li>
 * </ul>
 *
 * <p><b>PhotonCore Integration:</b> This robot uses PhotonCore's manual bulk caching mode
 * to optimize hardware read performance. All hardware reads are cached and must be cleared
 * at the end of each update loop using {@link #updateLoop(TelemetryData)}.</p>
 *
 * <p><b>Usage:</b> Access the robot instance via {@link #getInstance()}, call
 * {@link #init(HardwareMap)} during OpMode initialization, then call
 * {@link #updateLoop(TelemetryData)} every iteration of the OpMode's loop.</p>
 *
 * @see com.seattlesolvers.solverslib.command.Robot
 * @see <a href="https://gitlab.com/OutoftheBoxRobotics/PhotonCore">PhotonCore Documentation</a>
 */
public class Robot extends com.seattlesolvers.solverslib.command.Robot {

    /**
     * Singleton instance of the robot. Only one instance exists throughout the program.
     */
    private static final Robot instance = new Robot();

    /**
     * Returns the singleton instance of the robot.
     *
     * <p>This is the primary way to access the robot instance throughout the codebase.
     * The singleton pattern ensures consistent hardware and subsystem state across all code.</p>
     *
     * @return the robot instance
     */
    public static Robot getInstance() {
        return instance;
    }

    /**
     * The FTC SDK hardware map containing all configured hardware devices.
     * Initialized during {@link #init(HardwareMap)}.
     */
    public HardwareMap hardwareMap;

    /**
     * Performance profiler for measuring code execution times.
     * Outputs CSV logs to the /FIRST/logs/ directory for analysis.
     */
    public Profiler profiler;

    /**
     * File reference for the profiler output CSV.
     */
    public File file;

    /**
     * Voltage sensor for monitoring robot battery voltage.
     * Automatically retrieved from the hardware map during initialization.
     */
    public VoltageSensor voltageSensor;

    /**
     * Cached voltage reading to reduce sensor polling frequency.
     * Updated every 250ms via {@link #getVoltage()}.
     */
    private double cachedVoltage;

    /**
     * Timer for tracking when to update the cached voltage reading.
     */
    private ElapsedTime voltageTimer;

    // ----- Loop Optimization Timers -----

    /**
     * Timer for measuring loop execution time.
     * Used for performance profiling and optimization.
     */
    private ElapsedTime loopTimer;

    /**
     * Counter for tracking total loop iterations.
     * Used for throttling telemetry and logger updates.
     */
    private long loopCounter = 0;

    /**
     * Last measured loop time in milliseconds.
     * Updated every loop for performance monitoring.
     */
    private double lastLoopTimeMs = 0;

    /**
     * Rolling average of loop times (last 10 loops).
     * Used for smooth performance monitoring.
     */
    private double averageLoopTimeMs = 0;

    // ----- Hardware Items -----

    /**
     * Front-left swerve drive motor.
     * Connected to the Control Hub and configured with 0.01 tolerance caching.
     */
    public MotorEx frontLeftMotor;

    /**
     * Front-right swerve drive motor.
     * Connected to the Control Hub and configured with 0.01 tolerance caching.
     */
    public MotorEx frontRightMotor;

    /**
     * Back-left swerve drive motor.
     * Connected to the Control Hub and configured with 0.01 tolerance caching.
     */
    public MotorEx backLeftMotor;

    /**
     * Back-right swerve drive motor.
     * Connected to the Control Hub and configured with 0.01 tolerance caching.
     */
    public MotorEx backRightMotor;

    /**
     * Front-left swerve steering servo (Melon Super Servo).
     * Connected to the SRS (Secondary Robot Server) Hub via CRServoEx interface.
     * These are continuous rotation servos used for coaxial swerve steering.
     */
    public CRServoEx frontLeftServo;

    /**
     * Front-right swerve steering servo (Melon Super Servo).
     * Connected to the SRS Hub via CRServoEx interface.
     */
    public CRServoEx frontRightServo;

    /**
     * Back-left swerve steering servo (Melon Super Servo).
     * Connected to the SRS Hub via CRServoEx interface.
     */
    public CRServoEx backLeftServo;

    /**
     * Back-right swerve steering servo (Melon Super Servo).
     * Connected to the SRS Hub via CRServoEx interface.
     */
    public CRServoEx backRightServo;

    /**
     * GoBilda Pinpoint odometry sensor for dead-wheel localization.
     * Provides x, y position and heading measurements.
     * Initialized with default configuration and 4-bar encoder pods.
     */
    public GoBildaPinpointDriver pinpoint;

    /**
     * REV OctoQuad encoder processing unit using Firmware v3 driver.
     * Processes encoder signals from swerve drive motors and through-bore encoders.
     * Provides high-precision position feedback for PIDF control.
     *
     * <p>The OctoQuadFWv3 driver adds support for:</p>
     * <ul>
     *   <li>CRC-16 PROFIBUS validation for reliable encoder data</li>
     *   <li>Built-in IMU for heading tracking (MK2 feature)</li>
     *   <li>Absolute localizer for deadwheel odometry</li>
     *   <li>PWM absolute encoder support with wrap tracking</li>
     * </ul>
     */
    public OctoQuadFWv3 octoquad;

    /**
     * Array of Limelight3A vision cameras for AprilTag localization.
     * <p>
     * Supports multiple cameras for 360° coverage. Configure camera names in
     * {@link org.firstinspires.ftc.teamcode.globals.Constants#LIMELIGHT_NAMES}.
     * </p>
     * @deprecated Use {@link #limelightCameras} instead for multi-camera support
     */
    @Deprecated
    public Limelight3A limelight;

    /**
     * Array of Limelight camera wrappers with metadata for multi-camera systems.
     * <p>
     * Each camera includes priority, detection status, and pose measurements.
     * The Vision subsystem automatically selects the best camera based on:
     * <ul>
     *   <li>Tag visibility</li>
     *   <li>Tag distance (closer is better)</li>
     *   <li>Camera priority (configured in Constants)</li>
     *   <li>Robot heading (adaptive priority)</li>
     * </ul>
     * </p>
     */
    public org.firstinspires.ftc.teamcode.util.LimelightCamera[] limelightCameras;

    /**
     * SRS Hub for analog sensor monitoring.
     * Used to monitor current draw from GoBilda Floodgate power switch.
     *
     * <p>The SRS Hub provides:</p>
     * <ul>
     *   <li>12 analog/digital input pins</li>
     *   <li>6 encoder ports (quadrature or PWM)</li>
     *   <li>3 I2C buses for external sensors</li>
     *   <li>Bulk-read optimization via I2C daisy-chain</li>
     * </ul>
     *
     * <p><b>Current Use:</b> Pin 1 monitors GoBilda Floodgate analog current output.</p>
     */
    public SRSHub srsHub;

    // ----- Subsystems -----

    /**
     * Main drive subsystem combining swerve drivetrain control with PedroPathing.
     * Handles both manual teleop driving and autonomous path following.
     */
    public Drive drive;

    /**
     * Vision subsystem for AprilTag-based localization.
     * Integrates with Limelight to provide vision pose updates to the drive subsystem.
     */
    public Vision vision;

    // ----- Telemetry -----

    /**
     * Global telemetry instance for sending data to the driver station and FTC Dashboard.
     * Used throughout the codebase for debugging and monitoring robot state.
     */
    public TelemetryData telemetry;

    /**
     * CSV data logger for recording telemetry to file.
     * Creates timestamped CSV files in the /FIRST/logs/ directory for post-match analysis.
     */
    public DataLogger logger;

    // ===== Loop Optimization Configuration =====

    /**
     * Telemetry update interval (in loops).
     * Telemetry is updated every N loops to reduce overhead.
     * Default: 5 = ~100Hz (assuming 50Hz loop rate)
     * Set to 1 for every loop, 10 for ~50Hz, etc.
     */
    public static final int TELEMETRY_UPDATE_INTERVAL = 5;

    /**
     * Data logger flush interval (in loops).
     * The logger flushes to disk every N loops for crash recovery.
     * Default: 10 = ~5Hz (flushing every loop is too slow)
     * Higher values = faster loops but more data loss risk on crash
     */
    public static final int LOGGER_FLUSH_INTERVAL = 10;

    /**
     * Whether to enable loop time profiling.
     * Adds minimal overhead (~1-2 microseconds) but provides valuable performance data.
     */
    public static final boolean ENABLE_LOOP_PROFILING = true;

    /**
     * Initializes all robot hardware and subsystems.
     *
     * <p>This method sets up the entire robot in the following order:</p>
     *
     * <ol>
     *   <li>Creates profiler and logging infrastructure</li>
     *   <li>Configures PhotonCore for bulk caching optimization</li>
     *   <li>Initializes voltage sensor with 250ms caching</li>
     *   <li>Initializes swerve drive motors with caching tolerance</li>
     *   <li>Initializes swerve steering servos</li>
     *   <li>Configures Limelight vision camera (250Hz polling, pipeline 0)</li>
     *   <li>Initializes GoBilda Pinpoint odometry with 4-bar pods (includes IMU)</li>
     *   <li>Initializes OctoQuad encoder processor (Firmware v3 with CRC validation)</li>
     *   <li>Initializes OctoQuad localizer if enabled (optional deadwheel odometry)</li>
     *   <li>Creates drive and vision subsystems</li>
     * </ol>
     *
     * <h3>IMU Configuration:</h3>
     * <p><b>REV Hub IMU is NOT initialized.</b> We use the GoBilda Pinpoint IMU instead for heading,
     * which reduces I2C bus traffic and improves loop times. The Pinpoint IMU provides reliable
     * heading data on its own I2C bus.</p>
     *
     * <p><b>PhotonCore Configuration:</b> Manual bulk caching is enabled on both hubs
     * with a maximum of 8 parallel commands. Parallel servo updates are enabled for
     * optimal performance (~3-6ms faster loop times).</p>
     *
     * <p><b>Important:</b> This method must be called during OpMode initialization before
     * any subsystems are used. The update loop must call {@link #updateLoop(TelemetryData)}
     * to clear bulk caches each iteration.</p>
     *
     * @param hwMap the FTC SDK hardware map containing configured hardware devices
     * @see #updateLoop(TelemetryData)
     */
    public void init(HardwareMap hwMap) {
        this.hardwareMap = hwMap;

        // Logging & Profiler setup
        File logsFolder = new File(AppUtil.FIRST_FOLDER, "logs");
        if (!logsFolder.exists()) logsFolder.mkdirs();

        long timestamp = System.currentTimeMillis();
        file = new File(logsFolder, "profiler-" + timestamp + ".csv");
        profiler = Profiler.builder()
                .factory(new BasicProfilerEntryFactory())
                .exporter(new CSVProfilerExporter(file))
                .debugLog(false)
                .build();

        // ---------------------------------------------
        // Photon Configuration: Optimized for Performance
        // ---------------------------------------------
        PhotonCore.CONTROL_HUB.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        PhotonCore.EXPANSION_HUB.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        PhotonCore.experimental.setMaximumParallelCommands(org.firstinspires.ftc.teamcode.globals.Constants.PHOTON_PARALLEL_COMMANDS);

        // Enable parallel servo updates for faster loop times (~3-6ms improvement)
        // All servos are on USB-connected hubs, so this is safe to enable
        PhotonCore.PARALLELIZE_SERVOS = org.firstinspires.ftc.teamcode.globals.Constants.PARALLELIZE_SERVOS;
        PhotonCore.enable();

        voltageSensor = hwMap.voltageSensor.iterator().next();

        // Initialize Swerve Drive Motors with optimized cache tolerance
        double motorCacheTolerance = org.firstinspires.ftc.teamcode.globals.Constants.MOTOR_CACHE_TOLERANCE;
        frontLeftMotor = new MotorEx(hwMap, "frontLeftMotor").setCachingTolerance(motorCacheTolerance);
        frontRightMotor = new MotorEx(hwMap, "frontRightMotor").setCachingTolerance(motorCacheTolerance);
        backLeftMotor = new MotorEx(hwMap, "backLeftMotor").setCachingTolerance(motorCacheTolerance);
        backRightMotor = new MotorEx(hwMap, "backRightMotor").setCachingTolerance(motorCacheTolerance);

        // Initialize Swerve Servos with optimized cache tolerance
        double servoCacheTolerance = org.firstinspires.ftc.teamcode.globals.Constants.SERVO_CACHE_TOLERANCE;
        frontLeftServo = new CRServoEx(hwMap, "frontLeftServo").setCachingTolerance(servoCacheTolerance);
        frontRightServo = new CRServoEx(hwMap, "frontRightServo").setCachingTolerance(servoCacheTolerance);
        backLeftServo = new CRServoEx(hwMap, "backLeftServo").setCachingTolerance(servoCacheTolerance);
        backRightServo = new CRServoEx(hwMap, "backRightServo").setCachingTolerance(servoCacheTolerance);

        // Note: OctoQuad is present for processing motor encoders and REV through bores (axial rotation)
        // You usually interact with OctoQuad directly via its driver class to query the positions in the Drive subsystem
        // e.g., OctoQuad octoquad = hwMap.get(OctoQuad.class, "octoquad");

        // Initialize Vision and Performance Monitoring
        if (org.firstinspires.ftc.teamcode.globals.Constants.ENABLE_CSV_LOGGING) {
            logger = new DataLogger("robot-telemetry");
        }
        loopTimer = new ElapsedTime();
        loopTimer.reset();

        // Initialize Limelight cameras (multi-camera support with calibration)
        String[] cameraNames = org.firstinspires.ftc.teamcode.globals.Constants.LIMELIGHT_NAMES;
        double[] cameraPriorities = org.firstinspires.ftc.teamcode.globals.Constants.LIMELIGHT_PRIORITIES;
        double[][] mountPositions = org.firstinspires.ftc.teamcode.globals.Constants.LIMELIGHT_MOUNT_POSITIONS;
        double[] orientationOffsets = org.firstinspires.ftc.teamcode.globals.Constants.LIMELIGHT_ORIENTATION_OFFSETS;
        double[] positionNoise = org.firstinspires.ftc.teamcode.globals.Constants.LIMELIGHT_POSITION_NOISE;
        double[] headingNoise = org.firstinspires.ftc.teamcode.globals.Constants.LIMELIGHT_HEADING_NOISE;

        limelightCameras = new org.firstinspires.ftc.teamcode.util.LimelightCamera[cameraNames.length];

        for (int i = 0; i < cameraNames.length; i++) {
            Limelight3A ll = hwMap.get(Limelight3A.class, cameraNames[i]);
            ll.setPollRateHz(org.firstinspires.ftc.teamcode.globals.Constants.LIMELIGHT_POLL_RATE);
            ll.pipelineSwitch(0);
            ll.start();

            // Create camera with full calibration
            limelightCameras[i] = new org.firstinspires.ftc.teamcode.util.LimelightCamera(
                ll,
                cameraNames[i],
                cameraPriorities[i],
                mountPositions[i][0],  // Mount X offset
                mountPositions[i][1],  // Mount Y offset
                orientationOffsets[i],   // Orientation offset
                positionNoise[i],        // Position noise
                headingNoise[i]           // Heading noise
            );
        }

        // For backward compatibility, set limelight to first camera
        if (limelightCameras.length > 0) {
            limelight = limelightCameras[0].getLimelight();
        }

        // Initialize GoBilda Pinpoint
        // Assumes default pinpoint configuration. Adjust setOffsets() in the future if pinpoint is non-centered.
        pinpoint = hwMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.setOffsets(0, 0, DistanceUnit.MM); 
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        pinpoint.resetPosAndIMU();

        // Initialize OctoQuad with Firmware v3 driver
        octoquad = hwMap.get(OctoQuadFWv3.class, "octoquad");
        RobotLog.i("OctoQuad: Initialized - Chip ID: 0x" + Integer.toHexString(octoquad.getChipId() & 0xFF) +
                   " Firmware: " + octoquad.getFirmwareVersionString());

        // Initialize OctoQuad localizer (if enabled in Constants)
        initializeOctoQuadLocalizer();

        // Initialize SRS Hub for current monitoring (if enabled in Constants)
        initializeSRSHub();

        // Instantiate Subsystems
        drive = new Drive();
        vision = new Vision();
    }

    /**
     * Gets the current robot battery voltage with caching.
     *
     * <p>This method implements voltage sensor caching to reduce hardware polling frequency.
     * The voltage is only read from the sensor every 250 milliseconds. Between reads, a cached
     * value is returned. This reduces I2C/I2C bus traffic and improves performance.</p>
     *
     * <p><b>Fallback behavior:</b> If the voltage reading is NaN or 0 (indicating a sensor error
     * or disconnected battery), the method returns 12V as a safe default value.</p>
     *
     * @return the current battery voltage in volts (cached, updated every 250ms)
     */
    public double getVoltage() {
        if (voltageTimer == null) {
            voltageTimer = new ElapsedTime();
            cachedVoltage = voltageSensor.getVoltage();
        } else if (voltageTimer.milliseconds() > 250) {
            cachedVoltage = voltageSensor.getVoltage();
            voltageTimer.reset();
        }
        if (((Double) cachedVoltage).isNaN() || cachedVoltage == 0) {
            cachedVoltage = 12;
        }
        return cachedVoltage;
    }

    /**
     * Initializes the OctoQuad localizer with deadwheel odometry and IMU calibration.
     *
     * <p>This method configures the OctoQuad's built-in absolute localizer for position tracking
     * using deadwheel encoders and the built-in IMU. The localizer provides:</p>
     * <ul>
     *   <li><b>Position (X, Y):</b> Deadwheel odometry in millimeters</li>
     *   <li><b>Heading:</b> IMU-based heading tracking in radians</li>
     *   <li><b>Velocity:</b> X, Y, and angular velocity measurements</li>
     * </ul>
     *
     * <h3>Configuration Process:</h3>
     * <ol>
     *   <li>Set encoder ports for X and Y deadwheel tracking</li>
     *   <li>Configure ticks-per-mm scaling factors</li>
     *   <li>Set TCP (Tracking Center Point) offsets</li>
     *   <li>Reset localizer to (0,0,0) and calibrate IMU</li>
     *   <li>Wait for IMU calibration to complete</li>
     * </ol>
     *
     * <p><b>Important:</b> The robot must remain stationary during IMU calibration.
     * Calibration typically takes 3-5 seconds.</p>
     *
     * <h3>Current Status:</h3>
     * <p>Localizer is currently DISABLED in Constants.java because all 8 OctoQuad ports
     * are used for swerve drive (0-3: steering, 4-7: drive motors). To enable:</p>
     * <ul>
     *   <li>Set {@link org.firstinspires.ftc.teamcode.globals.Constants#LOCALIZER_ENABLED} to true</li>
     *   <li>Configure valid X/Y encoder ports in Constants.java</li>
     *   <li>Ensure those ports are not used by swerve modules</li>
     *   <li>Calibrate ticks-per-mm values</li>
     * </ul>
     *
     * @see org.firstinspires.ftc.teamcode.globals.Constants#LOCALIZER_ENABLED
     * @see org.firstinspires.ftc.teamcode.util.drivers.OctoQuadFWv3#resetLocalizerAndCalibrateIMU()
     */
    private void initializeOctoQuadLocalizer() {
        if (!org.firstinspires.ftc.teamcode.globals.Constants.LOCALIZER_ENABLED) {
            RobotLog.i("OctoQuad Localizer: DISABLED (check Constants.LOCALIZER_ENABLED)");
            return;
        }

        RobotLog.i("OctoQuad Localizer: Initializing...");

        try {
            // Configure encoder ports for X and Y tracking
            int xPort = org.firstinspires.ftc.teamcode.globals.Constants.LOCALIZER_X_ENCODER_PORT;
            int yPort = org.firstinspires.ftc.teamcode.globals.Constants.LOCALIZER_Y_ENCODER_PORT;

            if (xPort >= 0) {
                octoquad.setLocalizerPortX(xPort);
                RobotLog.i("OctoQuad Localizer: X encoder on port " + xPort);
            } else {
                RobotLog.w("OctoQuad Localizer: X encoder port not configured (set to -1)");
            }

            if (yPort >= 0) {
                octoquad.setLocalizerPortY(yPort);
                RobotLog.i("OctoQuad Localizer: Y encoder on port " + yPort);
            } else {
                RobotLog.w("OctoQuad Localizer: Y encoder port not configured (set to -1)");
            }

            // Configure ticks per millimeter scaling factors
            octoquad.setLocalizerCountsPerMM_X(org.firstinspires.ftc.teamcode.globals.Constants.LOCALIZER_TICKS_PER_MM_X);
            octoquad.setLocalizerCountsPerMM_Y(org.firstinspires.ftc.teamcode.globals.Constants.LOCALIZER_TICKS_PER_MM_Y);
            RobotLog.i("OctoQuad Localizer: Ticks/MM - X=" + org.firstinspires.ftc.teamcode.globals.Constants.LOCALIZER_TICKS_PER_MM_X +
                       " Y=" + org.firstinspires.ftc.teamcode.globals.Constants.LOCALIZER_TICKS_PER_MM_Y);

            // Configure TCP (Tracking Center Point) offsets
            octoquad.setLocalizerTcpOffsetMM_X(org.firstinspires.ftc.teamcode.globals.Constants.LOCALIZER_TCP_OFFSET_X_MM);
            octoquad.setLocalizerTcpOffsetMM_Y(org.firstinspires.ftc.teamcode.globals.Constants.LOCALIZER_TCP_OFFSET_Y_MM);
            RobotLog.i("OctoQuad Localizer: TCP Offset - X=" + org.firstinspires.ftc.teamcode.globals.Constants.LOCALIZER_TCP_OFFSET_X_MM +
                       "mm Y=" + org.firstinspires.ftc.teamcode.globals.Constants.LOCALIZER_TCP_OFFSET_Y_MM + "mm");

            // Configure IMU heading scalar
            octoquad.setLocalizerImuHeadingScalar(org.firstinspires.ftc.teamcode.globals.Constants.LOCALIZER_IMU_HEADING_SCALAR);

            // Configure velocity measurement interval
            octoquad.setLocalizerVelocityIntervalMS(org.firstinspires.ftc.teamcode.globals.Constants.LOCALIZER_VELOCITY_INTERVAL_MS);
            RobotLog.i("OctoQuad Localizer: Velocity interval = " + org.firstinspires.ftc.teamcode.globals.Constants.LOCALIZER_VELOCITY_INTERVAL_MS + "ms");

            // Reset localizer and start IMU calibration
            RobotLog.i("OctoQuad Localizer: Starting IMU calibration... (robot must remain stationary)");
            octoquad.resetLocalizerAndCalibrateIMU();

            RobotLog.i("OctoQuad Localizer: Initialization complete. IMU calibration in progress.");

        } catch (Exception e) {
            RobotLog.e("OctoQuad Localizer: Initialization failed - " + e.getMessage());
            e.printStackTrace();
        }
    }

    /**
     * Gets the current localizer status.
     *
     * <p>This method checks the status of the OctoQuad localizer algorithm.</p>
     *
     * @return LocalizerStatus enum value (NOT_INITIALIZED, WARMING_UP_IMU, CALIBRATING_IMU, RUNNING, FAULT_NO_IMU)
     * @see org.firstinspires.ftc.teamcode.util.drivers.OctoQuadFWv3.LocalizerStatus
     */
    public org.firstinspires.ftc.teamcode.util.drivers.OctoQuadFWv3.LocalizerStatus getLocalizerStatus() {
        if (octoquad == null) return org.firstinspires.ftc.teamcode.util.drivers.OctoQuadFWv3.LocalizerStatus.NOT_INITIALIZED;
        return octoquad.getLocalizerStatus();
    }

    /**
     * Reads the current localizer pose data.
     *
     * <p>This method returns the current position, heading, and velocity from the OctoQuad
     * localizer. The data includes CRC validation for reliability.</p>
     *
     * @return LocalizerDataBlock containing position (mm), heading (rad), and velocity data
     * @see org.firstinspires.ftc.teamcode.util.drivers.OctoQuadFWv3.LocalizerDataBlock
     */
    public org.firstinspires.ftc.teamcode.util.drivers.OctoQuadFWv3.LocalizerDataBlock getLocalizerData() {
        if (octoquad == null) return null;
        return octoquad.readLocalizerData();
    }

    /**
     * Gets the current IMU heading from the OctoQuad localizer.
     *
     * <p>This is a convenience method for quickly accessing just the heading value.</p>
     *
     * @return heading in radians, or 0.0 if localizer is not available or data is invalid
     */
    public double getIMUHeading() {
        org.firstinspires.ftc.teamcode.util.drivers.OctoQuadFWv3.LocalizerDataBlock data = getLocalizerData();
        if (data != null && data.isPoseDataValid()) {
            return data.heading_rad;
        }
        return 0.0;
    }

    /**
     * Checks if the OctoQuad localizer is ready and providing valid data.
     *
     * @return true if localizer status is RUNNING and CRC validation passes
     */
    public boolean isLocalizerReady() {
        org.firstinspires.ftc.teamcode.util.drivers.OctoQuadFWv3.LocalizerDataBlock data = getLocalizerData();
        return data != null && data.isPoseDataValid();
    }

    /**
     * Initializes the SRS Hub for analog current monitoring.
     *
     * <p>This method configures the SRS Hub to monitor current draw from a GoBilda Floodgate
     * power switch using one of its analog input ports. The hub provides:</p>
     * <ul>
     *   <li><b>12 Analog/Digital Pins:</b> Configurable as analog (12-bit ADC) or digital I/O</li>
     *   <li><b>6 Encoder Ports:</b> Quadrature or PWM encoder support</li>
     *   <li><b>3 I2C Buses:</b> Daisy-chain for additional sensors</li>
     *   <li><b>Bulk-Read Optimization:</b> All data read in single I2C transaction</li>
     * </ul>
     *
     * <h3>Configuration Process:</h3>
     * <ol>
     *   <li>Create SRS Hub configuration with analog pin enabled</li>
     *   <li>Configure current scaling (0-1 analog to amps)</li>
     *   <li>Initialize hub with 2.5 second calibration delay</li>
     *   <li>Hub performs CRC-16 validation on all reads</li>
     * </ol>
     *
     * <p><b>Current Use:</b> Pin 1 monitors GoBilda Floodgate analog current output.
     * The Floodgate outputs 0-3.3V proportional to current (0V = 0A, 3.3V = MAX_CURRENT_AMPS).</p>
     *
     * @see org.firstinspires.ftc.teamcode.globals.Constants#SRS_HUB_ENABLED
     * @see org.firstinspires.ftc.teamcode.globals.Constants#SRS_HUB_CURRENT_PIN
     * @see org.firstinspires.ftc.teamcode.globals.Constants#SRS_HUB_MAX_CURRENT_AMPS
     */
    private void initializeSRSHub() {
        if (!org.firstinspires.ftc.teamcode.globals.Constants.SRS_HUB_ENABLED) {
            RobotLog.i("SRS Hub: DISABLED (check Constants.SRS_HUB_ENABLED)");
            return;
        }

        RobotLog.i("SRS Hub: Initializing...");

        try {
            // Get SRS Hub from hardware map
            srsHub = hardwareMap.get(SRSHub.class, "srsHub");

            // Create configuration object
            SRSHub.Config config = new SRSHub.Config();

            // Configure analog pin for current monitoring (Pin 1)
            int currentPin = org.firstinspires.ftc.teamcode.globals.Constants.SRS_HUB_CURRENT_PIN;
            config.setAnalogDigitalDevice(currentPin, SRSHub.AnalogDigitalDevice.ANALOG);
            RobotLog.i("SRS Hub: Configured pin " + currentPin + " as ANALOG for current monitoring");

            // Initialize the SRS Hub (takes ~2.5 seconds for calibration)
            srsHub.init(config);

            RobotLog.i("SRS Hub: Initialization complete. Current monitoring active on pin " + currentPin);
            RobotLog.i("SRS Hub: Max current = " + org.firstinspires.ftc.teamcode.globals.Constants.SRS_HUB_MAX_CURRENT_AMPS + "A");

        } catch (Exception e) {
            RobotLog.e("SRS Hub: Initialization failed - " + e.getMessage());
            e.printStackTrace();
            srsHub = null;
        }
    }

    /**
     * Reads the current draw from the GoBilda Floodgate power switch.
     *
     * <p>This method returns the current in amps as measured by the SRS Hub's analog input.
     * The analog value (0-1) is scaled to amps using the MAX_CURRENT_AMPS constant.</p>
     *
     * <p><b>Formula:</b> {@code currentAmps = analogValue * MAX_CURRENT_AMPS}</p>
     *
     * <p><b>Updates:</b> This method automatically calls {@link SRSHub#update()} to read
     * the latest sensor data from the hub before returning the current value.</p>
     *
     * @return current draw in amps, or 0.0 if SRS Hub is not enabled or disconnected
     * @see org.firstinspires.ftc.teamcode.globals.Constants#SRS_HUB_MAX_CURRENT_AMPS
     */
    public double getCurrentDraw() {
        if (srsHub == null || !org.firstinspires.ftc.teamcode.globals.Constants.SRS_HUB_ENABLED) {
            return 0.0;
        }

        try {
            // Update SRS Hub to read latest sensor data
            srsHub.update();

            if (srsHub.disconnected()) {
                return 0.0;
            }

            // Read analog value from current pin and scale to amps
            int currentPin = org.firstinspires.ftc.teamcode.globals.Constants.SRS_HUB_CURRENT_PIN;
            double analogValue = srsHub.readAnalogDigitalDevice(currentPin);
            double currentAmps = analogValue * org.firstinspires.ftc.teamcode.globals.Constants.SRS_HUB_MAX_CURRENT_AMPS;

            // Check for current warning threshold
            double warningThreshold = org.firstinspires.ftc.teamcode.globals.Constants.SRS_HUB_CURRENT_WARNING_THRESHOLD;
            if (warningThreshold > 0 && currentAmps > warningThreshold) {
                RobotLog.w("SRS Hub: High current detected: " + String.format("%.2f", currentAmps) + "A (threshold: " +
                          String.format("%.2f", warningThreshold) + "A)");
            }

            return currentAmps;

        } catch (Exception e) {
            RobotLog.e("SRS Hub: Failed to read current - " + e.getMessage());
            return 0.0;
        }
    }

    /**
     * Main update loop that runs all robot subsystems and clears hardware caches.
     *
     * <p>This method should be called every iteration of the OpMode's loop method.
     * It performs the following operations in order:</p>
     *
     * <ol>
     *   <li>Starts loop timer (if profiling enabled)</li>
     *   <li>Runs the command scheduler, executing all scheduled commands and calling
     *       {@link com.seattlesolvers.solverslib.command.SubsystemBase#periodic()} on all
     *       registered subsystems</li>
     *   <li>Updates telemetry output (throttled to every N loops for performance)</li>
     *   <li>Flushes CSV logger data (throttled to every N loops)</li>
     *   <li>Logs loop time metrics</li>
     *   <li>Clears PhotonCore bulk caches on both hubs</li>
     * </ol>
     *
     * <h3>Performance Optimizations:</h3>
     * <ul>
     *   <li><b>Throttled Telemetry:</b> Updates every {@link #TELEMETRY_UPDATE_INTERVAL} loops
     *       (default: 5 = ~100Hz at 50Hz loop rate) to reduce overhead</li>
     *   <li><b>Throttled Logging:</b> Flushes every {@link #LOGGER_FLUSH_INTERVAL} loops
     *       (default: 10 = ~5Hz) to balance crash recovery with performance</li>
     *   <li><b>Reduced Limelight Polling:</b> Set to 100Hz instead of 250Hz to reduce I2C traffic</li>
     *   <li><b>Loop Profiling:</b> Optional profiling with minimal overhead (~1-2μs)</li>
     * </ul>
     *
     * <p><b>Critical:</b> The bulk cache clear at the end of this method is essential for
     * PhotonCore's manual caching mode. Without it, hardware reads will return stale data.</p>
     *
     * @param telemetryData the telemetry instance for sending data to driver station
     * @see PhotonCore#clearBulkCache()
     * @see com.seattlesolvers.solverslib.command.CommandScheduler#run()
     */
    public void updateLoop(TelemetryData telemetryData) {
        if (ENABLE_LOOP_PROFILING) {
            loopTimer.reset();
        }

        // Run Scheduled Robot Commands and Subsystems periodically
        CommandScheduler.getInstance().run();

        // Throttled telemetry update (every N loops instead of every loop)
        if (this.telemetry != null && (loopCounter % TELEMETRY_UPDATE_INTERVAL == 0)) {
            this.telemetry.update();
        }

        // Throttled logger flush (every N loops instead of every loop)
        if (this.logger != null && (loopCounter % LOGGER_FLUSH_INTERVAL == 0)) {
            this.logger.update();
        }

        // Log loop time metrics (if profiling enabled)
        if (ENABLE_LOOP_PROFILING && this.logger != null) {
            lastLoopTimeMs = loopTimer.milliseconds();

            // Update rolling average (exponential moving average with alpha=0.1)
            if (loopCounter == 0) {
                averageLoopTimeMs = lastLoopTimeMs;
            } else {
                averageLoopTimeMs = 0.1 * lastLoopTimeMs + 0.9 * averageLoopTimeMs;
            }

            // Log loop times every 10 loops
            if (loopCounter % 10 == 0) {
                this.logger.addData("Loop Time (ms)", lastLoopTimeMs);
                this.logger.addData("Avg Loop Time (ms)", averageLoopTimeMs);
            }
        }

        loopCounter++;

        // Always clear caches at the end of the run loop!
        PhotonCore.CONTROL_HUB.clearBulkCache();
        PhotonCore.EXPANSION_HUB.clearBulkCache();
    }

    /**
     * Gets the last measured loop execution time.
     *
     * @return loop time in milliseconds
     */
    public double getLastLoopTime() {
        return lastLoopTimeMs;
    }

    /**
     * Gets the rolling average loop time.
     *
     * @return average loop time in milliseconds (last 10 loops)
     */
    public double getAverageLoopTime() {
        return averageLoopTimeMs;
    }
}