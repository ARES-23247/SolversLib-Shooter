package org.firstinspires.ftc.teamcode.commandbase.subsystems;

import com.seattlesolvers.solverslib.command.SubsystemBase;
import org.firstinspires.ftc.teamcode.globals.Robot;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.swerve.OctoSwerveDrivetrainV2;
import org.firstinspires.ftc.teamcode.util.drivers.OctoQuadFWv3;
import org.firstinspires.ftc.teamcode.util.localization.SensorFusionLocalizer;
import org.firstinspires.ftc.teamcode.util.localization.SwerveOdometry;
import org.firstinspires.ftc.teamcode.util.monitoring.PinpointHealthMonitor;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.seattlesolvers.solverslib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.hardware.motors.CRServoEx;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.dashboard.canvas.Canvas;

import static org.firstinspires.ftc.teamcode.globals.Constants.*;

/**
 * Main drive subsystem that integrates swerve drivetrain control with PedroPathing autonomous navigation.
 *
 * <p>This subsystem serves as the primary interface for all robot motion control. It combines
 * two key components:</p>
 *
 * <ul>
 *   <li><b>PedroPathing Follower:</b> Handles autonomous path following, pose estimation,
 *       and field-centric navigation during autonomous</li>
 *   <li><b>OctoSwerveDrivetrainV2:</b> Manages the physical swerve drive modules with flexible
 *       encoder support (OctoQuad digital or SRS Hub analog)</li>
 * </ul>
 *
 * <h3>Encoder Flexibility:</h3>
 * <p>This drivetrain supports two encoder types, selectable via {@link org.firstinspires.ftc.teamcode.globals.Constants#SWERVE_ENCODER_TYPE}:</p>
 * <ul>
 *   <li><b>OctoQuad (Default):</b> High-precision digital encoders with 8192 CPR</li>
 *   <li><b>Analog (SRS Hub):</b> Potentiometers or magnetic encoders via SRS Hub analog inputs</li>
 * </ul>
 * <p>Switch between encoder types by changing one constant - no code changes needed!</p>
 *
 * <h3>IMU Configuration:</h3>
 * <p>REV Hub IMU is DISABLED to reduce I2C bus traffic and improve loop times.</p>
 * <ul>
 *   <li><b>Disabled:</b> REV Hub IMU (Control Hub/Expansion Hub built-in IMU)</li>
 *   <li><b>Primary Heading Source:</b> GoBilda Pinpoint IMU (used for PedroPathing odometry)</li>
 *   <li><b>Optional Heading Source:</b> OctoQuad IMU (when LOCALIZER_ENABLED is true)</li>
 * </ul>
 *
 * <p><b>Performance:</b> By disabling the REV Hub IMU and using only the GoBilda Pinpoint IMU,
 * we reduce I2C bus contention and improve loop times. The Pinpoint IMU is accessed via its
 * own I2C bus and provides reliable heading data.</p>
 *
 * <h3>Sensor Fusion:</h3>
 * <p>When enabled, this subsystem uses an Extended Kalman Filter (EKF) to fuse three localization
 * sources for maximum accuracy:</p>
 * <ol>
 *   <li><b>Swerve Odometry:</b> High-frequency updates from drive wheel encoders</li>
 *   <li><b>Pinpoint Odometry:</b> High-accuracy translation from deadwheel encoders</li>
 *   <li><b>Limelight Vision:</b> Global position correction from AprilTags</li>
 * </ol>
 *
 * <h3>Current Management:</h3>
 * <p>Includes slew rate limiting and current limiting to prevent 20A fuse trips:</p>
 * <ul>
 *   <li><b>Slew Rate Limiting:</b> Prevents rapid power changes that cause current spikes</li>
 *   <li><b>Current Limiting:</b> Scales motor power when Floodgage sensor detects high current</li>
 * </ul>
 *
 * <h3>Operation Modes:</h3>
 * <p>The drive subsystem supports three operation modes:</p>
 * <ol>
 *   <li><b>Field-Centric TeleOp:</b> Driver control relative to the field. Robot movement
 *       stays consistent regardless of robot heading. Best for game piece manipulation.</li>
 *   <li><b>Robot-Centric TeleOp:</b> Driver control relative to the robot. Forward always
 *       moves the robot in the direction it's facing. Best for fine positioning.</li>
 *   <li><b>Autonomous Path Following:</b> Automatic navigation along pre-generated paths
 *       using PedroPathing trajectory following.</li>
 * </ol>
 *
 * <h3>Coordinate Systems:</h3>
 * <ul>
 *   <li><b>Field Coordinates:</b> X = forward/back (inches), Y = left/right (inches),
 *       Heading = rotation (radians, 0 = facing red alliance wall)</li>
 *   <li><b>Chassis Speeds:</b> VX = forward velocity, VY = lateral velocity,
 *       Omega = rotational velocity (radians/sec)</li>
 * </ul>
 *
 * <h3>Telemetry & Logging:</h3>
 * <p>This subsystem provides comprehensive telemetry output:</p>
 * <ul>
 *   <li>FTC Dashboard field overlay with robot position and wheel orientations</li>
 *   <li>Encoder type display (OctoQuad or Analog)</li>
 *   <li>CSV logging of pose, velocity, and module states for post-match analysis</li>
 *   <li>Real-time target vs actual motor commands for tuning</li>
 * </ul>
 *
 * @see com.pedropathing.follower.Follower
 * @see org.firstinspires.ftc.teamcode.commandbase.subsystems.swerve.OctoSwerveDrivetrainV2
 * @see org.firstinspires.ftc.teamcode.commandbase.subsystems.swerve.encoders.SwerveEncoder
 * @see <a href="https://pedropathing.com/ Pedro Pathing Documentation</a>
 */
public class Drive extends SubsystemBase {

    /**
     * Reference to the robot singleton for accessing hardware and telemetry.
     */
    private final Robot robot = Robot.getInstance();

    /**
     * The PedroPathing follower instance for autonomous navigation.
     * Handles path following, pose estimation, and localization.
     * Integrates with odometry (Pinpoint) and vision (Limelight) for pose updates.
     */
    public final Follower follower;

    /**
     * The physical swerve drivetrain controller.
     * Manages four swerve modules with PIDF control for steering and drive motors.
     * Translates chassis speeds into individual module commands via inverse kinematics.
     *
     * <p>Supports both OctoQuad digital encoders and SRS Hub analog encoders.
     * Switch via {@link org.firstinspires.ftc.teamcode.globals.Constants#SWERVE_ENCODER_TYPE}.</p>
     */
    public final OctoSwerveDrivetrainV2 swerve;

    /**
     * Sensor fusion localizer for combining Pinpoint, swerve odometry, and Limelight.
     * Uses Extended Kalman Filter (EKF) for optimal pose estimation.
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
     * Tracks whether vision tag was visible in the previous loop.
     * Used for Pinpoint auto-reset logic.
     */
    private boolean visionTagWasVisible = false;

    /**
     * Cached TeleOp chassis speeds for current limiting in periodic().
     * Stores the most recent driver input from setTeleOpDrive().
     */
    private ChassisSpeeds teleOpSpeeds = new ChassisSpeeds(0, 0, 0);

    /**
     * Whether TeleOp drive is active (true) or autonomous is active (false).
     * Used to determine which chassis speeds to apply in periodic().
     */
    private boolean isTeleOpMode = false;

    /**
     * Tracks loop timing for performance monitoring.
     */
    private double loopStartTime = 0.0;
    private double lastLoopTime = 0.0;

    /**
     * Tracks whether current limiting is actively reducing motor power.
     */
    private boolean currentLimitingActive = false;

    /**
     * Tracks battery voltage for power monitoring.
     */
    private double batteryVoltage = 13.0;  // Default estimate

    /**
     * Loop counter for voltage caching (read voltage every N loops).
     */
    private int voltageLoopCounter = 0;

    /**
     * Loop counter for vision update throttling (update vision every N loops).
     */
    private int visionLoopCounter = 0;

    /**
     * Constructs the Drive subsystem and initializes all components.
     *
     * <p>This constructor:</p>
     * <ol>
     *   <li>Creates the PedroPathing follower with configured constants</li>
     *   <li>Initializes the OctoSwerveDrivetrain with motors, servos, and PIDF values</li>
     *   <li>Configures the drivetrain with robot physical dimensions</li>
     * </ol>
     *
     * <p><b>Hardware Requirements:</b> Requires the following to be initialized in Robot:
     * <ul>
     *   <li>4x drive motors (MotorEx)</li>
     *   <li>4x steering servos (CRServoEx)</li>
     *   <li>1x OctoQuad encoder processor</li>
     * </ul>
     * </p>
     */
    public Drive() {
        // Initialize the Pedro Pathing Engine
        follower = Constants.createFollower(robot.hardwareMap);

        // Initialize the Custom Coaxial Swerve Engine with OctoQuad or Analog encoders
        swerve = new OctoSwerveDrivetrainV2(
            TRACK_WIDTH,
            WHEEL_BASE,
            MAX_DRIVE_VELOCITY,
            SWERVE_SERVO_PIDF,
            SWERVE_DRIVE_PIDF,
            new MotorEx[] { robot.frontRightMotor, robot.frontLeftMotor, robot.backLeftMotor, robot.backRightMotor },
            new CRServoEx[] { robot.frontRightServo, robot.frontLeftServo, robot.backLeftServo, robot.backRightServo },
            robot.octoquad,
            robot.srsHub,  // Pass SRS Hub for analog encoder support
            new double[] { FRONT_RIGHT_OFFSET, FRONT_LEFT_OFFSET, BACK_LEFT_OFFSET, BACK_RIGHT_OFFSET }
        );

        // Initialize swerve odometry for drive wheel encoder localization
        swerveOdometry = new SwerveOdometry(0, 0, 0);

        // Initialize Pinpoint health monitor with constants
        pinpointHealthMonitor = new PinpointHealthMonitor(
            PINPOINT_HEALTH_DEVIATION_THRESHOLD,
            PINPOINT_HEALTH_CONSECUTIVE_BAD_READINGS
        );

        // Initialize sensor fusion EKF with noise parameters
        sensorFusion = new SensorFusionLocalizer();
        configureSensorFusionNoise();
    }

    /**
     * Initializes the drive subsystem for operation.
     *
     * <p>This method must be called before the drive subsystem is used. It performs
     * PedroPathing initialization and sets the robot's starting pose.</p>
     *
     * <p><b>Starting Pose:</b> Defaults to (0, 0, 0) which represents:
     * <ul>
     *   <li>X = 0 inches (starting position)</li>
     *   <li>Y = 0 inches (starting position)</li>
     *   <li>Heading = 0 radians (facing the red alliance wall)</li>
     * </ul>
     * </p>
     *
     * <p><b>Custom Starting Pose:</b> To start at a different position (e.g., for
     * autonomous from different starting tiles), call:
     * <pre>{@code
     * follower.setStartingPose(new Pose(startX, startY, startHeading));
     * }</pre>
     * </p>
     */
    public void init() {
        // Required PedroPathing follower initialization steps
        // Assuming robot starts at 0, 0, 0
        follower.setStartingPose(new Pose(0, 0, 0));
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
            FUSION_PROCESS_NOISE_HEADING,     // Heading uncertainty (radians)
            FUSION_PROCESS_NOISE_VELOCITY     // Velocity uncertainty (inches/sec)
        };

        // Swerve odometry noise: moderate uncertainty (wheel slip)
        double[] swerveNoise = {
            FUSION_SWERVE_NOISE_POSITION,     // Position uncertainty (inches)
            FUSION_SWERVE_NOISE_HEADING       // Heading uncertainty (radians)
        };

        // Pinpoint noise: low uncertainty (deadwheels are accurate)
        double[] pinpointNoise = {
            FUSION_PINPOINT_NOISE_POSITION,   // Position uncertainty (inches)
            FUSION_PINPOINT_NOISE_HEADING     // Heading uncertainty (radians)
        };

        // Vision noise: very low uncertainty (AprilTags are globally accurate)
        double[] visionNoise = {
            FUSION_VISION_NOISE_POSITION,     // Position uncertainty (inches)
            FUSION_VISION_NOISE_HEADING       // Heading uncertainty (radians)
        };

        sensorFusion.setNoiseParameters(processNoise, swerveNoise, pinpointNoise, visionNoise);
    }

    /**
     * Gets the sensor fusion localizer.
     *
     * <p>This provides access to the EKF for external subsystems (like Vision)
     * to apply pose corrections.</p>
     *
     * @return the sensor fusion localizer
     */
    public SensorFusionLocalizer getSensorFusion() {
        return sensorFusion;
    }

    /**
     * Gets the swerve odometry tracker.
     *
     * @return the swerve odometry tracker
     */
    public SwerveOdometry getSwerveOdometry() {
        return swerveOdometry;
    }

    /**
     * Gets the current TeleOp chassis speeds.
     *
     * @return the current TeleOp chassis speeds
     */
    public ChassisSpeeds getTeleOpSpeeds() {
        return teleOpSpeeds;
    }

    /**
     * Sets the drivetrain velocities for manual TeleOp control.
     *
     * <p>This method accepts driver input and converts it into chassis speeds for the
     * swerve drivetrain. Supports both field-centric and robot-centric driving modes.</p>
     *
     * <h3>Field-Centric Drive:</h3>
     * <p>When {@code isFieldCentric = true}, the driver's input is interpreted relative to
     * the field rather than the robot. This provides intuitive control:</p>
 * <ul>
     *   <li>Forward always moves the robot away from the driver</li>
     *   <li>Left always moves the robot to the driver's left</li>
     *   <li>Turning rotates the robot in place</li>
     * </ul>
     *
     * <p><b>Math:</b> Field-centric drive applies a rotation matrix to the input based on
     * the robot's current heading from the PedroPathing localizer:
     * <pre>
     * rotX = forward * cos(-heading) - lateral * sin(-heading)
     * rotY = forward * sin(-heading) + lateral * cos(-heading)
     * </pre>
     * </p>
     *
     * <h3>Robot-Centric Drive:</h3>
     * <p>When {@code isFieldCentric = false}, the driver's input is interpreted relative
     * to the robot's current orientation:</p>
     * <ul>
     *   <li>Forward moves in the direction the robot is facing</li>
     *   <li>Left moves perpendicular to the robot's heading</li>
     *   <li>Turn rotates the robot</li>
     * </ul>
     *
     * <h3>Current Limiting:</h3>
     * <p>The chassis speeds are stored and applied with current limiting in the periodic()
     * loop. This ensures current limiting is applied consistently in both TeleOp and auto modes.</p>
     *
     * <h3>Input Range:</h3>
     * <p>All inputs should be in the range [-1.0, 1.0]. The method will scale these to
     * the maximum drive velocity configured in Constants.</p>
     *
     * @param forward forward/backward velocity [-1.0, 1.0]. Positive = forward.
     * @param lateral left/right velocity [-1.0, 1.0]. Positive = left.
     * @param turn rotational velocity [-1.0, 1.0]. Positive = counter-clockwise.
     * @param isFieldCentric if true, input is field-centric; if false, input is robot-centric
     * @see ChassisSpeeds
     * @see OctoSwerveDrivetrain#drive(ChassisSpeeds, double)
     */
    public void setTeleOpDrive(double forward, double lateral, double turn, boolean isFieldCentric) {
        isTeleOpMode = true;  // Enable TeleOp mode

        if (isFieldCentric) {
            // Retrieve current heading from PedroPathing's Localizer
            double currentHeading = follower.getPoseTracker().getTotalHeading();

            // Apply field centric rotation
            double rotX = forward * Math.cos(-currentHeading) - lateral * Math.sin(-currentHeading);
            double rotY = forward * Math.sin(-currentHeading) + lateral * Math.cos(-currentHeading);

            // Store the chassis speeds for current limiting in periodic()
            teleOpSpeeds = new ChassisSpeeds(rotX, rotY, turn);
        } else {
            // Robot Centric
            teleOpSpeeds = new ChassisSpeeds(forward, lateral, turn);
        }
    }

    /**
     * Periodic update method called every loop iteration by the command scheduler.
     *
     * <p>This method is the main update loop for the drive subsystem. It performs the
     * sensor fusion EKF cycle and applies drive commands:</p>
     *
     * <h3>1. Sensor Fusion EKF Cycle</h3>
     * <p>Every loop iteration, the EKF fuses three localization sources:</p>
     * <ul>
     *   <li><b>Predict:</b> Use swerve odometry for high-frequency pose prediction</li>
     *   <li><b>Correct (Pinpoint):</b> Apply deadwheel odometry corrections</li>
     *   <li><b>Correct (Limelight):</b> Applied by Vision subsystem when tags are visible</li>
     * </ul>
     *
     * <h3>2. Update PedroPathing with Fused Pose</h3>
     * <p>The fused pose estimate from the EKF is used to update PedroPathing's localizer,
     * ensuring autonomous paths use the best available pose estimate.</p>
     *
     * <h3>3. Drive Command Execution</h3>
     * <p>This method applies either TeleOp or autonomous chassis speeds with current limiting:</p>
     * <ul>
     *   <li><b>Autonomous:</b> If PedroPathing follower is busy, extract desired velocities
     *       from the path follower and apply them with current limiting</li>
     *   <li><b>TeleOp:</b> If in TeleOp mode, apply the stored driver input with current limiting</li>
     * </ul>
     *
     * <p>Current limiting reads from the SRS Hub Floodgate sensor and scales chassis speeds
     * when approaching the 20A fuse limit to prevent trips.</p>
     *
     * <h3>4. Telemetry & Visualization</h3>
     * <p>Sends telemetry data to FTC Dashboard and CSV logger:</p>
     * <ul>
     *   <li><b>Field Overlay:</b> Draws robot box (18x18 inches), heading line, and
     *       wheel orientation indicators on the field</li>
     *   <li><b>Telemetry:</b> Sends target vs actual module angles and velocities</li>
     *   <li><b>CSV Logging:</b> Records robot pose, heading, and module states for
     *       post-match analysis</li>
     * </ul>
     *
     * <h3>Visualization Details:</h3>
     * <ul>
     *   <li><b>Robot Box:</b> Blue rectangle (#3F51B5) showing robot footprint</li>
     *   <li><b>Heading Line:</b> 9-inch line from robot center showing forward direction</li>
     *   <li><b>Wheel Indicators:</b> Orange 2.5-inch lines showing each wheel's actual
     *       steering angle (updates in real-time for tuning)</li>
     * </ul>
     *
     * <p><b>Performance Note:</b> This method runs approximately 50 times per second.
     * Keep operations efficient to maintain loop timing under 20ms.</p>
     */
    @Override
    public void periodic() {
        // ===== Loop Timing =====
        loopStartTime = System.nanoTime() / 1e9;  // Convert to seconds

        // Read battery voltage (cached for performance)
        voltageLoopCounter++;
        if (voltageLoopCounter >= org.firstinspires.ftc.teamcode.globals.Constants.VOLTAGE_CACHE_DURATION) {
            if (robot.voltageSensor != null) {
                batteryVoltage = robot.voltageSensor.getVoltage();
                if (Double.isNaN(batteryVoltage) || batteryVoltage == 0) {
                    batteryVoltage = 13.0;  // Default if sensor fails
                }
            }
            voltageLoopCounter = 0;
        }

        // ===== 1. Sensor Fusion EKF Cycle =====

        // Get current chassis speeds for EKF prediction step
        ChassisSpeeds currentSpeeds = isTeleOpMode ? teleOpSpeeds :
            (follower.isBusy() ? new ChassisSpeeds(
                follower.getPose().getX(),
                follower.getPose().getY(),
                follower.getPose().getHeading()
            ) : new ChassisSpeeds(0, 0, 0));

        // Predict with swerve odometry (high frequency)
        swerveOdometry.update(currentSpeeds);
        sensorFusion.predictWithSwerve(currentSpeeds);

        // Correct with Pinpoint odometry (high accuracy) - only if healthy
        if (robot.pinpoint != null) {
            robot.pinpoint.update();
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
                                  (!ENABLE_PINPOINT_HEALTH_MONITOR ||
                                  pinpointHealthMonitor.isHealthy(
                                      pinpointX, pinpointY,
                                      sensorFusion.getEstimatedPose().getX(),
                                      sensorFusion.getEstimatedPose().getY()
                                  ));

            if (usePinpoint) {
                sensorFusion.correctWithPinpoint(pinpointX, pinpointY, pinpointHeading);
            }
            // Else: Pinpoint is drifted or out of bounds, skip correction (rely on swerve + vision)
        }

        // Get fused pose estimate
        Pose fusedPose = sensorFusion.getEstimatedPose();

        // ===== Pinpoint Auto-Reset Logic =====
        // If Pinpoint is drifted and conditions are safe, auto-reset to vision pose
        if (ENABLE_PINPOINT_HEALTH_MONITOR && robot.pinpoint != null && !pinpointHealthMonitor.isHealthy()) {
            // Check if safe to reset: robot stationary + good vision
            ChassisSpeeds estimatedVel = sensorFusion.getEstimatedVelocity();
            double robotVelocity = Math.hypot(estimatedVel.vxMetersPerSecond, estimatedVel.vyMetersPerSecond);
            double[] posUncertainty = sensorFusion.getPositionUncertainty();
            double maxUncertainty = Math.max(posUncertainty[0], posUncertainty[1]);
            boolean visionTagVisible = (robot.vision != null && robot.vision.isTagVisible());

            boolean safeToReset = pinpointHealthMonitor.isSafeToReset(
                robotVelocity,
                maxUncertainty,
                visionTagVisible,
                PINPOINT_AUTO_RESET_MAX_VELOCITY,
                PINPOINT_AUTO_RESET_MAX_UNCERTAINTY
            );

            if (safeToReset) {
                // Reset Pinpoint to fused pose
                robot.pinpoint.resetPosAndIMU();
                // Pinpoint will start from (0,0,0), so we need to manually set position
                // Unfortunately, Pinpoint doesn't have a setPosition method, so we reset
                // and let it re-integrate from here. The vision correction in the EKF
                // will keep the fused pose accurate.

                // Reset health monitor to healthy state
                pinpointHealthMonitor.reset();

                // Log the reset event
                if (robot.telemetry != null) {
                    System.out.println("Pinpoint Auto-Reset: Stationary + Good Vision");
                }
            }
        }

        // Update PedroPathing localizer with fused pose
        follower.setPose(fusedPose);

        // Update PedroPathing odometry internally
        follower.update();

        // ===== 2. Drive Command Execution =====

        // Read current draw from SRS Hub (if available)
        double currentDraw = 0.0;
        boolean srsHubReady = org.firstinspires.ftc.teamcode.globals.Constants.SRS_HUB_ENABLED &&
                              robot.srsHub != null;
        if (srsHubReady) {
            currentDraw = robot.getCurrentDraw();
        }

        // Apply drive commands with current limiting
        if (follower.isBusy()) {
            // Autonomous mode: Extract Pedro's desired velocities and apply with current limiting
            Pose drivePower = follower.getPose();
            // Pedro Pose power: x component, y component, heading component
            swerve.drive(new ChassisSpeeds(drivePower.getX(), drivePower.getY(), drivePower.getHeading()), currentDraw);
            isTeleOpMode = false;  // Auto mode takes precedence
        } else if (isTeleOpMode) {
            // TeleOp mode: Apply stored driver input with current limiting
            swerve.drive(teleOpSpeeds, currentDraw);
        }

        // Track current limiting state
        currentLimitingActive = swerve.isCurrentLimitingActive();

        // Get current pose once (avoid redundant calls)
        Pose currentPose = follower.getPoseTracker().getPose();

        // Read OctoQuad localizer data once per loop (avoid redundant I2C reads)
        OctoQuadFWv3.LocalizerDataBlock localizerData = null;
        boolean localizerReady = org.firstinspires.ftc.teamcode.globals.Constants.LOCALIZER_ENABLED &&
                                  robot.isLocalizerReady();
        if (localizerReady) {
            localizerData = robot.getLocalizerData();
        }

        // Dashboard Telemetry & Field Drawing (only if enabled in Constants)
        if (robot.telemetry != null && org.firstinspires.ftc.teamcode.globals.Constants.ENABLE_DASHBOARD_OVERLAY) {
            TelemetryPacket packet = new TelemetryPacket();
            Canvas field = packet.fieldOverlay();

            // OctoQuad Localizer Telemetry (if enabled and ready)
            if (localizerReady && localizerData != null) {
                packet.put("OctoQuad Status", localizerData.localizerStatus.toString());
                packet.put("OctoQuad X (mm)", localizerData.posX_mm);
                packet.put("OctoQuad Y (mm)", localizerData.posY_mm);
                packet.put("OctoQuad Heading (deg)", Math.toDegrees(localizerData.heading_rad));
                packet.put("OctoQuad VelX (mm/s)", localizerData.velX_mmS);
                packet.put("OctoQuad VelY (mm/s)", localizerData.velY_mmS);
                packet.put("OctoQuad CRC Valid", localizerData.crcOk);
            } else if (org.firstinspires.ftc.teamcode.globals.Constants.LOCALIZER_ENABLED) {
                packet.put("OctoQuad Status", robot.getLocalizerStatus().toString());
            }

            // SRS Hub Current Telemetry (if enabled)
            if (srsHubReady) {
                packet.put("Current Draw (A)", String.format("%.2f", currentDraw));
                packet.put("SRS Hub Status", robot.srsHub.disconnected() ? "DISCONNECTED" : "OK");
                packet.put("Current Limiting Active", currentLimitingActive ? "YES" : "NO");
                packet.put("Current Limit Scale", String.format("%.2f", swerve.getCurrentLimitScale()));
            }

            // Power and Performance Telemetry
            packet.put("Battery Voltage (V)", String.format("%.2f", batteryVoltage));
            packet.put("Loop Time (ms)", String.format("%.2f", lastLoopTime * 1000));
            packet.put("Loop Frequency (Hz)", String.format("%.1f", 1.0 / Math.max(lastLoopTime, 0.001)));

            // Encoder Type Telemetry
            packet.put("Swerve Encoder Type", swerve.getEncoderType());

            // Controller Input Telemetry
            packet.put("Input Forward", String.format("%.3f", teleOpSpeeds.vxMetersPerSecond));
            packet.put("Input Lateral", String.format("%.3f", teleOpSpeeds.vyMetersPerSecond));
            packet.put("Input Turn", String.format("%.3f", teleOpSpeeds.omegaRadiansPerSecond));
            packet.put("Input Mode", isTeleOpMode ? "TeleOp" : "Auto");

            // Sensor Fusion Telemetry (if enabled)
            if (ENABLE_SENSOR_FUSION && sensorFusion != null) {
                fusedPose = sensorFusion.getEstimatedPose();
                double[] posUncertainty = sensorFusion.getPositionUncertainty();
                double headingUncertainty = sensorFusion.getHeadingUncertainty();

                packet.put("Fusion X (in)", String.format("%.2f", fusedPose.getX()));
                packet.put("Fusion Y (in)", String.format("%.2f", fusedPose.getY()));
                packet.put("Fusion Heading (deg)", String.format("%.2f", Math.toDegrees(fusedPose.getHeading())));
                packet.put("Fusion X Uncertainty (in)", String.format("%.3f", posUncertainty[0]));
                packet.put("Fusion Y Uncertainty (in)", String.format("%.3f", posUncertainty[1]));
                packet.put("Fusion Heading Uncertainty (deg)", String.format("%.3f", Math.toDegrees(headingUncertainty)));

                // Swerve odometry telemetry
                packet.put("Swerve Odometry X (in)", String.format("%.2f", swerveOdometry.getX()));
                packet.put("Swerve Odometry Y (in)", String.format("%.2f", swerveOdometry.getY()));
                packet.put("Swerve Odometry Heading (deg)", String.format("%.2f", Math.toDegrees(swerveOdometry.getHeading())));

                // Pinpoint odometry telemetry
                if (robot.pinpoint != null) {
                    packet.put("Pinpoint X (in)", String.format("%.2f", robot.pinpoint.getPosition().getX(org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.INCH)));
                    packet.put("Pinpoint Y (in)", String.format("%.2f", robot.pinpoint.getPosition().getY(org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.INCH)));
                    packet.put("Pinpoint Heading (deg)", String.format("%.2f", robot.pinpoint.getHeading(org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.RADIANS)));

                    // Check if Pinpoint is out of bounds
                    if (ENABLE_PINPOINT_BOUNDARY_CHECK) {
                        boolean outOfBounds = !isWithinFieldBounds(robot.pinpoint.getPosition().getX(org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.INCH), robot.pinpoint.getPosition().getY(org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.INCH));
                        packet.put("Pinpoint Out of Bounds", outOfBounds ? "YES" : "NO");
                    }

                    if (ENABLE_PINPOINT_HEALTH_MONITOR) {
                        String healthStatus = pinpointHealthMonitor.isHealthy() ? "YES" : "NO (DRIFTED)";
                        packet.put("Pinpoint Healthy", healthStatus);
                        packet.put("Pinpoint Bad Count", pinpointHealthMonitor.getBadReadingCount());

                        if (ENABLE_PINPOINT_AUTO_RESET && !pinpointHealthMonitor.isHealthy()) {
                            ChassisSpeeds estimatedVel = sensorFusion.getEstimatedVelocity();
                            double robotVelocity = Math.hypot(estimatedVel.vxMetersPerSecond, estimatedVel.vyMetersPerSecond);
                            posUncertainty = sensorFusion.getPositionUncertainty();
                            double maxUncertainty = Math.max(posUncertainty[0], posUncertainty[1]);
                            boolean visionTagVisible = (robot.vision != null && robot.vision.isTagVisible());

                            boolean safeToReset = pinpointHealthMonitor.isSafeToReset(
                                robotVelocity, maxUncertainty, visionTagVisible,
                                PINPOINT_AUTO_RESET_MAX_VELOCITY, PINPOINT_AUTO_RESET_MAX_UNCERTAINTY
                            );

                            packet.put("Pinpoint Can Auto-Reset", safeToReset ? "YES" : "NO");
                            packet.put("Reset Check Velocity", String.format("%.2f in/s", robotVelocity));
                            packet.put("Reset Check Uncertainty", String.format("%.2f in", maxUncertainty));
                            packet.put("Reset Check Vision", visionTagVisible ? "YES" : "NO");
                        }
                    }
                }
            }

            // Draw Robot Box (18x18 assumed default)
            field.setStroke("#3F51B5");
            field.strokeRect(currentPose.getX() - 9, currentPose.getY() - 9, 18, 18);

            // Draw Heading Line
            field.strokeLine(
                currentPose.getX(), currentPose.getY(),
                currentPose.getX() + 9 * Math.cos(currentPose.getHeading()),
                currentPose.getY() + 9 * Math.sin(currentPose.getHeading())
            );

            // Fetch Swerve Kinematics Target/Actual Logs
            swerve.drawTelemetry(packet, currentPose);

            // Send packet immediately to Dashboard HTML websocket
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
        }

        // CSV Logging
        if (robot.logger != null) {
            robot.logger.addData("Swerve Encoder Type", swerve.getEncoderType());
            robot.logger.addData("Robot X (Inches)", currentPose.getX());
            robot.logger.addData("Robot Y (Inches)", currentPose.getY());
            robot.logger.addData("Robot Heading (Rad)", currentPose.getHeading());

            // Log controller inputs (TeleOp or Auto)
            robot.logger.addData("Input Forward", teleOpSpeeds.vxMetersPerSecond);
            robot.logger.addData("Input Lateral", teleOpSpeeds.vyMetersPerSecond);
            robot.logger.addData("Input Turn", teleOpSpeeds.omegaRadiansPerSecond);
            robot.logger.addData("Input Mode", isTeleOpMode ? "TeleOp" : "Auto");

            // Log OctoQuad Localizer data (if available) - using cached data to avoid redundant I2C reads
            if (localizerReady && localizerData != null) {
                robot.logger.addData("OctoQuad X (mm)", localizerData.posX_mm);
                robot.logger.addData("OctoQuad Y (mm)", localizerData.posY_mm);
                robot.logger.addData("OctoQuad Heading (rad)", localizerData.heading_rad);
                robot.logger.addData("OctoQuad VelX (mm/s)", localizerData.velX_mmS);
                robot.logger.addData("OctoQuad VelY (mm/s)", localizerData.velY_mmS);
                robot.logger.addData("OctoQuad IMU Status", localizerData.localizerStatus.toString());
                robot.logger.addData("OctoQuad CRC Valid", localizerData.crcOk);
            }

            // Log SRS Hub current data (if available) - using cached data to avoid redundant I2C reads
            if (srsHubReady) {
                robot.logger.addData("Current Draw (A)", currentDraw);
                robot.logger.addData("SRS Hub Status", robot.srsHub.disconnected() ? "DISCONNECTED" : "OK");
            }

            // Log power and performance data
            robot.logger.addData("Battery Voltage (V)", batteryVoltage);
            robot.logger.addData("Loop Time (ms)", lastLoopTime * 1000);
            robot.logger.addData("Current Limiting Active", currentLimitingActive ? 1 : 0);
            robot.logger.addData("Current Limit Scale", swerve.getCurrentLimitScale());

            // Log Sensor Fusion data (if enabled)
            if (ENABLE_SENSOR_FUSION && sensorFusion != null) {
                fusedPose = sensorFusion.getEstimatedPose();
                double[] posUncertainty = sensorFusion.getPositionUncertainty();
                double headingUncertainty = sensorFusion.getHeadingUncertainty();

                robot.logger.addData("Fusion X (Inches)", fusedPose.getX());
                robot.logger.addData("Fusion Y (Inches)", fusedPose.getY());
                robot.logger.addData("Fusion Heading (Rad)", fusedPose.getHeading());
                robot.logger.addData("Fusion X Uncertainty", posUncertainty[0]);
                robot.logger.addData("Fusion Y Uncertainty", posUncertainty[1]);
                robot.logger.addData("Fusion Heading Uncertainty", headingUncertainty);

                // Log swerve odometry
                robot.logger.addData("Swerve Odometry X", swerveOdometry.getX());
                robot.logger.addData("Swerve Odometry Y", swerveOdometry.getY());
                robot.logger.addData("Swerve Odometry Heading", swerveOdometry.getHeading());

                // Log Pinpoint odometry
                if (robot.pinpoint != null) {
                    robot.logger.addData("Pinpoint X", robot.pinpoint.getPosition().getX(org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.INCH));
                    robot.logger.addData("Pinpoint Y", robot.pinpoint.getPosition().getY(org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.INCH));
                    robot.logger.addData("Pinpoint Heading", robot.pinpoint.getHeading(org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.RADIANS));

                    if (ENABLE_PINPOINT_HEALTH_MONITOR) {
                        robot.logger.addData("Pinpoint Healthy", pinpointHealthMonitor.isHealthy() ? 1 : 0);
                        robot.logger.addData("Pinpoint Bad Count", pinpointHealthMonitor.getBadReadingCount());
                    }
                }
            }

            swerve.logData(robot.logger);
        }

        // Calculate loop time at the end of periodic
        lastLoopTime = (System.nanoTime() / 1e9) - loopStartTime;
    }

    /**
     * Checks if a pose is within valid field boundaries.
     *
     * <p>Uses field boundary constants with safety margin to allow for edge cases
     * like robot slightly off field during gameplay.</p>
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
}
