package org.firstinspires.ftc.teamcode.subsystems.drive;

import com.seattlesolvers.solverslib.command.SubsystemBase;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.util.pathing.PedroPathingConstants;
import org.firstinspires.ftc.teamcode.subsystems.drive.OctoSwerveDrivetrainV2;
import org.firstinspires.ftc.teamcode.hardware.OctoQuadFWv3;
import org.firstinspires.ftc.teamcode.subsystems.localization.LocalizationSubsystem;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.seattlesolvers.solverslib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.hardware.motors.CRServoEx;

import static org.firstinspires.ftc.teamcode.Constants.*;

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
 * <p>This drivetrain supports two encoder types, selectable via {@link org.firstinspires.ftc.teamcode.Constants#SWERVE_ENCODER_TYPE}:</p>
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
 * <p>Telemetry and CSV logging are handled by {@link org.firstinspires.ftc.teamcode.subsystems.drive.DriveTelemetry}
 * to maintain separation of concerns. This subsystem provides getter methods for telemetry access.</p>
 * <ul>
 *   <li>Panels Dashboard field overlay with robot position and wheel orientations</li>
 *   <li>Encoder type display (OctoQuad or Analog)</li>
 *   <li>CSV logging of pose, velocity, and module states for post-match analysis</li>
 *   <li>Capture & Replay support for autonomous analysis and tuning</li>
 * </ul>
 *
 * @see com.pedropathing.follower.Follower
 * @see org.firstinspires.ftc.teamcode.subsystems.drive.OctoSwerveDrivetrainV2
 * @see org.firstinspires.ftc.teamcode.subsystems.encoders.SwerveEncoder
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
     * Switch via {@link org.firstinspires.ftc.teamcode.Constants#SWERVE_ENCODER_TYPE}.</p>
     */
    public final OctoSwerveDrivetrainV2 swerve;

    /**
     * Localization subsystem that manages sensor fusion, Pinpoint, and IMU backup.
     * Provides fused pose combining swerve odometry, Pinpoint, and vision.
     */
    private final org.firstinspires.ftc.teamcode.subsystems.localization.LocalizationSubsystem localization;

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
     * Cached fused pose from sensor fusion for telemetry access.
     */
    private Pose fusedPose = null;

    /**
     * Cached current draw from SRS Hub for telemetry access.
     */
    private double currentDraw = 0.0;

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
        follower = PedroPathingConstants.createFollower(robot.hardwareMap);

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

        // Localization is handled by LocalizationSubsystem
        // (Sensor fusion, Pinpoint management, IMU backup are all centralized there)
        localization = new org.firstinspires.ftc.teamcode.subsystems.localization.LocalizationSubsystem();
        localization.init();
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
    /**
     * Gets the localization subsystem.
     *
     * <p>This provides access to sensor fusion, Pinpoint management, and IMU backup
     * for external subsystems (like Vision) to apply pose corrections.</p>
     *
     * @return the localization subsystem
     */
    public org.firstinspires.ftc.teamcode.subsystems.localization.LocalizationSubsystem getLocalization() {
        return localization;
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
     * @see OctoSwerveDrivetrainV2#drive(ChassisSpeeds, double)
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
     * Resets the field-centric heading to the robot's current orientation.
     *
     * <p>This sets the robot's current heading as the new "forward" direction for
     * field-centric driving. Useful if the robot's heading becomes desynchronized or
     * if you want to quickly realign the controls.</p>
     *
     * <p><b>How it works:</b></p>
     * <ol>
     *   <li>Gets the current robot pose (x, y, heading)</li>
     *   <li>Creates a new pose with the same x, y but heading = 0</li>
     *   <li>Sets this as the new starting pose for PedroPathing's localizer</li>
     * </ol>
     *
     * <p><b>When to use:</b></p>
     * <ul>
     *   <li>Robot gets confused about its heading after collision</li>
     *   <li>Driver wants to quickly realign field-centric controls</li>
     *   <li>Vision/localization has drifted and needs manual correction</li>
     * </ul>
     *
     * <p><b>Note:</b> This only affects field-centric driving. Robot-centric driving
     * is unaffected by heading resets.</p>
     */
    public void resetFieldCentricHeading() {
        // Get current pose
        Pose currentPose = follower.getPoseTracker().getPose();

        // Create new pose with same x, y but heading = 0
        Pose resetPose = new Pose(currentPose.getX(), currentPose.getY(), 0.0);

        // Reset the localizer heading
        follower.setStartingPose(resetPose);
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
     * <p>Sends telemetry data to Panels Dashboard and CSV logger:</p>
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

        // ===== 1. Localization Update =====
        // Get current chassis speeds for localization
        ChassisSpeeds currentSpeeds = isTeleOpMode ? teleOpSpeeds :
            (follower.isBusy() ? new ChassisSpeeds(
                follower.getPose().getX(),
                follower.getPose().getY(),
                follower.getPose().getHeading()
            ) : new ChassisSpeeds(0, 0, 0));

        // Update localization subsystem (sensor fusion EKF, Pinpoint health, IMU backup)
        localization.update(currentSpeeds);

        // Get fused pose estimate and cache for telemetry
        fusedPose = localization.getFusedPose();

        // Update PedroPathing localizer with fused pose
        follower.setPose(fusedPose);

        // Update PedroPathing odometry internally
        follower.update();

        // ===== 2. Drive Command Execution =====

        // Read current draw from SRS Hub (if available) and cache for telemetry
        currentDraw = 0.0;
        boolean srsHubReady = org.firstinspires.ftc.teamcode.Constants.SRS_HUB_ENABLED &&
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

        // Calculate loop time at the end of periodic
        lastLoopTime = (System.nanoTime() / 1e9) - loopStartTime;
    }

    // ===== Telemetry Getter Methods =====
    // These methods provide read-only access to drive subsystem state for telemetry/logging

    /**
     * Gets the current robot pose from PedroPathing.
     *
     * @return current robot pose (x, y, heading)
     */
    public Pose getPose() {
        return follower.getPoseTracker().getPose();
    }

    /**
     * Gets the fused pose from sensor fusion (EKF output).
     *
     * @return fused pose combining all localization sources, or null if fusion disabled
     */
    public Pose getFusedPose() {
        return fusedPose;
    }

    /**
     * Gets the last drive inputs applied to the drivetrain.
     *
     * @return last chassis speeds (vx, vy, omega)
     */
    public ChassisSpeeds getLastInputs() {
        return teleOpSpeeds;
    }

    /**
     * Gets the last loop execution time in seconds.
     *
     * @return loop time in seconds
     */
    public double getLastLoopTime() {
        return lastLoopTime;
    }

    /**
     * Gets the current draw from the SRS Hub.
     *
     * @return current draw in amps
     */
    public double getCurrentDraw() {
        return currentDraw;
    }

    /**
     * Checks if current limiting is currently active.
     *
     * @return true if current limiting is reducing motor power
     */
    public boolean isCurrentLimitingActive() {
        return currentLimitingActive;
    }

    /**
     * Gets the encoder type being used.
     *
     * @return encoder type (e.g., "OctoQuad", "Analog (SRS Hub)")
     */
    public String getEncoderType() {
        return swerve.getEncoderType();
    }

    /**
     * Gets the swerve modules for telemetry access.
     *
     * @return array of 4 swerve modules [FR, FL, BL, BR]
     */
    public org.firstinspires.ftc.teamcode.subsystems.drive.OctoSwerveModuleV2[] getModules() {
        return swerve.getModules();
    }

    /**
     * Checks if the drive subsystem is currently in autonomous mode.
     *
     * @return true if PedroPathing is actively following a path
     */
    public boolean isAutonomousMode() {
        return !isTeleOpMode;
    }

    /**
     * Gets the position uncertainty from sensor fusion.
     *
     * @return array of [xUncertainty, yUncertainty] in inches
     */
    public double[] getPositionUncertainty() {
        return localization.getPositionUncertainty();
    }

    /**
     * Gets the heading uncertainty from sensor fusion.
     *
     * @return heading uncertainty in radians
     */
    public double getHeadingUncertainty() {
        return localization.getHeadingUncertainty();
    }

    /**
     * Checks if the Pinpoint odometry is healthy (no drift detected).
     *
     * @return true if Pinpoint is healthy, false if drift detected
     */
    public boolean isPinpointHealthy() {
        return localization.isPinpointHealthy();
    }

    /**
     * Gets the number of bad Pinpoint readings since last reset.
     *
     * @return bad reading count
     */
    public int getPinpointBadReadingCount() {
        return localization.getPinpointBadReadingCount();
    }
}
