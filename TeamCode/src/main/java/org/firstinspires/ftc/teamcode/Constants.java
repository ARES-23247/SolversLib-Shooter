package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.command.subsystems.encoders.SwerveEncoderFactory;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

/**
 * Robot configuration constants including physical dimensions, PIDF tuning values, and autonomous tolerances.
 *
 * <p>This class contains all robot-specific tuning parameters. These values must be adjusted
 * to match your robot's physical configuration and tuning results. The constants are organized
 * into three categories:</p>
 *
 * <ul>
 *   <li><b>Physical Dimensions:</b> Track width and wheel base for kinematics calculations</li>
 *   <li><b>PIDF Coefficients:</b> Control loop tuning for swerve modules</li>
 *   <li><b>Module Offsets:</b> Absolute encoder zero-position calibration values</li>
 *   <li><b>Autonomous Tolerances:</b> Position/heading error thresholds for path following</li>
 * </ul>
 *
 * <h3>⚡ PERFORMANCE MODE:</h3>
 * <p>Set {@link #PERFORMANCE_MODE} to <b>true</b> for maximum loop speed during competition.</p>
 * <p>When enabled, disables all non-essential features for maximum loop performance:</p>
 * <ul>
 *   <li>Dashboard overlay disabled (~2-3ms savings)</li>
 *   <li>CSV logging disabled (~0.5-1ms savings)</li>
 *   <li>Limelight polling reduced to 50Hz (~2-3ms savings)</li>
 *   <li>Vision updates every other loop (~1-2ms savings)</li>
 *   <li>Voltage cached every 10 loops (~0.1ms savings)</li>
 *   <li>SolversLib PhotonCore parallel commands increased to 12 (~1-2ms savings)</li>
 * </ul>
 * <p><b>Total savings: 7-12ms per loop (target: &lt;10ms loop time)</b></p>
 *
 *
 * <h3>PIDF Tuning Guide:</h3>
 * <ul>
 *   <li><b>P (Proportional):</b> Increases response to error. Too high causes oscillation.</li>
 *   <li><b>I (Integral):</b> Eliminates steady-state error. Use sparingly to prevent windup.</li>
 *   <li><b>D (Derivative):</b> Dampens oscillation and improves stability.</li>
 *   <li><b>F (Feed-Forward):</b> Compensates for gravity/friction. Important for velocity control.</li>
 * </ul>
 *
 * <h3>Module Offset Calibration:</h3>
 * <ol>
 *   <li>Physically align all wheels to point straight forward (parallel to robot sides)</li>
 *   <li>Read the current absolute encoder value for each module</li>
 *   <li>Set the offset constant to the negative of the reading</li>
 *   <li>Verify: wheels should now read 0 radians when pointing forward</li>
 * </ol>
 *
 * @see com.qualcomm.robotcore.hardware.PIDFCoefficients
 * @see org.firstinspires.ftc.teamcode.commandbase.subsystems.swerve.OctoSwerveModule
 */
public class Constants {

    // ===== Wheel Configuration =====

    /**
     * Drive wheel diameter in millimeters.
     * <p>
     * <b>Common FTC wheel sizes:</b>
     * <ul>
     *   <li>GoBilda 96mm Mecanum wheels</li>
     *   <li>GoBilda 72mm Omni wheels ← YOURS</li>
     *   <li>GoBilda 100mm Mecanum wheels</li>
     *   <li>Rev 96mm Omni wheels</li>
     * </ul>
     * </p>
     */
    public static final double WHEEL_DIAMETER_MM = 72.0;  // 72mm GoBilda Omni wheels

    /**
     * Drive wheel radius in inches.
     * Calculated from wheel diameter.
     */
    public static final double WHEEL_RADIUS_INCHES = (WHEEL_DIAMETER_MM / 25.4) / 2.0;  // ~1.417 inches

    /**
     * Drive wheel circumference in inches.
     * Used for velocity calculations: distance per wheel revolution.
     */
    public static final double WHEEL_CIRCUMFERENCE_INCHES = WHEEL_DIAMETER_MM / 25.4 * Math.PI;  // ~8.9 inches

    // ===== Motor & Gearing Configuration =====

    /**
     * Motor internal gear ratio.
     * <p>
     * GoBilda 5202 series motor with 5.18:1 internal gear reduction.</p>
     */
    public static final double MOTOR_GEAR_RATIO = 5.18;

    /**
     * External gearing stage 1 (same-size sprockets).
     * <p>
     * Ratio = 30 / 30 = 1:1 (no reduction)
     * Using 1:1 middle stage gives better top speed while maintaining precision.</p>
     *
     * <p><b>Your Configuration:</b></p>
     * <ul>
     *   <li>Driver: 30-tooth sprocket</li>
     *   <li>Driven: 30-tooth sprocket</li>
     *   <li>Ratio: 1:1 (no speed change)</li>
     * </ul>
     *
     * <p><b>Alternative Options:</b></p>
     * <ul>
     *   <li>24:24 sprockets (1:1) - Smaller sprockets, lighter weight</li>
     *   <li>36:36 sprockets (1:1) - Larger sprockets, more durable</li>
     *   <li>36:24 sprockets (1.5:1) - Higher reduction, more torque, slower speed</li>
     * </ul>
     */
    public static final double GEAR_STAGE_1_RATIO = 1.0;  // 1:1 (30:30)

    /**
     * External gearing stage 2 (2:1 sprockets or gears).
     * <p>
     * Ratio = 2:1 gear reduction</p>
     */
    public static final double GEAR_STAGE_2_RATIO = 2.0 / 1.0;  // 2:1

    /**
     * Total gear reduction from motor to wheel.
     * <p>
     * <b>Calculation (with 1:1 middle stage):</b></p>
     * <pre>
     * TOTAL_GEAR_RATIO = MOTOR_GEAR_RATIO × GEAR_STAGE_1_RATIO × GEAR_STAGE_2_RATIO
     *                = 5.18 × 1.0 × 2.0
     *                = 10.36:1
     * </pre>
     *
     * <p>For every 10.36 motor revolutions, the wheel makes 1 revolution.</p>
     *
     * <p><b>Performance:</b></p>
     * <ul>
     *   <li>50% faster than 15.54:1 reduction</li>
     *   <li>Still excellent precision for swerve drive</li>
     *   <li>Good torque for most FTC applications</li>
     * </ul>
     *
     * <p><b>Alternative Configurations:</b></p>
     * <ul>
     *   <li>36:24 (1.5:1) → 15.54:1 total (slower, more torque)</li>
     *   <li>24:24 (1.0:1) → 10.36:1 total (current - recommended) ⭐</li>
     *   <li>Direct drive → 5.18:1 total (fastest, less precision)</li>
     * </ul>
     */
    public static final double TOTAL_GEAR_RATIO = MOTOR_GEAR_RATIO * GEAR_STAGE_1_RATIO * GEAR_STAGE_2_RATIO;  // 10.36:1

    /**
     * OctoQuad encoder counts per revolution (CPR).
     * <p>
     * The OctoQuad provides 8192 counts per revolution for quadrature encoders.
     * This high resolution enables precise velocity control for swerve drive.</p>
     *
     * <p><b>Important:</b> The encoder is mounted on the MOTOR shaft, not the wheel shaft!</p>
     */
    public static final double OCTOQUAD_CPR = 8192.0;

    /**
     * Velocity conversion factor for drive motors.
     * <p>
     * Converts from OctoQuad ticks/second at the MOTOR to inches/second at the WHEEL,
     * accounting for the gear reduction.</p>
     *
     * <p><b>Formula:</b></p>
     * <pre>
     * VELOCITY_CONVERSION = (WHEEL_CIRCUMFERENCE / TOTAL_GEAR_RATIO) / OCTOQUAD_CPR
     *
     * With 1:1 middle stage (10.36:1 total):
     * = (8.906 inches / 10.36) / 8192
     * = 0.859 inches per motor revolution / 8192 counts
     * = 0.000105 inches per tick
     * </pre>
     *
     * <p><b>Your Configuration (with 1:1 middle stage):</b></p>
     * <ul>
     *   <li>Motor: SWYFT 5.18:1</li>
     *   <li>Stage 1: 24:24 or 36:36 (1:1) ← Same-size sprockets</li>
     *   <li>Stage 2: 2:1</li>
     *   <li>Total: 10.36:1</li>
     *   <li>Wheels: 72mm (8.906" circumference)</li>
     *   <li>Encoder: 8192 CPR (at motor)</li>
     *   <li>Conversion: 0.000105 inches/tick</li>
     * </ul>
     *
     * <p><b>Performance (50% faster than 15.54:1):</b></p>
     * <ul>
     *   <li>10,000 ticks/sec → 1.05 inches/sec</li>
     *   <li>50,000 ticks/sec → 5.25 inches/sec</li>
     *   <li>100,000 ticks/sec → 10.50 inches/sec</li>
     * </ul>
     *
     * <p><b>Estimated Top Speed:</b></p>
     * <ul>
     *   <li>Motor max: ~6,000 RPM</li>
     *   <li>Wheel max: 6,000 / 10.36 = 579 RPM</li>
     *   <li>Wheel speed: 579 RPM × 8.906 in/rev = 85.4 in/sec ≈ **7.1 ft/sec**</li>
     * </ul>
     *
     * <p><b>Trade-off:</b> 50% faster than 15.54:1, while maintaining excellent precision
     * and still having good torque for swerve drive applications.</p>
     */
    public static final double VELOCITY_CONVERSION = (WHEEL_CIRCUMFERENCE_INCHES / TOTAL_GEAR_RATIO) / OCTOQUAD_CPR;  // ~0.000105

    // ===== ⚡ PERFORMANCE MODE =====

    /**
     * ENABLE FOR MAXIMUM PERFORMANCE DURING COMPETITION!
     *
     * <p><b>When true:</b> Aggressively optimizes for loop speed. Disables all non-essential features.</p>
     * <p><b>When false:</b> Enables all tuning/diagnostic features. Use for testing and tuning only.</p>
     *
     * <p><b>Performance Impact:</b> Saves 7-12ms per loop, achieving sub-10ms loop times.</p>
     *
     * <p><b>Recommendation:</b> Set to true for competition matches. Set to false for practice/tuning.</p>
     */
    public static final boolean PERFORMANCE_MODE = false;

    /**
     * Limelight poll rate in Hz.
     * <p>
     * <b>Performance Mode:</b> 50Hz (competition - saves 2-3ms I2C time)</p>
     * <p>
     * <b>Tuning Mode:</b> 100Hz (practice - better vision responsiveness)</p>
     */
    public static final int LIMELIGHT_POLL_RATE = PERFORMANCE_MODE ? 50 : 100;

    /**
     * Vision update frequency (1 = every loop, 2 = every other loop, etc.)
     * <p>
     * <b>Performance Mode:</b> 2 (update every other loop - saves 1-2ms)</p>
     * <p>
     * <b>Tuning Mode:</b> 1 (update every loop - smoother vision)</p>
     */
    public static final int VISION_UPDATE_FREQUENCY = PERFORMANCE_MODE ? 2 : 1;

    /**
     * Voltage reading cache duration (number of loops to cache voltage reading).
     * <p>
     * <b>Performance Mode:</b> 10 loops (read voltage every 10 loops - saves ~0.1ms)</p>
     * <p>
     * <b>Tuning Mode:</b> 1 loop (read voltage every loop - accurate monitoring)</p>
     */
    public static final int VOLTAGE_CACHE_DURATION = PERFORMANCE_MODE ? 10 : 1;

    /**
     * Whether to enable dashboard field overlay.
     * <p>
     * <b>Performance Impact:</b> Adds ~1-2ms to loop time.</p>
     * <p>
     * <b>Performance Mode:</b> Disabled (not needed during competition)</p>
     * <p>
     * <b>Tuning Mode:</b> Enabled (essential for tuning)</p>
     */
    public static final boolean ENABLE_DASHBOARD_OVERLAY = !PERFORMANCE_MODE;

    /**
     * Whether to enable CSV logging.
     * <p>
     * <b>Performance Impact:</b> Adds ~0.5-1ms to loop time (flushing is throttled).</p>
     * <p>
     * <b>Performance Mode:</b> Disabled (not needed during competition)</p>
     * <p>
     * <b>Tuning Mode:</b> Enabled (essential for post-match analysis)</p>
     */
    public static final boolean ENABLE_CSV_LOGGING = !PERFORMANCE_MODE;

    /**
     * Whether to enable vision adaptive priority (boosts priority of cameras facing movement direction).
     * <p>
     * <b>Performance Impact:</b> Adds ~0.5ms to vision update loop.</p>
     * <p>
     * <b>Performance Mode:</b> Disabled (saves CPU cycles)</p>
     * <p>
     * <b>Tuning Mode:</b> Enabled (better multi-camera performance)</p>
     */
    public static final boolean VISION_ADAPTIVE_PRIORITY = !PERFORMANCE_MODE;

    /**
     * Whether to enable PhotonCore parallel servo updates.
     * <p>
     * <b>Performance Impact:</b> ~3-6ms faster loop times when enabled.</p>
     * <p>
     * <b>Performance Mode:</b> Enabled (all servos on USB hubs - safe and fast)</p>
     * <p>
     * <b>Tuning Mode:</b> Enabled (recommended for swerve drive)</p>
     */
    public static final boolean PARALLELIZE_SERVOS = true;

    // ===== Physical Dimensions =====

    /**
     * The left-to-right distance between the centers of the left and right wheels.
     * Measured in inches. This dimension is used for swerve kinematics calculations.
     * <p>
     * <b>How to measure:</b> Measure the distance from the center of the left wheel
     * to the center of the right wheel on the same axle (front or back).
     * </p>
     */
    public static final double TRACK_WIDTH = 15.0;

    /**
     * The front-to-back distance between the centers of the front and back wheels.
     * Measured in inches. This dimension is used for swerve kinematics calculations.
     * <p>
     * <b>How to measure:</b> Measure the distance from the center of the front wheel
     * to the center of the back wheel on the same side (left or right).
     * </p>
     */
    public static final double WHEEL_BASE = 15.0;

    /**
     * The maximum velocity the swerve drive motors can achieve.
     * Measured in inches per second. This value is used for velocity desaturation
     * to prevent the swerve modules from being commanded beyond their physical limits.
     * <p>
     * <b>How to tune:</b> Gradually increase until wheels slip or motors stall,
     * then back off 10-20% for safety margin.
     * </p>
     */
    public static final double MAX_DRIVE_VELOCITY = 60.0;

    // ===== PIDF Control Coefficients =====

    /**
     * PIDF coefficients for the swerve steering servos (Melon Super Servos).
     * These servos control the angular orientation of each swerve module.
     * <p>
     * <b>Tuning Tips:</b>
     * <ul>
     *   <li>Start with P only, increase until responsive but no oscillation</li>
     *   <li>Add small D to reduce overshoot</li>
     *   <li>I is typically 0 for position control unless holding against load</li>
     *   <li>F is usually 0 for servo position control</li>
     * </ul>
     * </p>
     *
     * @see com.qualcomm.robotcore.hardware.PIDFCoefficients
     */
    public static final PIDFCoefficients SWERVE_SERVO_PIDF = new PIDFCoefficients(1.2, 0.0, 0.05, 0.0);

    /**
     * PIDF coefficients for the swerve drive motors (velocity control).
     * These motors control the rotational speed of each swerve module's wheel.
     * Velocity feedback is provided by OctoQuad encoders.
     * <p>
     * <b>Tuning Tips:</b>
     * <ul>
     *   <li>P is primary gain for velocity control</li>
     *   <li>F (feed-forward) should match motor power needed to maintain target speed</li>
     *   <li>D helps prevent velocity overshoot</li>
     *   <li>I is typically kept low for velocity control</li>
     * </ul>
     * </p>
     *
     * @see com.qualcomm.robotcore.hardware.PIDFCoefficients
     * @see org.firstinspires.ftc.teamcode.util.OctoQuadFWv3
     */
    public static final PIDFCoefficients SWERVE_DRIVE_PIDF = new PIDFCoefficients(0.1, 0.0, 0.01, 0.0);  // F set to 0, using kV + kS instead

    // ===== Feedforward Constants =====

    /**
     * Static friction constant (kS) for drive motors.
     * <p>
     * This is the minimum power needed to overcome friction and start the motor moving.
     * Also called "breakaway voltage" or "stiction."</p>
     * <p>
     * <b>How to Measure:</b></p>
     * <ol>
     *   <li>Gradually increase motor power from 0 until wheels just start to spin</li>
     *   <li>Record this power value (typically 0.1 - 0.3 for most FTC motors)</li>
     *   <li>This is your kS constant</li>
     * </ol>
     * <p>
     * <b>Typical Values:</b>
     * <ul>
     *   <li><b>0.10 - 0.15:</b> Well-lubricated, low-friction systems</li>
     *   <li><b>0.15 - 0.25:</b> Typical FTC robots (normal friction)</li>
     *   <li><b>0.25 - 0.40:</b> High friction (heavy robots, dry bearings)</li>
     * </ul>
     * </p>
     * <p>
     * <b>Feedforward Equation:</b>
     * <pre>
     * motorPower = kS * sign(velocity) + kV * velocity
     * </pre>
     * </p>
     */
    public static final double DRIVE_KS = 0.15;  // Static friction constant

    /**
     * Velocity constant (kV) for drive motors.
     * <p>
     * This represents the motor's velocity per unit of applied power. It accounts for
     * the motor's back-EMF and electrical resistance. kV is the reciprocal of the motor's
     * KV rating (RPM per volt).</p>
     * <p>
     * <b>How to Measure:</b></p>
     * <ol>
     *   <li>Apply a fixed power (e.g., 0.5) and wait for steady-state velocity</li>
     *   <li>Measure the steady-state velocity in inches/second</li>
     *   <li>Calculate: {@code kV = (appliedPower - kS) / (measuredVelocity / maxVelocity)}</li>
     *   <li>Or simpler: {@code kV = appliedPower / (measuredVelocity / maxVelocity)}</li>
     * </ol>
     * <p>
     * <b>Example Measurement:</b>
     * <ul>
     *   <li>Apply 0.5 power</li>
     *   <li>Measure 30 in/s (out of 60 in/s max)</li>
     *   <li>Velocity fraction = 30/60 = 0.5</li>
     *   <li>kV = 0.5 / 0.5 = 1.0</li>
     * </ul>
     * </p>
     * <p>
     * <b>Typical Values:</b>
     * <ul>
     *   <li><b>0.8 - 1.0:</b> High-performance motors (good condition)</li>
     *   <li><b>1.0 - 1.2:</b> Typical FTC motors (normal)</li>
     *   <li><b>1.2 - 1.5:</b> Lower-performance motors (high friction, worn)</li>
     * </ul>
     * </p>
     * <p>
     * <b>Note:</b> kV is typically close to 1.0 for well-tuned systems. If kV is significantly
     * different from 1.0, check your MAX_DRIVE_VELOCITY calibration.</p>
     * </p>
     */
    public static final double DRIVE_KV = 1.0;  // Velocity constant


    // ===== Steering Feedforward Constants =====

    /**
     * Static friction constant (kS) for steering motors.
     *
     * <p>Minimum power to overcome gear resistance in the steering mechanism.</p>
     *
     * <p><b>Typical Values:</b> 0.02 - 0.05</p>
     * <p><b>How to Tune:</b> Use {@link org.firstinspires.ftc.teamcode.util.SteeringTuner}</p>
     *
     * <h3>About Steering Feedforward:</h3>
     * <p>Unlike drive motors, steering motors don't fight ground friction. They only need
     * a small kS to overcome gear resistance. Most swerve drives use position-only PID control
     * without feedforward.</p>
     *
     * <p><b>Note:</b> This constant is OPTIONAL. If your steering PID works well without it,
     * set STEERING_KS = 0.0 and rely on PID only.</p>
     */
    public static final double STEERING_KS = 0.02;  // Static friction (gear resistance)

    /**
     * Maximum steering velocity in radians per second.
     *
     * <p>This is the maximum rotational speed of the steering mechanism. Used for:</p>
     * <ul>
     *   <li>Velocity-based steering control (if using kV feedforward)</li>
     *   <li>Trajectory planning to ensure steering can keep up</li>
     *   <li>Detecting when steering is stuck or slowed</li>
     * </ul>
     *
     * <p><b>Typical Values:</b> 6.0 - 15.0 rad/s (depending on servo/gear ratio)</p>
     * <p><b>How to Measure:</b> Use {@link org.firstinspires.ftc.teamcode.util.SteeringTuner}</p>
     *
     * <h3>Calculation:</h3>
     * <pre>MAX_STEERING_VELOCITY = (Servo Speed × Gear Ratio) / 60</pre>
     * <p>For example: 100 RPM servo with 10:1 gears → 100 × 10 / 60 × 2π = 105 rad/s</p>
     *
     * <p><b>Note:</b> This is mainly for planning and diagnostics. Position-based PID control
     * (most common) doesn't use this value directly.</p>
     */
    public static final double MAX_STEERING_VELOCITY = 10.0;  // radians per second


    // ===== Module Encoder Offsets =====

    /**
     * Absolute encoder offset for the front-right swerve module.
     * Measured in radians. This value calibrates the zero position of the module's encoder.
     * <p>
     * <b>Calibration procedure:</b> With the wheel physically aligned to point straight forward,
     * read the encoder value and set this offset to the negative of that value. The module should
     * now read 0 radians when the wheel is forward.
     * </p>
     */
    public static final double FRONT_RIGHT_OFFSET = 0.0;

    /**
     * Absolute encoder offset for the front-left swerve module.
     * Measured in radians. See {@link #FRONT_RIGHT_OFFSET} for calibration procedure.
     */
    public static final double FRONT_LEFT_OFFSET = 0.0;

    /**
     * Absolute encoder offset for the back-left swerve module.
     * Measured in radians. See {@link #FRONT_RIGHT_OFFSET} for calibration procedure.
     */
    public static final double BACK_LEFT_OFFSET = 0.0;

    /**
     * Absolute encoder offset for the back-right swerve module.
     * Measured in radians. See {@link #FRONT_RIGHT_OFFSET} for calibration procedure.
     */
    public static final double BACK_RIGHT_OFFSET = 0.0;

    // ===== Autonomous Tolerances =====

    /**
     * Position tolerance for autonomous path following.
     * Measured in inches. The robot is considered "at target" when within this distance
     * of the target X/Y position.
     * <p>
     * <b>Tuning:</b> Smaller values = more precise but slower convergence.
     * Typical values: 0.5-2.0 inches depending on game requirements.
     * </p>
     */
    public static final double AUTO_XY_TOLERANCE = 1.0;

    /**
     * Heading tolerance for autonomous path following.
     * Measured in radians. The robot is considered "at target" when within this angle
     * of the target heading.
     * <p>
     * <b>Tuning:</b> Smaller values = more precise alignment. Converted from degrees
     * using {@code Math.toRadians(2.0)} for approximately 2 degrees of tolerance.
     * </p>
     */
    public static final double AUTO_HEADING_TOLERANCE = Math.toRadians(2.0);

    // ===== Loop Optimization Configuration =====

    /**
     * PhotonCore cache tolerance for motor reads.
     * <p>
     * <b>Purpose:</b> Prevents redundant motor reads if value hasn't changed significantly.
     * Higher values = fewer I2C reads but less responsive control.
     * </p>
     * <p>
     * <b>Recommended Values:</b>
     * <ul>
     *   <li>0.01 - Default, good balance (1% change triggers update)</li>
     *   <li>0.05 - More aggressive caching (5% change triggers update)</li>
     *   <li>0.001 - Very responsive, more I2C traffic</li>
     * </ul>
     * </p>
     */
    public static final double MOTOR_CACHE_TOLERANCE = 0.01;

    /**
     * PhotonCore cache tolerance for servo reads.
     * <p>
     * Servos can typically use more aggressive caching than motors since they
     * move slower and position precision is less critical.
     * </p>
     */
    public static final double SERVO_CACHE_TOLERANCE = 0.01;

    // ===== OctoQuad Configuration =====

    /**
     * OctoQuad is used ONLY for swerve drive encoders, NOT for localization.
     *
     * <p><b>Port Usage:</b></p>
     * <ul>
     *   <li>Ports 0-3: Through-bore encoders for steering angle feedback</li>
     *   <li>Ports 4-7: Drive motor encoders for velocity feedback</li>
     * </ul>
     *
     * <p><b>Primary Localization:</b> GoBilda Pinpoint for pose estimation (IMU + deadwheel odometry).</p>
     *
     * <p><b>Backup IMU:</b> OctoQuad's built-in IMU is available as a backup heading source if Pinpoint fails.</p>
     */
    public static final boolean OCTOQUAD_IMU_BACKUP_ENABLED = true;

    /**
     * OctoQuad IMU heading scalar for backup mode.
     * <p>
     * <b>Default:</b> 1.0 (no scaling)
     * </p>
     * <p>
     * <b>How to tune:</b> Use the HeadingScalarCalibrator example from DigitalChickenLabs
     * to determine the optimal scalar value for your IMU. This compensates for
     * systematic heading drift.
     * </p>
     */
    public static final float OCTOQUAD_IMU_HEADING_SCALAR = 1.0f;

    /**
     * OctoQuad IMU measurement noise for sensor fusion (radians).
     * <p>
     * This represents the uncertainty in OctoQuad IMU heading measurements when used as backup.
     * OctoQuad IMU is generally less accurate than Pinpoint IMU, so this should be higher.
     * </p>
     * <p>
     * <b>Note:</b> Higher values = filter trusts OctoQuad IMU less (only in backup mode).
     * </p>
     */
    public static final double FUSION_OCTOQUAD_IMU_NOISE_HEADING = 0.05;

    // ===== SRS Hub Current Monitoring =====

    /**
     * Whether to enable SRS Hub for analog current monitoring.
     * <p>
     * When enabled, the SRS Hub monitors current draw from a GoBilda Floodgate
     * power switch using one of its analog input ports.
     * </p>
     */
    public static final boolean SRS_HUB_ENABLED = true;

    /**
     * Analog pin number on the SRS Hub connected to the GoBilda Floodgate current output.
     * <p>
     * <b>Valid Range:</b> 1-12
     * </p>
     * <p>
     * <b>Connection:</b> Connect the Floodgate's analog current output to the specified
     * pin on the SRS Hub. The Floodgate outputs a voltage proportional to current draw.
     * </p>
     */
    public static final int SRS_HUB_CURRENT_PIN = 1;

    /**
     * Maximum current in amps that corresponds to the maximum analog reading (1.0).
     * <p>
     * <b>GoBilda Floodgate V2:</b> The analog output is scaled so that:
     * <ul>
     *   <li>0V = 0 amps</li>
     *   <li>3.3V = 80 amps (max measurable current)</li>
     * </ul>
     * The SRS Hub uses a 12-bit ADC, so:
     * <ul>
     *   <li>0 (0.0) = 0V = 0 amps</li>
     *   <li>4095 (1.0) = 3.3V = 80 amps</li>
     * </ul>
     * </p>
     * <p>
     * <b>Reference:</b> <a href="https://www.gobilda.com/floodgate-power-switch-xt30-current-sensing/">GoBilda Floodgate V2 Documentation</a>
     * </p>
     * <p>
     * <b>Formula:</b> {@code currentAmps = analogValue * MAX_CURRENT_AMPS}
     * </p>
     */
    public static final double SRS_HUB_MAX_CURRENT_AMPS = 80.0;

    // ===== Swerve Encoder Configuration =====

    /**
     * Type of encoders to use for swerve modules.
     * <p>
     * <b>Options:</b></p>
     * <ul>
     *   <li><b>OCTOQUAD:</b> High-precision digital encoders via OctoQuad (8192 CPR)</li>
     *   <li><b>ANALOG:</b> Analog potentiometers/magnetic encoders via SRS Hub</li>
     * </ul>
     * </p>
     * <p>
     * <b>Recommended:</b> OCTOQUAD for best accuracy and performance.
     * Use ANALOG only if OctoQuad ports are full or for lower-cost testing.
     * </p>
     * <p>
     * <b>Comparison:</b>
     * <table border="1">
     *   <tr><th>Feature</th><th>OctoQuad</th><th>Analog (SRS Hub)</th></tr>
     *   <tr><td>Resolution</td><td>8192 CPR</td><td>4096 levels</td></tr>
     *   <tr><td>Accuracy</td><td>High (quadrature)</td><td>Medium (voltage)</td></tr>
     *   <tr><td>Noise Immunity</td><td>Excellent (digital)</td><td>Poor (analog)</td></tr>
     *   <tr><td>Drift</td><td>None (absolute)</td><td>Yes (temperature)</td></tr>
     *   <tr><td>Cost</td><td>Higher</td><td>Lower</td></tr>
     * </table>
     * </p>
     */
    public static final SwerveEncoderFactory.SwerveEncoderType SWERVE_ENCODER_TYPE = SwerveEncoderFactory.SwerveEncoderType.OCTOQUAD;

    /**
     * SRS Hub analog pin assignments for steering encoders.
     * <p>
     * Only used if {@link #SWERVE_ENCODER_TYPE} is {@link SwerveEncoderFactory.SwerveEncoderType#ANALOG}.
     * </p>
     * <p>
     * Array indices correspond to modules: [FR, FL, BL, BR]</p>
     * <p>
     * Set pin to -1 to disable (not recommended - all modules need steering encoders).
     * </p>
     * <p>
     * <b>Pin Mapping:</b> 1-12 (SRS Hub analog input pins)</p>
     * </p>
     * <p>
     * <b>Recommended:</b> Use pins 1-4 for steering encoders
     * </p>
     */
    public static final int[] STEERING_ANALOG_PINS = {1, 2, 3, 4};

    /**
     * Analog encoder voltage range for steering encoders.
     * <p>
     * Only used if {@link #SWERVE_ENCODER_TYPE} is {@link SwerveEncoderFactory.SwerveEncoderType#ANALOG}.</p>
     * <p>
     * Most analog encoders output 0-3.3V over their full range.
     * </p>
     * <p>
     * <b>Example:</b> For a 270° potentiometer:
     * <ul>
     *   <li>0V = -135°</li>
     *   <li>1.65V = 0° (forward)</li>
     *   <li>3.3V = +135°</li>
     * </ul>
     * </p>
     */
    public static final double ANALOG_ENCODER_MIN_VOLTAGE = 0.0;
    public static final double ANALOG_ENCODER_MAX_VOLTAGE = 3.3;

    /**
     * Analog encoder angle range for steering encoders.
     * <p>
     * Only used if {@link #SWERVE_ENCODER_TYPE} is {@link SwerveEncoderFactory.SwerveEncoderType#ANALOG}.</p>
     * <p>
     * Most analog encoders cover ±π radians (±180°) for full steering range.
     * </p>
     * <p>
     * <b>270° Potentiometer:</b> Set to -3π/4 to +3π/4 (-135° to +135°)</p>
     * <p>
     * <b>360° Potentiometer:</b> Set to -π to +π (-180° to +180°)</p>
     */
    public static final double ANALOG_ENCODER_MIN_ANGLE = -Math.PI;
    public static final double ANALOG_ENCODER_MAX_ANGLE = Math.PI;

    /**
     * Current threshold in amps for triggering a warning.
     * <p>
     * If the current draw exceeds this threshold, a warning will be logged.
     * Set to 0 to disable warnings.
     * </p>
     * <p>
     * <b>Recommended Value:</b> 60-65A provides advance warning before the Floodgate's
     * 80A smart current limit trips. The internal 60A fuse provides final protection.
     * </p>
     */
    public static final double SRS_HUB_CURRENT_WARNING_THRESHOLD = 60.0;

    // ===== Current Management =====

    /**
     * Whether to enable slew rate limiting on drive motors.
     * <p>
     * Slew rate limiting prevents rapid power changes that can cause current spikes
     * and trip the 20A fuse. It smooths out motor power transitions.
     * </p>
     * <p>
     * <b>Recommended:</b> Enable for competition to prevent fuse trips.
     * </p>
     */
    public static final boolean ENABLE_SLEW_RATE_LIMIT = true;

    /**
     * Slew rate limit for drive motors in units per second.
     * <p>
     * This determines how fast motor power can change. A value of 2.0 means power
     * can change by at most 2.0 per second (e.g., from 0 to 1 takes 0.5 seconds).
     * </p>
     * <p>
     * <b>Tuning Guide:</b>
     * <ul>
     *   <li><b>1.0 - 2.0:</b> Very conservative, excellent protection, robot feels sluggish</li>
     *   <li><b>2.0 - 4.0:</b> Balanced, good protection with reasonable response (recommended)</li>
     *   <li><b>4.0 - 6.0:</b> Aggressive, minimal protection, very responsive</li>
     *   <li><b>6.0+:</b> Minimal effect, similar to no limiting</li>
     * </ul>
     * </p>
     * <p>
     * <b>Note:</b> This affects the drive motors only, not steering servos.
     * </p>
     */
    public static final double DRIVE_SLEW_RATE_LIMIT = 3.0;

    /**
     * Whether to enable current limiting based on Floodgate sensor.
     * <p>
     * Current limiting scales motor power down when approaching the current limit
     * to prevent tripping the 20A fuse. Works in conjunction with slew rate limiting.
     * </p>
     * <p>
     * <b>Recommended:</b> Enable for competition to prevent fuse trips during
     * high-current maneuvers (pushing, acceleration, climbing).
     * </p>
     */
    public static final boolean ENABLE_CURRENT_LIMIT = true;

    /**
     * Current threshold in amps where power scaling begins.
     * <p>
     * Below this value, motors receive full power. Above this value, power is
     * gradually reduced as current approaches the hard limit.
     * </p>
     * <p>
     * <b>Recommended Value:</b> 16-18A for a 20A fuse (2-4A headroom).
     * </p>
     * <p>
     * <b>Tuning:</b> Set lower if fuses still trip. Set higher for more power
     * if you never trip fuses.
     * </p>
     */
    public static final double CURRENT_LIMIT_WARNING_THRESHOLD = 17.0;

    /**
     * Hard current limit in amps.
     * <p>
     * At or above this current, motors are limited to minimum power fraction.
     * Should be set just below the fuse rating (typically 18-19A for a 20A fuse).
     * </p>
     * <p>
     * <b>Recommended Value:</b> 19A for a 20A fuse (1A headroom).
     * </p>
     */
    public static final double CURRENT_HARD_LIMIT = 19.0;

    /**
     * Minimum power fraction during current limiting [0.0, 1.0].
     * <p>
     * Even at maximum current overrun, motors will receive at least this much power.
     * Prevents the robot from becoming completely unresponsive during high-current events.
     * </p>
     * <p>
     * <b>Values:</b>
     * <ul>
     *   <li><b>0.3 - 0.4:</b> Very aggressive current protection, robot slows significantly</li>
     *   <li><b>0.5 - 0.6:</b> Balanced protection (recommended)</li>
     *   <li><b>0.7 - 0.8:</b> Minimal protection, more power but higher fuse trip risk</li>
     * </ul>
     * </p>
     */
    public static final double CURRENT_LIMIT_MIN_POWER_FRACTION = 0.5;

    /**
     * Time constant for current limit recovery in seconds.
     * <p>
     * When current drops, power gradually recovers at this rate to prevent oscillation.
     * A smaller value = faster recovery, larger value = smoother recovery.
     * </p>
     * <p>
     * <b>Typical Values:</b> 0.5 - 2.0 seconds.
     * </p>
     */
    public static final double CURRENT_LIMIT_RECOVERY_TIME_CONSTANT = 1.0;

    // ===== Sensor Fusion Configuration =====

    /**
     * Whether to enable sensor fusion for localization.
     * <p>
     * When enabled, the robot uses an Extended Kalman Filter (EKF) to fuse three
     * localization sources: Swerve odometry, Pinpoint deadwheel odometry, and Limelight vision.
     * </p>
     * <p>
     * <b>Recommended:</b> Enable for competition to maximize localization accuracy.
     * Disable to use PedroPathing's internal odometry only.
     * </p>
     */
    public static final boolean ENABLE_SENSOR_FUSION = true;

    /**
     * Process noise standard deviation for position (inches).
     * <p>
     * This represents how much the robot's position changes unpredictably per control loop.
     * Higher values = filter trusts predictions less, measurements more.
     * </p>
     * <p>
     * <b>Tuning Guide:</b>
     * <ul>
     *   <li><b>0.01 - 0.05:</b> Very predictable motion (smooth driving, low acceleration)</li>
     *   <li><b>0.05 - 0.15:</b> Typical motion (normal driving, moderate acceleration)</li>
     *   <li><b>0.15 - 0.30:</b> Unpredictable motion (aggressive driving, wheel slip)</li>
     * </ul>
     * </p>
     */
    public static final double FUSION_PROCESS_NOISE_POSITION = 0.1;

    /**
     * Process noise standard deviation for heading (radians).
     * <p>
     * This represents how much the robot's heading changes unpredictably per control loop.
     * </p>
     * <p>
     * <b>Tuning Guide:</b>
     * <ul>
     *   <li><b>0.001 - 0.005:</b> Very stable heading (smooth turning)</li>
     *   <li><b>0.005 - 0.015:</b> Typical motion (normal turning)</li>
     *   <li><b>0.015 - 0.030:</b> Unpredictable heading (aggressive turning, scrub)</li>
     * </ul>
     * </p>
     */
    public static final double FUSION_PROCESS_NOISE_HEADING = 0.01;

    /**
     * Process noise standard deviation for velocity (inches/second).
     * <p>
     * This represents how much the robot's velocity changes unpredictably per control loop.
     * </p>
     * <p>
     * <b>Typical Values:</b> 1.0 - 5.0 inches/second.
     * </p>
     */
    public static final double FUSION_PROCESS_NOISE_VELOCITY = 2.0;

    /**
     * Swerve odometry measurement noise standard deviation for position (inches).
     * <p>
     * This represents the uncertainty in swerve odometry position measurements.
     * Swerve odometry is prone to wheel slip, so this should be higher than Pinpoint noise.
     * </p>
     * <p>
     * <b>Tuning Guide:</b>
     * <ul>
     *   <li><b>0.5 - 1.0:</b> High slip conditions (tiled floor, aggressive driving)</li>
     *   <li><b>1.0 - 2.0:</b> Moderate slip (typical FTC field conditions)</li>
     *   <li><b>2.0 - 4.0:</b> Low slip (good traction, smooth driving)</li>
     * </ul>
     * </p>
     * <p>
     * <b>Note:</b> Higher values = filter trusts swerve odometry less.
     * </p>
     */
    public static final double FUSION_SWERVE_NOISE_POSITION = 1.5;

    /**
     * Swerve odometry measurement noise standard deviation for heading (radians).
     * <p>
     * This represents the uncertainty in swerve odometry heading measurements.
     * </p>
     * <p>
     * <b>Typical Values:</b> 0.05 - 0.15 radians (~3-9 degrees).
     * </p>
     */
    public static final double FUSION_SWERVE_NOISE_HEADING = 0.1;

    /**
     * Pinpoint odometry measurement noise standard deviation for position (inches).
     * <p>
     * This represents the uncertainty in Pinpoint deadwheel odometry measurements.
     * Deadwheels are very accurate, so this should be lower than swerve odometry noise.
     * </p>
     * <p>
     * <b>Tuning Guide:</b>
     * <ul>
     *   <li><b>0.1 - 0.3:</b> High accuracy deadwheels (well-calibrated, good pods)</li>
     *   <li><b>0.3 - 0.6:</b> Typical accuracy (standard calibration)</li>
     *   <li><b>0.6 - 1.0:</b> Lower accuracy (poor calibration, worn pods)</li>
     * </ul>
     * </p>
     * <p>
     * <b>Note:</b> Lower values = filter trusts Pinpoint more.
     * </p>
     */
    public static final double FUSION_PINPOINT_NOISE_POSITION = 0.3;

    /**
     * Pinpoint odometry measurement noise standard deviation for heading (radians).
     * <p>
     * This represents the uncertainty in Pinpoint IMU heading measurements.
     * The Pinpoint IMU is generally accurate, so this should be low.
     * </p>
     * <p>
     * <b>Typical Values:</b> 0.01 - 0.03 radians (~0.5-2 degrees).
     * </p>
     */
    public static final double FUSION_PINPOINT_NOISE_HEADING = 0.02;

    /**
     * Vision measurement noise standard deviation for position (inches).
     * <p>
     * This represents the uncertainty in Limelight AprilTag position measurements.
     * Vision provides global position correction and is very accurate when tags are visible.
     * </p>
     * <p>
     * <b>Tuning Guide:</b>
     * <ul>
     *   <li><b>0.5 - 1.0:</b> High accuracy vision (close range, good lighting, multiple tags)</li>
     *   <li><b>1.0 - 2.0:</b> Typical accuracy (medium range, decent lighting, single tag)</li>
     *   <li><b>2.0 - 4.0:</b> Lower accuracy (long range, poor lighting, tag ambiguity)</li>
     * </ul>
     * </p>
     * <p>
     * <b>Note:</b> Lower values = filter trusts vision more (aggressive correction).
     * Higher values = smoother correction (less aggressive).
     * </p>
     */
    public static final double FUSION_VISION_NOISE_POSITION = 1.0;

    /**
     * Vision measurement noise standard deviation for heading (radians).
     * <p>
     * This represents the uncertainty in Limelight AprilTag heading measurements.
     * </p>
     * <p>
     * <b>Typical Values:</b> 0.02 - 0.05 radians (~1-3 degrees).
     * </p>
     * <p>
     * <b>Note:</b> Lower values = more aggressive heading correction from vision.
     * </p>
     */
    public static final double FUSION_VISION_NOISE_HEADING = 0.03;

    // ===== Vision Field Boundary Validation =====

    /**
     * Whether to enable field boundary checking for vision measurements.
     * <p>
     * When enabled, vision readings outside field bounds are rejected to prevent
     * the EKF from updating with physically impossible positions.
     * </p>
     */
    public static final boolean ENABLE_VISION_BOUNDARY_CHECK = true;

    /**
     * Field boundary X limits (inches from center).
     * <p>
     * FTC field is 12x12 feet = 144x144 inches.
     * With origin at field center: [-72, +72]
     * </p>
     */
    public static final double FIELD_X_MIN = -72.0;
    public static final double FIELD_X_MAX = 72.0;

    /**
     * Field boundary Y limits (inches from center).
     * <p>
     * FTC field is 12x12 feet = 144x144 inches.
     * With origin at field center: [-72, +72]
     * </p>
     */
    public static final double FIELD_Y_MIN = -72.0;
    public static final double FIELD_Y_MAX = 72.0;

    /**
     * Safety margin outside field boundary (inches).
     * <p>
     * Allows robot to be slightly outside field for edge cases (carpet compression,
     * robot partially off field during gameplay, etc.).
     * </p>
     */
    public static final double FIELD_BOUNDARY_MARGIN = 6.0;

    // ===== Vision Z-Axis "Floating Robot" Detection =====

    /**
     * Maximum valid Z/height deviation for vision measurements (inches).
     * <p>
     * Readings with Z greater than this indicate the robot is "floating"
     * (likely due to camera seeing elevated AprilTags or bad perspective).
     * </p>
     * <p>
     * <b>Typical values:</b>
     * <ul>
     *   <li>2-4 inches: Robot on flat floor with normal camera mounting</li>
     *   <li>6-12 inches: Allows for some field irregularities</li>
     *   <li>12-18 inches: Permissive, handles large field irregularities</li>
     *   <li>< 0 inches: Robot below field (impossible, reject)</li>
     * </ul>
     * </p>
     * <p>
     * <b>Note:</b> Camera calibration (mount offsets) should account for normal
     * mounting height. This threshold catches UNUSUAL Z values that indicate
     * bad measurements, not normal camera height.
     * </p>
     */
    public static final double VISION_MAX_VALID_Z = 6.0;

    /**
     * Vision noise multiplier when Z-axis indicates floating robot.
     * <p>
     * Increases uncertainty for questionable measurements where the robot appears
     * to be floating above the field. This allows the EKF to still use potentially
     * valid x,y data while reducing the weight to prevent pose jumps.
     * </p>
     * <p>
     * <b>Values:</b>
     * <ul>
     *   <li>1.0: No change (trust floating readings)</li>
     *   <li>2.0-3.0: Moderate uncertainty increase (recommended)</li>
     *   <li>5.0+: High uncertainty (mostly ignore floating readings)</li>
     * </ul>
     * </p>
     */
    public static final double VISION_FLOATING_NOISE_MULTIPLIER = 3.0;

    // ===== Multi-Camera Vision Configuration =====

    /**
     * Names of Limelight3A cameras in the hardware map.
     * <p>
     * Add or remove camera names from this array to use multiple cameras.
     * Each camera must be configured in the RC with a matching device name.
     * </p>
     * <p>
     * <b>Examples:</b>
     * <ul>
     *   <li>Single camera: {"limelight"}</li>
     *   <li>Front + Rear: {"limelightFront", "limelightRear"}</li>
     *   <li>Full 360°: {"limelightFront", "limelightRear", "limelightLeft", "limelightRight"}</li>
     * </ul>
     * </p>
     */
    public static final String[] LIMELIGHT_NAMES = {"limelight"};

    /**
     * Priority weights for each camera when multiple see the same tag.
     * <p>
     * Higher priority cameras are preferred when multiple cameras have detections.
     * Array must match LIMELIGHT_NAMES length.
     * </p>
     * <p>
     * <b>Priority Guidelines:</b>
     * <ul>
     *   <li>1.0 = Highest priority (prefer this camera)</li>
     *   <li>0.5 = Medium priority (use if others unavailable)</li>
     *   <li>0.1 = Lowest priority (only use if no other option)</li>
     * </ul>
     * </p>
     * <p>
     * <b>Example:</b> Front camera has highest priority when driving forward.
     * </p>
     */
    public static final double[] LIMELIGHT_PRIORITIES = {1.0};

    /**
     * Maximum tag distance (inches) to trust a camera's pose measurement.
     * <p>
     * Detections beyond this distance are ignored due to low accuracy.
     * Typical values: 48-96 inches (4-8 feet).
     * </p>
     */
    public static final double VISION_MAX_TAG_DISTANCE = 72.0;

    /**
     * Minimum number of tags required for a valid vision measurement.
     * <p>
     * Higher values = more conservative (only trust when multiple tags visible).
     * Recommended: 1 for general use, 2+ for high-precision scenarios.
     * </p>
     */
    public static final int VISION_MIN_TAGS = 1;

    /**
     * Multi-camera fusion mode.
     * <p>
     * Determines how to combine measurements from multiple cameras:
     * <ul>
     *   <li><b>SELECT_BEST:</b> Use single best camera (default, most stable)</li>
     *   <li><b>WEIGHTED_AVERAGE:</b> Fuse all cameras with confidence weighting</li>
     *   <li><b>CONSENSUS:</b> Require multiple cameras to agree (rejects outliers)</li>
     * </ul>
     * </p>
     */
    public static final VisionFusionMode VISION_FUSION_MODE = VisionFusionMode.SELECT_BEST;

    /**
     * Per-camera mounting positions relative to robot center (inches).
     * <p>
     * Array format: {{x1, y1}, {x2, y2}, ...} for each camera
     * Used for pose transformations and outlier detection.
     * </p>
     * <p>
     * <b>Example:</b> Front camera mounted 6 inches forward, 0 inches left/right
     * </p>
     */
    public static final double[][] LIMELIGHT_MOUNT_POSITIONS = {{0.0, 0.0}};

    /**
     * Per-camera orientation offsets (radians).
     * <p>
     * Array format: {offset1, offset2, ...} for each camera
     * Use this if cameras are not perfectly aligned with robot forward.
     * </p>
     */
    public static final double[] LIMELIGHT_ORIENTATION_OFFSETS = {0.0};

    /**
     * Per-camera position noise standard deviations (inches).
     * <p>
     * Array format: {noise1, noise2, ...} for each camera
     * Higher values = less trust in that camera's position measurements.
     * </p>
     * <p>
     * <b>Typical values:</b> 0.5-2.0 inches depending on camera quality and mounting.
     * </p>
     */
    public static final double[] LIMELIGHT_POSITION_NOISE = {1.0};

    /**
     * Per-camera heading noise standard deviations (radians).
     * <p>
     * Array format: {noise1, noise2, ...} for each camera
     * Higher values = less trust in that camera's heading measurements.
     * </p>
     * <p>
     * <b>Typical values:</b> 0.02-0.1 radians depending on camera quality.
     * </p>
     */
    public static final double[] LIMELIGHT_HEADING_NOISE = {0.03};

    /**
     * Maximum allowed difference between camera measurements for consensus (inches).
     * <p>
     * If cameras disagree by more than this amount, the outlier is rejected.
     * Only used in CONSENSUS fusion mode.
     * </p>
     * <p>
     * <b>Typical values:</b> 3-6 inches.
     * </p>
     */
    public static final double VISION_CONSENSUS_MAX_DIFF = 4.0;

    /**
     * Preferred tag IDs for vision localization.
     * <p>
     * Array of tag IDs to prefer when multiple tags are visible.
     * Empty array = no preference (use closest tag).
     * </p>
     * <p>
     * <b>Example:</b> {1, 2, 3} prefers tags 1, 2, 3 over others.
     * </p>
     * <p>
     * <b>Use Case:</b> Use tags on your alliance's side of the field.
     * </p>
     */
    public static final int[] VISION_PREFERRED_TAG_IDS = {};

    /**
     * Camera health monitoring timeout (seconds).
     * <p>
     * If a camera has no valid detections for this long, it's marked as unhealthy.
     * Unhealthy cameras are excluded from fusion until they recover.
     * </p>
     * <p>
     * <b>Typical values:</b> 5-10 seconds.
     * </p>
     */
    public static final double VISION_CAMERA_HEALTH_TIMEOUT = 5.0;

    /**
     * Multi-camera fusion mode enum.
     */
    public enum VisionFusionMode {
        /**
         * Select single best camera (most stable, default).
         * Scores each camera and uses only the winner.
         */
        SELECT_BEST,

        /**
         * Weighted average fusion from all cameras.
         * Combines measurements with confidence-based weighting.
         * More accurate but potentially less stable if cameras disagree.
         */
        WEIGHTED_AVERAGE,

        /**
         * Consensus-based fusion (requires agreement).
         * Only uses measurements if cameras are within consensus threshold.
         * Rejects outliers automatically.
         */
        CONSENSUS
    }

    // ===== Pinpoint Health Monitoring =====

    /**
     * Whether to enable Pinpoint health monitoring.
     * <p>
     * When enabled, the system monitors Pinpoint for drift and automatically disables
     * Pinpoint corrections when it has drifted too far from the fused pose estimate.
     * </p>
     * <p>
     * This prevents drifted Pinpoint data from corrupting the localization estimate.
     * </p>
     */
    public static final boolean ENABLE_PINPOINT_HEALTH_MONITOR = true;

    /**
     * Whether to enable field boundary checking for Pinpoint odometry.
     * <p>
     * When enabled, Pinpoint readings outside field bounds are considered invalid
     * to prevent impossible positions from corrupting the localization estimate.
     * </p>
     * <p>
     * This can happen if deadwheels slip excessively or if Pinpoint malfunctions.
     * </p>
     */
    public static final boolean ENABLE_PINPOINT_BOUNDARY_CHECK = true;

    /**
     * Maximum allowed deviation between Pinpoint and fused pose before marking as unhealthy (inches).
     * <p>
     * If Pinpoint deviates from the fused pose estimate by more than this amount, it's
     * considered to have drifted. The system will stop using Pinpoint corrections until
     * it returns to acceptable range.</p>
     * <p>
     * <b>Tuning Guide:</b>
     * <ul>
     *   <li><b>3-4 inches:</b> Strict tolerance, catches drift early</li>
     *   <li><b>6-8 inches:</b> Moderate tolerance (recommended)</li>
     *   <li><b>10+ inches:</b> Loose tolerance, allows more drift before disabling</li>
     * </ul>
     * </p>
     * <p>
     * <b>Note:</b> If set too low, normal calibration errors may trigger false positives.
     * If set too high, significant drift may corrupt localization before being detected.
     * </p>
     */
    public static final double PINPOINT_HEALTH_DEVIATION_THRESHOLD = 6.0;

    /**
     * Number of consecutive bad readings before Pinpoint is marked as unhealthy.
     * <p>
     * Prevents false positives from temporary vision noise or AprilTag detection errors.
     * Pinpoint must exceed the deviation threshold for this many consecutive readings
     * before being marked as unhealthy.</p>
     * <p>
     * <b>Typical Values:</b> 3-10 readings.
     * </p>
     * <p>
     * At 50Hz update rate: 5 readings = 0.1 seconds of bad data before triggering.
     * </p>
     */
    public static final int PINPOINT_HEALTH_CONSECUTIVE_BAD_READINGS = 5;

    // ===== Pinpoint Auto-Reset Configuration =====

    /**
     * Whether to enable automatic Pinpoint reset when drifted.
     * <p>
     * When enabled, the system will automatically reset Pinpoint to the fused pose estimate
     * when all of the following conditions are met:</p>
     * <ul>
     *   <li>Pinpoint is marked as unhealthy (drifted)</li>
     *   <li>Robot is stationary (velocity below threshold)</li>
     *   <li>Vision has good data (tag visible + low uncertainty)</li>
     * </ul>
     * <p>
     * This allows Pinpoint to self-heal without manual intervention when it drifts.
     * </p>
     */
    public static final boolean ENABLE_PINPOINT_AUTO_RESET = true;

    /**
     * Maximum robot velocity to be considered "stationary" for auto-reset (inches/second).
     * <p>
     * The robot must be moving slower than this velocity for Pinpoint to be auto-reset.
     * This prevents resetting while the robot is moving, which could cause sudden jumps.</p>
     * <p>
     * <b>Tuning Guide:</b>
     * <ul>
     *   <li><b>0.5 - 1.0:</b> Very strict (robot must be almost stopped)</li>
     *   <li><b>1.0 - 2.0:</b> Moderate (robot is moving slowly)</li>
     *   <li><b>2.0 - 5.0:</b> Loose (allows reset while still moving somewhat)</li>
     * </ul>
     * </p>
     * <p>
     * <b>Recommended:</b> 1.0 inches/second (robot is essentially stopped).
     * </p>
     */
    public static final double PINPOINT_AUTO_RESET_MAX_VELOCITY = 1.0;

    /**
     * Maximum vision position uncertainty for auto-reset (inches).
     * <p>
     * The fused pose uncertainty must be below this threshold for Pinpoint to be auto-reset.
     * This ensures we only reset when vision has a confident, accurate measurement.</p>
     * <p>
     * <b>Tuning Guide:</b>
     * <ul>
     *   <li><b>0.5 - 1.0:</b> Very strict (requires very confident vision)</li>
     *   <li><b>1.0 - 2.0:</b> Moderate (good confidence required)</li>
     *   <li><b>2.0 - 4.0:</b> Loose (allows reset with moderate confidence)</li>
     * </ul>
     * </p>
     * <p>
     * <b>Recommended:</b> 1.5 inches (good vision lock).
     * </p>
     */
    public static final double PINPOINT_AUTO_RESET_MAX_UNCERTAINTY = 1.5;

    // ===== OctoQuad IMU Backup Auto-Reset Configuration =====

    /**
     * Whether to enable automatic OctoQuad IMU offset calibration when used as backup.
     * <p>
     * When enabled, the system will automatically calibrate the OctoQuad IMU offset to match
     * the fused pose estimate when all of the following conditions are met:</p>
     * <ul>
     *   <li>OctoQuad IMU is currently active as backup (Pinpoint failed)</li>
     *   <li>Robot is stationary (velocity below threshold)</li>
     *   <li>Vision has good data (tag visible + low uncertainty)</li>
     * </ul>
     * <p>
     * This keeps the OctoQuad IMU aligned with the true heading while it's being used as backup.
     * </p>
     * <p>
     * <b>Note:</b> OctoQuad IMU cannot be reset to an arbitrary heading like Pinpoint.
     * Instead, we maintain a software offset that's applied to the raw OctoQuad IMU reading.
     * </p>
     */
    public static final boolean ENABLE_OCTOQUAD_IMU_AUTO_CALIBRATE = true;

    /**
     * Maximum robot velocity to be considered "stationary" for OctoQuad IMU calibration (inches/second).
     * <p>
     * The robot must be moving slower than this velocity for OctoQuad IMU to be calibrated.
     * This prevents calibration while the robot is moving, which could cause sudden jumps.</p>
     * <p>
     * <b>Recommended:</b> 0.5-1.0 inches/second (very slow movement)</p>
     */
    public static final double OCTOQUAD_IMU_CALIBRATE_MAX_VELOCITY = 1.0;

    /**
     * Maximum vision position uncertainty for OctoQuad IMU calibration (inches).
     * <p>
     * The fused pose uncertainty must be below this threshold for OctoQuad IMU to be calibrated.
     * This ensures we only calibrate when vision has a confident, accurate measurement.</p>
     * <p>
     * <b>Recommended:</b> 1.0-2.0 inches (requires good vision lock)</p>
     */
    public static final double OCTOQUAD_IMU_CALIBRATE_MAX_UNCERTAINTY = 1.5;

    /**
     * Maximum heading difference between OctoQuad IMU and fused pose for calibration (radians).
     * <p>
     * If the OctoQuad IMU heading differs from the fused pose by more than this amount,
     * the calibration is skipped to prevent calibrating to bad data.</p>
     * <p>
     * <b>Recommended:</b> 0.1-0.2 radians (~5-10 degrees)</p>
     */
    public static final double OCTOQUAD_IMU_CALIBRATE_MAX_HEADING_DIFF = 0.15;

    // ===== Tuning Constants =====

    /**
     * Steering PID tuning parameters.
     * These constants control the automatic steering servo tuning process.
     */
    public static class SteeringTuning {
        /** Module names for display during tuning */
        public static final String[] MODULE_NAMES = {"Front-Right", "Front-Left", "Back-Left", "Back-Right"};

        /** Angle to test for velocity response (90 degrees) */
        public static final double VELOCITY_TEST_ANGLE = Math.PI / 2;

        /** Maximum time to wait for velocity response (seconds) */
        public static final double MAX_VELOCITY_TEST_TIME = 2.0;

        /** Starting P gain for steering */
        public static final double P_START = 0.1;

        /** P gain increment per step */
        public static final double P_INCREMENT = 0.05;

        /** Maximum P gain to test */
        public static final double P_MAX = 2.0;

        /** Starting D gain for steering */
        public static final double D_START = 0.0;

        /** D gain increment per step */
        public static final double D_INCREMENT = 0.001;

        /** Maximum D gain to test */
        public static final double D_MAX = 0.1;

        /** Starting I gain for steering */
        public static final double I_START = 0.0;

        /** I gain increment per step */
        public static final double I_INCREMENT = 0.0001;

        /** Maximum I gain to test */
        public static final double I_MAX = 0.01;

        /** Error threshold for steady state detection (radians) */
        public static final double STEADY_STATE_ERROR_THRESHOLD = 0.02;

        /** Angle step for step response test */
        public static final double STEP_ANGLE = Math.PI / 4;

        /** Threshold for detecting overshoot (radians) */
        public static final double OVERSHOOT_THRESHOLD = 0.05;

        /** Maximum time for settling (seconds) */
        public static final double SETTLING_TIME_THRESHOLD = 0.5;

        /** kS increment for feedforward tuning */
        public static final double KS_INCREMENT = 0.005;

        /** Maximum power to test for kS */
        public static final double KS_MAX_POWER = 0.1;

        /** Velocity threshold for kS detection (in/s) */
        public static final double KS_VELOCITY_THRESHOLD = 0.1;

        /** Maximum total tuning time (seconds) */
        public static final double MAX_TOTAL_TUNING_TIME = 120.0;
    }

    /**
     * Drive feedforward tuning parameters.
     * These constants control the automatic drive motor feedforward tuning process.
     */
    public static class FeedforwardTuning {
        /** Maximum power to test for kS (static friction) */
        public static final double KS_MAX_POWER = 0.30;

        /** Power increment for kS testing */
        public static final double KS_INCREMENT = 0.005;

        /** Velocity threshold to detect robot movement (in/s) */
        public static final double KS_VELOCITY_THRESHOLD = 5.0;

        /** Power levels to test for kV (velocity constant) */
        public static final double[] KV_TEST_POWERS = {0.2, 0.4, 0.6, 0.8};

        /** Number of samples to average for kV calculation */
        public static final int KV_SAMPLES = 20;

        /** Delay before measuring kV (samples) */
        public static final int KV_STEADY_STATE_DELAY = 10;

        /** Maximum travel distance before turning around (inches) */
        public static final double MAX_TRAVEL_DISTANCE = 120.0;

        /** Maximum time per power level (seconds) */
        public static final double MAX_TIME_PER_LEVEL = 3.0;

        /** Maximum total tuning time (seconds) */
        public static final double MAX_TOTAL_TUNING_TIME = 60.0;

        /** Velocity threshold to detect if robot is stuck (in/s) */
        public static final double STUCK_VELOCITY_THRESHOLD = 1.0;
    }

    /**
     * Automatic feedforward tuning parameters.
     * These constants control the automated feedforward tuning OpMode.
     */
    public static class AutoFeedforwardTuning {
        /** Maximum power to test for kS */
        public static final double KS_MAX_POWER = 0.30;

        /** Power increment step */
        public static final double KS_POWER_INCREMENT = 0.01;

        /** Velocity threshold for kS detection (in/s) */
        public static final double KS_VELOCITY_THRESHOLD = 1.0;

        /** Power levels to test for kV */
        public static final double[] KV_TEST_POWERS = {0.2, 0.4, 0.6, 0.8};

        /** Samples per power level */
        public static final int KV_SAMPLES_PER_LEVEL = 10;

        /** Samples to wait for steady state */
        public static final double KV_STEADY_STATE_SAMPLES = 5.0;
    }

    /**
     * Hardware constants for external devices.
     */
    public static class HardwareConstants {
        /** Conversion factor: degrees to radians */
        public static final double DEG_TO_RAD = Math.PI / 180.0;

        /** SRSHub I2C address */
        public static final int SRS_HUB_I2C_ADDRESS = 0x57;

        /** SRSHub device ID */
        public static final int SRS_HUB_DEVICE_ID = 0x61;

        /** SRSHub firmware version */
        public static final int SRS_HUB_FIRMWARE_MAJOR = 1;
        public static final int SRS_HUB_FIRMWARE_MINOR = 4;
        public static final int SRS_HUB_FIRMWARE_PATCH = 4;

        /** GoBilda Pinpoint tick-to-unit conversion factors */
        public static final double PINPOINT_TICKS_PER_INCH_X = 8192.0 / 50.0;  // Default: ~163.84
        public static final double PINPOINT_TICKS_PER_INCH_Y = 8192.0 / 50.0;  // Default: ~163.84

        /** OctoQuad firmware version */
        public static final int OCTOQUAD_FIRMWARE_VERSION = 3;

        /** OctoQuad I2C addresses */
        public static final int OCTOQUAD_I2C_ADDRESS_BASE = 0x30;
        public static final int OCTOQUAD_I2C_ADDRESS_FRONT = 0x30;
        public static final int OCTOQUAD_I2C_ADDRESS_REAR = 0x31;

        /** OctoQuad position/velocity scalar factors */
        public static final double OCTOQUAD_POSITION_SCALAR = 1.0;
        public static final double OCTOQUAD_VELOCITY_SCALAR = 1.0;
    }
}
