package org.firstinspires.ftc.teamcode.commandbase.subsystems.swerve;

import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import org.firstinspires.ftc.teamcode.util.drivers.OctoQuadFWv3;
import org.firstinspires.ftc.teamcode.util.control.SlewRateLimiter;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.swerve.encoders.SwerveEncoder;
import com.seattlesolvers.solverslib.hardware.motors.CRServoEx;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.geometry.Vector2d;
import com.seattlesolvers.solverslib.kinematics.wpilibkinematics.SwerveModuleState;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.kinematics.wpilibkinematics.ChassisSpeeds;

import static org.firstinspires.ftc.teamcode.globals.Constants.*;

/**
 * Individual swerve module with independent steering and drive control.
 *
 * <p>A swerve module is a wheel assembly that can independently:</p>
 * <ul>
 *   <li><b>Steer:</b> Rotate to any angle (±π radians from forward)</li>
 *   <li><b>Drive:</b> Spin at variable speeds in either direction</li>
 * </ul>
 *
 * <p>This combination allows the robot to move in any direction while rotating, enabling
 * holonomic motion. Four modules working together provide full omnidirectional control.</p>
 *
 * <h3>Hardware Components:</h3>
 * <ul>
 *   <li><b>Drive Motor:</b> MotorEx controlling wheel rotation speed (velocity control)</li>
 *   <li><b>Steering Servo:</b> CRServoEx controlling wheel orientation (position control)</li>
 *   <li><b>OctoQuad:</b> Encoder processor reading steering and drive encoder positions</li>
 * </ul>
 *
 * <h3>Control Strategy:</h3>
 * <p>This module uses a state optimization approach for efficient steering:</p>
 * <ol>
 *   <li>Calculate desired velocity vector from chassis speeds</li>
 *   <li>Determine target angle and magnitude</li>
 *   <li><b>Optimize:</b> If the angle error > 90°, reverse wheel direction and flip angle 180°.
 *       This reduces steering travel by taking the "short path" to the target heading.</li>
 *   <li>Apply PIDF control for steering (position) and drive (velocity)</li>
 * </ol>
 *
 * <h3>State Optimization Example:</h3>
 * <p>If the wheel is at 0° (forward) and needs to go 270° (left):</p>
 * <ul>
 *   <li><b>Without optimization:</b> Rotate 270° counter-clockwise</li>
 *   <li><b>With optimization:</b> Rotate 90° clockwise and reverse wheel direction</li>
 * </ul>
 * <p>This results in faster response and less wear on steering mechanisms.</p>
 *
 * <h3>Encoder Configuration:</h3>
 * <p>The OctoQuad encoder is assumed to be a REV Through-Bore encoder with:</p>
 * <ul>
 *   <li><b>Steering:</b> 8192 counts per revolution (absolute positioning)</li>
 *   <li><b>Drive:</b> Velocity in ticks/second (requires conversion to inches/second)</li>
 * </ul>
 *
 * <h3>PIDF Control:</h3>
 * <ul>
 *   <li><b>Steering PIDF:</b> Position control for steering servo. Compares target angle
 *       vs actual angle from OctoQuad.</li>
 *   <li><b>Drive PIDF:</b> Velocity control for drive motor. Combines feed-forward (target
 *       speed normalized) with PID correction (velocity error).</li>
 * </ul>
 *
 * <h3>Tuning Notes:</h3>
 * <p>The velocity-to-inches conversion factor (0.001) is a placeholder. For accurate
 * velocity control, replace with:</p>
 * <pre>
 * WHEEL_RADIUS * 2 * π / TICKS_PER_REV
 * </pre>
 * <p>This ensures the drive PID controller operates on physically meaningful units.</p>
 *
 * @see OctoSwerveDrivetrain
 * @see PIDFController
 * @see org.firstinspires.ftc.teamcode.util.OctoQuadFWv3
 */
public class OctoSwerveModule {

    /**
     * Drive motor controlling wheel rotation speed.
     * Operates in RawPower mode with manual velocity PIDF control.
     */
    private final MotorEx driveMotor;

    /**
     * Steering servo controlling wheel orientation.
     * Operates in RawPower mode with manual position PIDF control.
     */
    private final CRServoEx steerServo;

    /**
     * OctoQuad encoder processor (Firmware v3) for reading module positions and velocities.
     * Provides high-precision feedback for PIDF control with CRC validation.
     */
    private final OctoQuadFWv3 octoQuad;

    /**
     * OctoQuad port for the steering encoder (absolute position).
     * Typically ports 0-3 for the four modules.
     */
    private final int steerPort;

    /**
     * OctoQuad port for the drive wheel encoder (velocity).
     * Typically ports 4-7 for the four modules.
     */
    private final int drivePort;

    /**
     * Module position relative to robot center (inches).
     * Used for inverse kinematics calculations.
     * Example: Front-right module = (+trackWidth/2, +wheelBase/2)
     */
    private final Vector2d location;

    /**
     * PIDF controller for steering servo position control.
     * Compares target angle vs actual angle and outputs servo power.
     */
    private final PIDFController steerPID;

    /**
     * PIDF controller for drive motor velocity control.
     * Combines feed-forward with closed-loop correction.
     */
    private final PIDFController drivePID;

    /**
     * Maximum achievable velocity for this module (inches/second).
     * Used for normalizing target speeds and velocity desaturation.
     */
    private final double maxSpeed;

    /**
     * Encoder offset calibration value (radians).
     * Adjusted so that 0 radians corresponds to wheel pointing forward.
     * Set via {@link #setOffset(double)}.
     */
    private double offsetRadiant = 0;

    /**
     * Slew rate limiter for drive motor power.
     * Prevents rapid power changes that can cause current spikes and trip the 20A fuse.
     * Can be disabled via Constants.ENABLE_SLEW_RATE_LIMIT.
     */
    private final SlewRateLimiter slewRateLimiter;

    /**
     * Whether slew rate limiting is enabled for this module.
     * Controlled by Constants.ENABLE_SLEW_RATE_LIMIT.
     */
    private final boolean slewRateEnabled;

    // ----- Telemetry Caches -----

    /**
     * Cached target steering angle for telemetry output (radians).
     * Updated each time {@link #updateModuleWithVelocity(Vector2d)} is called.
     */
    private double currentTargetAngleRadians = 0;

    /**
     * Cached target velocity magnitude for telemetry output (inches/second).
     * Updated each time {@link #updateModuleWithVelocity(Vector2d)} is called.
     */
    private double currentTargetMagnitude = 0;

    /**
     * Cached current drive velocity from OctoQuad (ticks/second).
     * Read each update loop and converted to inches/second for control.
     */
    private double currentVelocityTicksPerSec = 0;

    /**
     * Constructs a swerve module with hardware, PIDF controllers, and configuration.
     *
     * <p>The module is initialized in RawPower mode for both motor and servo, allowing
     * manual PIDF control rather than using the built-in SolversLib position/velocity modes.
     * This is necessary because the OctoQuad provides encoder feedback, not the motor
     * encoders.</p>
     *
     * @param driveMotor motor controlling wheel rotation speed
     * @param steerServo servo controlling wheel orientation
     * @param octoQuad OctoQuad Firmware v3 encoder processor instance
     * @param steerPort OctoQuad port for steering encoder (0-3)
     * @param drivePort OctoQuad port for drive encoder (4-7)
     * @param location module position relative to robot center (inches)
     * @param maxSpeed maximum module velocity (inches/second)
     * @param swervoPIDFCoefficients PIDF gains for steering position control
     * @param drivePIDFCoefficients PIDF gains for drive velocity control
     */
    public OctoSwerveModule(MotorEx driveMotor, CRServoEx steerServo, OctoQuadFWv3 octoQuad, int steerPort, int drivePort, Vector2d location, double maxSpeed, PIDFCoefficients swervoPIDFCoefficients, PIDFCoefficients drivePIDFCoefficients) {
        this.driveMotor = driveMotor;
        this.steerServo = steerServo;
        this.octoQuad = octoQuad;
        this.steerPort = steerPort;
        this.drivePort = drivePort;
        this.location = location;
        this.maxSpeed = maxSpeed;

        // Disable SolversLib internal Positional Control because we manage it here with the OctoQuad
        this.steerServo.setRunMode(CRServoEx.RunMode.RawPower);
        this.driveMotor.setRunMode(MotorEx.RunMode.RawPower);

        this.steerPID = new PIDFController(swervoPIDFCoefficients);
        this.drivePID = new PIDFController(drivePIDFCoefficients);

        // Initialize slew rate limiter for drive motor current spike prevention
        this.slewRateEnabled = ENABLE_SLEW_RATE_LIMIT;
        this.slewRateLimiter = new SlewRateLimiter(DRIVE_SLEW_RATE_LIMIT, 0.0);
    }

    /**
     * Sets the encoder offset calibration value for this module.
     *
     * <p>The offset adjusts the zero point of the steering encoder so that 0 radians
     * corresponds to the wheel pointing perfectly forward. This is essential for
     * accurate kinematics and consistent behavior across modules.</p>
     *
     * <h3>Calibration Procedure:</h3>
     * <ol>
     *   <li>Physically align the wheel to point straight forward (parallel to robot sides)</li>
     *   <li>Read the raw encoder value: {@code octoQuad.readSinglePosition(steerPort)}</li>
     *   <li>Convert to radians: {@code (ticks % 8192) / 8192.0 * 2π}</li>
     *   <li>Set this offset to the negative of that reading</li>
     *   <li>Verify: {@code getModuleHeadingRadians()} should now return ~0 when wheel is forward</li>
     * </ol>
     *
     * @param offsetRadiant the offset value in radians (subtracted from raw reading)
     * @see #getModuleHeadingRadians()
     */
    public void setOffset(double offsetRadiant) {
        this.offsetRadiant = offsetRadiant;
    }

    /**
     * Gets the current steering angle of the module in radians.
     *
     * <p>This method reads the OctoQuad encoder, converts the position to radians,
     * applies the offset calibration, and returns the result. The angle is normalized
     * to the range [-π, π] where:</p>
     * <ul>
     *   <li>0 radians = wheel pointing forward</li>
     *   <li>+π/2 radians = wheel pointing left</li>
     *   <li>-π/2 radians = wheel pointing right</li>
     *   <li>±π radians = wheel pointing backward</li>
     * </ul>
     *
     * <h3>Encoder Configuration:</h3>
     * <p>Assumes a REV Through-Bore encoder with 8192 counts per revolution.</p>
     * <pre>
     * ticks = octoQuad.readSinglePosition(steerPort)
     * rads = (ticks % 8192) / 8192.0 * 2π
     * heading = rads - offset
     * </pre>
     *
     * <p><b>Note:</b> The angle is not normalized to [-π, π] in this method. Callers
     * should normalize if needed for calculations.</p>
     *
     * @return the module heading in radians (0 = forward, positive = counter-clockwise)
     * @see #setOffset(double)
     */
    public double getModuleHeadingRadians() {
        // OctoQuad counts per revolution is typically 8192 for REV Through-bore or 4096 for standard, depending on attached encoder
        // Assume 8192 counts per Rev for REV Through Bore Encoder
        int ticks = octoQuad.readSinglePosition(steerPort);
        double rads = ((ticks % 8192) / 8192.0) * 2 * Math.PI;
        return rads - offsetRadiant;
    }

    /**
     * Gets the current drive wheel velocity in inches per second.
     *
     * <p>This method converts the raw OctoQuad velocity reading (ticks/second at the motor) to
     * physical units (inches/second at the wheel) accounting for gear reduction.</p>
     *
     * <h3>Conversion Formula:</h3>
     * <p>Uses the physical wheel and gear parameters from {@link org.firstinspires.ftc.teamcode.globals.Constants#VELOCITY_CONVERSION}:</p>
     * <pre>
     * velocity_inches_per_sec = motor_ticks_per_sec × VELOCITY_CONVERSION
     * = motor_ticks_per_sec × (WHEEL_CIRCUMFERENCE / TOTAL_GEAR_RATIO) / OCTOQUAD_CPR
     * </pre>
     *
     * <p><b>Your Gearbox:</b></p>
     * <ul>
     *   <li>Motor: GoBilda 5.18:1 internal ratio</li>
     *   <li>Stage 1: 36:24 (1.5:1 reduction)</li>
     *   <li>Stage 2: 2:1 reduction</li>
     *   <li>Total: 15.54:1 gear reduction</li>
     * </ul>
     *
     * <p><b>For 72mm wheels with your gearing:</b></p>
     * <pre>
     * Wheel circumference: 8.906 inches
     * Effective circumference after gearing: 8.906 / 15.54 = 0.573 inches per motor rev
     * Conversion: 0.573 / 8192 = 0.00007 inches/tick
     * </pre>
     *
     * <p><b>Examples:</b></p>
     * <ul>
     *   <li>10,000 ticks/sec → 0.70 inches/sec</li>
     *   <li>50,000 ticks/sec → 3.50 inches/sec</li>
     *   <li>100,000 ticks/sec → 7.00 inches/sec</li>
     * </ul>
     *
     * <p><b>Performance Note:</b> Due to the 15.54:1 gear reduction, you have high torque
     * but lower top speed. The encoder reads motor speed, so the conversion must account
     * for the gear ratio.</p>
     *
     * @return the current wheel velocity in inches/second
     * @see org.firstinspires.ftc.teamcode.globals.Constants#VELOCITY_CONVERSION
     * @see org.firstinspires.ftc.teamcode.globals.Constants#TOTAL_GEAR_RATIO
     * @see #updateModuleWithVelocity(Vector2d)
     */
    public double getCurrentVelocityInchesPerSec() {
        // Convert motor ticks/sec to wheel inches/sec using gear ratio
        return currentVelocityTicksPerSec * org.firstinspires.ftc.teamcode.globals.Constants.VELOCITY_CONVERSION;
    }

    /**
     * Gets the target steering angle for this module.
     *
     * <p>Returns the most recent target angle set by {@link #updateModuleWithVelocity(Vector2d)}.
     * Useful for telemetry to compare target vs actual angle.</p>
     *
     * @return the target steering angle in radians (normalized to [-π, π])
     */
    public double getTargetAngleRadians() {
        return currentTargetAngleRadians;
    }

    /**
     * Gets the target velocity magnitude for this module.
     *
     * <p>Returns the most recent target velocity set by {@link #updateModuleWithVelocity(Vector2d)}.
     * Useful for telemetry to compare target vs actual velocity.</p>
     *
     * @return the target velocity magnitude in inches/second
     */
    public double getTargetMagnitude() {
        return currentTargetMagnitude;
    }

    /**
     * Gets the module's position relative to the robot center.
     *
     * <p>Returns the 2D position vector of this module. This is used by
     * {@link OctoSwerveDrivetrain} for inverse kinematics calculations to determine
     * each module's contribution to chassis motion.</p>
     *
     * @return the module's position as a Vector2d (inches, relative to robot center)
     */
    public Vector2d getLocation() {
        return location;
    }

    /**
     * Calculates the module's velocity vector for given chassis speeds using inverse kinematics.
     *
     * <p>This method transforms high-level chassis velocities into a 2D velocity vector
     * for this specific module. The calculation accounts for:</p>
     * <ul>
     *   <li><b>Linear Velocities:</b> Forward (vx) and lateral (vy) chassis motion</li>
     *   <li><b>Rotational Velocity:</b> Chassis rotation (omega) creates module motion
     *       proportional to the module's distance from the robot center</li>
     * </ul>
     *
     * <h3>Inverse Kinematics Equation:</h3>
     * <p>The module velocity vector is:</p>
     * <pre>
     * module_vx = chassis_vx - omega * module_y
     * module_vy = chassis_vy + omega * module_x
     * </pre>
     * <p>Where:</p>
     * <ul>
     *   <li>chassis_vx, chassis_vy = linear chassis velocities</li>
     *   <li>omega = rotational velocity (radians/sec)</li>
     *   <li>module_x, module_y = this module's position from robot center</li>
     * </ul>
     *
     * <h3>Physical Interpretation:</h3>
     * <ul>
     *   <li>Chassis forward motion → all modules drive forward</li>
     *   <li>Chassis rotation → modules drive perpendicular to their radius vector</li>
     *   <li>Combined motion → superposition of both effects</li>
     * </ul>
     *
     * @param targetVelocity the desired chassis velocities (vx, vy, omega)
     * @return the module's velocity vector (vx, vy in inches/second)
     * @see OctoSwerveDrivetrain#drive(ChassisSpeeds)
     */
    public Vector2d calculateVectorRobotCentric(ChassisSpeeds targetVelocity) {
        double vX = targetVelocity.vxMetersPerSecond - targetVelocity.omegaRadiansPerSecond * location.getY();
        double vY = targetVelocity.vyMetersPerSecond + targetVelocity.omegaRadiansPerSecond * location.getX();
        return new Vector2d(vX, vY);
    }

    /**
     * Updates the module with a target velocity vector, applying steering optimization and PIDF control.
     *
     * <p>This is the primary control method for the module. It takes a target velocity vector,
     * optimizes the steering angle to minimize rotation, and applies PIDF control to both
     * steering and drive motors.</p>
     *
     * <h3>Control Sequence:</h3>
     * <ol>
     *   <li><b>Extract Target:</b> Parse magnitude and angle from velocity vector</li>
     *   <li><b>Read Current State:</b> Get current steering angle from OctoQuad</li>
     *   <li><b>Normalize Angle:</b> Ensure current angle is in [-π, π] range</li>
     *   <li><b>Calculate Error:</b> Determine angle difference from target to current</li>
     *   <li><b>Optimize State:</b> If error > 90°, reverse direction and flip angle 180°</li>
     *   <li><b>Steering Control:</b> Apply PIDF to angle error, send power to servo</li>
     *   <li><b>Drive Control:</b> Read velocity, apply feed-forward + PIDF, send power to motor</li>
     *   <li><b>Slew Rate Limit:</b> Limit rate of power change to prevent current spikes (if enabled)</li>
     * </ol>
     *
     * <h3>State Optimization:</h3>
     * <p>To minimize steering travel, the module can reverse wheel direction instead of
     * rotating the long way around. This happens when the angle error > π/2 (90°):</p>
     * <ul>
     *   <li>Reverse target magnitude (multiply by -1)</li>
     *   <li>Flip angle by π (180°)</li>
     *   <li>Result: Same velocity vector, 90° less steering rotation</li>
     * </ul>
     * <p><b>Example:</b> Wheel at 0°, target 270° (left). Error = 270°.</p>
     * <ul>
     *   <li><b>Without optimization:</b> Rotate 270° CCW</li>
     *   <li><b>With optimization:</b> Rotate 90° CW, reverse wheel, result is same velocity</li>
     * </ul>
     *
     * <h3>Steering Control:</h3>
     * <p>Uses position PIDF on the angle error:</p>
     * <pre>
     * steerPower = steerPID.calculate(0, angleError)
     * servo.set(steerPower)
     * </pre>
     * <p>The PIDF controller outputs power [-1, 1] to steer toward the target angle.</p>
     *
     * <h3>Drive Control:</h3>
     * <p>Combines feed-forward with closed-loop correction:</p>
     * <pre>
     * targetNormalized = targetMagnitude / maxSpeed
     * feedForward = targetNormalized
     * velocityError = targetMagnitude - currentVelocity
     * pidCorrection = drivePID.calculate(0, -velocityError)
     * motorPower = feedForward + pidCorrection
     * </pre>
     * <ul>
     *   <li><b>Feed-Forward:</b> Baseline power to achieve target speed</li>
     *   <li><b>PIDF Correction:</b> Adjusts for error between target and actual velocity</li>
     *   <li><b>Velocity Reading:</b> Current velocity from OctoQuad (converted to inches/sec)</li>
     * </ul>
     *
     * <h3>Slew Rate Limiting:</h3>
     * <p>If enabled (Constants.ENABLE_SLEW_RATE_LIMIT), the drive motor power is slew rate
     * limited to prevent rapid power changes that can cause current spikes and trip the 20A fuse.</p>
     *
     * <h3>Tuning Notes:</h3>
     * <ul>
     *   <li><b>Steering PIDF:</b> High P for fast response, D to reduce overshoot</li>
     *   <li><b>Drive PIDF:</b> F should match motor power needed, P/D for velocity tracking</li>
     *   <li><b>Velocity Conversion:</b> Update the 0.001 factor with your wheel's physical specs</li>
     *   <li><b>Slew Rate:</b> Adjust DRIVE_SLEW_RATE_LIMIT in Constants (2.0-4.0 typical)</li>
     * </ul>
     *
     * @param targetVector the desired velocity vector (inches/second) for this module
     * @see #calculateVectorRobotCentric(ChassisSpeeds)
     * @see PIDFController
     */
    public void updateModuleWithVelocity(Vector2d targetVector) {
        double targetMagnitude = targetVector.magnitude();
        double targetAngle = Math.atan2(targetVector.getY(), targetVector.getX());

        double currentAngle = getModuleHeadingRadians();
        
        // Normalize Angle to -PI to PI
        while (currentAngle > Math.PI) currentAngle -= 2 * Math.PI;
        while (currentAngle <= -Math.PI) currentAngle += 2 * Math.PI;

        double error = targetAngle - currentAngle;
        
        // Optimize short path logic
        while (error > Math.PI) error -= 2 * Math.PI;
        while (error <= -Math.PI) error += 2 * Math.PI;

        if (Math.abs(error) > Math.PI / 2) {
            targetMagnitude *= -1;
            error -= Math.copySign(Math.PI, error);
            
            // Adjust cached target angle for accurate graphing (flips 180 degrees)
            targetAngle -= Math.copySign(Math.PI, error);
            while (targetAngle > Math.PI) targetAngle -= 2 * Math.PI;
            while (targetAngle <= -Math.PI) targetAngle += 2 * Math.PI;
        }

        this.currentTargetAngleRadians = targetAngle;
        this.currentTargetMagnitude = targetMagnitude;

        // Steer Power
        double steerPower = steerPID.calculate(0, error);
        steerServo.set(steerPower);

        // Drive Power (Velocity Control scaled to Ticks/sec proxy or Normalized Value)
        currentVelocityTicksPerSec = octoQuad.readSingleVelocity(drivePort);

        // As a simplified proxy for standard Motor operation, normalize target speed against max:
        double targetSpeedNormalized = targetMagnitude / maxSpeed;

        // Applying a basic Velocity PID constraint around the desired target normalized vs the measured tick velocity
        // To accurately tune this, you'll want to multiply currentVelocityTicksPerSec by your WHEEL_CIRCUMFERENCE / TICKS_PER_REV.
        // For now, we apply FeedForward (targetSpeedNormalized) + closed-loop adjustment
        double feedForward = targetSpeedNormalized;

        // You will need to build the conversion formula into this error metric for the drivePID to compute physically accurate values!
        double driveError = targetMagnitude - (currentVelocityTicksPerSec * 0.001); // 0.001 is a placeholder TICKS_TO_INCHES ratio
        double drivePIDCorrection = drivePID.calculate(0, -driveError);

        double motorPower = feedForward + drivePIDCorrection;

        // Apply slew rate limiting to prevent current spikes (if enabled)
        // Note: This assumes ~50Hz loop (0.02s per iteration). For accurate timing, pass actual time delta.
        if (slewRateEnabled) {
            motorPower = slewRateLimiter.calculate(motorPower, 0.02);  // 0.02s = 50Hz loop
        }

        driveMotor.set(motorPower);
    }

    /**
     * Stops all module motion by setting both steering servo and drive motor power to zero.
     *
     * <p>This method immediately stops the module by:</p>
     * <ul>
     *   <li>Sending zero power to the steering servo (holds current position)</li>
     *   <li>Sending zero power to the drive motor (coasts to stop)</li>
     * </ul>
     *
     * <p><b>Behavior:</b> The wheel will stop accelerating but may coast briefly due to
     * momentum. The steering angle remains locked at the last commanded position.</p>
     *
     * <p><b>Use Cases:</b></p>
     * <ul>
     *   <li>Emergency stop when robot should not move</li>
     *   <li>End of match (auto/teleop complete)</li>
     *   <li>Safety interlock triggered</li>
     *   <li>Robot disabled</li>
     * </ul>
     *
     * @see OctoSwerveDrivetrain#stop()
     */
    public void stop() {
        steerServo.set(0);
        driveMotor.set(0);
    }
}
