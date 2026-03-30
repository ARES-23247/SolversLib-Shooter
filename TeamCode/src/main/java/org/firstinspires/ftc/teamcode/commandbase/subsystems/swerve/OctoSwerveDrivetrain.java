package org.firstinspires.ftc.teamcode.commandbase.subsystems.swerve;

import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import org.firstinspires.ftc.teamcode.util.drivers.OctoQuadFWv3;
import org.firstinspires.ftc.teamcode.util.control.CurrentLimiter;
import com.seattlesolvers.solverslib.hardware.motors.CRServoEx;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.seattlesolvers.solverslib.geometry.Vector2d;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.pedropathing.localization.Pose;
import org.firstinspires.ftc.teamcode.util.DataLogger;

import static org.firstinspires.ftc.teamcode.globals.Constants.*;

/**
 * Four-module coaxial swerve drivetrain controller with inverse kinematics and velocity management.
 *
 * <p>This class manages four swerve modules to achieve holonomic (omnidirectional) motion.
 * It translates high-level chassis velocity commands into individual module steering angles
 * and drive wheel speeds using swerve inverse kinematics.</p>
 *
 * <h3>Swerve Drive Overview:</h3>
 * <p>Swerve drive allows each wheel to independently:</p>
 * <ul>
 *   <li><b>Steer:</b> Rotate to any angle (typically ±180° from forward)</li>
 *   <li><b>Drive:</b> Spin at variable speeds (forward or reverse)</li>
 * </ul>
 * <p>This combination enables the robot to move in any direction while rotating, providing
 * maximum mobility on the FTC field.</p>
 *
 * <h3>Module Configuration:</h3>
 * <p>The drivetrain consists of four modules positioned at the corners of the robot:</p>
 * <ul>
 *   <li><b>Index 0 - Front-Right (FR):</b> (+trackWidth/2, +wheelBase/2)</li>
 *   <li><b>Index 1 - Front-Left (FL):</b> (+trackWidth/2, -wheelBase/2)</li>
 *   <li><b>Index 2 - Back-Left (BL):</b> (-trackWidth/2, -wheelBase/2)</li>
 *   <li><b>Index 3 - Back-Right (BR):</b> (-trackWidth/2, +wheelBase/2)</li>
 * </ul>
 *
 * <h3>Inverse Kinematics:</h3>
 * <p>When {@link #drive(ChassisSpeeds)} is called, this class:</p>
 * <ol>
 *   <li>Calculates target velocity vectors for each module based on chassis speeds</li>
 *   <li>Desaturates velocities if any module exceeds maximum speed</li>
 *   <li>Sends steering angle and drive speed commands to each module</li>
 * </ol>
 *
 * <h3>Velocity Desaturation:</h3>
 * <p>If the requested chassis speeds would require any module to exceed {@code maxSpeed},
 * all module velocities are scaled down proportionally. This preserves the <b>direction</b>
 * of the motion while reducing the <b>magnitude</b> to within physical limits.</p>
 *
 * <p><b>Algorithm:</b></p>
 * <ol>
 *   <li>Calculate all module velocity vectors</li>
 *   <li>Find the maximum velocity magnitude across all modules</li>
 *   <li>If max > {@code maxSpeed}, scale all vectors by {@code maxSpeed / maxVelocity}</li>
 *   <li>Apply the scaled vectors to each module</li>
 * </ol>
 *
 * <h3>Telemetry & Visualization:</h3>
 * <p>This class provides real-time telemetry for tuning and debugging:</p>
 * <ul>
 *   <li><b>Target vs Actual:</b> Compare commanded vs achieved angles and velocities</li>
 *   <li><b>Field Overlay:</b> Draw wheel orientation indicators on FTC Dashboard</li>
 *   <li><b>CSV Logging:</b> Record module states for post-match analysis</li>
 * </ul>
 *
 * @see OctoSwerveModule
 * @see ChassisSpeeds
 * @see org.firstinspires.ftc.teamcode.commandbase.subsystems.Drive
 */
public class OctoSwerveDrivetrain {

    /**
     * Array of four swerve modules. Indices correspond to:
     * 0 = Front-Right, 1 = Front-Left, 2 = Back-Left, 3 = Back-Right
     */
    private final OctoSwerveModule[] modules = new OctoSwerveModule[4];

    /**
     * Maximum velocity any module can achieve in inches per second.
     * Used for velocity desaturation to prevent commanding motors beyond their limits.
     */
    private final double maxSpeed;

    /**
     * Maximum angular velocity of the robot in radians per second.
     * Calculated from maxSpeed and robot dimensions.
     */
    private final double maxAngularSpeed;

    /**
     * Current limiter for preventing fuse trips based on Floodgate sensor.
     * Scales down chassis speeds when current approaches the 20A fuse limit.
     * Can be disabled via Constants.ENABLE_CURRENT_LIMIT.
     */
    private final CurrentLimiter currentLimiter;

    /**
     * Whether current limiting is enabled.
     * Controlled by Constants.ENABLE_CURRENT_LIMIT.
     */
    private final boolean currentLimitEnabled;

    /**
     * Constructs a swerve drivetrain with four modules.
     *
     * <p>This constructor initializes all four swerve modules with their hardware,
     * PIDF controllers, and physical positions. Each module is configured with:</p>
     * <ul>
     *   <li>Drive motor with velocity PIDF control</li>
     *   <li>Steering servo with position PIDF control</li>
     *   <li>OctoQuad encoder for position feedback</li>
     *   <li>Module offset for zero-point calibration</li>
     *   <li>Position relative to robot center (for kinematics)</li>
     * </ul>
     *
     * <h3>OctoQuad Port Mapping:</h3>
     * <p>The OctoQuad has 8 encoder ports. This class assumes:</p>
     * <ul>
     *   <li>Ports 0-3: Steering encoders (one per module)</li>
     *   <li>Ports 4-7: Drive wheel encoders (one per module)</li>
     * </ul>
     *
     * <h3>Module Position Vectors:</h3>
     * <p>Each module's position is specified relative to the robot center:</p>
     * <ul>
     *   <li>Front-Right: (+trackWidth/2, +wheelBase/2)</li>
     *   <li>Front-Left: (+trackWidth/2, -wheelBase/2)</li>
     *   <li>Back-Left: (-trackWidth/2, -wheelBase/2)</li>
     *   <li>Back-Right: (-trackWidth/2, +wheelBase/2)</li>
     * </ul>
     *
     * @param trackWidth distance between left and right wheels (inches)
     * @param wheelBase distance between front and back wheels (inches)
     * @param maxSpeed maximum module velocity (inches/second)
     * @param swervoPIDFCoefficients PIDF gains for steering servo position control
     * @param drivePIDFCoefficients PIDF gains for drive motor velocity control
     * @param motors array of 4 drive motors (must be exactly 4)
     * @param swervos array of 4 steering servos (must be exactly 4)
     * @param octoQuad OctoQuad Firmware v3 encoder processor instance
     * @param moduleOffsets array of 4 encoder offsets (radians) for zero calibration
     * @throws IllegalArgumentException if motors, swervos, or moduleOffsets arrays are not length 4
     */
    public OctoSwerveDrivetrain(double trackWidth, double wheelBase, double maxSpeed, PIDFCoefficients swervoPIDFCoefficients, PIDFCoefficients drivePIDFCoefficients, MotorEx[] motors, CRServoEx[] swervos, OctoQuadFWv3 octoQuad, double[] moduleOffsets) {
        if (motors.length != 4 || swervos.length != 4 || moduleOffsets.length != 4) {
            throw new IllegalArgumentException("Hardware lists for swerve modules must have exactly 4 objects each");
        }

        this.maxSpeed = maxSpeed;
        this.maxAngularSpeed = maxSpeed / Math.hypot(trackWidth / 2, wheelBase / 2);

        // Initialize current limiter for fuse protection
        this.currentLimitEnabled = ENABLE_CURRENT_LIMIT;
        this.currentLimiter = new CurrentLimiter(
            CURRENT_LIMIT_WARNING_THRESHOLD,
            CURRENT_HARD_LIMIT,
            CURRENT_LIMIT_MIN_POWER_FRACTION,
            CURRENT_LIMIT_RECOVERY_TIME_CONSTANT
        );

        // Indices typically align: 0=FR, 1=FL, 2=BL, 3=BR
        // We assume ports 0-3 are for steering servos, and 4-7 are for drive wheels.
        this.modules[0] = new OctoSwerveModule(motors[0], swervos[0], octoQuad, 0, 4, new Vector2d(trackWidth / 2, wheelBase / 2), maxSpeed, swervoPIDFCoefficients, drivePIDFCoefficients);
        this.modules[1] = new OctoSwerveModule(motors[1], swervos[1], octoQuad, 1, 5, new Vector2d(trackWidth / 2, -wheelBase / 2), maxSpeed, swervoPIDFCoefficients, drivePIDFCoefficients);
        this.modules[2] = new OctoSwerveModule(motors[2], swervos[2], octoQuad, 2, 6, new Vector2d(-trackWidth / 2, -wheelBase / 2), maxSpeed, swervoPIDFCoefficients, drivePIDFCoefficients);
        this.modules[3] = new OctoSwerveModule(motors[3], swervos[3], octoQuad, 3, 7, new Vector2d(-trackWidth / 2, wheelBase / 2), maxSpeed, swervoPIDFCoefficients, drivePIDFCoefficients);
        this.modules[0].setOffset(moduleOffsets[0]);
        this.modules[1].setOffset(moduleOffsets[1]);
        this.modules[2].setOffset(moduleOffsets[2]);
        this.modules[3].setOffset(moduleOffsets[3]);
    }

    /**
     * Commands the drivetrain to move with the specified chassis velocities.
     *
     * <p>This method translates high-level chassis velocities into individual module commands
     * using swerve inverse kinematics. The process is:</p>
     *
     * <ol>
     *   <li><b>Calculate Module Vectors:</b> For each module, compute the velocity vector
     *       required to achieve the desired chassis motion (combination of forward, lateral,
     *       and rotational velocities)</li>
     *   <li><b>Find Maximum Velocity:</b> Determine the maximum velocity magnitude across
     *       all four modules</li>
     *   <li><b>Desaturate if Needed:</b> If the maximum exceeds {@link #maxSpeed}, scale
     *       all module velocities down proportionally to preserve direction while staying
     *       within physical limits</li>
     *   <li><b>Apply to Modules:</b> Send the final velocity vectors to each module, which
     *       sets steering angle and drive wheel speed</li>
     * </ol>
     *
     * <h3>Velocity Desaturation:</h3>
     * <p>When moving diagonally while rotating, some modules may be commanded beyond their
     * maximum speed. Desaturation scales all modules proportionally:</p>
     * <pre>
     * scaleFactor = maxSpeed / maxVelocity
     * for each module:
     *     module.velocity = module.velocity * scaleFactor
     * </pre>
     * <p>This ensures the robot maintains its intended direction but at a reduced overall
     * speed.</p>
     *
     * <h3>ChassisSpeeds Components:</h3>
     * <ul>
     *   <li><b>vxMetersPerSecond:</b> Forward velocity (positive = forward)</li>
     *   <li><b>vyMetersPerSecond:</b> Lateral velocity (positive = left)</li>
     *   <li><b>omegaRadiansPerSecond:</b> Rotational velocity (positive = counter-clockwise)</li>
     * </ul>
     *
     * @param speeds the chassis velocities (forward, lateral, and rotational) to achieve
     * @see ChassisSpeeds
     * @see OctoSwerveModule#calculateVectorRobotCentric(ChassisSpeeds)
     * @see OctoSwerveModule#updateModuleWithVelocity(Vector2d)
     */
    public void drive(ChassisSpeeds speeds) {
        drive(speeds, 0.0);  // Default: no current limiting
    }

    /**
     * Commands the drivetrain to move with the specified chassis velocities and current limiting.
     *
     * <p>This method is the same as {@link #drive(ChassisSpeeds)} but includes current limiting
     * to prevent tripping the 20A fuse. When current draw is high, chassis speeds are scaled
     * down proportionally to reduce power consumption.</p>
     *
     * <h3>Current Limiting:</h3>
     * <p>If enabled (Constants.ENABLE_CURRENT_LIMIT), this method scales the chassis speeds
     * based on the current draw from the Floodgate sensor:</p>
     * <ul>
     *   <li><b>Below warning threshold:</b> No scaling (100% power)</li>
     *   <li><b>Between warning and hard limit:</b> Linear scaling from 100% to 50%</li>
     *   <li><b>At or above hard limit:</b> Scale to 50% power (emergency reduction)</li>
     * </ul>
     *
     * <h3>Usage:</h3>
     * <p>Call this method from Drive.periodic() with the current draw from the SRS Hub:</p>
     * <pre>
     * double currentDraw = robot.getCurrentDraw();
     * swerve.drive(speeds, currentDraw);
     * </pre>
     *
     * @param speeds the chassis velocities (forward, lateral, and rotational) to achieve
     * @param currentDraw the measured current draw in amps (0.0 to disable current limiting)
     * @see ChassisSpeeds
     * @see OctoSwerveModule#calculateVectorRobotCentric(ChassisSpeeds)
     * @see OctoSwerveModule#updateModuleWithVelocity(Vector2d)
     * @see CurrentLimiter
     */
    public void drive(ChassisSpeeds speeds, double currentDraw) {
        // Apply current limiting if enabled and current draw is provided
        ChassisSpeeds limitedSpeeds = speeds;
        if (currentLimitEnabled && currentDraw > 0.0) {
            double scale = currentLimiter.calculate(1.0, currentDraw, 0.02);  // 0.02s = 50Hz loop
            limitedSpeeds = new ChassisSpeeds(
                speeds.vxMetersPerSecond * scale,
                speeds.vyMetersPerSecond * scale,
                speeds.omegaRadiansPerSecond * scale
            );
        }

        Vector2d[] targetVectors = new Vector2d[4];
        double maxVelocity = 0;

        for (int i = 0; i < 4; i++) {
            targetVectors[i] = modules[i].calculateVectorRobotCentric(limitedSpeeds);
            maxVelocity = Math.max(maxVelocity, targetVectors[i].magnitude());
        }

        // Desaturate Wheel Speeds
        if (maxVelocity > maxSpeed) {
            for (Vector2d targetVector : targetVectors) {
                targetVector.scale(maxSpeed / maxVelocity);
            }
        }

        for (int i = 0; i < 4; i++) {
            modules[i].updateModuleWithVelocity(targetVectors[i]);
        }
    }

    /**
     * Sends telemetry data and draws visualizations to FTC Dashboard.
     *
     * <p>This method provides comprehensive real-time telemetry for tuning and debugging:</p>
     *
     * <h3>Numeric Telemetry:</h3>
     * <p>For each of the 4 modules (FR, FL, BL, BR), this adds:</p>
     * <ul>
     *   <li><b>Target Angle (Rad):</b> Commanded steering angle</li>
     *   <li><b>Actual Angle (Rad):</b> Current measured steering angle</li>
     *   <li><b>Target Vel (In/s):</b> Commanded drive velocity</li>
     *   <li><b>Actual Vel (In/s):</b> Current measured drive velocity</li>
     * </ul>
     * <p>These values are essential for PIDF tuning. Compare target vs actual to identify:</p>
     * <ul>
     *   <li>Steering lag/inaccuracy → Adjust steering PIDF</li>
     *   <li>Drive velocity lag → Adjust drive PIDF</li>
     * </ul>
     *
     * <h3>Field Visualization:</h3>
     * <p>Draws visual indicators on the FTC Dashboard field overlay:</p>
     * <ul>
     *   <li><b>Wheel Orientation Lines:</b> Orange 2.5-inch lines showing each wheel's
     *       actual steering angle. Updates in real-time as modules steer.</li>
     *   <li><b>Module Centers:</b> Orange circles (0.5-inch radius) marking the physical
     *       location of each swerve module on the robot.</li>
     *   <li><b>Position Calculation:</b> Module positions are transformed from robot
     *       coordinates to field coordinates using the robot's current heading and pose.</li>
     * </ul>
     *
     * <h3>Coordinate Transform:</h3>
     * <p>Module positions are transformed from robot frame to field frame:</p>
     * <pre>
     * globalX = robotX + moduleX * cos(heading) - moduleY * sin(heading)
     * globalY = robotY + moduleX * sin(heading) + moduleY * cos(heading)
     * wheelHeading = robotHeading + moduleHeading
     * </pre>
     *
     * @param packet the telemetry packet to populate with data and drawings
     * @param robotPose the robot's current pose on the field (x, y, heading)
     * @see com.acmerobotics.dashboard.telemetry.TelemetryPacket
     * @see com.acmerobotics.dashboard.canvas.Canvas
     */
    public void drawTelemetry(TelemetryPacket packet, Pose robotPose) {
        String[] moduleNames = {"FR", "FL", "BL", "BR"};
        Canvas field = packet.fieldOverlay();
        
        for (int i = 0; i < 4; i++) {
            OctoSwerveModule module = modules[i];
            
            packet.put(moduleNames[i] + " Target Angle (Rad)", module.getTargetAngleRadians());
            packet.put(moduleNames[i] + " Actual Angle (Rad)", module.getModuleHeadingRadians());
            packet.put(moduleNames[i] + " Target Vel (In/s)", module.getTargetMagnitude());
            packet.put(moduleNames[i] + " Actual Vel (In/s)", module.getCurrentVelocityInchesPerSec());
            
            // Draw visual module orientation lines!
            if (robotPose != null) {
                // Determine module absolute field position based on the robot's heading
                double locX = module.getLocation().getX();
                double locY = module.getLocation().getY();
                double heading = robotPose.getHeading();
                
                double globalX = robotPose.getX() + locX * Math.cos(heading) - locY * Math.sin(heading);
                double globalY = robotPose.getY() + locX * Math.sin(heading) + locY * Math.cos(heading);
                
                // Determine absolute field direction the wheel is pointing
                double wheelHeading = heading + module.getModuleHeadingRadians();
                
                // Draw a small 2-inch indicator line showing the direction the wheel is actively facing
                field.setStroke("#FF5722"); // Orange
                field.strokeLine(
                    globalX, globalY,
                    globalX + 2.5 * Math.cos(wheelHeading), 
                    globalY + 2.5 * Math.sin(wheelHeading)
                );
                field.fillCircle(globalX, globalY, 0.5); // Hub center
            }
        }
    }

    /**
     * Logs module state data to the CSV logger for post-match analysis.
     *
     * <p>This method records the state of all four swerve modules to a CSV file for
     * offline analysis and tuning. Logged data includes:</p>
     * <ul>
     *   <li><b>Target Angle:</b> Commanded steering angle (radians)</li>
     *   <li><b>Actual Angle:</b> Measured steering angle (radians)</li>
     *   <li><b>Target Velocity:</b> Commanded drive velocity (inches/second)</li>
     *   <li><b>Actual Velocity:</b> Measured drive velocity (inches/second)</li>
     * </ul>
     *
     * <h3>Use Cases:</h3>
     * <ul>
     *   <li><b>PIDF Tuning:</b> Analyze step response and steady-state error</li>
     *   <li><b>Performance Analysis:</b> Check if modules are achieving commanded velocities</li>
     *   <li><b>Debugging:</b> Identify drift or inconsistency between modules</li>
     *   <li><b>Match Review:</b> Correlate robot behavior with module states</li>
     * </ul>
     *
     * <h3>CSV Output Format:</h3>
     * <p>The logged data is formatted as columns in the CSV:</p>
     * <pre>
     * FR Target Angle, FR Actual Angle, FR Target Vel, FR Actual Vel,
     * FL Target Angle, FL Actual Angle, FL Target Vel, FL Actual Vel,
     * BL Target Angle, BL Actual Angle, BL Target Vel, BL Actual Vel,
     * BR Target Angle, BR Actual Angle, BR Target Vel, BR Actual Vel
     * </pre>
     *
     * @param logger the DataLogger instance (null-safe, does nothing if null)
     * @see org.firstinspires.ftc.teamcode.util.DataLogger
     */
    public void logData(DataLogger logger) {
        if (logger == null) return;
        String[] moduleNames = {"FR", "FL", "BL", "BR"};
        for (int i = 0; i < 4; i++) {
            OctoSwerveModule module = modules[i];
            logger.addData(moduleNames[i] + " Target Angle", module.getTargetAngleRadians());
            logger.addData(moduleNames[i] + " Actual Angle", module.getModuleHeadingRadians());
            logger.addData(moduleNames[i] + " Target Vel", module.getTargetMagnitude());
            logger.addData(moduleNames[i] + " Actual Vel", module.getCurrentVelocityInchesPerSec());
        }
    }

    /**
     * Stops all swerve modules by setting drive motor power to zero.
     *
     * <p>This method immediately stops all module motion by:</p>
     * <ul>
     *   <li>Sending zero power commands to all drive motors</li>
     *   <li>Leaving steering servos at their current positions (no change)</li>
     * </ul>
     *
     * <p><b>Use Cases:</b></p>
     * <ul>
     *   <li>Emergency stop when robot should not move</li>
     *   <li>End of autonomous/teleop period</li>
     *   <li>Safety interlock triggered</li>
     *   <li>Manual debugging/positioning</li>
     * </ul>
     *
     * <p><b>Note:</b> This method only stops drive motors. Steering servos maintain
     * their current angles. To also reset steering to zero, call {@code stop()} and then
     * manually command steering to forward.</p>
     */
    public void stop() {
        for (OctoSwerveModule module : modules) {
            module.stop();
        }
    }
}
