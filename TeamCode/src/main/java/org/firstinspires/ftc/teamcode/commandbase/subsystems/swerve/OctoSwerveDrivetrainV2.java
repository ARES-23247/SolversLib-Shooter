package org.firstinspires.ftc.teamcode.commandbase.subsystems.swerve;

import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import org.firstinspires.ftc.teamcode.util.drivers.OctoQuadFWv3;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.swerve.encoders.SwerveEncoder;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.swerve.encoders.SwerveEncoderFactory;
import com.seattlesolvers.solverslib.hardware.motors.CRServoEx;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.seattlesolvers.solverslib.geometry.Vector2d;
import org.firstinspires.ftc.teamcode.hardware.SRSHub;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.pedropathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.util.DataLogger;
import org.firstinspires.ftc.teamcode.util.control.CurrentLimiter;

import static org.firstinspires.ftc.teamcode.globals.Constants.*;

/**
 * Encoder-agnostic four-module swerve drivetrain controller.
 *
 * <p>This is the updated version of {@link OctoSwerveDrivetrain} that supports both
 * OctoQuad digital encoders and SRS Hub analog encoders. Switch between encoder types
 * via {@link org.firstinspires.ftc.teamcode.globals.Constants#SWERVE_ENCODER_TYPE}.</p>
 *
 * <h3>Encoder Support:</h3>
 * <p>This drivetrain automatically uses the configured encoder type:</p>
 * <ul>
 *   <li><b>{@link SwerveEncoderFactory.SwerveEncoderType#OCTOQUAD}:</b> High-precision
 *       digital encoders with 8192 CPR (recommended)</li>
 *   <li><b>{@link SwerveEncoderFactory.SwerveEncoderType#ANALOG}:</b> SRS Hub analog
 *       encoders (potentiometers, magnetic encoders)</li>
 * </ul>
 *
 * <h3>How to Switch Encoders:</h3>
 * <p>Simply change the constant in {@code Constants.java}:</p>
 * <pre>
 * // Use OctoQuad digital encoders (default)
 * public static final SwerveEncoderType SWERVE_ENCODER_TYPE = SwerveEncoderType.OCTOQUAD;
 *
 * // Use SRS Hub analog encoders
 * public static final SwerveEncoderType SWERVE_ENCODER_TYPE = SwerveEncoderType.ANALOG;
 * </pre>
 *
 * <h3>Current Management:</h3>
 * <p>This drivetrain includes current limiting to prevent 20A fuse trips:</p>
 * <ul>
 *   <li><b>Slew Rate Limiting:</b> Prevents rapid power changes (causes current spikes)</li>
 *   <li><b>Current Limiting:</b> Scales power when Floodgage sensor detects high current</li>
 * </ul>
 *
 * @see OctoSwerveModuleV2
 * @see SwerveEncoder
 * @see SwerveEncoderFactory
 */
public class OctoSwerveDrivetrainV2 {

    private final OctoSwerveModuleV2[] modules = new OctoSwerveModuleV2[4];
    private final double maxSpeed;
    private final double maxAngularSpeed;
    private final CurrentLimiter currentLimiter;
    private final boolean currentLimitEnabled;

    /**
     * Constructs an encoder-agnostic swerve drivetrain.
     *
     * <p>This constructor automatically creates the appropriate encoder type based on
     * {@link org.firstinspires.ftc.teamcode.globals.Constants#SWERVE_ENCODER_TYPE}.</p>
     *
     * <h3>OctoQuad Port Mapping:</h3>
     * <ul>
     *   <li>Ports 0-3: Steering encoders</li>
     *   <li>Ports 4-7: Drive wheel encoders</li>
     * </ul>
     *
     * <h3>SRS Hub Analog Pin Mapping:</h3>
     * <ul>
     *   <li>Pins 1-4: Steering encoders (configured in STEERING_ANALOG_PINS)</li>
     *   <li>Drive encoders: Not supported with analog (uses motor encoders)</li>
     * </ul>
     *
     * @param trackWidth distance between left and right wheels (inches)
     * @param wheelBase distance between front and back wheels (inches)
     * @param maxSpeed maximum module velocity (inches/second)
     * @param swervoPIDFCoefficients PIDF gains for steering
     * @param drivePIDFCoefficients PIDF gains for drive
     * @param motors array of 4 drive motors [FR, FL, BL, BR]
     * @param swervos array of 4 steering servos [FR, FL, BL, BR]
     * @param octoQuad OctoQuad instance (used if encoder type is OCTOQUAD)
     * @param srsHub SRS Hub instance (used if encoder type is ANALOG)
     * @param moduleOffsets array of 4 encoder offsets (radians)
     * @throws IllegalArgumentException if arrays are not length 4
     */
    public OctoSwerveDrivetrainV2(double trackWidth, double wheelBase, double maxSpeed,
                                  PIDFCoefficients swervoPIDFCoefficients, PIDFCoefficients drivePIDFCoefficients,
                                  MotorEx[] motors, CRServoEx[] swervos,
                                  OctoQuadFWv3 octoQuad, SRSHub srsHub,
                                  double[] moduleOffsets) {
        if (motors.length != 4 || swervos.length != 4 || moduleOffsets.length != 4) {
            throw new IllegalArgumentException("Hardware lists must have exactly 4 objects each");
        }

        this.maxSpeed = maxSpeed;
        this.maxAngularSpeed = maxSpeed / Math.hypot(trackWidth / 2, wheelBase / 2);

        // Initialize current limiter
        this.currentLimitEnabled = ENABLE_CURRENT_LIMIT;
        this.currentLimiter = new CurrentLimiter(
            CURRENT_LIMIT_WARNING_THRESHOLD,
            CURRENT_HARD_LIMIT,
            CURRENT_LIMIT_MIN_POWER_FRACTION,
            CURRENT_LIMIT_RECOVERY_TIME_CONSTANT
        );

        // Create modules with encoder factory
        for (int i = 0; i < 4; i++) {
            // Create steering encoder via factory
            SwerveEncoder steeringEncoder = SwerveEncoderFactory.createSteeringEncoder(
                octoQuad, srsHub,
                i,                      // OctoQuad port (0-3)
                STEERING_ANALOG_PINS[i], // SRS Hub analog pin
                moduleOffsets[i],        // Offset (radians)
                8192                    // Counts per rev (OctoQuad only)
            );

            // Create drive encoder via factory
            SwerveEncoder driveEncoder = SwerveEncoderFactory.createDriveEncoder(
                octoQuad, srsHub,
                i + 4,  // OctoQuad port (4-7)
                -1      // SRS Hub analog pin (-1 = not used)
            );

            // Calculate module position
            Vector2d moduleLocation;
            switch (i) {
                case 0: moduleLocation = new Vector2d(trackWidth / 2, wheelBase / 2); break;  // FR
                case 1: moduleLocation = new Vector2d(trackWidth / 2, -wheelBase / 2); break; // FL
                case 2: moduleLocation = new Vector2d(-trackWidth / 2, -wheelBase / 2); break; // BL
                case 3: moduleLocation = new Vector2d(-trackWidth / 2, wheelBase / 2); break;  // BR
                default: throw new IllegalArgumentException("Invalid module index: " + i);
            }

            modules[i] = new OctoSwerveModuleV2(
                motors[i], swervos[i],
                steeringEncoder, driveEncoder,
                moduleLocation, maxSpeed,
                swervoPIDFCoefficients, drivePIDFCoefficients
            );
        }
    }

    /**
     * Commands the drivetrain with current limiting.
     *
     * @param speeds chassis velocities to achieve
     * @param currentDraw current draw from Floodgage sensor (0.0 to disable current limiting)
     */
    public void drive(ChassisSpeeds speeds, double currentDraw) {
        // Apply current limiting if enabled
        ChassisSpeeds limitedSpeeds = speeds;
        if (currentLimitEnabled && currentDraw > 0.0) {
            double scale = currentLimiter.calculate(1.0, currentDraw, 0.02);
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
     * Commands the drivetrain without current limiting.
     *
     * @param speeds chassis velocities to achieve
     */
    public void drive(ChassisSpeeds speeds) {
        drive(speeds, 0.0);
    }

    /**
     * Sends telemetry data to FTC Dashboard.
     */
    public void drawTelemetry(TelemetryPacket packet, Pose robotPose) {
        String[] moduleNames = {"FR", "FL", "BL", "BR"};
        Canvas field = packet.fieldOverlay();

        for (int i = 0; i < 4; i++) {
            OctoSwerveModuleV2 module = modules[i];

            // Position and velocity
            packet.put(moduleNames[i] + " Target Angle (Rad)", String.format("%.3f", module.getTargetAngleRadians()));
            packet.put(moduleNames[i] + " Actual Angle (Rad)", String.format("%.3f", module.getModuleHeadingRadians()));
            packet.put(moduleNames[i] + " Target Vel (In/s)", String.format("%.2f", module.getTargetMagnitude()));
            packet.put(moduleNames[i] + " Actual Vel (In/s)", String.format("%.2f", module.getCurrentVelocityInchesPerSec()));
            packet.put(moduleNames[i] + " Encoder Type", module.getEncoderType());

            // Drive motor telemetry
            packet.put(moduleNames[i] + " Drive FF", String.format("%.3f", module.getLastDriveFeedforward()));
            packet.put(moduleNames[i] + " Drive PID", String.format("%.3f", module.getLastDrivePIDCorrection()));
            packet.put(moduleNames[i] + " Drive Power", String.format("%.3f", module.getLastDriveMotorPower()));

            // Steering servo telemetry
            packet.put(moduleNames[i] + " Steer FF", String.format("%.4f", module.getLastSteerFeedforward()));
            packet.put(moduleNames[i] + " Steer PID", String.format("%.4f", module.getLastSteerPIDCorrection()));
            packet.put(moduleNames[i] + " Steer Power", String.format("%.4f", module.getLastSteerServoPower()));

            // Draw wheel orientation
            if (robotPose != null) {
                Vector2d loc = module.getLocation();
                double heading = robotPose.getHeading();

                double globalX = robotPose.getX() + loc.getX() * Math.cos(heading) - loc.getY() * Math.sin(heading);
                double globalY = robotPose.getY() + loc.getX() * Math.sin(heading) + loc.getY() * Math.cos(heading);

                double wheelHeading = heading + module.getModuleHeadingRadians();

                field.setStroke("#FF5722");
                field.strokeLine(
                    globalX, globalY,
                    globalX + 2.5 * Math.cos(wheelHeading),
                    globalY + 2.5 * Math.sin(wheelHeading)
                );
                field.fillCircle(globalX, globalY, 0.5);
            }
        }
    }

    /**
     * Logs module state to CSV.
     */
    public void logData(DataLogger logger) {
        if (logger == null) return;
        String[] moduleNames = {"FR", "FL", "BL", "BR"};
        for (int i = 0; i < 4; i++) {
            OctoSwerveModuleV2 module = modules[i];

            // Position and velocity
            logger.addData(moduleNames[i] + " Target Angle", module.getTargetAngleRadians());
            logger.addData(moduleNames[i] + " Actual Angle", module.getModuleHeadingRadians());
            logger.addData(moduleNames[i] + " Target Vel", module.getTargetMagnitude());
            logger.addData(moduleNames[i] + " Actual Vel", module.getCurrentVelocityInchesPerSec());

            // Drive motor data
            logger.addData(moduleNames[i] + " Drive FF", module.getLastDriveFeedforward());
            logger.addData(moduleNames[i] + " Drive PID", module.getLastDrivePIDCorrection());
            logger.addData(moduleNames[i] + " Drive Power", module.getLastDriveMotorPower());

            // Steering servo data
            logger.addData(moduleNames[i] + " Steer FF", module.getLastSteerFeedforward());
            logger.addData(moduleNames[i] + " Steer PID", module.getLastSteerPIDCorrection());
            logger.addData(moduleNames[i] + " Steer Power", module.getLastSteerServoPower());
        }
    }

    /**
     * Stops all modules.
     */
    public void stop() {
        for (OctoSwerveModuleV2 module : modules) {
            module.stop();
        }
    }

    /**
     * Checks if current limiting is currently reducing motor power.
     * @return true if current limiting is active (scale < 1.0)
     */
    public boolean isCurrentLimitingActive() {
        if (!currentLimitEnabled) return false;
        return currentLimiter.getScaleFactor() < 0.99;  // Small threshold for floating point
    }

    /**
     * Gets the current limiting scale factor.
     * @return scale factor (1.0 = no limiting, < 1.0 = limiting active)
     */
    public double getCurrentLimitScale() {
        if (!currentLimitEnabled) return 1.0;
        return currentLimiter.getScaleFactor();
    }

    /**
     * Gets the current encoder type being used.
     *
     * @return encoder type (e.g., "OctoQuad", "Analog (SRS Hub)")
     */
    public String getEncoderType() {
        return modules[0].getEncoderType();
    }
}
