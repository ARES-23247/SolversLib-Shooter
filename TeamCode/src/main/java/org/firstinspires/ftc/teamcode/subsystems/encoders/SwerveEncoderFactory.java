package org.firstinspires.ftc.teamcode.subsystems.encoders;

import org.firstinspires.ftc.teamcode.hardware.OctoQuadFWv3;
import org.firstinspires.ftc.teamcode.hardware.SRSHub;

import static org.firstinspires.ftc.teamcode.Constants.*;

/**
 * Factory for creating swerve encoders based on configuration.
 *
 * <p>This factory creates the appropriate encoder implementation (OctoQuad or Analog)
 * based on the constants configured in {@link org.firstinspires.ftc.teamcode.Constants}.</p>
 *
 * <h3>Encoder Types:</h3>
 * <ul>
 *   <li><b>OctoQuad (Digital):</b> High-precision digital encoders via OctoQuad</li>
 *   <li><b>Analog (SRS Hub):</b> Analog potentiometers/magnetic encoders via SRS Hub</li>
 * </ul>
 *
 * <h3>Configuration:</h3>
 * <p>Set the following constants in {@code Constants.java}:</p>
 * <pre>
 * // Use OctoQuad encoders
 * SWERVE_ENCODER_TYPE = SwerveEncoderType.OCTOQUAD
 *
 * // Use SRS Hub analog encoders
 * SWERVE_ENCODER_TYPE = SwerveEncoderType.ANALOG
 * </pre>
 *
 * <h3>Usage Example:</h3>
 * <pre>{@code
 * // Create steering encoder for front-right module
 * SwerveEncoder steeringEncoder = SwerveEncoderFactory.createSteeringEncoder(
 *     octoQuad, srsHub,
 *     0,           // OctoQuad port
 *     1,           // SRS Hub analog pin
 *     0.0,         // Module offset (radians)
 *     8192         // Counts per revolution (OctoQuad only)
 * );
 *
 * // Create drive encoder for front-right module
 * SwerveEncoder driveEncoder = SwerveEncoderFactory.createDriveEncoder(
 *     octoQuad, srsHub,
 *     4,           // OctoQuad port
 *     -1           // SRS Hub analog pin (-1 if not used)
 * );
 * }</pre>
 *
 * @see SwerveEncoder
 * @see OctoQuadSwerveEncoder
 * @see AnalogSwerveEncoder
 */
public class SwerveEncoderFactory {

    /**
     * Supported swerve encoder types.
     */
    public enum SwerveEncoderType {
        /**
         * OctoQuad digital encoders (high precision, 8192 CPR).
         */
        OCTOQUAD,

        /**
         * SRS Hub analog encoders (potentiometers, magnetic encoders).
         */
        ANALOG
    }

    /**
     * Creates a steering encoder based on the configured encoder type.
     *
     * <p>This factory method creates the appropriate encoder implementation based on
     * {@link org.firstinspires.ftc.teamcode.Constants#SWERVE_ENCODER_TYPE}.</p>
     *
     * @param octoQuad OctoQuad instance (used if type is OCTOQUAD)
     * @param srsHub SRS Hub instance (used if type is ANALOG)
     * @param octoQuadPort OctoQuad port number (0-3 for steering, ignored if type is ANALOG)
     * @param analogPin SRS Hub analog pin number (1-12, ignored if type is OCTOQUAD)
     * @param offsetRadians encoder offset for zero calibration (radians)
     * @param countsPerRev counts per revolution (only used for OCTOQUAD, default 8192)
     * @return configured steering encoder
     */
    public static SwerveEncoder createSteeringEncoder(OctoQuadFWv3 octoQuad, SRSHub srsHub,
                                                      int octoQuadPort, int analogPin,
                                                      double offsetRadians, int countsPerRev) {
        switch (SWERVE_ENCODER_TYPE) {
            case OCTOQUAD:
                return new OctoQuadSwerveEncoder(octoQuad, octoQuadPort, offsetRadians, countsPerRev);

            case ANALOG:
                if (srsHub == null) {
                    throw new IllegalStateException("SRS Hub must be initialized for analog encoders");
                }
                // Use default analog range: 0-3.3V maps to -π to +π
                return new AnalogSwerveEncoder(srsHub, analogPin, -Math.PI, Math.PI, offsetRadians);

            default:
                throw new IllegalArgumentException("Unknown encoder type: " + SWERVE_ENCODER_TYPE);
        }
    }

    /**
     * Creates a steering encoder with default counts per revolution (8192).
     *
     * @param octoQuad OctoQuad instance (used if type is OCTOQUAD)
     * @param srsHub SRS Hub instance (used if type is ANALOG)
     * @param octoQuadPort OctoQuad port number (0-3 for steering, ignored if type is ANALOG)
     * @param analogPin SRS Hub analog pin number (1-12, ignored if type is OCTOQUAD)
     * @param offsetRadians encoder offset for zero calibration (radians)
     * @return configured steering encoder
     */
    public static SwerveEncoder createSteeringEncoder(OctoQuadFWv3 octoQuad, SRSHub srsHub,
                                                      int octoQuadPort, int analogPin,
                                                      double offsetRadians) {
        return createSteeringEncoder(octoQuad, srsHub, octoQuadPort, analogPin, offsetRadians, 8192);
    }

    /**
     * Creates a drive encoder based on the configured encoder type.
     *
     * <p><b>Note:</b> Analog encoders are not recommended for drive wheels. Use OctoQuad
     * digital encoders for accurate velocity measurement. If using ANALOG type, this
     * will return 0 for velocity (drive wheels should use motor encoders instead).</p>
     *
     * @param octoQuad OctoQuad instance (used if type is OCTOQUAD)
     * @param srsHub SRS Hub instance (not used for drive encoders)
     * @param octoQuadPort OctoQuad port number (4-7 for drive, ignored if type is ANALOG)
     * @param analogPin SRS Hub analog pin number (not used for drive encoders)
     * @return configured drive encoder
     */
    public static SwerveEncoder createDriveEncoder(OctoQuadFWv3 octoQuad, SRSHub srsHub,
                                                   int octoQuadPort, int analogPin) {
        switch (SWERVE_ENCODER_TYPE) {
            case OCTOQUAD:
                return new OctoQuadSwerveEncoder(octoQuad, octoQuadPort);

            case ANALOG:
                // Analog encoders not recommended for drive wheels
                // Return a dummy encoder that returns 0 for velocity
                return new SwerveEncoder() {
                    @Override
                    public double getPosition() {
                        return 0.0;
                    }

                    @Override
                    public double getVelocity() {
                        return 0.0;
                    }

                    @Override
                    public void reset() {
                        // Do nothing
                    }

                    @Override
                    public String getEncoderType() {
                        return "Analog (Not Recommended for Drive)";
                    }
                };

            default:
                throw new IllegalArgumentException("Unknown encoder type: " + SWERVE_ENCODER_TYPE);
        }
    }

    /**
     * Gets the current encoder type from configuration.
     *
     * @return current encoder type
     */
    public static SwerveEncoderType getCurrentEncoderType() {
        return SWERVE_ENCODER_TYPE;
    }
}
