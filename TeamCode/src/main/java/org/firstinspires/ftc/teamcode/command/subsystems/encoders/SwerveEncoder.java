package org.firstinspires.ftc.teamcode.command.subsystems.encoders;

/**
 * Interface for swerve module encoders.
 *
 * <p>This interface abstracts different encoder implementations (OctoQuad digital,
 * SRS Hub analog, etc.) to allow easy switching between encoder hardware options.</p>
 *
 * <h3>Implementations:</h3>
 * <ul>
 *   <li>{@link OctoQuadSwerveEncoder}: OctoQuad digital encoders (current system)</li>
 *   <li>{@link AnalogSwerveEncoder}: SRS Hub analog encoders (magnetic/potentiometer)</li>
 * </ul>
 *
 * <h3>Encoder Requirements:</h3>
 * <p>Swer ve module encoders must provide:</p>
 * <ul>
 *   <li><b>Steering Position:</b> Absolute angle of the steering module (radians)</li>
 *   <li><b>Drive Velocity:</b> Rotational speed of the drive wheel (ticks/sec)</li>
 * </ul>
 *
 * <h3>Usage:</h3>
 * <pre>{@code
 * // Create encoder via factory
 * SwerveEncoder steeringEncoder = SwerveEncoderFactory.createSteeringEncoder(
 *     octoQuad, steeringPort, moduleOffset
 * );
 *
 * SwerveEncoder driveEncoder = SwerveEncoderFactory.createDriveEncoder(
 *     octoQuad, drivePort
 * );
 *
 * // Read encoder values
 * double steeringAngle = steeringEncoder.getPosition();
 * double driveVelocity = driveEncoder.getVelocity();
 * }</pre>
 *
 * @see OctoQuadSwerveEncoder
 * @see AnalogSwerveEncoder
 */
public interface SwerveEncoder {

    /**
     * Reads the current position from the encoder.
     *
     * <p>For steering encoders, this returns the absolute angle in radians.</p>
     * <p>For drive encoders, this returns position in ticks (if supported).</p>
     *
     * @return position in appropriate units (radians for steering, ticks for drive)
     */
    double getPosition();

    /**
     * Reads the current velocity from the encoder.
     *
     * <p>For drive encoders, this returns velocity in ticks per second.</p>
     * <p>For steering encoders, this may return 0 if velocity is not available.</p>
     *
     * @return velocity in appropriate units (typically ticks/second)
     */
    double getVelocity();

    /**
     * Resets the encoder to zero.
     *
     * <p>Not all encoders support this (absolute encoders cannot be reset).</p>
     */
    void reset();

    /**
     * Gets the encoder type for telemetry/debugging.
     *
     * @return encoder type name (e.g., "OctoQuad", "Analog")
     */
    String getEncoderType();
}
