package org.firstinspires.ftc.teamcode.command.subsystems.encoders;

import org.firstinspires.ftc.teamcode.hardware.OctoQuadFWv3;

/**
 * OctoQuad-based swerve encoder implementation.
 *
 * <p>This class wraps the OctoQuad Firmware v3 encoder processor for use with swerve
 * modules. It provides high-precision digital encoder readings with CRC validation.</p>
 *
 * <h3>Encoder Configuration:</h3>
 * <p>Assumes REV Through-Bore encoders with:</p>
 * <ul>
 *   <li><b>Steering:</b> 8192 counts per revolution (absolute positioning)</li>
 *   <li><b>Drive:</b> Velocity in ticks/second</li>
 * </ul>
 *
 * <h3>Port Mapping:</h3>
 * <p>The OctoQuad has 8 encoder ports. Typical swerve configuration:</p>
 * <ul>
 *   <li>Ports 0-3: Steering encoders (one per module)</li>
 *   <li>Ports 4-7: Drive wheel encoders (one per module)</li>
 * </ul>
 *
 * <h3>Performance:</h3>
 * <ul>
 *   <li><b>Update Rate:</b> Up to 1000Hz (1kHz)</li>
 *   <li><b>Resolution:</b> 8192 counts/rev (0.044 degrees per count)</li>
 *   <li><b>Accuracy:</b> High with CRC validation</li>
 * </ul>
 *
 * @see SwerveEncoder
 * @see org.firstinspires.ftc.teamcode.util.OctoQuadFWv3
 */
public class OctoQuadSwerveEncoder implements SwerveEncoder {

    /**
     * OctoQuad encoder processor instance.
     */
    private final OctoQuadFWv3 octoQuad;

    /**
     * OctoQuad port number for this encoder.
     */
    private final int port;

    /**
     * Encoder offset calibration value (radians).
     * Only used for steering encoders to set the zero position.
     */
    private final double offsetRadians;

    /**
     * Counts per revolution for the encoder.
     * Default: 8192 for REV Through-Bore encoder.
     */
    private final int countsPerRev;

    /**
     * Whether this encoder is a steering encoder (true) or drive encoder (false).
     * Steering encoders return position in radians, drive encoders return velocity in ticks/sec.
     */
    private final boolean isSteeringEncoder;

    /**
     * Cached velocity reading (ticks/second).
     * Updated each time getVelocity() is called.
     */
    private double cachedVelocityTicksPerSec = 0.0;

    /**
     * Creates an OctoQuad-based steering encoder.
     *
     * @param octoQuad OctoQuad Firmware v3 instance
     * @param port OctoQuad port number (0-3 for steering)
     * @param offsetRadians encoder offset for zero calibration (radians)
     * @param countsPerRev counts per revolution (8192 for REV Through-Bore)
     */
    public OctoQuadSwerveEncoder(OctoQuadFWv3 octoQuad, int port, double offsetRadians, int countsPerRev) {
        this.octoQuad = octoQuad;
        this.port = port;
        this.offsetRadians = offsetRadians;
        this.countsPerRev = countsPerRev;
        this.isSteeringEncoder = true;
    }

    /**
     * Creates an OctoQuad-based drive encoder.
     *
     * @param octoQuad OctoQuad Firmware v3 instance
     * @param port OctoQuad port number (4-7 for drive)
     */
    public OctoQuadSwerveEncoder(OctoQuadFWv3 octoQuad, int port) {
        this.octoQuad = octoQuad;
        this.port = port;
        this.offsetRadians = 0.0;
        this.countsPerRev = 8192;  // Default for REV Through-Bore
        this.isSteeringEncoder = false;
    }

    @Override
    public double getPosition() {
        if (isSteeringEncoder) {
            // Read steering position and convert to radians
            int ticks = octoQuad.readSinglePosition(port);
            double rads = ((ticks % countsPerRev) / (double) countsPerRev) * 2 * Math.PI;
            return rads - offsetRadians;
        } else {
            // Drive encoders typically don't use position (velocity only)
            // Return 0 for position
            return 0.0;
        }
    }

    @Override
    public double getVelocity() {
        if (isSteeringEncoder) {
            // Steering encoders typically don't provide velocity
            return 0.0;
        } else {
            // Read drive velocity in ticks/second
            cachedVelocityTicksPerSec = octoQuad.readSingleVelocity(port);
            return cachedVelocityTicksPerSec;
        }
    }

    @Override
    public void reset() {
        // OctoQuad encoders are absolute, cannot be reset
        // This method does nothing for absolute encoders
    }

    @Override
    public String getEncoderType() {
        return "OctoQuad";
    }

    /**
     * Gets the cached velocity reading (ticks/second).
     *
     * <p>Useful for telemetry to avoid redundant I2C reads.</p>
     *
     * @return cached velocity in ticks/second
     */
    public double getCachedVelocity() {
        return cachedVelocityTicksPerSec;
    }
}
