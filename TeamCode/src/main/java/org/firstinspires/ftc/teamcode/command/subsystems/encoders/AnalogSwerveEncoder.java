package org.firstinspires.ftc.teamcode.command.subsystems.encoders;

import org.firstinspires.ftc.teamcode.hardware.SRSHub;

/**
 * SRS Hub analog swerve encoder implementation.
 *
 * <p>This class uses the SRS Hub's analog inputs to read swerve module encoders.
 * It supports analog potentiometers and magnetic encoders that output a voltage
 * proportional to position.</p>
 *
 * <h3>Supported Encoder Types:</h3>
 * <ul>
 *   <li><b>Analog Potentiometers:</b> 0-3.3V output, typically 270° or 360° range</li>
 *   <li><b>Magnetic Encoders:</b> AS5048B, MA730, etc. with analog output</li>
 *   <li><b>East Loop Components:</b> Any analog encoder with voltage output</li>
 * </ul>
 *
 * <h3>Analog Configuration:</h3>
 * <p>SRS Hub analog inputs:</p>
 * <ul>
 *   <li><b>Voltage Range:</b> 0-3.3V (mapped to 0.0-1.0 normalized)</li>
 *   <li><b>Resolution:</b> 12-bit ADC (4096 levels)</li>
 *   <li><b>Pins:</b> 1-12 available</li>
 * </ul>
 *
 * <h3>Calibration:</h3>
 * <p>Analog encoders require calibration to map voltage to angle:</p>
 * <pre>
 * // Single-turn potentiometer (270° range)
 * AnalogSwerveEncoder encoder = new AnalogSwerveEncoder(
 *     srsHub, pin,
 *     0.0,           // minVoltage (0V)
 *     3.3,           // maxVoltage (3.3V)
 *     -Math.PI,      // minAngle (-180° in radians)
 *     Math.PI,       // maxAngle (+180° in radians)
 *     0.0            // offset (calibrated so forward = 0)
 * );
 * </pre>
 *
 * <h3>Performance vs Digital:</h3>
 * <ul>
 *   <li><b>Pros:</b> Lower cost, simpler wiring, no I2C congestion</li>
 *   <li><b>Cons:</b> Lower resolution (4096 vs 8192), susceptible to noise, drift over time</li>
 *   <li><b>Best For:</b> Low-budget builds, testing, or when OctoQuad ports are full</li>
 * </ul>
 *
 * @see SwerveEncoder
 * @see org.firstinspires.ftc.teamcode.hardware.SRSHub
 * @see OctoQuadSwerveEncoder
 */
public class AnalogSwerveEncoder implements SwerveEncoder {

    /**
     * SRS Hub instance for analog readings.
     */
    private final SRSHub srsHub;

    /**
     * Analog pin number on the SRS Hub (1-12).
     */
    private final int pin;

    /**
     * Minimum voltage corresponding to minAngle (volts).
     */
    private final double minVoltage;

    /**
     * Maximum voltage corresponding to maxAngle (volts).
     */
    private final double maxVoltage;

    /**
     * Minimum angle in radians corresponding to minVoltage.
     */
    private final double minAngle;

    /**
     * Maximum angle in radians corresponding to maxVoltage.
     */
    private final double maxAngle;

    /**
     * Angle offset to calibrate zero position (radians).
     * Subtracted from the reading to set forward = 0.
     */
    private final double offsetRadians;

    /**
     * Whether this encoder is a steering encoder (true) or drive encoder (false).
     */
    private final boolean isSteeringEncoder;

    /**
     * Cached position reading (radians).
     */
    private double cachedPositionRadians = 0.0;

    /**
     * Cached velocity reading (radians/second).
     */
    private double cachedVelocityRadiansPerSec = 0.0;

    /**
     * Previous position for velocity calculation.
     */
    private double previousPositionRadians = 0.0;

    /**
     * Previous timestamp for velocity calculation (nanoseconds).
     */
    private long previousTimestampNanos = System.nanoTime();

    /**
     * Creates an analog steering encoder.
     *
     * <p>This constructor assumes the full voltage range (0-3.3V) maps to the full angle range.</p>
     *
     * @param srsHub SRS Hub instance
     * @param pin analog pin number (1-12)
     * @param minAngle minimum angle in radians corresponding to 0V
     * @param maxAngle maximum angle in radians corresponding to 3.3V
     * @param offsetRadians angle offset to calibrate zero position (radians)
     */
    public AnalogSwerveEncoder(SRSHub srsHub, int pin, double minAngle, double maxAngle, double offsetRadians) {
        this(srsHub, pin, 0.0, 3.3, minAngle, maxAngle, offsetRadians);
    }

    /**
     * Creates an analog steering encoder with custom voltage range.
     *
     * <p>Use this if your encoder doesn't use the full 0-3.3V range.</p>
     *
     * @param srsHub SRS Hub instance
     * @param pin analog pin number (1-12)
     * @param minVoltage minimum voltage (volts, typically 0.0)
     * @param maxVoltage maximum voltage (volts, typically 3.3)
     * @param minAngle minimum angle in radians corresponding to minVoltage
     * @param maxAngle maximum angle in radians corresponding to maxVoltage
     * @param offsetRadians angle offset to calibrate zero position (radians)
     */
    public AnalogSwerveEncoder(SRSHub srsHub, int pin, double minVoltage, double maxVoltage,
                               double minAngle, double maxAngle, double offsetRadians) {
        this.srsHub = srsHub;
        this.pin = pin;
        this.minVoltage = minVoltage;
        this.maxVoltage = maxVoltage;
        this.minAngle = minAngle;
        this.maxAngle = maxAngle;
        this.offsetRadians = offsetRadians;
        this.isSteeringEncoder = true;
    }

    /**
     * Creates an analog drive encoder.
     *
     * <p><b>Note:</b> Analog encoders are not recommended for drive wheels (velocity measurement).
     * Use OctoQuad digital encoders for drive wheels instead. This implementation returns
     * velocity calculated from position changes, which is less accurate than quadrature.</p>
     *
     * @param srsHub SRS Hub instance
     * @param pin analog pin number (1-12)
     */
    public AnalogSwerveEncoder(SRSHub srsHub, int pin) {
        this.srsHub = srsHub;
        this.pin = pin;
        this.minVoltage = 0.0;
        this.maxVoltage = 3.3;
        this.minAngle = 0.0;
        this.maxAngle = 2 * Math.PI;
        this.offsetRadians = 0.0;
        this.isSteeringEncoder = false;
    }

    @Override
    public double getPosition() {
        if (!isSteeringEncoder) {
            // Drive encoders don't use position
            return 0.0;
        }

        // Read analog voltage (0.0-1.0 normalized, multiply by 3.3V)
        double voltage = srsHub.readAnalogDigitalDevice(pin) * 3.3;

        // Clamp voltage to valid range
        voltage = Math.max(minVoltage, Math.min(maxVoltage, voltage));

        // Map voltage to angle using linear interpolation
        double voltageRange = maxVoltage - minVoltage;
        double angleRange = maxAngle - minAngle;
        double voltageFraction = (voltage - minVoltage) / voltageRange;

        double angle = minAngle + (voltageFraction * angleRange);

        // Apply offset calibration
        angle -= offsetRadians;

        // Normalize to [-π, π]
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;

        // Cache for velocity calculation
        cachedPositionRadians = angle;

        return angle;
    }

    @Override
    public double getVelocity() {
        if (isSteeringEncoder) {
            // Calculate velocity from position changes
            long currentTimestampNanos = System.nanoTime();
            double dt = (currentTimestampNanos - previousTimestampNanos) / 1e9;

            if (dt <= 0 || dt > 1.0) {
                // Invalid time delta, return 0
                return 0.0;
            }

            // Calculate velocity: (currentPos - previousPos) / dt
            double currentPosition = getPosition();
            double velocity = (currentPosition - previousPositionRadians) / dt;

            // Update previous values
            previousPositionRadians = currentPosition;
            previousTimestampNanos = currentTimestampNanos;

            cachedVelocityRadiansPerSec = velocity;
            return velocity;
        } else {
            // Drive encoder velocity - not recommended for analog
            // Return 0 or use motor encoder instead
            return 0.0;
        }
    }

    @Override
    public void reset() {
        // Reset velocity calculation state
        previousPositionRadians = 0.0;
        previousTimestampNanos = System.nanoTime();
        cachedVelocityRadiansPerSec = 0.0;
    }

    @Override
    public String getEncoderType() {
        return "Analog (SRS Hub)";
    }

    /**
     * Gets the cached position reading (radians).
     *
     * <p>Useful for telemetry to avoid redundant analog reads.</p>
     *
     * @return cached position in radians
     */
    public double getCachedPosition() {
        return cachedPositionRadians;
    }

    /**
     * Gets the cached velocity reading (radians/second).
     *
     * @return cached velocity in radians/second
     */
    public double getCachedVelocity() {
        return cachedVelocityRadiansPerSec;
    }
}
