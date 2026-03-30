package org.firstinspires.ftc.teamcode.util.control;

/**
 * Rate limiter for smoothing control signals to prevent current spikes.
 *
 * <p>This class limits the rate of change of a signal, preventing sudden jumps that
 * can cause high current draws. This is particularly useful for motor control to:</p>
 *
 * <ul>
 *   <li>Prevent inrush current spikes when motors start</li>
 *   <li>Reduce mechanical stress on drivetrain components</li>
 *   <li>Avoid tripping the 20A fuse during rapid direction changes</li>
 *   <li>Improve traction by preventing wheel slip from sudden acceleration</li>
 * </ul>
 *
 * <h3>How It Works:</h3>
 * <p>The slew rate limiter ensures that the output can only change by a maximum amount
 * per second. If the input changes faster than the rate limit, the output ramps up/down
 * at the maximum allowed rate.</p>
 *
 * <h3>Usage Example:</h3>
 * <pre>{@code
 * // Create limiter: max change of 2.0 per second
 * SlewRateLimiter limiter = new SlewRateLimiter(2.0);
 *
 * // In control loop (assumes ~50Hz = 0.02s per loop)
 * double targetPower = driverInput;  // Could jump from 0 to 1 instantly
 * double limitedPower = limiter.calculate(targetPower, 0.02);
 * motor.set(limitedPower);  // Power ramps up smoothly over 0.5 seconds
 * }</pre>
 *
 * <h3>Choosing Rate Limit:</h3>
 * <ul>
 *   <li><b>Higher rate:</b> More responsive, but less current protection</li>
 *   <li><b>Lower rate:</b> More protection, but robot feels sluggish</li>
 *   <li><b>Typical values:</b> 1.0 to 5.0 units per second for drivetrain motors</li>
 * </ul>
 *
 * <h3>Performance:</h3>
 * <p>This class is designed for minimal overhead. The calculate() method uses only
 * basic arithmetic and is safe to call every control loop iteration.</p>
 */
public class SlewRateLimiter {

    /**
     * Maximum rate of change in units per second.
     * The output cannot change faster than this value.
     */
    private final double rateLimit;

    /**
     * Previous output value from the last calculate() call.
     * Used to compute rate of change.
     */
    private double prevVal;

    /**
     * Creates a new slew rate limiter with the specified rate limit.
     *
     * <p>The rate limit determines how quickly the output can change. A higher
     * value allows faster response but provides less current spike protection.</p>
     *
     * @param rateLimit maximum rate of change in units per second (must be positive)
     * @throws IllegalArgumentException if rateLimit is not positive
     */
    public SlewRateLimiter(double rateLimit) {
        if (rateLimit <= 0) {
            throw new IllegalArgumentException("Rate limit must be positive");
        }
        this.rateLimit = rateLimit;
        this.prevVal = 0;
    }

    /**
     * Creates a new slew rate limiter with specified rate limit and initial value.
     *
     * <p>This constructor allows you to set the starting output value, which is useful
     * if you want to avoid an initial ramp-up from zero.</p>
     *
     * @param rateLimit maximum rate of change in units per second (must be positive)
     * @param initialVal initial output value (typically 0 for motors)
     * @throws IllegalArgumentException if rateLimit is not positive
     */
    public SlewRateLimiter(double rateLimit, double initialVal) {
        this(rateLimit);
        this.prevVal = initialVal;
    }

    /**
     * Calculates the rate-limited output value.
     *
     * <p>This method limits the rate of change from the previous output to the new input.
     * If the change exceeds the rate limit, the output moves by at most rateLimit * timeDelta.</p>
     *
     * <h3>Algorithm:</h3>
     * <pre>
     * change = input - prevVal
     * maxChange = rateLimit * timeDelta
     *
     * if change > maxChange:
     *     output = prevVal + maxChange
     * else if change < -maxChange:
     *     output = prevVal - maxChange
     * else:
     *     output = input
     *
     * prevVal = output
     * return output
     * </pre>
     *
     * @param input the desired input value (typically target motor power)
     * @param timeDelta time elapsed since last call in seconds (e.g., 0.02 for 50Hz loop)
     * @return the rate-limited output value
     */
    public double calculate(double input, double timeDelta) {
        // Calculate the change from previous output
        double change = input - prevVal;

        // Calculate maximum allowed change for this time step
        double maxChange = rateLimit * timeDelta;

        // Limit the rate of change
        if (change > maxChange) {
            prevVal += maxChange;
        } else if (change < -maxChange) {
            prevVal -= maxChange;
        } else {
            prevVal = input;
        }

        return prevVal;
    }

    /**
     * Resets the slew rate limiter to the specified value.
     *
     * <p>This method is useful when you want to:</p>
     * <ul>
     *   <li>Reset the limiter when the robot is disabled</li>
     *   <li>Change the starting point without rate limiting</li>
     *   <li>Avoid an initial ramp-up from zero</li>
     * </ul>
     *
     * @param value the new starting value for the limiter
     */
    public void reset(double value) {
        prevVal = value;
    }

    /**
     * Gets the last output value from the rate limiter.
     *
     * <p>Useful for telemetry or debugging to see what the limiter is currently outputting.</p>
     *
     * @return the last output value
     */
    public double getLastValue() {
        return prevVal;
    }
}
