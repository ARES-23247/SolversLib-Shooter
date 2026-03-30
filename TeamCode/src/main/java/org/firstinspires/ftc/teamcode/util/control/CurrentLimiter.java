package org.firstinspires.ftc.teamcode.util.control;

/**
 * Current limiter that scales motor power based on measured current draw.
 *
 * <p>This class monitors current draw from a sensor (typically GoBilda Floodgate) and
 * scales down motor power when approaching the current limit. This prevents tripping
 * the 20A fuse by proactively reducing power before the limit is reached.</p>
 *
 * <h3>How It Works:</h3>
 * <p>The current limiter uses a "soft limiting" approach:</p>
 * <ol>
 *   <li><b>Monitor:</b> Continuously read current from the sensor</li>
 *   <li><b>Compare:</b> Check if current exceeds warning threshold</li>
 *   <li><b>Scale:</b> Reduce motor power proportionally as current approaches limit</li>
 *   <li><b>Recover:</b> Gradually restore power when current drops</li>
 * </ol>
 *
 * <h3>Scaling Behavior:</h3>
 * <p>The power scaling works as follows:</p>
 * <ul>
 *   <li><b>Below warning threshold:</b> No scaling (100% power)</li>
 *   <li><b>Between warning and hard limit:</b> Linear scaling from 100% to 50%</li>
 *   <li><b>At or above hard limit:</b> Scale to 50% power (emergency reduction)</li>
 * </ul>
 *
 * <h3>Usage Example:</h3>
 * <pre>{@code
 * // Create limiter with 18A warning threshold (2A below 20A fuse)
 * CurrentLimiter limiter = new CurrentLimiter(18.0, 20.0);
 *
 * // In control loop
 * double targetPower = 1.0;  // Full power requested
 * double currentDraw = srsHub.getCurrentDraw();  // Read from Floodgate
 * double limitedPower = limiter.calculate(targetPower, currentDraw);
 * motor.set(limitedPower);
 *
 * // If current is 19A: power scaled to ~75%
 * // If current is 20A+: power scaled to 50%
 * }</pre>
 *
 * <h3>Why Not Just Cut Power at 20A?</h3>
 * <p>Hard cutoffs cause the robot to suddenly lose power, which can:</p>
 * <ul>
 *   <li>Make the robot unpredictable to control</li>
 *   <li>Cause current spikes when power suddenly changes</li>
 *   <li>Create oscillation as power cuts in/out</li>
 * </ul>
 * <p>Soft limiting provides smooth, predictable behavior while still preventing fuse trips.</p>
 *
 * <h3>Integration with Slew Rate Limiting:</h3>
 * <p>For best results, use BOTH slew rate limiting AND current limiting:</p>
 * <ul>
 *   <li><b>Slew Rate:</b> Prevents rapid power changes that cause sudden current spikes</li>
 *   <li><b>Current Limiter:</b> Handles sustained high current from heavy loads</li>
 * </ul>
 * <pre>{@code
 * // Order of operations in control loop:
 * double targetPower = driverInput;
 * double slewLimited = slewRateLimiter.calculate(targetPower, dt);
 * double currentLimited = currentLimiter.calculate(slewLimited, currentDraw);
 * motor.set(currentLimited);
 * }</pre>
 */
public class CurrentLimiter {

    /**
     * Current threshold in amps where power scaling begins.
     * Below this value, no scaling is applied (100% power).
     */
    private final double warningThreshold;

    /**
     * Hard current limit in amps. At or above this value, power is scaled to minimum.
     * Should be set just below the fuse rating (e.g., 18-19A for a 20A fuse).
     */
    private final double hardLimit;

    /**
     * Minimum power output as a fraction [0.0, 1.0].
     * Even at maximum current overrun, the motor will still receive this much power.
     * Prevents the robot from becoming completely unresponsive.
     */
    private final double minPowerFraction;

    /**
     * Time constant for power recovery in seconds.
     * When current drops, power gradually recovers at this rate to prevent oscillation.
     */
    private final double recoveryTimeConstant;

    /**
     * Current applied power scaling factor [0.0, 1.0].
     * Updated each call to calculate() based on measured current.
     */
    private double currentScaleFactor;

    /**
     * Creates a new current limiter with specified thresholds.
     *
     * <p>This constructor uses default values for minimum power (50%) and recovery rate (1.0 sec).</p>
     *
     * @param warningThreshold current in amps where scaling begins (typically 2-3A below fuse)
     * @param hardLimit hard current limit in amps (typically 18-19A for 20A fuse)
     * @throws IllegalArgumentException if thresholds are invalid
     */
    public CurrentLimiter(double warningThreshold, double hardLimit) {
        this(warningThreshold, hardLimit, 0.5, 1.0);
    }

    /**
     * Creates a new current limiter with full configuration.
     *
     * @param warningThreshold current in amps where scaling begins (must be positive)
     * @param hardLimit hard current limit in amps (must be > warningThreshold)
     * @param minPowerFraction minimum power output as fraction [0.0, 1.0] (e.g., 0.5 = 50%)
     * @param recoveryTimeConstant time constant for power recovery in seconds
     * @throws IllegalArgumentException if parameters are invalid
     */
    public CurrentLimiter(double warningThreshold, double hardLimit, double minPowerFraction, double recoveryTimeConstant) {
        if (warningThreshold <= 0) {
            throw new IllegalArgumentException("Warning threshold must be positive");
        }
        if (hardLimit <= warningThreshold) {
            throw new IllegalArgumentException("Hard limit must be greater than warning threshold");
        }
        if (minPowerFraction < 0 || minPowerFraction > 1) {
            throw new IllegalArgumentException("Min power fraction must be in range [0.0, 1.0]");
        }
        if (recoveryTimeConstant <= 0) {
            throw new IllegalArgumentException("Recovery time constant must be positive");
        }

        this.warningThreshold = warningThreshold;
        this.hardLimit = hardLimit;
        this.minPowerFraction = minPowerFraction;
        this.recoveryTimeConstant = recoveryTimeConstant;
        this.currentScaleFactor = 1.0;
    }

    /**
     * Calculates the current-limited power output.
     *
     * <p>This method:</p>
     * <ol>
     *   <li>Determines the target scale factor based on current draw</li>
     *   <li>Gradually adjusts currentScaleFactor toward target (prevents oscillation)</li>
     *   <li>Applies the scale factor to the input power</li>
     * </ol>
     *
     * <h3>Scaling Logic:</h3>
     * <pre>
     * if current < warningThreshold:
     *     targetScale = 1.0  (no scaling)
     * elif current < hardLimit:
     *     ratio = (current - warningThreshold) / (hardLimit - warningThreshold)
     *     targetScale = 1.0 - ratio * (1.0 - minPowerFraction)
     * else:
     *     targetScale = minPowerFraction  (minimum power)
     * </pre>
     *
     * <h3>Recovery Smoothing:</h3>
     * <p>To prevent oscillation, the scale factor doesn't jump instantly. It approaches
     * the target value gradually based on recoveryTimeConstant:</p>
     * <pre>
     * alpha = 1.0 - exp(-dt / recoveryTimeConstant)
     * currentScaleFactor += alpha * (targetScale - currentScaleFactor)
     * </pre>
     *
     * @param inputPower the desired motor power [-1.0, 1.0]
     * @param currentDraw the measured current draw in amps
     * @param timeDelta time elapsed since last call in seconds (e.g., 0.02 for 50Hz loop)
     * @return the current-limited motor power
     */
    public double calculate(double inputPower, double currentDraw, double timeDelta) {
        // Determine target scale factor based on current draw
        double targetScaleFactor;

        if (currentDraw < warningThreshold) {
            // No scaling needed
            targetScaleFactor = 1.0;
        } else if (currentDraw < hardLimit) {
            // Linear scaling from 1.0 to minPowerFraction
            double ratio = (currentDraw - warningThreshold) / (hardLimit - warningThreshold);
            targetScaleFactor = 1.0 - ratio * (1.0 - minPowerFraction);
        } else {
            // At or above hard limit, apply minimum power
            targetScaleFactor = minPowerFraction;
        }

        // Smooth recovery to prevent oscillation
        // Use exponential moving average with time constant
        double alpha = 1.0 - Math.exp(-timeDelta / recoveryTimeConstant);
        currentScaleFactor += alpha * (targetScaleFactor - currentScaleFactor);

        // Clamp to valid range
        currentScaleFactor = Math.max(minPowerFraction, Math.min(1.0, currentScaleFactor));

        // Apply scaling to input power
        return inputPower * currentScaleFactor;
    }

    /**
     * Gets the current scale factor [0.0, 1.0].
     *
     * <p>Useful for telemetry to see how much the current limiter is reducing power.</p>
     *
     * @return the current scale factor (1.0 = no scaling, 0.5 = 50% power)
     */
    public double getScaleFactor() {
        return currentScaleFactor;
    }

    /**
     * Resets the current limiter to full power.
     *
     * <p>This method is useful when you want to:</p>
     * <ul>
     *   <li>Reset the limiter when the robot is disabled</li>
     *   <li>Manually restore full power after a fault condition</li>
     *   <li>Clear accumulated scaling from previous operation</li>
     * </ul>
     */
    public void reset() {
        currentScaleFactor = 1.0;
    }
}
