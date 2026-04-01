package org.firstinspires.ftc.teamcode.util.tuning;

import org.firstinspires.ftc.teamcode.commandbase.subsystems.Drive;
import org.firstinspires.ftc.teamcode.globals.Robot;

import com.seattlesolvers.solverslib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.pedropathing.geometry.Pose;

import com.seattlesolvers.solverslib.util.TelemetryData;

import static org.firstinspires.ftc.teamcode.globals.Constants.*;

/**
 * Simple feedforward tuning utility for swerve drive.
 *
 * <p>This class provides a straightforward way to automatically determine kS and kV
 * feedforward constants by running the motors at different power levels.</p>
 *
 * <h3>Quick Start:</h3>
 * <pre>{@code
 * // In your OpMode (init or loop):
 * FeedforwardTuner tuner = new FeedforwardTuner(drive);
 *
 * // Run the tuning routine
 * double[] results = tuner.tune();  // Returns [kS, kV]
 *
 * // Apply results to Constants.java
 * Constants.DRIVE_KS = results[0];
 * Constants.DRIVE_KV = results[1];
 * }</pre>
 *
 * <h3>Safety Requirements:</h3>
 * <ul>
 *   <li><b>Robot must be on the FIELD FLOOR!</b> Normal driving surface (carpet/tiles)</li>
 *   <li>Clear area around robot (4x4 feet minimum)</li>
 *   <li>Stay clear of moving parts</li>
 *   <li>Have emergency stop ready</li>
 *   <li>FTC field is 12x12 feet - robot will stop at 10 feet</li>
 * </ul>
 *
 * <h3>What It Does:</h3>
 * <ol>
 *   <li><b>Tune kS:</b> Ramp power 0→30% until wheels start spinning</li>
 *   <li><b>Tune kV:</b> Run at 20%, 40%, 60%, 80% power, calculate slope</li>
 *   <li><b>Report:</b> Display results to driver station telemetry</li>
 * </ol>
 *
 * @see org.firstinspires.ftc.teamcode.commandbase.subsystems.Drive
 */
public class FeedforwardTuner {

    private final Drive drive;
    private final TelemetryData telemetry;
    private final ElapsedTime timer;

    // Tuning parameters
    private static final double KS_MAX_POWER = 0.30;
    private static final double KS_INCREMENT = 0.005;
    private static final double KS_VELOCITY_THRESHOLD = 5.0;  // Robot must actually move (5 in/s)

    private static final double[] KV_TEST_POWERS = {0.2, 0.4, 0.6, 0.8};
    private static final int KV_SAMPLES = 20;
    private static final int KV_STEADY_STATE_DELAY = 10;

    // ===== SAFETY PARAMETERS =====

    /**
     * Maximum distance robot can travel during tuning (inches).
     * Prevents robot from driving off the field or into walls.
     * FTC field is 12x12 feet, so 10 feet provides 2 feet of safety margin.
     */
    private static final double MAX_TRAVEL_DISTANCE = 120.0;  // 10 feet maximum

    /**
     * Maximum time for kV tuning at each power level (seconds).
     * Prevents robot from running continuously if something goes wrong.
     */
    private static final double MAX_TIME_PER_LEVEL = 3.0;

    /**
     * Maximum total tuning time (seconds).
     * Auto-stop if exceeded for safety.
     */
    private static final double MAX_TOTAL_TUNING_TIME = 60.0;

    /**
     * Velocity threshold to detect if robot is stuck (in/s).
     * If velocity doesn't increase when power is applied, stop immediately.
     */
    private static final double STUCK_VELOCITY_THRESHOLD = 1.0;

    // Safety state tracking
    private double startingX = 0.0;
    private double startingY = 0.0;
    private double totalDistanceTraveled = 0.0;
    private double levelStartTime = 0.0;
    private double totalTuningTime = 0.0;

    // Results
    private double foundKS = 0.0;
    private double foundKV = 0.0;

    /**
     * Creates a new feedforward tuner.
     *
     * @param drive the drive subsystem to tune
     */
    public FeedforwardTuner(Drive drive) {
        this.drive = drive;
        this.telemetry = Robot.getInstance().telemetry;
        this.timer = new ElapsedTime();
    }

    /**
     * Runs the complete feedforward tuning routine with safety systems.
     *
     * <h3>Process:</h3>
     * <ol>
     *   <li>Safety check (5 seconds)</li>
     *   <li>Tune kS (~3 seconds)</li>
     *   <li>Tune kV (~15 seconds)</li>
     *   <li>Report results</li>
     * </ol>
     *
     * <h3>Safety Systems:</h3>
     * <ul>
     *   <li><b>Distance Limit:</b> Max 3 feet traveled → Auto-stop</li>
     *   *   <li><b>Time Limit:</b> Max 60 seconds total → Auto-stop</li>
     *   *   <li><b>Stuck Detection:</b> If not moving → Auto-stop</li>
     *   *   <li><b>Emergency Stop:</b> Driver can STOP anytime</li>
     * </ul>
     *
     * <h3>Total Duration:</h3> ~30 seconds (or until safety limit)
     *
     * @return array of [kS, kV] feedforward constants
     */
    public double[] tune() {
        timer.reset();
        totalTuningTime = 0.0;

        System.out.println("===========================================");
        System.out.println("   FEEDFORWARD AUTOTUNER STARTING");
        System.out.println("===========================================");
        System.out.println("");
        System.out.println("SAFETY SYSTEMS ACTIVE:");
        System.out.println("  ✓ Distance limit: 10 feet maximum");
        System.out.println("  ✓ Time limit: 60 seconds maximum");
        System.out.println("  ✓ Stuck detection: Auto-stop if not moving");
        System.out.println("  ✓ Emergency stop: Driver can STOP anytime");
        System.out.println("");
        System.out.println("ROBOT SETUP:");
        System.out.println("  ✓ Robot on field floor (carpet/tiles)");
        System.out.println("  ✓ Clear area: 4x4 feet minimum");
        System.out.println("  ✓ Robot will drive forward");
        System.out.println("");
        System.out.println("Starting in 5 seconds...");

        // Safety countdown
        for (int i = 5; i > 0; i--) {
            System.out.println(String.format("  %d...", i));
            sleep(1000);

            // Check for early stop
            if (isStopRequested() || totalTuningTime > MAX_TOTAL_TUNING_TIME) {
                System.out.println("  EMERGENCY STOP - Tuning cancelled!");
                drive.swerve.stop();
                return new double[]{0.0, 0.0};
            }
        }

        // Initialize position tracking
        initializePositionTracking();

        // Tune kS
        System.out.println("");
        System.out.println("STEP 1: TUNING kS (Static Friction)");
        System.out.println("----------------------------------------");
        foundKS = tuneKS();

        if (isStopRequested() || totalTuningTime > MAX_TOTAL_TUNING_TIME) {
            System.out.println("  EMERGENCY STOP - Tuning cancelled!");
            drive.swerve.stop();
            return new double[]{foundKS, 0.0};
        }

        // Brief pause
        drive.swerve.stop();
        System.out.println("  Pausing for 1 second...");
        sleep(1000);

        // Reset position for kV tuning
        resetPositionTracking();

        // Tune kV
        System.out.println("");
        System.out.println("STEP 2: TUNING kV (Velocity Constant)");
        System.out.println("----------------------------------------");
        foundKV = tuneKV();

        // Stop and report
        drive.swerve.stop();

        System.out.println("");
        System.out.println("===========================================");
        System.out.println("   TUNING COMPLETE!");
        System.out.println("===========================================");
        System.out.println("");
        System.out.println(String.format("kS (Static Friction): %.4f", foundKS));
        System.out.println(String.format("kV (Velocity):        %.4f", foundKV));
        System.out.println("");
        System.out.println("UPDATE Constants.java:");
        System.out.println(String.format("  DRIVE_KS = %.4f;", foundKS));
        System.out.println(String.format("  DRIVE_KV = %.4f;", foundKV));
        System.out.println("");
        System.out.println("Then restart robot to apply!");

        return new double[]{foundKS, foundKV};
    }

    /**
     * Tunes the kS (static friction) constant.
     *
     * <p>Gradually increases motor power until wheels just start spinning.</p>
     *
     * @return the measured kS value
     */
    private double tuneKS() {
        System.out.println("  Ramping power until robot moves...");

        double power = 0.0;
        double lastVelocity = 0.0;
        levelStartTime = timer.seconds();

        while (power <= KS_MAX_POWER) {
            // Safety checks
            if (isStopRequested()) {
                System.out.println("");
                System.out.println("  EMERGENCY STOP - Tuning cancelled!");
                drive.swerve.stop();
                return 0.15;  // Default value
            }

            if (checkDistanceLimit()) {
                drive.swerve.stop();
                return 0.15;  // Default value
            }

            if (checkTimeLimit(levelStartTime)) {
                drive.swerve.stop();
                return 0.15;  // Default value
            }

            drive.swerve.drive(new ChassisSpeeds(power, 0, 0));
            sleep(50);  // Wait for motors to respond

            double velocity = getAverageVelocity();
            System.out.println(String.format("  Power: %.3f → Velocity: %.1f in/s", power, velocity));

            // Check if robot started moving (velocity increased significantly)
            if (velocity > KS_VELOCITY_THRESHOLD && velocity > lastVelocity * 1.5) {
                System.out.println(String.format("  ✓ kS FOUND: %.4f", power));
                System.out.println(String.format("  Distance traveled: %.1f inches", totalDistanceTraveled));
                return power;
            }

            lastVelocity = velocity;
            power += KS_INCREMENT;
        }

        // Timeout - return default value
        System.out.println("  ⚠ TIMEOUT - Using default kS: 0.1500");
        System.out.println("  Robot did not start moving within power limit");
        return 0.15;
    }

    /**
     * Tunes the kV (velocity constant) using linear regression.
     *
     * <p>Runs motors at multiple power levels and fits a line to the data.</p>
     *
     * @return the calculated kV value
     */
    private double tuneKV() {
        System.out.println("  Testing power levels: 20%, 40%, 60%, 80%");
        System.out.println("");

        double[] powers = KV_TEST_POWERS;
        double[] velocities = new double[powers.length];

        // Test each power level
        for (int i = 0; i < powers.length; i++) {
            System.out.println(String.format("  Testing %d%% power...", (int)(powers[i] * 100)));

            drive.swerve.drive(new ChassisSpeeds(powers[i], 0, 0));

            levelStartTime = timer.seconds();

            // Wait for steady state
            sleep(KV_STEADY_STATE_DELAY * 50);

            // Collect velocity samples
            double sumVelocity = 0.0;
            int samples = KV_SAMPLES;

            for (int j = 0; j < KV_SAMPLES; j++) {
                sleep(50);

                // Safety checks before each sample
                if (isStopRequested()) {
                    System.out.println("");
                    System.out.println("  EMERGENCY STOP - Tuning cancelled!");
                    drive.swerve.stop();
                    return calculateKVPreliminary(powers, velocities, i);
                }

                if (checkDistanceLimit() || checkTimeLimit(levelStartTime)) {
                    drive.swerve.stop();
                    return calculateKVPreliminary(powers, velocities, i);
                }

                double velocity = getAverageVelocity();
                sumVelocity += velocity;

                // Stuck detection - if velocity is too low for the power applied
                if (j > KV_SAMPLES / 2 && velocity < STUCK_VELOCITY_THRESHOLD && powers[i] > 0.4) {
                    System.out.println("");
                    System.out.println("  ⚠ SAFETY: Robot appears stuck!");
                    System.out.println("  Very low velocity despite high power applied.");
                    drive.swerve.stop();
                    return calculateKVPreliminary(powers, velocities, i);
                }
            }

            velocities[i] = sumVelocity / KV_SAMPLES;
            System.out.println(String.format("    → Measured: %.1f in/s", velocities[i]));
            System.out.println(String.format("    → Distance: %.1f / %.1f inches",
                totalDistanceTraveled, MAX_TRAVEL_DISTANCE));
            System.out.println("");

            // Brief pause between levels
            drive.swerve.stop();
            sleep(200);
        }

        // Calculate kV using linear regression
        System.out.println("  Calculating kV using linear regression...");

        double sumX = 0, sumY = 0, sumXY = 0, sumX2 = 0;
        int n = powers.length;

        for (int i = 0; i < n; i++) {
            double velocityFraction = velocities[i] / MAX_DRIVE_VELOCITY;
            sumX += velocityFraction;
            sumY += powers[i];
            sumXY += velocityFraction * powers[i];
            sumX2 += velocityFraction * velocityFraction;
        }

        double slope = (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX);
        double intercept = (sumY - slope * sumX) / n;

        System.out.println("  Regression Results:");
        for (int i = 0; i < n; i++) {
            System.out.println(String.format("    %d%% Power → %.1f in/s",
                (int)(powers[i] * 100), velocities[i]));
        }
        System.out.println(String.format("  kV (Slope):    %.4f", slope));
        System.out.println(String.format("  kS (Intercept): %.4f", intercept));

        // kV is the slope
        return slope;
    }

    /**
     * Calculates kV from partial data (used when safety limits are hit).
     *
     * @param powers array of test powers
     * @param velocities array of measured velocities (may be partially filled)
     * @param validCount number of valid data points
     * @return calculated kV value
     */
    private double calculateKVPreliminary(double[] powers, double[] velocities, int validCount) {
        // Use only valid data points
        int n = validCount;
        if (n < 2) {
            System.out.println("");
            System.out.println("  ⚠ WARNING: Insufficient data for kV calculation!");
            System.out.println("  Using default kV: 1.0");
            return 1.0;  // Default value
        }

        System.out.println("");
        System.out.println("  Calculating kV from partial data...");

        double sumX = 0, sumY = 0, sumXY = 0, sumX2 = 0;

        for (int i = 0; i < n; i++) {
            double velocityFraction = velocities[i] / MAX_DRIVE_VELOCITY;
            sumX += velocityFraction;
            sumY += powers[i];
            sumXY += velocityFraction * powers[i];
            sumX2 += velocityFraction * velocityFraction;
        }

        double slope = (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX);

        System.out.println(String.format("  kV (from %d data points): %.4f", n, slope));
        return slope;
    }

    /**
     * Gets the average drive velocity across all modules.
     *
     * <p>TODO: Implement this by reading from your swerve drivetrain's modules</p>
     *
     * @return average velocity in inches/second
     */
    private double getAverageVelocity() {
        // TODO: Read actual module velocities from OctoSwerveDrivetrain
        // For now, return 0 as placeholder
        // Implementation should access: drive.swerve.modules[i].getCurrentVelocityInchesPerSec()
        return 0.0;
    }

    /**
     * Checks if the driver has requested a stop.
     */
    private boolean isStopRequested() {
        return Thread.currentThread().isInterrupted();
    }

    /**
     * Initializes position tracking for safety distance monitoring.
     */
    private void initializePositionTracking() {
        // Get current pose from sensor fusion
        Pose currentPose = drive.getSensorFusion().getEstimatedPose();
        if (currentPose != null) {
            startingX = currentPose.getX();
            startingY = currentPose.getY();
        } else {
            startingX = 0.0;
            startingY = 0.0;
        }
        totalDistanceTraveled = 0.0;
        levelStartTime = timer.seconds();
    }

    /**
     * Resets position tracking for next tuning phase.
     */
    private void resetPositionTracking() {
        initializePositionTracking();
    }

    /**
     * Checks if robot has traveled beyond safe distance limit.
     *
     * @return true if distance limit exceeded
     */
    private boolean checkDistanceLimit() {
        Pose currentPose = drive.getSensorFusion().getEstimatedPose();
        if (currentPose == null) {
            return false;  // Can't check without pose data
        }

        double currentX = currentPose.getX();
        double currentY = currentPose.getY();
        totalDistanceTraveled = Math.hypot(currentX - startingX, currentY - startingY);

        if (totalDistanceTraveled > MAX_TRAVEL_DISTANCE) {
            System.out.println("");
            System.out.println("  ⚠ SAFETY: Distance limit reached!");
            System.out.println(String.format("  Traveled: %.1f inches (limit: %.1f)",
                totalDistanceTraveled, MAX_TRAVEL_DISTANCE));
            return true;
        }

        return false;
    }

    /**
     * Checks if time limit has been exceeded.
     *
     * @param levelStartTime start time of current level
     * @return true if time limit exceeded
     */
    private boolean checkTimeLimit(double levelStartTime) {
        totalTuningTime = timer.seconds();
        double levelTime = totalTuningTime - levelStartTime;

        if (levelTime > MAX_TIME_PER_LEVEL) {
            System.out.println("");
            System.out.println("  ⚠ SAFETY: Time limit for this level reached!");
            System.out.println(String.format("  Level time: %.1f seconds (limit: %.1f)",
                levelTime, MAX_TIME_PER_LEVEL));
            return true;
        }

        return false;
    }

    /**
     * Sleeps for the specified number of milliseconds.
     */
    private void sleep(long millis) {
        try {
            Thread.sleep(millis);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }
}
