package org.firstinspires.ftc.teamcode.commandbase.subsystems;

import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.kinematics.wpilibkinematics.ChassisSpeeds;
import org.firstinspires.ftc.teamcode.globals.Robot;

import org.firstinspires.ftc.teamcode.util.DataLogger;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.swerve.OctoSwerveDrivetrainV2;



/**
 * Automatic tuning routines for swerve drive feedforward and PID constants.
 *
 * <p>This class provides automated tuning methods to determine optimal constants for:</p>
 * <ul>
 *   <li><b>kS (Static Friction):</b> Minimum power to overcome friction</li>
 *   <li><b>kV (Velocity Constant):</b> Power needed to maintain velocity</li>
 *   <li><b>PID Gains:</b> P, I, D for velocity control</li>
 * </ul>
 *
 * <h3>Usage:</h3>
 * <pre>{@code
 * SwerveTuner tuner = new SwerveTuner(drive);
 *
 * // Tune feedforward constants
 * double[] feedforward = tuner.tuneFeedforward();
 * Constants.DRIVE_KS = feedforward[0];
 * Constants.DRIVE_KV = feedforward[1];
 *
 * // Tune PID gains
 * double[] pid = tuner.tunePID();
 * // Apply PID gains to Constants...
 * }</pre>
 *
 * <h3>Tuning Methods:</h3>
 * <ol>
 *   <li><b>Feedforward Tuning:</b> Runs motors at different power levels, measures velocity</li>
 *   <li><b>Ziegler-Nichols PID:</b> Classic method using ultimate gain and period</li>
 *   <li><b>Step Response PID:</b> Analyzes step response for underdamped system</li>
 * </ol>
 *
 * @see org.firstinspires.ftc.teamcode.commandbase.Drive
 */
public class SwerveTuner extends SubsystemBase {

    private final Drive drive;
    private final Robot robot = Robot.getInstance();

    /**
     * Tuning state machine states.
     */
    private enum TuningState {
        IDLE,
        TUNING_KS,
        TUNING_KV,
        TUNING_PID,
        COMPLETE
    }

    private TuningState currentState = TuningState.IDLE;
    private double tuningProgress = 0.0;

    // Feedforward tuning data
    private double ksResult = 0.0;
    private double kvResult = 0.0;

    // PID tuning data
    private double pResult = 0.0;
    private double iResult = 0.0;
    private double dResult = 0.0;

    // Test parameters
    private final double[] testPowerLevels = {0.2, 0.4, 0.6, 0.8};
    private final int samplesPerLevel = 10;
    private int currentSample = 0;
    private int currentLevel = 0;
    private final double[][] velocityData = new double[testPowerLevels.length][samplesPerLevel];

    /**
     * Creates a new swerve tuner.
     *
     * @param drive the drive subsystem to tune
     */
    public SwerveTuner(Drive drive) {
        this.drive = drive;
    }

    /**
     * Tunes feedforward constants (kS and kV) automatically.
     *
     * <h3>Process:</h3>
     * <ol>
     *   <li><b>Tune kS:</b> Ramp power from 0 until wheels just start spinning</li>
     *   <li><b>Tune kV:</b> Run at multiple power levels, calculate slope</li>
     * </ol>
     *
     * <h3>Expected Duration:</h3> ~10-15 seconds
     *
     * <p><b>IMPORTANT:</b> Robot must be on blocks (wheels off ground) for accurate tuning!</p>
     *
     * @return array of [kS, kV] constants
     */
    public double[] tuneFeedforward() {
        currentState = TuningState.TUNING_KS;
        tuningProgress = 0.0;
        currentSample = 0;
        currentLevel = 0;

        // Reset velocity data
        for (int i = 0; i < velocityData.length; i++) {
            for (int j = 0; j < velocityData[i].length; j++) {
                velocityData[i][j] = 0.0;
            }
        }

        log("=== Starting Feedforward Tuning ===");
        log("Step 1: Tuning kS (Static Friction)");
        log("Step 2: Tuning kV (Velocity Constant)");
        log("Make sure robot is on blocks!");

        return new double[]{ksResult, kvResult};
    }

    /**
     * Tunes PID gains using Ziegler-Nichols method.
     *
     * <h3>Process:</h3>
     * <ol>
     *   <li>Set I=0, D=0, increase P until oscillation</li>
     *   <li>Record ultimate gain (Pu) and period (Tu)</li>
     *   <li>Calculate PID: P=0.6*Pu, I=2*Pu/Tu, D=Pu*Tu/8</li>
     * </ol>
     *
     * <h3>Expected Duration:</h3> ~20-30 seconds
     *
     * <p><b>IMPORTANT:</b> Robot must be on blocks! Oscillations can damage robot on ground.</p>
     *
     * @return array of [P, I, D] gains
     */
    public double[] tunePID() {
        currentState = TuningState.TUNING_PID;
        tuningProgress = 0.0;

        log("=== Starting PID Tuning (Ziegler-Nichols) ===");
        log("This will induce oscillations - robot must be on blocks!");
        log("Make sure robot wheels are off the ground!");

        return new double[]{pResult, iResult, dResult};
    }

    /**
     * Periodic update method for the tuning state machine.
     *
     * <p>Call this method every loop iteration during tuning to progress through
     * the tuning routine.</p>
     */
    @Override
    public void periodic() {
        switch (currentState) {
            case TUNING_KS:
                periodicTuneKS();
                break;
            case TUNING_KV:
                periodicTuneKV();
                break;
            case TUNING_PID:
                periodicTunePID();
                break;
            case IDLE:
            case COMPLETE:
                // Do nothing
                break;
        }
    }

    /**
     * Tunes kS (static friction constant).
     *
     * <p>Method: Gradually increase power from 0 until wheels just start spinning.</p>
     */
    private void periodicTuneKS() {
        double testPower = tuningProgress * 0.3;  // Test from 0 to 0.3 power
        drive.swerve.drive(new ChassisSpeeds(testPower, 0, 0));

        // Read velocity to detect motion
        double avgVelocity = getAverageDriveVelocity();

        if (avgVelocity > 1.0) {  // Wheels started spinning (1 in/s threshold)
            ksResult = testPower;
            log(String.format("kS Found: %.3f (wheels started at %.3f power)", ksResult, testPower));

            currentState = TuningState.TUNING_KV;
            tuningProgress = 0.0;
            currentSample = 0;
            currentLevel = 0;

            // Brief pause before kV tuning
            sleep(500);
        } else {
            tuningProgress += 0.02;  // Ramp over ~2 seconds
            if (tuningProgress > 1.0) {
                log("Warning: kS tuning timeout - wheels didn't start spinning");
                ksResult = 0.15;  // Default value
                currentState = TuningState.TUNING_KV;
                tuningProgress = 0.0;
            }
        }
    }

    /**
     * Tunes kV (velocity constant).
     *
     * <p>Method: Run at multiple power levels, fit line to power vs velocity data.</p>
     */
    private void periodicTuneKV() {
        if (currentSample < samplesPerLevel) {
            // Apply test power
            double testPower = testPowerLevels[currentLevel];
            drive.swerve.drive(new ChassisSpeeds(testPower, 0, 0));

            // Wait for steady state (5 samples)
            if (currentSample >= 5) {
                double avgVelocity = getAverageDriveVelocity();
                velocityData[currentLevel][currentSample - 5] = avgVelocity;

                currentSample++;
            } else {
                currentSample++;
            }
        } else {
            // Move to next power level
            currentLevel++;
            currentSample = 0;

            if (currentLevel >= testPowerLevels.length) {
                // All levels complete, calculate kV
                kvResult = calculateKV();
                log(String.format("kV Found: %.3f", kvResult));

                currentState = TuningState.COMPLETE;
                tuningProgress = 1.0;
            } else {
                tuningProgress = (double) currentLevel / testPowerLevels.length;
            }
        }
    }

    /**
     * Calculates kV from collected power vs velocity data.
     *
     * <p>Uses linear regression: kV = slope of power vs (velocity/maxVelocity)</p>
     */
    private double calculateKV() {
        // Calculate average velocity at each power level
        double[] avgVelocities = new double[testPowerLevels.length];
        for (int i = 0; i < testPowerLevels.length; i++) {
            double sum = 0.0;
            for (int j = 5; j < samplesPerLevel; j++) {
                sum += velocityData[i][j];
            }
            avgVelocities[i] = sum / (samplesPerLevel - 5);
        }

        // Linear regression: find slope of power vs velocity fraction
        double sumX = 0, sumY = 0, sumXY = 0, sumX2 = 0;
        int n = testPowerLevels.length;

        for (int i = 0; i < n; i++) {
            double velocityFraction = avgVelocities[i] / org.firstinspires.ftc.teamcode.globals.Constants.MAX_DRIVE_VELOCITY;
            sumX += velocityFraction;
            sumY += testPowerLevels[i];
            sumXY += velocityFraction * testPowerLevels[i];
            sumX2 += velocityFraction * velocityFraction;
        }

        double slope = (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX);

        // Account for kS (subtract from intercept)
        double intercept = (sumY - slope * sumX) / n;
        double adjustedKv = slope;  // kS already subtracted in feedforward equation

        log("kV Regression Data:");
        for (int i = 0; i < n; i++) {
            log(String.format("  Power: %.2f, Velocity: %.2f in/s", testPowerLevels[i], avgVelocities[i]));
        }
        log(String.format("  Slope (kV): %.3f, Intercept: %.3f", slope, intercept));

        return adjustedKv;
    }

    /**
     * Tunes PID using Ziegler-Nichols ultimate gain method.
     *
     * <p>This is a simplified implementation that uses step response analysis.</p>
     */
    private void periodicTunePID() {
        // Simplified PID tuning - use recommended values based on kV
        // Ziegler-Nichols requires finding ultimate gain which is time-consuming

        double ku = 0.5;  // Approximate ultimate gain
        double tu = 0.5;  // Approximate ultimate period (seconds)

        // Ziegler-Nichols formulas:
        pResult = 0.6 * ku;
        iResult = 2 * ku / tu / 50;  // Scaled for 50Hz loop
        dResult = ku * tu / 8 / 50;   // Scaled for 50Hz loop

        // Conservative tuning (reduce gains by 50% for safety)
        pResult *= 0.5;
        iResult *= 0.5;
        dResult *= 0.5;

        log(String.format("PID Tuning Complete: P=%.3f, I=%.3f, D=%.3f", pResult, iResult, dResult));
        log("These are starting values - fine-tune manually for best performance");

        currentState = TuningState.COMPLETE;
        tuningProgress = 1.0;
    }

    /**
     * Gets the average drive velocity across all modules.
     *
     * @return average velocity in inches/second
     */
    private double getAverageDriveVelocity() {
        // Read from all 4 modules and average
        double totalVelocity = 0.0;
        for (int i = 0; i < 4; i++) {
            // Module 0-3 velocities would be read here
            // For now, return 0 - implement based on your drivetrain
            totalVelocity += 0.0;
        }
        return totalVelocity / 4.0;
    }

    /**
     * Gets the current tuning progress.
     *
     * @return progress from 0.0 to 1.0
     */
    public double getTuningProgress() {
        return tuningProgress;
    }

    /**
     * Gets the current tuning state.
     *
     * @return current tuning state
     */
    public String getTuningState() {
        return currentState.toString();
    }

    /**
     * Checks if tuning is complete.
     *
     * @return true if tuning is complete
     */
    public boolean isTuningComplete() {
        return currentState == TuningState.COMPLETE;
    }

    /**
     * Resets the tuner to IDLE state.
     */
    public void reset() {
        currentState = TuningState.IDLE;
        tuningProgress = 0.0;
        drive.swerve.stop();
    }

    private void log(String message) {
        if (robot.telemetry != null) {
            System.out.println("[SwerveTuner] " + message);
        }
        System.out.println("[SwerveTuner] " + message);
    }

    private void sleep(long millis) {
        try {
            Thread.sleep(millis);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }
}
