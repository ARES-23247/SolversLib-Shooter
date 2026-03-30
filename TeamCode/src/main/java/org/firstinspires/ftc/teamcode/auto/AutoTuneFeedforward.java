package org.firstinspires.ftc.teamcode.auto;

import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.commandbase.subsystems.Drive;
import org.firstinspires.ftc.teamcode.globals.Robot;
import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Automatic feedforward tuning routine for swerve drive.
 *
 * <p>This command automatically determines:</p>
 * <ul>
 *   <li><b>kS (Static Friction):</b> Power needed to overcome friction</li>
 *   <li><b>kV (Velocity Constant):</b> Power needed to maintain velocity</li>
 * </ul>
 *
 * <h3>Usage:</h3>
 * <pre>{@code
 * // In your Auto or TeleOp OpMode:
 * AutoTuneFeedforward tune = new AutoTuneFeedforward(drive);
 * tune.schedule();  // Runs automatically
 * // Results logged to telemetry and dashboard
 * }</pre>
 *
 * <h3>IMPORTANT SAFETY NOTES:</h3>
 * <ul>
 *   <li><b>ROBOT MUST BE ON BLOCKS!</b> Wheels off the ground</li>
 *   <li>Clear area around robot (wheels will spin)</li>
 *   <li>Stay clear of moving parts</li>
 *   <li>Have finger on STOP button</li>
 * </ul>
 *
 * <h3>Process:</h3>
 * <ol>
 *   <li><b>kS Tune:</b> Ramp power 0→30% until wheels spin (~3 seconds)</li>
 *   <li><b>Pause:</b> Let motors rest (~1 second)</li>
 *   <li><b>kV Tune:</b> Run at 20%, 40%, 60%, 80% power (~12 seconds)</li>
 *   <li><b>Calculate:</b> Linear regression to find kV</li>
 *   <li><b>Report:</b> Display results to telemetry</li>
 * </ol>
 *
 * <h3>Expected Duration:</h3> ~20 seconds total
 *
 * <h3>Output:</h3>
 * <p>Results are displayed on telemetry and logged. Update Constants.java with:</p>
 * <pre>
 * DRIVE_KS = [kS result]
 * DRIVE_KV = [kV result]
 * </pre>
 */
public class AutoTuneFeedforward extends SequentialCommandGroup {

    private final Drive drive;
    private final Telemetry telemetry;

    // Test parameters
    private static final double KS_MAX_POWER = 0.30;  // Max power to test for kS
    private static final double KS_POWER_INCREMENT = 0.01;  // Power increment step
    private static final double KS_VELOCITY_THRESHOLD = 1.0;  // Velocity threshold (in/s)

    private static final double[] KV_TEST_POWERS = {0.2, 0.4, 0.6, 0.8};  // Power levels to test
    private static final int KV_SAMPLES_PER_LEVEL = 10;  // Samples per power level
    private static final double KV_STEADY_STATE_SAMPLES = 5;  // Samples to wait for steady state

    // Results storage
    private double foundKS = 0.0;
    private double foundKV = 0.0;

    /**
     * Creates a new automatic feedforward tuning command.
     *
     * @param drive the drive subsystem to tune
     */
    public AutoTuneFeedforward(Drive drive) {
        this.drive = drive;
        this.telemetry = Robot.getInstance().telemetry;

        addCommands(
            new InstantCommand(() -> telemetry.log().add("=== FEEDFORWARD TUNING STARTED ===")),
            new InstantCommand(() -> telemetry.log().add("ROBOT MUST BE ON BLOCKS!")),
            new InstantCommand(() -> telemetry.log().add("Clear area around robot!")),
            new WaitCommand(1.0),  // Safety pause

            // Step 1: Tune kS (Static Friction)
            new TuneKSCommand(),

            // Pause between tests
            new InstantCommand(() -> drive.swerve.stop()),
            new WaitCommand(1.0),

            // Step 2: Tune kV (Velocity Constant)
            new TuneKVCommand(),

            // Report results
            new InstantCommand(() -> reportResults()),
            new InstantCommand(() -> drive.swerve.stop())
        );
    }

    /**
     * Tunes kS (static friction constant).
     * <p>Ramps power from 0 to 30% until wheels just start spinning.</p>
     */
    private class TuneKSCommand extends Command {

        private double currentPower = 0.0;
        private boolean found = false;

        public TuneKSCommand() {
            addRequirements(drive);
        }

        @Override
        public void initialize() {
            currentPower = 0.0;
            found = false;
            telemetry.log().add("Tuning kS (Static Friction)...");
            telemetry.log().add("Ramping power until wheels spin...");
        }

        @Override
        public void execute() {
            if (found) return;

            drive.swerve.drive(new ChassisSpeeds(currentPower, 0, 0));

            // Check if wheels are spinning
            double avgVelocity = getAverageDriveVelocity();

            if (avgVelocity > KS_VELOCITY_THRESHOLD) {
                foundKS = currentPower;
                found = true;
                telemetry.log().add(String.format("kS FOUND: %.3f", foundKS));
                telemetry.log().add(String.format("  (Wheels started spinning at %.3f power)", foundKS));
            } else {
                currentPower += KS_POWER_INCREMENT;
                if (currentPower > KS_MAX_POWER) {
                    foundKS = 0.15;  // Default value
                    found = true;
                    telemetry.log().add("kS TUNE TIMEOUT - Using default: 0.15");
                }
            }

            telemetry.addData("kS Test Power", currentPower);
            telemetry.addData("kS Velocity", avgVelocity);
        }

        @Override
        public boolean isFinished() {
            return found;
        }

        @Override
        public void end(boolean interrupted) {
            drive.swerve.stop();
        }
    }

    /**
     * Tunes kV (velocity constant).
     * <p>Runs motors at multiple power levels, performs linear regression.</p>
     */
    private class TuneKVCommand extends Command {

        private int currentLevel = 0;
        private int currentSample = 0;
        private final double[][] velocityData = new double[KV_TEST_POWERS.length][KV_SAMPLES_PER_LEVEL];

        public TuneKVCommand() {
            addRequirements(drive);
        }

        @Override
        public void initialize() {
            telemetry.log().add("Tuning kV (Velocity Constant)...");
            telemetry.log().add("Testing power levels: 20%, 40%, 60%, 80%");

            // Initialize data array
            for (int i = 0; i < velocityData.length; i++) {
                for (int j = 0; j < velocityData[i].length; j++) {
                    velocityData[i][j] = 0.0;
                }
            }
        }

        @Override
        public void execute() {
            if (currentLevel >= KV_TEST_POWERS.length) {
                // All levels complete, calculate kV
                foundKV = calculateKV();
                telemetry.log().add(String.format("kV FOUND: %.3f", foundKV));
                return;
            }

            double testPower = KV_TEST_POWERS[currentLevel];
            drive.swerve.drive(new ChassisSpeeds(testPower, 0, 0));

            // Wait for steady state
            if (currentSample >= KV_STEADY_STATE_SAMPLES) {
                // Collect data sample
                double avgVelocity = getAverageDriveVelocity();
                velocityData[currentLevel][currentSample - KV_STEADY_STATE_SAMPLES] = avgVelocity;

                currentSample++;

                if (currentSample >= KV_SAMPLES_PER_LEVEL) {
                    // Move to next power level
                    currentLevel++;
                    currentSample = 0;
                    telemetry.log().add(String.format("Level %d complete: %.2f in/s at %.0f%% power",
                        currentLevel, avgVelocity, testPower * 100));
                }
            } else {
                currentSample++;
            }

            telemetry.addData("kV Level", currentLevel + 1);
            telemetry.addData("kV Power", testPower);
            telemetry.addData("kV Sample", currentSample);
            telemetry.addData("kV Velocity", getAverageDriveVelocity());
        }

        @Override
        public boolean isFinished() {
            return currentLevel >= KV_TEST_POWERS.length;
        }

        @Override
        public void end(boolean interrupted) {
            drive.swerve.stop();
        }

        /**
         * Calculates kV using linear regression.
         */
        private double calculateKV() {
            // Calculate average velocity at each power level
            double[] avgVelocities = new double[KV_TEST_POWERS.length];
            for (int i = 0; i < KV_TEST_POWERS.length; i++) {
                double sum = 0.0;
                int count = 0;
                for (int j = KV_STEADY_STATE_SAMPLES; j < KV_SAMPLES_PER_LEVEL; j++) {
                    if (velocityData[i][j] > 0) {  // Only use valid samples
                        sum += velocityData[i][j];
                        count++;
                    }
                }
                avgVelocities[i] = count > 0 ? sum / count : 0.0;
            }

            // Linear regression: find slope of power vs velocity fraction
            double sumX = 0, sumY = 0, sumXY = 0, sumX2 = 0;
            int n = KV_TEST_POWERS.length;

            for (int i = 0; i < n; i++) {
                double velocityFraction = avgVelocities[i] / MAX_DRIVE_VELOCITY;
                sumX += velocityFraction;
                sumY += KV_TEST_POWERS[i];
                sumXY += velocityFraction * KV_TEST_POWERS[i];
                sumX2 += velocityFraction * velocityFraction;
            }

            double slope = (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX);
            double intercept = (sumY - slope * sumX) / n;

            // kV is the slope (kS is the intercept)
            telemetry.log().add("kV Regression Data:");
            for (int i = 0; i < n; i++) {
                telemetry.log().add(String.format("  Power: %.1f%% → Velocity: %.1f in/s",
                    KV_TEST_POWERS[i] * 100, avgVelocities[i]));
            }
            telemetry.log().add(String.format("  Slope (kV): %.3f", slope));
            telemetry.log().add(String.format("  Intercept (kS): %.3f", intercept));

            return slope;
        }
    }

    /**
     * Gets average drive velocity across all modules.
     * <p>TODO: Implement this by reading from OctoSwerveDrivetrain</p>
     */
    private double getAverageDriveVelocity() {
        // Placeholder - implement by reading actual module velocities
        return 0.0;
    }

    /**
     * Reports tuning results to telemetry.
     */
    private void reportResults() {
        telemetry.log().add("=== FEEDFORWARD TUNING COMPLETE ===");
        telemetry.log().add(String.format("RESULTS:"));
        telemetry.log().add(String.format("kS (Static Friction): %.3f", foundKS));
        telemetry.log().add(String.format("kV (Velocity): %.3f", foundKV));
        telemetry.log().add("");
        telemetry.log().add("UPDATE Constants.java WITH:");
        telemetry.log().add(String.format("DRIVE_KS = %.3f;", foundKS));
        telemetry.log().add(String.format("DRIVE_KV = %.3f;", foundKV));
        telemetry.log().add("");
        telemetry.log().add("Then restart robot to apply new constants!");
        telemetry.log().add("===================================");
    }
}
