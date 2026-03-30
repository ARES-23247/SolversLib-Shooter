package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.globals.Robot;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.Drive;
import org.firstinspires.ftc.teamcode.util.tuning.FeedforwardTuner;

import static org.firstinspires.ftc.teamcode.globals.Constants.*;

/**
 * Example OpMode demonstrating automatic feedforward tuning.
 *
 * <p>This OpMode runs the feedforward auto-tuner and displays results on telemetry.</p>
 *
 * <h3>HOW TO USE:</h3>
 * <ol>
 *   <li><b>PREPARE:</b> Put robot on blocks (wheels off ground)</li>
 *   <li><b>RUN:</b> Start this OpMode</li>
 *   <li><b>WAIT:</b> Let tuner run (~25 seconds)</li>
 *   <li><b>READ:</b> Check driver station telemetry for results</li>
 *   <li><b>APPLY:</b> Update Constants.java with results</li>
 * </ol>
 *
 * <h3>SAFETY NOTES:</h3>
 * <ul>
 *   <li>Robot MUST be on blocks!</li>
 *   <li>Clear area around robot</li>
 *   <li>Wheels will spin up to 80% power</li>
 *   <li>Stay clear of moving parts</li>
 * </ul>
 *
 * <h3>What Happens:</h3>
 * <ol>
 *   <li>5-second safety countdown</li>
 *   <li>Tune kS: Ramp power until wheels spin (~3 sec)</li>
 *   <li>Tune kV: Test at 20%, 40%, 60%, 80% power (~15 sec)</li>
 *   <li>Display results to telemetry</li>
 * </ol>
 *
 * @see org.firstinspires.ftc.teamcode.util.FeedforwardTuner
 */
@Autonomous(name = "Auto Tune Feedforward", group = "Tuning")
@TeleOp(name = "Auto Tune Feedforward")
public class AutoTuneOpMode extends LinearOpMode {

    @Override
    public void runOpMode() {
        // Initialize robot
        Robot robot = Robot.getInstance();
        robot.init(hardwareMap);

        // Initialize drive subsystem
        Drive drive = new Drive();
        drive.init();

        // Wait for start
        waitForStart();

        // Run auto-tuner
        FeedforwardTuner tuner = new FeedforwardTuner(drive);

        telemetry.log().add("===========================================");
        telemetry.log().add("   AUTOTUNE OP MODE");
        telemetry.log().add("===========================================");
        telemetry.log().add("");
        telemetry.log().add("Starting automatic feedforward tuning...");
        telemetry.log().add("");
        telemetry.log().add("IMPORTANT: Robot must be on FIELD FLOOR!");
        telemetry.log().add("- Normal driving surface (carpet/tiles)");
        telemetry.log().add("- Clear area 4x4 feet minimum");
        telemetry.log().add("- Robot will drive forward!");
        telemetry.update();

        if (isStopRequested()) return;

        // Run the tuner (takes ~25 seconds)
        double[] results = tuner.tune();

        // Results are automatically logged to telemetry
        // Update Constants.java with:
        //   DRIVE_KS = results[0];
        //   DRIVE_KV = results[1];

        telemetry.log().add("");
        telemetry.log().add("Autotuning complete!");
        telemetry.log().add("Check telemetry above for results.");
        telemetry.update();

        // Wait for user to stop
        while (opModeIsActive() && !isStopRequested()) {
            telemetry.update();
            idle();
        }
    }
}
