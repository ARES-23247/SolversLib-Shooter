package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous; 

import org.firstinspires.ftc.teamcode.globals.Robot;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.Drive;
import org.firstinspires.ftc.teamcode.util.tuning.SteeringTuner;

import static org.firstinspires.ftc.teamcode.globals.Constants.*;

/**
 * OpMode for automatic steering PID tuning.
 *
 * <p>This OpMode runs the steering auto-tuner and displays results on telemetry.</p>
 *
 * <h3>HOW TO USE:</h3>
 * <ol>
 *   <li><b>PREPARE:</b> Put robot on blocks OR field floor (steering doesn't move robot)</li>
 *   <li><b>RUN:</b> Start this OpMode</li>
 *   <li><b>WAIT:</b> Let tuner run (~2-3 minutes for all 4 modules)</li>
 *   <li><b>READ:</b> Check driver station telemetry for results</li>
 *   <li><b>APPLY:</b> Update Constants.java with results</li>
 * </ol>
 *
 * <h3>SAFETY NOTES:</h3>
 * <ul>
 *   <li>Robot can be on blocks OR field floor</li>
 *   <li>Clear area around robot (wheels will rotate in place)</li>
 *   <li>Stay clear of moving parts</li>
 *   <li>Have finger on STOP button</li>
 * </ul>
 *
 * <h3>What Happens:</h3>
 * <ol>
 *   <li>5-second safety countdown</li>
 *   <li>For each of 4 modules:
 *     <ul>
 *       <li>Measure max steering velocity (~2 sec)</li>
 *       <li>Tune kS (~3 sec)</li>
 *       <li>Tune P gain (~20 sec)</li>
 *       <li>Tune I gain (~15 sec)</li>
 *       <li>Tune D gain (~15 sec)</li>
 *     </ul>
 *   </li>
 *   <li>Display averaged results to telemetry</li>
 * </ol>
 *
 * <h3>About Steering Feedforward:</h3>
 * <p>Unlike drive motors, steering motors don't fight ground friction. They only need:</p>
 * <ul>
 *   <li><b>kS (small):</b> Overcome gear friction (typical: 0.02-0.05)</li>
 *   <li><b>Ki (integral):</b> Eliminate steady-state error (typical: 0.001-0.005)</li>
 *   <li><b>kV (optional):</b> Only for velocity-based steering (rarely used)</li>
 * </ul>
 * <p>Adding kS + Ki helps the steering reach and hold precise angles more consistently,
 * especially when the robot is stationary or moving slowly.</p>
 *
 * @see org.firstinspires.ftc.teamcode.util.SteeringTuner
 * @see org.firstinspires.ftc.teamcode.util.FeedforwardTuner
 */
@Autonomous(name = "Auto Tune Steering", group = "Tuning")
@TeleOp(name = "Auto Tune Steering")
public class AutoTuneSteering extends LinearOpMode {

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
        SteeringTuner tuner = new SteeringTuner(drive);

        telemetry.log().add("===========================================");
        telemetry.log().add("   STEERING AUTOTUNE OP MODE");
        telemetry.log().add("===========================================");
        telemetry.log().add("");
        telemetry.log().add("Starting automatic steering tuning...");
        telemetry.log().add("");
        telemetry.log().add("IMPORTANT:");
        telemetry.log().add("- Robot can be on blocks OR field floor");
        telemetry.log().add("- Clear area around robot");
        telemetry.log().add("- Wheels will rotate in place!");
        telemetry.log().add("- This will take ~2-3 minutes");
        telemetry.update();

        if (isStopRequested()) return;

        // Run the tuner (takes ~3-4 minutes)
        double[] results = tuner.tuneAllModules();

        // Results are automatically logged to telemetry
        // Update Constants.java with:
        //   SWERVE_SERVO_PIDF.p = results[0];
        //   SWERVE_SERVO_PIDF.i = results[1];
        //   SWERVE_SERVO_PIDF.d = results[2];
        //   STEERING_KS = results[3];
        //   MAX_STEERING_VELOCITY = results[4];

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
