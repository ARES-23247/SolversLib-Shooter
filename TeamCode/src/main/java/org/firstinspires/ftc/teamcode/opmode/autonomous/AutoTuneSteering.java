package org.firstinspires.ftc.teamcode.opmode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous; 

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drive;
import org.firstinspires.ftc.teamcode.util.tuning.SteeringTuner;

import static org.firstinspires.ftc.teamcode.Constants.*;

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
 * @see org.firstinspires.ftc.teamcode.util.tuning.SteeringTuner
 * @see org.firstinspires.ftc.teamcode.util.tuning.FeedforwardTuner
 */
@Autonomous(name = "Auto Tune Steering", group = "Tuning")

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

        System.out.println("===========================================");
        System.out.println("   STEERING AUTOTUNE OP MODE");
        System.out.println("===========================================");
        System.out.println("");
        System.out.println("Starting automatic steering tuning...");
        System.out.println("");
        System.out.println("IMPORTANT:");
        System.out.println("- Robot can be on blocks OR field floor");
        System.out.println("- Clear area around robot");
        System.out.println("- Wheels will rotate in place!");
        System.out.println("- This will take ~2-3 minutes");
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

        System.out.println("");
        System.out.println("Autotuning complete!");
        System.out.println("Check telemetry above for results.");
        telemetry.update();

        // Wait for user to stop
        while (opModeIsActive() && !isStopRequested()) {
            telemetry.update();
            idle();
        }
    }
}
