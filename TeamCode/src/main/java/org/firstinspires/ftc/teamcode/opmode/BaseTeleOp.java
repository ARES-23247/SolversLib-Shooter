package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.util.TelemetryData;

import org.firstinspires.ftc.teamcode.Robot;

/**
 * TeleOp (driver-controlled) OpMode for the ARES robot.
 *
 * <p>This OpMode is the main entry point for driver-controlled operation during the
 * TeleOp period of FTC matches. It extends {@link CommandOpMode} to use the command-based
 * architecture pattern.</p>
 *
 * <p><b>Key Responsibilities:</b></p>
 * <ul>
 *   <li>Initialize robot hardware via {@link Robot#init(HardwareMap)}</li>
 *   <li>Execute robot update loop via {@link Robot#updateLoop(TelemetryData)}</li>
 *   <li>Display loop timing telemetry for performance monitoring</li>
 * </ul>
 *
 * <p><b>Operation:</b></p>
 * <ol>
 *   <li><b>Initialization:</b> Calls {@code robot.init(hardwareMap)} to initialize all hardware,
 *       subsystems, and PhotonCore bulk caching</li>
 *   <li><b>Update Loop:</b> Calls {@code robot.updateLoop(telemetryData)} which runs the command
 *       scheduler, updates subsystems, clears PhotonCore caches, and outputs telemetry</li>
 *   <li><b>Telemetry:</b> Outputs to both driver station and Panels Dashboard via MultipleTelemetry.
 *       During competition, Panels Dashboard is unavailable and all data shows on driver station.</li>
 * </ol>
 *
 * <p><b>Delegation:</b> This OpMode delegates hardware management, command scheduling, and
 * bulk cache operations to {@link Robot}. See {@link Robot} class documentation for details
 * on those responsibilities.</p>
 *
 * @see com.seattlesolvers.solverslib.command.CommandOpMode
 * @see org.firstinspires.ftc.teamcode.Robot
 */
@TeleOp(name = "Base TeleOp", group = "TeleOp")
public class BaseTeleOp extends CommandOpMode {

    /**
     * Timer for measuring loop execution time.
     * Used to monitor code performance and detect loop time spikes.
     */
    public ElapsedTime timer;

    /**
     * Telemetry instance for driver station display.
     * Panels dashboard is used separately for web-based visualization during testing.
     */
    TelemetryData telemetryData = new TelemetryData(telemetry);

    /**
     * Reference to the singleton robot instance.
     * Used to initialize hardware and run the update loop.
     */
    private final Robot robot = Robot.getInstance();

    /**
     * Initializes the robot for TeleOp operation.
     *
     * <p>This method is called once when the OpMode is initialized. It performs the following:</p>
     * <ol>
     *   <li>Resets the command scheduler to clear any previous state</li>
     *   <li>Initializes robot hardware by calling {@link Robot#init(HardwareMap)}</li>
     *   <li>Sets up telemetry output with MultipleTelemetry for dual output</li>
     * </ol>
     */
    @Override
    public void initialize() {
        super.reset();

        robot.init(hardwareMap);
        robot.telemetry = this.telemetryData;
    }

    /**
     * Initialization loop that runs repeatedly before the TeleOp period starts.
     *
     * <p>This method runs continuously after {@link #initialize()} completes but before
     * the play button is pressed. It is used to:</p>
     * <ul>
     *   <li>Display initialization status to drivers</li>
     *   <li>Clear PhotonCore bulk caches to prepare for operation</li>
     * </ul>
     *
     * <p><b>Purpose:</b> Allows drivers to verify the robot is ready before starting TeleOp.
     * The bulk cache clears prevent stale data from the initialization phase from affecting
     * the first loop iteration.</p>
     */
    @Override
    public void initialize_loop() {
        telemetryData.addData("Status", "Initialized. Waiting for Start.");
        telemetryData.update();



    }

    /**
     * Main loop method that runs every iteration during TeleOp.
     *
     * <p>This method is called approximately 50 times per second by the FTC SDK.
     * It performs the following operations:</p>
     * <ol>
     *   <li>Measures and reports loop execution time for performance monitoring</li>
     *   <li>Runs the robot's update loop which includes:
     *       <ul>
     *         <li>Command scheduler execution</li>
     *         <li>Subsystem periodic updates</li>
     *         <li>Telemetry output</li>
     *         <li>Bulk cache clearing</li>
     *       </ul>
     *   </li>
     * </ol>
     *
     * <p><b>Loop Time Monitoring:</b> Displays the time (in milliseconds) taken for the
     * previous loop iteration. This helps identify performance bottlenecks. Well-optimized
     * code should run in under 20ms.</p>
     *
     * @see Robot#updateLoop(TelemetryData)
     * @see com.seattlesolvers.solverslib.command.CommandScheduler#run()
     */
    @Override
    public void run() {
        if (timer == null) {
            timer = new ElapsedTime();
        }

        telemetryData.addData("Loop Time", timer.milliseconds());
        timer.reset();

        // Run the master loop which ticks subsystems, command scheduler, and clears bulk cache
        robot.updateLoop(telemetryData);
    }
}
