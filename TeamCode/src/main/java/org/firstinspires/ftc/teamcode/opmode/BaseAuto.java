package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.util.TelemetryData;

import org.firstinspires.ftc.teamcode.globals.Robot;

/**
 * Autonomous OpMode for the ARES robot.
 *
 * <p>This OpMode is the entry point for the autonomous period of FTC matches. It extends
 * {@link CommandOpMode} to use the command-based architecture pattern. Currently serves as
 * a template for implementing autonomous routines.</p>
 *
 * <p><b>Key Responsibilities:</b></p>
 * <ul>
 *   <li>Initialize robot hardware and subsystems</li>
 *   <li>Schedule autonomous command sequences</li>
 *   <li>Run the command scheduler every loop iteration</li>
 *   <li>Clear PhotonCore bulk caches</li>
 *   <li>Provide loop timing telemetry</li>
 * </ul>
 *
 * <p><b>Current State:</b> This is a template implementation. Autonomous paths need to be
 * generated using Pedro Pathing and scheduled via {@link SequentialCommandGroup}. See the
 * TODO comment in {@link #initialize()} for implementation details.</p>
 *
 * <p><b>Autonomous Development:</b> To implement autonomous routines:</p>
 * <ol>
 *   <li>Generate paths using the Pedro Pathing tuning tool</li>
 *   <li>Create command classes for each auto action</li>
 *   <li>Compose commands into a SequentialCommandGroup</li>
 *   <li>Schedule the group in {@link #initialize()}</li>
 * </ol>
 *
 * @see com.seattlesolvers.solverslib.command.CommandOpMode
 * @see com.seattlesolvers.solverslib.command.SequentialCommandGroup
 * @see org.firstinspires.ftc.teamcode.globals.Robot
 * @see <a href="https://pedropathing.com/ Pedro Pathing Documentation</a>
 */
@Autonomous(name = "Base Auto", group = "Autonomous")
public class BaseAuto extends CommandOpMode {

    /**
     * Timer for measuring loop execution time.
     * Used to monitor code performance and detect loop time spikes during autonomous.
     */
    public ElapsedTime timer;

    /**
     * Telemetry instance that sends data to both the driver station and FTC Dashboard.
     * MultipleTelemetry allows simultaneous output to both destinations.
     */
    TelemetryData telemetryData = new TelemetryData(new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry()));

    /**
     * Reference to the singleton robot instance.
     * Used to initialize hardware and run the update loop.
     */
    private final Robot robot = Robot.getInstance();

    /**
     * Initializes the robot for autonomous operation.
     *
     * <p>This method is called once when the OpMode is initialized. It performs the following:</p>
     * <ol>
     *   <li>Resets the command scheduler to clear any previous state</li>
     *   <li>Initializes all robot hardware via {@link Robot#init(HardwareMap)}</li>
     *   <li>Sets up telemetry output</li>
     *   <li>Schedules autonomous command sequences</li>
     * </ol>
     *
     * <p><b>TODO:</b> Implement autonomous path generation and command scheduling.
     * Current steps needed:</p>
     * <ul>
     *   <li>Generate autonomous paths using Pedro Pathing web tool</li>
     *   <li>Create path-following commands for each auto segment</li>
     *   <li>Compose commands into SequentialCommandGroups</li>
     *   <li>Schedule the group using {@code schedule()}</li>
     * </ul>
     *
     * <p><b>Example structure:</b></p>
     * <pre>{@code
     * schedule(
     *     new SequentialCommandGroup(
     *         new PathCommand("path1"),
     *         new ScoreCommand(),
     *         new PathCommand("path2"),
     *         new ParkCommand()
     *     )
     * );
     * }</pre>
     */
    @Override
    public void initialize() {
        super.reset();

        robot.init(hardwareMap);
        robot.telemetry = this.telemetryData;

        // TODO: Generate auto paths via Pedro Pathing, setup initial positions
        
        // Example schedule of auto routines
        schedule(
                new SequentialCommandGroup(
                        // Auto tasks go here!
                )
        );
    }

    /**
     * Initialization loop that runs repeatedly before the autonomous period starts.
     *
     * <p>This method runs continuously after {@link #initialize()} completes but before
     * the play button is pressed. It is used to:</p>
     * <ul>
     *   <li>Display initialization status to drivers</li>
     *   <li>Clear PhotonCore bulk caches to prepare for operation</li>
     * </ul>
     *
     * <p><b>Purpose:</b> Allows drivers to verify the robot is ready and positioned correctly
     * before starting autonomous. The bulk cache clears prevent stale data from the
     * initialization phase from affecting the first loop iteration.</p>
     */
    @Override
    public void initialize_loop() {
        telemetryData.addData("Status", "Auto Initialized. Waiting for Start.");
        telemetryData.update();

        PhotonCore.CONTROL_HUB.clearBulkCache();
        PhotonCore.EXPANSION_HUB.clearBulkCache();
    }

    /**
     * Main loop method that runs every iteration during autonomous.
     *
     * <p>This method is called approximately 50 times per second by the FTC SDK.
     * It performs the following operations:</p>
     * <ol>
     *   <li>Measures and reports loop execution time for performance monitoring</li>
     *   <li>Runs the robot's update loop which includes:
     *       <ul>
     *         <li>Command scheduler execution (runs scheduled auto commands)</li>
     *         <li>Subsystem periodic updates</li>
     *         <li>Telemetry output</li>
     *         <li>Bulk cache clearing</li>
     *       </ul>
     *   </li>
     *   <li>Profiles the update loop execution time</li>
     * </ol>
     *
     * <p><b>Loop Time Monitoring:</b> Displays the time (in milliseconds) taken for the
     * previous loop iteration. This helps identify performance bottlenecks. Well-optimized
     * autonomous code should run in under 20ms to ensure precise path following.</p>
     *
     * <p><b>Command Execution:</b> The command scheduler automatically runs all scheduled
     * commands. When autonomous completes (no more commands), the scheduler continues to
     * run subsystem periodic methods but executes no commands.</p>
     *
     * @see Robot#updateLoop(TelemetryData)
     * @see com.seattlesolvers.solverslib.command.CommandScheduler#run()
     */
    @Override
    public void run() {
        if (timer == null) {
            timer = new ElapsedTime();
        }

        telemetryData.addData("Auto Loop Time", timer.milliseconds());
        timer.reset();

        robot.profiler.start("Run + Update");
        robot.updateLoop(telemetryData);
        robot.profiler.end("Run + Update");
    }
}
