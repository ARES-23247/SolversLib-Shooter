package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.util.TelemetryData;

import org.firstinspires.ftc.teamcode.commandbase.commands.TeleOpDrive;
import org.firstinspires.ftc.teamcode.globals.Robot;

/**
 * TeleOp (driver-controlled) OpMode for the ARES robot.
 *
 * <p>This OpMode is the main entry point for driver-controlled operation during the
 * TeleOp period of FTC matches. It extends {@link CommandOpMode} to use the command-based
 * architecture pattern.</p>
 *
 * <p><b>Key Responsibilities:</b></p>
 * <ul>
 *   <li>Initialize robot hardware and subsystems</li>
 *   <li>Bind default commands to subsystems</li>
 *   <li>Run the command scheduler every loop iteration</li>
 *   <li>Clear PhotonCore bulk caches</li>
 *   <li>Provide loop timing telemetry</li>
 * </ul>
 *
 * <p><b>Operation:</b> The TeleOpDrive command is bound as the default command for the
 * drive subsystem, enabling continuous driver control via gamepad1. Field-centric driving
 * is enabled to simplify control for drivers.</p>
 *
 * <p><b>Telemetry:</b> Sends telemetry to both the driver station phone and FTC Dashboard
 * via MultipleTelemetry for dual output.</p>
 *
 * @see com.seattlesolvers.solverslib.command.CommandOpMode
 * @see org.firstinspires.ftc.teamcode.commandbase.commands.TeleOpDrive
 * @see org.firstinspires.ftc.teamcode.globals.Robot
 */
@TeleOp(name = "Base TeleOp", group = "TeleOp")
public class BaseTeleOp extends CommandOpMode {

    /**
     * Timer for measuring loop execution time.
     * Used to monitor code performance and detect loop time spikes.
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
     * Initializes the robot for TeleOp operation.
     *
     * <p>This method is called once when the OpMode is initialized. It performs the following:</p>
     * <ol>
     *   <li>Resets the command scheduler to clear any previous state</li>
     *   <li>Initializes all robot hardware via {@link Robot#init(HardwareMap)}</li>
     *   <li>Sets up telemetry output</li>
     *   <li>Binds the TeleOpDrive command as the default command for the drive subsystem</li>
     * </ol>
     *
     * <p><b>Default Command:</b> The TeleOpDrive command is bound to the drive subsystem,
     * which enables continuous driver control via gamepad1 with field-centric driving enabled.</p>
     */
    @Override
    public void initialize() {
        super.reset();

        robot.init(hardwareMap);
        robot.telemetry = this.telemetryData;

        // Bind default drive command to Gamepad 1 with Field Centric routing enabled
        robot.drive.setDefaultCommand(new TeleOpDrive(gamepad1, true));
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

        PhotonCore.CONTROL_HUB.clearBulkCache();
        PhotonCore.EXPANSION_HUB.clearBulkCache();
    }

    /**
     * Main loop method that runs every iteration during TeleOp.
     *
     * <p>This method is called approximately 50 times per second by the FTC SDK.
     * It performs the following operations:</p>
     * <ol>
     *   <li>Measures and reports loop execution time for performance monitoring</li>
     *   <li>Outputs TeleOpDrive telemetry (throttle, button states, heading resets)</li>
     *   <li>Runs the robot's update loop which includes:
     *       <ul>
     *         <li>Command scheduler execution</li>
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

        // Output TeleOpDrive telemetry (throttle, buttons, heading resets)
        if (robot.drive.getDefaultCommand() instanceof org.firstinspires.ftc.teamcode.commandbase.commands.TeleOpDrive) {
            org.firstinspires.ftc.teamcode.commandbase.commands.TeleOpDrive driveCommand =
                (org.firstinspires.ftc.teamcode.commandbase.commands.TeleOpDrive) robot.drive.getDefaultCommand();
            driveCommand.outputTelemetry(telemetryData);
        }

        // Run the master loop which ticks subsystems, command scheduler, and clears bulk cache
        robot.profiler.start("Run + Update");
        robot.updateLoop(telemetryData);
        robot.profiler.end("Run + Update");
    }
}
