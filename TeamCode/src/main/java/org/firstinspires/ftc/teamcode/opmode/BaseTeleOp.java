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

@TeleOp(name = "Base TeleOp", group = "TeleOp")
public class BaseTeleOp extends CommandOpMode {
    public ElapsedTime timer;
    TelemetryData telemetryData = new TelemetryData(new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry()));

    private final Robot robot = Robot.getInstance();

    @Override
    public void initialize() {
        super.reset();

        robot.init(hardwareMap);
        robot.telemetry = this.telemetryData;

        // Bind default drive command to Gamepad 1 with Field Centric routing enabled
        robot.drive.setDefaultCommand(new TeleOpDrive(gamepad1, true));
    }

    @Override
    public void initialize_loop() {
        telemetryData.addData("Status", "Initialized. Waiting for Start.");
        telemetryData.update();

        PhotonCore.CONTROL_HUB.clearBulkCache();
        PhotonCore.EXPANSION_HUB.clearBulkCache();
    }

    @Override
    public void run() {
        if (timer == null) {
            timer = new ElapsedTime();
        }

        telemetryData.addData("Loop Time", timer.milliseconds());
        timer.reset();

        // Run the master loop which ticks subsystems, command scheduler, and clears bulk cache
        robot.profiler.start("Run + Update");
        robot.updateLoop(telemetryData);
        robot.profiler.end("Run + Update");
    }
}
