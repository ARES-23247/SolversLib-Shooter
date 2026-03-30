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

@Autonomous(name = "Base Auto", group = "Autonomous")
public class BaseAuto extends CommandOpMode {
    public ElapsedTime timer;
    TelemetryData telemetryData = new TelemetryData(new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry()));

    private final Robot robot = Robot.getInstance();

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

    @Override
    public void initialize_loop() {
        telemetryData.addData("Status", "Auto Initialized. Waiting for Start.");
        telemetryData.update();

        PhotonCore.CONTROL_HUB.clearBulkCache();
        PhotonCore.EXPANSION_HUB.clearBulkCache();
    }

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
