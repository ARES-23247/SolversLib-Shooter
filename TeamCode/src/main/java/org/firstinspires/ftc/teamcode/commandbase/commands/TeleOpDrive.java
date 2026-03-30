package org.firstinspires.ftc.teamcode.commandbase.commands;

import com.seattlesolvers.solverslib.command.CommandBase;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.globals.Robot;

public class TeleOpDrive extends CommandBase {
    private final Robot robot = Robot.getInstance();
    private final Gamepad gamepad;
    private final boolean fieldCentric;

    public TeleOpDrive(Gamepad gamepad, boolean fieldCentric) {
        this.gamepad = gamepad;
        this.fieldCentric = fieldCentric;

        // Require the Drive subsystem so we don't conflict with autos
        addRequirements(robot.drive);
    }

    @Override
    public void execute() {
        // Assuming standard FTC driver mapping:
        // Left stick Y (up/down) - Forward
        // Left stick X (left/right) - Lateral/Strafe
        // Right stick X (left/right) - Heading/Turn
        
        // FTC Gamepads flip Y axes (up is negative).
        double forward = -gamepad.left_stick_y;
        double lateral = -gamepad.left_stick_x;
        double turn = -gamepad.right_stick_x;

        robot.drive.setTeleOpDrive(forward, lateral, turn, fieldCentric);
    }

    @Override
    public boolean isFinished() {
        return false; // Continuous command during TeleOp
    }

    @Override
    public void end(boolean interrupted) {
        // Stop the drivetrain when the command ends
        robot.drive.setTeleOpDrive(0, 0, 0, fieldCentric);
    }
}
