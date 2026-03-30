package org.firstinspires.ftc.teamcode.commandbase.commands;

import com.seattlesolvers.solverslib.command.CommandBase;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.globals.Robot;

/**
 * Default command for manual driver-controlled robot movement during TeleOp.
 *
 * <p>This command runs continuously during TeleOp to translate driver gamepad inputs
 * into robot motion. It is bound as the default command for the Drive subsystem,
 * meaning it automatically runs whenever no other commands are using the drive.</p>
 *
 * <h3>Gamepad Controls:</h3>
 * <p>Uses the standard FTC driver layout (Logitech/FghterStick style):</p>
 * <ul>
 *   <li><b>Left Stick Y:</b> Forward/Backward (up = forward, down = backward)</li>
 *   <li><b>Left Stick X:</b> Left/Right strafing (left = strafe left, right = strafe right)</li>
 *   <li><b>Right Stick X:</b> Robot rotation (left = turn CCW, right = turn CW)</li>
 * </ul>
 *
 * <h3>Field-Centric vs Robot-Centric:</h3>
 * <p>The command supports two driving modes, selected at construction:</p>
 * <ul>
 *   <li><b>Field-Centric (recommended):</b> Controls are relative to the field.
 *       "Forward" always moves the robot away from the driver, regardless of robot
 *       orientation. Best for game piece manipulation and consistent movement.</li>
 *   <li><b>Robot-Centric:</b> Controls are relative to the robot. "Forward" moves
 *       in the direction the robot is currently facing. Best for fine adjustments
 *       or when vision/localization is unreliable.</li>
 * </ul>
 *
 * <h3>Input Processing:</h3>
 * <p>Raw gamepad values are processed as follows:</p>
 * <ul>
 *   <li><b>Y-Axis Inversion:</b> FTC gamepads report up as negative, so inputs are
 *       negated to make up = positive (forward)</li>
 *   <li><b>Range:</b> Inputs are in [-1.0, 1.0] from the gamepad</li>
 *   <li><b>Deadzone:</b> Consider adding a small deadzone to prevent drift</li>
 * </ul>
 *
 * <h3>Command Lifecycle:</h3>
 * <ul>
 *   <li><b>Initialize:</b> No special initialization needed</li>
 *   <li><b>Execute:</b> Called every loop (~50Hz) to send gamepad inputs to drive</li>
 *   <li><b>Is Finished:</b> Returns false (continuous command, never finishes on its own)</li>
 *   <li><b>End:</b> Stops the drivetrain when command is interrupted or replaced</li>
 * </ul>
 *
 * <h3>Usage Example:</h3>
 * <pre>
 * // In BaseTeleOp.initialize():
 * robot.drive.setDefaultCommand(new TeleOpDrive(gamepad1, true));
 * </pre>
 *
 * @see org.firstinspires.ftc.teamcode.commandbase.subsystems.Drive
 * @see Drive#setTeleOpDrive(double, double, double, boolean)
 */
public class TeleOpDrive extends CommandBase {

    /**
     * Reference to the robot singleton for accessing the drive subsystem.
     */
    private final Robot robot = Robot.getInstance();

    /**
     * The gamepad providing driver input.
     * Typically gamepad1 for the primary driver.
     */
    private final Gamepad gamepad;

    /**
     * Whether to use field-centric or robot-centric driving.
     * True = field-centric (recommended), False = robot-centric.
     */
    private final boolean fieldCentric;

    /**
     * Last processed controller inputs for telemetry.
     */
    private double lastForward = 0.0;
    private double lastLateral = 0.0;
    private double lastTurn = 0.0;

    /**
     * Constructs a TeleOpDrive command with the specified gamepad and driving mode.
     *
     * <p>This command requires the Drive subsystem, ensuring it doesn't conflict with
     * autonomous commands or other drive commands. When another command requires the
     * drive subsystem, this command will be interrupted.</p>
     *
     * @param gamepad the gamepad providing driver input (typically gamepad1)
     * @param fieldCentric if true, use field-centric drive; if false, use robot-centric
     */
    public TeleOpDrive(Gamepad gamepad, boolean fieldCentric) {
        this.gamepad = gamepad;
        this.fieldCentric = fieldCentric;

        // Require the Drive subsystem so we don't conflict with autos
        addRequirements(robot.drive);
    }

    /**
     * Executes the drive command by reading gamepad inputs and sending them to the drivetrain.
     *
     * <p>This method is called approximately 50 times per second by the command scheduler.
     * It:</p>
     * <ol>
     *   <li>Reads gamepad joystick positions</li>
     *   <li>Inverts Y-axis (FTC gamepads report up as negative)</li>
     *   <li>Sends inputs to Drive subsystem for kinematics calculation and motor control</li>
     * </ol>
     *
     * <h3>Input Mapping:</h3>
     * <ul>
     *   <li><b>Forward:</b> -left_stick_y (negated because FTC gamepads are flipped)</li>
     *   <li><b>Lateral:</b> -left_stick_x (negated for left-positive convention)</li>
     *   <li><b>Turn:</b> -right_stick_x (negated for left-positive CCW)</li>
     * </ul>
     *
     * <p>All inputs are in the range [-1.0, 1.0]. Consider adding deadzones:</p>
     * <pre>
     * double forward = Math.abs(-gamepad.left_stick_y) > 0.1 ? -gamepad.left_stick_y : 0;
     * </pre>
     *
     * @see org.firstinspires.ftc.teamcode.commandbase.subsystems.Drive#setTeleOpDrive(double, double, double, boolean)
     */
    @Override
    public void execute() {
        // Assuming standard FTC driver mapping:
        // Left stick Y (up/down) - Forward
        // Left stick X (left/right) - Lateral/Strafe
        // Right stick X (left/right) - Heading/Turn

        // FTC Gamepads flip Y axes (up is negative).
        lastForward = -gamepad.left_stick_y;
        lastLateral = -gamepad.left_stick_x;
        lastTurn = -gamepad.right_stick_x;

        robot.drive.setTeleOpDrive(lastForward, lastLateral, lastTurn, fieldCentric);
    }

    /**
     * Checks if the command has finished execution.
     *
     * <p>This method always returns false, indicating that this is a continuous command
     * that should run indefinitely. The command will only stop when:</p>
     * <ul>
     *   <li>Another command requiring the Drive subsystem is scheduled (interrupts this)</li>
     *   <li>The OpMode is terminated (end() is called with interrupted=true)</li>
     *   <li>The command is explicitly canceled from code</li>
     * </ul>
     *
     * <p><b>Design Pattern:</b> Default commands should typically return false from
     * isFinished() so they continuously run and provide manual control whenever no
     * other commands are active.</p>
     *
     * @return false always (continuous command)
     */
    @Override
    public boolean isFinished() {
        return false; // Continuous command during TeleOp
    }

    /**
     * Called when the command ends (either naturally or by interruption).
     *
     * <p>This cleanup method stops the drivetrain by sending zero velocities. This prevents
     * the robot from continuing to move after the command ends. Situations where this is
     * called:</p>
     * <ul>
     *   <li>Another command requiring the Drive subsystem is scheduled</li>
     *   <li>The OpMode is terminated (match ends, E-stop pressed)</li>
     *   <li>The robot is disabled</li>
     * </ul>
     *
     * <p><b>Safety:</b> Always stopping the drivetrain on command end ensures the robot
     * doesn't continue with the last commanded velocities after the command terminates.</p>
     *
     * @param interrupted true if the command was interrupted by another command,
     *                    false if it ended naturally (should never be false for this command)
     * @see org.firstinspires.ftc.teamcode.commandbase.subsystems.Drive#setTeleOpDrive(double, double, double, boolean)
     */
    @Override
    public void end(boolean interrupted) {
        // Stop the drivetrain when the command ends
        robot.drive.setTeleOpDrive(0, 0, 0, fieldCentric);
    }

    // Telemetry getters

    /**
     * Gets the last processed forward input from the gamepad.
     * @return forward value in range [-1.0, 1.0]
     */
    public double getLastForward() {
        return lastForward;
    }

    /**
     * Gets the last processed lateral input from the gamepad.
     * @return lateral value in range [-1.0, 1.0]
     */
    public double getLastLateral() {
        return lastLateral;
    }

    /**
     * Gets the last processed turn input from the gamepad.
     * @return turn value in range [-1.0, 1.0]
     */
    public double getLastTurn() {
        return lastTurn;
    }

    /**
     * Gets whether field-centric driving is enabled.
     * @return true if field-centric, false if robot-centric
     */
    public boolean isFieldCentric() {
        return fieldCentric;
    }
}
