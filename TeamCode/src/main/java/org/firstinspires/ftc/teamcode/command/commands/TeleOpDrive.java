package org.firstinspires.ftc.teamcode.command.commands;

import com.seattlesolvers.solverslib.command.CommandBase;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot;

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
 *   <li><b>Right Trigger:</b> Speed throttle (released = 50%, fully pressed = 100%)</li>
 *   <li><b>Y Button:</b> Reset field-centric heading (sets current heading as forward)</li>
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
 *   <li><b>Speed Scaling:</b> Base speed is 50%, right trigger scales up to 100%</li>
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
 * <h3>Speed Control:</h3>
 * <p>The robot operates at 50% speed by default for precise control. The right trigger
 * acts as a throttle to increase speed up to 100% when fully pressed. This applies to
 * all movement (forward, strafe, and rotation).</p>
 *
 * <h3>Heading Reset:</h3>
 * <p>Press Y to reset the field-centric heading. This sets the robot's current orientation
 * as the new "forward" direction. Useful if the robot gets confused about its heading
 * or if you want to quickly realign field-centric controls.</p>
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
    private double lastSpeedScale = 0.5;

    /**
     * Speed control constants for throttle system.
     */
    private static final double BASE_SPEED_SCALE = 0.5;  // 50% speed when trigger not pressed
    private static final double MAX_SPEED_BONUS = 0.5;    // Additional 50% when trigger fully pressed

    /**
     * Heading reset state tracking.
     */
    private boolean lastYButtonState = false;
    private int headingResetCount = 0;
    private double lastHeadingResetTime = 0.0;
    private final ElapsedTime resetTimer = new ElapsedTime();

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
     *   <li>Checks for Y button press to reset field-centric heading</li>
     *   <li>Reads gamepad joystick positions</li>
     *   <li>Calculates speed scale from right trigger (50% base to 100% max)</li>
     *   <li>Inverts Y-axis (FTC gamepads report up as negative)</li>
     *   <li>Applies speed scaling to all inputs</li>
     *   <li>Sends inputs to Drive subsystem for kinematics calculation and motor control</li>
     * </ol>
     *
     * <h3>Input Mapping:</h3>
     * <ul>
     *   <li><b>Forward:</b> -left_stick_y (negated because FTC gamepads are flipped)</li>
     *   <li><b>Lateral:</b> -left_stick_x (negated for left-positive convention)</li>
     *   <li><b>Turn:</b> -right_stick_x (negated for left-positive CCW)</li>
     *   <li><b>Speed Scale:</b> BASE_SPEED_SCALE + (right_trigger * MAX_SPEED_BONUS)</li>
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
        // Handle heading reset on Y button press (rising edge)
        if (gamepad.y && !lastYButtonState && fieldCentric) {
            resetHeading();
        }
        lastYButtonState = gamepad.y;

        // Calculate speed scale from right trigger
        // Trigger ranges from 0.0 (released) to 1.0 (fully pressed)
        // Speed scale ranges from 50% (trigger released) to 100% (trigger fully pressed)
        lastSpeedScale = BASE_SPEED_SCALE + (gamepad.right_trigger * MAX_SPEED_BONUS);

        // Assuming standard FTC driver mapping:
        // Left stick Y (up/down) - Forward
        // Left stick X (left/right) - Lateral/Strafe
        // Right stick X (left/right) - Heading/Turn

        // FTC Gamepads flip Y axes (up is negative).
        lastForward = -gamepad.left_stick_y * lastSpeedScale;
        lastLateral = -gamepad.left_stick_x * lastSpeedScale;
        lastTurn = -gamepad.right_stick_x * lastSpeedScale;

        robot.drive.setTeleOpDrive(lastForward, lastLateral, lastTurn, fieldCentric);
    }

    /**
     * Resets the field-centric heading to the robot's current orientation.
     *
     * <p>This sets the robot's current heading as the new "forward" direction for
     * field-centric driving. Useful if the robot's heading becomes desynchronized or
     * if you want to quickly realign the controls.</p>
     *
     * <p>Only works in field-centric mode.</p>
     */
    private void resetHeading() {
        if (!fieldCentric) return;

        // Get current pose
        Pose currentPose = robot.drive.follower.getPoseTracker().getPose();

        // Create new pose with same x, y but heading = 0
        Pose resetPose = new Pose(currentPose.getX(), currentPose.getY(), 0.0);

        // Reset the localizer heading
        robot.drive.follower.setStartingPose(resetPose);

        // Track reset event for telemetry
        headingResetCount++;
        lastHeadingResetTime = resetTimer.seconds();
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
     * Gets the current speed scale factor.
     * @return speed scale (0.5 to 1.0, representing 50% to 100%)
     */
    public double getSpeedScale() {
        return lastSpeedScale;
    }

    /**
     * Gets the current speed as a percentage.
     * @return speed percentage (50 to 100)
     */
    public double getSpeedPercentage() {
        return lastSpeedScale * 100.0;
    }

    /**
     * Gets the right trigger value (throttle input).
     * @return trigger value (0.0 to 1.0)
     */
    public double getThrottleInput() {
        return gamepad.right_trigger;
    }

    /**
     * Gets the Y button state.
     * @return true if Y button is pressed
     */
    public boolean getYButton() {
        return gamepad.y;
    }

    /**
     * Gets the number of heading resets performed.
     * @return reset count
     */
    public int getHeadingResetCount() {
        return headingResetCount;
    }

    /**
     * Gets the time since last heading reset.
     * @return seconds since last reset, or -1 if never reset
     */
    public double getTimeSinceLastReset() {
        if (headingResetCount == 0) {
            return -1;
        }
        return resetTimer.seconds() - lastHeadingResetTime;
    }

    /**
     * Gets whether field-centric driving is enabled.
     * @return true if field-centric, false if robot-centric
     */
    public boolean isFieldCentric() {
        return fieldCentric;
    }

    /**
     * Outputs comprehensive telemetry data for the TeleOpDrive command.
     *
     * <p>This method logs all button states, throttle inputs, and system status
     * to the telemetry output. Call this from your OpMode's telemetry section.</p>
     *
     * <h3>Output Data:</h3>
     * <ul>
     *   <li>Drive Mode: Field-Centric or Robot-Centric</li>
     *   <li>Speed: Current speed percentage (50-100%)</li>
     *   <li>Throttle: Right trigger value (0-100%)</li>
     *   <li>Y Button: Current state and press count</li>
     *   <li>Last Reset: Time since last heading reset</li>
     *   <li>Inputs: Raw forward, lateral, and turn values</li>
     * </ul>
     */
    public void outputTelemetry(com.seattlesolvers.solverslib.util.TelemetryData telemetry) {
        telemetry.addData("Drive Mode", fieldCentric ? "Field-Centric" : "Robot-Centric");
        telemetry.addData("Speed", String.format("%.1f%%", getSpeedPercentage()));
        telemetry.addData("Throttle", String.format("%.1f%%", getThrottleInput() * 100.0));
        telemetry.addData("Y Button", gamepad.y ? "PRESSED" : "released");
        telemetry.addData("Heading Resets", String.format("%d", headingResetCount));

        if (headingResetCount > 0) {
            double timeSinceReset = getTimeSinceLastReset();
            telemetry.addData("Last Reset", String.format("%.1fs ago", timeSinceReset));
        } else {
            telemetry.addData("Last Reset", "Never");
        }

        telemetry.addData("Forward Input", String.format("%.3f", lastForward));
        telemetry.addData("Lateral Input", String.format("%.3f", lastLateral));
        telemetry.addData("Turn Input", String.format("%.3f", lastTurn));
    }
}
