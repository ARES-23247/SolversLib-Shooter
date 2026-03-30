package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;
import com.seattlesolvers.solverslib.util.TelemetryData;

import org.firstinspires.ftc.teamcode.commandbase.subsystems.Drive;
import org.firstinspires.ftc.teamcode.globals.Robot;

/**
 * Autonomous OpMode for the ARES robot with vision localization.
 *
 * <p>This OpMode implements autonomous navigation using Pedro Pathing with vision-based
 * pose correction via Limelight AprilTag detection.</p>
 *
 * <p><b>Key Features:</b></p>
 * <ul>
 *   <li><b>Vision Localization:</b> Limelight vision system provides global pose corrections
 *       using AprilTags for accurate positioning throughout auto</li>
 *   <li><b>Sensor Fusion:</b> Extended Kalman Filter fuses swerve odometry, Pinpoint deadwheel
 *       odometry, and Limelight vision for optimal localization</li>
 *   <li><b>Path Following:</b> PedroPathing trajectory following with smooth acceleration
 *       and heading control</li>
 *   <li><b>Command-Based:</b> Uses command scheduler for sequenced auto routines</li>
 * </ul>
 *
 * <p><b>Field Coordinates (Red Alliance):</b></p>
 * <ul>
 *   <li><b>Start:</b> (9, 111, -90°) - Starting tile, facing backdrop</li>
 *   <li><b>Score:</b> (16, 128, -45°) - Scoring position at backdrop</li>
 *   <li><b>Pickup Zones:</b> Various positions for intake (30, 121, 0°), (30, 131, 0°), (45, 128, 90°)</li>
 *   <li><b>Park:</b> (68, 96, -90°) - Parking position</li>
 * </ul>
 *
 * <p><b>Coordinate System:</b> Inches, heading in radians (0 = facing red alliance wall,
 * positive = counterclockwise)</p>
 *
 * <p><b>Vision Integration:</b> The Drive subsystem's sensor fusion EKF automatically
 * applies vision pose corrections when AprilTags are visible. No manual vision code needed
 * in this OpMode - it's handled in {@link Drive#periodic()}.</p>
 *
 * @see com.seattlesolvers.solverslib.command.CommandOpMode
 * @see org.firstinspires.ftc.teamcode.commandbase.subsystems.Drive
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
     * Drive subsystem containing the PedroPathing follower and swerve drivetrain.
     * The follower is already initialized in Drive's constructor.
     */
    private Drive drive;

    // ==================== Autonomous Poses ====================

    /**
     * Starting pose for Red alliance.
     * Positioned on starting tile, facing the backdrop for scoring.
     */
    private final Pose startPose = new Pose(9, 111, Math.toRadians(-90));

    /**
     * Scoring pose at the backdrop.
     * Positioned to deposit game pieces into the scoring area.
     */
    private final Pose scorePose = new Pose(16, 128, Math.toRadians(-45));

    /**
     * First pickup pose.
     * Positioned to intake the first game piece from the field.
     */
    private final Pose pickup1Pose = new Pose(30, 121, Math.toRadians(0));

    /**
     * Second pickup pose.
     * Positioned to intake the second game piece from the field.
     */
    private final Pose pickup2Pose = new Pose(30, 131, Math.toRadians(0));

    /**
     * Third pickup pose.
     * Positioned to intake the third game piece from the field.
     */
    private final Pose pickup3Pose = new Pose(45, 128, Math.toRadians(90));

    /**
     * Parking pose.
     * Final position for end of autonomous period.
     */
    private final Pose parkPose = new Pose(68, 96, Math.toRadians(-90));

    // ==================== Path Chains ====================

    /**
     * Path from start to scoring position for the preload game piece.
     */
    private PathChain scorePreload;

    /**
     * Paths for first game piece pickup and score cycle.
     */
    private PathChain grabPickup1, scorePickup1;

    /**
     * Paths for second game piece pickup and score cycle.
     */
    private PathChain grabPickup2, scorePickup2;

    /**
     * Paths for third game piece pickup and score cycle.
     */
    private PathChain grabPickup3, scorePickup3;

    /**
     * Path to final parking position.
     */
    private PathChain park;

    // ==================== Path Building ====================

    /**
     * Builds all autonomous path chains using Pedro Pathing.
     *
     * <p>This method constructs PathChains for each segment of the autonomous routine.
     * Paths use Bezier curves for smooth motion and linear heading interpolation.</p>
     *
     * <p><b>Path Construction:</b></p>
     * <ul>
     *   <li>BezierLine: Straight line paths with heading interpolation</li>
     *   <li>BezierCurve: Curved paths with control points for smooth turns</li>
     * </ul>
     *
     * <p><b>Vision Integration:</b> The sensor fusion EKF in Drive subsystem will
     * automatically apply vision pose corrections during path following.</p>
     */
    private void buildPaths() {
        // Score preload: Start → Score position
        scorePreload = drive.follower.pathBuilder()
                .addPath(new BezierLine(startPose, scorePose))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                .build();

        // First pickup cycle
        grabPickup1 = drive.follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup1Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Pose.getHeading())
                .build();

        scorePickup1 = drive.follower.pathBuilder()
                .addPath(new BezierLine(pickup1Pose, scorePose))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), scorePose.getHeading())
                .build();

        // Second pickup cycle
        grabPickup2 = drive.follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup2Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup2Pose.getHeading())
                .build();

        scorePickup2 = drive.follower.pathBuilder()
                .addPath(new BezierLine(pickup2Pose, scorePose))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), scorePose.getHeading())
                .build();

        // Third pickup cycle
        grabPickup3 = drive.follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup3Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup3Pose.getHeading())
                .build();

        scorePickup3 = drive.follower.pathBuilder()
                .addPath(new BezierLine(pickup3Pose, scorePose))
                .setLinearHeadingInterpolation(pickup3Pose.getHeading(), scorePose.getHeading())
                .build();

        // Park: Score → Park position with curved path
        park = drive.follower.pathBuilder()
                .addPath(new BezierCurve(
                        scorePose,
                        new Pose(68, 110), // Control point for smooth curve
                        parkPose)
                )
                .setLinearHeadingInterpolation(scorePose.getHeading(), parkPose.getHeading())
                .build();
    }

    // ==================== Placeholder Commands ====================

    /**
     * Placeholder command for scoring a game piece.
     * TODO: Replace with your actual scoring mechanism command.
     *
     * @return command to score game piece
     */
    private com.seattlesolvers.solverslib.command.InstantCommand scoreCommand() {
        return new com.seattlesolvers.solverslib.command.InstantCommand(() -> {
            // TODO: Implement scoring mechanism
            // Example: robot.outtake.score();
            telemetryData.log().add("Scoring game piece");
        });
    }

    /**
     * Placeholder command for intaking a game piece.
     * TODO: Replace with your actual intake mechanism command.
     *
     * @return command to intake game piece
     */
    private com.seattlesolvers.solverslib.command.InstantCommand intakeCommand() {
        return new com.seattlesolvers.solverslib.command.InstantCommand(() -> {
            // TODO: Implement intake mechanism
            // Example: robot.intake.grab();
            telemetryData.log().add("Intaking game piece");
        });
    }

    /**
     * Placeholder command for initiating hang/ascent.
     * TODO: Replace with your actual hang mechanism command.
     *
     * @return command to start hang
     */
    private com.seattlesolvers.solverslib.command.InstantCommand hangCommand() {
        return new com.seattlesolvers.solverslib.command.InstantCommand(() -> {
            // TODO: Implement hang mechanism
            // Example: robot.hang.level1Ascent();
            telemetryData.log().add("Starting hang");
        });
    }

    // ==================== Initialization ====================

    /**
     * Initializes the robot for autonomous operation.
     *
     * <p>This method is called once when the OpMode is initialized. It performs the following:</p>
     * <ol>
     *   <li>Resets the command scheduler to clear any previous state</li>
     *   <li>Initializes all robot hardware via {@link Robot#init(HardwareMap)}</li>
     *   <li>Gets the Drive subsystem with integrated PedroPathing follower</li>
     *   <li>Sets the starting pose for the robot</li>
     *   <li>Builds all autonomous path chains</li>
     *   <li>Schedules the autonomous command sequence</li>
     * </ol>
     *
     * <p><b>Vision Localization:</b> The Drive subsystem's sensor fusion EKF automatically
     * applies vision pose corrections from Limelight when AprilTags are visible. This happens
     * in the background during path following - no additional code needed here.</p>
     *
     * <p><b>Autonomous Sequence:</b></p>
     * <ol>
     *   <li>Score preload game piece</li>
     *   <li>Cycle 1: Pickup and score first game piece</li>
     *   <li>Cycle 2: Pickup and score second game piece</li>
     *   <li>Cycle 3: Pickup and score third game piece</li>
     *   <li>Park in final position</li>
     * </ol>
     */
    @Override
    public void initialize() {
        super.reset();

        robot.init(hardwareMap);
        robot.telemetry = this.telemetryData;

        // Get the drive subsystem (PedroPathing follower is already initialized)
        drive = robot.drive;

        // Initialize drivetrain with starting pose
        drive.init();
        drive.follower.setStartingPose(startPose);

        // Build all autonomous paths
        buildPaths();

        // Schedule autonomous sequence
        schedule(
                new SequentialCommandGroup(
                        // Score preload
                        new FollowPathCommand(drive.follower, scorePreload),
                        scoreCommand(),
                        new WaitCommand(1000), // Wait 1 second for scoring

                        // First pickup cycle
                        new FollowPathCommand(drive.follower, grabPickup1).setGlobalMaxPower(0.5),
                        intakeCommand(),
                        new FollowPathCommand(drive.follower, scorePickup1),
                        scoreCommand(),

                        // Second pickup cycle
                        new FollowPathCommand(drive.follower, grabPickup2),
                        intakeCommand(),
                        new FollowPathCommand(drive.follower, scorePickup2, 1.0),
                        scoreCommand(),

                        // Third pickup cycle
                        new FollowPathCommand(drive.follower, grabPickup3),
                        intakeCommand(),
                        new FollowPathCommand(drive.follower, scorePickup3),
                        scoreCommand(),

                        // Park
                        new FollowPathCommand(drive.follower, park, false),
                        hangCommand()
                )
        );
    }

    /**
     * Initialization loop that runs repeatedly before the autonomous period starts.
     *
     * <p>This method runs continuously after {@link #initialize()} completes but before
     * the play button is pressed. It is used to:</p>
     * <ul>
     *   <li>Display initialization status and vision detection status to drivers</li>
     *   <li>Clear PhotonCore bulk caches to prepare for operation</li>
     *   <li>Allow vision system to detect AprilTags for pose initialization</li>
     * </ul>
     *
     * <p><b>Vision Preview:</b> Shows whether AprilTags are visible before starting,
     * allowing drivers to verify the vision system is working properly.</p>
     */
    @Override
    public void initialize_loop() {
        // Display vision status if available
        if (robot.vision != null) {
            boolean tagVisible = robot.vision.isTagVisible();
            telemetryData.addData("Status", "Auto Initialized. Waiting for Start.");
            telemetryData.addData("Vision Tag Visible", tagVisible ? "YES" : "NO");
        } else {
            telemetryData.addData("Status", "Auto Initialized. Waiting for Start.");
            telemetryData.addData("Vision", "Not Available");
        }

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
     *         <li>Subsystem periodic updates (includes PedroPathing follower update and sensor fusion)</li>
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
     * <p><b>Vision Localization:</b> The sensor fusion EKF automatically applies vision
     * pose corrections when AprilTags are visible. The Drive subsystem handles this in
     * its periodic() method - no additional code needed here.</p>
     *
     * @see Robot#updateLoop(TelemetryData)
     * @see com.seattlesolvers.solverslib.command.CommandScheduler#run()
     * @see com.pedropathing.follower.Follower#update()
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
