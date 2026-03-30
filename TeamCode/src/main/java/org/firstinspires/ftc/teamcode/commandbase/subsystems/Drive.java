package org.firstinspires.ftc.teamcode.commandbase.subsystems;

import com.seattlesolvers.solverslib.command.SubsystemBase;
import org.firstinspires.ftc.teamcode.globals.Robot;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.swerve.OctoSwerveDrivetrain;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.seattlesolvers.solverslib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.hardware.motors.CRServoEx;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.dashboard.canvas.Canvas;

import static org.firstinspires.ftc.teamcode.globals.Constants.*;

public class Drive extends SubsystemBase {
    private final Robot robot = Robot.getInstance();

    // The PedroPathing Path Planner
    public final Follower follower;

    // The Physical Swerve Drivetrain
    public final OctoSwerveDrivetrain swerve;

    public Drive() {
        // Initialize the Pedro Pathing Engine
        follower = Constants.createFollower(robot.hardwareMap);

        // Initialize the Custom Coaxial Swerve Engine with OctoQuad
        swerve = new OctoSwerveDrivetrain(
            TRACK_WIDTH,
            WHEEL_BASE,
            MAX_DRIVE_VELOCITY,
            SWERVE_SERVO_PIDF,
            SWERVE_DRIVE_PIDF,
            new MotorEx[] { robot.frontRightMotor, robot.frontLeftMotor, robot.backLeftMotor, robot.backRightMotor },
            new CRServoEx[] { robot.frontRightServo, robot.frontLeftServo, robot.backLeftServo, robot.backRightServo },
            robot.octoquad,
            new double[] { FRONT_RIGHT_OFFSET, FRONT_LEFT_OFFSET, BACK_LEFT_OFFSET, BACK_RIGHT_OFFSET }
        );
    }

    public void init() {
        // Required PedroPathing follower initialization steps
        // Assuming robot starts at 0, 0, 0
        follower.setStartingPose(new Pose(0, 0, 0));
    }

    /**
     * Steer the Swerve Drive manually during TeleOp by passing velocities straight into Kinematics
     */
    public void setTeleOpDrive(double forward, double lateral, double turn, boolean isFieldCentric) {
        if (isFieldCentric) {
            // Retrieve current heading from PedroPathing's Localizer
            double currentHeading = follower.getPoseTracker().getTotalHeading();
            
            // Apply field centric rotation
            double rotX = forward * Math.cos(-currentHeading) - lateral * Math.sin(-currentHeading);
            double rotY = forward * Math.sin(-currentHeading) + lateral * Math.cos(-currentHeading);
            
            // Pass the generated chassis speeds directly to the physical Swerve Solver
            swerve.drive(new ChassisSpeeds(rotX, rotY, turn));
        } else {
            // Robot Centric
            swerve.drive(new ChassisSpeeds(forward, lateral, turn));
        }
    }

    @Override
    public void periodic() {
        // Update PedroPathing Odometry internally
        follower.update();

        // If performing autonomous moves (i.e. follower is busy building paths), extract Pedro's desired velocities
        // and inject them straight into our physical Swerve modules!
        if (follower.isBusy()) {
            Pose drivePower = follower.getTotalPower();
            // Pedro Pose power: x component, y component, heading component
            swerve.drive(new ChassisSpeeds(drivePower.getX(), drivePower.getY(), drivePower.getHeading()));
        }

        // Dashboard Telemetry & Field Drawing
        if (robot.telemetry != null) {
            TelemetryPacket packet = new TelemetryPacket();
            Canvas field = packet.fieldOverlay();
            
            // Get Current Pedro Pathing Robot Location
            Pose currentPose = follower.getPoseTracker().getPose();
            
            // Draw Robot Box (18x18 assumed default)
            field.setStroke("#3F51B5");
            field.strokeRect(currentPose.getX() - 9, currentPose.getY() - 9, 18, 18);
            
            // Draw Heading Line
            field.strokeLine(
                currentPose.getX(), currentPose.getY(),
                currentPose.getX() + 9 * Math.cos(currentPose.getHeading()), 
                currentPose.getY() + 9 * Math.sin(currentPose.getHeading())
            );

            // Fetch Swerve Kinematics Target/Actual Logs
            swerve.drawTelemetry(packet, currentPose);

            // Send packet immediately to Dashboard HTML websocket
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
        }

        // CSV Logging
        if (robot.logger != null) {
            Pose currentPose = follower.getPoseTracker().getPose();
            robot.logger.addData("Robot X (Inches)", currentPose.getX());
            robot.logger.addData("Robot Y (Inches)", currentPose.getY());
            robot.logger.addData("Robot Heading (Rad)", currentPose.getHeading());
            
            swerve.logData(robot.logger);
        }
    }
}
