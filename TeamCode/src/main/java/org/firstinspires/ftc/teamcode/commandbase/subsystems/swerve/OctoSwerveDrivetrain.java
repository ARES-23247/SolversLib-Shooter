package org.firstinspires.ftc.teamcode.commandbase.subsystems.swerve;

import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.hardware.digitalinc.OctoQuad;
import com.seattlesolvers.solverslib.hardware.motors.CRServoEx;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.seattlesolvers.solverslib.geometry.Vector2d;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.pedropathing.localization.Pose;
import org.firstinspires.ftc.teamcode.util.DataLogger;

public class OctoSwerveDrivetrain {
    private final OctoSwerveModule[] modules = new OctoSwerveModule[4];
    private final double maxSpeed;
    private final double maxAngularSpeed;

    public OctoSwerveDrivetrain(double trackWidth, double wheelBase, double maxSpeed, PIDFCoefficients swervoPIDFCoefficients, PIDFCoefficients drivePIDFCoefficients, MotorEx[] motors, CRServoEx[] swervos, OctoQuad octoQuad, double[] moduleOffsets) {
        if (motors.length != 4 || swervos.length != 4 || moduleOffsets.length != 4) {
            throw new IllegalArgumentException("Hardware lists for swerve modules must have exactly 4 objects each");
        }

        this.maxSpeed = maxSpeed;
        this.maxAngularSpeed = maxSpeed / Math.hypot(trackWidth / 2, wheelBase / 2);

        // Indices typically align: 0=FR, 1=FL, 2=BL, 3=BR
        // We assume ports 0-3 are for steering servos, and 4-7 are for drive wheels.
        this.modules[0] = new OctoSwerveModule(motors[0], swervos[0], octoQuad, 0, 4, new Vector2d(trackWidth / 2, wheelBase / 2), maxSpeed, swervoPIDFCoefficients, drivePIDFCoefficients);
        this.modules[1] = new OctoSwerveModule(motors[1], swervos[1], octoQuad, 1, 5, new Vector2d(trackWidth / 2, -wheelBase / 2), maxSpeed, swervoPIDFCoefficients, drivePIDFCoefficients);
        this.modules[2] = new OctoSwerveModule(motors[2], swervos[2], octoQuad, 2, 6, new Vector2d(-trackWidth / 2, -wheelBase / 2), maxSpeed, swervoPIDFCoefficients, drivePIDFCoefficients);
        this.modules[3] = new OctoSwerveModule(motors[3], swervos[3], octoQuad, 3, 7, new Vector2d(-trackWidth / 2, wheelBase / 2), maxSpeed, swervoPIDFCoefficients, drivePIDFCoefficients);
        this.modules[0].setOffset(moduleOffsets[0]);
        this.modules[1].setOffset(moduleOffsets[1]);
        this.modules[2].setOffset(moduleOffsets[2]);
        this.modules[3].setOffset(moduleOffsets[3]);
    }

    public void drive(ChassisSpeeds speeds) {
        Vector2d[] targetVectors = new Vector2d[4];
        double maxVelocity = 0;

        for (int i = 0; i < 4; i++) {
            targetVectors[i] = modules[i].calculateVectorRobotCentric(speeds);
            maxVelocity = Math.max(maxVelocity, targetVectors[i].magnitude());
        }

        // Desaturate Wheel Speeds
        if (maxVelocity > maxSpeed) {
            for (Vector2d targetVector : targetVectors) {
                targetVector.scale(maxSpeed / maxVelocity);
            }
        }

        for (int i = 0; i < 4; i++) {
            modules[i].updateModuleWithVelocity(targetVectors[i]);
        }
    }

    public void drawTelemetry(TelemetryPacket packet, Pose robotPose) {
        String[] moduleNames = {"FR", "FL", "BL", "BR"};
        Canvas field = packet.fieldOverlay();
        
        for (int i = 0; i < 4; i++) {
            OctoSwerveModule module = modules[i];
            
            packet.put(moduleNames[i] + " Target Angle (Rad)", module.getTargetAngleRadians());
            packet.put(moduleNames[i] + " Actual Angle (Rad)", module.getModuleHeadingRadians());
            packet.put(moduleNames[i] + " Target Vel (In/s)", module.getTargetMagnitude());
            packet.put(moduleNames[i] + " Actual Vel (In/s)", module.getCurrentVelocityInchesPerSec());
            
            // Draw visual module orientation lines!
            if (robotPose != null) {
                // Determine module absolute field position based on the robot's heading
                double locX = module.getLocation().getX();
                double locY = module.getLocation().getY();
                double heading = robotPose.getHeading();
                
                double globalX = robotPose.getX() + locX * Math.cos(heading) - locY * Math.sin(heading);
                double globalY = robotPose.getY() + locX * Math.sin(heading) + locY * Math.cos(heading);
                
                // Determine absolute field direction the wheel is pointing
                double wheelHeading = heading + module.getModuleHeadingRadians();
                
                // Draw a small 2-inch indicator line showing the direction the wheel is actively facing
                field.setStroke("#FF5722"); // Orange
                field.strokeLine(
                    globalX, globalY,
                    globalX + 2.5 * Math.cos(wheelHeading), 
                    globalY + 2.5 * Math.sin(wheelHeading)
                );
                field.fillCircle(globalX, globalY, 0.5); // Hub center
            }
        }
    }

    public void logData(DataLogger logger) {
        if (logger == null) return;
        String[] moduleNames = {"FR", "FL", "BL", "BR"};
        for (int i = 0; i < 4; i++) {
            OctoSwerveModule module = modules[i];
            logger.addData(moduleNames[i] + " Target Angle", module.getTargetAngleRadians());
            logger.addData(moduleNames[i] + " Actual Angle", module.getModuleHeadingRadians());
            logger.addData(moduleNames[i] + " Target Vel", module.getTargetMagnitude());
            logger.addData(moduleNames[i] + " Actual Vel", module.getCurrentVelocityInchesPerSec());
        }
    }

    public void stop() {
        for (OctoSwerveModule module : modules) {
            module.stop();
        }
    }
}
