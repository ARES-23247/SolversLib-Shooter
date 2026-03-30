package org.firstinspires.ftc.teamcode.commandbase.subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.pedropathing.localization.Pose;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.globals.Robot;

public class Vision extends SubsystemBase {
    private final Robot robot = Robot.getInstance();

    public Vision() {
        // Initialization if needed
    }

    @Override
    public void periodic() {
        // Poll Limelight continuously via the Command Scheduler
        LLResult result = robot.limelight.getLatestResult();

        boolean isTagVisible = false;

        if (result != null && result.isValid()) {
            
            // Certainty Check: Only hard reset if we see a valid frame and tags
            // We use standard Fiducial checks. You can narrow this threshold (e.g. require 2+ tags for MT1)
            if (result.getFiducialResults() != null && result.getFiducialResults().size() >= 1) {
                isTagVisible = true;
                
                Pose3D botpose = result.getBotpose();
                if (botpose != null) {
                    
                    // The FTC SDK Pose3D from Limelight is in meters natively
                    double xInches = botpose.getPosition().x * 39.3701;
                    double yInches = botpose.getPosition().y * 39.3701;
                    double headingRadians = botpose.getOrientation().getYaw(AngleUnit.RADIANS);

                    Pose visionPose = new Pose(xInches, yInches, headingRadians);

                    // Hard Reset PedroPathing Localizer seamlessly
                    // Ensure the Drive subsystem is properly initialized first
                    if (robot.drive != null && robot.drive.follower != null) {
                        robot.drive.follower.setPose(visionPose);
                    }
                }
            }
        }

        // Broadcast directly to Driver Station Screen and Dashboard Text Log
        if (robot.telemetry != null) {
            robot.telemetry.addData("Vision Active Tag Spotted", isTagVisible);
        }
    }
}
