package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.localization.Pose;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.globals.Robot;

/**
 * PedroPathing configuration constants.
 *
 * <p>This class configures the PedroPathing autonomous navigation system.</p>
 *
 * <h3>IMU Configuration:</h3>
 * <p>REV Hub IMU is DISABLED to reduce I2C bus traffic and improve loop times.</p>
 * <ul>
 *   <li><b>Disabled:</b> REV Hub IMU (Control Hub/Expansion Hub built-in IMU)</li>
 *   <li><b>Enabled:</b> GoBilda Pinpoint IMU (used for heading tracking)</li>
 *   <li><b>Optional:</b> OctoQuad IMU (can be enabled in Constants.LOCALIZER_ENABLED)</li>
 * </ul>
 *
 * <p><b>Performance Benefit:</b> Disabling the REV Hub IMU reduces I2C overhead
 * since we're already using the GoBilda Pinpoint IMU for heading.</p>
 */
public class Constants {

    /**
     * PedroPathing follower constants.
     *
     * <p>IMU is disabled to reduce I2C bus traffic. We use GoBilda Pinpoint for heading instead.</p>
     */
    public static FollowerConstants followerConstants = new FollowerConstants(
            0,                    // Primary PIDF P gain
            0,                    // Primary PIDF I gain
            0,                    // Primary PIDF D gain
            0,                    // Primary PIDF F gain (feedforward)
            0,                    // Secondary PIDF P gain
            0,                    // Secondary PIDF I gain
            0,                    // Secondary PIDF D gain
            0,                    // Secondary PIDF F gain
            0.5,                  // Heading PIDF P gain
            0,                    // Heading PIDF I gain
            0.001,                // Heading PIDF D gain
            0,                    // Heading PIDF F gain
            0.595,                // Drive radius (inches)
            0.595,                // Turn radius (inches)
            1.0,                  // Max wheel velocity (normalized)
            1.0,                  // Min profile acceleration
            1.0,                  // Max profile acceleration
            0.75,                 // Profile time gain
            0,                    // Zero power acceleration (in/s^2)
            0                     // Zero power deceleration (in/s^2)
    );

    /**
     * Path constraints for autonomous movement.
     */
    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    /**
     * Creates a PedroPathing Follower instance with REV Hub IMU disabled.
     *
     * <p><b>Important:</b> The REV Hub IMU is not used. Heading is provided by
     * GoBilda Pinpoint odometry sensor instead.</p>
     *
     * @param hardwareMap the hardware map (IMU will NOT be accessed from here)
     * @return configured Follower instance
     */
    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .build();
    }
}
