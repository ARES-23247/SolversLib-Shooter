package org.firstinspires.ftc.teamcode.util.localization;

import com.seattlesolvers.solverslib.geometry.Vector2d;
import com.seattlesolvers.solverslib.kinematics.wpilibkinematics.ChassisSpeeds;

/**
 * Odometry tracker for swerve drive using drive wheel encoders.
 *
 * <p>This class tracks robot position and heading by integrating velocity measurements
 * from the swerve drive modules. Unlike deadwheel odometry, this uses the actual
 * drive wheels for localization.</p>
 *
 * <h3>How It Works:</h3>
 * <p>At each time step, this class:</p>
 * <ol>
 *   <li>Takes chassis speeds (forward, lateral, rotational velocities)</li>
 *   <li>Integrates velocities to compute position delta</li>
 *   <li>Rotates the velocity vector by the robot's heading</li>
 *   <li>Updates the pose estimate</li>
 * </ol>
 *
 * <h3>Advantages:</h3>
 * <ul>
 *   <li><b>High Update Rate:</b> Updates at control loop frequency (50Hz+)</li>
 *   <li><b>No Extra Hardware:</b> Uses existing drive wheel encoders</li>
 *   <li><b>Seamless Integration:</b> Works with swerve drive kinematics</li>
 * </ul>
 *
 * <h3>Limitations:</h3>
 * <ul>
 *   <li><b>Wheel Slip:</b> Slip causes drift (less accurate than deadwheels)</li>
 *   <li><b>Integration Error:</b> Errors accumulate over time</li>
 *   <li><b>Requires Correction:</b> Needs vision/deadwheel correction for accuracy</li>
 * </ul>
 *
 * <h3>Usage in Sensor Fusion:</h3>
 * <p>In a sensor fusion system, swerve odometry provides:</p>
 * <ul>
 *   <li><b>High-frequency updates:</b> Keeps pose estimate fresh between corrections</li>
 *   <li><b>Velocity estimates:</b> Useful for Kalman filter prediction step</li>
 *   <li><b>Heading integration:</b> Complements IMU heading</li>
 * </ul>
 *
 * <h3>Coordinate System:</h3>
 * <ul>
 *   <li><b>X:</b> Forward position (inches)</li>
 *   <li><b>Y:</b> Lateral position (inches)</li>
 *   <li><b>Heading:</b> Rotation angle (radians, 0 = forward)</li>
 * </ul>
 *
 * @see org.firstinspires.ftc.teamcode.util.localization.SensorFusionLocalizer
 * @see ChassisSpeeds
 */
public class SwerveOdometry {

    /**
     * Current X position estimate (inches).
     */
    private double x;

    /**
     * Current Y position estimate (inches).
     */
    private double y;

    /**
     * Current heading estimate (radians).
     */
    private double heading;

    /**
     * Previous timestamp for computing time deltas (nanoseconds).
     */
    private long prevTimeNanos;

    /**
     * Creates a new swerve odometry tracker with initial pose.
     *
     * @param initialX initial X position (inches)
     * @param initialY initial Y position (inches)
     * @param initialHeading initial heading (radians)
     */
    public SwerveOdometry(double initialX, double initialY, double initialHeading) {
        this.x = initialX;
        this.y = initialY;
        this.heading = initialHeading;
        this.prevTimeNanos = System.nanoTime();
    }

    /**
     * Creates a new swerve odometry tracker at origin.
     */
    public SwerveOdometry() {
        this(0, 0, 0);
    }

    /**
     * Updates the odometry estimate with new chassis speeds.
     *
     * <p>This method integrates the chassis speeds to update the pose estimate.
     * The integration process:</p>
     *
     * <h3>1. Compute Time Delta:</h3>
     * <pre>dt = (currentTime - prevTime) / 1e9  (convert nanoseconds to seconds)</pre>
     *
     * <h3>2. Compute Position Delta in Robot Frame:</h3>
     * <pre>dxRobot = vx * dt
     * dyRobot = vy * dt
     * dHeading = omega * dt</pre>
     *
     * <h3>3. Rotate to Field Frame:</h3>
     * <pre>dxField = dxRobot * cos(heading) - dyRobot * sin(heading)
     * dyField = dxRobot * sin(heading) + dyRobot * cos(heading)</pre>
     *
     * <h3>4. Update Pose:</h3>
     * <pre>x += dxField
     * y += dyField
     * heading += dHeading</pre>
     *
     * @param speeds the chassis velocities (vx, vy, omega) in inches/second and radians/second
     */
    public void update(ChassisSpeeds speeds) {
        long currentTimeNanos = System.nanoTime();
        double dt = (currentTimeNanos - prevTimeNanos) / 1e9;  // Convert to seconds
        prevTimeNanos = currentTimeNanos;

        // Integrate position
        double dxRobot = speeds.vxMetersPerSecond * dt;
        double dyRobot = speeds.vyMetersPerSecond * dt;
        double dHeading = speeds.omegaRadiansPerSecond * dt;

        // Rotate position delta by current heading to transform from robot frame to field frame
        double cosHeading = Math.cos(heading);
        double sinHeading = Math.sin(heading);

        double dxField = dxRobot * cosHeading - dyRobot * sinHeading;
        double dyField = dxRobot * sinHeading + dyRobot * cosHeading;

        // Update pose
        x += dxField;
        y += dyField;
        heading += dHeading;

        // Normalize heading to [-π, π]
        while (heading > Math.PI) heading -= 2 * Math.PI;
        while (heading < -Math.PI) heading += 2 * Math.PI;
    }

    /**
     * Resets the odometry to a specific pose.
     *
     * @param x new X position (inches)
     * @param y new Y position (inches)
     * @param heading new heading (radians)
     */
    public void resetPose(double x, double y, double heading) {
        this.x = x;
        this.y = y;
        this.heading = heading;
        this.prevTimeNanos = System.nanoTime();
    }

    /**
     * Gets the current X position.
     *
     * @return X position in inches
     */
    public double getX() {
        return x;
    }

    /**
     * Gets the current Y position.
     *
     * @return Y position in inches
     */
    public double getY() {
        return y;
    }

    /**
     * Gets the current heading.
     *
     * @return heading in radians
     */
    public double getHeading() {
        return heading;
    }

    /**
     * Gets the current pose as a Vector2d (position only).
     *
     * @return position vector (x, y) in inches
     */
    public Vector2d getPosition() {
        return new Vector2d(x, y);
    }

    /**
     * Gets the current time since last update in seconds.
     *
     * @return time delta in seconds
     */
    public double getLastUpdateTime() {
        return (System.nanoTime() - prevTimeNanos) / 1e9;
    }
}
