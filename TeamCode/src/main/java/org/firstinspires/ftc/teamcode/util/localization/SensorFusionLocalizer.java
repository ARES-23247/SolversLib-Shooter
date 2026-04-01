package org.firstinspires.ftc.teamcode.util.localization;

import com.seattlesolvers.solverslib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.hardware.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.hardware.SRSHub;

/**
 * Extended Kalman Filter (EKF) for fusing multiple localization sources.
 *
 * <p>This class implements sensor fusion to combine three localization sources:</p>
 * <ol>
 *   <li><b>Swerve Odometry:</b> High-frequency updates from drive wheel encoders</li>
 *   <li><b>Pinpoint Odometry:</b> High-accuracy translation from deadwheel encoders</li>
 *   <li><b>Limelight Vision:</b> Global position correction from AprilTags</li>
 * </ol>
 *
 * <h3>How the EKF Works:</h3>
 * <p>The Extended Kalman Filter maintains a state estimate with uncertainty:</p>
 *
 * <h4>State Vector:</h4>
 * <pre>x = [x, y, heading, vx, vy, omega]^T</pre>
 * <ul>
 *   <li><b>x, y:</b> Position (inches)</li>
 *   <li><b>heading:</b> Orientation (radians)</li>
 *   <li><b>vx, vy:</b> Velocity (inches/sec)</li>
 *   <li><b>omega:</b> Angular velocity (radians/sec)</li>
 * </ul>
 *
 * <h4>EKF Cycle:</h4>
 * <ol>
 *   <li><b>Predict:</b> Use swerve odometry to predict next state (high frequency)</li>
 *   <li><b>Correct (Pinpoint):</b> Correct pose estimate with deadwheel measurements</li>
 *   <li><b>Correct (Limelight):</b> Correct pose estimate with vision measurements</li>
 * </ol>
 *
 * <h3>Sensor Characteristics:</h3>
 * <table border="1">
 *   <tr><th>Sensor</th><th>Update Rate</th><th>Accuracy</th><th>Uncertainty</th></tr>
 *   <tr><td>Swerve Odometry</td><td>50 Hz</td><td>Low (slip)</td><td>High</td></tr>
 *   <tr><td>Pinpoint</td><td>50 Hz</td><td>High (deadwheels)</td><td>Low</td></tr>
 *   <tr><td>Limelight</td><td>10-30 Hz</td><td>Very High (global)</td><td>Very Low</td></tr>
 * </table>
 *
 * <h3>Tuning Parameters:</h3>
 * <p>The filter behavior is controlled by noise covariance matrices:</p>
 * <ul>
 *   <li><b>Process Noise (Q):</b> How much the system changes unpredictably</li>
 *   <li><b>Swerve Measurement Noise (R_swerve):</b> Uncertainty in swerve odometry</li>
 *   <li><b>Pinpoint Measurement Noise (R_pinpoint):</b> Uncertainty in deadwheel odometry</li>
 *   <li><b>Vision Measurement Noise (R_vision):</b> Uncertainty in vision measurements</li>
 * </ul>
 *
 * <h3>Usage Example:</h3>
 * <pre>{@code
 * // Create filter
 * SensorFusionLocalizer fusion = new SensorFusionLocalizer();
 *
 * // In control loop (every 20ms):
 * // 1. Predict with swerve odometry
 * fusion.predictWithSwerve(chassisSpeeds);
 *
 * // 2. Correct with Pinpoint (if available)
 * fusion.correctWithPinpoint(pinpoint.getX(), pinpoint.getY(), pinpoint.getHeading());
 *
 * // 3. Correct with Limelight (if tag visible)
 * if (limelightHasTag) {
 *     fusion.correctWithVision(visionX, visionY, visionHeading);
 * }
 *
 * // Get fused pose estimate
 * Pose estimatedPose = fusion.getEstimatedPose();
 * }}</pre>
 *
 * @see org.firstinspires.ftc.teamcode.util.localization.SwerveOdometry
 * @see GoBildaPinpointDriver
 */
public class SensorFusionLocalizer {

    // ===== State Vector =====
    /** State vector: [x, y, heading, vx, vy, omega] */
    private double[] x = new double[6];

    /** State covariance matrix (6x6) - tracks uncertainty in state estimate */
    private double[][] P = new double[6][6];

    // ===== EKF Matrices =====
    /** State transition matrix (6x6) - predicts next state from current state */
    private double[][] F = new double[6][6];

    /** Process noise covariance matrix (6x6) - models system uncertainty */
    private double[][] Q = new double[6][6];

    /** Measurement matrix for pose measurements (3x6) - maps state to measurement space */
    private double[][] H_pose = new double[3][6];

    /** Measurement noise covariance for swerve odometry (3x3) */
    private double[][] R_swerve = new double[3][3];

    /** Measurement noise covariance for Pinpoint (3x3) */
    private double[][] R_pinpoint = new double[3][3];

    /** Measurement noise covariance for vision (3x3) */
    private double[][] R_vision = new double[3][3];

    // ===== Timing =====
    /** Previous timestamp for computing time deltas (nanoseconds) */
    private long prevTimeNanos;

    // ===== Constants =====
    private static final double DEG_TO_RAD = Math.PI / 180.0;

    /**
     * Creates a new sensor fusion EKF with default tuning.
     *
     * <p>Default noise parameters are set for typical FTC robot configurations.
     * Adjust these in Constants.java for your specific setup.</p>
     */
    public SensorFusionLocalizer() {
        // Initialize state to origin
        resetPose(0, 0, 0);

        // Initialize covariance to high uncertainty (will converge as measurements arrive)
        for (int i = 0; i < 6; i++) {
            P[i][i] = 1000.0;  // High initial uncertainty
        }

        // Initialize measurement matrix (we measure x, y, heading directly)
        H_pose[0][0] = 1.0;  // Measure x
        H_pose[1][1] = 1.0;  // Measure y
        H_pose[2][2] = 1.0;  // Measure heading

        prevTimeNanos = System.nanoTime();
    }

    /**
     * Resets the filter to a specific pose.
     *
     * @param x initial X position (inches)
     * @param y initial Y position (inches)
     * @param heading initial heading (radians)
     */
    public void resetPose(double x, double y, double heading) {
        this.x[0] = x;        // Position X
        this.x[1] = y;        // Position Y
        this.x[2] = heading;  // Heading
        this.x[3] = 0;        // Velocity X
        this.x[4] = 0;        // Velocity Y
        this.x[5] = 0;        // Angular velocity

        // Reset covariance to high uncertainty
        for (int i = 0; i < 6; i++) {
            for (int j = 0; j < 6; j++) {
                P[i][j] = (i == j) ? 1000.0 : 0.0;
            }
        }

        prevTimeNanos = System.nanoTime();
    }

    /**
     * Sets the noise parameters for the EKF.
     *
     * <p>These parameters control how much the filter trusts each sensor:</p>
     * <ul>
     *   <li><b>Lower noise = more trust</b> in that sensor</li>
     *   <li><b>Higher noise = less trust</b> in that sensor</li>
 * </ul>
     *
     * @param processNoise process noise standard deviation [pos, heading, vel]
     * @param swerveNoise swerve odometry noise standard deviation [pos, heading]
     * @param pinpointNoise Pinpoint noise standard deviation [pos, heading]
     * @param visionNoise vision noise standard deviation [pos, heading]
     */
    public void setNoiseParameters(double[] processNoise, double[] swerveNoise,
                                   double[] pinpointNoise, double[] visionNoise) {
        // Process noise covariance Q
        Q[0][0] = processNoise[0] * processNoise[0];  // Position X
        Q[1][1] = processNoise[0] * processNoise[0];  // Position Y
        Q[2][2] = processNoise[1] * processNoise[1];  // Heading
        Q[3][3] = processNoise[2] * processNoise[2];  // Velocity X
        Q[4][4] = processNoise[2] * processNoise[2];  // Velocity Y
        Q[5][5] = processNoise[2] * processNoise[2];  // Angular velocity

        // Swerve measurement noise R_swerve
        R_swerve[0][0] = swerveNoise[0] * swerveNoise[0];  // Position X
        R_swerve[1][1] = swerveNoise[0] * swerveNoise[0];  // Position Y
        R_swerve[2][2] = swerveNoise[1] * swerveNoise[1];  // Heading

        // Pinpoint measurement noise R_pinpoint
        R_pinpoint[0][0] = pinpointNoise[0] * pinpointNoise[0];  // Position X
        R_pinpoint[1][1] = pinpointNoise[0] * pinpointNoise[0];  // Position Y
        R_pinpoint[2][2] = pinpointNoise[1] * pinpointNoise[1];  // Heading

        // Vision measurement noise R_vision
        R_vision[0][0] = visionNoise[0] * visionNoise[0];  // Position X
        R_vision[1][1] = visionNoise[0] * visionNoise[0];  // Position Y
        R_vision[2][2] = visionNoise[1] * visionNoise[1];  // Heading
    }

    /**
     * Prediction step using swerve odometry control input.
     *
     * <p>This method predicts the next state based on the chassis speeds (control input).
     * It updates the state transition matrix F and propagates the state and covariance.</p>
     *
     * <h3>Prediction Equations:</h3>
     * <pre>x̂(k|k-1) = F * x̂(k-1|k-1) + B * u
     * P(k|k-1) = F * P(k-1|k-1) * F^T + Q</pre>
     *
     * @param speeds the chassis velocities (control input)
     */
    public void predictWithSwerve(ChassisSpeeds speeds) {
        long currentTimeNanos = System.nanoTime();
        double dt = (currentTimeNanos - prevTimeNanos) / 1e9;
        prevTimeNanos = currentTimeNanos;

        if (dt <= 0 || dt > 1.0) {
            return;  // Invalid time delta
        }

        // Update state transition matrix F
        // x_new = x_old + vx * dt
        // y_new = y_old + vy * dt
        // heading_new = heading_old + omega * dt
        // vx, vy, omega stay roughly constant (assuming low acceleration)
        F[0][0] = 1.0;
        F[0][3] = dt;
        F[1][1] = 1.0;
        F[1][4] = dt;
        F[2][2] = 1.0;
        F[2][5] = dt;
        F[3][3] = 1.0;
        F[4][4] = 1.0;
        F[5][5] = 1.0;

        // Predict state: x = F * x
        double[] x_pred = new double[6];
        for (int i = 0; i < 6; i++) {
            for (int j = 0; j < 6; j++) {
                x_pred[i] += F[i][j] * x[j];
            }
        }

        // Predict covariance: P = F * P * F^T + Q
        double[][] P_pred = matrixAdd(matrixMultiply(matrixMultiply(F, P), transpose(F)), Q);

        // Update state and covariance
        x = x_pred;
        P = P_pred;
    }

    /**
     * Correction step using Pinpoint odometry measurements.
     *
     * <p>This method corrects the state estimate using high-accuracy deadwheel measurements.</p>
     *
     * @param x measured X position (inches)
     * @param y measured Y position (inches)
     * @param heading measured heading (radians)
     */
    public void correctWithPinpoint(double x, double y, double heading) {
        correctWithMeasurement(new double[]{x, y, heading}, R_pinpoint);
    }

    /**
     * Correction step using vision measurements.
     *
     * <p>This method corrects the state estimate using global position measurements from AprilTags.</p>
     *
     * @param x measured X position (inches)
     * @param y measured Y position (inches)
     * @param heading measured heading (radians)
     */
    public void correctWithVision(double x, double y, double heading) {
        correctWithMeasurement(new double[]{x, y, heading}, R_vision);
    }

    /**
     * Corrects the state estimate with vision measurement using custom noise parameters.
     *
     * <p>This overload allows specifying custom measurement noise for cases where
     * vision quality is degraded (e.g., floating robot detection, poor camera angle).</p>
     *
     * @param x measured X position (inches)
     * @param y measured Y position (inches)
     * @param heading measured heading (radians)
     * @param positionNoise position measurement noise std dev (inches)
     * @param headingNoise heading measurement noise std dev (radians)
     */
    public void correctWithVision(double x, double y, double heading,
                                  double positionNoise, double headingNoise) {
        // Build custom noise covariance matrix
        double[][] R_custom = {
            {positionNoise * positionNoise, 0, 0},
            {0, positionNoise * positionNoise, 0},
            {0, 0, headingNoise * headingNoise}
        };

        correctWithMeasurement(new double[]{x, y, heading}, R_custom);
    }

    /**
     * Correction step using OctoQuad IMU heading measurement (backup mode).
     *
     * <p>This method corrects ONLY the heading state estimate using OctoQuad IMU.
     * This is used as a fallback when Pinpoint IMU is unavailable or unhealthy.</p>
     *
     * <p><b>When to use:</b> Only use this correction when Pinpoint has failed.
     * OctoQuad IMU is less accurate than Pinpoint, so it should only be used as backup.</p>
     *
     * @param heading measured heading from OctoQuad IMU (radians)
     * @param headingNoise heading measurement noise std dev (radians)
     */
    public void correctWithOctoQuadIMU(double heading, double headingNoise) {
        // Build measurement matrix for heading-only correction
        // We only measure heading, not position
        double[][] H_heading = {
            {0, 0, 0, 0, 0, 0},  // Don't measure x
            {0, 0, 0, 0, 0, 0},  // Don't measure y
            {0, 0, 1, 0, 0, 0}   // Measure heading
        };

        // Build noise covariance for heading-only measurement
        double[][] R_octoquad = {
            {1000.0 * 1000.0, 0, 0},           // Huge noise for x (don't use)
            {0, 1000.0 * 1000.0, 0},           // Huge noise for y (don't use)
            {0, 0, headingNoise * headingNoise}  // Actual noise for heading
        };

        // Compute innovation: y = z - H * x
        double[] y = {
            0,  // Don't correct x
            0,  // Don't correct y
            heading - x[2]  // Heading innovation
        };

        // Normalize heading innovation to [-π, π]
        while (y[2] > Math.PI) y[2] -= 2 * Math.PI;
        while (y[2] < -Math.PI) y[2] += 2 * Math.PI;

        // Compute innovation covariance: S = H * P * H^T + R
        double[][] S = matrixAdd(
            matrixMultiply(matrixMultiply(H_heading, P), transpose(H_heading)),
            R_octoquad
        );

        // Compute Kalman gain: K = P * H^T * S^-1
        double[][] K = matrixMultiply(
            matrixMultiply(P, transpose(H_heading)),
            inverse(S)
        );

        // Update state: x = x + K * y
        double[] x_new = x.clone();
        for (int i = 0; i < 6; i++) {
            for (int j = 0; j < 3; j++) {
                x_new[i] += K[i][j] * y[j];
            }
        }

        // Update covariance: P = (I - K * H) * P
        double[][] KH = matrixMultiply(K, H_heading);
        double[][] I_KH = subtractFromIdentity(KH);
        double[][] P_new = matrixMultiply(I_KH, P);

        // Normalize heading
        while (x_new[2] > Math.PI) x_new[2] -= 2 * Math.PI;
        while (x_new[2] < -Math.PI) x_new[2] += 2 * Math.PI;

        x = x_new;
        P = P_new;
    }

    /**
     * Generic correction step using a pose measurement.
     *
     * <h3>Correction Equations:</h3>
     * <pre>y = z - H * x̂  (innovation)
     * S = H * P * H^T + R  (innovation covariance)
     * K = P * H^T * S^-1  (Kalman gain)
     * x̂ = x̂ + K * y  (updated state)
     * P = (I - K * H) * P  (updated covariance)</pre>
     *
     * @param z measurement vector [x, y, heading]
     * @param R measurement noise covariance
     */
    private void correctWithMeasurement(double[] z, double[][] R) {
        // Compute innovation: y = z - H * x
        double[] y = new double[3];
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 6; j++) {
                y[i] += H_pose[i][j] * x[j];
            }
            y[i] = z[i] - y[i];  // Innovation
        }

        // Normalize heading innovation to [-π, π]
        while (y[2] > Math.PI) y[2] -= 2 * Math.PI;
        while (y[2] < -Math.PI) y[2] += 2 * Math.PI;

        // Compute innovation covariance: S = H * P * H^T + R
        double[][] S = matrixAdd(matrixMultiply(matrixMultiply(H_pose, P), transpose(H_pose)), R);

        // Compute Kalman gain: K = P * H^T * S^-1
        double[][] K = matrixMultiply(matrixMultiply(P, transpose(H_pose)), inverse(S));

        // Update state: x = x + K * y
        double[] x_new = x.clone();
        for (int i = 0; i < 6; i++) {
            for (int j = 0; j < 3; j++) {
                x_new[i] += K[i][j] * y[j];
            }
        }

        // Update covariance: P = (I - K * H) * P
        double[][] KH = matrixMultiply(K, H_pose);
        double[][] I_KH = subtractFromIdentity(KH);
        double[][] P_new = matrixMultiply(I_KH, P);

        // Normalize heading
        while (x_new[2] > Math.PI) x_new[2] -= 2 * Math.PI;
        while (x_new[2] < -Math.PI) x_new[2] += 2 * Math.PI;

        x = x_new;
        P = P_new;
    }

    /**
     * Gets the current estimated pose.
     *
     * @return estimated pose [x, y, heading]
     */
    public Pose getEstimatedPose() {
        return new Pose(x[0], x[1], x[2]);
    }

    /**
     * Gets the current estimated velocity.
     *
     * @return estimated velocity [vx, vy, omega]
     */
    public ChassisSpeeds getEstimatedVelocity() {
        return new ChassisSpeeds(x[3], x[4], x[5]);
    }

    /**
     * Gets the position uncertainty (standard deviation).
     *
     * @return position uncertainty [x_sigma, y_sigma] in inches
     */
    public double[] getPositionUncertainty() {
        return new double[]{
            Math.sqrt(P[0][0]),
            Math.sqrt(P[1][1])
        };
    }

    /**
     * Gets the heading uncertainty (standard deviation).
     *
     * @return heading uncertainty in radians
     */
    public double getHeadingUncertainty() {
        return Math.sqrt(P[2][2]);
    }

    // ===== Matrix Utility Functions =====

    private double[][] transpose(double[][] m) {
        double[][] result = new double[m[0].length][m.length];
        for (int i = 0; i < m.length; i++) {
            for (int j = 0; j < m[0].length; j++) {
                result[j][i] = m[i][j];
            }
        }
        return result;
    }

    private double[][] matrixMultiply(double[][] A, double[][] B) {
        int rowsA = A.length;
        int colsA = A[0].length;
        int colsB = B[0].length;
        double[][] result = new double[rowsA][colsB];
        for (int i = 0; i < rowsA; i++) {
            for (int j = 0; j < colsB; j++) {
                for (int k = 0; k < colsA; k++) {
                    result[i][j] += A[i][k] * B[k][j];
                }
            }
        }
        return result;
    }

    private double[][] matrixAdd(double[][] A, double[][] B) {
        int rows = A.length;
        int cols = A[0].length;
        double[][] result = new double[rows][cols];
        for (int i = 0; i < rows; i++) {
            for (int j = 0; j < cols; j++) {
                result[i][j] = A[i][j] + B[i][j];
            }
        }
        return result;
    }

    private double[][] subtractFromIdentity(double[][] m) {
        int rows = m.length;
        int cols = m[0].length;
        double[][] result = new double[rows][cols];
        for (int i = 0; i < rows; i++) {
            for (int j = 0; j < cols; j++) {
                result[i][j] = (i == j ? 1.0 : 0.0) - m[i][j];
            }
        }
        return result;
    }

    private double[][] inverse(double[][] m) {
        // Simple 3x3 matrix inversion using Cramer's rule
        double det = m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
                   - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
                   + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);

        if (Math.abs(det) < 1e-10) {
            // Matrix is singular, return scaled identity
            double[][] result = new double[3][3];
            result[0][0] = result[1][1] = result[2][2] = 1000.0;
            return result;
        }

        double[][] inv = new double[3][3];
        inv[0][0] = (m[1][1] * m[2][2] - m[1][2] * m[2][1]) / det;
        inv[0][1] = (m[0][2] * m[2][1] - m[0][1] * m[2][2]) / det;
        inv[0][2] = (m[0][1] * m[1][2] - m[0][2] * m[1][1]) / det;
        inv[1][0] = (m[1][2] * m[2][0] - m[1][0] * m[2][2]) / det;
        inv[1][1] = (m[0][0] * m[2][2] - m[0][2] * m[2][0]) / det;
        inv[1][2] = (m[0][2] * m[1][0] - m[0][0] * m[1][2]) / det;
        inv[2][0] = (m[1][0] * m[2][1] - m[1][1] * m[2][0]) / det;
        inv[2][1] = (m[0][1] * m[2][0] - m[0][0] * m[2][1]) / det;
        inv[2][2] = (m[0][0] * m[1][1] - m[0][1] * m[1][0]) / det;

        return inv;
    }
}
