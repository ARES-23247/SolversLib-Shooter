package org.firstinspires.ftc.teamcode.command.subsystems.drive;

import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import org.firstinspires.ftc.teamcode.command.subsystems.encoders.SwerveEncoder;
import com.seattlesolvers.solverslib.hardware.motors.CRServoEx;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.geometry.Vector2d;
import com.seattlesolvers.solverslib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.seattlesolvers.solverslib.controller.PIDFController;
import org.firstinspires.ftc.teamcode.util.control.SlewRateLimiter;

import static org.firstinspires.ftc.teamcode.Constants.*;

/**
 * Encoder-agnostic swerve module with independent steering and drive control.
 *
 * <p>This is the updated version of OctoSwerveModule that supports both OctoQuad digital
 * encoders and SRS Hub analog encoders via the {@link SwerveEncoder} interface.</p>
 *
 * <h3>Encoder Flexibility:</h3>
 * <p>This module can use:</p>
 * <ul>
 *   <li><b>OctoQuad Digital:</b> High-precision 8192 CPR encoders (recommended)</li>
 *   <li><b>SRS Hub Analog:</b> Potentiometers or magnetic encoders (lower cost)</li>
 * </ul>
 * <p>Switch between encoder types via {@link org.firstinspires.ftc.teamcode.Constants#SWERVE_ENCODER_TYPE}</p>
 *
 * <h3>Construction:</h3>
 * <p>Use {@link OctoSwerveDrivetrainV2} which automatically creates the correct
 * encoder type based on configuration.</p>
 *
 * @see SwerveEncoder
 * @see org.firstinspires.ftc.teamcode.commandbase.subsystems.swerve.encoders.OctoQuadSwerveEncoder
 * @see org.firstinspires.ftc.teamcode.commandbase.subsystems.swerve.encoders.AnalogSwerveEncoder
 */
public class OctoSwerveModuleV2 {

    private final MotorEx driveMotor;
    private final CRServoEx steerServo;
    private final SwerveEncoder steeringEncoder;
    private final SwerveEncoder driveEncoder;
    private final Vector2d location;
    private final PIDFController steerPID;
    private final PIDFController drivePID;
    private final double maxSpeed;
    private final SlewRateLimiter slewRateLimiter;
    private final boolean slewRateEnabled;
    private double offsetRadiant = 0;
    private double currentTargetAngleRadians = 0;
    private double currentTargetMagnitude = 0;
    private double currentVelocityTicksPerSec = 0;

    // Telemetry/debugging values
    private double lastDriveFeedforward = 0.0;
    private double lastDrivePIDCorrection = 0.0;
    private double lastDriveMotorPower = 0.0;
    private double lastSteerFeedforward = 0.0;
    private double lastSteerPIDCorrection = 0.0;
    private double lastSteerServoPower = 0.0;

    public OctoSwerveModuleV2(MotorEx driveMotor, CRServoEx steerServo,
                              SwerveEncoder steeringEncoder, SwerveEncoder driveEncoder,
                              Vector2d location, double maxSpeed,
                              PIDFCoefficients swervoPIDFCoefficients, PIDFCoefficients drivePIDFCoefficients) {
        this.driveMotor = driveMotor;
        this.steerServo = steerServo;
        this.steeringEncoder = steeringEncoder;
        this.driveEncoder = driveEncoder;
        this.location = location;
        this.maxSpeed = maxSpeed;

        this.steerServo.setRunMode(CRServoEx.RunMode.RawPower);
        this.driveMotor.setRunMode(MotorEx.RunMode.RawPower);

        this.steerPID = new PIDFController(swervoPIDFCoefficients);
        this.drivePID = new PIDFController(drivePIDFCoefficients);

        this.slewRateEnabled = ENABLE_SLEW_RATE_LIMIT;
        this.slewRateLimiter = new SlewRateLimiter(DRIVE_SLEW_RATE_LIMIT, 0.0);
    }

    public void setOffset(double offsetRadiant) {
        this.offsetRadiant = offsetRadiant;
    }

    public double getModuleHeadingRadians() {
        return steeringEncoder.getPosition() - offsetRadiant;
    }

    public double getCurrentVelocityInchesPerSec() {
        return currentVelocityTicksPerSec * 0.001;
    }

    public double getTargetAngleRadians() {
        return currentTargetAngleRadians;
    }

    public double getTargetMagnitude() {
        return currentTargetMagnitude;
    }

    public Vector2d getLocation() {
        return location;
    }

    public Vector2d calculateVectorRobotCentric(ChassisSpeeds targetVelocity) {
        double vX = targetVelocity.vxMetersPerSecond - targetVelocity.omegaRadiansPerSecond * location.getY();
        double vY = targetVelocity.vyMetersPerSecond + targetVelocity.omegaRadiansPerSecond * location.getX();
        return new Vector2d(vX, vY);
    }

    public void updateModuleWithVelocity(Vector2d targetVector) {
        double targetMagnitude = targetVector.magnitude();
        double targetAngle = Math.atan2(targetVector.getY(), targetVector.getX());

        double currentAngle = getModuleHeadingRadians();

        // Normalize Angle to -PI to PI
        while (currentAngle > Math.PI) currentAngle -= 2 * Math.PI;
        while (currentAngle <= -Math.PI) currentAngle += 2 * Math.PI;

        double error = targetAngle - currentAngle;

        // Optimize short path logic
        while (error > Math.PI) error -= 2 * Math.PI;
        while (error <= -Math.PI) error += 2 * Math.PI;

        if (Math.abs(error) > Math.PI / 2) {
            targetMagnitude *= -1;
            error -= Math.copySign(Math.PI, error);

            targetAngle -= Math.copySign(Math.PI, error);
            while (targetAngle > Math.PI) targetAngle -= 2 * Math.PI;
            while (targetAngle <= -Math.PI) targetAngle += 2 * Math.PI;
        }

        this.currentTargetAngleRadians = targetAngle;
        this.currentTargetMagnitude = targetMagnitude;

        // Steer Power with kS feedforward
        // Feedforward helps overcome gear resistance in the steering mechanism
        lastSteerFeedforward = STEERING_KS * Math.signum(error);
        lastSteerPIDCorrection = steerPID.calculate(0, error);
        lastSteerServoPower = lastSteerFeedforward + lastSteerPIDCorrection;
        steerServo.set(lastSteerServoPower);

        // Drive Power
        currentVelocityTicksPerSec = driveEncoder.getVelocity();

        // Improved feedforward with kV (velocity) and kS (static friction)
        double targetSpeedNormalized = targetMagnitude / maxSpeed;

        // Feedforward equation: motorPower = kS * sign(velocity) + kV * velocity
        lastDriveFeedforward = DRIVE_KS * Math.signum(targetSpeedNormalized) + DRIVE_KV * targetSpeedNormalized;

        double driveError = targetMagnitude - (currentVelocityTicksPerSec * 0.001);
        lastDrivePIDCorrection = drivePID.calculate(0, -driveError);

        lastDriveMotorPower = lastDriveFeedforward + lastDrivePIDCorrection;

        // Apply slew rate limiting
        if (slewRateEnabled) {
            lastDriveMotorPower = slewRateLimiter.calculate(lastDriveMotorPower, 0.02);
        }

        driveMotor.set(lastDriveMotorPower);
    }

    public void stop() {
        steerServo.set(0);
        driveMotor.set(0);
    }

    public String getEncoderType() {
        return steeringEncoder.getEncoderType();
    }

    // Telemetry/debugging getters

    /**
     * Gets the last calculated drive feedforward value.
     * @return feedforward = kS * sign(velocity) + kV * velocity
     */
    public double getLastDriveFeedforward() {
        return lastDriveFeedforward;
    }

    /**
     * Gets the last drive PID correction value.
     * @return PID correction term
     */
    public double getLastDrivePIDCorrection() {
        return lastDrivePIDCorrection;
    }

    /**
     * Gets the last motor power applied to the drive motor.
     * @return motor power after feedforward + PID + slew rate limiting
     */
    public double getLastDriveMotorPower() {
        return lastDriveMotorPower;
    }

    /**
     * Gets the last calculated steering feedforward value (kS).
     * @return kS * sign(error)
     */
    public double getLastSteerFeedforward() {
        return lastSteerFeedforward;
    }

    /**
     * Gets the last steering PID correction value.
     * @return PID correction term
     */
    public double getLastSteerPIDCorrection() {
        return lastSteerPIDCorrection;
    }

    /**
     * Gets the last servo power applied to the steering servo.
     * @return servo power after feedforward + PID
     */
    public double getLastSteerServoPower() {
        return lastSteerServoPower;
    }
}
