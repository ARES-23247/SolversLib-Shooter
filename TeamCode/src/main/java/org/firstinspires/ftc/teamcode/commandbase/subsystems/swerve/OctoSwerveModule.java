package org.firstinspires.ftc.teamcode.commandbase.subsystems.swerve;

import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.hardware.digitalinc.OctoQuad;
import com.seattlesolvers.solverslib.hardware.motors.CRServoEx;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.geometry.Vector2d;
import com.seattlesolvers.solverslib.kinematics.wpilibkinematics.SwerveModuleState;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.kinematics.wpilibkinematics.ChassisSpeeds;

public class OctoSwerveModule {
    private final MotorEx driveMotor;
    private final CRServoEx steerServo;
    private final OctoQuad octoQuad;
    private final int steerPort;
    private final int drivePort;
    private final Vector2d location;
    private final PIDFController steerPID;
    private final PIDFController drivePID;
    private final double maxSpeed;

    private double offsetRadiant = 0;
    
    // Telemetry Caches
    private double currentTargetAngleRadians = 0;
    private double currentTargetMagnitude = 0;
    private double currentVelocityTicksPerSec = 0;

    public OctoSwerveModule(MotorEx driveMotor, CRServoEx steerServo, OctoQuad octoQuad, int steerPort, int drivePort, Vector2d location, double maxSpeed, PIDFCoefficients swervoPIDFCoefficients, PIDFCoefficients drivePIDFCoefficients) {
        this.driveMotor = driveMotor;
        this.steerServo = steerServo;
        this.octoQuad = octoQuad;
        this.steerPort = steerPort;
        this.drivePort = drivePort;
        this.location = location;
        this.maxSpeed = maxSpeed;
        
        // Disable SolversLib internal Positional Control because we manage it here with the OctoQuad
        this.steerServo.setRunMode(CRServoEx.RunMode.RawPower);
        this.driveMotor.setRunMode(MotorEx.RunMode.RawPower);
        
        this.steerPID = new PIDFController(swervoPIDFCoefficients);
        this.drivePID = new PIDFController(drivePIDFCoefficients);
    }

    public void setOffset(double offsetRadiant) {
        this.offsetRadiant = offsetRadiant;
    }

    public double getModuleHeadingRadians() {
        // OctoQuad counts per revolution is typically 8192 for REV Through-bore or 4096 for standard, depending on attached encoder
        // Assume 8192 counts per Rev for REV Through Bore Encoder
        int ticks = octoQuad.readSinglePosition(steerPort);
        double rads = ((ticks % 8192) / 8192.0) * 2 * Math.PI;
        return rads - offsetRadiant;
    }

    public double getCurrentVelocityInchesPerSec() {
        // Approximate conversion using target vs tick comparison
        // NOTE: Replace 0.001 with physical WHEEL_RADIUS * Math.PI * 2 / TICKS_PER_REV if known!
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
            
            // Adjust cached target angle for accurate graphing (flips 180 degrees)
            targetAngle -= Math.copySign(Math.PI, error);
            while (targetAngle > Math.PI) targetAngle -= 2 * Math.PI;
            while (targetAngle <= -Math.PI) targetAngle += 2 * Math.PI;
        }

        this.currentTargetAngleRadians = targetAngle;
        this.currentTargetMagnitude = targetMagnitude;

        // Steer Power
        double steerPower = steerPID.calculate(0, error);
        steerServo.set(steerPower);

        // Drive Power (Velocity Control scaled to Ticks/sec proxy or Normalized Value)
        currentVelocityTicksPerSec = octoQuad.readSingleVelocity(drivePort);
        
        // As a simplified proxy for standard Motor operation, normalize target speed against max:
        double targetSpeedNormalized = targetMagnitude / maxSpeed;
        
        // Applying a basic Velocity PID constraint around the desired target normalized vs the measured tick velocity
        // To accurately tune this, you'll want to multiply currentVelocityTicksPerSec by your WHEEL_CIRCUMFERENCE / TICKS_PER_REV.
        // For now, we apply FeedForward (targetSpeedNormalized) + closed-loop adjustment
        double feedForward = targetSpeedNormalized;
        
        // You will need to build the conversion formula into this error metric for the drivePID to compute physically accurate values!
        double driveError = targetMagnitude - (currentVelocityTicksPerSec * 0.001); // 0.001 is a placeholder TICKS_TO_INCHES ratio
        double drivePIDCorrection = drivePID.calculate(0, -driveError);
        
        driveMotor.set(feedForward + drivePIDCorrection);
    }

    public void stop() {
        steerServo.set(0);
        driveMotor.set(0);
    }
}
