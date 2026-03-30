package org.firstinspires.ftc.teamcode.globals;

import com.qualcomm.robotcore.hardware.PIDFCoefficients;

public class Constants {
    // Basic Track dimensions (Replace with your actual robot measurements in inches)
    public static final double TRACK_WIDTH = 15.0; // Left-to-Right distance between wheel centers
    public static final double WHEEL_BASE = 15.0; // Front-to-Back distance between wheel centers
    
    // Max absolute kinematic speed allowed for modules
    public static final double MAX_DRIVE_VELOCITY = 60.0; // Inches per second
    
    // PID values for the Melon Super Servos (Module Steering)
    public static final PIDFCoefficients SWERVE_SERVO_PIDF = new PIDFCoefficients(1.2, 0.0, 0.05, 0.0);
    
    // PID values for the Drive Motors (Velocity Control via OctoQuad)
    public static final PIDFCoefficients SWERVE_DRIVE_PIDF = new PIDFCoefficients(0.1, 0.0, 0.01, 0.1);
    
    // Swerve Module Absolute Encoder Offsets (in Radians)
    // Tune these so that all wheels point perfectly forward when reading 0 radians!
    public static final double FRONT_RIGHT_OFFSET = 0.0;
    public static final double FRONT_LEFT_OFFSET = 0.0;
    public static final double BACK_LEFT_OFFSET = 0.0;
    public static final double BACK_RIGHT_OFFSET = 0.0;
    
    // Auto/Pathing PID Tolerance Defaults
    public static final double AUTO_XY_TOLERANCE = 1.0; 
    public static final double AUTO_HEADING_TOLERANCE = Math.toRadians(2.0);
}
