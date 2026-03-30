package org.firstinspires.ftc.teamcode.globals;

import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.hardware.motors.CRServoEx;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.util.TelemetryData;
import org.firstinspires.ftc.teamcode.util.DataLogger;

import com.qualcomm.hardware.digitalinc.OctoQuad;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.Drive;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.Vision;

import java.io.File;
import java.io.IOException;

import dev.nullftc.profiler.Profiler;
import dev.nullftc.profiler.entry.BasicProfilerEntryFactory;
import dev.nullftc.profiler.exporter.CSVProfilerExporter;

public class Robot extends com.seattlesolvers.solverslib.command.Robot {

    private static final Robot instance = new Robot();
    public static Robot getInstance() {
        return instance;
    }

    public HardwareMap hardwareMap;
    public Profiler profiler;
    public File file;

    public VoltageSensor voltageSensor;
    private double cachedVoltage;
    private ElapsedTime voltageTimer;

    // ----- Hardware Items -----
    // Swerve Motors
    public MotorEx frontLeftMotor;
    public MotorEx frontRightMotor;
    public MotorEx backLeftMotor;
    public MotorEx backRightMotor;

    // Swerve Coaxial Servos (Melon Super Servos on an SRS Hub)
    public CRServoEx frontLeftServo;
    public CRServoEx frontRightServo;
    public CRServoEx backLeftServo;
    public CRServoEx backRightServo;

    // Pinpoint
    public GoBildaPinpointDriver pinpoint;

    // OctoQuad
    public OctoQuad octoquad;

    // Vision
    public Limelight3A limelight;

    // Subsystems
    public Drive drive;
    public Vision vision;

    // Global Telemetry Instance
    public TelemetryData telemetry;
    public DataLogger logger;

    public void init(HardwareMap hwMap) {
        this.hardwareMap = hwMap;

        // Logging & Profiler setup
        File logsFolder = new File(AppUtil.FIRST_FOLDER, "logs");
        if (!logsFolder.exists()) logsFolder.mkdirs();

        long timestamp = System.currentTimeMillis();
        file = new File(logsFolder, "profiler-" + timestamp + ".csv");
        profiler = Profiler.builder()
                .factory(new BasicProfilerEntryFactory())
                .exporter(new CSVProfilerExporter(file))
                .debugLog(false)
                .build();

        // ---------------------------------------------
        // Photon Configuration: Emulating Decode-2026 
        // ---------------------------------------------
        PhotonCore.CONTROL_HUB.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        PhotonCore.EXPANSION_HUB.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        PhotonCore.experimental.setMaximumParallelCommands(8);

        // Disabling parallel servos because of the SRS Hub (Non-USB hub) causing issues
        PhotonCore.PARALLELIZE_SERVOS = false; 
        PhotonCore.enable();

        voltageSensor = hwMap.voltageSensor.iterator().next();

        // Initialize Swerve Drive Motors
        frontLeftMotor = new MotorEx(hwMap, "frontLeftMotor").setCachingTolerance(0.01);
        frontRightMotor = new MotorEx(hwMap, "frontRightMotor").setCachingTolerance(0.01);
        backLeftMotor = new MotorEx(hwMap, "backLeftMotor").setCachingTolerance(0.01);
        backRightMotor = new MotorEx(hwMap, "backRightMotor").setCachingTolerance(0.01);

        // Initialize Swerve Servos (Melon Super Servos)
        // These are typically continuous rotation logic for Coaxial swerve, so binding them as CRServoEx
        frontLeftServo = new CRServoEx(hwMap, "frontLeftServo").setCachingTolerance(0.01);
        frontRightServo = new CRServoEx(hwMap, "frontRightServo").setCachingTolerance(0.01);
        backLeftServo = new CRServoEx(hwMap, "backLeftServo").setCachingTolerance(0.01);
        backRightServo = new CRServoEx(hwMap, "backRightServo").setCachingTolerance(0.01);

        // Note: OctoQuad is present for processing motor encoders and REV through bores (axial rotation)
        // You usually interact with OctoQuad directly via its driver class to query the positions in the Drive subsystem
        // e.g., OctoQuad octoquad = hwMap.get(OctoQuad.class, "octoquad");

        // Initialize Vision
        logger = new DataLogger("robot-telemetry");
        
        limelight = hwMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(250);
        limelight.pipelineSwitch(0);
        limelight.start();

        // Initialize GoBilda Pinpoint
        // Assumes default pinpoint configuration. Adjust setOffsets() in the future if pinpoint is non-centered.
        pinpoint = hwMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.setOffsets(0, 0, DistanceUnit.MM); 
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        pinpoint.resetPosAndIMU();

        // Initialize OctoQuad
        octoquad = hwMap.get(OctoQuad.class, "octoquad");

        // Instantiate Subsystems
        drive = new Drive();
        vision = new Vision();
    }

    public double getVoltage() {
        if (voltageTimer == null) {
            voltageTimer = new ElapsedTime();
            cachedVoltage = voltageSensor.getVoltage();
        } else if (voltageTimer.milliseconds() > 250) { 
            cachedVoltage = voltageSensor.getVoltage();
            voltageTimer.reset();
        }
        if (((Double) cachedVoltage).isNaN() || cachedVoltage == 0) {
            cachedVoltage = 12;
        }
        return cachedVoltage;
    }

    /**
     * Unified loop to handle command scheduling and clearing bulk caches
     */
    public void updateLoop(TelemetryData telemetryData) {
        // Run Scheduled Robot Commands and Subsystems periodically
        CommandScheduler.getInstance().run();

        if (this.telemetry != null) {
            this.telemetry.update();
        }
        
        if (this.logger != null) {
            this.logger.update();
        }

        // Always clear caches at the end of the run loop!
        PhotonCore.CONTROL_HUB.clearBulkCache();
        PhotonCore.EXPANSION_HUB.clearBulkCache();
    }
}