package org.firstinspires.ftc.teamcode.opmode.samples;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.SRSHub;
import org.firstinspires.ftc.teamcode.hardware.VL53L5CX;

/**
 * Sample OpMode for testing the VL53L5CX distance sensor.
 *
 * <p>This OpMode demonstrates:</p>
 * <ul>
 *   <li>Sensor initialization</li>
 *   <li>Reading distance data</li>
 *   <li>Multi-zone distance visualization</li>
 *   <li>Performance monitoring</li>
 * </ul>
 *
 * <h3>Controls:</h3>
 * <ul>
 *   <li>Gamepad A: Toggle between 4×4 and 8×8 resolution</li>
 *   <li>Gamepad B: Change frequency</li>
 *   <li>Gamepad X: Print detailed zone data</li>
 * </ul>
 *
 * @see VL53L5CX
 */
@TeleOp(name = "VL53L5CX Test", group = "Test")
public class VL53L5CXSample extends LinearOpMode {

    private VL53L5CX sensor;
    private SRSHub srsHub;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initializing VL53L5CX...");
        telemetry.update();

        // Initialize SRS Hub
        srsHub = hardwareMap.get(SRSHub.class, "srsHub");

        // Create sensor with default I2C address (0x29)
        sensor = new VL53L5CX(srsHub);

        // Initialize sensor
        if (!sensor.initialize()) {
            telemetry.addData("Error", "Failed to initialize VL53L5CX!");
            telemetry.update();
            return;
        }

        telemetry.addData("Status", "VL53L5CX initialized!");
        telemetry.addData("Resolution", sensor.getResolution().toString());
        telemetry.addData("Frequency", sensor.getFrequency().hz + "Hz");
        telemetry.addData("", "Press PLAY to start");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        telemetry.clearAll();
        long lastUpdateTime = 0;
        int frameCount = 0;
        long fpsStartTime = System.nanoTime();

        while (opModeIsActive() && !isStopRequested()) {
            // Handle gamepad input
            if (gamepad1.a) {
                toggleResolution();
                Thread.sleep(200);  // Debounce
            }
            if (gamepad1.b) {
                cycleFrequency();
                Thread.sleep(200);  // Debounce
            }
            if (gamepad1.x) {
                printDetailedZoneData();
                Thread.sleep(500);  // Debounce
            }

            // Read sensor data
            try {
                long startTime = System.nanoTime();
                VL53L5CX.DistanceFrame frame = sensor.readDistanceFrame(100);
                long readTime = (System.nanoTime() - startTime) / 1_000_000;  // ms

                if (frame != null) {
                    // Calculate FPS
                    frameCount++;
                    long elapsedNanos = System.nanoTime() - fpsStartTime;
                    double fps = (frameCount * 1_000_000_000.0) / elapsedNanos;

                    // Display center zone distance
                    VL53L5CX.ZoneDistance center = frame.getCenterZone();
                    telemetry.addData("Center Distance", center.toString());

                    // Display minimum distance
                    int minDist = frame.getMinDistanceMm();
                    telemetry.addData("Min Distance", minDist + "mm (" + (minDist / 25.4) + "in)");

                    // Display zone count with objects
                    int closeZones = frame.countZonesCloserThan(500);  // Objects within 50cm
                    telemetry.addData("Zones < 50cm", closeZones + " / " + frame.resolution.totalZones);

                    // Display closest zone location
                    int[] closest = frame.findClosestZone();
                    telemetry.addData("Closest Zone", "Row " + closest[0] + ", Col " + closest[1]);

                    // Display resolution and frequency
                    telemetry.addData("Resolution", frame.resolution.width + "×" + frame.resolution.width);
                    telemetry.addData("Frequency", sensor.getFrequency().hz + "Hz");

                    // Display read time and FPS
                    telemetry.addData("Read Time", readTime + "ms");
                    telemetry.addData("FPS", String.format("%.1f", fps));

                    // Visual zone representation (8×8 max)
                    if (frame.resolution == VL53L5CX.Resolution.RES_4X4) {
                        printZones4x4(frame);
                    } else {
                        printZones8x8(frame);
                    }

                    telemetry.addLine();
                    telemetry.addData("Controls", "A: Res | B: Freq | X: Details");

                } else {
                    telemetry.addData("Error", "No data received (timeout)");
                }

            } catch (Exception e) {
                telemetry.addData("Error", e.getMessage());
                e.printStackTrace();
            }

            telemetry.update();
            Thread.yield();
        }

        // Cleanup
        sensor.close();
    }

    /**
     * Toggles between 4×4 and 8×8 resolution.
     */
    private void toggleResolution() {
        try {
            VL53L5CX.Resolution current = sensor.getResolution();
            VL53L5CX.Resolution newRes = (current == VL53L5CX.Resolution.RES_4X4)
                    ? VL53L5CX.Resolution.RES_8X8
                    : VL53L5CX.Resolution.RES_4X4;

            sensor.setResolution(newRes);
            System.out.println("Resolution: " + newRes);
        } catch (Exception e) {
            System.out.println("Failed to change resolution: " + e.getMessage());
        }
    }

    /**
     * Cycles through available frequencies.
     */
    private void cycleFrequency() {
        try {
            VL53L5CX.Frequency[] freqs = VL53L5CX.Frequency.values();
            VL53L5CX.Frequency current = sensor.getFrequency();

            int nextIndex = (current.ordinal() + 1) % freqs.length;
            VL53L5CX.Frequency next = freqs[nextIndex];

            sensor.setFrequency(next);
            System.out.println("Frequency: " + next.hz + "Hz");
        } catch (Exception e) {
            System.out.println("Failed to change frequency: " + e.getMessage());
        }
    }

    /**
     * Prints detailed zone data to telemetry log.
     */
    private void printDetailedZoneData() {
        try {
            VL53L5CX.DistanceFrame frame = sensor.readDistanceFrame(100);
            if (frame == null) return;

            System.out.println("=== VL53L5CX Zone Data ===");
            System.out.println("Resolution: " + frame.resolution.width + "×" + frame.resolution.width);

            for (int row = 0; row < frame.resolution.width; row++) {
                StringBuilder sb = new StringBuilder();
                sb.append(String.format("Row %d: ", row));
                for (int col = 0; col < frame.resolution.width; col++) {
                    VL53L5CX.ZoneDistance zone = frame.zones[row][col];
                    if (zone.isValid()) {
                        sb.append(String.format("%4d ", zone.distanceMm));
                    } else {
                        sb.append(" ---- ");
                    }
                }
                System.out.println(sb.toString());
            }
            System.out.println("========================");
        } catch (Exception e) {
            System.out.println("Failed to read: " + e.getMessage());
        }
    }

    /**
     * Prints a 4×4 ASCII visualization of distance zones.
     */
    private void printZones4x4(VL53L5CX.DistanceFrame frame) {
        telemetry.addLine("4×4 Distance Zones (mm):");

        for (int row = 0; row < 4; row++) {
            StringBuilder sb = new StringBuilder();
            for (int col = 0; col < 4; col++) {
                VL53L5CX.ZoneDistance zone = frame.zones[row][col];
                if (zone.isValid()) {
                    // Use symbols to represent distance
                    char symbol = getDistanceSymbol(zone.distanceMm);
                    sb.append(symbol).append(" ");
                } else {
                    sb.append("? ");
                }
            }
            telemetry.addLine(sb.toString());
        }
    }

    /**
     * Prints an 8×8 ASCII visualization of distance zones.
     */
    private void printZones8x8(VL53L5CX.DistanceFrame frame) {
        telemetry.addLine("8×8 Distance Zones:");

        for (int row = 0; row < 8; row++) {
            StringBuilder sb = new StringBuilder();
            for (int col = 0; col < 8; col++) {
                VL53L5CX.ZoneDistance zone = frame.zones[row][col];
                if (zone.isValid()) {
                    char symbol = getDistanceSymbol(zone.distanceMm);
                    sb.append(symbol);
                } else {
                    sb.append("?");
                }
            }
            telemetry.addLine(sb.toString());
        }
    }

    /**
     * Returns a symbol representing the distance range.
     */
    private char getDistanceSymbol(int distanceMm) {
        if (distanceMm < 200) return '■';      // Very close
        if (distanceMm < 500) return '▪';      // Close
        if (distanceMm < 1000) return '▫';     // Medium
        if (distanceMm < 2000) return '·';      // Far
        return ' ';                             // Very far
    }
}
