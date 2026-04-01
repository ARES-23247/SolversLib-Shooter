package org.firstinspires.ftc.teamcode.hardware;

import org.firstinspires.ftc.teamcode.hardware.SRSHub;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.io.IOException;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;

/**
 * VL53L5CX Multi-Zone Time-of-Flight Distance Sensor Driver.
 *
 * <p>The VL53L5CX is an advanced ToF sensor with 8×8 zone matrix (64 distance zones).
 * This driver provides low-level I2C communication and high-level distance reading.</p>
 *
 * <h3>Key Features:</h3>
 * <ul>
 *   <li><b>8×8 Zone Matrix:</b> 64 individual distance measurements (4×4 mode also supported)</li>
 *   <li><b>Range:</b> Up to 4000mm (4 meters)</li>
 *   <li><b>Refresh Rate:</b> 1Hz to 60Hz configurable</li>
 *   <li><b>Field of View:</b> 63° diagonal, 55° horizontal, 37° vertical</li>
 *   <li><b>Accuracy:</b> ±3% typical across range</li>
 * </ul>
 *
 * <h3>Typical Applications:</h3>
 * <ul>
 *   <li>Game piece detection (intake, scoring)</li>
 *   <li>Backdrop distance measurement</li>
 *   <li>Robot obstacle detection</li>
 *   <li>Wall following / navigation</li>
 * </ul>
 *
 * <h3>Connection:</h3>
 * <p>Connect to SRS Hub I2C bus:</p>
 * <ul>
 *   <li>VDD: 3.3V or 5V (check sensor module)</li>
 *   <li>GND: Ground</li>
 *   <li>SDA: I2C data (A4 on SRS)</li>
 *   <li>SCL: I2C clock (A5 on SRS)</li>
 *   <li>INT: Optional interrupt pin (not used in polling mode)</li>
 * </ul>
 *
 * <h3>Default I2C Address:</h> 0x29 (can be changed via XSHUT pin)</p>
 *
 * @see <a href="https://www.st.com/resource/en/datasheet/vl53l5cx.pdf">VL53L5CX Datasheet</a>
 * @see <a href="https://www.st.com/resource/en/user_manual/dm00677537.pdf">VL53L5CX User Manual</a>
 */
public class VL53L5CX {

    /**
     * Default I2C address for VL53L5CX.
     */
    public static final int DEFAULT_I2C_ADDRESS = 0x29;

    /**
     * VL53L5CX device ID (should match 0xF7 for VL53L5CX).
     */
    private static final int DEVICE_ID = 0xF7;

    // ===== Register Addresses =====

    private static final int REG_DEVICE_ID = 0x010F;
    private static final int REG_I2C_ADDRESS = 0x0001;
    private static final int REG_POWER_MODE = 0x0009;
    private static final int REG_MOTION_INDICATOR = 0x000D;
    private static final int REG_SYSTEM_MODE = 0x0051;
    private static final int REG_RESOLUTION = 0x0053;
    private static final int REG_FREQUENCY = 0x0064;
    private static final int REG_INTEGRATION_TIME = 0x0066;
    private static final int REG_SHARPENER_PERCENT = 0x0068;
    private static final int REG_TARGET_ORDER = 0x006A;
    private static final int REG_RANGING_MODE = 0x006C;
    private static final int REG_START_RANGING = 0x0087;

    // ===== Result Register Addresses =====

    private static final int REG_RESULT_HEADER = 0x0000;
    private static final int REG_RESULT_DATA = 0x0096;

    // ===== Constants =====

    private static final int POWER_MODE_ON = 0x0001;
    private static final int POWER_MODE_OFF = 0x0000;
    private static final int SYSTEM_MODE_RESET = 0x0000;
    private static final int SYSTEM_MODE_NORMAL = 0x0001;
    private static final int RES_4X4 = 0x0100;  // 4×4 zones
    private static final int RES_8X8 = 0x0200;  // 8×8 zones
    private static final int RANGING_MODE_CONTINUOUS = 0x0003;
    private static final int RANGING_MODE_AUTONOMOUS = 0x0002;
    private static final int START_RANGING = 0x0001;
    private static final int STOP_RANGING = 0x0000;

    /**
     * Resolution modes for the sensor.
     */
    public enum Resolution {
        /** 4×4 zones (16 zones total) - faster processing, wider zones */
        RES_4X4(4, 16, 0x0100),
        /** 8×8 zones (64 zones total) - detailed spatial awareness */
        RES_8X8(8, 64, 0x0200);

        public final int width;
        public final int totalZones;
        public final int registerValue;

        Resolution(int width, int totalZones, int registerValue) {
            this.width = width;
            this.totalZones = totalZones;
            this.registerValue = registerValue;
        }
    }

    /**
     * Ranging frequency in Hz.
     */
    public enum Frequency {
        FREQ_1HZ(1),
        FREQ_10HZ(10),
        FREQ_15HZ(15),
        FREQ_30HZ(30),
        FREQ_60HZ(60);

        public final int hz;
        Frequency(int hz) {
            this.hz = hz;
        }
    }

    /**
     * Distance measurement from a single zone.
     */
    public static class ZoneDistance {
        /** Distance in millimeters (0 to 4000mm) */
        public final int distanceMm;
        /** Zone status (0 = valid measurement, non-zero = error/invalid) */
        public final int status;
        /** Quality indicator (0-100, higher = better) */
        public final int quality;

        public ZoneDistance(int distanceMm, int status, int quality) {
            this.distanceMm = distanceMm;
            this.status = status;
            this.quality = quality;
        }

        /**
         * Returns true if this zone has a valid distance measurement.
         */
        public boolean isValid() {
            return status == 0 && distanceMm > 0 && distanceMm <= 4000;
        }

        /**
         * Returns distance in inches.
         */
        public double getDistanceInches() {
            return distanceMm / 25.4;
        }

        /**
         * Returns distance in centimeters.
         */
        public double getDistanceCm() {
            return distanceMm / 10.0;
        }

        @Override
        public String toString() {
            return String.format("%.1fmm (%.1fin, Q=%d)", distanceMm, getDistanceInches(), quality);
        }
    }

    /**
     * Complete frame of distance data from the sensor.
     */
    public static class DistanceFrame {
        /** Distance matrix (zones[row][col]) */
        public final ZoneDistance[][] zones;
        /** Resolution of this frame */
        public final Resolution resolution;
        /** Timestamp when frame was captured (nanoseconds) */
        public final long timestampNanos;

        public DistanceFrame(ZoneDistance[][] zones, Resolution resolution, long timestampNanos) {
            this.zones = zones;
            this.resolution = resolution;
            this.timestampNanos = timestampNanos;
        }

        /**
         * Gets the minimum distance across all zones.
         */
        public int getMinDistanceMm() {
            int min = Integer.MAX_VALUE;
            for (ZoneDistance[] row : zones) {
                for (ZoneDistance zone : row) {
                    if (zone.isValid()) {
                        min = Math.min(min, zone.distanceMm);
                    }
                }
            }
            return min == Integer.MAX_VALUE ? -1 : min;
        }

        /**
         * Gets the center zone distance (useful for game piece detection).
         */
        public ZoneDistance getCenterZone() {
            int centerRow = resolution.width / 2;
            int centerCol = resolution.width / 2;
            return zones[centerRow][centerCol];
        }

        /**
         * Gets a specific row of zones (useful for horizontal detection).
         */
        public ZoneDistance[] getRow(int rowIndex) {
            return zones[rowIndex].clone();
        }

        /**
         * Gets a specific column of zones (useful for vertical detection).
         */
        public ZoneDistance[] getColumn(int colIndex) {
            ZoneDistance[] column = new ZoneDistance[resolution.width];
            for (int i = 0; i < resolution.width; i++) {
                column[i] = zones[i][colIndex];
            }
            return column;
        }

        /**
         * Counts how many zones have objects closer than the threshold distance.
         */
        public int countZonesCloserThan(int thresholdMm) {
            int count = 0;
            for (ZoneDistance[] row : zones) {
                for (ZoneDistance zone : row) {
                    if (zone.isValid() && zone.distanceMm < thresholdMm) {
                        count++;
                    }
                }
            }
            return count;
        }

        /**
         * Finds the zone with the minimum distance.
         */
        public int[] findClosestZone() {
            int minDist = Integer.MAX_VALUE;
            int minRow = -1, minCol = -1;

            for (int row = 0; row < resolution.width; row++) {
                for (int col = 0; col < resolution.width; col++) {
                    ZoneDistance zone = zones[row][col];
                    if (zone.isValid() && zone.distanceMm < minDist) {
                        minDist = zone.distanceMm;
                        minRow = row;
                        minCol = col;
                    }
                }
            }

            return new int[]{minRow, minCol};
        }

        @Override
        public String toString() {
            return String.format("Frame[%dx%d, center=%s, min=%dmm]",
                    resolution.width, resolution.width,
                    getCenterZone(), getMinDistanceMm());
        }
    }

    // ===== Instance Variables =====

    private final SRSHub hub;
    private final int i2cAddress;
    private Resolution resolution = Resolution.RES_8X8;
    private Frequency frequency = Frequency.FREQ_15HZ;
    private boolean initialized = false;
    private boolean ranging = false;
    private final ElapsedTime initTimer = new ElapsedTime();

    /**
     * Creates a VL53L5CX sensor instance.
     *
     * @param hub SRS Hub instance for I2C communication
     * @param i2cAddress I2C address (default: 0x29)
     */
    public VL53L5CX(SRSHub hub, int i2cAddress) {
        this.hub = hub;
        this.i2cAddress = i2cAddress;
    }

    /**
     * Creates a VL53L5CX sensor instance with default I2C address.
     *
     * @param hub SRS Hub instance for I2C communication
     */
    public VL53L5CX(SRSHub hub) {
        this(hub, DEFAULT_I2C_ADDRESS);
    }

    // ===== Low-Level I2C Methods =====

    /**
     * Reads a single byte from a register.
     */
    private byte readByte(int register) throws IOException {
        byte[] data = hub.getDeviceClient().read(register, 1);
        return data[0];
    }

    /**
     * Reads multiple bytes from a register.
     */
    private byte[] readBytes(int register, int length) throws IOException {
        return hub.getDeviceClient().read(register, length);
    }

    /**
     * Writes a single byte to a register.
     */
    private void writeByte(int register, byte value) throws IOException {
        hub.getDeviceClient().write(register, new byte[]{value});
    }

    /**
     * Writes multiple bytes to a register.
     */
    private void writeBytes(int register, byte[] values) throws IOException {
        hub.getDeviceClient().write(register, values);
    }

    /**
     * Writes a 16-bit value (little-endian).
     */
    private void writeWord(int register, int value) throws IOException {
        byte[] data = {
            (byte) (value & 0xFF),
            (byte) ((value >> 8) & 0xFF)
        };
        writeBytes(register, data);
    }

    /**
     * Reads a 16-bit value (little-endian).
     */
    private int readWord(int register) throws IOException {
        byte[] data = readBytes(register, 2);
        return ByteBuffer.wrap(data).order(ByteOrder.LITTLE_ENDIAN).getShort() & 0xFFFF;
    }

    // ===== Initialization =====

    /**
     * Initializes the VL53L5CX sensor.
     *
     * <p>This method:</p>
     * <ol>
     *   <li>Verifies device ID</li>
     *   <li>Powers on the sensor</li>
     *   <li>Configures resolution and frequency</li>
     *   <li>Sets ranging mode to continuous</li>
     *   <li>Starts distance measurements</li>
     * </ol>
     *
     * @return true if initialization succeeded
     */
    public boolean initialize() {
        try {
            initTimer.reset();

            // Wait for sensor boot (minimum 2ms after power-on)
            while (initTimer.milliseconds() < 2) {
                Thread.sleep(1);
            }

            // Verify device ID
            int deviceId = readWord(REG_DEVICE_ID);
            if (deviceId != DEVICE_ID) {
                System.err.println("VL53L5CX: Wrong device ID! Expected 0x" +
                        Integer.toHexString(DEVICE_ID) + ", got 0x" +
                        Integer.toHexString(deviceId));
                return false;
            }

            // Power on the sensor
            writeWord(REG_POWER_MODE, POWER_MODE_ON);
            Thread.sleep(5);  // Wait for power-on (datasheet: max 2ms)

            // Disable motion indicator
            writeByte(REG_MOTION_INDICATOR, (byte) 0x00);

            // Initialize system mode
            writeWord(REG_SYSTEM_MODE, SYSTEM_MODE_RESET);
            Thread.sleep(2);
            writeWord(REG_SYSTEM_MODE, SYSTEM_MODE_NORMAL);
            Thread.sleep(2);

            // Set resolution (8×8 default)
            writeWord(REG_RESOLUTION, resolution.registerValue);
            Thread.sleep(2);

            // Set frequency (15Hz default)
            writeWord(REG_FREQUENCY, frequency.hz);
            Thread.sleep(2);

            // Set integration time (recommended: 100ms for 8×8, 50ms for 4×4)
            int integrationTime = (resolution == Resolution.RES_8X8) ? 100 : 50;
            writeWord(REG_INTEGRATION_TIME, integrationTime);

            // Set sharpener percent (default: 5)
            writeWord(REG_SHARPENER_PERCENT, 5);

            // Set target order (default: closest)
            writeWord(REG_TARGET_ORDER, 0);

            // Set ranging mode to continuous
            writeWord(REG_RANGING_MODE, RANGING_MODE_CONTINUOUS);
            Thread.sleep(2);

            initialized = true;
            return true;

        } catch (IOException | InterruptedException e) {
            System.err.println("VL53L5CX: Initialization failed: " + e.getMessage());
            e.printStackTrace();
            return false;
        }
    }

    /**
     * Sets the sensor resolution.
     *
     * @param resolution New resolution (4×4 or 8×8)
     * @throws IOException if I2C communication fails
     */
    public void setResolution(Resolution resolution) throws IOException {
        if (this.resolution == resolution) return;

        stopRanging();
        writeWord(REG_RESOLUTION, resolution.registerValue);
        this.resolution = resolution;

        // Adjust integration time for resolution
        int integrationTime = (resolution == Resolution.RES_8X8) ? 100 : 50;
        writeWord(REG_INTEGRATION_TIME, integrationTime);

        startRanging();
    }

    /**
     * Sets the ranging frequency.
     *
     * @param frequency New frequency in Hz
     * @throws IOException if I2C communication fails
     */
    public void setFrequency(Frequency frequency) throws IOException {
        writeWord(REG_FREQUENCY, frequency.hz);
        this.frequency = frequency;
    }

    /**
     * Starts continuous ranging.
     *
     * @throws IOException if I2C communication fails
     */
    public void startRanging() throws IOException {
        if (!initialized) {
            throw new IllegalStateException("Sensor not initialized. Call initialize() first.");
        }

        writeWord(REG_START_RANGING, START_RANGING);
        ranging = true;
    }

    /**
     * Stops continuous ranging.
     *
     * @throws IOException if I2C communication fails
     */
    public void stopRanging() throws IOException {
        if (!ranging) return;

        writeWord(REG_START_RANGING, STOP_RANGING);
        ranging = false;
    }

    /**
     * Checks if new data is available.
     *
     * @return true if new distance data is ready to read
     * @throws IOException if I2C communication fails
     */
    public boolean isDataReady() throws IOException {
        byte status = readByte(REG_RESULT_HEADER);
        return (status & 0x01) != 0;
    }

    /**
     * Reads the latest distance frame from the sensor.
     *
     * <p>This method blocks until new data is available.</p>
     *
     * @return DistanceFrame with zone data
     * @throws IOException if I2C communication fails
     */
    public DistanceFrame readDistanceFrame() throws IOException {
        if (!ranging) {
            throw new IllegalStateException("Ranging not started. Call startRanging() first.");
        }

        // Wait for new data (timeout: 200ms)
        long timeoutNanos = System.nanoTime() + 200_000_000L;
        while (!isDataReady()) {
            if (System.nanoTime() > timeoutNanos) {
                throw new IOException("Timeout waiting for VL53L5CX data");
            }
            Thread.yield();
        }

        // Read distance data
        int zoneCount = resolution.totalZones;
        int dataSize = zoneCount * 3;  // Each zone: 2 bytes distance + 1 byte status

        byte[] data = readBytes(REG_RESULT_DATA, dataSize);

        // Parse zone data
        ZoneDistance[][] zones = new ZoneDistance[resolution.width][resolution.width];
        int index = 0;

        for (int row = 0; row < resolution.width; row++) {
            for (int col = 0; col < resolution.width; col++) {
                // Distance is 16-bit (little-endian)
                int distanceMm = ByteBuffer.wrap(new byte[]{data[index], data[index + 1]})
                        .order(ByteOrder.LITTLE_ENDIAN).getShort() & 0xFFFF;
                index += 2;

                // Status byte (bit 0: valid, bit 1: signal too low, etc.)
                int status = data[index] & 0xFF;
                index++;

                // Quality estimation (0-100, derived from status)
                int quality = (status == 0) ? 100 : 0;

                zones[row][col] = new ZoneDistance(distanceMm, status, quality);
            }
        }

        return new DistanceFrame(zones, resolution, System.nanoTime());
    }

    /**
     * Reads distance frame with timeout.
     *
     * @param timeoutMs Maximum time to wait for new data (milliseconds)
     * @return DistanceFrame if data available, null if timeout
     * @throws IOException if I2C communication fails
     */
    public DistanceFrame readDistanceFrame(long timeoutMs) throws IOException {
        if (!ranging) {
            throw new IllegalStateException("Ranging not started. Call startRanging() first.");
        }

        long timeoutNanos = System.nanoTime() + timeoutMs * 1_000_000L;

        while (!isDataReady()) {
            if (System.nanoTime() > timeoutNanos) {
                return null;  // Timeout
            }
            Thread.yield();
        }

        return readDistanceFrame();
    }

    /**
     * Gets the current resolution.
     */
    public Resolution getResolution() {
        return resolution;
    }

    /**
     * Gets the current frequency.
     */
    public Frequency getFrequency() {
        return frequency;
    }

    /**
     * Checks if sensor is initialized.
     */
    public boolean isInitialized() {
        return initialized;
    }

    /**
     * Checks if ranging is active.
     */
    public boolean isRanging() {
        return ranging;
    }

    /**
     * Closes the sensor and releases resources.
     */
    public void close() {
        try {
            stopRanging();
            writeWord(REG_POWER_MODE, POWER_MODE_OFF);
        } catch (IOException e) {
            System.err.println("VL53L5CX: Error closing sensor: " + e.getMessage());
        }
        initialized = false;
    }
}
