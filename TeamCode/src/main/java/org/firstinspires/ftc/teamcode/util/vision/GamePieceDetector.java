package org.firstinspires.ftc.teamcode.util.vision;

import org.firstinspires.ftc.teamcode.hardware.SRSHub;
import org.firstinspires.ftc.teamcode.hardware.VL53L5CX;

/**
 * Game Piece Detection System using VL53L5CX sensor.
 *
 * <p>This class uses the VL53L5CX multi-zone ToF sensor to detect game pieces
 * in front of the intake. It provides methods for:</p>
 *
 * <ul>
 *   <li>Detecting if a game piece is in the intake zone</li>
 *   <li>Measuring distance to game piece</li>
 *   <li>Detecting if game piece is centered</li>
 *   <li>Detecting multiple pieces (spike detection)</li>
 *   <li>Auto-indexing (positioning piece correctly)</li>
 * </ul>
 *
 * <h3>Mounting:</h3>
 * <p>Mount the VL53L5CX at the front of the robot, centered on the intake,
 * angled 15-20° downward to detect game pieces on the floor.</p>
 *
 * <h3>Usage Example:</h3>
 * <pre>{@code
 * // In Robot.java
 * gamePieceDetector = new GamePieceDetector(srsHub);
 * gamePieceDetector.initialize();
 *
 * // In teleop
 * if (gamePieceDetector.hasGamePiece()) {
 *     intake.run();
 *     if (gamePieceDetector.isPieceCentered()) {
 *         intake.stop();
 *     }
 * }
 * }</pre>
 *
 * @see VL53L5CX
 */
public class GamePieceDetector {

    /**
     * Detection threshold distance (mm).
     * Objects closer than this are considered "in the intake zone".
     */
    private static final int DETECTION_THRESHOLD_MM = 300;  // 30cm

    /**
     * Center tolerance for determining if piece is centered (mm).
     */
    private static final int CENTER_TOLERANCE_MM = 50;  // 5cm

    /**
     * Multiple piece detection threshold (number of zones with objects).
     */
    private static final int MULTI_PIECE_ZONE_COUNT = 4;

    private final VL53L5CX sensor;
    private boolean initialized = false;
    private VL53L5CX.DistanceFrame lastFrame;

    /**
     * Creates a game piece detector.
     *
     * @param srsHub SRS Hub for I2C communication
     */
    public GamePieceDetector(SRSHub srsHub) {
        // Use 4×4 resolution for faster processing and wider zones
        this.sensor = new VL53L5CX(srsHub);
        try {
            sensor.setResolution(VL53L5CX.Resolution.RES_4X4);
        } catch (Exception e) {
            System.err.println("Failed to set resolution: " + e.getMessage());
        }
    }

    /**
     * Creates a game piece detector with custom I2C address.
     *
     * @param srsHub SRS Hub for I2C communication
     * @param i2cAddress Custom I2C address
     */
    public GamePieceDetector(SRSHub srsHub, int i2cAddress) {
        this.sensor = new VL53L5CX(srsHub, i2cAddress);
    }

    /**
     * Initializes the game piece detector.
     *
     * @return true if initialization succeeded
     */
    public boolean initialize() {
        initialized = sensor.initialize();
        if (initialized) {
            try {
                sensor.startRanging();
            } catch (Exception e) {
                System.err.println("Failed to start ranging: " + e.getMessage());
                initialized = false;
            }
        }
        return initialized;
    }

    /**
     * Updates the detector with latest sensor data.
     *
     * <p>Call this once per loop iteration.</p>
     */
    public void update() {
        if (!initialized) return;

        try {
            VL53L5CX.DistanceFrame frame = sensor.readDistanceFrame(50);
            if (frame != null) {
                lastFrame = frame;
            }
        } catch (Exception e) {
            System.err.println("Failed to read distance: " + e.getMessage());
        }
    }

    /**
     * Checks if a game piece is detected in the intake zone.
     *
     * @return true if an object is closer than DETECTION_THRESHOLD_MM
     */
    public boolean hasGamePiece() {
        if (lastFrame == null) return false;

        int minDist = lastFrame.getMinDistanceMm();
        return minDist > 0 && minDist < DETECTION_THRESHOLD_MM;
    }

    /**
     * Gets the distance to the game piece (center zone).
     *
     * @return Distance in millimeters, or -1 if no piece detected
     */
    public int getPieceDistanceMm() {
        if (lastFrame == null) return -1;

        VL53L5CX.ZoneDistance center = lastFrame.getCenterZone();
        return center.isValid() ? center.distanceMm : -1;
    }

    /**
     * Gets the distance to the game piece in inches.
     *
     * @return Distance in inches, or -1 if no piece detected
     */
    public double getPieceDistanceInches() {
        int mm = getPieceDistanceMm();
        return mm > 0 ? mm / 25.4 : -1;
    }

    /**
     * Checks if the game piece is centered in the intake.
     *
     * <p>Uses the center 2×2 zones to verify the piece is centered.</p>
     *
     * @return true if piece is detected and centered within tolerance
     */
    public boolean isPieceCentered() {
        if (!hasGamePiece()) return false;

        // Get center zones (2×2 grid)
        VL53L5CX.ZoneDistance[][] zones = lastFrame.zones;
        int center = zones.length / 2;

        // Check center 2×2 zones
        int[] distances = {
            zones[center - 1][center - 1].distanceMm,
            zones[center - 1][center].distanceMm,
            zones[center][center - 1].distanceMm,
            zones[center][center].distanceMm
        };

        // Calculate variance (should be low if piece is centered)
        int min = Integer.MAX_VALUE;
        int max = Integer.MIN_VALUE;
        for (int d : distances) {
            if (d > 0 && d < DETECTION_THRESHOLD_MM) {
                min = Math.min(min, d);
                max = Math.max(max, d);
            }
        }

        // Piece is centered if all center zones have similar distances
        return (max - min) < CENTER_TOLERANCE_MM;
    }

    /**
     * Detects if there are multiple game pieces close together (spike).
     *
     * @return true if multiple zones detect objects
     */
    public boolean hasMultiplePieces() {
        if (lastFrame == null) return false;

        int closeZones = lastFrame.countZonesCloserThan(DETECTION_THRESHOLD_MM);
        return closeZones >= MULTI_PIECE_ZONE_COUNT;
    }

    /**
     * Gets the direction to adjust to center the game piece.
     *
     * @return -1 (move left), 0 (centered), 1 (move right), or null if no piece
     */
    public Integer getCenteringDirection() {
        if (!hasGamePiece()) return null;
        if (isPieceCentered()) return 0;

        VL53L5CX.ZoneDistance[][] zones = lastFrame.zones;
        int center = zones.length / 2;

        // Compare left vs right distances
        int leftDist = 0, rightDist = 0;
        int leftCount = 0, rightCount = 0;

        for (int row = 0; row < zones.length; row++) {
            for (int col = 0; col < zones.length; col++) {
                VL53L5CX.ZoneDistance zone = zones[row][col];
                if (zone.isValid() && zone.distanceMm < DETECTION_THRESHOLD_MM) {
                    if (col < center) {
                        leftDist += zone.distanceMm;
                        leftCount++;
                    } else if (col > center) {
                        rightDist += zone.distanceMm;
                        rightCount++;
                    }
                }
            }
        }

        if (leftCount == 0 || rightCount == 0) return 0;

        double leftAvg = (double) leftDist / leftCount;
        double rightAvg = (double) rightDist / rightCount;

        // If left is closer, move right (return 1), and vice versa
        return (leftAvg < rightAvg) ? 1 : -1;
    }

    /**
     * Gets the minimum distance detected.
     *
     * @return Minimum distance in millimeters
     */
    public int getMinDistanceMm() {
        if (lastFrame == null) return -1;
        return lastFrame.getMinDistanceMm();
    }

    /**
     * Gets the last complete distance frame.
     *
     * @return Last frame, or null if no data available
     */
    public VL53L5CX.DistanceFrame getLastFrame() {
        return lastFrame;
    }

    /**
     * Checks if the detector is initialized.
     */
    public boolean isInitialized() {
        return initialized;
    }

    /**
     * Closes the detector and releases resources.
     */
    public void close() {
        sensor.close();
        initialized = false;
    }
}
