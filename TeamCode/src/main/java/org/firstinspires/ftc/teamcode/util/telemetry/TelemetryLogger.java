package org.firstinspires.ftc.teamcode.util.telemetry;

import org.firstinspires.ftc.teamcode.util.DataLogger;

/**
 * Telemetry logging utility that abstracts CSV and dashboard logging.
 *
 * <p>This class provides a clean interface for logging robot data, separating
 * the concerns of data collection from data presentation. It supports both
 * CSV file logging and dashboard display.</p>
 *
 * <h3>Usage Pattern:</h3>
 * <pre>
 * // Create logger
 * TelemetryLogger logger = new TelemetryLogger("drive");
 *
 * // Add data
 * logger.addData("Robot X", pose.getX());
 * logger.addData("Robot Y", pose.getY());
 *
 * // Update (writes CSV and flushes to disk)
 * logger.update();
 * </pre>
 *
 * <h3>Benefits:</h3>
 * <ul>
 *   <li><b>Separation of Concerns:</b> Logging logic separated from business logic</li>
 *   <li><b>Easy to Disable:</b> Just don't instantiate or use ENABLE_LOGGING flag</li>
 *   <li><b>Performance Mode:</b> Can be completely removed from build in competition</li>
 *   <li><b>Consistent API:</b> Same interface for all subsystems</li>
 * </ul>
 *
 * @see org.firstinspires.ftc.teamcode.util.logging.DataLogger
 */
public class TelemetryLogger {

    private final DataLogger csvLogger;
    private final boolean csvEnabled;
    private final boolean dashboardEnabled;
    private long lastUpdateTimestamp = 0;
    private final long minUpdateIntervalMs;

    /**
     * Creates a new TelemetryLogger with both CSV and dashboard logging enabled.
     *
     * @param filename base filename for CSV logs (without extension)
     */
    public TelemetryLogger(String filename) {
        this(filename, true, true, 0);
    }

    /**
     * Creates a new TelemetryLogger with configurable logging options.
     *
     * @param filename base filename for CSV logs (without extension)
     * @param csvEnabled enable CSV file logging
     * @param dashboardEnabled enable dashboard updates (not yet implemented)
     * @param minUpdateIntervalMs minimum time between updates in milliseconds (0 = every loop)
     */
    public TelemetryLogger(String filename, boolean csvEnabled, boolean dashboardEnabled, long minUpdateIntervalMs) {
        this.csvEnabled = csvEnabled;
        this.dashboardEnabled = dashboardEnabled;
        this.minUpdateIntervalMs = minUpdateIntervalMs;

        if (csvEnabled) {
            this.csvLogger = new DataLogger(filename);
        } else {
            this.csvLogger = null;
        }
    }

    /**
     * Adds or updates a data field for the current log entry.
     *
     * @param key the field name (will become the CSV column header)
     * @param value the value to log (any object, converted to string)
     */
    public void addData(String key, Object value) {
        if (csvEnabled && csvLogger != null) {
            csvLogger.addData(key, value);
        }
    }

    /**
     * Updates the logger, writing data to CSV and flushing to disk.
     *
     * <p>This method respects the minimum update interval to reduce I/O overhead
     * during high-frequency loops.</p>
     */
    public void update() {
        if (!shouldUpdate()) {
            return;
        }

        if (csvEnabled && csvLogger != null) {
            csvLogger.update();
        }

        lastUpdateTimestamp = System.currentTimeMillis();
    }

    /**
     * Checks if enough time has passed since the last update.
     *
     * @return true if the minimum update interval has elapsed
     */
    private boolean shouldUpdate() {
        if (minUpdateIntervalMs <= 0) {
            return true; // Update every loop
        }

        long currentTime = System.currentTimeMillis();
        return (currentTime - lastUpdateTimestamp) >= minUpdateIntervalMs;
    }

    /**
     * Closes the logger and flushes any remaining data to disk.
     *
     * <p>Should be called when the OpMode stops or when switching between
     * autonomous and teleop.</p>
     */
    public void close() {
        if (csvEnabled && csvLogger != null) {
            csvLogger.close();
        }
    }

    /**
     * Gets the underlying CSV DataLogger for direct access if needed.
     *
     * @return the DataLogger instance, or null if CSV logging is disabled
     */
    public DataLogger getCsvLogger() {
        return csvLogger;
    }

    /**
     * Checks if CSV logging is enabled.
     *
     * @return true if CSV logging is enabled
     */
    public boolean isCsvEnabled() {
        return csvEnabled;
    }

    /**
     * Checks if dashboard logging is enabled.
     *
     * @return true if dashboard logging is enabled
     */
    public boolean isDashboardEnabled() {
        return dashboardEnabled;
    }
}
