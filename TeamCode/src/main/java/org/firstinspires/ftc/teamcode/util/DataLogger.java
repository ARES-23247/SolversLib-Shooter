package org.firstinspires.ftc.teamcode.util;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.LinkedHashMap;
import java.util.Map;

/**
 * CSV-based telemetry logging utility for recording robot data to disk.
 *
 * <p>This utility provides robust data logging for post-match analysis, PIDF tuning,
 * and debugging. It writes timestamped CSV files to the /FIRST/logs/ directory on
 * the Robot Controller phone.</p>
 *
 * <h3>Key Features:</h3>
 * <ul>
 *   <li><b>Automatic File Management:</b> Creates folders, generates timestamps, and handles
 *       file I/O errors gracefully</li>
 *   <li><b>Crash Recovery:</b> Flushes data to disk every update to prevent data loss if
 *       the Robot Controller app crashes</li>
 *   <li><b>Flexible Data Model:</b> Add any key-value pairs; headers are auto-generated
 *       from first call</li>
 *   <li><b>Data Retention:</b> Preserves data between updates, allowing sparse field
 *       updates (fields not updated retain their previous values)</li>
 * </ul>
 *
 * <h3>Usage Pattern:</h3>
 * <pre>
 * // Create logger (typically in Robot.init())
 * DataLogger logger = new DataLogger("match-teleop");
 *
 * // In subsystem periodic() or command execute():
 * logger.addData("Robot X", pose.getX());
 * logger.addData("Robot Y", pose.getY());
 * logger.addData("Heading", pose.getHeading());
 * logger.update();  // Write line to CSV and flush to disk
 * </pre>
 *
 * <h3>CSV Format:</h3>
 * <p>Each update() call writes one line to the CSV. The first line contains headers,
 * subsequent lines contain data:</p>
 * <pre>
 * Robot X,Robot Y,Heading
 * 0.0,0.0,0.0
 * 1.2,0.5,0.1
 * 2.5,1.0,0.2
 * ...
 * </pre>
 *
 * <h3>File Naming:</h3>
 * <p>Files are named with the pattern: <code>[filename]-[timestamp].csv</code></p>
 * <ul>
 *   <li><b>filename:</b> Provided by user (e.g., "match-teleop")</li>
 *   <li><b>timestamp:</b> Unix timestamp in milliseconds (e.g., "1699123456789")</li>
 *   <li><b>location:</b> /sdcard/FIRST/logs/ on the Robot Controller</li>
 * </ul>
 *
 * <h3>Data Retention Strategy:</h3>
 * <p>By default, data is NOT cleared after each update(). This allows fields to be
 * updated sparsely (e.g., vision data only updates when tags are visible) while
 * maintaining the last known value. To clear data each update, uncomment the
 * {@code data.clear()} line in update().</p>
 *
 * <h3>Error Handling:</h3>
 * <p>All I/O errors are caught and printed to stacktrace. The logger will silently
 * fail if:</p>
 * <ul>
 *   <li>Storage is full</li>
 *   <li>Folder creation fails</li>
 *   <li>File write permissions are denied</li>
 * </ul>
 *
 * <h3>Performance Considerations:</h3>
 * <ul>
 *   <li><b>Flush on Every Update:</b> Ensures crash recovery but adds I/O overhead.
 *       Consider flushing every N updates if performance is critical.</li>
 *   <li><b>String Conversion:</b> All values are converted to strings via {@code String.valueOf()}.
 *       Consider formatting for floating-point values if needed.</li>
 *   <li><b>LinkedHashMap:</b> Maintains insertion order for consistent column ordering</li>
 * </ul>
 *
 * @see java.io.FileWriter
 * @see java.util.LinkedHashMap
 */
public class DataLogger {

    /**
     * File writer for writing CSV data to disk.
     * Initialized in append mode to allow continuous writing.
     */
    private FileWriter writer;

    /**
     * Data storage for the current update line.
     * LinkedHashMap maintains insertion order for consistent CSV column ordering.
     * Maps field names to their string values.
     */
    private final LinkedHashMap<String, String> data = new LinkedHashMap<>();

    /**
     * Flag indicating whether CSV headers have been written.
     * Headers are written on the first call to update().
     */
    private boolean headersWritten = false;

    /**
     * Full file path for the CSV log file.
     * Format: /sdcard/FIRST/logs/[filename]-[timestamp].csv
     */
    private String filePath;

    /**
     * Creates a new DataLogger with the specified base filename.
     *
     * <p>This constructor:</p>
     * <ol>
     *   <li>Creates the /sdcard/FIRST/logs/ directory if it doesn't exist</li>
     *   <li>Generates a timestamped filename using the current time in milliseconds</li>
     *   <li>Opens a FileWriter in append mode for continuous writing</li>
     * </ol>
     *
     * <p><b>Note:</b> The file is created but no data is written until the first call to
     * {@link #update()}. This allows headers to be auto-generated from the first data
     * added.</p>
     *
     * @param filename the base filename (without extension) for the log file.
     *                 Example: "match-teleop" creates "match-teleop-1699123456789.csv"
     * @throws RuntimeException if file creation fails (exception is printed to stacktrace)
     */
    public DataLogger(String filename) {
        File folder = new File(AppUtil.FIRST_FOLDER.getAbsolutePath(), "logs");
        if (!folder.exists()) {
            folder.mkdirs();
        }
        
        filePath = folder.getAbsolutePath() + "/" + filename + "-" + System.currentTimeMillis() + ".csv";
        
        try {
            writer = new FileWriter(filePath, true);
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    /**
     * Adds or updates a data field for the current log entry.
     *
     * <p>This method stores a key-value pair that will be written to the CSV on the next
     * call to {@link #update()}. If the key already exists, the value is overwritten.</p>
     *
     * <h3>Usage Pattern:</h3>
     * <pre>
     * // Add multiple fields before calling update()
     * logger.addData("Robot X", pose.getX());
     * logger.addData("Robot Y", pose.getY());
     * logger.addData("Heading", pose.getHeading());
     * logger.addData("Match Time", timer.milliseconds());
     * logger.update();  // Writes all fields to CSV
     * </pre>
     *
     * <h3>Data Retention:</h3>
     * <p>Fields persist between update() calls unless {@code data.clear()} is uncommented.
     * This means:</p>
     * <ul>
     *   <li>Fields added in previous updates remain unless overwritten</li>
     *   <li>Sparse updates are allowed (e.g., only update vision data when tags visible)</li>
     *   <li>All fields are written to every CSV line (even if unchanged)</li>
     * </ul>
     *
     * <h3>Type Handling:</h3>
     * <p>Any object can be logged; it will be converted to a string via
     * {@code String.valueOf(value)}. For numeric values, consider formatting:</p>
     * <pre>
     * logger.addData("Pose X", String.format("%.2f", pose.getX()));
     * </pre>
     *
     * @param key the field name (will become the CSV column header). Should be consistent
     *            across updates to maintain column structure.
     * @param value the value to log (any object, converted to string)
     * @see #update()
     */
    public void addData(String key, Object value) {
        data.put(key, String.valueOf(value));
    }

    /**
     * Writes the current data to the CSV file and flushes to disk.
     *
     * <p>This method performs the following operations:</p>
     * <ol>
     *   <li><b>Write Headers (first call only):</b> If this is the first update, write CSV
     *       headers using the current data keys as column names</li>
     *   <li><b>Write Data Line:</b> Write all current data values as a CSV line</li>
     *   <li><b>Flush to Disk:</b> Immediately flush the file buffer to prevent data loss if
     *       the Robot Controller crashes</li>
     * </ol>
     *
     * <h3>CSV Format:</h3>
     * <p>Data is written in standard CSV format:</p>
     * <ul>
     *   <li>Columns separated by commas</li>
     *   <li>One line per update() call</li>
     *   <li>First line contains headers (keys)</li>
     *   <li>Subsequent lines contain values</li>
     * </ul>
     *
     * <h3>Data Retention:</h3>
     * <p>By default, the data map is NOT cleared after each update. This allows fields
     * to be updated sparsely while maintaining their last known value. To clear data each
     * update (requiring all fields to be populated every call), uncomment:</p>
     * <pre>
     * data.clear();
     * </pre>
     *
     * <h3>Performance Note:</h3>
     * <p>Flushing every update ensures crash recovery but adds I/O overhead. For high-frequency
     * logging (>50Hz), consider flushing every N updates to improve performance.</p>
     *
     * <h3>Error Handling:</h3>
     * <p>IOExceptions are caught and printed to stacktrace. The logger will continue functioning
     * but data may be lost if writes fail.</p>
     *
     * <p><b>Important:</b> This method should be called once per loop iteration, typically at
     * the end of the update loop after all data has been added.</p>
     *
     * @see #addData(String, Object)
     * @see #close()
     */
    public void update() {
        if (writer == null) return;
        
        try {
            if (!headersWritten) {
                // Write CSV Headers for the very first execution row
                StringBuilder headerLine = new StringBuilder();
                for (String key : data.keySet()) {
                    headerLine.append(key).append(",");
                }
                writer.append(headerLine.toString()).append("\n");
                headersWritten = true;
            }

            // Write CSV Values
            StringBuilder dataLine = new StringBuilder();
            for (String value : data.values()) {
                dataLine.append(value).append(",");
            }
            writer.append(dataLine.toString()).append("\n");
            
            // Forces write to disk to prevent data loss if the robot controller crashes
            writer.flush();
        } catch (IOException e) {
            e.printStackTrace();
        }
        
        // Un-comment to clear map each tick. Warning: This means ALL subsystems MUST push their data consistently every loop,
        // or the headers/columns won't perfectly align frame-by-frame. Retaining the data allows fields that update slowly to carry over.
        // data.clear(); 
    }

    /**
     * Closes the file writer and flushes any remaining data to disk.
     *
     * <p>This method should be called when logging is complete (typically at the end of
     * the match or when the OpMode is terminated). It ensures all buffered data is written
     * to the CSV file and resources are properly released.</p>
     *
     * <h3>When to Call:</h3>
     * <ul>
     *   <li>At the end of an OpMode (in the stop() method)</li>
     *   <li>When switching between autonomous and teleop (if using separate loggers)</li>
     *   <li>When the Robot Controller app is shutting down</li>
     * </ul>
     *
     * <h3>Automatic Cleanup:</h3>
     * <p>If close() is not called explicitly, the FileWriter will be closed when the
     * DataLogger object is garbage collected. However, relying on garbage collection is
     * not recommended as it may leave buffered data unwritten.</p>
     *
     * <h3>Safety:</h3>
     * <p>This method is null-safe and will not throw exceptions. If the writer is null or
     * closing fails, the exception is printed to stacktrace and execution continues.</p>
     *
     * <h3>Best Practice:</h3>
     * <pre>
     * // In OpMode:
     * DataLogger logger;
     *
     * public void init() {
     *     logger = new DataLogger("match-auto");
     * }
     *
     * public void loop() {
     *     logger.addData("...", ...);
     *     logger.update();
     * }
     *
     * public void stop() {
     *     logger.close();  // Always close when done
     * }
     * </pre>
     *
     * @see #update()
     * @see java.io.FileWriter#close()
     */
    public void close() {
        if (writer != null) {
            try {
                writer.close();
            } catch (IOException e) {
                e.printStackTrace();
            }
        }
    }
}
