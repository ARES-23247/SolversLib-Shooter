package org.firstinspires.ftc.teamcode.util;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.LinkedHashMap;
import java.util.Map;

public class DataLogger {
    private FileWriter writer;
    private final LinkedHashMap<String, String> data = new LinkedHashMap<>();
    private boolean headersWritten = false;
    private String filePath;

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

    public void addData(String key, Object value) {
        data.put(key, String.valueOf(value));
    }

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
