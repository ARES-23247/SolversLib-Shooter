# Limelight3A Object Detection Guide

## 🎯 What Can Limelight3A Detect?

The Limelight3A can detect:
- **AprilTags** (Robust pose estimation - already in your code!)
- **Colored Objects** (Game pieces, scoring elements)
- **Shapes** (Circles, squares, custom contours)
- **Barcodes/QR Codes**
- **Retroreflective Tape** (Legacy support)

---

## 🚀 Object Detection Quick Start

### **Step 1: Access Limelight Web Interface**

1. Connect to Limelight WiFi: `Limelight3A-XXXX`
2. Go to: `http://10.0.0.1` or `http://192.168.1.11`
3. Password: `limelight` (if prompted)

### **Step 2: Configure Detection Pipeline**

1. Navigate to **"Pipeline"** tab
2. Choose detection type:
   - **AprilTag** - For pose estimation (you have this!)
   - **Color** - For colored game pieces
   - **Shape** - For geometric shapes

---

## 🎨 Color Detection (Game Pieces)

### **How It Works:**

Limelight uses HSV color filtering:
1. **Hue** (0-180): Color type (red, blue, yellow, etc.)
2. **Saturation** (0-255): Color intensity
3. **Value** (0-255): Brightness

### **Pipeline Setup:**

1. Go to **Pipeline** → **Calibration** tab
2. Point camera at game piece
3. Adjust HSV sliders until only the game piece is highlighted
4. Set **Contour Mode** to filter by:
   - Area (minimum/maximum size)
   - Fullness (how filled the shape is)
   - Aspect ratio (width/height ratio)

---

## 📦 Example: Detecting Game Pieces

Let's say you're detecting **yellow pixels** (CenterStage game piece):

### **In Limelight Web Interface:**

```
Pipeline Settings:
├── Input: Camera 0
├── Exposure: 20 (adjust for lighting)
├── Red Color: OFF
├── Green Color: OFF
├── Blue Color: OFF
└── Yellow Color: ON (or use custom HSV)

HSV Ranges (Yellow):
├── Hue: 20-35 (yellow range)
├── Saturation: 150-255 (vibrant colors)
└── Value: 100-255 (bright enough)

Contour Filters:
├── Min Area: 200 pixels (noise rejection)
├── Max Area: 50,000 pixels (maximum size)
├── Fullness: 20-100% (allow partially visible)
└── Aspect Ratio: 0.5-2.0 (rectangular pieces)

Output:
├── Crosshair: ON (helps with targeting)
└── Snap to Largest Contour: ON (track main piece)
```

---

## 💻 Code Examples

### **Example 1: Detect Yellow Game Piece (Color)**

```java
package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.hardware.Limelight3A;

/**
 * Yellow Pixel Detector using Limelight3A color detection.
 */
public class PixelDetector {

    private final Limelight3A limelight;
    private boolean detected = false;
    private double tx = 0;  // Horizontal offset (degrees)
    private double ty = 0;  // Vertical offset (degrees)
    private double ta = 0;  // Target area (0-100%)

    public PixelDetector(HardwareMap hwMap) {
        limelight = hwMap.get(Limelight3A.class, "limelight");

        // Switch to pipeline 1 (color detection)
        // Assumes you've configured pipeline 1 for yellow detection
        limelight.pipelineSwitch(1);
        limelight.start();
    }

    /**
     * Updates detection data.
     * Call this every loop iteration.
     */
    public void update() {
        // Get latest detection results
        Limelight3A.RawResult result = limelight.getRawResult();

        detected = result.isValid();
        if (detected) {
            tx = result.tx;  // Horizontal angle to target
            ty = result.ty;  // Vertical angle to target
            ta = result.ta;  // Target area (percentage)
        }
    }

    /**
     * Checks if a game piece is detected.
     */
    public boolean isDetected() {
        return detected;
    }

    /**
     * Gets horizontal offset to target.
     * @return Degrees (negative = left, positive = right)
     */
    public double getHorizontalOffset() {
        return tx;
    }

    /**
     * Gets vertical offset to target.
     * @return Degrees (negative = up, positive = down)
     */
    public double getVerticalOffset() {
        return ty;
    }

    /**
     * Gets target size (larger = closer).
     * @return Area percentage (0-100)
     */
    public double getTargetArea() {
        return ta;
    }

    /**
     * Calculates distance to target (requires calibration).
     * @return Estimated distance in inches
     */
    public double getEstimatedDistance() {
        // This formula requires calibration for your camera height/angle
        // Distance = (targetHeight - cameraHeight) / tan(ty + cameraAngle)
        double cameraHeight = 8.0;      // inches
        double cameraAngle = 20.0;       // degrees
        double targetHeight = 2.0;       // inches (pixel height)

        double tyRadians = Math.toRadians(ty + cameraAngle);
        double distance = (targetHeight - cameraHeight) / Math.tan(tyRadians);

        return distance;
    }

    /**
     * Gets strafe adjustment to center on target.
     * @return -1 to 1 (negative = strafe left, positive = strafe right)
     */
    public double getStrafeAdjustment() {
        // Proportional control based on horizontal offset
        // Adjust 0.02 for sensitivity
        return -tx * 0.02;
    }
}
```

---

### **Example 2: Multiple Detection Pipelines**

```java
/**
 * Multi-Detection System using Limelight3A.
 *
 * Pipeline 0: AprilTag (pose estimation)
 * Pipeline 1: Yellow pixels (game pieces)
 * Pipeline 2: Red elements (team marker)
 * Pipeline 3: Blue elements (opponent)
 */
public class MultiDetector {

    private final Limelight3A limelight;
    private int currentPipeline = 0;

    public enum DetectionMode {
        APRILTAG(0),
        YELLOW_PIXEL(1),
        RED_ELEMENT(2),
        BLUE_ELEMENT(3);

        final int pipeline;
        DetectionMode(int pipeline) {
            this.pipeline = pipeline;
        }
    }

    public MultiDetector(HardwareMap hwMap) {
        limelight = hwMap.get(Limelight3A.class, "limelight");
        limelight.start();
    }

    /**
     * Switches detection mode.
     */
    public void setMode(DetectionMode mode) {
        if (currentPipeline != mode.pipeline) {
            limelight.pipelineSwitch(mode.pipeline);
            currentPipeline = mode.pipeline;

            // Small delay for pipeline to switch
            try {
                Thread.sleep(50);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }
    }

    /**
     * Detects yellow pixels for game pieces.
     */
    public boolean detectYellowPixel() {
        setMode(DetectionMode.YELLOW_PIXEL);

        Limelight3A.RawResult result = limelight.getRawResult();
        return result.isValid();
    }

    /**
     * Detects AprilTags for localization.
     */
    public boolean detectAprilTag() {
        setMode(DetectionMode.APRILTAG);

        Limelight3A.RawResult result = limelight.getRawResult();
        return result.isValid();
    }
}
```

---

### **Example 3: Auto-Alignment to Game Piece**

```java
/**
 * Auto-Align to Yellow Pixel using Limelight.
 */
public class PixelAligner {

    private final PixelDetector detector;
    private final Drive drive;

    // Alignment parameters
    private static final double ALIGNMENT_TOLERANCE = 1.0;  // degrees
    private static final double MAX_STRAFE_SPEED = 0.3;
    private static final double STRAFE_GAIN = 0.02;

    public PixelAligner(PixelDetector detector, Drive drive) {
        this.detector = detector;
        this.drive = drive;
    }

    /**
     * Aligns robot to game piece.
     * @return true if aligned, false if lost target
     */
    public boolean alignToPiece() {
        detector.update();

        if (!detector.isDetected()) {
            // Lost target - stop!
            drive.drive(0, 0, 0);
            return false;
        }

        double tx = detector.getHorizontalOffset();

        // Check if aligned
        if (Math.abs(tx) < ALIGNMENT_TOLERANCE) {
            drive.drive(0, 0, 0);
            return true;
        }

        // Calculate strafe speed (proportional control)
        double strafe = -tx * STRAFE_GAIN;

        // Clamp to maximum speed
        strafe = Math.max(-MAX_STRAFE_SPEED, Math.min(MAX_STRAFE_SPEED, strafe));

        // Drive to align (no forward/backward, just strafe)
        drive.drive(0, strafe, 0);

        return false;
    }

    /**
     * Drives toward game piece while maintaining alignment.
     * @param forwardSpeed Forward speed (0-1)
     * @return true if at piece, false if still approaching
     */
    public boolean driveToPiece(double forwardSpeed) {
        detector.update();

        if (!detector.isDetected()) {
            drive.drive(0, 0, 0);
            return false;
        }

        double tx = detector.getHorizontalOffset();
        double ta = detector.getTargetArea();

        // Check if at piece (target area is large enough)
        if (ta > 15.0) {  // Adjust threshold based on testing
            drive.drive(0, 0, 0);
            return true;
        }

        // Align while driving forward
        double strafe = -tx * STRAFE_GAIN;
        strafe = Math.max(-MAX_STRAFE_SPEED, Math.min(MAX_STRAFE_SPEED, strafe));

        drive.drive(forwardSpeed, strafe, 0);

        return false;
    }
}
```

---

## 🔧 Pipeline Tuning Tips

### **1. Lighting Conditions**

```java
// Adjust camera exposure for your venue
limelight.setExposure(20);  // Lower = brighter, but more noise
                           // Higher = darker, but less noise
                           // Range: 0-100
```

**Recommended:** Start at 20, adjust for venue lighting

---

### **2. Color Range Selection**

**In Limelight web interface:**
- Use **"HSV Color Picker"** tool
- Click on the game piece in camera view
- Limelight auto-selects the best HSV range
- Fine-tune manually if needed

**Pro Tip:** Calibrate at the competition venue (lighting varies!)

---

### **3. Contour Filtering**

```
Min Area: 200 pixels     → Removes small noise
Max Area: 50,000 pixels  → Removes far/background objects
Fullness: 20-100%       → Allows partially visible pieces
Aspect Ratio: 0.5-2.0   → Filters by shape (width/height)
```

**For Round Objects (balls):**
- Fullness: 70-100%
- Aspect Ratio: 0.8-1.2

**For Rectangular Objects (pixels):**
- Fullness: 40-100%
- Aspect Ratio: 0.6-1.5

---

### **4. Calibration**

**Distance Calibration:**
1. Place game piece at known distances (12", 24", 36", 48")
2. Record `ty` (vertical angle) at each distance
3. Plot distance vs. ty
4. Fit formula: `distance = A / tan(ty + B)`

**Code for calibration:**

```java
public void calibrateDistance() {
    detector.update();

    telemetry.addData("ty", detector.getVerticalOffset());
    telemetry.addData("ta", detector.getTargetArea());
    telemetry.addData("tx", detector.getHorizontalOffset());

    // Move robot to known distances and record ty values
    // Then fit a curve to the data
}
```

---

## 🎯 FTC-Specific Use Cases

### **Use Case 1: Autonomous Game Piece Intake**

```java
@Override
public void initialize() {
    schedule(
        new FollowPathCommand(drive.follower, toGamePiece),
        new RunCommand(() -> {
            if (pixelDetector.isDetected()) {
                aligner.driveToPiece(0.3);  // Drive forward at 30%
            } else {
                drive.drive(0.3, 0, 0);  // Keep looking
            }
        }).withTimeout(5.0),
        new InstantCommand(() -> intake.run()),
        new WaitCommand(1000),
        new InstantCommand(() -> intake.stop())
    );
}
```

---

### **Use Case 2: TeleOp Auto-Align**

```java
@Override
public void run() {
    // Driver controls
    double forward = -gamepad1.left_stick_y;
    double lateral = gamepad1.left_stick_x;
    double turn = gamepad1.right_stick_x;

    // Auto-align when right bumper held
    if (gamepad1.right_bumper) {
        pixelDetector.update();

        if (pixelDetector.isDetected()) {
            // Auto-align strafe
            double adjustment = pixelDetector.getStrafeAdjustment();
            lateral = adjustment;

            telemetry.addData("Auto-Align", "ON");
            telemetry.addData("Offset", pixelDetector.getHorizontalOffset());
        } else {
            telemetry.addData("Auto-Align", "NO TARGET");
        }
    }

    drive.setTeleOpDrive(forward, lateral, turn, true);
}
```

---

### **Use Case 3: Detect Team Elements**

```java
/**
 * Detects red vs blue team marker.
 */
public class TeamDetector {

    private final Limelight3A limelight;

    public enum TeamColor {
        RED(2),    // Pipeline 2 for red
        BLUE(3),   // Pipeline 3 for blue
        UNKNOWN(-1);

        final int pipeline;
        TeamColor(int pipeline) {
            this.pipeline = pipeline;
        }
    }

    public TeamColor detectTeamMarker() {
        // Try red detection
        limelight.pipelineSwitch(2);
        Limelight3A.RawResult redResult = limelight.getRawResult();
        if (redResult.isValid()) {
            return TeamColor.RED;
        }

        // Try blue detection
        limelight.pipelineSwitch(3);
        Limelight3A.RawResult blueResult = limelight.getRawResult();
        if (blueResult.isValid()) {
            return TeamColor.BLUE;
        }

        return TeamColor.UNKNOWN;
    }
}
```

---

## 📊 Telemetry & Debugging

```java
public void updateTelemetry() {
    detector.update();

    telemetry.addData("Detected", detector.isDetected());

    if (detector.isDetected()) {
        telemetry.addData("Horizontal", detector.getHorizontalOffset() + "°");
        telemetry.addData("Vertical", detector.getVerticalOffset() + "°");
        telemetry.addData("Area", detector.getTargetArea() + "%");
        telemetry.addData("Distance", detector.getEstimatedDistance() + "in");

        // Visual alignment indicator
        if (Math.abs(detector.getHorizontalOffset()) < 1.0) {
            telemetry.addData("Alignment", "✓ ALIGNED");
        } else {
            telemetry.addData("Alignment", (detector.getHorizontalOffset() > 0) ? "→ TURN LEFT" : "← TURN RIGHT");
        }
    } else {
        telemetry.addData("Status", "SEARCHING...");
    }
}
```

---

## ⚡ Performance Tips

1. **Poll Rate:** Set to 50-60Hz (you have this!)
2. **Pipeline Switching:** Minimize switches (adds ~50ms delay)
3. **Snap to Largest:** Always track the biggest contour
4. **Crosshair:** Enable for debugging, disable for competition

```java
// Optimize Limelight settings
limelight.setPollRateHz(50);  // Already in your code!
limelight.pipelineSwitch(1);  // Set once, don't keep switching
```

---

## 🔧 Common Issues & Solutions

| Issue | Solution |
|-------|----------|
| **No detection** | Check HSV range, adjust exposure |
| **False positives** | Increase min area, adjust saturation |
| **Lost target** | Reduce max area, check lighting |
| **Slow detection** | Lower poll rate, simplify filters |
| **Wrong distance** | Calibrate ty formula for your setup |

---

## 📝 Next Steps

1. **Set up pipeline** in Limelight web interface
2. **Test with VL53L5CXSample** approach (create similar test OpMode)
3. **Calibrate** HSV ranges at competition venue
4. **Integrate** into your autonomous/teleop code
5. **Tune** alignment gains for smooth performance

Want me to create a complete working example for your specific game piece?
