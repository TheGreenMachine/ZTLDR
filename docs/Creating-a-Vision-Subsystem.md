# Creating a Vision Subsystem with PhotonVision

This tutorial guides you through creating a vision subsystem that uses PhotonVision and multiple cameras to detect AprilTags and provide field-relative positioning for your robot.

**Prerequisites:** You have completed [Creating a Subsystem](Subsystem-Creation.md) and understand the Superstructure pattern from [Why We Use a Superstructure](Why-We-Use-a-Superstructure.md).

---

## Understanding Vision in FRC

Before we write code, let's understand what the vision system does and why we need it.

### Why AprilTag Vision?

Your swerve drivetrain can track where it *thinks* it is using wheel encoders and gyro (this is called **odometry**). But wheels slip, gyros drift, and small errors accumulate over time. **AprilTag vision provides absolute positioning** - the robot looks at known AprilTag positions on the field and calculates its exact field-relative pose.

### Why Multiple Cameras?

A single camera can only see tags in front of it. With cameras on all four sides:
- The robot can see tags no matter which direction it's facing
- Multiple cameras can see the same tag, improving accuracy through triangulation
- If one camera is blocked or has its view obscured, the others continue working
- You get reliable pose estimates throughout the entire match

### The Vision Processing Pipeline

Here's what happens every robot loop:
1. PhotonVision coprocessor captures images from each camera
2. Each camera's image is processed to detect AprilTags
3. For visible tags, PhotonVision calculates where the camera is relative to each tag
4. Using the known robot-to-camera transform, it calculates robot pose
5. With multiple tags visible, it uses multi-tag pose estimation for better accuracy
6. Our code collects all camera estimates and selects the best one
7. The best pose estimate is fed into the drivetrain's odometry as a "correction"

**Key insight:** The vision system doesn't *replace* odometry - it **fuses** with it using a Kalman filter. The drivetrain combines wheel odometry (fast, continuous, but drifting) with vision (absolute, but slower and sometimes unavailable) to get the best of both.

---

## Step 1: Think About Your Cameras

Before writing code, you need to decide:

### Camera Location and Naming
Each camera needs a unique name that matches the PhotonVision configuration. Conventionally, we use descriptive names like:
- `frontCamera`
- `rightCamera`
- `backCamera`
- `leftCamera`

### Robot-to-Camera Transform
This is **critical** - you must measure precisely where each camera is mounted on the robot relative to the center of the robot. The transform includes:

- **Translation (x, y, z):** How far forward/back, left/right, and up/down is the camera lens from robot center?
- **Rotation (roll, pitch, yaw):** How is the camera oriented?
  - *Roll:* Usually 0 (camera isn't tilted sideways)
  - *Pitch:* Usually pitched down 10-20 degrees to see the field
  - *Yaw:* 0 for front, -90° for right, 180° for back, 90° for left

**Why this matters:** PhotonVision uses this transform to convert "camera sees tag at position X" to "robot is at position Y." Get this wrong and your pose estimates will be consistently wrong.

---

## Step 2: Create the Vision Subsystem File

Navigate to `season/subsystems` and create `Vision.java`.

### Subsystem Foundation

Start with the same foundation as any other subsystem:

```java
public class Vision extends SubsystemBase implements ITestableSubsystem {
    public static final String NAME = "vision";
    
    // Vision state management
    public enum VISION_STATE {
        OFF,
        ACTIVE
    }
    
    private VISION_STATE wantedState = VISION_STATE.ACTIVE;
    
    // TODO: Add camera fields here
    
    // TODO: Add pose estimate fields here
    
    public Vision() {
        // TODO: Initialize cameras
    }
    
    @Override
    public void periodic() {
        readFromHardware();
        applyState();
    }
    
    @Override
    public void readFromHardware() {
        // TODO: Collect pose estimates from cameras
    }
    
    private void applyState() {
        // TODO: Implement state-based behavior
    }
    
    public void setWantedState(VISION_STATE state) {
        this.wantedState = state;
    }
}
```

**Teaching point:** Notice this follows the exact same pattern as `Indexer.java` and `Shooter.java`. We define states, track the wanted state, and implement `readFromHardware()` and `applyState()`. Consistent patterns make the code easier to understand and maintain.

---

## Step 3: Model Your Cameras

Each camera is a complex object with several properties. Rather than having fields like `frontCamera`, `rightCameraName`, `rightCameraTransform`, etc., we should model a camera as its own entity.

### Create a Camera Configuration Enum

Create an enum to define each camera's properties:

```java
public enum CameraLocation {
    FRONT("frontCamera", new Transform3d(...)),
    RIGHT("rightCamera", new Transform3d(...)),
    BACK("backCamera", new Transform3d(...)),
    LEFT("leftCamera", new Transform3d(...));

    public final String cameraName;
    public final Transform3d robotToCamera;

    CameraLocation(String cameraName, Transform3d robotToCamera) {
        this.cameraName = cameraName;
        this.robotToCamera = robotToCamera;
    }
}
```

**Why an enum?** This "encapsulates" the camera configuration - the camera name and its physical mounting location on the robot are inseparable properties that should always travel together. Using an enum prevents mistakes like mixing up the front camera's name with the right camera's transform.

**The Transform3d constructor** takes:
1. A `Translation3d` (x, y, z position relative to robot center)
2. A `Rotation3d` (roll, pitch, yaw orientation)

Use meters for translation and radians for rotation. Positive X is forward, positive Y is left, positive Z is up.

---

## Step 4: Create a Camera Wrapper Class

Now we need a class that wraps the PhotonVision `PhotonCamera` and `PhotonPoseEstimator` for each physical camera.

### Designing the Camera Class

Think about what a camera needs to do:
1. Connect to PhotonVision using its name
2. Configure the pose estimator with the robot-to-camera transform
3. Provide a way to get the latest pose estimate
4. Track how many tags are visible (for quality assessment)

Create an inner class inside Vision:

```java
public static class Camera {
    private final PhotonCamera camera;
    private final PhotonPoseEstimator poseEstimator;
    private int lastVisibleTagCount = 0;

    public Camera(CameraLocation config, AprilTagFieldLayout fieldLayout) {
        // TODO: Initialize PhotonCamera
        // TODO: Initialize PhotonPoseEstimator
        // TODO: Configure pose strategy
    }

    public Optional<EstimatedRobotPose> getEstimatedPose() {
        // TODO: Query PhotonVision for latest result
        // TODO: Return pose estimate if available
    }

    public int getVisibleTagCount() {
        return lastVisibleTagCount;
    }
}
```

**Key concept: `Optional<EstimatedRobotPose>`**

Vision estimates aren't always available - maybe no tags are in view, or they're too far away, or the camera is disconnected. We use `Optional` to handle these cases gracefully. Callers check `if (estimate.isPresent())` rather than checking for null.

### Understanding PhotonPoseEstimator Strategies

The pose estimator has multiple "strategies" for calculating pose:

- **`MULTI_TAG_PNP_ON_COPROCESSOR`:** Uses all visible tags simultaneously for one pose estimate. Most accurate when multiple tags are visible.
- **`LOWEST_AMBIGUITY`:** Uses the single tag with the clearest view (lowest pose ambiguity). Best fallback when only one tag is visible.

Set the primary strategy to multi-tag, but provide a fallback:

```java
poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
```

**Why multi-tag is better:** When multiple tags are visible, the system uses the corners of ALL tags to solve for position. This geometric constraint improves accuracy and reduces noise compared to using a single tag.

---

## Step 5: Initialize All Cameras

In the Vision subsystem constructor, initialize one Camera instance for each location:

```java
private final Camera[] cameras;

public Vision() {
    // Load the AprilTag field layout (tells us where all tags are on the field)
    AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(
        AprilTagFields.k2024Crescendo  // or k2025Reefscape
    );

    // Initialize array to hold all cameras
    cameras = new Camera[CameraLocation.values().length];
    
    // Create a Camera for each location
    for (int i = 0; i < CameraLocation.values().length; i++) {
        cameras[i] = new Camera(CameraLocation.values()[i], fieldLayout);
    }
}
```

**What is AprilTagFieldLayout?**

FIRST provides a JSON file describing exactly where every AprilTag is located on the field (position and rotation). `AprilTagFieldLayout.loadField()` loads this official data. PhotonVision uses this to convert "I see tag #5" into "camera is 2.3 meters from the tag's known location."

---

## Step 6: Implement readFromHardware()

This method collects pose estimates from all cameras and selects the best one. Here's the thinking process:

### Algorithm Design

For each camera:
1. Ask it for a pose estimate
2. If it has one, note:
   - The estimated pose
   - How many tags were used to calculate it
   - The timestamp when the image was captured

After checking all cameras:
1. If we have NO estimates, report "no valid vision pose"
2. If we have ONE estimate, use it
3. If we have MULTIPLE estimates, pick the best one

**How do we choose "best"?**

More tags = better accuracy. So prefer estimates with higher tag counts. If tied on tag count, prefer more recent estimates (lower latency).

### Implementation Approach

```java
private Pose2d estimatedPose = new Pose2d();
private double lastTimestamp = 0;
private boolean hasValidPose = false;

@Override
public void readFromHardware() {
    if (wantedState == VISION_STATE.OFF) {
        hasValidPose = false;
        return;
    }

    VisionResult bestResult = null;
    
    for (Camera camera : cameras) {
        Optional<EstimatedRobotPose> estimate = camera.getEstimatedPose();
        
        if (estimate.isPresent()) {
            // Evaluate this estimate vs. our current best
            // TODO: Implement comparison logic
        }
    }

    if (bestResult != null) {
        estimatedPose = bestResult.pose;
        lastTimestamp = bestResult.timestamp;
        hasValidPose = true;
    } else {
        hasValidPose = false;
    }
}
```

**Create a VisionResult helper class** to track candidate estimates:

```java
private static class VisionResult {
    final Pose2d pose;
    final int tagCount;
    final double timestamp;
    final Matrix<N3, N1> stdDevs;

    // TODO: Constructor
}
```

**What are stdDevs (standard deviations)?**

These tell the drivetrain's Kalman filter "how much should we trust this pose estimate?" Lower values mean higher confidence.

- Single-tag estimates: Use higher values (less trust) because they're less accurate
- Multi-tag estimates: Use lower values (more trust)

PhotonLib provides mechanisms to adjust pose estimation confidence based on tag detection criteria [6].

---

## Step 7: Integrate with the Drivetrain

The vision subsystem needs to communicate pose estimates to the drivetrain. **But we should NOT have Vision directly call methods on the drivetrain** - that would violate the Superstructure pattern (see [Why We Use a Superstructure](Why-We-Use-a-Superstructure.md)).

Instead, the Superstructure will coordinate between Vision and drivetrain. But first, the Vision subsystem needs to provide "getter" methods for its data:

```java
public Pose2d getEstimatedPose() {
    return estimatedPose;
}

public double getLastTimestamp() {
    return lastTimestamp;
}

public boolean hasValidPose() {
    return hasValidPose;
}

public Matrix<N3, N1> getStdDevs() {
    // Return appropriate values based on current estimate
}
```

### Drivetrain Modifications

The drivetrain needs to accept vision measurements. This requires:

1. Adding a method to `IDrivetrain` interface:
```java
void addVisionMeasurement(Pose2d visionPose, double timestamp, 
                         Matrix<N3, N1> stdDevs);
```

2. Implementing it in the drivetrain implementation to fuse with existing odometry

**What does addVisionMeasurement do?**

WPILib's swerve drive pose estimator uses a Kalman filter. Every loop it:
1. Predicts pose based on wheel movements
2. Corrects based on any vision measurements provided

The Kalman filter automatically weights vision vs. odometry based on the stdDevs. High stdDevs for vision = "I don't trust vision much, mostly use odometry." Low stdDevs = "Vision is very accurate, trust it heavily."

---

## Step 8: Superstructure Coordination

The Superstructure coordinates when vision estimates should be applied.

### Add Vision SuperState

```java
public enum VisionSuperState {
    VISION_ON,    // Actively using vision for pose estimation
    VISION_OFF    // Vision disabled (for testing/debugging)
}

protected VisionSuperState visionSuperState = VisionSuperState.VISION_ON;
```

### Create State Setter and Command

```java
private void setVisionSuperState(VisionSuperState visionSuperState) {
    this.visionSuperState = visionSuperState;
}

public Command setVisionCommand(VisionSuperState visionSuperState) {
    return new InstantCommand(() -> setVisionSuperState(visionSuperState));
}
```

### Update applyStates()

```java
switch (visionSuperState) {
    case VISION_ON:
        vision.setWantedState(Vision.VISION_STATE.ACTIVE);
        break;
    case VISION_OFF:
        vision.setWantedState(Vision.VISION_STATE.OFF);
        break;
}

// If vision is active and has valid estimate, send to drivetrain
if (visionSuperState == VisionSuperState.VISION_ON && vision.hasValidPose()) {
    // Get the pose estimate
    Pose2d visionPose = vision.getEstimatedPose();
    double timestamp = vision.getLastTimestamp();
    Matrix<N3, N1> stdDevs = vision.getStdDevs();
    
    // Apply to drivetrain
    drivetrain.addVisionMeasurement(visionPose, timestamp, stdDevs);
}
```

**Why put this in the Superstructure rather than having Vision directly talk to drivetrain?**

- The Superstructure decides *when* vision should influence pose (e.g., maybe disable during climbing)
- We can add logic like "only use vision when stationary" or "blend vision differently during high-acceleration"
- Centralized coordination means we can see the whole picture in one place
- Subsystems remain decoupled and independently testable

---

## Step 9: Debugging and Visualization

Add visualization so you can see what the vision system is doing:

### SmartDashboard Outputs

```java
SmartDashboard.putString("Vision State", wantedState.toString());
SmartDashboard.putBoolean("Vision Has Pose", hasValidPose);
SmartDashboard.putString("Vision Est. Pose", estimatedPose.toString());
```

### Field2d Visualization

```java
private final Field2d visionField = new Field2d();

// In constructor:
SmartDashboard.putData("Vision Field", visionField);

// When pose is valid:
visionField.setRobotPose(estimatedPose);
```

This creates a mini field display on the dashboard showing where vision thinks the robot is. Compare it to the drivetrain's estimated pose to see how well they agree.

---

## Step 10: Configure Button Bindings

In RobotContainer, add a way to toggle vision on/off for debugging:

```java
// Maybe use the back button to momentarily disable vision
controller.back()
    .onTrue(superstructure.setVisionCommand(Superstructure.VisionSuperState.VISION_OFF))
    .onFalse(superstructure.setVisionCommand(Superstructure.VisionSuperState.VISION_ON));
```

Or use it to demonstrate vision vs. no-vision driving:
- Press back: vision turns off, rely only on odometry
- Release back: vision turns back on

---

## Step 11: Physical Camera Configuration

This happens on the PhotonVision coprocessor (Raspberry Pi, Orange Pi, etc.), not in robot code.

### For Each Camera:

1. **Set the name** to exactly match your enum (`frontCamera`, `rightCamera`, etc.)
2. **Calibrate the camera** using PhotonVision's calibration tool
   - Print the calibration chessboard pattern
   - Take multiple images from different angles
   - PhotonVision calculates lens distortion and field of view
3. **Configure the AprilTag pipeline:**
   - AprilTag family: tag16h5 (standard FRC)
   - Enable 3D mode
   - Set tag size (0.1651m for 2024, check your year's game manual)
   - Tune exposure/brightness for your field lighting
4. **Verify robot-to-camera transform** matches your code exactly

**Important:** Camera calibration must be done AFTER the camera is rigidly mounted on the robot. If you move the camera, you must recalibrate.

---

## Step 12: Tuning and Testing

### Simulation Testing

PhotonLib provides simulated vision system functionality [5]. You can test your code without physical cameras:

1. Create simulated camera properties (resolution, FPS, latency)
2. Define where AprilTags are in simulation
3. Verify your pose estimation logic works

This lets you test autonomous routines that rely on vision before the physical robot is ready.

### Real-World Tuning

**Standard Deviation Tuning:**

Start with conservative (high) values for single-tag and optimistic (low) values for multi-tag:
- Single-tag: [4, 4, 8] meters/radians
- Multi-tag: [0.5, 0.5, 1.0] meters/radians

Observe behavior:
- If the robot pose "jumps around" too much, increase stdDevs (trust vision less)
- If odometry drifts significantly before vision corrects it, decrease stdDevs (trust vision more)

**Latency Considerations:**

Vision has latency - the image was captured milliseconds ago, but processing takes time. The timestamp passed to `addVisionMeasurement()` tells the Kalman filter *when* the measurement was taken, allowing it to correctly fuse with odometry.

### Diagnostic Tips

- Check PhotonVision dashboard - are cameras detecting tags?
- Compare "Vision Field" pose to "Drivetrain Field" pose - they should generally agree
- Watch the "hasValidPose" boolean - false means no tags visible
- Check tag counts - higher is better for accuracy

---

## Integration Checklist

Before your vision system is ready:

- [ ] Camera names match between code and PhotonVision exactly
- [ ] Robot-to-camera transforms measured and entered correctly
- [ ] AprilTag field layout loaded (correct year)
- [ ] Multi-tag strategy configured with single-tag fallback
- [ ] Vision subsystem provides getters for pose, timestamp, stdDevs
- [ ] Drivetrain interface has addVisionMeasurement method
- [ ] Drivetrain implements the fusion correctly
- [ ] Superstructure coordinates vision state and applies measurements
- [ ] SmartDashboard shows diagnostic information
- [ ] Camera calibration completed on PhotonVision
- [ ] Standard deviations tuned for your robot and field
- [ ] Tested in simulation
- [ ] Tested on real field with real tags

---

## Architecture Review

Notice how this follows our architectural principles:

| Principle | How Vision Subsystem Follows It |
|-----------|--------------------------------|
| **Single Responsibility** | Vision only handles camera communication and pose estimation |
| **Separation of Concerns** | Vision calculates poses; drivetrain fuses with odometry; superstructure coordinates |
| **Loose Coupling** | Vision doesn't know about drivetrain; only provides data via getters |
| **Superstructure Pattern** | Central coordinator decides when to use vision data |
| **State Management** | VISION_STATE controls whether vision is active or disabled |

---

## Common Mistakes to Avoid

1. **Wrong camera transforms** - Measure carefully! Even small errors matter
2. **Name mismatches** - Camera names in code must exactly match PhotonVision
3. **Not handling Optional** - Always check `isPresent()` before using estimates
4. **Ignoring timestamps** - Pass the correct timestamp for proper Kalman filtering
5. **Using wrong stdDevs** - Too low = erratic pose; too high = slow corrections
6. **No fallback strategy** - Multi-tag fails with <2 tags visible; you need a fallback

---

## Further Reading

- [PhotonVision Documentation](https://docs.photonvision.org/)
- WPILib Pose Estimation
- Kalman Filters (for understanding the fusion math)
- Your game manual's AprilTag specifications

---

*Remember: A well-tuned vision system can give your robot centimeter-level field positioning accuracy. That translates directly to better autonomous routines and more reliable game piece manipulation. The effort to learn this subsystem pays off in competition performance!*
