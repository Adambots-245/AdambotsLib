# PhotonVision

PhotonVision integration for AprilTag-based vision pose estimation with multi-camera support.

## Table of Contents

- [Overview](#overview)
- [Quick Start](#quick-start)
- [Features](#features)
- [Camera Management](#camera-management)
- [Core Methods](#core-methods)
- [Real-World Examples](#real-world-examples)
- [API Reference](#api-reference)
- [Best Practices](#best-practices)
- [Troubleshooting](#troubleshooting)

---

## Overview

The `PhotonVision` class provides comprehensive AprilTag vision functionality for FRC robots, including:

- **Multi-camera support** - Manage multiple PhotonVision cameras independently
- **Vision-corrected odometry** - Automatically update swerve drive pose estimates
- **Tag filtering** - Enable specific cameras for different game zones
- **Simulation support** - Full PhotonVision simulation integration
- **Distance and angle calculations** - Find distances, yaw angles, and closest tags
- **Camera state management** - Dynamically enable/disable cameras during match

This class is designed to integrate seamlessly with YAGSL's swerve drive and WPILib's pose estimation system.

**Based on:** Modified from [Ironclad 2024's Vision class](https://gitlab.com/ironclad_code/ironclad-2024/-/blob/master/src/main/java/frc/robot/vision/Vision.java)

---

## Quick Start

### 1. Create PhotonVision Instance

```java
public class SwerveSubsystem extends SubsystemBase {
    private final PhotonVision vision;

    public SwerveSubsystem() {
        // Initialize vision with pose supplier and field
        vision = new PhotonVision(this::getPose, swerveDrive.field);
    }
}
```

### 2. Update Pose Estimation

```java
@Override
public void periodic() {
    // Update odometry with vision measurements
    vision.updatePoseEstimation(swerveDrive);

    // Update Field2d with visible tags (optional)
    vision.updateVisionField();
}
```

### 3. Use Vision Methods

```java
// Get distance to tag
double distance = vision.getDistanceFromAprilTag(7);

// Get yaw angle to tag
Rotation2d yaw = vision.getYawToAprilTag(7);

// Check if tag is visible
boolean canSeeTag = vision.isTagVisible(7);

// Find closest tag
int closestTag = vision.getClosestVisibleTag();
```

---

## Features

### Multi-Camera Support

PhotonVision supports multiple cameras defined in the `Cameras` enum:

- **LEFT_CAM** - Front left camera for reef tags
- **RIGHT_CAM** - Front right camera for reef tags
- **CENTER_CAM** - Back camera for human player station tags

Each camera has:
- Independent pose estimation
- Configurable standard deviations
- AprilTag filtering
- Ambiguity-based quality scoring

### Camera Filtering

Cameras can be configured to only process specific AprilTag IDs:

```java
// Cameras enum with tag filtering
LEFT_CAM("Left",
    rotation, translation,
    singleTagStdDevs, multiTagStdDevs,
    getReefTagIDs()  // Only process reef tags
);

CENTER_CAM("Middle",
    rotation, translation,
    singleTagStdDevs, multiTagStdDevs,
    getHumanPlayerTagIDs()  // Only process human player tags
);
```

### Vision-Corrected Odometry

PhotonVision automatically updates the swerve drive's pose estimator with vision measurements:

- **Dynamic standard deviations** - Trust decreases with distance
- **Multi-tag fusion** - Higher confidence when seeing multiple tags
- **Ambiguity filtering** - Rejects low-quality measurements
- **Camera mode switching** - Use different cameras based on robot location

### Simulation Support

Full PhotonVision simulation integration:
- Simulated cameras with realistic properties
- Field visualization with detected tags
- Pose estimation visualization
- Same code works in simulation and on robot

---

## Camera Management

### Switch Camera Modes

```java
// Use only human player camera (CENTER_CAM)
vision.useHumanPlayerCamerasOnly();

// Use all cameras (LEFT_CAM, RIGHT_CAM)
vision.useAllCameras();

// Disable all vision
vision.disableAllCameras();

// Re-enable vision
vision.enableAllCameras();
```

### Check Camera State

```java
// Check if vision is disabled
boolean disabled = vision.areAllCamerasDisabled();

// Check if using human player cameras only
boolean humanPlayerMode = vision.isUsingHumanPlayerCamerasOnly();
```

### When to Switch Camera Modes

| Location | Camera Mode | Reason |
|----------|-------------|--------|
| **Reef (scoring)** | `useAllCameras()` | Front cameras see reef tags (6-11, 17-22) |
| **Human Player Station** | `useHumanPlayerCamerasOnly()` | Back camera sees HP tags (1-2, 4-5, 12-15) |
| **Vision unreliable** | `disableAllCameras()` | Temporarily disable vision, use wheel odometry |
| **Normal operation** | `useAllCameras()` | Use all available cameras |

---

## Core Methods

### Distance Methods

```java
// Get distance to specific tag
double distance = vision.getDistanceFromAprilTag(7);  // meters

// Get distance to closest visible tag
double closestDistance = vision.getDistanceToClosestTag();

// Get full transform (distance + angle) to tag
Transform2d transform = vision.getTransformToAprilTag(7);
double dx = transform.getX();
double dy = transform.getY();
```

### Angle Methods

```java
// Get yaw angle to tag (positive = left, negative = right)
Rotation2d yaw = vision.getYawToAprilTag(7);
double yawDegrees = yaw.getDegrees();

// Use in turret control
double turretSpeed = turretPID.calculate(yaw.getRadians(), 0);
```

### Tag Detection Methods

```java
// Check if specific tag is visible (any camera)
boolean canSee = vision.isTagVisible(7);

// Check if tag visible in specific camera
boolean inLeftCam = vision.isTagVisibleInCamera(7, Cameras.LEFT_CAM);

// Check if any of multiple tags visible
int visibleTag = vision.hasID(new int[]{6, 7, 8});  // Returns first found, or -1

// Get all detected tags
List<Integer> allTags = PhotonVision.getAllDetectedTags();
```

### Closest Tag Methods

```java
// Find closest visible tag
int closestTag = vision.getClosestVisibleTag();  // -1 if none visible

// Get distance to closest tag
double distance = vision.getDistanceToClosestTag();

// Check if any tag visible
boolean hasTarget = vision.hasTarget();
```

### Target Tracking

```java
// Get best target from specific camera
PhotonTrackedTarget bestTarget = vision.getBestTargetFromCamera(Cameras.LEFT_CAM);
if (bestTarget != null) {
    int tagID = bestTarget.getFiducialId();
    double ambiguity = bestTarget.getPoseAmbiguity();
}

// Get specific target from camera
PhotonTrackedTarget target = vision.getTargetFromId(7, Cameras.LEFT_CAM);
```

### Static Field Methods

```java
// Get pose of AprilTag on field
Pose2d tagPose = PhotonVision.getAprilTagPose(7, new Transform2d());

// Get tag IDs for specific field zones
int[] reefTags = PhotonVision.getReefTagIDs();  // {6,7,8,9,10,11,17,18,19,20,21,22}
int[] hpTags = PhotonVision.getHumanPlayerTagIDs();  // {1,2,4,5,12,13,14,15}
```

---

## Real-World Examples

### Example 1: Auto-Align to AprilTag

```java
public Command autoAlignToTag(int tagID, double tolerance) {
    return Commands.run(() -> {
        // Get yaw to target tag
        Rotation2d yawToTag = vision.getYawToAprilTag(tagID);
        if (yawToTag == null) {
            // Tag not visible, stop
            drive(new ChassisSpeeds(0, 0, 0));
            return;
        }

        // Calculate rotation speed with PID
        double rotationSpeed = headingPID.calculate(
            getHeading().getRadians(),
            (getHeading().plus(yawToTag)).getRadians()
        );

        // Drive with rotation
        drive(new ChassisSpeeds(0, 0, rotationSpeed));
    }, this)
    .until(() -> {
        Rotation2d yaw = vision.getYawToAprilTag(tagID);
        return yaw != null && Math.abs(yaw.getDegrees()) < tolerance;
    })
    .withName("AutoAlignToTag(" + tagID + ")");
}
```

### Example 2: Approach Closest Tag

```java
public Command approachClosestTag(double targetDistance) {
    return Commands.run(() -> {
        // Find closest tag
        int closestTag = vision.getClosestVisibleTag();
        if (closestTag == -1) {
            drive(new ChassisSpeeds(0, 0, 0));
            return;
        }

        // Get current distance and yaw
        double distance = vision.getDistanceFromAprilTag(closestTag);
        Rotation2d yaw = vision.getYawToAprilTag(closestTag);

        // Calculate speeds
        double forwardSpeed = distancePID.calculate(distance, targetDistance);
        double rotationSpeed = headingPID.calculate(yaw.getRadians(), 0);

        // Drive toward tag
        drive(new ChassisSpeeds(forwardSpeed, 0, rotationSpeed));
    }, this)
    .until(() -> {
        int tag = vision.getClosestVisibleTag();
        if (tag == -1) return false;
        double distance = vision.getDistanceFromAprilTag(tag);
        return Math.abs(distance - targetDistance) < 0.1;  // Within 10cm
    })
    .withName("ApproachClosestTag");
}
```

### Example 3: Camera Mode Switching

```java
public class RobotContainer {
    private final SwerveSubsystem swerve;

    private void configureButtonBindings() {
        // Switch to human player mode when backing up
        Buttons.XboxBack.onTrue(
            Commands.runOnce(() -> swerve.vision.useHumanPlayerCamerasOnly())
                .withName("SwitchToHPCameras")
        );

        // Switch back to reef cameras when driving forward
        Buttons.XboxStart.onTrue(
            Commands.runOnce(() -> swerve.vision.useAllCameras())
                .withName("SwitchToReefCameras")
        );

        // Emergency disable vision
        Buttons.XboxLeftBumper.and(Buttons.XboxRightBumper).onTrue(
            Commands.runOnce(() -> swerve.vision.disableAllCameras())
                .withName("DisableVision")
        );
    }
}
```

### Example 4: Tag-Based State Machine

```java
public class IntakeSubsystem extends SubsystemBase {
    enum State { STOWED, EXTENDED }
    private final PhotonVision vision;
    private final StateMachine<State, Void> sm;

    public Command autoExtendAtReef() {
        return Commands.runOnce(() -> {
            // Check if we see any reef tags
            int reefTag = vision.hasID(PhotonVision.getReefTagIDs());
            if (reefTag != -1) {
                // Check if close enough
                double distance = vision.getDistanceFromAprilTag(reefTag);
                if (distance < 3.0) {  // Within 3 meters
                    sm.to(State.EXTENDED).request();
                }
            }
        });
    }

    @Override
    public void periodic() {
        sm.periodic();

        // Auto-stow if we can't see any reef tags
        int reefTag = vision.hasID(PhotonVision.getReefTagIDs());
        if (reefTag == -1 && sm.getCurrentState() == State.EXTENDED) {
            sm.to(State.STOWED).request();
        }
    }
}
```

### Example 5: Vision-Based LED Feedback

```java
public class LEDSubsystem extends SubsystemBase {
    private final PhotonVision vision;

    @Override
    public void periodic() {
        // Show different colors based on vision state
        if (vision.areAllCamerasDisabled()) {
            setColor(Color.kRed);  // Red = vision disabled
        } else if (vision.hasTarget()) {
            int closest = vision.getClosestVisibleTag();
            double distance = vision.getDistanceFromAprilTag(closest);

            if (distance < 1.5) {
                setColor(Color.kGreen);  // Green = in range
            } else if (distance < 3.0) {
                setColor(Color.kOrange);  // Orange = close
            } else {
                setColor(Color.kYellow);  // Yellow = far
            }
        } else {
            setColor(Color.kBlue);  // Blue = no target
        }
    }
}
```

---

## API Reference

### Constructor

#### `PhotonVision(Supplier<Pose2d> currentPose, Field2d field)`

Create a new PhotonVision instance.

**Parameters:**
- `currentPose` - Supplier for current robot pose (e.g., `swerve::getPose`)
- `field` - Field2d object for visualization (e.g., `swerve.field`)

**Example:**
```java
vision = new PhotonVision(this::getPose, swerveDrive.field);
```

---

### Pose Estimation Methods

#### `updatePoseEstimation(SwerveDrive swerveDrive)`

Update the swerve drive's pose estimator with vision measurements from all cameras.

**Parameters:**
- `swerveDrive` - SwerveDrive instance to update

**Call Frequency:** Every robot periodic loop

**Example:**
```java
@Override
public void periodic() {
    vision.updatePoseEstimation(swerveDrive);
}
```

#### `updateVisionField()`

Update the Field2d object with currently visible AprilTags for visualization.

**Example:**
```java
vision.updateVisionField();  // Updates SmartDashboard field widget
```

---

### Distance Methods

#### `getDistanceFromAprilTag(int id)`

Get the distance from the robot to the specified AprilTag.

**Parameters:**
- `id` - AprilTag ID

**Returns:** Distance in meters, or `-1.0` if tag doesn't exist

**Example:**
```java
double distance = vision.getDistanceFromAprilTag(7);
if (distance > 0 && distance < 3.0) {
    // Within 3 meters of tag 7
}
```

#### `getDistanceToClosestTag()`

Get the distance to the closest visible AprilTag.

**Returns:** Distance in meters, or `-1.0` if no tags visible

**Example:**
```java
double distance = vision.getDistanceToClosestTag();
```

#### `getTransformToAprilTag(int id)`

Get the full transform (position and rotation) from robot to AprilTag.

**Parameters:**
- `id` - AprilTag ID

**Returns:** `Transform2d` from robot to tag

**Example:**
```java
Transform2d transform = vision.getTransformToAprilTag(7);
double xDistance = transform.getX();  // Forward/back
double yDistance = transform.getY();  // Left/right
```

---

### Angle Methods

#### `getYawToAprilTag(int id)`

Get the yaw angle (left/right) from the robot to the AprilTag.

**Parameters:**
- `id` - AprilTag ID

**Returns:** `Rotation2d` representing yaw, or `null` if tag doesn't exist
- Positive = tag is to the left
- Negative = tag is to the right

**Example:**
```java
Rotation2d yaw = vision.getYawToAprilTag(7);
if (yaw != null) {
    double turnSpeed = pid.calculate(yaw.getRadians(), 0);
}
```

---

### Tag Detection Methods

#### `isTagVisible(int tagID)`

Check if a specific AprilTag is visible in any camera.

**Parameters:**
- `tagID` - AprilTag ID to check

**Returns:** `true` if visible

**Example:**
```java
if (vision.isTagVisible(7)) {
    // Tag 7 is visible
}
```

#### `isTagVisibleInCamera(int tagID, Cameras camera)`

Check if a specific AprilTag is visible in a specific camera.

**Parameters:**
- `tagID` - AprilTag ID to check
- `camera` - Camera to check (`Cameras.LEFT_CAM`, etc.)

**Returns:** `true` if visible in specified camera

**Example:**
```java
if (vision.isTagVisibleInCamera(7, Cameras.LEFT_CAM)) {
    // Tag 7 visible in left camera
}
```

#### `hasID(int[] tagIDs)`

Check if any of the specified AprilTag IDs are visible.

**Parameters:**
- `tagIDs` - Array of tag IDs to check

**Returns:** First matching tag ID found, or `-1` if none visible

**Example:**
```java
// Check for any reef tags
int visibleReefTag = vision.hasID(PhotonVision.getReefTagIDs());
if (visibleReefTag != -1) {
    // Found reef tag
}
```

#### `hasTarget()`

Check if any AprilTag is currently visible.

**Returns:** `true` if any tag is visible

**Example:**
```java
if (vision.hasTarget()) {
    // At least one tag visible
}
```

#### `getAllDetectedTags()` (static)

Get a list of all currently detected AprilTag IDs across all cameras.

**Returns:** `List<Integer>` of detected tag IDs

**Example:**
```java
List<Integer> tags = PhotonVision.getAllDetectedTags();
for (int tagID : tags) {
    System.out.println("Seeing tag: " + tagID);
}
```

---

### Closest Tag Methods

#### `getClosestVisibleTag()`

Find the ID of the closest visible AprilTag.

**Returns:** Tag ID of closest tag, or `-1` if no tags visible

**Example:**
```java
int closestTag = vision.getClosestVisibleTag();
if (closestTag != -1) {
    double distance = vision.getDistanceFromAprilTag(closestTag);
}
```

---

### Camera Management Methods

#### `useHumanPlayerCamerasOnly()`

Switch to using only the human player camera (CENTER_CAM).
Disables front cameras (LEFT_CAM, RIGHT_CAM) from pose estimation.

**Use when:** Robot is at human player station

**Example:**
```java
vision.useHumanPlayerCamerasOnly();
```

#### `useAllCameras()`

Switch to using all cameras for pose estimation.
Enables front cameras and disables back camera.

**Use when:** Robot is on the field (reef, normal driving)

**Example:**
```java
vision.useAllCameras();
```

#### `disableAllCameras()`

Disable all cameras from contributing to pose estimation.

**Use when:** Vision is unreliable or causing issues

**Example:**
```java
vision.disableAllCameras();
```

#### `enableAllCameras()`

Enable all cameras (respects current filtering mode).

**Example:**
```java
vision.enableAllCameras();
```

#### `areAllCamerasDisabled()`

Check if vision is currently disabled.

**Returns:** `true` if all cameras are disabled

**Example:**
```java
if (vision.areAllCamerasDisabled()) {
    SmartDashboard.putString("Vision", "DISABLED");
}
```

#### `isUsingHumanPlayerCamerasOnly()`

Check if only human player cameras are active.

**Returns:** `true` if only CENTER_CAM is used

**Example:**
```java
if (vision.isUsingHumanPlayerCamerasOnly()) {
    SmartDashboard.putString("Camera Mode", "HP Station");
}
```

---

### Target Tracking Methods

#### `getTargetFromId(int id, Cameras camera)`

Get the tracked target for a specific tag from a specific camera.

**Parameters:**
- `id` - AprilTag ID
- `camera` - Camera to check

**Returns:** `PhotonTrackedTarget` or `null` if not found

**Example:**
```java
PhotonTrackedTarget target = vision.getTargetFromId(7, Cameras.LEFT_CAM);
if (target != null) {
    double ambiguity = target.getPoseAmbiguity();
}
```

#### `getBestTargetFromCamera(Cameras camera)`

Get the target with the lowest ambiguity from a specific camera.

**Parameters:**
- `camera` - Camera to check

**Returns:** `PhotonTrackedTarget` with lowest ambiguity, or `null`

**Example:**
```java
PhotonTrackedTarget best = vision.getBestTargetFromCamera(Cameras.LEFT_CAM);
```

---

### Static Field Methods

#### `getAprilTagPose(int aprilTag, Transform2d robotOffset)` (static)

Get the field pose of an AprilTag with an optional robot offset.

**Parameters:**
- `aprilTag` - AprilTag ID
- `robotOffset` - Transform to apply to tag pose (for robot positioning)

**Returns:** `Pose2d` of the tag (with offset applied)

**Throws:** `RuntimeException` if tag doesn't exist

**Example:**
```java
// Get tag pose with no offset
Pose2d tagPose = PhotonVision.getAprilTagPose(7, new Transform2d());

// Get pose 1m in front of tag
Transform2d offset = new Transform2d(new Translation2d(1.0, 0), new Rotation2d());
Pose2d approachPose = PhotonVision.getAprilTagPose(7, offset);
```

#### `getHumanPlayerTagIDs()` (static)

Get array of AprilTag IDs for human player stations (both alliances).

**Returns:** `int[]` - {1, 2, 4, 5, 12, 13, 14, 15}

**Example:**
```java
int[] hpTags = PhotonVision.getHumanPlayerTagIDs();
int visibleHP = vision.hasID(hpTags);
```

#### `getReefTagIDs()` (static)

Get array of AprilTag IDs for reefs (both alliances).

**Returns:** `int[]` - {6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22}

**Example:**
```java
int[] reefTags = PhotonVision.getReefTagIDs();
int visibleReef = vision.hasID(reefTags);
```

---

### Simulation Methods

#### `getVisionSim()`

Get the vision system simulator (only valid in simulation).

**Returns:** `VisionSystemSim` instance

**Example:**
```java
if (RobotBase.isSimulation()) {
    VisionSystemSim sim = vision.getVisionSim();
    Field2d debugField = sim.getDebugField();
}
```

---

## Best Practices

### 1. Camera Placement

**Front Cameras (LEFT/RIGHT):**
- Mount high for long-range detection
- Angle slightly upward (5-15°)
- Wide FOV to see multiple tags
- Position symmetrically for consistent readings

**Back Camera (CENTER):**
- Mount facing backward/upward
- Position to see human player station tags
- Higher mounting = better range

### 2. Standard Deviations

Adjust standard deviations based on testing:

```java
// Low standard deviations = more trust in vision
VecBuilder.fill(0.1, 0.1, 0.1)  // High confidence

// High standard deviations = less trust in vision
VecBuilder.fill(1.0, 1.0, 1.0)  // Low confidence

// Typical values (from code)
VecBuilder.fill(0.5, 0.5, 0.5)  // Single tag
VecBuilder.fill(0.5, 0.5, 1.0)  // Multi tag (less trust in rotation)
```

**Guidelines:**
- Single tag readings: Higher std devs
- Multi-tag readings: Lower std devs
- Close range: Lower std devs
- Far range: Higher std devs (handled automatically)

### 3. Camera Mode Switching

Switch camera modes based on robot location:

```java
// Create trigger for auto-switching
new Trigger(() -> {
    int hpTag = vision.hasID(PhotonVision.getHumanPlayerTagIDs());
    return hpTag != -1;
}).onTrue(
    Commands.runOnce(() -> vision.useHumanPlayerCamerasOnly())
).onFalse(
    Commands.runOnce(() -> vision.useAllCameras())
);
```

### 4. Null Checking

Always check for `null` or invalid returns:

```java
// DON'T
Rotation2d yaw = vision.getYawToAprilTag(7);
double degrees = yaw.getDegrees();  // NullPointerException if tag not found!

// DO
Rotation2d yaw = vision.getYawToAprilTag(7);
if (yaw != null) {
    double degrees = yaw.getDegrees();
}

// Or check visibility first
if (vision.isTagVisible(7)) {
    Rotation2d yaw = vision.getYawToAprilTag(7);
    // Safe to use yaw
}
```

### 5. Performance Considerations

**updatePoseEstimation() is expensive:**
- Already called in subsystem `periodic()`
- Don't call multiple times per loop
- Results are cached and auto-update

**Cache results if using multiple times:**
```java
// DON'T
if (vision.getClosestVisibleTag() != -1) {
    double distance = vision.getDistanceFromAprilTag(vision.getClosestVisibleTag());
}

// DO
int closestTag = vision.getClosestVisibleTag();
if (closestTag != -1) {
    double distance = vision.getDistanceFromAprilTag(closestTag);
}
```

### 6. Debugging Vision Issues

**Check camera state:**
```java
SmartDashboard.putBoolean("Vision Disabled", vision.areAllCamerasDisabled());
SmartDashboard.putBoolean("HP Mode", vision.isUsingHumanPlayerCamerasOnly());
SmartDashboard.putBoolean("Has Target", vision.hasTarget());
```

**Log detected tags:**
```java
List<Integer> tags = PhotonVision.getAllDetectedTags();
SmartDashboard.putString("Detected Tags", tags.toString());
```

**Monitor closest tag:**
```java
int closest = vision.getClosestVisibleTag();
if (closest != -1) {
    SmartDashboard.putNumber("Closest Tag", closest);
    SmartDashboard.putNumber("Distance", vision.getDistanceFromAprilTag(closest));
}
```

### 7. Simulation Testing

Use simulation to test vision logic before deploying to robot:

```java
if (RobotBase.isSimulation()) {
    // Test vision alignment commands
    // Test camera switching logic
    // Verify pose estimation accuracy
}
```

---

## Troubleshooting

### Vision Not Updating Pose

**Problem:** Robot pose not being corrected by vision

**Solutions:**
1. Check if vision is disabled: `vision.areAllCamerasDisabled()`
2. Verify camera is connected in PhotonVision UI
3. Check if any tags are visible: `vision.hasTarget()`
4. Verify standard deviations aren't too high
5. Check that `updatePoseEstimation()` is called in `periodic()`

### Wrong Tags Being Detected

**Problem:** Camera seeing tags it shouldn't

**Solutions:**
1. Use camera filtering: Verify tag IDs in `getReefTagIDs()` and `getHumanPlayerTagIDs()`
2. Switch camera modes: Use `useHumanPlayerCamerasOnly()` or `useAllCameras()`
3. Adjust camera angles to avoid seeing wrong zones
4. Check ambiguity filtering - may need to tune thresholds

### Pose Jumps Around

**Problem:** Robot pose suddenly jumps to wrong location

**Solutions:**
1. Increase standard deviations for less trust in vision
2. Check for reflective surfaces causing false detections
3. Verify AprilTag field layout is correct
4. Use multi-tag measurements only (more reliable)
5. Filter out high-ambiguity readings

### Can't See Tags at Expected Range

**Problem:** Tags not detected at expected distances

**Solutions:**
1. Check camera focus in PhotonVision UI
2. Verify camera calibration
3. Check lighting conditions
4. Increase camera exposure if too dark
5. Verify tags are correct size and not damaged

### Camera Latency Too High

**Problem:** Vision measurements delayed

**Solutions:**
1. Reduce camera resolution in PhotonVision
2. Lower camera FPS if network bandwidth limited
3. Check network switch performance
4. Verify robot radio not overloaded

### Simulation Not Working

**Problem:** Vision doesn't work in simulation

**Solutions:**
1. Verify `RobotBase.isSimulation()` check in code
2. Check that `visionSim` is being updated with robot pose
3. Verify field layout loaded correctly
4. Use `getVisionSim()` to access debug field

---

## See Also

- [PhotonVision Documentation](https://docs.photonvision.org/)
- [WPILib Pose Estimation](https://docs.wpilib.org/en/stable/docs/software/vision-processing/index.html)
- [YAGSL Documentation](https://github.com/BroncBotz3481/YAGSL)
- [AprilTag Field Layouts](https://github.com/wpilibsuite/allwpilib/tree/main/apriltag/src/main/native/resources/edu/wpi/first/apriltag)

---

**Need Help?** Check the PhotonVision forums or FRC Discord #programming channel for vision support.
