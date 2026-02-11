# PhotonVision

PhotonVision integration for AprilTag-based vision pose estimation with multi-camera support.

## Table of Contents

- [Overview](#overview)
- [Configuration (Recommended)](#configuration-recommended)
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

The `PhotonVision` class implements the `VisionSystem` interface and provides comprehensive AprilTag vision functionality for FRC robots, including:

- **Multi-camera support** - Manage multiple PhotonVision cameras independently
- **Vision-corrected odometry** - Automatically update swerve drive pose estimates
- **Tag filtering** - Enable specific cameras for different game zones
- **Simulation support** - Full PhotonVision simulation integration
- **Distance and angle calculations** - Find distances, yaw angles, and closest tags
- **Camera state management** - Dynamically enable/disable cameras during match

This class is designed to integrate seamlessly with YAGSL's swerve drive and WPILib's pose estimation system. The `VisionSystem` interface abstraction allows teams to swap in different vision implementations (e.g., Limelight) without modifying subsystem code.

**Based on:** Modified from [Ironclad 2024's Vision class](https://gitlab.com/ironclad_code/ironclad-2024/-/blob/master/src/main/java/frc/robot/vision/Vision.java)

---

## Configuration (Recommended)

**NEW:** The recommended way to set up PhotonVision is using the configurable camera system. This allows full customization without modifying AdambotsLib source code.

### Step 1: Define Your Configuration

In your robot project's Constants file:

```java
import static edu.wpi.first.units.Units.*;

public static final class VisionConstants {
    // Game-specific tag groups
    public static final int[] SCORING_TAGS = {6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22};
    public static final int[] ALIGNMENT_TAGS = {1, 2, 4, 5, 12, 13, 14, 15};

    public static final VisionSystemConfig CONFIG = VisionConfigBuilder.create()
        .addCamera("Left")
            .position(Inches.of(15), Inches.of(11.75), Inches.of(8))
            .rotation(Degrees.of(0), Degrees.of(0), Degrees.of(-30))
            .purpose(CameraPurpose.ODOMETRY)
            .allowedTags(SCORING_TAGS)
            .done()
        .addCamera("Right")
            .position(Inches.of(15), Inches.of(-11.75), Inches.of(8))
            .rotation(Degrees.of(0), Degrees.of(0), Degrees.of(30))
            .purpose(CameraPurpose.ODOMETRY)
            .allowedTags(SCORING_TAGS)
            .done()
        .addCamera("Middle")
            .position(Inches.of(8), Inches.of(0), Inches.of(41))
            .rotation(Degrees.of(0), Degrees.of(-43), Degrees.of(177))
            .purpose(CameraPurpose.ALIGNMENT)
            .allowedTags(ALIGNMENT_TAGS)
            .done()
        .ambiguityThreshold(0.25)
        .build();
}
```

### Step 2: Initialize Vision

```java
// In RobotContainer.java
PhotonVision vision = new PhotonVision(
    VisionConstants.CONFIG,
    swerve::getPose,
    swerve.getField()
);
swerve.setupVision(vision);
```

### Step 3: Control Cameras at Runtime

```java
// Enable/disable by purpose
vision.enableCamerasWithPurpose(CameraPurpose.ODOMETRY);
vision.disableCamerasWithPurpose(CameraPurpose.ALIGNMENT);

// Enable/disable by name
vision.enableCamera("Left");
vision.disableCamera("Middle");

// Get cameras by purpose
List<VisionCamera> odometryCameras = vision.getCamerasWithPurpose(CameraPurpose.ODOMETRY);
```

For complete configuration documentation, see **[VisionConfiguration.md](VisionConfiguration.md)**.

---

## Quick Start

### 1. Create Vision Configuration

```java
// In your Constants file
import static edu.wpi.first.units.Units.*;

public static final VisionSystemConfig VISION_CONFIG = VisionConfigBuilder.create()
    .addCamera("Left")
        .position(Inches.of(15), Inches.of(11.75), Inches.of(8))
        .rotation(Degrees.of(0), Degrees.of(0), Degrees.of(-30))
        .purpose(CameraPurpose.ODOMETRY)
        .allowedTags(6, 7, 8, 9, 10, 11)
        .done()
    .ambiguityThreshold(0.25)
    .build();
```

### 2. Initialize Vision in RobotContainer

```java
// In RobotContainer
PhotonVision vision = new PhotonVision(
    VisionConstants.VISION_CONFIG,
    swerve::getPose,
    swerve.getField()
);
swerve.setupVision(vision);
```

### 3. Update Pose Estimation

```java
@Override
public void periodic() {
    // Update odometry with vision measurements
    vision.updatePoseEstimation(swerveDrive);

    // Update Field2d with visible tags (optional)
    vision.updateVisionField();
}
```

### 4. Use Vision Methods

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

PhotonVision supports multiple cameras configured via `VisionConfigBuilder`:

```java
VisionConfigBuilder.create()
    .addCamera("Left")
        .position(Inches.of(15), Inches.of(11.75), Inches.of(8))
        .rotation(Degrees.of(0), Degrees.of(0), Degrees.of(-30))
        .purpose(CameraPurpose.ODOMETRY)
        .allowedTags(SCORING_TAGS)
        .done()
    .addCamera("Right")
        .position(Inches.of(15), Inches.of(-11.75), Inches.of(8))
        .rotation(Degrees.of(0), Degrees.of(0), Degrees.of(30))
        .purpose(CameraPurpose.ODOMETRY)
        .allowedTags(SCORING_TAGS)
        .done()
    .build();
```

Each camera has:
- Independent pose estimation
- Configurable standard deviations
- AprilTag filtering via `allowedTags()`
- Distance-based filtering via `maxTagDistance()`
- Purpose-based filtering (ODOMETRY, ALIGNMENT, BOTH)
- Ambiguity-based quality scoring

### Camera Filtering

Cameras can be configured to only process specific AprilTag IDs using `allowedTags()` and limit detection range using `maxTagDistance()`:

```java
.addCamera("Left")
    .allowedTags(6, 7, 8, 9, 10, 11)  // Only process scoring area tags
    .done()

.addCamera("Middle")
    .allowedTags(1, 2, 4, 5, 12, 13, 14, 15)  // Only process HP tags
    .maxTagDistance(Meters.of(3.0))  // Reject tags beyond 3 meters
    .done()
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

### Enable/Disable Cameras

```java
// Disable all vision
vision.disableAllCameras();

// Re-enable vision
vision.enableAllCameras();

// Enable/disable by name
vision.enableCamera("Left");
vision.disableCamera("Middle");

// Enable/disable by purpose
vision.enableCamerasWithPurpose(CameraPurpose.ODOMETRY);
vision.disableCamerasWithPurpose(CameraPurpose.ALIGNMENT);
```

### Check Camera State

```java
// Check if vision is disabled
boolean disabled = vision.areAllCamerasDisabled();

// Get cameras by purpose
List<VisionCamera> odometryCameras = vision.getCamerasWithPurpose(CameraPurpose.ODOMETRY);

// Get specific camera
VisionCamera leftCam = vision.getCamera("Left");
```

### Camera Purpose Guide

| Purpose | Use Case |
|---------|----------|
| `CameraPurpose.ODOMETRY` | Cameras for pose estimation |
| `CameraPurpose.ALIGNMENT` | Cameras for targeting/alignment only |
| `CameraPurpose.BOTH` | Cameras for both purposes |

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

// Check if any of multiple tags visible
int visibleTag = vision.hasID(new int[]{6, 7, 8});  // Returns first found, or -1

// Get all detected tags
List<Integer> allTags = PhotonVision.getAllDetectedTagIds();
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
// Get best target from specific camera (returns VisionTarget interface)
Optional<VisionTarget> bestTarget = vision.getBestTargetFromCamera("Left");
bestTarget.ifPresent(target -> {
    int tagID = target.getFiducialId();
    double ambiguity = target.getPoseAmbiguity();
});

// Get specific target from camera
PhotonTrackedTarget target = vision.getTargetFromId(7, "Left");
```

### Static Field Methods

```java
// Get pose of AprilTag on field
Pose2d tagPose = PhotonVision.getAprilTagPose(7, new Transform2d());
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

### Example 3: Camera Purpose-Based Switching

```java
public class RobotContainer {
    private final SwerveSubsystem swerve;

    private void configureButtonBindings() {
        // Enable only alignment cameras when at scoring position
        Buttons.XboxBack.onTrue(
            Commands.runOnce(() -> {
                swerve.vision.disableCamerasWithPurpose(CameraPurpose.ODOMETRY);
                swerve.vision.enableCamerasWithPurpose(CameraPurpose.ALIGNMENT);
            }).withName("SwitchToAlignmentCameras")
        );

        // Enable only odometry cameras for general driving
        Buttons.XboxStart.onTrue(
            Commands.runOnce(() -> {
                swerve.vision.enableCamerasWithPurpose(CameraPurpose.ODOMETRY);
                swerve.vision.disableCamerasWithPurpose(CameraPurpose.ALIGNMENT);
            }).withName("SwitchToOdometryCameras")
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

    // Define your game-specific tag IDs
    private static final int[] SCORING_TAGS = {6, 7, 8, 9, 10, 11};

    public Command autoExtendAtScoringArea() {
        return Commands.runOnce(() -> {
            // Check if we see any scoring tags
            int scoringTag = vision.hasID(SCORING_TAGS);
            if (scoringTag != -1) {
                // Check if close enough
                double distance = vision.getDistanceFromAprilTag(scoringTag);
                if (distance < 3.0) {  // Within 3 meters
                    sm.to(State.EXTENDED).request();
                }
            }
        });
    }

    @Override
    public void periodic() {
        sm.periodic();

        // Auto-stow if we can't see any scoring tags
        int scoringTag = vision.hasID(SCORING_TAGS);
        if (scoringTag == -1 && sm.getCurrentState() == State.EXTENDED) {
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

#### `isTagVisibleInCamera(int tagID, String cameraName)`

Check if a specific AprilTag is visible in a specific camera.

**Parameters:**
- `tagID` - AprilTag ID to check
- `cameraName` - Name of the camera to check

**Returns:** `true` if visible in specified camera

**Example:**
```java
if (vision.isTagVisibleInCamera(7, "Left")) {
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
// Define your game-specific tag groups in Constants
int[] SCORING_TAGS = {6, 7, 8, 9, 10, 11};

// Check for any scoring tags
int visibleTag = vision.hasID(SCORING_TAGS);
if (visibleTag != -1) {
    // Found scoring tag
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

#### `getAllDetectedTagIds()` (static)

Get a list of all currently detected AprilTag IDs across all cameras.

**Returns:** `List<Integer>` of detected tag IDs

**Example:**
```java
List<Integer> tags = PhotonVision.getAllDetectedTagIds();
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

#### `disableAllCameras()`

Disable all cameras from contributing to pose estimation.

**Use when:** Vision is unreliable or causing issues

**Example:**
```java
vision.disableAllCameras();
```

#### `enableAllCameras()`

Enable all cameras for pose estimation.

**Example:**
```java
vision.enableAllCameras();
```

#### `enableCamera(String name)` / `disableCamera(String name)`

Enable or disable a specific camera by name.

**Parameters:**
- `name` - The camera name as defined in VisionConfigBuilder

**Example:**
```java
vision.enableCamera("Left");
vision.disableCamera("Middle");
```

#### `enableCamerasWithPurpose(CameraPurpose purpose)` / `disableCamerasWithPurpose(CameraPurpose purpose)`

Enable or disable cameras by their configured purpose.

**Parameters:**
- `purpose` - `CameraPurpose.ODOMETRY`, `CameraPurpose.ALIGNMENT`, or `CameraPurpose.BOTH`

**Example:**
```java
// Enable odometry cameras, disable alignment cameras
vision.enableCamerasWithPurpose(CameraPurpose.ODOMETRY);
vision.disableCamerasWithPurpose(CameraPurpose.ALIGNMENT);
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

#### `getCamerasWithPurpose(CameraPurpose purpose)`

Get a list of cameras configured for a specific purpose.

**Returns:** `List<VisionCamera>` matching the purpose

**Example:**
```java
List<VisionCamera> odometryCameras = vision.getCamerasWithPurpose(CameraPurpose.ODOMETRY);
```

#### `getCameras()`

Get a list of all configured cameras.

**Returns:** `List<VisionCamera>` of all cameras

**Example:**
```java
List<VisionCamera> allCameras = vision.getCameras();
for (VisionCamera cam : allCameras) {
    System.out.println("Camera: " + cam.getName() + " enabled: " + cam.isEnabled());
}
```

---

### Target Tracking Methods

#### `getTargetFromId(int id, String cameraName)`

Get the tracked target for a specific tag from a specific camera.

**Parameters:**
- `id` - AprilTag ID
- `cameraName` - Name of the camera to check

**Returns:** `PhotonTrackedTarget` or `null` if not found

**Example:**
```java
PhotonTrackedTarget target = vision.getTargetFromId(7, "Left");
if (target != null) {
    double ambiguity = target.getPoseAmbiguity();
}
```

#### `getBestTargetFromCamera(String cameraName)`

Get the target with the lowest ambiguity from a specific camera.

**Parameters:**
- `cameraName` - Name of the camera to check

**Returns:** `Optional<VisionTarget>` with lowest ambiguity

**Example:**
```java
Optional<VisionTarget> best = vision.getBestTargetFromCamera("Left");
best.ifPresent(target -> {
    System.out.println("Best target: " + target.getFiducialId());
});
```

**Note:** If you need access to PhotonVision-specific target features, use `getBestPhotonTargetFromCamera()` instead.

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

Switch camera modes based on robot purpose:

```java
// Create trigger for auto-switching based on field region
new Trigger(() -> {
    // Check if robot is in a specific field region
    Pose2d pose = swerve.getPose();
    return pose.getX() > 13.0; // Near loading zone
}).onTrue(
    Commands.runOnce(() -> vision.enableCamerasWithPurpose(CameraPurpose.ALIGNMENT))
).onFalse(
    Commands.runOnce(() -> vision.enableCamerasWithPurpose(CameraPurpose.ODOMETRY))
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
SmartDashboard.putBoolean("Has Target", vision.hasTarget());
```

**Log detected tags:**
```java
List<Integer> tags = PhotonVision.getAllDetectedTagIds();
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
1. Use camera filtering: Configure `allowedTags()` per-camera in VisionConfigBuilder
2. Switch camera modes: Use `enableCamerasWithPurpose()` or `disableCamerasWithPurpose()`
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
