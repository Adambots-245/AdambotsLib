# Vision

AprilTag vision integration for accurate robot localization and target tracking.

## Overview

The vision package provides a flexible vision system abstraction for FRC robots, enabling vision-corrected odometry and intelligent target tracking with AprilTags. The default implementation uses PhotonVision, but the abstraction layer allows teams to integrate other vision systems (Limelight, custom solutions) without modifying subsystem code.

## Architecture

The vision system uses an interface-based abstraction:

- **`VisionSystem`** - Main interface for vision functionality (pose estimation, tag detection)
- **`VisionCameraInterface`** - Interface for individual cameras
- **`VisionResult`** - Interface for vision processing results
- **`VisionTarget`** - Interface for detected targets

The `PhotonVision` class implements `VisionSystem` and provides the default PhotonVision integration.

## Configuration Guide

**NEW:** The vision system is now fully configurable for year-to-year reuse. See **[VisionConfiguration.md](VisionConfiguration.md)** for the complete guide.

```java
import static edu.wpi.first.units.Units.*;

// Quick example - define in your Constants file
VisionSystemConfig config = VisionConfigBuilder.create()
    .addCamera("Left")
        .position(Inches.of(15), Inches.of(11.75), Inches.of(8))
        .rotation(Degrees.of(0), Degrees.of(0), Degrees.of(-30))
        .purpose(CameraPurpose.ODOMETRY)
        .allowedTags(SCORING_TAGS)
        .done()
    .ambiguityThreshold(0.25)
    .build();

// Create vision system and initialize in RobotContainer
PhotonVision vision = new PhotonVision(config, swerve::getPose, swerve.getField());
swerve.setupVision(vision);
```

## Available Classes

### PhotonVision Integration

- **[PhotonVision](PhotonVision.md)** - Complete AprilTag vision system
  - Multi-camera support (front, back cameras)
  - Vision-corrected pose estimation
  - Camera filtering by purpose (odometry vs alignment)
  - Distance and angle calculations to tags
  - Closest tag detection
  - Tag visibility checking
  - Simulation support

### Configuration Classes

- **[VisionConfiguration](VisionConfiguration.md)** - Configuration guide and API reference
  - `VisionSystemConfig` - System-wide configuration
  - `VisionCameraConfig` - Per-camera configuration
  - `VisionStdDevs` - Standard deviation presets
  - `VisionConfigBuilder` - Fluent configuration builder
  - `VisionCamera` - Configurable camera class

---

## When to Use Vision

### Use PhotonVision When:
- You need accurate robot localization beyond wheel odometry
- You want to auto-align to scoring positions
- You need to detect and track game pieces via tags
- You want to correct for wheel slip and drift
- You need precise pose estimation for autonomous
- You want to know robot position relative to field elements

**Common Use Cases:**
- Auto-align to scoring positions
- Vision-corrected autonomous paths
- Accurate pose estimation during teleop
- Auto-approach to game elements
- Field-relative control with pose correction
- Detecting when in scoring range

### Don't Use Vision When:
- Tags not visible (use wheel odometry fallback)
- Lighting conditions too poor
- Tags obscured or damaged
- Network bandwidth limited (reduce resolution)
- Vision causing pose estimation issues (disable temporarily)

---

## Quick Start

### 1. Basic Setup

```java
// In RobotContainer.java
public class RobotContainer {
    private final SwerveSubsystem swerve;

    public RobotContainer() {
        // Create swerve subsystem first
        swerve = new SwerveSubsystem(
            new File(Filesystem.getDeployDirectory(), "swerve/kraken")
        );

        // Create vision system with reference to swerve
        PhotonVision vision = new PhotonVision(
            VisionConstants.CONFIG,
            swerve::getPose,
            swerve.getField()
        );

        // Pass vision to swerve (handles circular dependency)
        swerve.setupVision(vision);
    }
}
```

The SwerveSubsystem automatically calls `vision.updatePoseEstimation()` and `vision.updateVisionField()` in its `periodic()` method when vision is configured.

### 2. Basic Vision Queries

```java
// Check if tag visible
if (vision.isTagVisible(7)) {
    double distance = vision.getDistanceFromAprilTag(7);
    Rotation2d yaw = vision.getYawToAprilTag(7);
}

// Find closest tag
int closest = vision.getClosestVisibleTag();
if (closest != -1) {
    double distance = vision.getDistanceToClosestTag();
}

// Check if any tags visible
if (vision.hasTarget()) {
    List<Integer> tags = PhotonVision.getAllDetectedTagIds();
}
```

### 3. Camera Mode Switching

```java
// Enable only alignment cameras (for scoring/pickup)
vision.enableCamerasWithPurpose(CameraPurpose.ALIGNMENT);
vision.disableCamerasWithPurpose(CameraPurpose.ODOMETRY);

// Enable only odometry cameras (for general driving)
vision.enableCamerasWithPurpose(CameraPurpose.ODOMETRY);
vision.disableCamerasWithPurpose(CameraPurpose.ALIGNMENT);

// Disable vision temporarily
vision.disableAllCameras();
```

---

## Key Features

### Multi-Camera Support

PhotonVision supports multiple cameras with independent configurations:

| Camera | Location | Purpose | Tag Filter |
|--------|----------|---------|------------|
| **LEFT_CAM** | Front left | Odometry | Configurable per game |
| **RIGHT_CAM** | Front right | Odometry | Configurable per game |
| **CENTER_CAM** | Back/up | Alignment | Configurable per game |

### Vision-Corrected Odometry

Automatic pose estimation with intelligent filtering:

```java
// Happens automatically in periodic()
vision.updatePoseEstimation(swerveDrive);
```

**Features:**
- Dynamic standard deviations (distance-based trust)
- Multi-tag fusion for higher confidence
- Ambiguity filtering (rejects bad measurements)
- Camera mode switching (zone-based filtering)

### Distance & Angle Calculations

```java
// Distance to specific tag
double distance = vision.getDistanceFromAprilTag(7);

// Yaw angle to tag (for auto-alignment)
Rotation2d yaw = vision.getYawToAprilTag(7);

// Full transform to tag
Transform2d transform = vision.getTransformToAprilTag(7);
```

### Tag Detection

```java
// Check single tag
boolean canSee = vision.isTagVisible(7);

// Check multiple tags
int visibleTag = vision.hasID(new int[]{6, 7, 8});

// Get all detected tags
List<Integer> allTags = PhotonVision.getAllDetectedTagIds();

// Find closest
int closest = vision.getClosestVisibleTag();
```

---

## Real-World Usage Patterns

### Pattern 1: Auto-Align to Tag

```java
public Command autoAlignToTag(int tagID, double tolerance) {
    return Commands.run(() -> {
        Rotation2d yaw = vision.getYawToAprilTag(tagID);
        if (yaw != null) {
            double rotationSpeed = pid.calculate(yaw.getRadians(), 0);
            drive(new ChassisSpeeds(0, 0, rotationSpeed));
        }
    }, this)
    .until(() -> {
        Rotation2d yaw = vision.getYawToAprilTag(tagID);
        return yaw != null && Math.abs(yaw.getDegrees()) < tolerance;
    });
}
```

### Pattern 2: Approach Target Distance

```java
public Command approachTag(int tagID, double targetDistance) {
    return Commands.run(() -> {
        double distance = vision.getDistanceFromAprilTag(tagID);
        Rotation2d yaw = vision.getYawToAprilTag(tagID);

        double forwardSpeed = distancePID.calculate(distance, targetDistance);
        double rotationSpeed = headingPID.calculate(yaw.getRadians(), 0);

        drive(new ChassisSpeeds(forwardSpeed, 0, rotationSpeed));
    }, this)
    .until(() -> {
        double distance = vision.getDistanceFromAprilTag(tagID);
        return Math.abs(distance - targetDistance) < 0.1;
    });
}
```

### Pattern 3: Purpose-Based Camera Switching

```java
// Define your game-specific tag groups in Constants
int[] ALIGNMENT_TAGS = {1, 2, 4, 5, 12, 13, 14, 15};

// Auto-switch cameras based on detected tags
new Trigger(() -> {
    int alignmentTag = vision.hasID(ALIGNMENT_TAGS);
    return alignmentTag != -1;
}).onTrue(
    Commands.runOnce(() -> {
        vision.enableCamerasWithPurpose(CameraPurpose.ALIGNMENT);
        vision.disableCamerasWithPurpose(CameraPurpose.ODOMETRY);
    })
).onFalse(
    Commands.runOnce(() -> {
        vision.enableCamerasWithPurpose(CameraPurpose.ODOMETRY);
        vision.disableCamerasWithPurpose(CameraPurpose.ALIGNMENT);
    })
);
```

### Pattern 4: Vision-Based State Machine

```java
public class ArmSubsystem extends SubsystemBase {
    enum State { STOWED, SCORING }

    // Define your game-specific tag groups in Constants
    private static final int[] SCORING_TAGS = {6, 7, 8, 9, 10, 11};

    public void periodic() {
        // Auto-extend arm when close to scoring area
        int scoringTag = vision.hasID(SCORING_TAGS);
        if (scoringTag != -1) {
            double distance = vision.getDistanceFromAprilTag(scoringTag);
            if (distance < 2.0 && sm.getCurrentState() == State.STOWED) {
                sm.to(State.SCORING).request();
            }
        }
    }
}
```

---

## Configuration

### Camera Transform Configuration

Each camera needs a transform (position + rotation) relative to robot center:

```java
import static edu.wpi.first.units.Units.*;

// Use VisionConfigBuilder instead - see VisionConfiguration.md
VisionConfigBuilder.create()
    .addCamera("Left")
        .position(Inches.of(15), Inches.of(11.75), Inches.of(8))     // X, Y, Z from robot center
        .rotation(Degrees.of(0), Degrees.of(0), Degrees.of(-30))      // Roll, Pitch, Yaw
        .purpose(CameraPurpose.ODOMETRY)
        .allowedTags(SCORING_TAGS)                                     // Your game-specific tags
        .singleTagStdDevs(Meters.of(0.5), Meters.of(0.5), Radians.of(0.5))
        .multiTagStdDevs(Meters.of(0.5), Meters.of(0.5), Radians.of(1.0))
        .done()
    .build();
```

**Measurement Tips:**
1. Define robot center (typically center of rotation)
2. Measure camera position from center with tape measure
3. Measure camera angles with digital angle finder or phone app
4. Positive X = forward, Positive Y = left, Positive Z = up

### Standard Deviation Tuning

Adjust trust in vision measurements:

```java
// Higher values = less trust in vision
VecBuilder.fill(0.5, 0.5, 0.5)  // Typical single tag
VecBuilder.fill(0.5, 0.5, 1.0)  // Multi tag (less trust in rotation)

// Very confident (close range, multiple tags)
VecBuilder.fill(0.1, 0.1, 0.1)

// Low confidence (far range, single tag)
VecBuilder.fill(1.0, 1.0, 1.0)
```

**Guidelines:**
- Start with conservative values (0.5, 0.5, 0.5)
- Tune on actual robot with test paths
- Single tag readings need higher std devs
- Multi-tag readings can have lower std devs
- Distance-based scaling happens automatically

---

## Best Practices

### 1. Camera Placement

**Front Cameras:**
- Mount as high as practical for long range
- Angle slightly upward (5-15°)
- Wide FOV to see multiple tags
- Position symmetrically for consistent readings

**Back Camera:**
- Mount facing backward/upward
- Position to see human player station
- Higher = better range

### 2. Always Check for Null/Invalid

```java
// DON'T
Rotation2d yaw = vision.getYawToAprilTag(7);
double degrees = yaw.getDegrees();  // NullPointerException!

// DO
Rotation2d yaw = vision.getYawToAprilTag(7);
if (yaw != null) {
    double degrees = yaw.getDegrees();
}
```

### 3. Cache Results When Reusing

```java
// DON'T (calls method twice)
if (vision.getClosestVisibleTag() != -1) {
    double distance = vision.getDistanceFromAprilTag(vision.getClosestVisibleTag());
}

// DO (cache result)
int closestTag = vision.getClosestVisibleTag();
if (closestTag != -1) {
    double distance = vision.getDistanceFromAprilTag(closestTag);
}
```

### 4. Switch Camera Modes Intelligently

```java
// At scoring/alignment position
vision.enableCamerasWithPurpose(CameraPurpose.ALIGNMENT);
vision.disableCamerasWithPurpose(CameraPurpose.ODOMETRY);

// General driving
vision.enableCamerasWithPurpose(CameraPurpose.ODOMETRY);
vision.disableCamerasWithPurpose(CameraPurpose.ALIGNMENT);

// Vision causing issues
vision.disableAllCameras();
```

### 5. Debug with SmartDashboard

```java
SmartDashboard.putBoolean("Vision Enabled", !vision.areAllCamerasDisabled());
SmartDashboard.putBoolean("Has Target", vision.hasTarget());
SmartDashboard.putString("Detected Tags", PhotonVision.getAllDetectedTagIds().toString());

int closest = vision.getClosestVisibleTag();
if (closest != -1) {
    SmartDashboard.putNumber("Closest Tag", closest);
    SmartDashboard.putNumber("Distance", vision.getDistanceFromAprilTag(closest));
}
```

### 6. Test in Simulation First

```java
if (RobotBase.isSimulation()) {
    // Test vision alignment commands
    // Verify camera switching logic
    // Check pose estimation accuracy
}
```

---

## Troubleshooting

| Problem | Solution |
|---------|----------|
| **Pose not updating** | Check `updatePoseEstimation()` called in `periodic()`, verify tags visible, check if vision disabled |
| **Wrong tags detected** | Verify tag filtering, switch camera modes, check camera angles |
| **Pose jumps around** | Increase standard deviations, check for reflections, verify field layout |
| **Can't see tags at range** | Check camera focus, verify calibration, check lighting, increase exposure |
| **High latency** | Reduce camera resolution, lower FPS, check network |
| **Simulation not working** | Verify `visionSim.update()` called with correct pose |

---

## Integration with Other Subsystems

### With Swerve Drive

```java
// In RobotContainer - setup vision with swerve
PhotonVision vision = new PhotonVision(
    VisionConstants.CONFIG,
    swerve::getPose,
    swerve.getField()
);
swerve.setupVision(vision);

// SwerveSubsystem automatically handles periodic updates
// when vision is configured via setupVision()
```

### With LEDs

```java
public class LEDSubsystem extends SubsystemBase {
    @Override
    public void periodic() {
        if (vision.hasTarget()) {
            double distance = vision.getDistanceToClosestTag();
            setColor(distance < 2.0 ? Color.kGreen : Color.kOrange);
        } else {
            setColor(Color.kBlue);
        }
    }
}
```

### With State Machines

```java
// Define your game-specific tag groups in Constants
int[] SCORING_TAGS = {6, 7, 8, 9, 10, 11};

// Trigger state transitions based on vision
new Trigger(() -> {
    int scoringTag = vision.hasID(SCORING_TAGS);
    if (scoringTag == -1) return false;
    return vision.getDistanceFromAprilTag(scoringTag) < 2.0;
}).onTrue(
    intake.extendCommand()
);
```

---

## See Also

- [Swerve Drive Documentation](../subsystems/SwerveSubsystem.md)
- [Utils Documentation](../utils/README.md) - Angle utilities for vision calculations
- [PhotonVision Documentation](https://docs.photonvision.org/)
- [WPILib Vision Processing](https://docs.wpilib.org/en/stable/docs/software/vision-processing/index.html)

---

**Need Help?** Check the detailed [PhotonVision documentation](PhotonVision.md) or refer to usage examples in the code.
