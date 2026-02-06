# Vision Configuration Guide

Complete guide to configuring PhotonVision cameras for year-to-year reuse.

## Table of Contents

- [Overview](#overview)
- [Quick Start](#quick-start)
- [Configuration Classes](#configuration-classes)
- [Builder API Reference](#builder-api-reference)
- [Standard Deviation Tuning](#standard-deviation-tuning)
- [Camera Purpose Guide](#camera-purpose-guide)
- [Migration Checklist](#migration-checklist)
- [Examples](#examples)

---

## Overview

The vision configuration system allows teams to fully customize their PhotonVision setup without modifying AdambotsLib source code. This enables:

- **Year-to-year reuse** - Configure cameras for each season's robot
- **Easy customization** - Change positions, rotations, and filtering per camera
- **Game-specific tag filtering** - Assign different tag groups to different cameras
- **Distance-based filtering** - Limit maximum detection range per camera
- **Purpose-based cameras** - Separate cameras for odometry vs alignment

---

## Coordinate System

Understanding the coordinate system is critical for accurate camera positioning. AdambotsLib uses the WPILib coordinate system.

### Top-Down View (X/Y Axes)

```
              FRONT OF ROBOT
                    ↑
                    │ +X
                    │
        +Y ←────────┼────────→ -Y
                    │
           (Robot   │
            Center) │
                    ↓ -X
               BACK OF ROBOT

    Example: Camera at X=15", Y=11.75"
             is 15" forward, 11.75" left
```

### Side View (Z Axis / Height)

```
    ↑ +Z (height from floor)
    │
    │    ┌─────────┐ ← Camera at Z=8"
    │    │  CAM    │
    │    └────┬────┘
    │         │
    │   ┌─────┴─────┐
    │   │   ROBOT   │
    │   │   FRAME   │
────┴───┴───────────┴──── Floor (Z=0)
```

### Rotation: Yaw (Top View)

```
    Robot Forward (0°)
           ↑
           │
    ←──────┼──────→
   +Yaw    │    -Yaw
   (left)  │   (right)

    Example: Yaw = -30° means camera
             faces 30° to the RIGHT
```

### Rotation: Pitch (Side View)

```
    Camera tilted UP (+pitch)
              ↗
             /
    ────────●──────── Level (0°)
             \
              ↘
    Camera tilted DOWN (-pitch)
```

### Measurement Tips

1. **Find Robot Center**: Typically the center of rotation (where swerve modules pivot)
2. **Measure X**: Distance forward/backward from center to camera lens
3. **Measure Y**: Distance left/right from center to camera lens
4. **Measure Z**: Height from floor to camera lens
5. **Measure Yaw**: Angle camera faces relative to robot forward
6. **Measure Pitch**: Up/down tilt of camera

---

## Quick Start

### 1. Define Game-Specific Tags (in your Constants file)

```java
public static final class VisionConstants {
    // Define game-specific tag groups based on your field layout
    public static final int[] SCORING_TAGS = {6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22};
    public static final int[] INTAKE_TAGS = {1, 2, 4, 5, 12, 13, 14, 15};
}
```

### 2. Build Vision Configuration

```java
import static edu.wpi.first.units.Units.*;

public static final VisionSystemConfig VISION_CONFIG = VisionConfigBuilder.create()
    .addCamera("Left")
        .position(Inches.of(15), Inches.of(11.75), Inches.of(8))      // X, Y, Z from robot center
        .rotation(Degrees.of(0), Degrees.of(0), Degrees.of(-30))       // Roll, Pitch, Yaw
        .purpose(CameraPurpose.ODOMETRY)
        .allowedTags(SCORING_TAGS)
        .singleTagStdDevs(Meters.of(0.5), Meters.of(0.5), Radians.of(0.5))
        .multiTagStdDevs(Meters.of(0.5), Meters.of(0.5), Radians.of(1.0))
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
        .maxTagDistance(Meters.of(3.0))  // Limit to close-range tags
        .done()
    .ambiguityThreshold(0.25)
    .maxPoseJump(Meters.of(10.0))
    .build();
```

### 3. Initialize in Robot

```java
// In RobotContainer.java
PhotonVision vision = new PhotonVision(
    VisionConstants.VISION_CONFIG,
    swerve::getPose,
    swerve.getField()
);
swerve.setupVision(vision);
```

---

## Configuration Classes

### VisionSystemConfig

Top-level configuration containing all camera configs and system parameters.

| Field | Type | Description |
|-------|------|-------------|
| `cameras` | `List<VisionCameraConfig>` | All camera configurations |
| `ambiguityThreshold` | `double` | Max ambiguity for pose estimates (0-1) |
| `maxPoseJumpMeters` | `double` | Max allowed pose jump in meters |

### VisionCameraConfig

Configuration for a single camera.

| Field | Type | Description |
|-------|------|-------------|
| `name` | `String` | PhotonVision camera name |
| `purpose` | `CameraPurpose` | ODOMETRY, ALIGNMENT, or BOTH |
| `robotToCamTranslation` | `Translation3d` | Position relative to robot center |
| `robotToCamRotation` | `Rotation3d` | Rotation relative to robot forward |
| `singleTagStdDevs` | `VisionStdDevs` | Std devs for single-tag estimates |
| `multiTagStdDevs` | `VisionStdDevs` | Std devs for multi-tag estimates |
| `allowedTagIDs` | `int[]` | Tag filtering (empty = all tags) |
| `maxTagDistanceMeters` | `double` | Max distance for single-tag estimates (default: 4.0) |

### VisionStdDevs

Standard deviation configuration with presets.

| Field | Description |
|-------|-------------|
| `x` | X position uncertainty in meters |
| `y` | Y position uncertainty in meters |
| `theta` | Rotation uncertainty in radians |

**Presets:**
- `VisionStdDevs.DEFAULT_SINGLE_TAG` - (0.5, 0.5, 0.5)
- `VisionStdDevs.DEFAULT_MULTI_TAG` - (0.5, 0.5, 1.0)
- `VisionStdDevs.HIGH_CONFIDENCE` - (0.1, 0.1, 0.1)
- `VisionStdDevs.LOW_CONFIDENCE` - (1.0, 1.0, 1.0)

### CameraPurpose

| Value | Description |
|-------|-------------|
| `ODOMETRY` | Camera for pose estimation only |
| `ALIGNMENT` | Camera for targeting/alignment only |
| `BOTH` | Camera for both purposes |

---

## Builder API Reference

### VisionConfigBuilder

```java
VisionConfigBuilder.create()           // Start building
    .addCamera(String name)            // Add a camera (returns CameraBuilder)
    .ambiguityThreshold(double)        // Set ambiguity threshold (default: 0.25)
    .maxPoseJump(double)               // Set max pose jump in meters (default: 10.0)
    .build()                           // Build final config
```

### CameraBuilder

```java
.addCamera("CameraName")
    .position(Distance x, Distance y, Distance z)  // Position (use Inches.of() or Meters.of())
    .rotation(Angle roll, Angle pitch, Angle yaw)  // Rotation (use Degrees.of() or Radians.of())
    .purpose(CameraPurpose)                        // Camera purpose (default: BOTH)
    .singleTagStdDevs(Distance x, Distance y, Angle theta)  // Single tag std devs
    .singleTagStdDevs(VisionStdDevs)               // Single tag std devs (preset)
    .multiTagStdDevs(Distance x, Distance y, Angle theta)   // Multi tag std devs
    .multiTagStdDevs(VisionStdDevs)                // Multi tag std devs (preset)
    .allowedTags(int... tagIDs)                    // Tag filtering
    .maxTagDistance(Distance)                      // Max distance for single-tag (default: 4m)
    .done()                                        // Return to VisionConfigBuilder
```

---

## Standard Deviation Tuning

Standard deviations control how much the pose estimator trusts vision measurements.

### Guidelines

| Situation | Recommended Std Devs | Reason |
|-----------|---------------------|--------|
| Close range, multiple tags | `(0.1, 0.1, 0.1)` | High confidence |
| Normal operation | `(0.5, 0.5, 0.5)` | Balanced |
| Far range, single tag | `(1.0, 1.0, 1.0)` | Lower confidence |
| Multi-tag rotation | `(0.5, 0.5, 1.0)` | Less trust in rotation |

### Automatic Adjustment

The system automatically adjusts standard deviations based on:
- **Number of tags visible** - More tags = lower std devs
- **Average distance** - Farther = higher std devs
- **Single tag beyond max distance** - Rejected (configurable via `maxTagDistance()`, default 4m)

### Tuning Process

1. Start with default values
2. Test on actual robot with known positions
3. If pose jumps around: Increase std devs
4. If pose doesn't correct: Decrease std devs
5. Test multi-tag vs single-tag scenarios separately

---

## Camera Purpose Guide

### ODOMETRY Cameras

Cameras that contribute to robot pose estimation.

**Use for:**
- General robot localization
- Autonomous navigation
- Vision-corrected odometry

**Characteristics:**
- Wide field of view preferred
- Good coverage of field AprilTags
- Mounted for optimal tag visibility

### ALIGNMENT Cameras

Cameras used for precise targeting/alignment only.

**Use for:**
- Scoring alignment
- Pickup alignment
- Targeting game elements

**Characteristics:**
- May be narrower FOV
- Optimized for specific game elements
- NOT used for pose estimation

### BOTH Purpose

Cameras that serve both odometry and alignment needs.

**Use when:**
- Limited camera count
- Camera sees both scoring and general tags
- General-purpose setup

---

## Migration Checklist

### Year-to-Year Migration Steps

1. **Measure New Robot**
   - [ ] Robot center point (center of rotation)
   - [ ] Camera positions (X, Y, Z from center)
   - [ ] Camera rotations (Roll, Pitch, Yaw)

2. **Update Tag Groups**
   - [ ] Review new game's AprilTag layout
   - [ ] Define game-specific tag groups
   - [ ] Assign tags to cameras by purpose

3. **Update Configuration**
   - [ ] Update VisionConstants with new positions
   - [ ] Update VisionConstants with new tag groups
   - [ ] Adjust std devs if needed

4. **Test and Tune**
   - [ ] Test pose estimation accuracy
   - [ ] Verify tag filtering works correctly
   - [ ] Tune std devs based on real-world testing

### Migration Example

```java
// Previous season configuration
public static final int[] OLD_SCORING_TAGS = {6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22};

// New season configuration
public static final int[] SCORING_TAGS = {5, 6, 7, 14, 15, 16};  // New game tags
public static final int[] INTAKE_TAGS = {1, 2, 3, 11, 12, 13};   // New game tags
```

---

## Examples

### Example 1: Two-Camera Setup

```java
VisionSystemConfig config = VisionConfigBuilder.create()
    .addCamera("FrontLeft")
        .position(Inches.of(12), Inches.of(10), Inches.of(8))
        .rotation(Degrees.of(0), Degrees.of(-15), Degrees.of(-30))
        .purpose(CameraPurpose.BOTH)
        .done()
    .addCamera("FrontRight")
        .position(Inches.of(12), Inches.of(-10), Inches.of(8))
        .rotation(Degrees.of(0), Degrees.of(-15), Degrees.of(30))
        .purpose(CameraPurpose.BOTH)
        .done()
    .build();
```

### Example 2: Specialized Cameras

```java
VisionSystemConfig config = VisionConfigBuilder.create()
    // Wide-angle cameras for odometry
    .addCamera("OdometryLeft")
        .position(Inches.of(10), Inches.of(12), Inches.of(6))
        .rotation(Degrees.of(0), Degrees.of(0), Degrees.of(-45))
        .purpose(CameraPurpose.ODOMETRY)
        .done()
    .addCamera("OdometryRight")
        .position(Inches.of(10), Inches.of(-12), Inches.of(6))
        .rotation(Degrees.of(0), Degrees.of(0), Degrees.of(45))
        .purpose(CameraPurpose.ODOMETRY)
        .done()
    // Narrow camera for scoring alignment with limited range
    .addCamera("ScoringCam")
        .position(Inches.of(14), Inches.of(0), Inches.of(24))
        .rotation(Degrees.of(0), Degrees.of(-30), Degrees.of(0))
        .purpose(CameraPurpose.ALIGNMENT)
        .allowedTags(SCORING_TAGS)
        .maxTagDistance(Meters.of(3.0))  // Limit to close-range tags only
        .done()
    .ambiguityThreshold(0.2)
    .build();
```

### Example 3: Using Camera Filtering at Runtime

```java
// Disable all alignment cameras
vision.disableCamerasWithPurpose(CameraPurpose.ALIGNMENT);

// Enable specific camera
vision.enableCamera("ScoringCam");

// Get cameras by purpose
List<VisionCamera> odometryCameras = vision.getCamerasWithPurpose(CameraPurpose.ODOMETRY);
```

### Example 4: Custom Std Devs

```java
VisionSystemConfig config = VisionConfigBuilder.create()
    .addCamera("HighConfidenceCam")
        .position(Inches.of(8), Inches.of(0), Inches.of(12))
        .rotation(Degrees.of(0), Degrees.of(-20), Degrees.of(0))
        .purpose(CameraPurpose.ODOMETRY)
        .singleTagStdDevs(VisionStdDevs.HIGH_CONFIDENCE)
        .multiTagStdDevs(Meters.of(0.05), Meters.of(0.05), Radians.of(0.1))
        .done()
    .addCamera("LowConfidenceCam")
        .position(Inches.of(-6), Inches.of(0), Inches.of(30))
        .rotation(Degrees.of(0), Degrees.of(-45), Degrees.of(180))
        .purpose(CameraPurpose.ODOMETRY)
        .singleTagStdDevs(VisionStdDevs.LOW_CONFIDENCE)
        .multiTagStdDevs(VisionStdDevs.DEFAULT_MULTI_TAG)
        .done()
    .build();
```

---

## See Also

- [PhotonVision Documentation](PhotonVision.md) - Full PhotonVision class reference
- [Vision README](README.md) - Vision package overview
- [PhotonVision Docs](https://docs.photonvision.org/) - Official PhotonVision documentation
- [WPILib Coordinate System](https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html)

---

**Need Help?** Check the PhotonVision forums or FRC Discord #programming channel for vision support.
