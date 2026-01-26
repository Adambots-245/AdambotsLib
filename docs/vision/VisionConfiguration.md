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
- **Purpose-based cameras** - Separate cameras for odometry vs alignment

---

## Quick Start

### 1. Define Game-Specific Tags (in your Constants file)

```java
public static final class VisionConstants {
    // 2025 Reefscape example
    public static final int[] REEF_TAGS = {6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22};
    public static final int[] HP_TAGS = {1, 2, 4, 5, 12, 13, 14, 15};
}
```

### 2. Build Vision Configuration

```java
public static final VisionSystemConfig VISION_CONFIG = VisionConfigBuilder.create()
    .addCamera("Left")
        .positionInches(15, 11.75, 8)      // X, Y, Z from robot center
        .rotationDegrees(0, 0, -30)         // Roll, Pitch, Yaw
        .purpose(CameraPurpose.ODOMETRY)
        .allowedTags(REEF_TAGS)
        .singleTagStdDevs(0.5, 0.5, 0.5)
        .multiTagStdDevs(0.5, 0.5, 1.0)
        .done()
    .addCamera("Right")
        .positionInches(15, -11.75, 8)
        .rotationDegrees(0, 0, 30)
        .purpose(CameraPurpose.ODOMETRY)
        .allowedTags(REEF_TAGS)
        .done()
    .addCamera("Middle")
        .positionInches(8, 0, 41)
        .rotationDegrees(0, -43, 177)
        .purpose(CameraPurpose.ALIGNMENT)
        .allowedTags(HP_TAGS)
        .done()
    .ambiguityThreshold(0.25)
    .maxPoseJump(10.0)
    .build();
```

### 3. Initialize in Robot

```java
// In RobotContainer.java
swerve.setupPhotonVision(VisionConstants.VISION_CONFIG);
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
    .positionMeters(x, y, z)           // Position in meters
    .positionInches(x, y, z)           // Position in inches (auto-converted)
    .rotationRadians(roll, pitch, yaw) // Rotation in radians
    .rotationDegrees(roll, pitch, yaw) // Rotation in degrees (auto-converted)
    .purpose(CameraPurpose)            // Camera purpose (default: BOTH)
    .singleTagStdDevs(x, y, theta)     // Single tag std devs
    .singleTagStdDevs(VisionStdDevs)   // Single tag std devs (preset)
    .multiTagStdDevs(x, y, theta)      // Multi tag std devs
    .multiTagStdDevs(VisionStdDevs)    // Multi tag std devs (preset)
    .allowedTags(int... tagIDs)        // Tag filtering
    .done()                            // Return to VisionConfigBuilder
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
- **Single tag at distance > 4m** - Rejected (MAX_VALUE std devs)

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
// 2025 Configuration
public static final int[] REEF_TAGS = {6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22};

// 2026 Configuration (hypothetical)
public static final int[] SCORING_TAGS = {5, 6, 7, 14, 15, 16};  // New game tags
public static final int[] INTAKE_TAGS = {1, 2, 3, 11, 12, 13};   // New game tags
```

---

## Examples

### Example 1: Two-Camera Setup

```java
VisionSystemConfig config = VisionConfigBuilder.create()
    .addCamera("FrontLeft")
        .positionInches(12, 10, 8)
        .rotationDegrees(0, -15, -30)
        .purpose(CameraPurpose.BOTH)
        .done()
    .addCamera("FrontRight")
        .positionInches(12, -10, 8)
        .rotationDegrees(0, -15, 30)
        .purpose(CameraPurpose.BOTH)
        .done()
    .build();
```

### Example 2: Specialized Cameras

```java
VisionSystemConfig config = VisionConfigBuilder.create()
    // Wide-angle cameras for odometry
    .addCamera("OdometryLeft")
        .positionInches(10, 12, 6)
        .rotationDegrees(0, 0, -45)
        .purpose(CameraPurpose.ODOMETRY)
        .done()
    .addCamera("OdometryRight")
        .positionInches(10, -12, 6)
        .rotationDegrees(0, 0, 45)
        .purpose(CameraPurpose.ODOMETRY)
        .done()
    // Narrow camera for scoring alignment
    .addCamera("ScoringCam")
        .positionInches(14, 0, 24)
        .rotationDegrees(0, -30, 0)
        .purpose(CameraPurpose.ALIGNMENT)
        .allowedTags(SCORING_TAGS)
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
        .positionInches(8, 0, 12)
        .rotationDegrees(0, -20, 0)
        .purpose(CameraPurpose.ODOMETRY)
        .singleTagStdDevs(VisionStdDevs.HIGH_CONFIDENCE)
        .multiTagStdDevs(new VisionStdDevs(0.05, 0.05, 0.1))
        .done()
    .addCamera("LowConfidenceCam")
        .positionInches(-6, 0, 30)
        .rotationDegrees(0, -45, 180)
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
