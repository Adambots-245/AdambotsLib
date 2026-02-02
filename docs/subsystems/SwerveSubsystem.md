# SwerveSubsystem

YAGSL-based swerve drive subsystem for holonomic drivetrain control with PhotonVision integration.

## Table of Contents

- [Overview](#overview)
- [Getting Started](#getting-started)
- [SwerveConfig](#swerveconfig)
- [JSON Configuration](#json-configuration)
- [Zeroing Swerve Modules](#zeroing-swerve-modules)
- [Command Factories](#command-factories)
- [Game Target Configuration](#game-target-configuration)
- [Trigger Methods](#trigger-methods)
- [Usage Examples](#usage-examples)
- [Troubleshooting](#troubleshooting)

---

## Overview

`SwerveSubsystem` is a comprehensive swerve drive implementation using YAGSL (Yet Another Generic Swerve Library). Unlike traditional approaches that require custom `SwerveModule` classes, YAGSL manages all swerve module hardware and kinematics through JSON configuration files.

**Key Features:**
- ✅ JSON-based configuration (no custom SwerveModule class needed)
- ✅ PhotonVision integration for vision-corrected odometry
- ✅ PathPlanner integration for autonomous path following
- ✅ Game-specific target configuration (JSON or builder pattern)
- ✅ 26 command factory methods for all drive operations
- ✅ 18 trigger methods for state-based command composition
- ✅ Support for multiple motor types (Kraken X60/X44, Falcon 500, NEO)

**Important:** AdambotsLib provides the `SwerveSubsystem` class and example JSON files, but **each robot project must create its own JSON configuration files** with robot-specific CAN IDs, dimensions, and motor types.

---

## Getting Started

### Step 1: Copy Example JSON Files

1. Navigate to AdambotsLib's example configuration:
   ```
   AdambotsLib/src/main/deploy/swerve/kraken/
   ```

2. Copy the entire `kraken` folder to your robot project:
   ```
   YourRobotProject/src/main/deploy/swerve/kraken/
   ```

3. Your robot project structure should look like:
   ```
   YourRobotProject/
   └── src/main/deploy/swerve/
       └── kraken/
           ├── swervedrive.json
           ├── controllerproperties.json
           └── modules/
               ├── frontleft.json
               ├── frontright.json
               ├── backleft.json
               ├── backright.json
               ├── pidfproperties.json
               └── physicalproperties.json
   ```

### Step 2: Create SwerveSubsystem in Your Robot

```java
import com.adambots.lib.subsystems.SwerveSubsystem;
import com.adambots.lib.subsystems.SwerveConfig;
import edu.wpi.first.wpilibj.Filesystem;
import java.io.File;

public class RobotContainer {
  private final SwerveSubsystem swerve;

  public RobotContainer() {
    // Option 1: Use default configuration
    swerve = new SwerveSubsystem(
      new File(Filesystem.getDeployDirectory(), "swerve/kraken")
    );

    // Option 2: With custom PathPlanner PID and behavior settings
    SwerveConfig config = new SwerveConfig()
      .withTranslationPID(5.0, 0.0, 0.0)  // PathPlanner translation PID
      .withRotationPID(3.0, 0.0, 0.0)      // PathPlanner rotation PID
      .withHeadingCorrection(false)
      .withCosineCompensation(true);

    swerve = new SwerveSubsystem(
      new File(Filesystem.getDeployDirectory(), "swerve/kraken"),
      config
    );

    configureButtonBindings();
  }

  private void configureButtonBindings() {
    // Set default drive command
    swerve.setDefaultCommand(
      swerve.driveCommand(
        () -> -controller.getLeftY(),
        () -> -controller.getLeftX(),
        () -> -controller.getRightX()
      )
    );
  }
}
```

### Step 3: Configure JSON Files for Your Robot

See [JSON Configuration](#json-configuration) section below.

---

## SwerveConfig

`SwerveConfig` allows you to customize PathPlanner PID values and drive behavior settings without modifying AdambotsLib source code.

### Why Use SwerveConfig?

Previously, PathPlanner PID values were hardcoded in AdambotsLib's Constants file, making it impossible for robot projects to customize them without recreating the entire AutoBuilder setup. `SwerveConfig` solves this by providing a clean builder-style API.

### Available Settings

| Setting | Method | Default | Description |
|---------|--------|---------|-------------|
| Translation PID | `withTranslationPID(p, i, d)` | 5.0, 0, 0 | PathPlanner X/Y position following |
| Rotation PID | `withRotationPID(p, i, d)` | 5.0, 0, 0 | PathPlanner heading following |
| Feedforward | `withFeedforward(bool)` | true | Use feedforward in PathPlanner auto |
| Heading Correction | `withHeadingCorrection(bool)` | false | Auto-maintain heading during teleop |
| Cosine Compensation | `withCosineCompensation(bool)` | true | Adjust speed based on wheel alignment |
| Angular Velocity Comp | `withAngularVelocityCompensation(bool, coeff)` | true, 0.1 | Counteract skewing during rotation |

### Basic Usage

```java
// In Constants.java
public static final SwerveConfig SWERVE_CONFIG = new SwerveConfig()
    .withTranslationPID(5.0, 0.0, 0.0)
    .withRotationPID(3.0, 0.0, 0.0);

// In RobotContainer.java
SwerveSubsystem swerve = new SwerveSubsystem(
    new File(Filesystem.getDeployDirectory(), "swerve/kraken"),
    Constants.SWERVE_CONFIG
);
```

### PathPlanner PID Tuning

The translation and rotation PID values control how accurately the robot follows PathPlanner autonomous paths.

#### Translation PID (X/Y Position)

Controls how the robot corrects position errors during path following.

```java
.withTranslationPID(5.0, 0.0, 0.0)  // Default
```

**Tuning Tips:**
- **Start with P=5.0** - This is a good baseline for most robots
- **Increase P** if the robot is sluggish or cuts corners
- **Decrease P** if the robot oscillates around the path
- **Add D (0.1-0.5)** to reduce overshoot at waypoints
- **I is rarely needed** - avoid unless you have persistent steady-state error

#### Rotation PID (Heading)

Controls how the robot corrects heading errors during path following.

```java
.withRotationPID(3.0, 0.0, 0.0)  // Slightly lower than translation
```

**Tuning Tips:**
- **Start with P=3.0-5.0** - Rotation typically needs less gain
- **Increase P** if the robot doesn't turn sharply enough
- **Decrease P** if heading oscillates after turns
- **Add D (0.1-0.5)** if heading overshoots target

### PathPlanner Feedforward

Controls whether PathPlanner uses feedforward-based driving for autonomous paths.

```java
.withFeedforward(true)   // Default: enabled
.withFeedforward(false)  // Disable for simulation
```

**When Enabled (default):**
```java
swerveDrive.drive(speedsRobotRelative, swerveModuleStates, moduleFeedForwards.linearForces());
```

**When Disabled:**
```java
swerveDrive.setChassisSpeeds(speedsRobotRelative);
```

**When to Disable:**
- Running in simulation with **maple-sim** (feedforward doesn't work properly)
- Debugging autonomous path following issues
- If feedforward causes erratic behavior

**Simulation-Aware Configuration:**
```java
SwerveConfig config = new SwerveConfig()
    .withTranslationPID(5.0, 0.0, 0.0)
    .withRotationPID(5.0, 0.0, 0.0)
    .withFeedforward(!RobotBase.isSimulation());  // Auto-disable in sim
```

---

### Drive Behavior Settings

#### Heading Correction

When enabled, the robot automatically maintains its heading during teleop driving.

```java
.withHeadingCorrection(true)
```

**When to Enable:**
- Robot tends to drift rotationally during straight-line driving
- You want the robot to resist external rotation forces

**When to Disable (default):**
- Driver prefers full manual control
- Heading correction causes jerky behavior

#### Cosine Compensation

Adjusts wheel speeds based on their alignment with the desired movement direction.

```java
.withCosineCompensation(true)  // Default: enabled
```

**Note:** Does not work in simulation. Test on real robot.

#### Angular Velocity Compensation

Counteracts the skewing effect when the robot translates and rotates simultaneously.

```java
.withAngularVelocityCompensation(true, 0.1)  // enabled, coefficient
```

### Full Configuration Example

```java
public static final SwerveConfig SWERVE_CONFIG = new SwerveConfig()
    // PathPlanner autonomous PID
    .withTranslationPID(5.0, 0.0, 0.0)
    .withRotationPID(3.0, 0.0, 0.5)  // Added D for heading stability
    .withFeedforward(!RobotBase.isSimulation())  // Disable in sim for maple-sim

    // Drive behavior
    .withHeadingCorrection(false)
    .withCosineCompensation(true)
    .withAngularVelocityCompensation(true, 0.1);
```

### Accessing Config at Runtime

```java
// Get the current configuration
SwerveConfig config = swerve.getSwerveConfig();

// Check settings (for debugging/logging)
PIDConstants translationPID = config.getTranslationPID();
boolean headingCorrection = config.isHeadingCorrectionEnabled();
```

---

## JSON Configuration

YAGSL uses JSON files to configure all aspects of your swerve drive. Here's what each file does and how to configure them.

### File Structure

```
swerve/kraken/
├── swervedrive.json          # Main config: IMU and module list
├── controllerproperties.json # Joystick and heading PID
└── modules/
    ├── frontleft.json        # Per-module: motors, encoders, offsets
    ├── frontright.json
    ├── backleft.json
    ├── backright.json
    ├── pidfproperties.json   # Drive and angle PID tuning
    └── physicalproperties.json # Gear ratios, wheel size, limits
```

---

### 1. swervedrive.json

**Purpose:** Defines the IMU (gyroscope) and lists all swerve modules.

```json
{
  "imu": {
    "type": "pigeon2",
    "id": 6,
    "canbus": null
  },
  "invertedIMU": false,
  "modules": [
    "frontleft.json",
    "frontright.json",
    "backleft.json",
    "backright.json"
  ]
}
```

**Configuration Steps:**
1. Set `"id"` to your Pigeon2's CAN ID
2. Set `"canbus"` to `"canivore"` if using CANivore, or `null` for rio CAN bus
3. Set `"invertedIMU"` to `true` if your gyro readings are inverted
4. List all module JSON files in the `"modules"` array

**Supported IMU Types:**
- `"pigeon2"` - CTRE Pigeon 2.0 (recommended)
- `"navx"` - NavX gyroscope
- `"adis16448"`, `"adis16470"` - Analog Devices IMUs

---

### 2. Module Configuration (frontleft.json, etc.)

**Purpose:** Defines motors, encoders, inversions, absolute encoder offset, and physical location for each module.

**Example: All Kraken Configuration (X60 drive, X44 angle)**

```json
{
  "drive": {
    "type": "talonfx",
    "id": 1,
    "canbus": null
  },
  "angle": {
    "type": "talonfx",
    "id": 5,
    "canbus": null
  },
  "encoder": {
    "type": "cancoder",
    "id": 9,
    "canbus": null
  },
  "inverted": {
    "drive": false,
    "angle": false
  },
  "absoluteEncoderOffset": 0.0,
  "location": {
    "front": 12.0,
    "left": 12.0
  }
}
```

**Configuration Steps:**

#### Motor Types
- **Kraken X60 or Falcon 500:** `"type": "talonfx"`
- **NEO or NEO 550:** `"type": "sparkmax_neo"`
- **NEO V1.1:** `"type": "sparkmax_neo_v1.1"`
- **NEO Vortex:** `"type": "sparkmax_neo_vortex"`

#### CAN IDs
Replace the `"id"` values with your actual CAN IDs:

**Front Left Module:**
- Drive motor: CAN ID (e.g., 1)
- Angle motor: CAN ID (e.g., 5)
- CANcoder: CAN ID (e.g., 9)

**Front Right Module:**
- Drive motor: CAN ID (e.g., 2)
- Angle motor: CAN ID (e.g., 6)
- CANcoder: CAN ID (e.g., 10)

**Back Left Module:**
- Drive motor: CAN ID (e.g., 3)
- Angle motor: CAN ID (e.g., 7)
- CANcoder: CAN ID (e.g., 11)

**Back Right Module:**
- Drive motor: CAN ID (e.g., 4)
- Angle motor: CAN ID (e.g., 8)
- CANcoder: CAN ID (e.g., 12)

#### Motor Inversions
Set `"inverted": true` or `false` for drive and angle motors. Test drive to verify:
- **Drive motors:** All modules should drive in the same direction when given positive input
- **Angle motors:** All modules should turn in the same direction (typically counterclockwise when viewed from above)

#### Absolute Encoder Offset
This is configured during the **zeroing process** (see [Zeroing Swerve Modules](#zeroing-swerve-modules)). Leave at `0.0` initially.

#### Module Locations
Specify the distance from the robot center to each module in **inches**:
```json
"location": {
  "front": 12.0,  // inches from center to front
  "left": 12.0    // inches from center to left (positive = left side)
}
```

**Examples:**
- Front Left: `"front": 12.0, "left": 12.0`
- Front Right: `"front": 12.0, "left": -12.0`
- Back Left: `"front": -12.0, "left": 12.0`
- Back Right: `"front": -12.0, "left": -12.0`

---

### 3. physicalproperties.json

**Purpose:** Defines gear ratios, wheel diameter, current limits, and physical constraints.

```json
{
  "conversionFactors": {
    "angle": {
      "gearRatio": 21.43,
      "factor": 16.8
    },
    "drive": {
      "gearRatio": 5.9,
      "diameter": 4.0,
      "factor": 0.0542
    }
  },
  "currentLimit": {
    "drive": 60,
    "angle": 40
  },
  "rampRate": {
    "drive": 0.25,
    "angle": 0.25
  },
  "wheelGripCoefficientOfFriction": 1.19,
  "optimalVoltage": 12
}
```

**Configuration Steps:**

#### Gear Ratios
Set based on your swerve module type:

**SDS Mk4i Modules:**
- L1 (slow): Drive `8.14:1`, Angle `21.43:1`
- L2 (standard): Drive `6.75:1`, Angle `21.43:1`
- L3 (fast): Drive `6.12:1`, Angle `21.43:1`

**SDS Mk4 Modules:**
- L1: Drive `8.14:1`, Angle `12.8:1`
- L2: Drive `6.75:1`, Angle `12.8:1`
- L3: Drive `6.12:1`, Angle `12.8:1`

**SDS Mk3 Modules:**
- Standard: Drive `6.86:1`, Angle `12.8:1`
- Fast: Drive `5.14:1`, Angle `12.8:1`

#### Wheel Diameter
Measure your wheel diameter in **inches**. Common sizes:
- 4.0 inches (Colson wheels)
- 3.0 inches (smaller wheels)

**Note:** Wheels wear down over time - measure actual diameter for accuracy.

#### Current Limits
Set appropriate current limits in **amps**:

**Kraken X60/X44:**
- Drive: 60-80A (continuous), can spike higher
- Angle: 40A (lower load)

**Falcon 500:**
- Drive: 40-60A
- Angle: 30-40A

**NEO:**
- Drive: 40-50A
- Angle: 20-30A

**Tip:** Start conservative, increase if you need more performance.

#### Ramp Rate
Time in seconds for motor to ramp from 0 to full power:
- `0.25` seconds is a good starting point
- Lower = more responsive but jerky
- Higher = smoother but sluggish

#### Conversion Factors
These are calculated automatically by YAGSL based on gear ratios and wheel diameter. You can leave the `"factor"` fields - YAGSL will compute them.

---

### 4. pidfproperties.json

**Purpose:** PID tuning for drive velocity control and angle position control.

```json
{
  "drive": {
    "p": 0.1,
    "i": 0.0,
    "d": 0.0,
    "f": 0.0,
    "iz": 0
  },
  "angle": {
    "p": 0.01,
    "i": 0.0,
    "d": 0.0,
    "f": 0.0,
    "iz": 0
  }
}
```

**Configuration Steps:**

#### Drive PID (Velocity Control)
Tunes how accurately the modules follow commanded speeds.

**Starting Values:**
- Kraken: `p=0.1, i=0, d=0, f=0`
- Falcon: `p=0.1, i=0, d=0.05, f=0`
- NEO: `p=0.0001, i=0, d=0, f=0` (much smaller due to different units)

**Tuning Process:**
1. Start with P only
2. Increase P until oscillation
3. Reduce P by 30-50%
4. Add D if needed to reduce oscillation
5. Add I only if steady-state error exists (rarely needed)

#### Angle PID (Position Control)
Tunes how quickly and accurately modules rotate to target angles.

**Starting Values:**
- Kraken: `p=0.01, i=0, d=0, f=0`
- Falcon: `p=0.01, i=0, d=0.1, f=0`
- NEO: `p=0.025, i=0, d=1.25, f=0`

**Tuning Process:**
1. Start with small P value (0.01)
2. Increase P until modules snap to angles quickly
3. If oscillation occurs, add D term
4. Angle control typically needs higher D than drive

**Tip:** Use Phoenix Tuner or REV Hardware Client to tune PID values live, then copy final values to JSON.

---

### 5. controllerproperties.json

**Purpose:** Joystick deadband and heading controller PID.

```json
{
  "angleJoystickRadiusDeadband": 0.1,
  "heading": {
    "p": 0.001,
    "i": 0,
    "d": 0
  }
}
```

**Configuration Steps:**

#### Joystick Deadband
Eliminates joystick drift when centered:
- `0.0` = no deadband (sensitive, may drift)
- `0.1` = 10% deadband (good default)
- `0.2` = 20% deadband (less sensitive)

#### Heading PID
Controls how the robot maintains or rotates to target headings:
- Start with `p=0.001, i=0, d=0`
- Increase P if heading response is too slow
- Add D if heading oscillates

---

## Zeroing Swerve Modules

**Zeroing** is the process of calibrating the absolute encoder offsets so that each module knows its true angle at startup. This is **critical** for proper swerve operation.

### Why Zeroing is Needed

- CANcoders measure absolute position but don't know the module's physical orientation
- We need to tell YAGSL: "When the CANcoder reads X degrees, the wheel is actually pointing forward"
- Without correct offsets, modules will point in wrong directions

### Zeroing Process

#### Step 1: Physically Align All Modules

1. **Power off the robot**
2. **Manually rotate each module** so the bevel gear faces inward (toward robot center)
3. All wheels should be pointing **straight forward** (parallel to robot's front)
4. Use a straight edge or measuring tool to ensure alignment

**Visual Check:**
```
Front of Robot
     ↑
[FL] ↑ [FR]
     ↑
[BL] ↑ [BR]
     ↑
```
All wheels should point up (↑) when looking from above.

#### Step 2: Read CANcoder Values

1. **Power on the robot**
2. **Open Phoenix Tuner** (for CANcoders)
3. **Select each CANcoder** and note its current reading
4. Record the values:
   - Front Left CANcoder: _____ degrees
   - Front Right CANcoder: _____ degrees
   - Back Left CANcoder: _____ degrees
   - Back Right CANcoder: _____ degrees

**Alternative:** Use SmartDashboard or Shuffleboard if you've published CANcoder values.

#### Step 3: Update JSON with Offset Values

Open each module JSON file and set `"absoluteEncoderOffset"` to the recorded value:

**frontleft.json:**
```json
{
  "absoluteEncoderOffset": 119.5,
  ...
}
```

**frontright.json:**
```json
{
  "absoluteEncoderOffset": 45.2,
  ...
}
```

Repeat for all four modules.

#### Step 4: Verify Zeroing

1. **Deploy code** with updated offsets
2. **Enable the robot** in teleop mode
3. **Give gentle forward drive input**
4. **All modules should drive forward** without rotating first

**If a module rotates 90°, 180°, or 270° before driving:**
- The offset is incorrect
- Re-check physical alignment and CANcoder reading
- Adjust the offset in JSON

**If a module drives backward:**
- The drive motor inversion is wrong
- Flip `"inverted": {"drive": true/false}` in that module's JSON

### Common Zeroing Issues

| Problem | Solution |
|---------|----------|
| Module rotates 180° before driving | Add or subtract 180 from offset |
| Module rotates 90° before driving | Re-check physical alignment, wheels must be perfectly straight |
| Module drives backward | Invert drive motor: `"inverted": {"drive": true}` |
| Module angle doesn't respond | Check angle motor CAN ID and wiring |
| CANcoder value doesn't change | Check CANcoder CAN ID and wiring |

---

## Command Factories

`SwerveSubsystem` provides 17 command factory methods for all drive operations. These replace the old `DriveCommands` utility class.

### Vision-Based Commands

#### aimAtAprilTagCommand(int tagId, double tolerance)
Rotates robot to aim at a specific AprilTag.

```java
// Aim at tag 4 with 2° tolerance
Command aimCommand = swerve.aimAtAprilTagCommand(4, 2.0);
```

**Parameters:**
- `tagId` - AprilTag ID to aim at
- `tolerance` - Angle tolerance in degrees (command ends when within tolerance)

**Use Case:** Aligning with game pieces or scoring positions.

---

#### alignAndStrafeCommand(int tagId, double strafeDistance, double strafeSpeed, double alignmentTolerance)
Aligns with AprilTag while strafing to a target distance.

```java
// Align with tag 7, strafe to 2.5m away at 1 m/s
Command alignStrafe = swerve.alignAndStrafeCommand(7, 2.5, 1.0, 2.0);
```

**Parameters:**
- `tagId` - AprilTag to align with
- `strafeDistance` - Target distance in meters
- `strafeSpeed` - Strafe speed in m/s
- `alignmentTolerance` - Angle tolerance in degrees

**Use Case:** Approaching scoring positions while maintaining alignment.

---

#### driveToNearestPoseWithVisionCommand(List<Pose2d> targetPoses)
Drives to the nearest pose from a list using PathPlanner pathfinding with vision updates.

```java
List<Pose2d> scoringPositions = List.of(
  new Pose2d(2.0, 3.0, Rotation2d.fromDegrees(0)),
  new Pose2d(5.0, 3.0, Rotation2d.fromDegrees(0))
);

Command driveToNearest = swerve.driveToNearestPoseWithVisionCommand(scoringPositions);
```

**Use Case:** Autonomous driving to flexible target locations.

---

#### aimAtTargetCommand(String cameraName)
Aims at the best detected target from a specific camera.

```java
Command aimAtBest = swerve.aimAtTargetCommand("Center");
```

**Use Case:** Quick target acquisition from specific camera perspective.

---

#### getDistanceFromAprilTagCommand(int tagID)
Prints the distance to a specific AprilTag (debugging command).

```java
Command printDistance = swerve.getDistanceFromAprilTagCommand(4);
```

---

### Vision Control Commands

#### enableVisionCommand() / disableVisionCommand()
Enable or disable all vision cameras.

```java
Command enableVision = swerve.enableVisionCommand();
Command disableVision = swerve.disableVisionCommand();
```

**Use Case:** Disable vision during teleop if it interferes with driving, enable for auto.

---

#### Camera Purpose Control
Control cameras by their configured purpose using PhotonVision directly:

```java
// Enable/disable cameras by purpose
swerve.vision.enableCamerasWithPurpose(CameraPurpose.ODOMETRY);
swerve.vision.disableCamerasWithPurpose(CameraPurpose.ALIGNMENT);

// Enable/disable specific cameras by name
swerve.vision.enableCamera("Left");
swerve.vision.disableCamera("Middle");
```

**Use Case:** Switch camera focus based on robot location or operation mode.

---

### PathPlanner Commands

#### driveToPoseCommand(Pose2d pose)
Drives to a target pose using PathPlanner pathfinding.

```java
Pose2d target = new Pose2d(3.0, 2.0, Rotation2d.fromDegrees(90));
Command driveToPose = swerve.driveToPoseCommand(target);
```

**Use Case:** Autonomous navigation to specific field positions.

---

#### getAutonomousCommand(String pathName)
Executes a named PathPlanner auto path.

```java
Command auto = swerve.getAutonomousCommand("MyAutoPath");
```

**Use Case:** Running autonomous routines created in PathPlanner GUI.

---

### Manual Drive Commands

#### driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier angularRotationX)
Standard field-oriented drive with rotation control.

```java
Command drive = swerve.driveCommand(
  () -> -controller.getLeftY(),    // Forward/backward
  () -> -controller.getLeftX(),    // Left/right
  () -> -controller.getRightX()    // Rotation
);

swerve.setDefaultCommand(drive);
```

**Use Case:** Teleop driving with joystick control.

---

#### driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier headingX, DoubleSupplier headingY)
Field-oriented drive with heading targeting (right stick aims robot).

```java
Command driveWithHeading = swerve.driveCommand(
  () -> -controller.getLeftY(),    // Forward/backward
  () -> -controller.getLeftX(),    // Left/right
  () -> -controller.getRightX(),   // Heading X
  () -> -controller.getRightY()    // Heading Y
);
```

**Use Case:** Aim robot with right stick while driving with left stick.

---

#### driveFieldOrientedCommand(ChassisSpeeds fieldRelativeSpeeds)
Direct field-oriented driving with ChassisSpeeds.

```java
ChassisSpeeds speeds = new ChassisSpeeds(1.0, 0.0, 0.0); // 1 m/s forward
Command driveFieldOriented = swerve.driveFieldOrientedCommand(speeds);
```

**Use Case:** Programmatic driving with specific velocities.

---

#### centerModulesCommand()
Stops robot and centers all modules (wheels pointing forward).

```java
Command center = swerve.centerModulesCommand();
```

**Use Case:** Resetting module positions or stopping smoothly.

---

### Distance-Based Commands

#### driveToDistanceCommand(double distanceInMeters, double speedInMetersPerSecond)
Drives forward/backward a specific distance (robot-relative).

```java
Command driveForward = swerve.driveToDistanceCommand(2.0, 1.5); // 2m at 1.5 m/s
```

**Use Case:** Simple autonomous movements.

---

#### driveToDistanceFieldOrientedCommand(double distanceInMeters, double speedInMetersPerSecond)
Drives a specific distance using the robot's current field-relative heading.

```java
// Drive 3 meters at 2 m/s in the direction the robot is currently facing
Command driveFieldOriented = swerve.driveToDistanceFieldOrientedCommand(3.0, 2.0);
```

**Note:** The direction is determined by the robot's heading at the start of the command. Use negative speed to drive backward.

---

#### driveForwardDistanceCommand(double distanceInMeters, double speedInMetersPerSecond)
Drives forward (field-relative) a specific distance.

```java
Command driveForward = swerve.driveForwardDistanceCommand(1.5, 1.0);
```

---

### Simple Movement Commands

#### turnToAngleCommand(Rotation2d targetAngle, double toleranceDegrees)
Rotates robot to face a specific field-relative angle.

```java
// Turn to face 0 degrees (toward red alliance wall)
swerve.turnToAngleCommand(Rotation2d.fromDegrees(0), 2.0).schedule();

// Turn to face speaker before shooting
Commands.sequence(
    swerve.turnToAngleCommand(Rotation2d.fromDegrees(180), 1.0),
    shooter.shootCommand()
);

// Compose turn-then-drive sequence
Commands.sequence(
    swerve.turnToAngleCommand(Rotation2d.fromDegrees(45), 2.0),
    swerve.driveToDistanceCommand(1.5, 2.0)
);
```

**Parameters:**
- `targetAngle` - Field-relative target angle as Rotation2d
- `toleranceDegrees` - Angle tolerance in degrees (command ends when within tolerance)

**Use Case:** Pre-positioning for shots, autonomous alignment, resetting robot orientation.

---

#### strafeCommand(double distanceMeters, double speedMetersPerSecond)
Strafes (moves sideways) a specified distance while maintaining current heading.

```java
// Strafe 0.5 meters left at 1.0 m/s
swerve.strafeCommand(0.5, 1.0).schedule();

// Strafe 0.3 meters right at 0.5 m/s
swerve.strafeCommand(-0.3, 0.5).schedule();

// Align then strafe sequence
Commands.sequence(
    swerve.turnToAngleCommand(Rotation2d.fromDegrees(90), 2.0),
    swerve.strafeCommand(0.4, 0.8)
);
```

**Parameters:**
- `distanceMeters` - Distance to strafe in meters (positive = left, negative = right)
- `speedMetersPerSecond` - Speed in meters per second (always positive)

**Use Case:** Manual positioning adjustments, aligning with game pieces or scoring positions.

---

#### creepForwardCommand() / creepCommand(double speedMetersPerSecond)
Slow, continuous movement for fine positioning.

```java
// Creep forward while button is held (0.3 m/s default)
Buttons.XboxAButton.whileTrue(swerve.creepForwardCommand());

// Creep at custom speed while button is held
Buttons.XboxLeftBumper.whileTrue(swerve.creepCommand(0.2));

// Creep backward slowly
Buttons.XboxRightBumper.whileTrue(swerve.creepCommand(-0.2));

// Creep until sensor detects game piece
swerve.creepForwardCommand()
    .until(() -> intake.hasGamePiece())
    .schedule();
```

**Parameters (creepCommand only):**
- `speedMetersPerSecond` - Speed in m/s (positive = forward, negative = backward)

**Note:** These commands run indefinitely until cancelled. Use with `.until()`, `.withTimeout()`, or `.whileTrue()` for controlled execution.

**Use Case:** Slow approach to game pieces, fine-tuning position before scoring.

---

#### driveToPositionWithHeadingCommand(Translation2d targetPosition, Rotation2d heading, double positionToleranceMeters)
Drives to a position while maintaining a specified heading (decoupled from path rotation).

```java
// Drive to position (2, 5) while facing 90 degrees
swerve.driveToPositionWithHeadingCommand(
    new Translation2d(2.0, 5.0),
    Rotation2d.fromDegrees(90),
    0.1
).schedule();

// Approach speaker while facing it
swerve.driveToPositionWithHeadingCommand(
    new Translation2d(1.5, 5.5),
    Rotation2d.fromDegrees(180),
    0.05
).schedule();
```

**Parameters:**
- `targetPosition` - Target X,Y position on the field
- `heading` - Desired heading to maintain throughout the movement
- `positionToleranceMeters` - Position tolerance in meters (command ends when within this)

**Key Difference from driveToPoseCommand:**
- `driveToPositionWithHeadingCommand`: Maintains specified heading throughout entire movement
- `driveToPoseCommand`: Rotates as part of the path trajectory (PathPlanner controls rotation)

**Use Case:** Drive to pickup spot while facing the game piece, approach scoring while facing target.

---

### SysId Commands

#### sysIdDriveMotorCommand() / sysIdAngleMotorCommand()
System identification commands for characterizing drive and angle motors.

```java
Command sysIdDrive = swerve.sysIdDriveMotorCommand();
Command sysIdAngle = swerve.sysIdAngleMotorCommand();
```

**Use Case:** Measuring motor constants (kS, kV, kA) for feedforward control.

---

## Game Target Configuration

Game-specific targets allow you to define semantic names (like "tower", "hub", "source") that map to AprilTag positions with robot offsets. This eliminates the need to hardcode field positions that change every year.

### Why Use Game Targets?

- **Semantic names**: Use `"tower-blue"` instead of remembering tag ID 15
- **Alliance-aware**: Automatically select blue/red targets based on alliance
- **Offset-based**: Define where the robot should be relative to the tag
- **Tolerance support**: Built-in alignment checking

### Setup Methods

#### JSON Configuration (Recommended)

Create a JSON file in your deploy folder:

**deploy/gametargets.json:**
```json
{
  "gameYear": 2026,
  "gameName": "REBUILT",
  "targets": [
    {
      "name": "tower-blue",
      "tagIds": [15, 16],
      "offset": { "x": -0.5, "y": 0.0, "rotation": 180 },
      "tolerance": { "position": 0.05, "rotation": 2.0 }
    },
    {
      "name": "tower-red",
      "tagIds": [31, 32],
      "offset": { "x": -0.5, "y": 0.0, "rotation": 0 },
      "tolerance": { "position": 0.05, "rotation": 2.0 }
    }
  ]
}
```

Load in RobotContainer:
```java
swerve.setupGameTargets("gametargets.json");
```

#### Builder Pattern (Code-Based)

```java
import com.adambots.lib.targets.GameTargetConfigBuilder;
import static edu.wpi.first.units.Units.*;

GameTargetConfig config = GameTargetConfigBuilder.create()
    .gameYear(2026)
    .gameName("REBUILT")
    .addTarget("tower-blue")
        .tagIds(15, 16)
        .offset(Meters.of(-0.5), Meters.of(0), Degrees.of(180))
        .tolerance(0.05, 2.0)
        .done()
    .addTarget("tower-red")
        .tagIds(31, 32)
        .offset(Meters.of(-0.5), Meters.of(0), Degrees.of(0))
        .tolerance(0.05, 2.0)
        .done()
    .build();

swerve.setupGameTargets(config);
```

### Game Target Commands

#### driveToTargetCommand(String targetName)
Drives to a named game target using PathPlanner pathfinding.

```java
// Drive to the tower scoring position
Buttons.XboxAButton.onTrue(swerve.driveToTargetCommand("tower-blue"));
```

---

#### driveToAllianceTargetCommand(String baseName)
Drives to the alliance-appropriate target (auto-selects blue/red).

```java
// Drives to "tower-blue" or "tower-red" based on alliance
Buttons.XboxBButton.onTrue(swerve.driveToAllianceTargetCommand("tower"));
```

---

#### aimAtGameTargetCommand(String targetName, double toleranceDegrees)
Rotates to face the target position (does not drive toward it).

```java
// Aim at tower while in range
swerve.isInRangeOfTargetTrigger("tower-blue", 2.0, 5.0)
    .whileTrue(swerve.aimAtGameTargetCommand("tower-blue", 2.0));
```

---

#### aimAtAllianceGameTargetCommand(String baseName, double toleranceDegrees)
Aims at the alliance-appropriate target.

```java
// Aim at alliance tower
swerve.aimAtAllianceGameTargetCommand("tower", 2.0);
```

### Game Target Triggers

#### isAtTargetTrigger(String targetName)
True when robot is at the target position within tolerance.

```java
swerve.isAtTargetTrigger("tower-blue")
    .onTrue(shooter.shootCommand());
```

---

#### isAtAllianceTargetTrigger(String baseName)
True when at alliance-specific target.

```java
swerve.isAtAllianceTargetTrigger("tower")
    .onTrue(Commands.print("At alliance tower!"));
```

---

#### isInRangeOfTargetTrigger(String targetName, double minDistance, double maxDistance)
True when within distance range of target.

```java
swerve.isInRangeOfTargetTrigger("tower-blue", 1.0, 3.0)
    .whileTrue(swerve.aimAtGameTargetCommand("tower-blue", 2.0));
```

---

#### isAlignedWithTargetTrigger(String targetName, Angle tolerance)
True when robot heading matches target rotation.

```java
swerve.isAlignedWithTargetTrigger("tower-blue", Degrees.of(3.0))
    .onTrue(shooter.spinUpCommand());
```

### Getting Target Poses

```java
// Get specific target pose
Optional<Pose2d> towerPose = swerve.getTargetPose("tower-blue");

// Get alliance-appropriate target pose
Optional<Pose2d> allianceTowerPose = swerve.getAllianceTargetPose("tower");

// Use in custom commands
towerPose.ifPresent(pose -> {
    System.out.println("Tower at: " + pose);
});
```

---

## Trigger Methods

`SwerveSubsystem` provides 18 trigger methods for state-based command composition.

### Pose-Based Triggers

#### atPoseTrigger(Pose2d targetPose, double positionToleranceMeters, double rotationToleranceDegrees)
Returns a trigger that is true when robot is within tolerance of target pose.

```java
Pose2d scoringPose = new Pose2d(2.0, 3.0, Rotation2d.fromDegrees(0));

// Trigger when within 0.1m and 5° of scoring pose
Trigger atScoringPose = swerve.atPoseTrigger(scoringPose, 0.1, 5.0);

atScoringPose.onTrue(intake.scoreCommand());
```

---

#### inRegionTrigger(double minX, double maxX, double minY, double maxY)
Returns a trigger that is true when robot is within a rectangular field region.

```java
// Trigger when in community zone (x: 0-3m, y: 2-5m)
Trigger inCommunity = swerve.inRegionTrigger(0, 3, 2, 5);

inCommunity.whileTrue(leds.setColorCommand(Color.kBlue));
```

---

### Velocity-Based Triggers

#### isMovingTrigger() / isMovingTrigger(double velocityThreshold)
Returns a trigger that is true when robot is moving (linear velocity above threshold).

```java
// Convenience: uses 0.1 m/s default
Trigger isMoving = swerve.isMovingTrigger();

// Custom threshold
Trigger isMovingFast = swerve.isMovingTrigger(3.0); // > 3 m/s

isMovingFast.whileTrue(leds.strobeCommand(Color.kRed, 0.1));
```

---

#### isStoppedTrigger()
Returns a trigger that is true when robot is fully stopped (both linear AND angular velocity below thresholds).

```java
// Uses defaults: 0.05 m/s linear, 0.1 rad/s angular
Trigger isStopped = swerve.isStoppedTrigger();

isStopped.whileTrue(shooter.spinUpCommand());
```

---

#### isStationaryTrigger(double velocityThreshold)
Returns a trigger that is true when robot linear velocity is below threshold.

```java
Trigger isStopped = swerve.isStationaryTrigger(0.1); // < 0.1 m/s

isStopped.whileTrue(shooter.spinUpCommand());
```

---

#### isFullyStoppedTrigger(double linearThreshold, double angularThreshold)
Returns a trigger that is true when both linear AND angular velocity are below thresholds.

```java
// Stricter than isStationaryTrigger - checks rotation too
Trigger fullyStopped = swerve.isFullyStoppedTrigger(0.05, 0.1);

fullyStopped.onTrue(vision.takeMeasurementCommand());
```

---

### Rotation Triggers

#### isRotatingTrigger(double thresholdRadPerSec)
Returns a trigger that is true when angular velocity exceeds threshold.

```java
Trigger isSpinning = swerve.isRotatingTrigger(0.5); // > 0.5 rad/s

// Disable vision during fast rotation
isSpinning.whileTrue(swerve.disableVisionCommand());
```

---

#### isNotRotatingTrigger(double thresholdRadPerSec)
Returns a trigger that is true when angular velocity is below threshold.

```java
Trigger settledRotation = swerve.isNotRotatingTrigger(0.1);

settledRotation.onTrue(vision.enableVisionCommand());
```

---

### Direction Triggers

#### isMovingInDirectionTrigger(Rotation2d direction, double toleranceDegrees, double minVelocity)
Returns a trigger that is true when moving in specified field direction.

```java
// Trigger when moving toward blue alliance wall (+X direction)
Trigger movingForward = swerve.isMovingInDirectionTrigger(
    Rotation2d.fromDegrees(0), 45, 0.1);

// Trigger when moving left (field +Y direction)
Trigger movingLeft = swerve.isMovingInDirectionTrigger(
    Rotation2d.fromDegrees(90), 30, 0.2);
```

---

#### isMovingForwardTrigger()
Convenience trigger for forward movement (field +X direction, 45° tolerance, 0.1 m/s min).

```java
Trigger goingForward = swerve.isMovingForwardTrigger();
```

---

#### isStrafingTrigger(double minVelocity)
Returns a trigger that is true when moving perpendicular to heading (60-120° from heading).

```java
Trigger strafing = swerve.isStrafingTrigger(0.2);

strafing.whileTrue(leds.setColorCommand(Color.kYellow));
```

---

### Position Triggers

#### distanceFromPointTrigger(Translation2d point, double minDistance, double maxDistance)
Returns a trigger that is true when within distance range of a field point.

```java
Translation2d scoringSpot = new Translation2d(2.0, 5.5);

// Trigger when 1-3 meters from scoring spot
Trigger nearScoring = swerve.distanceFromPointTrigger(scoringSpot, 1.0, 3.0);

nearScoring.whileTrue(swerve.aimAtAprilTagCommand(7, 2.0));
```

---

### Physical State Triggers

#### isPitchedTrigger(double thresholdDegrees)
Returns a trigger that is true when robot pitch exceeds threshold (for ramps/climbing).

```java
Trigger onRamp = swerve.isPitchedTrigger(10.0); // > 10 degrees pitch

onRamp.whileTrue(driveSlowCommand());
```

---

### Vision-Based Triggers

#### isAlignedWithTagTrigger(int tagID, double toleranceDegrees)
Returns a trigger that is true when aligned with an AprilTag within tolerance.

```java
Trigger alignedWithTag7 = swerve.isAlignedWithTagTrigger(7, 2.0);

alignedWithTag7.onTrue(shooter.shootCommand());
```

---

#### hasVisionTargetTrigger()
Returns a trigger that is true when any AprilTag is visible.

```java
Trigger hasTarget = swerve.hasVisionTargetTrigger();

hasTarget.whileTrue(leds.setColorCommand(Color.kGreen));
```

---

#### isInRangeOfTagTrigger(int tagID, double minDistance, double maxDistance)
Returns a trigger that is true when within distance range of an AprilTag.

```java
Trigger inShootingRange = swerve.isInRangeOfTagTrigger(7, 1.0, 3.0);

inShootingRange.whileTrue(shooter.aimAndSpinCommand());
```

---

### Heading-Based Trigger

#### isFacingHeadingTrigger(Rotation2d targetHeading, double toleranceDegrees)
Returns a trigger that is true when robot is facing a target heading.

```java
Trigger facingAway = swerve.isFacingHeadingTrigger(Rotation2d.fromDegrees(180), 5.0);

facingAway.whileTrue(intake.intakeCommand());
```

---

## Usage Examples

### Example 1: Basic Teleop Drive

```java
public class RobotContainer {
  private final XboxController driver = new XboxController(0);
  private final SwerveSubsystem swerve;

  public RobotContainer() {
    swerve = new SwerveSubsystem(
      new File(Filesystem.getDeployDirectory(), "swerve/kraken")
    );

    configureBindings();
  }

  private void configureBindings() {
    // Default drive command
    swerve.setDefaultCommand(
      swerve.driveCommand(
        () -> -driver.getLeftY(),
        () -> -driver.getLeftX(),
        () -> -driver.getRightX()
      )
    );

    // Reset gyro to 0° when back button pressed
    new JoystickButton(driver, XboxController.Button.kBack.value)
      .onTrue(Commands.runOnce(() -> swerve.zeroGyro()));
  }
}
```

---

### Example 2: Vision-Assisted Scoring

```java
private void configureBindings() {
  // A button: Aim at AprilTag 7
  new JoystickButton(driver, XboxController.Button.kA.value)
    .whileTrue(swerve.aimAtAprilTagCommand(7, 2.0));

  // B button: Drive to scoring position with vision
  List<Pose2d> scoringPoses = List.of(
    new Pose2d(2.0, 3.0, Rotation2d.fromDegrees(0)),
    new Pose2d(2.0, 4.0, Rotation2d.fromDegrees(0))
  );

  new JoystickButton(driver, XboxController.Button.kB.value)
    .whileTrue(swerve.driveToNearestPoseWithVisionCommand(scoringPoses));
}
```

---

### Example 3: State-Based LED Feedback

```java
private void configureBindings() {
  // LEDs green when AprilTag visible
  swerve.hasVisionTargetTrigger()
    .whileTrue(leds.setColorCommand(Color.kGreen))
    .whileFalse(leds.setColorCommand(Color.kRed));

  // LEDs pulse orange when aligned with scoring tag
  swerve.isAlignedWithTagTrigger(7, 2.0)
    .whileTrue(leds.pulseCommand(Color.kOrange, 1.0));

  // LEDs blue when in community zone
  swerve.inRegionTrigger(0, 3, 2, 5)
    .whileTrue(leds.setColorCommand(Color.kBlue));
}
```

---

### Example 4: Automatic Shooter Spin-Up

```java
private void configureBindings() {
  // Spin up shooter when aligned and in range
  Trigger readyToShoot = swerve.isAlignedWithTagTrigger(7, 3.0)
    .and(swerve.isInRangeOfTagTrigger(7, 1.5, 4.0))
    .and(swerve.isStationaryTrigger(0.2));

  readyToShoot.whileTrue(shooter.spinUpCommand());

  // Shoot when ready and driver presses right trigger
  readyToShoot.and(new Trigger(() -> driver.getRightTriggerAxis() > 0.5))
    .onTrue(shooter.shootCommand());
}
```

---

### Example 5: Autonomous Path Following

```java
public Command getAutonomousCommand() {
  return Commands.sequence(
    // Reset odometry to starting position
    Commands.runOnce(() -> swerve.resetOdometry(
      new Pose2d(1.5, 5.5, Rotation2d.fromDegrees(0))
    )),

    // Enable vision for autonomous
    swerve.enableVisionCommand(),

    // Run PathPlanner auto
    swerve.getAutonomousCommand("4PieceAuto"),

    // Stop and center modules
    swerve.centerModulesCommand()
  );
}
```

---

### Example 6: Camera Switching Based on Field Position

```java
private void configureBindings() {
  // Switch camera modes based on field region
  Trigger nearLoadingZone = swerve.inRegionTrigger(13, 16.5, 0, 8);
  nearLoadingZone
    .onTrue(Commands.runOnce(() -> {
        swerve.vision.disableCamerasWithPurpose(CameraPurpose.ODOMETRY);
        swerve.vision.enableCamerasWithPurpose(CameraPurpose.ALIGNMENT);
    }))
    .onFalse(Commands.runOnce(() -> {
        swerve.vision.enableCamerasWithPurpose(CameraPurpose.ODOMETRY);
        swerve.vision.disableCamerasWithPurpose(CameraPurpose.ALIGNMENT);
    }));
}
```

---

## Troubleshooting

### Robot Doesn't Move

**Check:**
1. ✓ JSON files exist in deploy folder
2. ✓ All CAN IDs are correct and unique
3. ✓ Motors are powered and connected
4. ✓ Driver station shows no CAN errors
5. ✓ `driveCommand()` is set as default command

**Test:**
```java
// Add to periodic() for debugging
@Override
public void periodic() {
  System.out.println("Pose: " + getPose());
  System.out.println("Gyro: " + getHeading());
}
```

---

### Modules Point Wrong Direction

**Cause:** Incorrect absolute encoder offsets

**Fix:**
1. Re-zero modules following [Zeroing Process](#zeroing-swerve-modules)
2. Ensure wheels are physically aligned before reading CANcoder values
3. Deploy code with updated offsets

---

### One Module Drives Backward

**Cause:** Drive motor inversion incorrect

**Fix:**
In that module's JSON, flip the drive inversion:
```json
"inverted": {
  "drive": true  // was false
}
```

---

### Robot Spins Uncontrollably

**Cause:** Angle PID too aggressive or angle motor inverted wrong

**Fix:**
1. Reduce angle P gain in `pidfproperties.json`:
   ```json
   "angle": {
     "p": 0.005  // was 0.01
   }
   ```
2. Check angle motor inversions in module JSON files
3. Verify all angle motors turn the same direction

---

### Vision Not Working

**Check:**
1. ✓ PhotonVision is running and cameras are connected
2. ✓ AprilTag field layout matches game year
3. ✓ Camera calibration is complete
4. ✓ Camera names in code match PhotonVision names

**Test:**
```java
// Add to configureBindings()
new JoystickButton(driver, 10)
  .onTrue(swerve.getDistanceFromAprilTagCommand(7));
```

Check console output to verify vision data.

---

### PathPlanner Paths Don't Work

**Check:**
1. ✓ PathPlanner installed and paths created in GUI
2. ✓ Paths saved to `src/main/deploy/pathplanner/paths/`
3. ✓ AutoBuilder configured in SwerveSubsystem constructor (already done in AdambotsLib)
4. ✓ Robot config matches physical robot (wheelbase, mass, etc.)

---

### CAN Errors / Timeouts

**Cause:** CAN bus issues, duplicate IDs, or bad wiring

**Fix:**
1. Check Driver Station for specific CAN device errors
2. Use Phoenix Tuner to verify all devices are detected
3. Ensure no duplicate CAN IDs
4. Check CAN bus termination (120Ω resistors at each end)
5. Inspect CAN wiring for loose connections

---

### Odometry Drifts Over Time

**Cause:** Wheel slippage, incorrect wheel diameter, or missing vision corrections

**Fix:**
1. Measure actual wheel diameter (wheels wear down)
2. Update `"diameter"` in `physicalproperties.json`
3. Enable vision pose estimation for drift correction
4. Consider re-tuning drive PID if wheels are slipping

---

## Additional Resources

- **YAGSL Documentation:** https://docs.yagsl.com/
- **YAGSL GitHub:** https://github.com/BroncBotz3481/YAGSL
- **YAGSL Example Project:** https://github.com/BroncBotz3481/YAGSL-Example
- **PathPlanner Documentation:** https://pathplanner.dev/
- **PhotonVision Documentation:** https://docs.photonvision.org/
- **WPILib Swerve Guide:** https://docs.wpilib.org/en/stable/docs/software/kinematics-and-odometry/swerve-drive-kinematics.html

---

## Need Help?

- Check the [AdambotsLib GitHub Issues](https://github.com/Adambots/AdambotsLib/issues)
- Ask on Chief Delphi's programming forum
- Consult your team's programming mentor

---

**Last Updated:** 2026-02-02
**AdambotsLib Version:** 2026.2.0+
