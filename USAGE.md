# AdambotsLib - Usage Guide

This guide is for team members who want to use AdambotsLib in their FRC robot projects.

## What is AdambotsLib?

AdambotsLib is Team 245's reusable components library containing:
- Motor controller wrappers (NEO, TalonFX, etc.)
- Swerve drive subsystems
- Vision processing utilities (PhotonVision integration)
- Common commands and utilities
- Sensor wrappers and abstractions

## Installation

### Option 1: Install via VS Code (Recommended)

1. **Open your robot project** in VS Code
2. **Open Command Palette**:
   - Windows/Linux: `Ctrl+Shift+P`
   - Mac: `Cmd+Shift+P`
3. Type: `WPILib: Manage Vendor Libraries`
4. Select: `Install new library (online)`
5. Enter this URL:
   ```
   https://raw.githubusercontent.com/Adambots-245/AdambotsLib/main/AdambotsLib.json
   ```
6. Press Enter and wait for installation to complete

### Option 2: Manual Installation

1. Download [`AdambotsLib.json`](https://raw.githubusercontent.com/Adambots-245/AdambotsLib/main/AdambotsLib.json)
2. Copy it to your robot project's `vendordeps/` folder
3. Build your project - the library will be automatically downloaded

## What Gets Installed

When you install AdambotsLib, you automatically get:

**Core Library:**
- `com.adambots.lib:AdambotsLib-java:2026.1.0`

**Included Dependencies** (automatically installed):
- Phoenix6 (26.1.0) - CTRE motor controllers
- Phoenix5 (5.36.0) - Legacy CTRE support
- REVLib (2026.0.1) - REV motor controllers
- ReduxLib (2026.1.1) - Redux motor controllers
- YAGSL (2026.1.12) - Swerve drive library
- PathplannerLib (2026.1.2) - Path planning
- PhotonLib (v2025.3.1) - Vision processing
- And other dependencies

All these dependencies are automatically installed when you add AdambotsLib!

## Using AdambotsLib in Your Code

### Import the Library

In your robot code, import the classes you need:

```java
// Actuators
import com.adambots.lib.actuators.TalonFXMotor;
import com.adambots.lib.actuators.NEOMotor;
import com.adambots.lib.actuators.MinionMotor;

// Subsystems
import com.adambots.lib.subsystems.SwerveSubsystem;

// Vision
import com.adambots.lib.vision.PhotonVision;

// And more...
```

### Available Components

AdambotsLib provides reusable components for:

**Actuators:**
- `NEOMotor` - REV Robotics NEO motor wrapper
- `TalonFXMotor` - CTRE TalonFX/Kraken motor wrapper
- `MinionMotor` - TalonFXS motor controller wrapper
- `AngularHubServo` - REV Robotics servo control

**Subsystems:**
- `SwerveSubsystem` - YAGSL-based swerve drive with command factories
- Vision subsystems

**Utilities:**
- Vision processing helpers
- Motor configuration helpers
- And more...

## Quick Start Guide

### Step 1: Add AdambotsLib to Your Project

#### Option A: Online Installation (Recommended)

1. Open your robot project in VS Code
2. Press `Ctrl+Shift+P` (Windows/Linux) or `Cmd+Shift+P` (Mac)
3. Type: **"WPILib: Manage Vendor Libraries"**
4. Select **"Install new library (online)"**
5. Enter this URL:
   ```
   https://raw.githubusercontent.com/Adambots-245/AdambotsLib/main/AdambotsLib.json
   ```
6. Press Enter - the library will be downloaded and added to your project

#### Option 2: Manual Installation

1. Download `AdambotsLib.json` from: https://github.com/Adambots-245/AdambotsLib
2. Copy it to your robot project's `vendordeps/` folder
3. Build your project - the library will be downloaded automatically

## Verifying Installation

After installation, you should see:
- `AdambotsLib.json` in your `vendordeps/` folder
- The library appears in your project dependencies when you build

Build your project to download the library:
```bash
./gradlew build
```

## Using AdambotsLib in Your Code

### Import the Classes

```java
// Import specific classes as needed
import com.adambots.lib.actuators.TalonFXMotor;
import com.adambots.lib.actuators.NEOMotor;
import com.adambots.lib.subsystems.SwerveSubsystem;
import com.adambots.lib.vision.PhotonVision;
// ... and other classes
```

### Key Library Components

**Actuators:**
- `TalonFXMotor` - Wrapper for CTRE Falcon 500 / Kraken X60 motors
- `MinionMotor` - Wrapper for CTRE TalonFXS motors
- `NEOMotor` - Wrapper for REV NEO motors with SparkMax/SparkFlex
- `AngularHubServo` - Servo control wrapper

**Subsystems:**
- `SwerveSubsystem` - Swerve drive subsystem using YAGSL with built-in command factories

**Vision:**
- `PhotonVision` - Camera and vision processing helpers
- `LimelightHelpers` - Limelight integration utilities

**Utilities:**
- Various helper classes in `com.adambots.lib.utils`

## Using AdambotsLib in Your Robot Project

### Step 1: Install the Vendor Dependency

**Method 1: Online Installation (Recommended)**

1. Open your robot project in VS Code
2. Press `Ctrl+Shift+P` (Windows/Linux) or `Cmd+Shift+P` (Mac)
3. Type and select: **"WPILib: Manage Vendor Libraries"**
4. Select **"Install new library (online)"**
5. Enter this URL:
   ```
   https://raw.githubusercontent.com/Adambots-245/AdambotsLib/main/AdambotsLib.json
   ```
6. Press Enter

The library will be automatically downloaded and added to your project.

**Option 2: Manual Installation**

1. Download `AdambotsLib.json` from: https://github.com/Adambots-245/AdambotsLib
2. Copy the file to your robot project's `vendordeps/` folder
3. Build your project - the library will be downloaded automatically

## Using AdambotsLib in Your Code

### Import Classes

Once installed, you can import and use AdambotsLib classes:

```java
import com.adambots.lib.actuators.NEOMotor;
import com.adambots.lib.actuators.TalonFXMotor;
import com.adambots.lib.subsystems.SwerveSubsystem;
import com.adambots.lib.vision.PhotonVision;
```

### Example Usage

**Creating a NEO Motor:**

```java
import com.adambots.lib.actuators.NEOMotor;
import com.adambots.lib.actuators.BaseMotor.ControlMode;

public class MySubsystem extends SubsystemBase {
    private final NEOMotor motor;

    public MySubsystem() {
        // Create a NEO motor on CAN ID 1, brushless, not inverted
        motor = new NEOMotor(1, false);

        // Configure using builder pattern (recommended)
        motor.configure()
            .pid(0.1, 0.0, 0.05, 0.0)
            .currentLimits(40, 60, 5000)
            .brakeMode(true)
            .voltageCompensation(12.0)
            .apply();
    }

    public void setSpeed(double speed) {
        motor.set(ControlMode.PERCENT_OUTPUT, speed);
    }
}
```

**Creating a TalonFX Motor:**

```java
import com.adambots.lib.actuators.TalonFXMotor;
import com.adambots.lib.actuators.BaseMotor.ControlMode;

public class ShooterSubsystem extends SubsystemBase {
    private final TalonFXMotor shooterMotor;

    public ShooterSubsystem() {
        // Create a Kraken X60 motor on CAN ID 5, not inverted, is Kraken
        shooterMotor = new TalonFXMotor(5, false, true);

        // Configure using builder pattern (recommended)
        shooterMotor.configure()
            .pid(0.05, 0.0, 0.001, 0.12)
            .currentLimits(60, 80, 3000)
            .motionMagic(80.0, 160.0, 0.0)
            .brakeMode(false)  // Coast mode for shooter
            .voltageCompensation(12.0)
            .apply();
    }

    public void setVelocity(double rps) {
        shooterMotor.set(ControlMode.VELOCITY, rps);
    }
}
```

**Using PhotonVision:**

```java
import com.adambots.lib.vision.PhotonVision;
import org.photonvision.targeting.PhotonTrackedTarget;

public class VisionSubsystem extends SubsystemBase {
    private final PhotonVision vision;

    public VisionSubsystem() {
        vision = new PhotonVision("photonvision");
    }

    public Optional<PhotonTrackedTarget> getBestTarget() {
        return vision.getBestTarget();
    }

    public double getTargetYaw() {
        return vision.getTargetYaw();
    }
}
```

## Available Components

### Actuators (Motor Controllers)

- **NEOMotor** - Wrapper for REV Robotics NEO/NEO 550 motors via SparkMax/SparkFlex
- **TalonFXMotor** - Wrapper for CTRE Falcon 500/Kraken X60 motors via TalonFX
- **MinionMotor** - Wrapper for CTRE Minion motors via TalonFXS
- **AngularHubServo** - Wrapper for REV Robotics Angular Hub Servos

### Subsystems

- **SwerveSubsystem** - Complete swerve drive subsystem using YAGSL
  - Supports field-relative driving
  - Auto-alignment features
  - PathPlanner integration
  - Built-in command factories for all drive operations
  - Trigger methods for state-based command composition

### Vision

- **PhotonVision** - PhotonVision camera integration
  - Target tracking
  - AprilTag detection
  - Pose estimation

## Control Modes

AdambotsLib provides a unified `ControlMode` enum across all motor types:

- `PERCENT_OUTPUT` - Direct power control (-1.0 to 1.0)
- `POSITION` - Position control (encoder units or rotations)
- `VELOCITY` - Velocity control (RPM)
- `CURRENT` - Current/torque control (amps)
- `VOLTAGE` - Voltage control (volts)
- `MOTION_MAGIC` - Motion profiled position control
- `MOTION_MAGIC_FOC_TORQUE` - FOC torque-based motion magic (TalonFX only)
- `FOLLOWER` - Follow another motor

## Motor Configuration

### Builder Pattern (Recommended)

**New in 2026.2.0** - Configure motors using the fluent builder pattern for clean, readable code:

```java
// Configure a drive motor with common settings
motor.configure()
    .pid(0.1, 0.0, 0.05, 0.0)
    .currentLimits(40, 60, 5000)
    .motionMagic(50.0, 100.0, 200.0)
    .brakeMode(true)
    .voltageCompensation(12.0)
    .apply();

// Minimal configuration for a simple mechanism
motor.configure()
    .currentLimits(20, 40, 3000)
    .brakeMode(true)
    .apply();
```

**Available Builder Methods:**

- `.pid(kP, kI, kD, kF)` - Configure PID gains for slot 0
- `.pid(slot, kP, kI, kD, kF)` - Configure PID gains for specific slot
- `.currentLimits(stallAmps, freeAmps, limitRpm)` - Set current limits
- `.motionMagic(cruiseVelRPS, accelRPSPerSec, jerkRPSPerSecPerSec)` - Configure motion profiling
- `.brakeMode(brake)` - Set brake (true) or coast (false) mode
- `.voltageCompensation(voltage)` - Enable voltage compensation (typically 12.0)
- `.apply()` - Apply all configured settings (must call last)

**Parameter Units:**

All parameters now include units in their names for clarity:
- `cruiseVelocityRPS` - rotations per second
- `accelerationRPSPerSec` - rotations per second²
- `jerkRPSPerSecPerSec` - rotations per second³
- `stallLimitAmps` - amperes
- `freeLimitAmps` - amperes
- `limitRpmThreshold` - RPM
- `forwardLimitRotations` - rotations
- `reverseLimitRotations` - rotations

### Traditional Methods

All motor classes also support direct configuration methods:

```java
// PID Configuration
motor.setPID(int slot, double kP, double kI, double kD, double kF);

// Current Limiting
motor.configureCurrentLimits(double stallLimitAmps, double freeLimitAmps, double limitRpmThreshold);

// Soft Limits
motor.configureSoftLimits(double forwardLimitRotations, double reverseLimitRotations, boolean enable);

// Motion Magic/Profiling
motor.configureMotionMagic(double cruiseVelocityRPS, double accelerationRPSPerSec, double jerkRPSPerSecPerSec);

// Motor Inversion
motor.setInverted(boolean inverted);

// Brake/Coast Mode
motor.setBrakeMode(boolean brake);

// Voltage Compensation
motor.enableVoltageCompensation(double voltage);

// Follower Mode
motor.setStrictFollower(int leaderDeviceID);
```

## Updating AdambotsLib

### To Update to Latest Version

1. In VS Code, open Command Palette (Ctrl+Shift+P / Cmd+Shift+P)
2. Type "WPILib: Manage Vendor Libraries"
3. Select "Check for updates (online)"
4. Select AdambotsLib if an update is available
5. Build your project

### To Check Current Version

Look at your `vendordeps/AdambotsLib.json` file:

```json
{
    "version": "2026.1.0",
    ...
}
```

### To Force Reinstall

1. Delete `vendordeps/AdambotsLib.json` from your project
2. Follow the installation steps again

## Troubleshooting

### Problem: Build fails with "Could not find AdambotsLib-java"

**Solution**:
1. Check your internet connection
2. Verify `vendordeps/AdambotsLib.json` exists in your project
3. Run: `./gradlew clean build --refresh-dependencies`
4. Check that https://adambots-245.github.io/AdambotsLib/maven is accessible

### Problem: Import statements show as errors

**Solution**:
1. Build the project once: `./gradlew build`
2. Reload VS Code window: Ctrl+Shift+P → "Reload Window"
3. Check that AdambotsLib is listed in `vendordeps/`

### Problem: Old version still being used after update

**Solution**:
```bash
./gradlew clean build --refresh-dependencies
```

### Problem: Need to use unreleased features

**Solution**: For testing development versions, ask a maintainer to publish a local build:
```bash
# Maintainer runs:
./gradlew publishToMavenLocal

# Then copy AdambotsLib.json to your vendordeps/
# Your build.gradle should include:
repositories {
    mavenLocal()
}
```

## Library Information

**Current Version**: 2026.1.0
**Supported FRC Year**: 2026
**WPILib Version**: 2026.1.1

**Dependencies** (automatically included):
- Phoenix6 (26.1.0)
- Phoenix5 (5.36.0)
- REVLib (2026.0.1)
- ReduxLib (2026.1.1)
- YAGSL (2026.1.12)
- PathplannerLib (2026.1.2)
- PhotonLib (v2025.3.1)
- And others

**Repository**: https://github.com/Adambots-245/AdambotsLib
**Maven**: https://adambots-245.github.io/AdambotsLib/maven
**Vendordep JSON**: https://raw.githubusercontent.com/Adambots-245/AdambotsLib/main/AdambotsLib.json

## Getting Help

- Check the library source code on GitHub
- Ask the programming leads
- Create an issue on the GitHub repository
- Check the FRC documentation for WPILib and vendor libraries

## Best Practices

1. **Always use the latest version** of AdambotsLib for new projects
2. **Test motor configurations** on the robot before competition
3. **Use appropriate control modes** - velocity for flywheels, position for arms, etc.
4. **Configure current limits** to protect motors and breakers
5. **Enable soft limits** for mechanisms with physical constraints
6. **Use brake mode** for mechanisms that need to hold position

## Quick Reference

**Installation URL:**
```
https://raw.githubusercontent.com/Adambots-245/AdambotsLib/main/AdambotsLib.json
```

**Maven Coordinates:**
```
com.adambots.lib:AdambotsLib-java:2026.1.0
```

**UUID:**
```
245a0b0a-d4f5-4c3e-9f1a-2458bcd245ab
```
