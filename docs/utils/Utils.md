# Utils - General Utility Functions

Comprehensive collection of utility functions for common FRC programming tasks.

---

## Table of Contents

- [Overview](#overview)
- [Epsilon Comparison](#epsilon-comparison)
- [Geometry Utilities](#geometry-utilities)
- [Alliance Utilities](#alliance-utilities)
- [Math Utilities](#math-utilities)
- [Angle Utilities](#angle-utilities)
- [Boolean Utilities](#boolean-utilities)
- [Array Utilities](#array-utilities)
- [Time Utilities](#time-utilities)
- [Error Reporting](#error-reporting)
- [Complete Examples](#complete-examples)
- [API Reference](#api-reference)
- [Best Practices](#best-practices)

---

## Overview

The `Utils` class provides general-purpose utility functions that complement WPILib's built-in utilities. These functions solve common problems in FRC robot code without duplicating WPILib functionality.

### Categories

- **Epsilon Comparison** - Floating-point comparison with tolerance *
- **Geometry Utilities** - Pose/Transform conversions and operations *
- **Alliance Utilities** - Check alliance color, mirror poses for red/blue
- **Math Utilities** - Linear interpolation, range mapping, progress calculation
- **Angle Utilities** - Normalize angles, compute shortest angular difference
- **Boolean Utilities** - Debouncing, edge detection for sensors and buttons
- **Array Utilities** - Average, min, max, and index finding for arrays
- **Time Utilities** - Time conversion and elapsed time checks
- **Error Reporting** - Standardized error/warning messages to DriverStation

*Adapted from [FRC Team 6328 Mechanical Advantage](https://github.com/Mechanical-Advantage/RobotCode2026Public)

### When to Use

✅ **Use Utils when:**
- You need alliance-specific logic (mirror poses, check colors)
- You need angle normalization or shortest angular difference
- You need to debounce sensors or detect button edges
- You need array operations (average, min/max)
- You need simple time checks or conversions
- You want standardized error reporting

❌ **Don't use Utils when:**
- WPILib already has the function (clamp, deadband, etc.)
- You need complex mathematical operations (use Math or MathUtil)
- You need unit conversions (use edu.wpi.first.math.util.Units)

---

## Epsilon Comparison

> *Adapted from [FRC Team 6328 Mechanical Advantage](https://github.com/Mechanical-Advantage/RobotCode2026Public)*

Floating-point arithmetic can produce small rounding errors, making direct equality comparison unreliable. These methods check if values are "close enough" to be considered equal.

### Compare Doubles

```java
// Compare with default epsilon (1e-9)
double a = 0.1 + 0.2;  // May not equal exactly 0.3
if (Utils.epsilonEquals(a, 0.3)) {
    // Values are close enough
}

// Compare with custom epsilon for larger tolerances
if (Utils.epsilonEquals(motorPosition, targetPosition, 0.001)) {
    // Within 0.001 of target
}

// Check if motor has reached setpoint
if (Utils.epsilonEquals(encoder.getPosition(), setpoint, 0.5)) {
    // Within 0.5 units of setpoint
}
```

### Compare Twist2d

```java
// Compare velocity vectors
Twist2d currentTwist = drivetrain.getTwist();
Twist2d targetTwist = new Twist2d(1.0, 0.0, 0.5);

if (Utils.epsilonEquals(currentTwist, targetTwist)) {
    // Velocities match (dx, dy, and dtheta all within epsilon)
}
```

### When to Use

- **Motor control**: Check if position/velocity reached setpoint
- **Odometry**: Compare pose estimates
- **Path following**: Check if waypoint reached
- **Unit tests**: Compare calculated vs expected values

---

## Geometry Utilities

> *Adapted from [FRC Team 6328 Mechanical Advantage](https://github.com/Mechanical-Advantage/RobotCode2026Public)*

Conversions and operations for WPILib geometry types (Pose2d, Transform2d, Twist2d, etc.).

### Pose2d ↔ Transform2d Conversion

```java
// Convert Pose2d to Transform2d (for kinematic chains)
Pose2d cameraPose = new Pose2d(0.3, 0.0, Rotation2d.fromDegrees(0));
Transform2d cameraTransform = Utils.toTransform2d(cameraPose);

// Convert Transform2d back to Pose2d
Pose2d pose = Utils.toPose2d(cameraTransform);
```

### Create Pose from Components

```java
// Create Pose2d from Translation2d only (zero rotation)
Translation2d point = new Translation2d(2.0, 3.0);
Pose2d pose = Utils.toPose2d(point);  // (2.0, 3.0, 0°)

// Create Pose2d from Rotation2d only (zero translation)
Rotation2d heading = Rotation2d.fromDegrees(45);
Pose2d pose = Utils.toPose2d(heading);  // (0, 0, 45°)
```

### Pose Inverse

```java
// Compute inverse pose (useful for relative transformations)
Pose2d robotPose = odometry.getPose();
Pose2d inversePose = Utils.inverse(robotPose);

// robotPose.transformBy(Utils.toTransform2d(inversePose)) ≈ origin
```

### Twist2d Operations

```java
// Scale a twist (velocity) by a factor
Twist2d velocity = new Twist2d(2.0, 1.0, 0.5);
Twist2d halfVelocity = Utils.multiply(velocity, 0.5);
// halfVelocity = (1.0, 0.5, 0.25)

// Convert ChassisSpeeds to Twist2d
ChassisSpeeds speeds = swerve.getChassisSpeeds();
Twist2d twist = Utils.toTwist2d(speeds);
```

### Modify Pose Components

```java
// Replace translation while keeping rotation
Pose2d original = new Pose2d(1.0, 2.0, Rotation2d.fromDegrees(45));
Translation2d newTranslation = new Translation2d(5.0, 6.0);
Pose2d modified = Utils.withTranslation(original, newTranslation);
// modified = (5.0, 6.0, 45°)

// Replace rotation while keeping translation
Rotation2d newRotation = Rotation2d.fromDegrees(90);
Pose2d modified2 = Utils.withRotation(original, newRotation);
// modified2 = (1.0, 2.0, 90°)
```

### 3D Geometry

```java
// Convert Pose3d to Transform3d
Pose3d cameraPose3d = new Pose3d(...);
Transform3d cameraTransform3d = Utils.toTransform3d(cameraPose3d);

// Convert Transform3d to Pose3d
Pose3d pose3d = Utils.toPose3d(cameraTransform3d);

// Project 3D transform to 2D (XY plane)
Transform2d transform2d = Utils.toTransform2d(cameraTransform3d);
```

### Real-World Example: Vision Target Tracking

```java
public Pose2d getTargetPose(Pose2d robotPose, Transform2d cameraToRobot) {
    // Get target position relative to camera
    Pose2d targetRelativeToCamera = vision.getTargetPose();

    // Convert to transform for composition
    Transform2d targetTransform = Utils.toTransform2d(targetRelativeToCamera);

    // Compose transforms: robot -> camera -> target
    return robotPose
        .transformBy(cameraToRobot)
        .transformBy(targetTransform);
}
```

---

## Alliance Utilities

Check alliance color and mirror field positions for red/blue symmetry.

### Check Alliance

```java
// Check current alliance
if (Utils.isOnRedAlliance()) {
    targetPose = FieldPositions.RED_SPEAKER;
} else if (Utils.isOnBlueAlliance()) {
    targetPose = FieldPositions.BLUE_SPEAKER;
}

// Get alliance as a Color object
Color allianceColor = Utils.getAllianceColor();  // Red, Blue, or Black (unknown)
leds.setColor(allianceColor);
```

### Mirror Poses

FRC fields are symmetric - define positions for blue alliance, then mirror for red:

```java
// Mirror X coordinate
double blueSpeakerX = 0.5;  // Blue speaker position
double speakerX = Utils.mirrorXForAlliance(blueSpeakerX, 16.54);
// Blue alliance: returns 0.5
// Red alliance: returns 16.04

// Mirror Translation2d
Translation2d blueSpeaker = new Translation2d(0.5, 5.5);
Translation2d speaker = Utils.mirrorTranslationForAlliance(blueSpeaker, 16.54);

// Mirror Pose2d (also rotates heading 180°)
Pose2d bluePose = new Pose2d(2.0, 5.5, Rotation2d.fromDegrees(0));
Pose2d alliancePose = Utils.mirrorPoseForAlliance(bluePose, 16.54);
// Blue: (2.0, 5.5, 0°)
// Red: (14.54, 5.5, 180°)
```

---

## Math Utilities

Common mathematical operations for robotics.

### Linear Interpolation (lerp)

Smoothly blend between two values:

```java
// Interpolate from 0 to 100
double value = Utils.lerp(0, 100, 0.5);   // Returns 50
double value2 = Utils.lerp(0, 100, 0.25); // Returns 25
double value3 = Utils.lerp(0, 100, 1.0);  // Returns 100

// Blend between two colors (RGB)
double r = Utils.lerp(255, 0, 0.7);  // 76.5
double g = Utils.lerp(0, 255, 0.7);  // 178.5
double b = Utils.lerp(0, 0, 0.7);    // 0
```

**Note**: For WPILib geometry types, use `MathUtil.interpolate()` instead.

### Map Range

Convert values from one range to another:

```java
// Map joystick (-1 to 1) to motor speed (0 to 100)
double motorSpeed = Utils.mapRange(joystick.getY(), -1, 1, 0, 100);

// Map encoder counts (0 to 4096) to degrees (0 to 360)
double angleDeg = Utils.mapRange(encoder.getPosition(), 0, 4096, 0, 360);

// Map distance sensor (0 to 200cm) to percentage (0 to 1)
double proximity = Utils.mapRange(sensor.getDistance(), 0, 200, 1.0, 0.0);
```

### Check Range

```java
// Check if encoder is in safe zone
if (Utils.isInRange(encoder.getPosition(), 100, 200)) {
    // Safe to operate
} else {
    // Stop mechanism
}

// Check if arm is near target
if (Utils.isInRange(arm.getAngle(), targetAngle - 2, targetAngle + 2)) {
    // Within ±2 degrees of target
}
```

### Percent Progress

Calculate how far along a transition you are:

```java
// Arm deploying from 0° to 90°
double progress = Utils.percentProgress(arm.getAngle(), 0, 90);
// If angle = 45°, progress = 0.5 (50%)

// LED fade based on elevator height
double brightness = Utils.percentProgress(elevator.getHeight(), 0, 60);
leds.setBrightness(brightness);

// Timeout visualization
double timeProgress = Utils.percentProgress(
    System.currentTimeMillis(),
    startTime,
    startTime + 5000
);
dashboard.putNumber("Time Progress", timeProgress);
```

---

## Angle Utilities

Critical for angle control and rotation - always use shortest path!

### Normalize Angles

Wrap angles to standard range:

```java
// Wrap to [-180, 180] degrees
double angle = Utils.wrapAngleDeg(270);    // Returns -90
double angle2 = Utils.wrapAngleDeg(-190);  // Returns 170
double angle3 = Utils.wrapAngleDeg(540);   // Returns 180

// Wrap to [-π, π] radians
double angleRad = Utils.wrapAngleRad(Math.PI * 1.5);  // Returns -π/2
```

### Shortest Angular Difference

**CRITICAL for angle control** - prevents robot from rotating the long way:

```java
// Current heading = 10°, Target = 350°
// Wrong: 350 - 10 = 340° (rotate 340° clockwise)
// Right: Use shortestAngleDifference = -20° (rotate 20° counter-clockwise)

double error = Utils.shortestAngleDifferenceDeg(10, 350);  // Returns -20

// Use in PID control
double rotationOutput = rotationPID.calculate(
    0,  // Setpoint is always 0
    Utils.shortestAngleDifferenceDeg(currentHeading, targetHeading)
);
```

### Real-World Example

```java
public class TurretSubsystem extends SubsystemBase {
    private final TalonFX turretMotor;
    private final PIDController turretPID;

    public Command aimAtTargetCommand(double targetAngle) {
        return run(() -> {
            double currentAngle = getTurretAngle();

            // WRONG - Don't do this!
            // double error = targetAngle - currentAngle;

            // RIGHT - Use shortest angle difference
            double error = Utils.shortestAngleDifferenceDeg(currentAngle, targetAngle);

            double output = turretPID.calculate(0, error);
            turretMotor.set(output);
        }).until(() -> Math.abs(
            Utils.shortestAngleDifferenceDeg(getTurretAngle(), targetAngle)
        ) < 2.0);  // Within 2°
    }
}
```

---

## Boolean Utilities

Handle noisy sensors and detect state changes.

### Debounce

Prevent false triggers from noisy sensors:

```java
public class IntakeSubsystem extends SubsystemBase {
    private final DigitalInput gamePieceSensor;
    private boolean lastSensorValue = false;
    private long lastChangeTime = 0;

    public boolean hasGamePiece() {
        boolean currentValue = !gamePieceSensor.get();  // Inverted

        // Debounce - must be stable for 100ms
        boolean debounced = Utils.debounce(
            currentValue,
            lastSensorValue,
            lastChangeTime,
            100  // 100ms debounce time
        );

        // Update state tracking
        if (currentValue != lastSensorValue) {
            lastChangeTime = System.currentTimeMillis();
            lastSensorValue = currentValue;
        }

        return debounced;
    }
}
```

### Rising Edge Detection

Detect when a button is pressed or sensor activates:

```java
public class ClimberSubsystem extends SubsystemBase {
    private boolean lastLimitSwitch = false;

    @Override
    public void periodic() {
        boolean limitSwitch = limitSwitchTop.get();

        // Detect when limit switch just activated
        if (Utils.risingEdge(limitSwitch, lastLimitSwitch)) {
            // Just reached top - stop motor
            climbMotor.stopMotor();
            System.out.println("Climber reached top!");
        }

        lastLimitSwitch = limitSwitch;
    }
}
```

### Falling Edge Detection

Detect when a button is released or sensor deactivates:

```java
private boolean lastButtonState = false;

@Override
public void periodic() {
    boolean buttonPressed = button.get();

    // Detect button release
    if (Utils.fallingEdge(buttonPressed, lastButtonState)) {
        // Button was just released
        onButtonRelease();
    }

    lastButtonState = buttonPressed;
}
```

---

## Array Utilities

Operations on arrays of sensor values, motor currents, etc.

### Average

```java
// Average motor currents
double[] motorCurrents = {12.5, 13.2, 11.8, 12.9};
double avgCurrent = Utils.average(motorCurrents);  // Returns 12.6

// Average multiple distance measurements
double[] distances = {2.45, 2.48, 2.52, 2.46};
double avgDistance = Utils.average(distances);  // Returns 2.4775

// Empty array safety
double empty = Utils.average(new double[0]);  // Returns 0.0
```

### Min/Max

```java
// Find hottest motor
double[] motorTemps = {45.2, 52.1, 48.7, 51.3};
double hottestTemp = Utils.max(motorTemps);  // Returns 52.1

// Find closest target
double[] targetDistances = {2.5, 1.2, 3.1, 1.8};
double closestDistance = Utils.min(targetDistances);  // Returns 1.2

// Empty array safety
double empty = Utils.max(new double[0]);  // Returns Double.NEGATIVE_INFINITY
```

### Find Index

Find which element is min/max:

```java
// Find closest target
Pose2d[] targets = {pose1, pose2, pose3, pose4};
double[] distances = new double[targets.length];
for (int i = 0; i < targets.length; i++) {
    distances[i] = currentPose.getTranslation()
        .getDistance(targets[i].getTranslation());
}

int closestIndex = Utils.minIndex(distances);
Pose2d closestTarget = targets[closestIndex];

// Find fastest swerve module
double[] moduleSpeeds = {3.2, 4.1, 3.8, 2.9};
int fastestModule = Utils.maxIndex(moduleSpeeds);  // Returns 1
System.out.println("Module " + fastestModule + " is fastest");
```

---

## Time Utilities

Simple time operations for timeouts and delays.

### Check Elapsed Time

```java
public class ShooterSubsystem extends SubsystemBase {
    private long spinUpStartTime;
    private boolean spinning = false;

    public Command spinUpCommand() {
        return runOnce(() -> {
            shooter.setVelocity(targetRPM);
            spinUpStartTime = System.currentTimeMillis();
            spinning = true;
        }).andThen(
            Commands.waitUntil(() ->
                Utils.hasElapsed(spinUpStartTime, 1500)  // Wait 1.5 seconds
            )
        );
    }
}
```

### Time Conversions

```java
// Convert seconds to milliseconds for API calls
long timeoutMs = Utils.secondsToMillis(2.5);  // Returns 2500
Thread.sleep(timeoutMs);

// Convert milliseconds to seconds for display
long elapsed = System.currentTimeMillis() - startTime;
double elapsedSeconds = Utils.millisToSeconds(elapsed);
dashboard.putNumber("Elapsed Time", elapsedSeconds);
```

---

## Error Reporting

Standardized error and warning messages to DriverStation.

### Report Errors

```java
public SwerveSubsystem() {
    try {
        gyro = new AHRS(SPI.Port.kMXP);
        if (!gyro.isConnected()) {
            Utils.reportError("Gyro not connected - check MXP connection");
        }
    } catch (Exception e) {
        Utils.reportError("Failed to initialize gyro: " + e.getMessage());
    }
}
```

### Report Warnings

```java
@Override
public void periodic() {
    double voltage = battery.getVoltage();

    if (voltage < 11.5) {
        Utils.reportWarning("Battery voltage low: " + voltage + "V");
    }

    double motorTemp = motor.getDeviceTemp().getValueAsDouble();
    if (motorTemp > 70.0) {
        Utils.reportWarning("Motor temperature high: " + motorTemp + "°C");
    }
}
```

### Report Info

```java
public Command getAutonomousCommand() {
    String pathName = autoChooser.getSelected();
    Utils.reportInfo("Autonomous path selected: " + pathName);
    return new PathPlannerAuto(pathName);
}
```

---

## Complete Examples

### Example 1: Alliance-Aware Auto

```java
public class AutonomousCommands {
    private static final double FIELD_LENGTH = 16.54;  // meters

    public static Command driveToSpeakerCommand(SwerveSubsystem swerve) {
        // Define speaker pose for blue alliance
        Pose2d blueSpeaker = new Pose2d(0.5, 5.5, Rotation2d.fromDegrees(0));

        // Mirror for current alliance
        Pose2d speakerPose = Utils.mirrorPoseForAlliance(blueSpeaker, FIELD_LENGTH);

        // Show alliance on LEDs
        Color allianceColor = Utils.getAllianceColor();

        Utils.reportInfo("Driving to " +
            (Utils.isOnRedAlliance() ? "RED" : "BLUE") + " speaker");

        return Commands.sequence(
            Commands.runOnce(() -> leds.setColor(allianceColor)),
            swerve.driveToPoseCommand(speakerPose)
        );
    }
}
```

### Example 2: Angle Control

```java
public class TurretSubsystem extends SubsystemBase {
    private final TalonFX turretMotor;
    private final PIDController turretPID;
    private final CANcoder encoder;

    public TurretSubsystem() {
        turretPID = new PIDController(0.02, 0, 0.001);
        turretPID.enableContinuousInput(-180, 180);
    }

    public Command aimAtAngleCommand(double targetDeg) {
        return run(() -> {
            double currentAngle = encoder.getAbsolutePosition().getValueAsDouble();

            // Normalize current angle
            currentAngle = Utils.wrapAngleDeg(currentAngle);

            // Calculate shortest path to target
            double error = Utils.shortestAngleDifferenceDeg(currentAngle, targetDeg);

            // PID control
            double output = turretPID.calculate(0, error);
            turretMotor.set(output);

            // Dashboard
            Dash.add("Turret Angle", () -> currentAngle);
            Dash.add("Turret Error", () -> error);
            Dash.add("Turret Target", () -> targetDeg);
        }).until(() ->
            Math.abs(Utils.shortestAngleDifferenceDeg(
                Utils.wrapAngleDeg(encoder.getAbsolutePosition().getValueAsDouble()),
                targetDeg
            )) < 2.0  // Within 2 degrees
        );
    }
}
```

### Example 3: Sensor Debouncing

```java
public class IntakeSubsystem extends SubsystemBase {
    private final TalonFX intakeMotor;
    private final DigitalInput gamePieceSensor;

    // Debouncing state
    private boolean lastSensorValue = false;
    private long lastSensorChangeTime = 0;
    private static final long DEBOUNCE_TIME_MS = 100;

    @Override
    public void periodic() {
        // Get current sensor state
        boolean currentValue = !gamePieceSensor.get();  // Inverted sensor

        // Debounce the sensor
        boolean hasGamePiece = Utils.debounce(
            currentValue,
            lastSensorValue,
            lastSensorChangeTime,
            DEBOUNCE_TIME_MS
        );

        // Update tracking
        if (currentValue != lastSensorValue) {
            lastSensorChangeTime = System.currentTimeMillis();
            lastSensorValue = currentValue;
        }

        // Stop motor when game piece detected
        if (Utils.risingEdge(hasGamePiece, lastHasGamePiece)) {
            intakeMotor.stopMotor();
            Utils.reportInfo("Game piece acquired!");
        }

        lastHasGamePiece = hasGamePiece;

        // Dashboard
        Dash.add("Has Game Piece", () -> hasGamePiece);
        Dash.add("Sensor Raw", () -> currentValue);
    }

    private boolean lastHasGamePiece = false;
}
```

### Example 4: Array Operations

```java
public class SwerveSubsystem extends SubsystemBase {
    private final SwerveModule[] modules = new SwerveModule[4];

    public Command checkModuleHealthCommand() {
        return Commands.runOnce(() -> {
            // Get all module temperatures
            double[] temps = new double[4];
            for (int i = 0; i < 4; i++) {
                temps[i] = modules[i].getDriveMotorTemp();
            }

            // Check average temperature
            double avgTemp = Utils.average(temps);
            if (avgTemp > 60.0) {
                Utils.reportWarning("Average module temp high: " + avgTemp + "°C");
            }

            // Check for hottest module
            double maxTemp = Utils.max(temps);
            int hottestIndex = Utils.maxIndex(temps);
            if (maxTemp > 70.0) {
                Utils.reportError("Module " + hottestIndex +
                    " temperature critical: " + maxTemp + "°C");
            }

            // Display on dashboard
            Dash.addDoubleArray("Module Temps", () -> temps);
            Dash.add("Avg Temp", () -> avgTemp);
            Dash.add("Max Temp", () -> maxTemp);
        });
    }
}
```

---

## API Reference

### Epsilon Comparison

```java
static boolean epsilonEquals(double a, double b, double epsilon)
```
Compares two doubles with specified epsilon tolerance.

---

```java
static boolean epsilonEquals(double a, double b)
```
Compares two doubles with default epsilon (1e-9).

---

```java
static boolean epsilonEquals(Twist2d a, Twist2d b)
```
Compares two Twist2d objects (dx, dy, dtheta) with default epsilon.

---

### Geometry Utilities

```java
static Transform2d toTransform2d(Pose2d pose)
```
Converts a Pose2d to a Transform2d.

---

```java
static Pose2d toPose2d(Transform2d transform)
```
Converts a Transform2d to a Pose2d.

---

```java
static Pose2d toPose2d(Translation2d translation)
```
Creates a Pose2d from Translation2d with zero rotation.

---

```java
static Pose2d toPose2d(Rotation2d rotation)
```
Creates a Pose2d from Rotation2d with zero translation.

---

```java
static Pose2d inverse(Pose2d pose)
```
Computes the inverse of a Pose2d.

---

```java
static Twist2d multiply(Twist2d twist, double scalar)
```
Multiplies all Twist2d components by a scalar.

---

```java
static Twist2d toTwist2d(ChassisSpeeds speeds)
```
Converts ChassisSpeeds to Twist2d.

---

```java
static Pose2d withTranslation(Pose2d pose, Translation2d translation)
```
Returns a new Pose2d with the given translation, preserving original rotation.

---

```java
static Pose2d withRotation(Pose2d pose, Rotation2d rotation)
```
Returns a new Pose2d with the given rotation, preserving original translation.

---

```java
static Transform3d toTransform3d(Pose3d pose)
```
Converts a Pose3d to a Transform3d.

---

```java
static Pose3d toPose3d(Transform3d transform)
```
Converts a Transform3d to a Pose3d.

---

```java
static Transform2d toTransform2d(Transform3d transform)
```
Reduces a Transform3d to a Transform2d (XY plane projection).

---

### Alliance Utilities

```java
static boolean isOnRedAlliance()
```
Returns `true` if robot is on red alliance.

---

```java
static boolean isOnBlueAlliance()
```
Returns `true` if robot is on blue alliance.

---

```java
static Color getAllianceColor()
```
Returns alliance color: `Color.kRed`, `Color.kBlue`, or `Color.kBlack` (unknown).

---

```java
static double mirrorXForAlliance(double x, double fieldLength)
```
Mirrors X coordinate for current alliance.

---

```java
static Translation2d mirrorTranslationForAlliance(Translation2d translation, double fieldLength)
```
Mirrors Translation2d for current alliance.

---

```java
static Pose2d mirrorPoseForAlliance(Pose2d pose, double fieldLength)
```
Mirrors Pose2d (position and rotation) for current alliance.

---

### Math Utilities

```java
static double lerp(double start, double end, double t)
```
Linear interpolation from start to end.

---

```java
static double mapRange(double value, double inMin, double inMax, double outMin, double outMax)
```
Maps value from one range to another.

---

```java
static boolean isInRange(double value, double min, double max)
```
Checks if value is within range [min, max].

---

```java
static double percentProgress(double current, double start, double end)
```
Calculates progress from start to end as 0.0 to 1.0.

---

### Angle Utilities

```java
static double wrapAngleDeg(double angleDeg)
```
Normalizes angle to [-180, 180] degrees.

---

```java
static double wrapAngleRad(double angleRad)
```
Normalizes angle to [-π, π] radians.

---

```java
static double shortestAngleDifferenceDeg(double currentDeg, double targetDeg)
```
Calculates shortest angular difference in degrees.

---

```java
static double shortestAngleDifferenceRad(double currentRad, double targetRad)
```
Calculates shortest angular difference in radians.

---

### Boolean Utilities

```java
static boolean debounce(boolean currentValue, boolean lastValue,
                       long lastChangeTimeMs, long debounceTimeMs)
```
Debounces boolean value to prevent rapid toggling.

---

```java
static boolean risingEdge(boolean currentValue, boolean lastValue)
```
Detects false to true transition.

---

```java
static boolean fallingEdge(boolean currentValue, boolean lastValue)
```
Detects true to false transition.

---

### Array Utilities

```java
static double average(double[] values)
```
Calculates average of array.

---

```java
static double max(double[] values)
```
Finds maximum value in array.

---

```java
static double min(double[] values)
```
Finds minimum value in array.

---

```java
static int minIndex(double[] values)
```
Finds index of minimum value.

---

```java
static int maxIndex(double[] values)
```
Finds index of maximum value.

---

### Time Utilities

```java
static boolean hasElapsed(long startTimeMs, long durationMs)
```
Checks if duration has elapsed since start time.

---

```java
static long secondsToMillis(double seconds)
```
Converts seconds to milliseconds.

---

```java
static double millisToSeconds(long millis)
```
Converts milliseconds to seconds.

---

### Error Reporting

```java
static void reportError(String message)
```
Reports error to DriverStation.

---

```java
static void reportWarning(String message)
```
Reports warning to DriverStation.

---

```java
static void reportInfo(String message)
```
Reports info message to console.

---

## Best Practices

### ✓ DO

- **Use shortestAngleDifference for ALL angle control** - Prevents wrong-way rotation
- **Debounce noisy sensors** - Especially for game piece detection
- **Mirror poses for alliance** - Define once for blue, mirror for red
- **Check ranges before operations** - Prevent mechanism damage
- **Use array utilities for multiple sensors** - Find hottest motor, closest target
- **Report errors early** - Help debug during initialization
- **Normalize angles before comparison** - Always wrap angles first

### ✗ DON'T

- **Don't use subtraction for angles** - Use `shortestAngleDifference` instead
- **Don't duplicate WPILib functions** - Use `MathUtil.clamp`, `Units.convert`, etc.
- **Don't skip debouncing sensors** - Noisy sensors cause false triggers
- **Don't hardcode field dimensions** - Use constants (16.54m)
- **Don't forget alliance mirroring** - Test on both red and blue
- **Don't ignore edge cases** - Check for empty arrays, division by zero

### Common Mistakes

```java
// ❌ WRONG - Don't subtract angles directly
double error = targetAngle - currentAngle;
// If current = 10° and target = 350°, error = 340° (wrong!)

// ✓ CORRECT - Use shortest angular difference
double error = Utils.shortestAngleDifferenceDeg(currentAngle, targetAngle);
// Returns -20° (rotate 20° counter-clockwise)
```

```java
// ❌ WRONG - Noisy sensor without debouncing
if (sensor.get()) {
    intake.stop();  // May trigger falsely!
}

// ✓ CORRECT - Debounce the sensor
if (debouncedSensorValue) {
    intake.stop();  // Only triggers when stable
}
```

```java
// ❌ WRONG - Hardcoded for blue alliance
Pose2d speaker = new Pose2d(0.5, 5.5, Rotation2d.fromDegrees(0));

// ✓ CORRECT - Mirror for alliance
Pose2d blueSpeaker = new Pose2d(0.5, 5.5, Rotation2d.fromDegrees(0));
Pose2d speaker = Utils.mirrorPoseForAlliance(blueSpeaker, 16.54);
```

---

## Relation to WPILib

Utils complements (doesn't duplicate) WPILib utilities:

### Use WPILib For:

- **Clamping**: `MathUtil.clamp(value, min, max)`
- **Deadband**: `MathUtil.applyDeadband(value, deadband)`
- **Interpolation**: `MathUtil.interpolate(start, end, t)` for geometries
- **Unit Conversion**: `Units.inchesToMeters()`, `Units.degreesToRadians()`, etc.
- **Input Shaping**: `MathUtil.inputModulus()` for continuous inputs

### Use Utils For:

- **Alliance Logic**: `isOnRedAlliance()`, `mirrorPoseForAlliance()`
- **Angle Control**: `shortestAngleDifference()`, `wrapAngle()`
- **Sensor Debouncing**: `debounce()`, `risingEdge()`, `fallingEdge()`
- **Array Operations**: `average()`, `min()`, `max()`, `minIndex()`, `maxIndex()`
- **Range Operations**: `mapRange()`, `isInRange()`, `percentProgress()`

---

## See Also

- [Buttons Documentation](Buttons.md) - Controller input abstraction
- [Dash Documentation](Dash.md) - Shuffleboard utility
- [StateMachine Documentation](StateMachine.md) - State machine utility
- [Utils Documentation](README.md) - Overview of all utilities
- [WPILib MathUtil](https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/math/MathUtil.html)
- [WPILib Units](https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/math/util/Units.html)

---

**Last Updated:** 2026-01-27
