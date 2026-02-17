# Dash - Shuffleboard Utility

Simple, clean API for adding telemetry data to Shuffleboard without dealing with NetworkTables complexity.

---

## Table of Contents

- [Overview](#overview)
- [Quick Start](#quick-start)
- [Basic Usage](#basic-usage)
- [Tab Management](#tab-management)
- [Tunable Values](#tunable-values)
- [Command Buttons](#command-buttons)
- [Advanced Features](#advanced-features)
- [Complete Examples](#complete-examples)
- [API Reference](#api-reference)
- [Best Practices](#best-practices)
- [Troubleshooting](#troubleshooting)

---

## Overview

`Dash` provides a simplified interface to WPILib's Shuffleboard dashboard. Instead of dealing with NetworkTables entries and Shuffleboard complexity, you can add telemetry data with a single method call.

###Key Features

- **Auto-Updating Values**: All values automatically update every robot loop
- **Type-Safe**: Compile-time type checking for all values
- **Multiple Tabs**: Organize data across different Shuffleboard tabs
- **Tunable Values**: Edit PID constants and parameters live during testing
- **Command Buttons**: Add buttons to trigger commands from Shuffleboard
- **Simple API**: Clean, intuitive method names

### When to Use

✅ **Use Dash when:**
- You need to display telemetry data (speeds, positions, states)
- You want to tune PID constants live
- You need debug buttons for testing subsystems
- You want to organize data across multiple tabs
- You prefer simple API over Shuffleboard complexity

❌ **Don't use Dash when:**
- You need complex custom widgets
- You're building a custom dashboard (not Shuffleboard)

---

## Quick Start

### 1. Add Basic Telemetry

```java
public class SwerveSubsystem extends SubsystemBase {
    public SwerveSubsystem() {
        // Add auto-updating values
        Dash.add("Robot Speed", () -> getRobotVelocity());
        Dash.add("Gyro Angle", () -> getHeading().getDegrees());
        Dash.add("Is At Target", () -> isAtTarget());
    }
}
```

### 2. Add Tunable PID Constants

```java
public class ArmSubsystem extends SubsystemBase {
    private GenericEntry kP;
    private GenericEntry kD;

    public ArmSubsystem() {
        // Add tunable PID values
        kP = Dash.addTunable("Arm kP", 0.5);
        kD = Dash.addTunable("Arm kD", 0.1);
    }

    @Override
    public void periodic() {
        // Read tuned values every loop
        controller.setPID(kP.getDouble(0.5), 0, kD.getDouble(0.1));
    }
}
```

### 3. Add Command Buttons

```java
public class RobotContainer {
    public RobotContainer() {
        // Add test buttons
        Dash.addCommand("Reset Gyro", swerve.resetGyroCommand());
        Dash.addCommand("Deploy Intake", intake.deployCommand());
    }
}
```

---

## Basic Usage

### Adding Values

All `add()` methods take a Supplier that provides the current value. The value auto-updates every robot loop.

#### Numbers (double)

```java
// Robot velocities
Dash.add("Forward Speed", () -> swerve.getForwardVelocity());
Dash.add("Strafe Speed", () -> swerve.getStrafeVelocity());

// Positions
Dash.add("Arm Angle", () -> arm.getAngleDegrees());
Dash.add("Elevator Height", () -> elevator.getHeightInches());

// From sensors
Dash.add("Left Encoder", () -> leftEncoder.getPosition());
Dash.add("Right Encoder", () -> rightEncoder.getPosition());
```

#### Integers (long)

```java
// Counts
Dash.add("Loop Counter", () -> loopCount);
Dash.add("Auto Step", () -> (long) autoStateMachine.getCurrentStep());

// Times
Dash.add("Match Time", () -> (long) DriverStation.getMatchTime());
```

#### Booleans

```java
// Subsystem states
Dash.add("Intake Deployed", () -> intake.isDeployed());
Dash.add("At Target Position", () -> elevator.isAtTarget());
Dash.add("Shooter At Speed", () -> shooter.isAtSpeed());

// Sensor values
Dash.add("Has Game Piece", () -> intakeSensor.get());
Dash.add("Limit Switch", () -> limitSwitch.get());
```

#### Strings

```java
// State machines
Dash.add("Arm State", () -> armStateMachine.getCurrentState().toString());
Dash.add("Auto Mode", () -> autonomousChooser.getSelected());

// Alliance color
Dash.add("Alliance", () -> DriverStation.getAlliance().toString());

// Debug messages
Dash.add("Status", () -> getStatusMessage());
```

#### Arrays

```java
// Swerve module speeds
Dash.addDoubleArray("Module Speeds", () -> new double[] {
    frontLeft.getVelocity(),
    frontRight.getVelocity(),
    rearLeft.getVelocity(),
    rearRight.getVelocity()
});

// Multiple states
Dash.addStringArray("Subsystem States", () -> new String[] {
    intake.getState().toString(),
    shooter.getState().toString(),
    climber.getState().toString()
});
```

---

## Tab Management

Organize your dashboard data across multiple tabs for better organization.

### Using Custom Tabs

```java
public class SwerveSubsystem extends SubsystemBase {
    public SwerveSubsystem() {
        // Switch to Swerve tab
        Dash.useTab("Swerve");

        // All adds go to "Swerve" tab
        Dash.add("FL Velocity", () -> frontLeft.getVelocity());
        Dash.add("FR Velocity", () -> frontRight.getVelocity());
        Dash.add("RL Velocity", () -> rearLeft.getVelocity());
        Dash.add("RR Velocity", () -> rearRight.getVelocity());

        Dash.add("FL Angle", () -> frontLeft.getAngle());
        Dash.add("FR Angle", () -> frontRight.getAngle());
        Dash.add("RL Angle", () -> rearLeft.getAngle());
        Dash.add("RR Angle", () -> rearRight.getAngle());

        // Switch back to default tab
        Dash.useDefaultTab();
    }
}
```

### Organizing by Subsystem

```java
public class RobotContainer {
    public RobotContainer() {
        // Swerve tab
        Dash.useTab("Swerve");
        Dash.add("Robot Speed", () -> swerve.getSpeed());
        Dash.add("Gyro Angle", () -> swerve.getHeading().getDegrees());

        // Intake tab
        Dash.useTab("Intake");
        Dash.add("Deployed", () -> intake.isDeployed());
        Dash.add("Has Piece", () -> intake.hasGamePiece());

        // Shooter tab
        Dash.useTab("Shooter");
        Dash.add("Velocity", () -> shooter.getVelocity());
        Dash.add("At Speed", () -> shooter.isAtSpeed());

        // Back to default
        Dash.useDefaultTab();
    }
}
```

### Tab Methods

```java
// Switch to custom tab
Dash.useTab("Debug");

// Switch to default tab ("debug" from Constants)
Dash.useDefaultTab();

// Get current tab object
ShuffleboardTab tab = Dash.getCurrentTab();

// Focus current tab in Shuffleboard UI
Dash.selectCurrentTab();
```

---

## Widget Positioning (v2026.3.0+)

All `add`, `addTunable`, `addDoubleArray`, `addStringArray`, and `addCommand` methods have overloads that accept `column` and `row` parameters for precise grid placement on Shuffleboard.

### Basic Positioning

```java
// Place widgets at specific grid positions (column, row are 0-indexed)
Dash.add("Speed", () -> swerve.getSpeed(), 0, 0);         // Top-left
Dash.add("Heading", () -> swerve.getHeading(), 1, 0);      // Next to it
Dash.add("At Target", () -> swerve.isAtTarget(), 0, 1);    // Below speed
```

### Organized Dashboard Layout

```java
Dash.useTab("Shooter");

// Row 0: Velocity readings
Dash.add("Left Velocity", () -> leftShooter.getVelocity(), 0, 0);
Dash.add("Right Velocity", () -> rightShooter.getVelocity(), 1, 0);
Dash.add("At Speed", () -> shooter.isAtSpeed(), 2, 0);

// Row 1: Tunable PID
kP = Dash.addTunable("kP", 0.1, 0, 1);
kI = Dash.addTunable("kI", 0.0, 1, 1);
kD = Dash.addTunable("kD", 0.01, 2, 1);

// Row 2: Commands
Dash.addCommand("Spin Up", shooter.spinUpCommand(), 0, 2);
Dash.addCommand("Stop", shooter.stopCommand(), 1, 2);

Dash.useDefaultTab();
```

### Positioning Methods

All positioning overloads follow the same pattern — add `column` and `row` as the last two parameters:

```java
// Values
Dash.add("name", doubleSupplier, column, row);
Dash.add("name", longSupplier, column, row);
Dash.add("name", booleanSupplier, column, row);
Dash.add("name", stringSupplier, column, row);

// Arrays
Dash.addDoubleArray("name", arraySupplier, column, row);
Dash.addStringArray("name", arraySupplier, column, row);

// Tunables
Dash.addTunable("name", defaultDouble, column, row);
Dash.addTunable("name", defaultBool, column, row);
Dash.addTunable("name", defaultString, column, row);

// Commands
Dash.addCommand("name", command, column, row);
```

---

## Tunable Values

Tunable values can be edited in real-time on Shuffleboard - perfect for PID tuning and testing.

### PID Tuning

```java
public class ShooterSubsystem extends SubsystemBase {
    private final TalonFX shooterMotor;
    private GenericEntry kP, kI, kD, kF;

    public ShooterSubsystem() {
        Dash.useTab("Shooter");

        // Add tunable PID constants
        kP = Dash.addTunable("Shooter kP", 0.1);
        kI = Dash.addTunable("Shooter kI", 0.0);
        kD = Dash.addTunable("Shooter kD", 0.01);
        kF = Dash.addTunable("Shooter kF", 0.045);

        Dash.useDefaultTab();
    }

    @Override
    public void periodic() {
        // Read tuned values and update motor controller
        var config = new Slot0Configs();
        config.kP = kP.getDouble(0.1);
        config.kI = kI.getDouble(0.0);
        config.kD = kD.getDouble(0.01);
        config.kV = kF.getDouble(0.045);

        shooterMotor.getConfigurator().apply(config);
    }
}
```

### Speed Limits

```java
public class SwerveSubsystem extends SubsystemBase {
    private GenericEntry maxSpeed;
    private GenericEntry maxRotation;

    public SwerveSubsystem() {
        maxSpeed = Dash.addTunable("Max Speed", 5.0);
        maxRotation = Dash.addTunable("Max Rotation", 10.0);
    }

    public void drive(ChassisSpeeds speeds) {
        // Apply tunable speed limits
        double speedLimit = maxSpeed.getDouble(5.0);
        double rotationLimit = maxRotation.getDouble(10.0);

        speeds = new ChassisSpeeds(
            MathUtil.clamp(speeds.vxMetersPerSecond, -speedLimit, speedLimit),
            MathUtil.clamp(speeds.vyMetersPerSecond, -speedLimit, speedLimit),
            MathUtil.clamp(speeds.omegaRadiansPerSecond, -rotationLimit, rotationLimit)
        );

        // ... apply speeds
    }
}
```

### Feature Toggles

```java
public class VisionSubsystem extends SubsystemBase {
    private GenericEntry enableVision;
    private GenericEntry enableMultiTag;

    public VisionSubsystem() {
        Dash.useTab("Vision");
        enableVision = Dash.addTunable("Enable Vision", true);
        enableMultiTag = Dash.addTunable("Enable MultiTag", false);
        Dash.useDefaultTab();
    }

    @Override
    public void periodic() {
        if (!enableVision.getBoolean(true)) {
            return;  // Vision disabled
        }

        if (enableMultiTag.getBoolean(false)) {
            updateWithMultiTag();
        } else {
            updateWithSingleTag();
        }
    }
}
```

### Auto Path Selection

```java
public class RobotContainer {
    private GenericEntry autoPath;

    public RobotContainer() {
        autoPath = Dash.addTunable("Auto Path", "4PieceAuto");
    }

    public Command getAutonomousCommand() {
        String pathName = autoPath.getString("4PieceAuto");
        return swerve.getAutonomousCommand(pathName);
    }
}
```

---

## Command Buttons

Add command buttons to Shuffleboard for easy testing and debugging.

### Basic Commands

```java
public class RobotContainer {
    public RobotContainer() {
        // System commands
        Dash.addCommand("Reset Gyro", swerve.resetGyroCommand());
        Dash.addCommand("Lock Wheels", swerve.lockWheelsCommand());

        // Subsystem testing
        Dash.addCommand("Deploy Intake", intake.deployCommand());
        Dash.addCommand("Retract Intake", intake.retractCommand());
        Dash.addCommand("Spin Up Shooter", shooter.spinUpCommand());
        Dash.addCommand("Stop Shooter", shooter.stopCommand());
    }
}
```

### Organized by Tab

```java
public class RobotContainer {
    public RobotContainer() {
        // Swerve commands
        Dash.useTab("Swerve");
        Dash.addCommand("Reset Gyro", swerve.resetGyroCommand());
        Dash.addCommand("Center Modules", swerve.centerModulesCommand());
        Dash.addCommand("Drive to Amp", swerve.driveToPoseCommand(FieldPositions.AMP));

        // Intake commands
        Dash.useTab("Intake");
        Dash.addCommand("Deploy", intake.deployCommand());
        Dash.addCommand("Retract", intake.retractCommand());
        Dash.addCommand("Run Intake", intake.intakeCommand());
        Dash.addCommand("Eject", intake.ejectCommand());

        // Back to default
        Dash.useDefaultTab();
    }
}
```

### Complex Commands

```java
public class RobotContainer {
    public RobotContainer() {
        // Auto-shoot sequence
        Dash.addCommand("Auto Shoot", Commands.sequence(
            shooter.spinUpCommand().withTimeout(1.5),
            Commands.waitUntil(() -> shooter.isAtSpeed()),
            shooter.shootCommand()
        ));

        // Climb sequence
        Dash.addCommand("Deploy Climber", Commands.sequence(
            climber.deployCommand(),
            Commands.waitSeconds(0.5),
            climber.extendCommand().withTimeout(2.0)
        ));

        // Emergency stop all
        Dash.addCommand("EMERGENCY STOP", Commands.parallel(
            shooter.stopCommand(),
            intake.stopCommand(),
            climber.stopCommand()
        ).withTimeout(0.1));
    }
}
```

---

## Advanced Features

### Clear Tab

```java
public class Robot extends TimedRobot {
    @Override
    public void autonomousInit() {
        // Clear debug tab during competition
        Dash.useTab("Debug");
        Dash.clearTab();
        Dash.useDefaultTab();
    }
}
```

### Select Tab Programmatically

```java
public class Robot extends TimedRobot {
    @Override
    public void autonomousInit() {
        // Show autonomous tab
        Dash.useTab("Autonomous");
        Dash.selectCurrentTab();  // Brings tab to front
    }

    @Override
    public void teleopInit() {
        // Show teleop tab
        Dash.useTab("Teleop");
        Dash.selectCurrentTab();
    }
}
```

### Access Underlying Tab

```java
// For advanced Shuffleboard features not in Dash
ShuffleboardTab tab = Dash.getCurrentTab();

// Use Shuffleboard API directly
tab.add("Custom Widget", customSendable)
    .withWidget(BuiltInWidgets.kGraph)
    .withPosition(0, 0)
    .withSize(4, 2);
```

---

## Complete Examples

### Example 1: Subsystem Telemetry

```java
package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.adambots.lib.utils.Dash;

public class IntakeSubsystem extends SubsystemBase {
    private final TalonFX intakeMotor;
    private final DigitalInput gamePieceSensor;
    private final Solenoid deploymentSolenoid;

    private StateMachine<State, Props> stateMachine;

    enum State { STOWED, DEPLOYED, INTAKING, EJECTING }

    public IntakeSubsystem() {
        // Initialize hardware
        intakeMotor = new TalonFX(10);
        gamePieceSensor = new DigitalInput(0);
        deploymentSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, 0);

        // Set up dashboard telemetry on custom tab
        Dash.useTab("Intake");

        // Basic state
        Dash.add("Current State", () -> stateMachine.getCurrentState().toString());
        Dash.add("Deployed", () -> isDeployed());
        Dash.add("Has Game Piece", () -> hasGamePiece());

        // Motor data
        Dash.add("Motor Velocity", () -> intakeMotor.getVelocity().getValueAsDouble());
        Dash.add("Motor Current", () -> intakeMotor.getStatorCurrent().getValueAsDouble());
        Dash.add("Motor Temp", () -> intakeMotor.getDeviceTemp().getValueAsDouble());

        // Sensor data
        Dash.add("Sensor Raw", () -> gamePieceSensor.get());

        // Test commands
        Dash.addCommand("Deploy", deployCommand());
        Dash.addCommand("Retract", retractCommand());
        Dash.addCommand("Run Intake", intakeCommand());
        Dash.addCommand("Eject", ejectCommand());

        Dash.useDefaultTab();
    }

    public boolean isDeployed() {
        return deploymentSolenoid.get();
    }

    public boolean hasGamePiece() {
        return !gamePieceSensor.get();  // Sensor is normally open
    }

    // Command factories...
    public Command deployCommand() { /* ... */ }
    public Command retractCommand() { /* ... */ }
    public Command intakeCommand() { /* ... */ }
    public Command ejectCommand() { /* ... */ }
}
```

### Example 2: PID Tuning

```java
package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.GenericEntry;
import com.adambots.lib.utils.Dash;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

public class ShooterSubsystem extends SubsystemBase {
    private final TalonFX leftShooter;
    private final TalonFX rightShooter;

    // Tunable PID constants
    private GenericEntry kP, kI, kD, kV;
    private GenericEntry targetVelocity;

    public ShooterSubsystem() {
        leftShooter = new TalonFX(15);
        rightShooter = new TalonFX(16);

        Dash.useTab("Shooter");

        // Add tunable PID values
        kP = Dash.addTunable("kP", 0.1);
        kI = Dash.addTunable("kI", 0.0);
        kD = Dash.addTunable("kD", 0.01);
        kV = Dash.addTunable("kV", 0.12);

        // Tunable target velocity for testing
        targetVelocity = Dash.addTunable("Target Velocity", 80.0);

        // Current state
        Dash.add("Left Velocity", () -> leftShooter.getVelocity().getValueAsDouble());
        Dash.add("Right Velocity", () -> rightShooter.getVelocity().getValueAsDouble());
        Dash.add("At Speed", () -> isAtSpeed());

        // Test commands
        Dash.addCommand("Spin Up", spinUpCommand());
        Dash.addCommand("Stop", stopCommand());

        Dash.useDefaultTab();
    }

    @Override
    public void periodic() {
        // Apply tuned PID values every loop
        var config = new Slot0Configs();
        config.kP = kP.getDouble(0.1);
        config.kI = kI.getDouble(0.0);
        config.kD = kD.getDouble(0.01);
        config.kV = kV.getDouble(0.12);

        leftShooter.getConfigurator().apply(config);
        rightShooter.getConfigurator().apply(config);
    }

    public Command spinUpCommand() {
        return runOnce(() -> {
            double target = targetVelocity.getDouble(80.0);
            leftShooter.setControl(new VelocityVoltage(target));
            rightShooter.setControl(new VelocityVoltage(target));
        });
    }

    public Command stopCommand() {
        return runOnce(() -> {
            leftShooter.stopMotor();
            rightShooter.stopMotor();
        });
    }

    public boolean isAtSpeed() {
        double target = targetVelocity.getDouble(80.0);
        double leftVel = leftShooter.getVelocity().getValueAsDouble();
        double rightVel = rightShooter.getVelocity().getValueAsDouble();
        return Math.abs(leftVel - target) < 2.0 && Math.abs(rightVel - target) < 2.0;
    }
}
```

### Example 3: Multi-Tab Organization

```java
package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import com.adambots.lib.utils.Dash;
import frc.robot.subsystems.*;

public class RobotContainer {
    // Subsystems
    private final SwerveSubsystem swerve = new SwerveSubsystem();
    private final IntakeSubsystem intake = new IntakeSubsystem();
    private final ShooterSubsystem shooter = new ShooterSubsystem();
    private final ClimberSubsystem climber = new ClimberSubsystem();

    public RobotContainer() {
        setupDashboard();
        configureBindings();
    }

    private void setupDashboard() {
        // ================================================================
        // Main Tab - Critical Match Info
        // ================================================================
        Dash.useTab("Match");

        Dash.add("Match Time", () -> (long) DriverStation.getMatchTime());
        Dash.add("Alliance", () -> DriverStation.getAlliance().toString());
        Dash.add("Robot Enabled", () -> DriverStation.isEnabled());

        Dash.add("Swerve Speed", () -> swerve.getSpeed());
        Dash.add("Has Game Piece", () -> intake.hasGamePiece());
        Dash.add("Shooter Ready", () -> shooter.isAtSpeed());

        // ================================================================
        // Swerve Tab - Drivetrain Detailed
        // ================================================================
        Dash.useTab("Swerve");

        Dash.add("Robot X", () -> swerve.getPose().getX());
        Dash.add("Robot Y", () -> swerve.getPose().getY());
        Dash.add("Robot Heading", () -> swerve.getHeading().getDegrees());

        Dash.addDoubleArray("Module Velocities", () -> new double[] {
            swerve.getFrontLeft().getVelocity(),
            swerve.getFrontRight().getVelocity(),
            swerve.getRearLeft().getVelocity(),
            swerve.getRearRight().getVelocity()
        });

        Dash.addCommand("Reset Gyro", swerve.resetGyroCommand());
        Dash.addCommand("Lock Wheels", swerve.lockWheelsCommand());

        // ================================================================
        // Debug Tab - Testing and Tuning
        // ================================================================
        Dash.useTab("Debug");

        // All subsystem states
        Dash.addStringArray("Subsystem States", () -> new String[] {
            "Intake: " + intake.getCurrentState(),
            "Shooter: " + shooter.getCurrentState(),
            "Climber: " + climber.getCurrentState()
        });

        // Vision data
        Dash.add("Has Vision Target", () -> swerve.hasVisionTarget());
        Dash.add("Vision Distance", () -> swerve.getVisionDistance());

        // Back to default
        Dash.useDefaultTab();
    }

    private void configureBindings() {
        // ... button bindings ...
    }

    public Command getAutonomousCommand() {
        return swerve.getAutonomousCommand("4PieceAuto");
    }
}
```

---

## API Reference

### Tab Management

```java
static void useTab(String tabName)
```
Switch to a different Shuffleboard tab. All subsequent `add()` calls use this tab.

---

```java
static void useDefaultTab()
```
Switch back to the default tab (from `Constants.kDefaultShuffleboardTab`).

---

```java
static ShuffleboardTab getCurrentTab()
```
Get the currently active ShuffleboardTab object.

---

```java
static void selectCurrentTab()
```
Bring the current tab to the front in the Shuffleboard UI.

---

```java
static void clearTab()
```
Remove all widgets from the current tab. **Warning**: Clears ALL widgets, not just those added by Dash.

---

### Add Values (Auto-Update)

```java
static void add(String name, DoubleSupplier supplier)
static void add(String name, DoubleSupplier supplier, int column, int row)
```
Add auto-updating double value. Position overload places widget at specific grid location.

---

```java
static void add(String name, LongSupplier supplier)
static void add(String name, LongSupplier supplier, int column, int row)
```
Add auto-updating long/integer value.

---

```java
static void add(String name, BooleanSupplier supplier)
static void add(String name, BooleanSupplier supplier, int column, int row)
```
Add auto-updating boolean value.

---

```java
static void add(String name, Supplier<String> supplier)
static void add(String name, Supplier<String> supplier, int column, int row)
```
Add auto-updating String value.

---

```java
static void addDoubleArray(String name, Supplier<double[]> supplier)
static void addDoubleArray(String name, Supplier<double[]> supplier, int column, int row)
```
Add auto-updating double array.

---

```java
static void addStringArray(String name, Supplier<String[]> supplier)
static void addStringArray(String name, Supplier<String[]> supplier, int column, int row)
```
Add auto-updating String array.

---

### Tunable Values

```java
static GenericEntry addTunable(String name, double defaultValue)
static GenericEntry addTunable(String name, double defaultValue, int column, int row)
```
Add tunable number (editable text view on Shuffleboard). Returns `GenericEntry` to read current value.

---

```java
static GenericEntry addTunable(String name, boolean defaultValue)
static GenericEntry addTunable(String name, boolean defaultValue, int column, int row)
```
Add tunable boolean (toggle button on Shuffleboard). Returns `GenericEntry` to read current value.

---

```java
static GenericEntry addTunable(String name, String defaultValue)
static GenericEntry addTunable(String name, String defaultValue, int column, int row)
```
Add tunable String (text field on Shuffleboard). Returns `GenericEntry` to read current value.

---

### Commands

```java
static void addCommand(String name, Command command)
static void addCommand(String name, Command command, int column, int row)
```
Add command button to Shuffleboard. Button runs the command when pressed.

---

## Best Practices

### ✓ DO

- **Organize by tab** - Group related data on custom tabs
- **Use tunable values for PID** - Makes tuning much faster
- **Add command buttons for testing** - Easy to test subsystems
- **Use descriptive names** - "Shooter Velocity" not "vel"
- **Add units to names** - "Arm Angle (deg)" not "Arm Angle"
- **Keep suppliers simple** - Just return a value, don't compute
- **Initialize in constructor** - Add values once during subsystem construction

### ✗ DON'T

- **Don't add values in periodic()** - Only add once in constructor
- **Don't compute in suppliers** - Keep them fast (just return a field)
- **Don't add too many values** - Clutters dashboard, slows NetworkTables
- **Don't use for production** - Remove debug values before competition
- **Don't forget units** - Always specify units in the name
- **Don't use cryptic abbreviations** - "FL Vel" vs "Front Left Velocity"

### Naming Conventions

```java
// ✓ GOOD - Clear with units
Dash.add("Shooter Velocity (RPS)", () -> shooter.getVelocity());
Dash.add("Arm Angle (deg)", () -> arm.getAngleDegrees());
Dash.add("Elevator Height (in)", () -> elevator.getHeightInches());

// ✗ BAD - Unclear, no units
Dash.add("vel", () -> shooter.getVelocity());
Dash.add("angle", () -> arm.getAngle());
Dash.add("height", () -> elevator.getHeight());
```

### Performance

```java
// ✓ GOOD - Fast, just returns field
private double currentVelocity;

public void periodic() {
    currentVelocity = motor.getVelocity().getValueAsDouble();
}

// Constructor
Dash.add("Velocity", () -> currentVelocity);

// ✗ BAD - Calls hardware every dashboard update
Dash.add("Velocity", () -> motor.getVelocity().getValueAsDouble());
```

---

## Troubleshooting

### Values Not Updating

**Symptom**: Dashboard shows old/stale values

**Causes**:
- Using static value instead of Supplier
- Supplier not returning current value

**Fix**:
```java
// ✗ WRONG - Static value, won't update
Dash.add("Speed", () -> 5.0);

// ✓ CORRECT - Supplier returns current value
Dash.add("Speed", () -> getCurrentSpeed());
```

---

### Values Added Multiple Times

**Symptom**: Multiple entries with same name on Shuffleboard

**Cause**: Adding values in `periodic()` instead of constructor

**Fix**:
```java
// ✗ WRONG - Adds new entry every loop
@Override
public void periodic() {
    Dash.add("Speed", () -> getSpeed());  // DON'T DO THIS
}

// ✓ CORRECT - Add once in constructor
public MySubsystem() {
    Dash.add("Speed", () -> getSpeed());  // Add once
}
```

---

### Wrong Tab

**Symptom**: Values appearing on wrong tab

**Cause**: Forgot to switch tabs before adding

**Fix**:
```java
// Switch to correct tab BEFORE adding
Dash.useTab("Swerve");
Dash.add("Speed", () -> getSpeed());
Dash.useDefaultTab();  // Switch back
```

---

### Tunable Values Not Changing

**Symptom**: Editing value on Shuffleboard has no effect

**Cause**: Not reading the GenericEntry in `periodic()`

**Fix**:
```java
private GenericEntry kP;

public MySubsystem() {
    kP = Dash.addTunable("kP", 0.5);
}

// ✓ CORRECT - Read in periodic()
@Override
public void periodic() {
    double p = kP.getDouble(0.5);  // Read current value
    controller.setP(p);
}

// ✗ WRONG - Never reads the value
@Override
public void periodic() {
    controller.setP(0.5);  // Hardcoded, ignores Shuffleboard
}
```

---

### Performance Issues

**Symptom**: Robot code running slow

**Cause**: Too many dashboard values or expensive Supplier calls

**Fix**:
```java
// Limit number of values (< 50 per tab recommended)
// Cache expensive calculations
private double cachedValue;

@Override
public void periodic() {
    cachedValue = expensiveCalculation();  // Once per loop
}

// Supplier just returns cached value
Dash.add("Value", () -> cachedValue);  // Fast
```

---

## Migration from SmartDashboard

### Before (SmartDashboard)

```java
public class MySubsystem extends SubsystemBase {
    @Override
    public void periodic() {
        SmartDashboard.putNumber("Speed", getSpeed());
        SmartDashboard.putBoolean("At Target", isAtTarget());
        SmartDashboard.putString("State", currentState.toString());
    }
}
```

### After (Dash)

```java
public class MySubsystem extends SubsystemBase {
    public MySubsystem() {
        // Add once in constructor - auto-updates every loop
        Dash.add("Speed", () -> getSpeed());
        Dash.add("At Target", () -> isAtTarget());
        Dash.add("State", () -> currentState.toString());
    }

    // No need for periodic() updates!
}
```

**Benefits:**
- ✓ Cleaner code (no periodic() updates)
- ✓ Auto-updating (handled by Shuffleboard)
- ✓ Better organization (tabs)
- ✓ Type-safe (compile-time checking)

---

## See Also

- [Buttons Documentation](Buttons.md) - Controller input abstraction
- [StateMachine Documentation](StateMachine.md) - State machine utility
- [Utils Documentation](README.md) - Overview of all utilities
- [WPILib Shuffleboard](https://docs.wpilib.org/en/stable/docs/software/dashboards/shuffleboard/index.html)
- [WPILib NetworkTables](https://docs.wpilib.org/en/stable/docs/software/networktables/index.html)

---

**Last Updated:** 2026-02-17
