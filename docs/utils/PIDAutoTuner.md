# PIDAutoTuner - Automatic PID Tuning Utility

Automatically tune PID gains using the Relay Feedback Method (Ziegler-Nichols) and calculate feedforward values using step response analysis.

---

## Table of Contents

- [Overview](#overview)
- [Quick Start](#quick-start)
- [Position Tuning](#position-tuning)
- [Velocity Tuning](#velocity-tuning)
- [Builder Pattern](#builder-pattern)
- [Getting Results](#getting-results)
- [How It Works](#how-it-works)
- [Safety Features](#safety-features)
- [Dashboard Integration](#dashboard-integration)
- [Complete Examples](#complete-examples)
- [API Reference](#api-reference)
- [Best Practices](#best-practices)
- [Troubleshooting](#troubleshooting)

---

## Overview

`PIDAutoTuner` provides automatic PID tuning for any motor or subsystem. Instead of manually adjusting kP, kI, and kD values through trial and error, this utility automatically determines optimal gains by inducing controlled oscillations and analyzing the system response.

### Key Features

- **Simple API**: Just pass method references - `motor::getPosition` and `motor::setPercentOutput`
- **Relay Feedback Method**: Classic Ziegler-Nichols algorithm for reliable PID tuning
- **Feedforward Calculation**: Automatically calculates kV and kS for velocity control
- **Safety Built-In**: Range limits, timeouts, and automatic motor stop on abort
- **Dashboard Integration**: Results displayed on Shuffleboard for easy copying
- **Command-Based**: Returns WPILib Command for easy button binding

### When to Use

**Use PIDAutoTuner when:**
- Tuning a new mechanism (turret, arm, elevator, shooter)
- Starting point for PID gains is unknown
- Manual tuning is taking too long
- You want reproducible tuning results

**Don't use when:**
- Mechanism has dangerous failure modes (use manual tuning with small increments)
- System has significant nonlinearities (may need manual adjustment)
- You need very precise tuning (use as starting point, then fine-tune manually)

---

## Quick Start

### 1. Create Tuning Command

```java
// In RobotContainer or subsystem
Command tuneCmd = PIDAutoTuner.tunePosition(
    "Turret",                    // name for dashboard
    turret::getAngle,            // measurement method
    turret::setPercentOutput,    // output method
    -180, 180                    // safe range (degrees)
);
```

### 2. Bind to Button

```java
// Bind to controller button for easy triggering
Buttons.XboxAButton.onTrue(tuneCmd);
```

### 3. Run Tuning

1. Enable robot in teleop
2. Press the bound button
3. Watch the mechanism oscillate (3-10 cycles)
4. Results printed to console and displayed on dashboard

### 4. Copy Results

```java
// After tuning, get results
TuningResult result = PIDAutoTuner.getLastResult("Turret");

// Print code-ready string
System.out.println(result.toCodeString());
// Output: kP = 0.05, kI = 0.001, kD = 0.02, kF = 0.0

// Or apply directly to motor
result.applyTo(turretMotor, 0);
```

---

## Position Tuning

Use `tunePosition()` for mechanisms that need to reach and hold a position (turrets, arms, elevators).

### Basic Usage

```java
// Turret position tuning
Command tuneCmd = PIDAutoTuner.tunePosition(
    "Turret",
    turret::getAngle,           // Returns current position (degrees)
    turret::setPercentOutput,   // Accepts -1.0 to 1.0
    -180, 180                   // Safe rotation range
);
```

### With BaseMotor

```java
// Using BaseMotor interface
BaseMotor elevatorMotor = new TalonFXMotor(10, true, 40, false);

Command tuneCmd = PIDAutoTuner.tunePosition(
    "Elevator",
    elevatorMotor::getPosition,
    speed -> elevatorMotor.set(ControlMode.PERCENT_OUTPUT, speed),
    0, 50                       // 0 to 50 rotations
);
```

### Parameters

| Parameter | Description |
|-----------|-------------|
| `name` | Identifier for dashboard and results cache |
| `measureFunc` | `DoubleSupplier` returning current position |
| `outputFunc` | `DoubleConsumer` accepting output (-1 to 1) |
| `minRange` | Minimum safe position value |
| `maxRange` | Maximum safe position value |

---

## Velocity Tuning

Use `tuneVelocity()` for mechanisms that need to maintain a velocity (shooters, flywheels, conveyor belts). This also calculates feedforward values (kV, kS).

### Basic Usage

```java
// Shooter velocity tuning
Command tuneCmd = PIDAutoTuner.tuneVelocity(
    "Shooter",
    shooter::getVelocityRPS,    // Returns current velocity (RPS)
    shooter::setVoltage,        // Accepts voltage (-12 to 12)
    0, 100                      // Velocity range (RPS)
);
```

### With BaseMotor

```java
BaseMotor shooterMotor = new TalonFXMotor(15, true, 60, true);

Command tuneCmd = PIDAutoTuner.tuneVelocity(
    "Shooter",
    () -> shooterMotor.getVelocity().in(RotationsPerSecond),
    voltage -> shooterMotor.set(ControlMode.VOLTAGE, voltage),
    0, 100
);
```

### Feedforward Output

Velocity tuning also calculates:
- **kV**: Velocity feedforward (voltage per unit velocity)
- **kS**: Static friction compensation (voltage to overcome friction)

```java
TuningResult result = PIDAutoTuner.getLastResult("Shooter");
System.out.println(result.toCodeString());
// Output:
// kP = 0.100000, kI = 0.010000, kD = 0.025000, kF = 0.000000
// kV = 0.120000, kS = 0.250000, kA = 0.000000, kG = 0.000000
```

---

## Builder Pattern

For advanced configuration, use the builder pattern:

```java
Command tuneCmd = PIDAutoTuner.create("Elevator")
    .measureWith(elevator::getPosition)
    .controlWith(elevator::setPercentOutput)
    .range(0, 50)                    // Safe position range
    .maxOutput(0.3)                  // Limit oscillation to 30%
    .timeout(15.0)                   // 15 second timeout per phase
    .withFeedforward(elevator::setVoltage)  // Enable FF calculation
    .buildCommand();
```

### Builder Methods

| Method | Description | Default |
|--------|-------------|---------|
| `measureWith(DoubleSupplier)` | Set measurement function | Required |
| `controlWith(DoubleConsumer)` | Set output function | Required |
| `range(min, max)` | Set safe operating range | -Infinity to Infinity |
| `maxOutput(double)` | Max output during oscillation | 0.3 (30%) |
| `timeout(seconds)` | Timeout per tuning phase | 10.0 seconds |
| `velocityMode()` | Enable velocity tuning mode | Position mode |
| `withFeedforward(DoubleConsumer)` | Enable FF calculation with voltage output | Disabled |

---

## Getting Results

### From Cache

```java
// Get the last result for a named tuning session
TuningResult result = PIDAutoTuner.getLastResult("Turret");

if (result != null) {
    System.out.println("kP: " + result.kP());
    System.out.println("kI: " + result.kI());
    System.out.println("kD: " + result.kD());
}
```

### Apply to Motor

```java
// Apply to any BaseMotor
result.applyTo(turretMotor, 0);  // Applies kP, kI, kD, kF to slot 0

// Apply extended values to TalonFXMotor (includes kV, kS, kA, kG)
result.applyExtendedTo((TalonFXMotor) shooterMotor, 0);
```

### Convert to PIDConfig

```java
// Create PIDConfig record for configuration
PIDConfig config = result.toPIDConfig(0);
```

### Get Code String

```java
// For copying into Constants file
System.out.println(result.toCodeString());
// Output: kP = 0.050000, kI = 0.001000, kD = 0.020000, kF = 0.000000

// Or as Java constants
System.out.println(result.toConstantsString());
// Output:
// public static final double kP = 0.050000;
// public static final double kI = 0.001000;
// public static final double kD = 0.020000;
// public static final double kF = 0.000000;
```

---

## How It Works

### Relay Feedback Method (Ziegler-Nichols)

1. **Move to Setpoint**: The tuner first moves the mechanism to the midpoint of the safe range
2. **Induce Oscillation**: Bang-bang control (full positive/negative output) creates sustained oscillation
3. **Measure Response**: The tuner records the oscillation period (Tu) and amplitude
4. **Calculate Ku**: Ultimate gain is calculated from the relay amplitude and oscillation amplitude
5. **Apply Formulas**: Classic Ziegler-Nichols formulas calculate PID gains:
   - kP = 0.6 × Ku
   - kI = 1.2 × Ku / Tu
   - kD = 0.075 × Ku × Tu

### Step Response (for Feedforward)

1. **Ramp Voltage**: Slowly increase voltage until mechanism moves (finds kS)
2. **Apply Step**: Apply constant voltage and wait for steady state
3. **Measure**: Record steady-state velocity
4. **Calculate kV**: kV = (Voltage - kS) / Velocity

---

## Safety Features

### Range Limits

The tuner monitors the measurement value and stops immediately if it exceeds the safe range:

```java
.range(0, 50)  // Stops if position goes below 0 or above 50
```

### Output Limiting

The maximum output during oscillation is limited (default 30%):

```java
.maxOutput(0.3)  // Never exceeds 30% output
```

### Timeouts

Each tuning phase has a timeout (default 10 seconds):

```java
.timeout(15.0)  // 15 second timeout per phase
```

### Automatic Stop

The motor is automatically stopped when:
- Tuning completes
- Timeout is reached
- Safety limit is exceeded
- Command is interrupted/cancelled

---

## Dashboard Integration

### Automatic Dashboard Tab

When tuning runs, a "PID Tuning" tab is created in Shuffleboard with:
- Tunable kP, kI, kD, kF values
- Extended values (kV, kS, kA, kG) if feedforward is calculated
- Status display

### Reading from Dashboard

After tuning, you can read the values from the dashboard or from the results cache. The dashboard values are editable, allowing you to fine-tune manually.

---

## Complete Examples

### Example 1: Turret Position Tuning

```java
package frc.robot.subsystems;

import com.adambots.lib.actuators.TalonFXMotor;
import com.adambots.lib.actuators.BaseMotor.ControlMode;
import com.adambots.lib.utils.tuning.PIDAutoTuner;
import com.adambots.lib.utils.tuning.TuningResult;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TurretSubsystem extends SubsystemBase {
    private final TalonFXMotor turretMotor;

    public TurretSubsystem() {
        turretMotor = new TalonFXMotor(20, true, 30, false);
        turretMotor.setBrakeMode(true);
    }

    public double getAngle() {
        return turretMotor.getPosition() * 360.0 / 50.0; // 50:1 gear ratio
    }

    public void setPercentOutput(double output) {
        turretMotor.set(ControlMode.PERCENT_OUTPUT, output);
    }

    public Command createTuningCommand() {
        return PIDAutoTuner.tunePosition(
            "Turret",
            this::getAngle,
            this::setPercentOutput,
            -180, 180  // Safe rotation range
        );
    }

    public void applyTunedValues() {
        TuningResult result = PIDAutoTuner.getLastResult("Turret");
        if (result != null) {
            result.applyTo(turretMotor, 0);
            System.out.println("Applied tuned values: " + result.toCodeString());
        }
    }
}
```

### Example 2: Shooter Velocity Tuning

```java
package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;
import com.adambots.lib.actuators.TalonFXMotor;
import com.adambots.lib.actuators.BaseMotor.ControlMode;
import com.adambots.lib.utils.tuning.PIDAutoTuner;
import com.adambots.lib.utils.tuning.TuningResult;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
    private final TalonFXMotor leftShooter;
    private final TalonFXMotor rightShooter;

    public ShooterSubsystem() {
        leftShooter = new TalonFXMotor(15, true, 60, true);
        rightShooter = new TalonFXMotor(16, true, 60, true);

        leftShooter.setBrakeMode(false);  // Coast for flywheel
        rightShooter.setBrakeMode(false);
        rightShooter.setStrictFollower(15, true);  // Follow and oppose
    }

    public double getVelocityRPS() {
        return leftShooter.getVelocity().in(RotationsPerSecond);
    }

    public void setVoltage(double voltage) {
        leftShooter.set(ControlMode.VOLTAGE, voltage);
    }

    public Command createTuningCommand() {
        return PIDAutoTuner.tuneVelocity(
            "Shooter",
            this::getVelocityRPS,
            this::setVoltage,
            0, 100  // 0-100 RPS range
        );
    }

    public void applyTunedValues() {
        TuningResult result = PIDAutoTuner.getLastResult("Shooter");
        if (result != null) {
            // Apply extended values for Phoenix 6
            result.applyExtendedTo(leftShooter, 0);
            System.out.println("Applied tuned values:\n" + result.toConstantsString());
        }
    }
}
```

### Example 3: Elevator with Custom Settings

```java
package frc.robot.subsystems;

import com.adambots.lib.actuators.TalonFXMotor;
import com.adambots.lib.actuators.BaseMotor.ControlMode;
import com.adambots.lib.utils.tuning.PIDAutoTuner;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {
    private final TalonFXMotor elevatorMotor;

    public ElevatorSubsystem() {
        elevatorMotor = new TalonFXMotor(25, true, 40, false);
        elevatorMotor.setBrakeMode(true);
        elevatorMotor.configureSoftLimits(48, -1, true);
    }

    public double getPosition() {
        return elevatorMotor.getPosition();
    }

    public void setPercentOutput(double output) {
        elevatorMotor.set(ControlMode.PERCENT_OUTPUT, output);
    }

    public void setVoltage(double voltage) {
        elevatorMotor.set(ControlMode.VOLTAGE, voltage);
    }

    public Command createTuningCommand() {
        return PIDAutoTuner.create("Elevator")
            .measureWith(this::getPosition)
            .controlWith(this::setPercentOutput)
            .range(5, 45)              // Stay away from limits
            .maxOutput(0.25)           // Conservative for heavy mechanism
            .timeout(12.0)             // Extra time for slow response
            .withFeedforward(this::setVoltage)  // Calculate kG for gravity
            .buildCommand();
    }
}
```

### Example 4: RobotContainer Integration

```java
package frc.robot;

import com.adambots.lib.utils.Buttons;
import com.adambots.lib.utils.tuning.PIDAutoTuner;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.*;

public class RobotContainer {
    private final TurretSubsystem turret = new TurretSubsystem();
    private final ShooterSubsystem shooter = new ShooterSubsystem();
    private final ElevatorSubsystem elevator = new ElevatorSubsystem();

    public RobotContainer() {
        configureTuningButtons();
        configureOperatorControls();
    }

    private void configureTuningButtons() {
        // Tune turret when A is pressed
        Buttons.XboxAButton.onTrue(turret.createTuningCommand());

        // Tune shooter when B is pressed
        Buttons.XboxBButton.onTrue(shooter.createTuningCommand());

        // Tune elevator when X is pressed
        Buttons.XboxXButton.onTrue(elevator.createTuningCommand());

        // Apply all tuned values when Y is pressed
        Buttons.XboxYButton.onTrue(Commands.runOnce(() -> {
            turret.applyTunedValues();
            shooter.applyTunedValues();
        }));

        // Print all results to console
        Buttons.XboxStartButton.onTrue(Commands.runOnce(() -> {
            System.out.println("\n=== TUNING RESULTS ===");

            var turretResult = PIDAutoTuner.getLastResult("Turret");
            if (turretResult != null) {
                System.out.println("\n// Turret PID");
                System.out.println(turretResult.toConstantsString());
            }

            var shooterResult = PIDAutoTuner.getLastResult("Shooter");
            if (shooterResult != null) {
                System.out.println("\n// Shooter PID");
                System.out.println(shooterResult.toConstantsString());
            }

            System.out.println("\n======================\n");
        }));
    }

    private void configureOperatorControls() {
        // Normal operator controls...
    }
}
```

---

## API Reference

### PIDAutoTuner

#### Static Methods

```java
static Command tunePosition(String name, DoubleSupplier measureFunc,
                           DoubleConsumer outputFunc, double minRange, double maxRange)
```
Creates a position tuning command.

---

```java
static Command tuneVelocity(String name, DoubleSupplier measureFunc,
                           DoubleConsumer voltageOutputFunc, double minVelocity, double maxVelocity)
```
Creates a velocity tuning command with feedforward calculation.

---

```java
static TuningResult getLastResult(String name)
```
Gets the last tuning result for a given name. Returns null if not yet tuned.

---

```java
static TunerBuilder create(String name)
```
Creates a new builder for advanced configuration.

---

### TuningResult

#### Factory Methods

```java
static TuningResult pid(double kP, double kI, double kD)
```
Creates result with PID values only.

---

```java
static TuningResult pidWithFF(double kP, double kI, double kD, double kF)
```
Creates result with PID and simple feedforward.

---

```java
static TuningResult extended(double kP, double kI, double kD,
                            double kV, double kS, double kA, double kG)
```
Creates result with extended feedforward values.

---

#### Accessor Methods

```java
double kP()   // Proportional gain
double kI()   // Integral gain
double kD()   // Derivative gain
double kF()   // Simple feedforward
double kV()   // Velocity feedforward
double kS()   // Static friction feedforward
double kA()   // Acceleration feedforward
double kG()   // Gravity feedforward
```

---

#### Apply Methods

```java
void applyTo(BaseMotor motor, int slot)
```
Applies kP, kI, kD, kF to any BaseMotor.

---

```java
void applyExtendedTo(TalonFXMotor motor, int slot)
```
Applies all values including kV, kS, kA, kG to TalonFXMotor.

---

#### Conversion Methods

```java
PIDConfig toPIDConfig(int slot)
```
Converts to PIDConfig record.

---

```java
String toCodeString()
```
Returns code-friendly string for copying.

---

```java
String toConstantsString()
```
Returns Java constant declarations.

---

```java
boolean hasExtendedFF()
```
Returns true if any extended feedforward value is non-zero.

---

## Best Practices

### **DO**

- **Start with conservative settings**: Use lower `maxOutput()` for heavy or dangerous mechanisms
- **Ensure safe range**: Set range limits that keep mechanism away from hard stops
- **Test incrementally**: Tune one mechanism at a time
- **Log results**: Print and save tuning results for reference
- **Fine-tune manually**: Use auto-tuned values as starting point, adjust as needed
- **Test at competition weight**: Tune with actual game pieces loaded

### **DON'T**

- **Don't tune dangerous mechanisms unattended**: Always have e-stop ready
- **Don't use on mechanisms with backlash**: Results will be inaccurate
- **Don't tune at extreme positions**: Stay in middle of travel range
- **Don't expect perfect results**: May need 10-20% manual adjustment
- **Don't forget to apply results**: Call `applyTo()` or copy values manually

---

## Troubleshooting

### Insufficient Oscillations Detected

**Symptom**: Error message about insufficient oscillations

**Causes**:
- Range too restrictive
- `maxOutput` too low
- Mechanism has high friction
- Timeout too short

**Fix**:
```java
PIDAutoTuner.create("Mechanism")
    .maxOutput(0.5)     // Increase output
    .timeout(15.0)      // Longer timeout
    .range(-200, 200)   // Wider range
    // ...
```

---

### Motor Doesn't Move

**Symptom**: No oscillation occurs

**Causes**:
- Output function not working
- Motor not enabled
- Brake mode holding position

**Fix**:
- Verify motor works with direct commands
- Check that robot is enabled
- Ensure output function accepts values correctly

---

### Results Are Zero

**Symptom**: kP, kI, kD all zero

**Cause**: Tuning failed or was aborted

**Fix**:
- Check console for error messages
- Ensure range doesn't cause immediate abort
- Verify mechanism can actually oscillate

---

### Tuning Seems Wrong

**Symptom**: Gains cause instability when applied

**Causes**:
- Measurement and output not at same point in mechanism
- Significant delay in measurement
- Nonlinear system behavior

**Fix**:
- Use gains as starting point, reduce by 20-50%
- Manually tune kD higher if oscillating
- Consider mechanism-specific tuning approach

---

## See Also

- [Dash Documentation](Dash.md) - Dashboard utilities for displaying results
- [BaseMotor Documentation](../actuators/BaseMotor.md) - Motor interface reference
- [TalonFXMotor Documentation](../actuators/TalonFXMotor.md) - Extended PID configuration
- [WPILib PID Control](https://docs.wpilib.org/en/stable/docs/software/advanced-controls/controllers/pidcontroller.html)

---

**Last Updated:** 2026-02-03
