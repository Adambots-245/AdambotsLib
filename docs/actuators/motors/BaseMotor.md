# BaseMotor API

Complete interface for motor control with unified API across NEO, Falcon 500, Kraken X60, and Minion motors.

## Interface Definition

```java
public interface BaseMotor extends BaseActuator
```

All motor implementations (NEOMotor, TalonFXMotor, MinionMotor) implement this interface, enabling motor interchangeability without code changes.

## Control Modes

```java
public enum ControlMode {
    PERCENT_OUTPUT,           // Open-loop duty cycle control
    POSITION,                 // Closed-loop position control
    VELOCITY,                 // Closed-loop velocity control (RPS)
    VOLTAGE,                  // Direct voltage control
    CURRENT,                  // Torque current control (if supported)
    MOTION_MAGIC,            // Position with motion profiling
    MOTION_MAGIC_FOC_TORQUE, // Motion profiling with FOC (TalonFX only)
    FOLLOWER                 // Follow another motor
}
```

### Control Mode Details

| Mode | Value Interpretation | Units | Supported By |
|------|---------------------|-------|--------------|
| **PERCENT_OUTPUT** | Duty cycle | -1.0 to 1.0 | All motors |
| **POSITION** | Target position | Rotations | All motors |
| **VELOCITY** | Target velocity | RPS (rotations per second) | All motors |
| **VOLTAGE** | Output voltage | Volts (-12.0 to 12.0) | All motors |
| **CURRENT** | Torque current | Amperes | NEO, Kraken only |
| **MOTION_MAGIC** | Target position with profiling | Rotations | All motors |
| **MOTION_MAGIC_FOC_TORQUE** | FOC torque mode | Rotations | TalonFX only |
| **FOLLOWER** | Device ID to follow | CAN ID | All motors |

### Compatibility Matrix

| Control Mode | NEO | TalonFX (Falcon) | TalonFX (Kraken) | Minion |
|-------------|-----|------------------|------------------|--------|
| PERCENT_OUTPUT | ✓ | ✓ | ✓ | ✓ |
| POSITION | ✓ | ✓ | ✓ | ✓ |
| VELOCITY | ✓ | ✓ | ✓ | ✓ |
| VOLTAGE | ✓ | ✓ | ✓ | ✓ |
| CURRENT | ✓ | ⚠ Fallback | ✓ | ⚠ Fallback |
| MOTION_MAGIC | ✓ | ✓ | ✓ | ✓ |
| MOTION_MAGIC_FOC_TORQUE | ⚠ Fallback | ✓ | ✓ | ⚠ Fallback |
| FOLLOWER | ✓ | ✓ | ✓ | ✓ |

✓ = Fully supported | ⚠ = Supported with fallback (logs warning, uses alternate mode)

## Core Methods

### Motor Control

```java
void set(ControlMode mode, double value)
```

**Description:** Sets the motor control mode and output value.

**Parameters:**
- `mode` - Control mode to use (see Control Modes above)
- `value` - Setpoint value (interpretation depends on mode)

**Examples:**
```java
// Open-loop control
motor.set(ControlMode.PERCENT_OUTPUT, 0.5);  // 50% forward

// Position control
motor.set(ControlMode.POSITION, 10.0);  // Move to 10 rotations

// Velocity control (RPS units)
motor.set(ControlMode.VELOCITY, 83.33);  // 83.33 RPS = 5000 RPM

// Motion profiling
motor.set(ControlMode.MOTION_MAGIC, 25.5);  // Smooth move to 25.5 rotations

// Follower mode
motor.set(ControlMode.FOLLOWER, 5);  // Follow motor with CAN ID 5
```

---

```java
void set(double speed)
```

**Description:** Sets motor speed in open-loop mode (shorthand for PERCENT_OUTPUT).

**Parameters:**
- `speed` - Output value from -1.0 (full reverse) to 1.0 (full forward)

**Example:**
```java
motor.set(0.75);  // 75% forward power
```

---

### PID Configuration

```java
void setPID(int slotIdx, double kP, double kI, double kD, double kF)
```

**Description:** Configures PID gains for closed-loop control.

**Parameters:**
- `slotIdx` - PID gain slot index (0-2, varies by motor controller)
- `kP` - Proportional gain (primary error correction)
- `kI` - Integral gain (eliminates steady-state error)
- `kD` - Derivative gain (dampens oscillation)
- `kF` - Feed-forward gain (compensates for system dynamics)

**PID Tuning Guide:**
- **kP:** Start with 0.1-1.0 for position control, 0.0001-0.001 for velocity
- **kI:** Usually 0 or very small (0.0001) to avoid windup
- **kD:** Typically 0.1-10x kP to reduce oscillation
- **kF:** For velocity control, ~0.001-0.01 (or calculate from motor specs)

**Examples:**
```java
// Position control - slot 0
motor.setPID(0, 0.1, 0.0, 0.05, 0.0);

// Velocity control with feed-forward - slot 1
motor.setPID(1, 0.0001, 0, 0.00005, 0.000176);

// Aggressive position control
motor.setPID(0, 0.5, 0.0001, 0.1, 0.0);
```

---

### Extended PID with Feedforward (v2026.2.6+)

```java
default void setPID(int slotIdx, double kP, double kI, double kD,
                    double kV, double kS, double kA, double kG)
```

**Description:** Configures PID and extended feedforward gains for closed-loop control. Supported by TalonFXMotor and MinionMotor. Motors that don't support it (e.g., NEOMotor) fall back to basic setPID using kV as kF.

**Parameters:**
- `slotIdx` - PID gain slot index (0-2)
- `kP` - Proportional gain
- `kI` - Integral gain
- `kD` - Derivative gain
- `kV` - Velocity feedforward (output per unit velocity)
- `kS` - Static feedforward (minimum output to overcome friction)
- `kA` - Acceleration feedforward (output per unit acceleration)
- `kG` - Gravity feedforward (requires `configureGravity()` to set type)

**Examples:**
```java
// Arm with gravity compensation
motor.configureGravity(BaseMotor.GravityType.ARM_COSINE);
motor.setPID(0, 100, 0, 2.0, 0, 0.35, 0, 0.2);

// Velocity control with friction compensation
motor.setPID(0, 0.1, 0, 0.01, 0.12, 0.25, 0.01, 0);
```

---

### Gravity Compensation (v2026.2.6+)

```java
default void configureGravity(GravityType type)
```

**Description:** Configures gravity compensation type for the motor's closed-loop control. Must be used alongside kG set via the extended `setPID` method.

**GravityType values:**
- `NONE` - No gravity compensation (default)
- `ARM_COSINE` - Output = kG × cos(angle). Use for rotating arms. Maximum at horizontal, zero at vertical.
- `ELEVATOR_STATIC` - Constant kG output regardless of position. Use for elevators.

**Tuning kG for arms:**
1. Position the arm horizontally (parallel to ground)
2. In Phoenix Tuner X, slowly increase voltage until the arm just holds steady
3. That voltage = your kG value
4. The controller automatically scales by cos(angle) at all other positions

**Tuning order:** kG first → kS → kP → kD → Motion Magic → kI (almost never needed)

**Examples:**
```java
// Arm with gravity compensation
motor.configureGravity(BaseMotor.GravityType.ARM_COSINE);
motor.setPID(0, 100, 0, 2.0, 0, 0.35, 0, 0.2);

// Elevator with gravity compensation
motor.configureGravity(BaseMotor.GravityType.ELEVATOR_STATIC);
motor.setPID(0, 10, 0, 0.5, 0, 0.1, 0, 0.3);

// Using the builder pattern
motor.configure()
    .gravity(BaseMotor.GravityType.ARM_COSINE)
    .brakeMode(true)
    .apply();
motor.setPID(0, 100, 0, 2.0, 0, 0.35, 0, 0.2);
```

---

### Motion Magic Configuration

```java
void configureMotionMagic(AngularVelocity cruiseVelocity,
                         AngularAcceleration acceleration,
                         double jerkRPSPerSecPerSec)
```

**Description:** Configures Motion Magic trapezoidal motion profiling parameters using WPILib typed units.

**Parameters:**
- `cruiseVelocity` - Maximum velocity during motion (AngularVelocity type)
- `acceleration` - Acceleration rate (AngularAcceleration type)
- `jerkRPSPerSecPerSec` - Jerk limiting (rotations per second³, 0 to disable)

**Parameter Guidance:**
- Set cruise velocity to 70-80% of mechanism's maximum speed
- Acceleration typically 2-4x cruise velocity
- Jerk optional: 0 = disabled, or 5-10x acceleration for smoothing

**Examples:**
```java
import static edu.wpi.first.units.Units.*;

// Elevator motion profile
motor.configureMotionMagic(
    RotationsPerSecond.of(50),           // 50 RPS cruise
    RotationsPerSecondPerSecond.of(150), // 150 RPS² accel
    500.0                                 // 500 RPS³ jerk
);

// Fast arm movement
motor.configureMotionMagic(
    RotationsPerSecond.of(100),          // 100 RPS cruise
    RotationsPerSecondPerSecond.of(400), // 400 RPS² accel
    0.0                                   // no jerk limiting
);

// Smooth drivetrain
motor.configureMotionMagic(
    RotationsPerSecond.of(80),           // 80 RPS cruise
    RotationsPerSecondPerSecond.of(160), // 160 RPS² accel
    800.0                                 // 800 RPS³ jerk
);
```

---

### Current Limiting

```java
void configureCurrentLimits(double stallLimitAmps,
                           double freeLimitAmps,
                           double limitRpmThreshold)
```

**Description:** Configures current limiting to protect motor and battery.

**Parameters:**
- `stallLimitAmps` - Current limit when motor is under heavy load
- `freeLimitAmps` - Current limit when motor is spinning freely
- `limitRpmThreshold` - RPM threshold below which stall limit applies

**Typical Values:**
- **Drive motors:** stall=40-60A, free=80-120A, threshold=5000 RPM
- **Mechanism motors:** stall=20-40A, free=40-80A, threshold=3000 RPM
- **Small actuators:** stall=10-20A, free=20-40A, threshold=2000 RPM

**Examples:**
```java
// NEO on drive motor - conservative limits
motor.configureCurrentLimits(40, 60, 5000);

// TalonFX on shooter - allow higher free-spin current
motor.configureCurrentLimits(60, 120, 3000);

// Small mechanism motor
motor.configureCurrentLimits(20, 40, 2000);
```

---

### Soft Limits

```java
void configureSoftLimits(double forwardLimitRotations,
                        double reverseLimitRotations,
                        boolean enable)
```

**Description:** Configures software (virtual) position limits to prevent mechanism damage.

**Parameters:**
- `forwardLimitRotations` - Forward soft limit position (rotations from zero)
- `reverseLimitRotations` - Reverse soft limit position (rotations from zero)
- `enable` - True to enable, false to disable

**Best Practices:**
- Set limits slightly inside mechanical hard stops
- Use positive values for forward, negative for reverse
- Test carefully during development
- Consider hardware limit switches as backup

**Examples:**
```java
// Elevator with 100 rotation range
motor.configureSoftLimits(98.0, -2.0, true);
// Forward: 98 rotations, Reverse: -2 rotations, Enabled

// Arm with 90-degree range (assuming 100:1 gearbox)
motor.configureSoftLimits(25.0, 0.0, true);
// Forward: 25 rotations, Reverse: 0 (starting position)

// Disable soft limits temporarily
motor.configureSoftLimits(98.0, -2.0, false);
```

---

```java
void enableSoftLimits(boolean enable)
```

**Description:** Enables or disables software limits without reconfiguring values.

**Example:**
```java
// Disable limits for testing
motor.enableSoftLimits(false);

// Re-enable for competition
motor.enableSoftLimits(true);
```

---

### Motor Configuration

```java
void setInverted(boolean inverted)
```

**Description:** Sets motor output direction inversion.

**Example:**
```java
// Invert left side of drivetrain
leftMotor.setInverted(true);
rightMotor.setInverted(false);
```

---

### Motor Direction (v2026.3.3+)

```java
default void setDirection(MotorDirection direction)
```

**Description:** Sets the positive direction of the motor using explicit clockwise/counter-clockwise semantics. CTRE motors (TalonFX, Minion) map directly to Phoenix 6 `InvertedValue`. REV motors (NEO) map `CLOCKWISE_POSITIVE` to inverted=true and `COUNTER_CLOCKWISE_POSITIVE` to inverted=false.

**MotorDirection values:**
- `COUNTER_CLOCKWISE_POSITIVE` - Counter-clockwise rotation is positive output (default for most motors)
- `CLOCKWISE_POSITIVE` - Clockwise rotation is positive output

**Examples:**
```java
// Direct method call
motor.setDirection(BaseMotor.MotorDirection.CLOCKWISE_POSITIVE);

// Using the builder pattern
motor.configure()
    .direction(BaseMotor.MotorDirection.CLOCKWISE_POSITIVE)
    .brakeMode(true)
    .apply();
```

---

```java
void setBrakeMode(boolean brake)
```

**Description:** Configures motor neutral mode behavior.

**Modes:**
- **Brake (true):** Motor actively resists motion, holds position
- **Coast (false):** Motor freewheels when stopped

**Examples:**
```java
driveMotor.setBrakeMode(false);    // Coast for drivetrain
elevatorMotor.setBrakeMode(true);  // Brake to hold position
armMotor.setBrakeMode(true);       // Brake for arm
```

---

```java
void setPosition(double rotations)
```

**Description:** Resets motor position sensor to specified value (does not move motor).

**Examples:**
```java
// Reset elevator to bottom
motor.setPosition(0.0);

// Set arm to calibrated starting position
motor.setPosition(15.5);
```

---

```java
void enableVoltageCompensation(double voltageNominal)
```

**Description:** Enables voltage compensation for consistent performance.

**Benefits:**
- Consistent autonomous routines throughout match
- Predictable PID tuning
- Repeatable robot behavior

**Typical Value:** 12.0 volts

**Example:**
```java
motor.enableVoltageCompensation(12.0);
```

**Note:** For Phoenix 6 motors (TalonFX, Minion), use voltage-based control modes instead (VoltageOut, PositionVoltage, VelocityVoltage, MotionMagicVoltage).

---

### Follower Configuration

```java
void setStrictFollower(int leaderDeviceID)
```

**Description:** Configures motor to follow another motor's output.

**Important:** Follower motor must be same type and on same CAN bus as leader.

**Examples:**
```java
// Multi-motor drivetrain
BaseMotor leftLeader = new NEOMotor(1, false, 40, false);
BaseMotor leftFollower = new NEOMotor(2, false, 40, false);
leftFollower.setStrictFollower(1);  // Follow motor 1

// Multi-motor shooter
BaseMotor shooterLeader = new TalonFXMotor(5, false, 60, true);
BaseMotor shooterFollower = new TalonFXMotor(6, false, 60, true);
shooterFollower.setStrictFollower(5);  // Follow motor 5
```

---

### Hardware Limits

```java
void configureHardLimits(boolean enableForward,
                        boolean enableReverse,
                        double forwardResetValueRotations,
                        double reverseResetValueRotations)
```

**Description:** Configures hardware limit switches for automatic position zeroing.

**Parameters:**
- `enableForward` - Enable forward limit switch
- `enableReverse` - Enable reverse limit switch
- `forwardResetValueRotations` - Position to set when forward limit triggered
- `reverseResetValueRotations` - Position to set when reverse limit triggered

**Example:**
```java
// Elevator that zeros at bottom
motor.configureHardLimits(
    false,  // Don't use forward limit
    true,   // Enable reverse (bottom) limit
    0.0,    // Not used
    0.0     // Reset to 0 when hitting bottom
);
```

---

## Sensor Methods

### Position and Velocity

```java
double getPosition()
```

**Returns:** Current motor position in rotations from last reset point.

---

```java
AngularVelocity getVelocity()
```

**Returns:** Current motor velocity as a WPILib `AngularVelocity` type.

**BREAKING CHANGE (v2026.2.0):** All motors now return typed units instead of raw doubles.

**Usage:**
```java
import static edu.wpi.first.units.Units.*;

AngularVelocity velocity = motor.getVelocity();
double rps = velocity.in(RotationsPerSecond);  // rotations per second
double rpm = velocity.in(RPM);                  // revolutions per minute
```

---

```java
AngularAcceleration getAcceleration()
```

**Returns:** Current motor acceleration as a WPILib `AngularAcceleration` type.

**Note:** NEO motors do not support acceleration measurement (returns zero).

**Usage:**
```java
import static edu.wpi.first.units.Units.*;

AngularAcceleration accel = motor.getAcceleration();
double rpsPerSec = accel.in(RotationsPerSecondPerSecond);
```

---

### Electrical Monitoring

```java
Current getCurrentDraw()
```

**Returns:** Current draw as a WPILib `Current` type.

**Usage:**
```java
import static edu.wpi.first.units.Units.*;

Current current = motor.getCurrentDraw();
double amps = current.in(Amps);
```

**Use Cases:**
- Detect motor stalls
- Monitor for current limit violations
- Identify mechanical binding

---

```java
double getOutputPercent()
```

**Returns:** Current motor output percentage (-1.0 to 1.0).

**Use Cases:**
- Debug control loops
- Verify motor commands
- Monitor actual output vs commanded

---

```java
double getTemperature()
```

**Returns:** Motor controller temperature in degrees Celsius.

**Warning:** Most controllers shut down or reduce output at 80-100°C.

---

### Limit Switches

```java
boolean getForwardLimitSwitch()
```

**Returns:** True if forward hardware limit switch is pressed/triggered.

---

```java
boolean getReverseLimitSwitch()
```

**Returns:** True if reverse hardware limit switch is pressed/triggered.

---

## Capability Detection

```java
boolean supportsControlMode(ControlMode mode)
```

**Description:** Checks if motor supports a specific control mode.

**Returns:** True if mode is fully supported, false otherwise.

**Example:**
```java
// Safe control mode selection
if (motor.supportsControlMode(ControlMode.CURRENT)) {
    motor.set(ControlMode.CURRENT, 10.0);
} else {
    motor.set(ControlMode.PERCENT_OUTPUT, 0.5);
}

// Check before using FOC
if (motor.supportsControlMode(ControlMode.MOTION_MAGIC_FOC_TORQUE)) {
    motor.set(ControlMode.MOTION_MAGIC_FOC_TORQUE, target);
} else {
    motor.set(ControlMode.MOTION_MAGIC, target);
}
```

---

```java
String getMotorType()
```

**Description:** Gets motor type name for logging and debugging.

**Returns:** Motor type string (e.g., "NEOMotor (SPARK MAX)", "TalonFXMotor (Kraken X60)")

**Example:**
```java
System.out.println("Using: " + motor.getMotorType());
// Output: "Using: TalonFXMotor (Kraken X60)"
```

---

## Builder Pattern Configuration

```java
MotorConfigBuilder configure()
```

**Description:** Creates a fluent configuration builder for this motor.

**Benefits:**
- Reads like English - easy to understand
- All parameters have clear names with units
- Optional parameters - only configure what you need
- Type-safe - compiler catches errors
- Self-documenting - IDE shows parameter meanings

**Example:**
```java
// Complete drive motor configuration
motor.configure()
    .pid(0.1, 0.0, 0.05, 0.0)
    .currentLimits(40, 60, 5000)
    .motionMagic(50.0, 100.0, 200.0)
    .inverted(true)
    .brakeMode(true)
    .voltageCompensation(12.0)
    .softLimits(100.0, 0.0, true)
    .apply();

// Minimal configuration
motor.configure()
    .currentLimits(20, 40, 3000)
    .brakeMode(true)
    .apply();
```

---

## Usage Patterns

### Motor-Agnostic Subsystem

```java
public class Shooter {
    private final BaseMotor motor;

    public Shooter(BaseMotor motor) {
        this.motor = motor;  // Works with NEO, TalonFX, or Minion

        // Configure once in constructor
        motor.setPID(0, 0.0001, 0.0, 0.0, 0.00017);
        motor.setBrakeMode(false);
        motor.enableVoltageCompensation(12.0);
    }

    public void setVelocity(double rps) {
        motor.set(ControlMode.VELOCITY, rps);
    }

    public boolean atSpeed(double targetRPS, double tolerance) {
        return Math.abs(motor.getVelocity() - targetRPS) < tolerance;
    }
}
```

### Builder Pattern Configuration

```java
public class Elevator {
    private final BaseMotor elevatorMotor;

    public Elevator(BaseMotor motor) {
        this.elevatorMotor = motor;

        // Fluent configuration
        elevatorMotor.configure()
            .pid(0.1, 0.0, 0.01, 0.0)
            .motionMagic(50.0, 150.0, 500.0)
            .currentLimits(30, 50, 3000)
            .softLimits(100.0, 0.0, true)
            .brakeMode(true)
            .apply();
    }

    public void setHeight(double rotations) {
        elevatorMotor.set(ControlMode.MOTION_MAGIC, rotations);
    }
}
```

### Multi-Motor System with Followers

```java
public class DriveSubsystem {
    private final BaseMotor leftLeader;
    private final BaseMotor leftFollower;
    private final BaseMotor rightLeader;
    private final BaseMotor rightFollower;

    public DriveSubsystem() {
        // Leader motors
        leftLeader = new NEOMotor(1, false, 40, false);
        rightLeader = new NEOMotor(3, false, 40, true);

        // Follower motors
        leftFollower = new NEOMotor(2, false, 40, false);
        rightFollower = new NEOMotor(4, false, 40, true);

        // Configure followers
        leftFollower.setStrictFollower(1);
        rightFollower.setStrictFollower(3);

        // Configure leaders only (followers mirror settings)
        for (BaseMotor leader : List.of(leftLeader, rightLeader)) {
            leader.configure()
                .currentLimits(40, 60, 5000)
                .brakeMode(false)
                .voltageCompensation(12.0)
                .apply();
        }
    }

    public void tankDrive(double left, double right) {
        leftLeader.set(left);
        rightLeader.set(right);
    }
}
```

### Safe Control Mode Handling

```java
public void setTargetCurrent(BaseMotor motor, double amps) {
    if (motor.supportsControlMode(ControlMode.CURRENT)) {
        motor.set(ControlMode.CURRENT, amps);
    } else {
        // Fallback to percent output with warning
        double percentOutput = amps / 40.0;  // Rough approximation
        motor.set(ControlMode.PERCENT_OUTPUT, percentOutput);
        DriverStation.reportWarning(
            motor.getMotorType() + " doesn't support current control", false);
    }
}
```

## Migration Guide

### RPS Units Breaking Change (v2026.2.0)

**What Changed:**
- All motors now use RPS (rotations per second) for velocity
- NEOMotor previously used RPM, now uses RPS
- TalonFX and Minion already used RPS (no change needed)

**Migration:**

```java
// Old (before v2026.2.0)
neoMotor.set(ControlMode.VELOCITY, 5000);  // RPM
double rpm = neoMotor.getVelocity();       // Returns RPM

// New (v2026.2.0+)
neoMotor.set(ControlMode.VELOCITY, 83.33);  // RPS (5000 RPM / 60)
double rps = neoMotor.getVelocity();        // Returns RPS

// Helper conversion
public static final double RPM_TO_RPS = 1.0 / 60.0;
public static final double RPS_TO_RPM = 60.0;

double velocityRPS = velocityRPM * RPM_TO_RPS;
double velocityRPM = velocityRPS * RPS_TO_RPM;
```

## See Also

- **[Motors Overview](README.md)** - Complete guide with examples
- **[NEOMotor](NEOMotor.md)** - REV NEO motor implementation
- **[TalonFXMotor](TalonFXMotor.md)** - Falcon 500 / Kraken X60 implementation
- **[MinionMotor](MinionMotor.md)** - Minion motor implementation
- **[Actuators Overview](../README.md)** - All actuator types
