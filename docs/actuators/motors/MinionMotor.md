# MinionMotor

CTRE Minion motor implementation using TalonFXS controller with Phoenix 6 API. Compact, lightweight motor for space-constrained mechanisms.

## Constructors

```java
public MinionMotor(int deviceID)
```

**Parameters:**
- `deviceID` - CAN ID for the TalonFXS controller (0-62)

**Example:**
```java
// Minion on regular CAN bus
BaseMotor intake = new MinionMotor(10);
```

---

```java
public MinionMotor(int deviceID, String canBus)
```

**Parameters:**
- `deviceID` - CAN ID for the TalonFXS controller (0-62)
- `canBus` - CAN bus name (e.g., "CANivore1", "*" for auto-detect)

**Example:**
```java
// Minion on CANivore
BaseMotor wrist = new MinionMotor(15, "CANivore1");

// Auto-detect CANivore
BaseMotor roller = new MinionMotor(12, "*");
```

## Hardware Specifications

| Specification | Value |
|--------------|-------|
| **Free Speed** | 5800 RPM (96.7 RPS) |
| **Stall Torque** | 1.26 Nm |
| **Stall Current** | 135A |
| **Weight** | 150g (with TalonFXS) |
| **Encoder** | Integrated (2048 CPR) |
| **FOC** | No |
| **Controller** | TalonFXS (compact TalonFX variant) |

## Controller: TalonFXS

- **CAN ID Range:** 0-62
- **CAN Buses:** Regular CAN or CANivore
- **PID Slots:** 1 slot (slot 0)
- **Control Modes:** Most BaseMotor modes (see compatibility below)
- **Motion Profiling:** Motion Magic with jerk control
- **Firmware:** Phoenix 6 API
- **Configuration:** Factory reset on init, automatic retry
- **Initialization:** Applies factory reset (`TalonFXSConfiguration`) to clear any stale configs persisted in flash memory

## Key Features

### Compact and Lightweight

**Weight Comparison:**
- Minion: 150g
- NEO: 453g (3x heavier)
- Falcon 500: 435g (2.9x heavier)
- Kraken X60: 435g (2.9x heavier)

**Best For:**
- Intakes and rollers
- Wrist mechanisms
- Climber winches
- Any space-constrained mechanism
- Mechanisms where weight matters

### Velocity Units: RPS

MinionMotor uses **rotations per second (RPS)** for all velocity operations:

```java
// Set velocity in RPS
motor.set(ControlMode.VELOCITY, 50.0);  // 50 RPS = 3000 RPM

// Get velocity in RPS
double rps = motor.getVelocity();  // Returns RPS

// Conversion
double rpm = rps * 60.0;
double rps = rpm / 60.0;
```

### Control Mode Support

| Mode | Support | Notes |
|------|---------|-------|
| **PERCENT_OUTPUT** | ✓ Full | Duty cycle control |
| **POSITION** | ✓ Full | Closed-loop position with PositionVoltage |
| **VELOCITY** | ✓ Full | Closed-loop velocity with VelocityVoltage (RPS) |
| **VOLTAGE** | ✓ Full | Direct voltage control |
| **CURRENT** | ⚠ Fallback | Falls back to PERCENT_OUTPUT with warning |
| **MOTION_MAGIC** | ✓ Full | Trapezoidal profiling with MotionMagicVoltage |
| **MOTION_MAGIC_FOC_TORQUE** | ⚠ Fallback | Falls back to MOTION_MAGIC with warning |
| **FOLLOWER** | ✓ Full | Follow another TalonFXS or TalonFX |

**Limitations:**
- No FOC (Field-Oriented Control)
- No direct current control (TorqueCurrentFOC)
- Single PID slot (slot 0 only)

### Phoenix 6 Configuration Best Practices

MinionMotor follows Phoenix 6 best practices:

**Factory Reset on Initialization (v2026.2.9+)**

TalonFXS motor controllers persist their configuration in flash memory across power cycles. This can cause unexpected behavior when motor configs from previous code deployments or testing sessions remain active. MinionMotor automatically applies a factory reset on construction before applying any library configurations:

```java
// Handled automatically in the MinionMotor constructor
configurator.apply(new TalonFXSConfiguration(), 0.050);
// All subsequent configuration starts from a clean slate
```

This ensures:
- No stale inversion, brake mode, or PID settings carry over
- Consistent behavior regardless of what was previously deployed
- Eliminates hard-to-debug issues from persistent flash configs

**Automatic Retry Logic**

Configuration failures are automatically retried up to 3 times:

```java
// Automatically handled by MinionMotor
motor.setBrakeMode(true);
// If first attempt fails, retries up to 3 times
// Logs warning to DriverStation if all retries fail
```

**Voltage-Based Control Modes**

MinionMotor uses voltage-based control modes for proper voltage compensation:

```java
// Position control uses PositionVoltage
motor.set(ControlMode.POSITION, 10.0);

// Velocity control uses VelocityVoltage
motor.set(ControlMode.VELOCITY, 50.0);

// Motion Magic uses MotionMagicVoltage
motor.set(ControlMode.MOTION_MAGIC, 25.0);

// Direct voltage control
motor.set(ControlMode.VOLTAGE, 6.0);
```

## Common Patterns

### Intake with Velocity Control

```java
public class Intake {
    private final MinionMotor intakeMotor;

    public Intake() {
        intakeMotor = new MinionMotor(10);

        // Velocity PID
        intakeMotor.setPID(0, 0.05, 0.0, 0.0, 0.01);

        // Coast mode for compliance
        intakeMotor.setBrakeMode(false);

        // Current limiting
        intakeMotor.configureCurrentLimits(30, 50, 3000);
    }

    public void intake() {
        intakeMotor.set(ControlMode.VELOCITY, 80.0);  // 80 RPS
    }

    public void outtake() {
        intakeMotor.set(ControlMode.VELOCITY, -60.0);  // -60 RPS
    }

    public void stop() {
        intakeMotor.set(0.0);
    }

    public boolean hasGamePiece() {
        // Detect by current spike
        return intakeMotor.getCurrentDraw() > 15.0;
    }
}
```

### Wrist with Position Control

```java
public class Wrist {
    private final MinionMotor wristMotor;

    public enum Angle {
        STOWED(0.0),
        INTAKE(15.0),
        SCORE(30.0);

        public final double rotations;
        Angle(double rotations) { this.rotations = rotations; }
    }

    public Wrist() {
        wristMotor = new MinionMotor(12);

        // Position PID
        wristMotor.setPID(0, 0.2, 0.0, 0.02, 0.0);

        // Brake mode to hold position
        wristMotor.setBrakeMode(true);

        // Soft limits for safety
        wristMotor.configureSoftLimits(35.0, -5.0, true);

        // Current limiting for small mechanism
        wristMotor.configureCurrentLimits(20, 35, 2000);

        // Zero at stowed position
        wristMotor.setPosition(0.0);
    }

    public void setAngle(Angle angle) {
        wristMotor.set(ControlMode.POSITION, angle.rotations);
    }

    public double getAngle() {
        return wristMotor.getPosition();
    }

    public boolean atTarget(double tolerance) {
        return Math.abs(wristMotor.getPosition() - getTargetAngle()) < tolerance;
    }
}
```

### Climber with Motion Magic

```java
public class Climber {
    private final MinionMotor climberMotor;

    public Climber() {
        climberMotor = new MinionMotor(15, "CANivore1");

        // Motion Magic configuration (smooth climbing)
        climberMotor.configureMotionMagic(
            30.0,   // 30 RPS cruise velocity
            60.0,   // 60 RPS² acceleration
            120.0   // 120 RPS³ jerk limiting
        );

        // Position PID
        climberMotor.setPID(0, 0.15, 0.0, 0.015, 0.0);

        // Brake mode to hold position
        climberMotor.setBrakeMode(true);

        // Soft limits to prevent over-extension
        climberMotor.configureSoftLimits(150.0, 0.0, true);

        // Higher current limit for climbing
        climberMotor.configureCurrentLimits(35, 60, 2000);

        // Zero at bottom
        climberMotor.setPosition(0.0);
    }

    public void extend() {
        climberMotor.set(ControlMode.MOTION_MAGIC, 150.0);
    }

    public void retract() {
        climberMotor.set(ControlMode.MOTION_MAGIC, 0.0);
    }

    public boolean isExtended() {
        return climberMotor.getPosition() > 140.0;
    }

    public boolean isRetracted() {
        return climberMotor.getPosition() < 10.0;
    }
}
```

### Simple Roller with Open-Loop Control

```java
public class Roller {
    private final MinionMotor rollerMotor;

    public Roller() {
        rollerMotor = new MinionMotor(20);

        // Coast mode for roller
        rollerMotor.setBrakeMode(false);

        // Low current limit
        rollerMotor.configureCurrentLimits(15, 30, 2000);
    }

    public void spin(double speed) {
        // Simple open-loop control
        rollerMotor.set(speed);
    }

    public void intake() {
        spin(0.8);
    }

    public void outtake() {
        spin(-0.6);
    }

    public void stop() {
        spin(0.0);
    }
}
```

### Multi-Minion System with Follower

```java
public class DoubleIntake {
    private final MinionMotor leader;
    private final MinionMotor follower;

    public DoubleIntake() {
        // Leader motor
        leader = new MinionMotor(10);
        leader.configure()
            .currentLimits(25, 40, 3000)
            .brakeMode(false)
            .apply();

        // Follower motor (mirrors leader)
        follower = new MinionMotor(11);
        follower.setStrictFollower(10);  // Follow CAN ID 10

        // Note: Only configure leader, follower mirrors
    }

    public void intake() {
        leader.set(0.8);  // Follower automatically matches
    }

    public void outtake() {
        leader.set(-0.5);
    }

    public double getTotalCurrent() {
        return leader.getCurrentDraw() + follower.getCurrentDraw();
    }
}
```

## Configuration Examples

### Builder Pattern (Recommended)

```java
// Complete configuration
motor.configure()
    .pid(0.1, 0.0, 0.01, 0.0)
    .motionMagic(30.0, 60.0, 120.0)
    .currentLimits(20, 40, 2000)
    .softLimits(50.0, 0.0, true)
    .inverted(true)
    .brakeMode(true)
    .apply();

// Minimal configuration for simple mechanisms
motor.configure()
    .currentLimits(15, 30, 2000)
    .brakeMode(false)
    .apply();
```

### Traditional Method Calls

```java
// Position control configuration
motor.setPID(0, 0.2, 0.0, 0.02, 0.0);
motor.configureSoftLimits(35.0, -5.0, true);
motor.setBrakeMode(true);
motor.configureCurrentLimits(20, 35, 2000);

// Velocity control configuration
motor.setPID(0, 0.05, 0.0, 0.0, 0.01);
motor.setBrakeMode(false);
motor.configureCurrentLimits(25, 45, 3000);
```

## PID Tuning Guide

### Starting Values by Application

**Position Control (Wrist, Small Arm):**
```java
motor.setPID(0, 0.2, 0.0, 0.02, 0.0);
// kP: 0.2 - Higher than heavy mechanisms
// kI: 0.0 - Usually not needed
// kD: 0.02 - Dampen oscillation
// kV: 0.0 - Not used for position
```

**Velocity Control (Intake, Roller):**
```java
motor.setPID(0, 0.05, 0.0, 0.0, 0.01);
// kP: 0.05 - Adjust for tracking
// kI: 0.0 - Avoid windup
// kD: 0.0 - Usually not needed
// kV: 0.01 - Feed-forward gain
```

**kV Calculation for Velocity:**
```java
// Minion: 5800 RPM = 96.7 RPS free speed
double maxVelocityRPS = 96.7;
double kV = 1.0 / maxVelocityRPS;  // ≈ 0.0103

// Adjusted for efficiency (80%)
double kV_adjusted = kV * 0.8;  // ≈ 0.0129
```

### Tuning Process for Lightweight Mechanisms

1. **Start with higher kP** (lightweight = more responsive)
2. **Add kD early** (small inertia = prone to oscillation)
3. **Keep kI minimal** (avoid windup on light mechanisms)
4. **Test current limits** (ensure you're not hitting limits)

## Extended Feedforward and Gravity Compensation (v2026.2.6+)

MinionMotor supports the extended `setPID` with kS, kA, and kG feedforward gains, as well as `configureGravity()` for arm/elevator gravity compensation.

### Arm with Gravity Compensation

```java
// Configure gravity type FIRST, then set PID with kG
motor.configureGravity(BaseMotor.GravityType.ARM_COSINE);
motor.setPID(0,
    100,   // kP - proportional
    0,     // kI - integral
    2.0,   // kD - derivative
    0,     // kV - velocity feedforward
    0.35,  // kS - static friction
    0,     // kA - acceleration feedforward
    0.2    // kG - gravity feedforward
);

motor.configureMotionMagic(
    RotationsPerSecond.of(2.0),
    RotationsPerSecondPerSecond.of(1.0),
    0
);
motor.setBrakeMode(true);
```

### Elevator with Gravity Compensation

```java
motor.configureGravity(BaseMotor.GravityType.ELEVATOR_STATIC);
motor.setPID(0, 10, 0, 0.5, 0, 0.1, 0, 0.3);
```

### Tuning kG for Arms

1. Position the arm horizontally (parallel to ground)
2. In Phoenix Tuner X, slowly increase voltage until the arm just holds steady
3. That voltage = your kG value
4. The controller automatically scales by cos(angle) at all other positions

**Tuning order:** kG → kS → kP → kD → Motion Magic → kI (almost never needed with proper kG)

---

## Voltage Compensation (Phoenix 6)

**Important:** Phoenix 6 voltage compensation works differently than Phoenix 5.

```java
// WRONG - enableVoltageCompensation() doesn't work in Phoenix 6
motor.enableVoltageCompensation(12.0);  // ❌ Logs warning, no effect

// CORRECT - MinionMotor automatically uses voltage-based control
motor.set(ControlMode.VELOCITY, 50.0);  // ✓ Uses VelocityVoltage
motor.set(ControlMode.POSITION, 10.0);  // ✓ Uses PositionVoltage
motor.set(ControlMode.MOTION_MAGIC, 25.0);  // ✓ Uses MotionMagicVoltage
motor.set(ControlMode.VOLTAGE, 6.0);  // ✓ Uses VoltageOut

// Only PERCENT_OUTPUT doesn't compensate
motor.set(ControlMode.PERCENT_OUTPUT, 0.5);  // Uses DutyCycleOut (no compensation)
```

## Current Limiting Recommendations

| Application | Stall Limit | Free Limit | Threshold RPM |
|------------|-------------|------------|---------------|
| **Intake/Roller** | 20-30A | 40-50A | 3000 RPM |
| **Wrist/Small Arm** | 15-25A | 30-45A | 2000 RPM |
| **Climber Winch** | 30-40A | 50-70A | 2000 RPM |
| **Feeder/Indexer** | 15-20A | 25-40A | 3000 RPM |

**Note:** Minion has lower stall torque than Falcon/Kraken, so current limits should be lower to prevent overheating.

## Troubleshooting

### Configuration Failures

**Symptom:** DriverStation warnings about config failures

**Solution:** Automatic retry handles this
```java
// MinionMotor automatically retries up to 3 times
motor.setBrakeMode(true);
// Check DriverStation for warnings if all retries fail
```

### Motor doesn't move

```java
// Verify motor responds
System.out.println(motor.getMotorType());  // Should print "MinionMotor (TalonFXS)"

// Check current draw
System.out.println("Current: " + motor.getCurrentDraw() + "A");

// Test with open-loop
motor.set(0.25);

// Check limits
motor.enableSoftLimits(false);  // Temporarily disable for testing
```

### Insufficient torque

**Minion has lower torque than Falcon/Kraken:**
- Check gearing ratio (may need higher reduction)
- Verify current limits aren't too restrictive
- Consider using Falcon/Kraken if more torque needed

```java
// Check if hitting current limit
double current = motor.getCurrentDraw();
if (current > 25.0) {
    System.out.println("Hitting current limit!");
    // Increase limit or use different motor
}
```

### Velocity inconsistency

```java
// Ensure using velocity voltage mode (automatic)
motor.set(ControlMode.VELOCITY, 50.0);

// Tune feed-forward
motor.setPID(0, 0.05, 0.0, 0.0, 0.012);

// Increase current limits
motor.configureCurrentLimits(30, 50, 3000);
```

## Comparison with Other Motors

| Feature | Minion | NEO | Falcon 500 | Kraken X60 |
|---------|--------|-----|------------|------------|
| **Free Speed** | 5800 RPM | 5880 RPM | 6380 RPM | 6000 RPM |
| **Stall Torque** | 1.26 Nm | 3.36 Nm | 4.69 Nm | 7.09 Nm |
| **Weight** | 150g | 453g | 435g | 435g |
| **Size** | Compact | Standard | Standard | Standard |
| **Current Control** | ✗ No | ✓ Yes | ✗ No | ✓ Yes |
| **FOC** | ✗ No | ✗ No | ✓ Yes | ✓ Yes |
| **API** | Phoenix 6 | REVLib | Phoenix 6 | Phoenix 6 |
| **Best For** | Lightweight mechanisms | General purpose | High torque | Maximum torque |

### When to Use Minion

**Advantages:**
- Lightest option (150g vs 435-453g)
- Most compact footprint
- Lower cost than Falcon/Kraken
- Phoenix 6 ecosystem
- Integrated encoder

**Best Applications:**
- Intakes and rollers
- Wrist mechanisms
- Small arms
- Climber winches
- Any space-constrained mechanism
- Weight-critical designs

### When NOT to Use Minion

**Limitations:**
- Low torque (1.26 Nm)
- No FOC
- No current control
- Single PID slot

**Use Alternatives If:**
- Need high torque → Use Falcon 500 or Kraken
- Need current control → Use NEO or Kraken
- Need FOC efficiency → Use Falcon or Kraken
- Need multiple PID slots → Use NEO or Falcon

## Migration from TalonFX

```java
// TalonFX (Falcon/Kraken)
TalonFXMotor falcon = new TalonFXMotor(1, false, 40, false);
falcon.enableFOC();
falcon.set(ControlMode.VELOCITY, 50.0);

// MinionMotor (same API, different hardware)
MinionMotor minion = new MinionMotor(1);
// No FOC available
minion.set(ControlMode.VELOCITY, 50.0);

// Key Differences:
// - No FOC on Minion
// - No current control on Minion
// - Single PID slot on Minion
// - Otherwise API is identical
```

## See Also

- **[BaseMotor API](BaseMotor.md)** - Complete interface documentation
- **[Motors Overview](README.md)** - All motor types and patterns
- **[NEOMotor](NEOMotor.md)** - REV NEO alternative
- **[TalonFXMotor](TalonFXMotor.md)** - Higher torque option
- **[Actuators Overview](../README.md)** - All actuator types
