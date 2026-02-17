# NEOMotor

REV NEO and NEO Vortex motor implementation using SPARK MAX controller with REVLib.

## Constructors

```java
public NEOMotor(int portNum, boolean brushed, int supplyCurrentLimit, boolean inverted)
```

**Parameters:**
- `portNum` - CAN ID for the SPARK MAX controller (1-62)
- `brushed` - True for brushed motors, false for brushless (NEO/NEO Vortex are brushless)
- `supplyCurrentLimit` - Supply current limit in amperes (CRITICAL for NEO safety)
- `inverted` - True to invert motor direction, false for normal

**Example:**
```java
// NEO motor with 40A current limit
BaseMotor shooter = new NEOMotor(5, false, 40, false);

// Inverted NEO for left side of drivetrain
BaseMotor leftDrive = new NEOMotor(1, false, 60, true);

// NEO 550 with lower current limit
BaseMotor intake = new NEOMotor(10, false, 20, false);
```

---

```java
@Deprecated
public NEOMotor(int portNum, boolean brushed)
```

**Deprecated:** Use the 4-parameter constructor to explicitly specify current limits and inversion.

**Default Behavior:** 40A current limit, not inverted.

## Hardware Specifications

| Specification | NEO | NEO Vortex | NEO 550 |
|--------------|-----|------------|---------|
| **Free Speed** | 5880 RPM | 6784 RPM | 11000 RPM |
| **Stall Torque** | 3.36 Nm | 3.60 Nm | 0.97 Nm |
| **Stall Current** | 166A | 211A | 111A |
| **Weight** | 453g | 388g | 218g |
| **Encoder** | Integrated hall-effect | Integrated hall-effect | Integrated hall-effect |
| **Resolution** | 42 counts/rev | 42 counts/rev | 42 counts/rev |
| **Controller** | SPARK MAX | SPARK MAX / SPARK Flex | SPARK MAX |

## Controller: SPARK MAX

- **CAN ID Range:** 1-62
- **Current Limiting:** Smart current limiting with stall/free limits
- **PID Slots:** 4 slots (0-3)
- **Control Modes:** All BaseMotor modes except MOTION_MAGIC_FOC_TORQUE
- **Motion Profiling:** MAXMotion (REV's Motion Magic equivalent)
- **Encoder:** Built-in relative encoder support
- **Firmware:** REVLib API (2025+)

## Key Features

### Simulation Support

NEOMotor exposes simulation methods using REV's `SparkSim`. The sim object is created on construction when running in simulation.

**Key Difference from CTRE:** REV's `SparkSim` uses `iterate(velocityRPM, busVoltage, dt)` as the primary update mechanism rather than setting position/velocity directly. `setSimVelocity()` drives `iterate()` with the given velocity, and `setSimPosition()` calls `iterate()` with zero velocity. This structural difference means NEO sim works best for velocity-driven simulations.

```java
@Override
public void simulationPeriodic() {
    motor.setSimSupplyVoltage(RobotController.getBatteryVoltage());

    sim.setInputVoltage(motor.getSimMotorVoltage());
    sim.update(0.020);

    motor.setSimVelocity(sim.getVelocityRadPerSec() / (2 * Math.PI));
}
```

### CRITICAL: Current Limiting

NEO motors have low internal resistance and can draw very high currents. **Always configure current limits to prevent:**
- Motor overheating and damage
- Battery brownouts during matches
- Breaker trips

**Recommended Current Limits:**

| Application | Stall Limit | Free Limit | Threshold RPM |
|------------|-------------|------------|---------------|
| **Drive Motors** | 40-60A | 60-80A | 5000 RPM |
| **Shooter/Flywheel** | 30-40A | 60-80A | 3000 RPM |
| **Intake/Manipulator** | 20-30A | 40-60A | 3000 RPM |
| **Small Mechanisms** | 10-20A | 20-40A | 2000 RPM |
| **NEO 550** | 15-20A | 30-40A | 8000 RPM |

```java
// Configure current limits
motor.configureCurrentLimits(40, 60, 5000);
// 40A stall, 60A free spin, threshold at 5000 RPM
```

### Velocity Units: RPS

**BREAKING CHANGE (v2026.2.0):** NEOMotor now uses **rotations per second (RPS)** instead of RPM for consistency with Phoenix 6 motors.

```java
// Old (before v2026.2.0)
motor.set(ControlMode.VELOCITY, 5000);  // 5000 RPM
double rpm = motor.getVelocity();       // Returns 5000

// New (v2026.2.0+)
motor.set(ControlMode.VELOCITY, 83.33);  // 83.33 RPS (5000 RPM / 60)
double rps = motor.getVelocity();        // Returns 83.33

// Helper conversion
double rps = rpm / 60.0;
double rpm = rps * 60.0;
```

### Control Mode Support

| Mode | Support | Notes |
|------|---------|-------|
| **PERCENT_OUTPUT** | ✓ Full | Open-loop duty cycle |
| **POSITION** | ✓ Full | Closed-loop position control |
| **VELOCITY** | ✓ Full | Closed-loop velocity control (RPS) |
| **VOLTAGE** | ✓ Full | Direct voltage control |
| **CURRENT** | ✓ Full | Torque current control (unique to NEO) |
| **MOTION_MAGIC** | ✓ Full | Uses REV MAXMotion profiling |
| **MOTION_MAGIC_FOC_TORQUE** | ⚠ Fallback | Falls back to MOTION_MAGIC with warning |
| **FOLLOWER** | ✓ Full | Follow another SPARK MAX |

### REVLib Best Practices

NEOMotor follows REVLib best practices for optimal performance:

**Configuration Persistence:**
- **Initial Setup:** Uses `PersistMode.kPersistParameters` to survive brownouts
- **Runtime Changes:** Uses `PersistMode.kNoPersistParameters` to avoid blocking flash writes
- **Follower Setup:** Uses `PersistMode.kPersistParameters` (one-time configuration)

```java
// Constructor: PERSIST on initial setup (survives brownouts)
motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

// Runtime: NO PERSIST to avoid blocking flash writes
motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

// Follower: PERSIST because it's one-time setup
motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
```

**Why This Matters:**
- Flash writes block for ~20ms each
- Frequent flash writes during operation cause lag
- Initial persistence ensures configuration survives power cycles

## Common Patterns

### Shooter with Velocity Control

```java
public class Shooter {
    private final NEOMotor shooterMotor;

    public Shooter() {
        shooterMotor = new NEOMotor(5, false, 40, false);

        // Velocity PID with feed-forward
        shooterMotor.setPID(0, 0.0001, 0.0, 0.0, 0.00017);

        // Coast mode for flywheel
        shooterMotor.setBrakeMode(false);

        // Voltage compensation for consistency
        shooterMotor.enableVoltageCompensation(12.0);

        // Current limiting
        shooterMotor.configureCurrentLimits(40, 80, 3000);
    }

    public void setVelocity(double rps) {
        shooterMotor.set(ControlMode.VELOCITY, rps);
    }

    public boolean atSpeed(double targetRPS, double tolerance) {
        return Math.abs(shooterMotor.getVelocity() - targetRPS) < tolerance;
    }

    public double getVelocity() {
        return shooterMotor.getVelocity();  // Returns RPS
    }
}
```

### Elevator with Position Control

```java
public class Elevator {
    private final NEOMotor elevatorMotor;

    public enum Height {
        BOTTOM(0.0),
        MID(50.0),
        TOP(100.0);

        public final double rotations;
        Height(double rotations) { this.rotations = rotations; }
    }

    public Elevator() {
        elevatorMotor = new NEOMotor(8, false, 30, false);

        // Position PID
        elevatorMotor.setPID(0, 0.1, 0.0, 0.01, 0.0);

        // Brake mode to hold position
        elevatorMotor.setBrakeMode(true);

        // Soft limits for safety
        elevatorMotor.configureSoftLimits(100.0, 0.0, true);

        // Current limiting
        elevatorMotor.configureCurrentLimits(30, 50, 3000);

        // Zero at bottom
        elevatorMotor.setPosition(0.0);
    }

    public void setHeight(Height height) {
        elevatorMotor.set(ControlMode.POSITION, height.rotations);
    }

    public double getHeight() {
        return elevatorMotor.getPosition();
    }

    public boolean atTarget(double tolerance) {
        return Math.abs(elevatorMotor.getPosition() - getTargetHeight()) < tolerance;
    }
}
```

### Drive Motor with MAXMotion

```java
public class DriveModule {
    private final NEOMotor driveMotor;

    public DriveModule(int canID, boolean inverted) {
        driveMotor = new NEOMotor(canID, false, 40, inverted);

        // MAXMotion configuration (RPS units)
        driveMotor.configureMotionMagic(
            80.0,   // 80 RPS cruise velocity (4800 RPM)
            160.0,  // 160 RPS² acceleration
            0.0     // No jerk limiting
        );

        // Motion Magic PID
        driveMotor.setPID(0, 0.05, 0.0, 0.0, 0.0);

        // Brake mode for position control
        driveMotor.setBrakeMode(true);

        // Current limiting for drive
        driveMotor.configureCurrentLimits(40, 60, 5000);
    }

    public void setTargetPosition(double rotations) {
        driveMotor.set(ControlMode.MOTION_MAGIC, rotations);
    }

    public boolean atTarget(double tolerance) {
        double error = Math.abs(driveMotor.getPosition() - getTarget());
        return error < tolerance;
    }
}
```

### Multi-Motor Subsystem with Followers

```java
public class Intake {
    private final NEOMotor leader;
    private final NEOMotor follower;

    public Intake() {
        // Leader motor
        leader = new NEOMotor(10, false, 25, false);
        leader.configure()
            .currentLimits(25, 40, 3000)
            .brakeMode(false)
            .apply();

        // Follower motor (mirrors leader)
        follower = new NEOMotor(11, false, 25, false);
        follower.setStrictFollower(10);  // Follow CAN ID 10

        // Note: Only configure leader, follower mirrors all commands
    }

    public void intake() {
        leader.set(0.8);  // Follower automatically matches
    }

    public void outtake() {
        leader.set(-0.5);
    }

    public void stop() {
        leader.set(0.0);
    }

    public double getCurrent() {
        // Monitor both motors
        return leader.getCurrentDraw() + follower.getCurrentDraw();
    }
}
```

## Configuration Examples

### Builder Pattern (Recommended)

```java
// Complete drive motor setup
motor.configure()
    .pid(0.1, 0.0, 0.05, 0.0)
    .currentLimits(40, 60, 5000)
    .motionMagic(80.0, 160.0, 0.0)
    .brakeMode(false)
    .voltageCompensation(12.0)
    .apply();

// Minimal setup for simple mechanism
motor.configure()
    .currentLimits(20, 40, 3000)
    .brakeMode(true)
    .apply();
```

### Traditional Method Calls

```java
// Position control configuration
motor.setPID(0, 0.1, 0.0, 0.01, 0.0);
motor.configureSoftLimits(100.0, 0.0, true);
motor.setBrakeMode(true);
motor.configureCurrentLimits(30, 50, 3000);
motor.enableVoltageCompensation(12.0);

// Velocity control configuration
motor.setPID(0, 0.0001, 0.0, 0.0, 0.00017);
motor.setBrakeMode(false);
motor.configureCurrentLimits(40, 80, 3000);
motor.enableVoltageCompensation(12.0);
```

## PID Tuning Guide

### Starting Values by Application

**Position Control (Elevator, Arm):**
```java
motor.setPID(0, 0.1, 0.0, 0.01, 0.0);
// kP: 0.1 - Adjust until minimal oscillation
// kI: 0.0 - Usually not needed
// kD: 0.01 - Add to reduce overshoot
// kF: 0.0 - Not used for position
```

**Velocity Control (Shooter, Flywheel):**
```java
motor.setPID(0, 0.0001, 0.0, 0.0, 0.00017);
// kP: 0.0001 - Start small for velocity
// kI: 0.0 - Avoid integral windup
// kD: 0.0 - Usually not needed
// kF: 0.00017 - Calculate: 1 / (max_velocity_rps)
```

**Feed-Forward Calculation:**
```java
// For velocity control
double maxVelocityRPS = 98.0;  // NEO free speed: 5880 RPM / 60
double kF = 1.0 / maxVelocityRPS;
// kF ≈ 0.0102 for NEO
// kF ≈ 0.0088 for NEO Vortex

// Adjusted for system efficiency (80%)
double kF_adjusted = kF * 0.8;
```

### Tuning Process

1. **Start with kP only:**
   - Increase until system responds
   - Watch for oscillation

2. **Add kD if oscillating:**
   - Start with kD = 10 * kP
   - Reduce oscillation

3. **Add kI if steady-state error:**
   - Start very small (0.0001)
   - Watch for instability

4. **Tune kF for velocity:**
   - Calculate from motor specs
   - Fine-tune empirically

## Troubleshooting

### Motor doesn't move

**Possible Causes:**
- Incorrect CAN ID
- Motor controller not powered
- Current limit too low
- Brake mode enabled (may need coast)

**Solutions:**
```java
// Verify CAN ID
motor.getMotorType();  // Check if motor responds

// Check current limit
motor.configureCurrentLimits(40, 60, 5000);

// Try coast mode
motor.setBrakeMode(false);

// Test with open-loop
motor.set(0.25);
```

### Inconsistent velocity

**Possible Causes:**
- No voltage compensation
- PID gains need tuning
- Current limiting triggering

**Solutions:**
```java
// Add voltage compensation
motor.enableVoltageCompensation(12.0);

// Increase current limits
motor.configureCurrentLimits(60, 80, 3000);

// Tune feed-forward
double maxRPS = 98.0;  // NEO @ 5880 RPM
motor.setPID(0, 0.0001, 0.0, 0.0, 1.0 / maxRPS);
```

### Position drifts

**Possible Causes:**
- Coast mode instead of brake
- Insufficient kP
- No kI for steady-state correction

**Solutions:**
```java
// Enable brake mode
motor.setBrakeMode(true);

// Increase kP
motor.setPID(0, 0.2, 0.0, 0.02, 0.0);

// Add small kI
motor.setPID(0, 0.1, 0.0001, 0.01, 0.0);
```

### Current limit violations

**Monitor current draw:**
```java
double current = motor.getCurrentDraw();
System.out.println("Current: " + current + "A");

// Check if hitting stall limit
if (current > 35) {
    System.out.println("Motor stalled or overloaded!");
}
```

**Adjust limits:**
```java
// Increase stall limit if needed
motor.configureCurrentLimits(50, 80, 5000);
```

## Comparison with Other Motors

| Feature | NEO | Falcon 500 | Kraken X60 | Minion |
|---------|-----|------------|------------|--------|
| **Free Speed** | 5880 RPM | 6380 RPM | 6000 RPM | 5800 RPM |
| **Stall Torque** | 3.36 Nm | 4.69 Nm | 7.09 Nm | 1.26 Nm |
| **Weight** | 453g | 435g | 435g | 150g |
| **Current Control** | ✓ Yes | ✗ No | ✓ Yes | ✗ No |
| **FOC** | ✗ No | ✓ Yes | ✓ Yes | ✗ No |
| **API** | REVLib | Phoenix 6 | Phoenix 6 | Phoenix 6 |
| **Velocity Units** | RPS | RPS | RPS | RPS |

**When to Use NEO:**
- Need current control mode
- Prefer REV ecosystem
- Already using SPARK MAX controllers
- Want integrated encoder (no external required)

**When to Use TalonFX:**
- Need higher torque (Falcon/Kraken)
- Want FOC for efficiency
- Prefer CTRE ecosystem
- Need Phoenix 6 features

## Migration from WPILib CANSparkMax

```java
// WPILib CANSparkMax
CANSparkMax motor = new CANSparkMax(5, MotorType.kBrushless);
motor.set(0.5);

// AdambotsLib NEOMotor
NEOMotor motor = new NEOMotor(5, false, 40, false);
motor.set(0.5);

// PID Configuration
// WPILib
SparkPIDController pid = motor.getPIDController();
pid.setP(0.1);
pid.setI(0.0);
pid.setD(0.01);
pid.setFF(0.0);

// AdambotsLib
motor.setPID(0, 0.1, 0.0, 0.01, 0.0);

// Velocity Control
// WPILib (RPM)
pid.setReference(5000, CANSparkMax.ControlType.kVelocity);

// AdambotsLib (RPS)
motor.set(ControlMode.VELOCITY, 83.33);  // 5000 RPM / 60
```

## See Also

- **[BaseMotor API](BaseMotor.md)** - Complete interface documentation
- **[Motors Overview](README.md)** - All motor types and patterns
- **[TalonFXMotor](TalonFXMotor.md)** - Falcon 500 / Kraken X60 comparison
- **[MinionMotor](MinionMotor.md)** - Compact motor alternative
- **[Actuators Overview](../README.md)** - All actuator types
