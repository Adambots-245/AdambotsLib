# TalonFXMotor

CTRE TalonFX motor controller implementation for Falcon 500 and Kraken X60 motors using Phoenix 6 API.

## Constructors

```java
public TalonFXMotor(int portNum, boolean isOnCANivore, double supplyCurrentLimit, boolean isKraken)
```

**Parameters:**
- `portNum` - CAN ID for the TalonFX controller (0-62)
- `isOnCANivore` - True if motor is on CANivore bus, false for regular CAN bus
- `supplyCurrentLimit` - Supply current limit in amperes
- `isKraken` - True for Kraken X60, false for Falcon 500

**Example:**
```java
// Falcon 500 on regular CAN bus
BaseMotor falcon = new TalonFXMotor(1, false, 40, false);

// Kraken X60 on CANivore
BaseMotor kraken = new TalonFXMotor(5, true, 60, true);

// Falcon 500 drive motor with high current limit
BaseMotor drive = new TalonFXMotor(2, false, 80, false);
```

## Hardware Specifications

### Falcon 500

| Specification | Value |
|--------------|-------|
| **Free Speed** | 6380 RPM (106.3 RPS) |
| **Stall Torque** | 4.69 Nm |
| **Stall Current** | 257A |
| **Weight** | 435g (with TalonFX) |
| **Encoder** | Integrated CANCoder (2048 CPR) |
| **FOC** | Yes |
| **Controller** | TalonFX |

### Kraken X60

| Specification | Value |
|--------------|-------|
| **Free Speed** | 6000 RPM (100 RPS) |
| **Stall Torque** | 7.09 Nm (51% more than Falcon) |
| **Stall Current** | 366A |
| **Weight** | 435g (with TalonFX) |
| **Encoder** | Integrated (2048 CPR) |
| **FOC** | Yes (enhanced) |
| **Current Control** | TorqueCurrentFOC supported |

## Controller: TalonFX

- **CAN ID Range:** 0-62
- **CAN Buses:** Regular CAN or CANivore
- **PID Slots:** 3 slots (0-2)
- **Control Modes:** All BaseMotor modes
- **Motion Profiling:** Motion Magic with jerk control
- **Status Frames:** Optimized update frequencies (50 Hz position/velocity, 25 Hz limits)
- **Firmware:** Phoenix 6 API
- **Configuration:** Factory reset on init, automatic retry
- **Initialization:** Applies factory reset (`TalonFXConfiguration`) to clear any stale configs persisted in flash memory

## Key Features

### Field-Oriented Control (FOC)

Both Falcon 500 and Kraken X60 support FOC for improved efficiency and torque delivery. **FOC is enabled by default** — it requires a Phoenix Pro license to take effect, but is safely ignored without one (no-op).

```java
TalonFXMotor motor = new TalonFXMotor(1, false, 40, false);

// FOC is active for all control modes by default:
motor.set(ControlMode.PERCENT_OUTPUT, 0.5);  // Uses FOC
motor.set(ControlMode.VELOCITY, 50.0);       // Uses FOC
```

**FOC Benefits (with Phoenix Pro license):**
- ~10-15% efficiency improvement
- Smoother low-speed operation
- Better torque control
- Reduced heat generation

### Kraken-Specific: Current Control

Kraken X60 supports direct current control via TorqueCurrentFOC:

```java
TalonFXMotor kraken = new TalonFXMotor(5, true, 60, true);

// Direct current control (Kraken only)
kraken.set(ControlMode.CURRENT, 20.0);  // 20A torque current

// Falcon 500 fallback
TalonFXMotor falcon = new TalonFXMotor(1, false, 40, false);
falcon.set(ControlMode.CURRENT, 20.0);  // Logs warning, falls back to duty cycle
```

### Velocity Units: RPS

TalonFXMotor uses **rotations per second (RPS)** for all velocity operations:

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

| Mode | Falcon 500 | Kraken X60 | Notes |
|------|------------|------------|-------|
| **PERCENT_OUTPUT** | ✓ Full | ✓ Full | Duty cycle control |
| **POSITION** | ✓ Full | ✓ Full | Closed-loop position |
| **VELOCITY** | ✓ Full | ✓ Full | Closed-loop velocity (RPS) |
| **VOLTAGE** | ✓ Full | ✓ Full | Direct voltage control |
| **CURRENT** | ⚠ Fallback | ✓ Full | Kraken: TorqueCurrentFOC<br>Falcon: Falls back to duty cycle |
| **MOTION_MAGIC** | ✓ Full | ✓ Full | Trapezoidal profiling |
| **MOTION_MAGIC_FOC_TORQUE** | ✓ Full | ✓ Full | Motion Magic with FOC torque |
| **FOLLOWER** | ✓ Full | ✓ Full | Follow another TalonFX |

### Simulation Support

TalonFXMotor exposes simulation methods for use with WPILib physics simulations. The sim state is cached on construction (only when running in simulation) and provides access to the underlying `TalonFXSimState`.

**Sim Loop Pattern (Arm Example):**

```java
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class ArmSubsystem extends SubsystemBase {
    private final BaseMotor motor;
    private final SingleJointedArmSim armSim;
    private static final double GEAR_RATIO = 50.0;

    public ArmSubsystem() {
        motor = new TalonFXMotor(1, false, 40, false);
        motor.configureSensorToMechanismRatio(GEAR_RATIO);

        armSim = new SingleJointedArmSim(
            DCMotor.getFalcon500(1),
            GEAR_RATIO,
            1.0,  // MOI kg*m^2
            0.5,  // arm length meters
            -Math.PI / 2, Math.PI / 2,
            true, 0
        );
    }

    @Override
    public void simulationPeriodic() {
        // 1. Feed battery voltage into the sim state
        motor.setSimSupplyVoltage(RobotController.getBatteryVoltage());

        // 2. Get commanded voltage and feed into physics sim
        armSim.setInputVoltage(motor.getSimMotorVoltage());
        armSim.update(0.020);

        // 3. Write physics results back (rotor units = mechanism * gear ratio)
        motor.setSimPosition(armSim.getAngleRads() / (2 * Math.PI) * GEAR_RATIO);
        motor.setSimVelocity(armSim.getVelocityRadPerSec() / (2 * Math.PI) * GEAR_RATIO);
    }
}
```

**Important:** `setSimPosition` and `setSimVelocity` use **rotor** units (before gear ratio). If you've configured `configureSensorToMechanismRatio()`, multiply mechanism values by the gear ratio.

### Motor Direction (v2026.3.3+)

TalonFXMotor overrides `setDirection()` to map directly to Phoenix 6 `InvertedValue.Clockwise_Positive` and `InvertedValue.CounterClockwise_Positive`:

```java
// Explicit CW/CCW direction
motor.setDirection(BaseMotor.MotorDirection.CLOCKWISE_POSITIVE);

// Or via the builder pattern
motor.configure()
    .direction(BaseMotor.MotorDirection.CLOCKWISE_POSITIVE)
    .brakeMode(true)
    .apply();

// The simple boolean API still works
motor.setInverted(true);  // Equivalent to CLOCKWISE_POSITIVE
```

### Phoenix 6 Configuration Best Practices

TalonFXMotor follows Phoenix 6 best practices:

**Factory Reset on Initialization (v2026.2.9+)**

TalonFX motor controllers persist their configuration in flash memory across power cycles. This can cause unexpected behavior when motor configs from previous code deployments or testing sessions remain active. TalonFXMotor automatically applies a factory reset on construction before applying any library configurations:

```java
// Handled automatically in the TalonFXMotor constructor
motor.getConfigurator().apply(new TalonFXConfiguration(), 0.050);
// All subsequent configuration starts from a clean slate
```

This ensures:
- No stale inversion, brake mode, or PID settings carry over
- Consistent behavior regardless of what was previously deployed
- Eliminates hard-to-debug issues from persistent flash configs

**Automatic Retry Logic**

Configuration failures are automatically retried up to 3 times:

```java
// Automatically handled by TalonFXMotor
motor.setPID(0, 0.1, 0.0, 0.01, 0.0);
// If first attempt fails, retries up to 3 times
// Logs warning to DriverStation if all retries fail
```

**Status Frame Optimization**

Status frames are pre-configured for optimal CAN bus usage:

```java
// Configured automatically in constructor
motor.getVelocity().setUpdateFrequency(50);   // 50 Hz
motor.getPosition().setUpdateFrequency(50);   // 50 Hz
motor.getForwardLimit().setUpdateFrequency(25);  // 25 Hz
motor.getReverseLimit().setUpdateFrequency(25);  // 25 Hz
motor.optimizeBusUtilization();
```

## Common Patterns

### Shooter with FOC Velocity Control

```java
public class Shooter {
    private final TalonFXMotor shooterMotor;

    public Shooter() {
        // Kraken X60 on CANivore (FOC enabled by default)
        shooterMotor = new TalonFXMotor(5, true, 60, true);

        // Velocity PID with feed-forward
        shooterMotor.setPID(0, 0.05, 0.0, 0.0, 0.01);

        // Coast mode for flywheel
        shooterMotor.setBrakeMode(false);

        // High current limit for acceleration
        shooterMotor.configureCurrentLimits(60, 120, 3000);
    }

    public void setVelocity(double rps) {
        shooterMotor.set(ControlMode.VELOCITY, rps);
    }

    public boolean atSpeed(double targetRPS, double tolerance) {
        return Math.abs(shooterMotor.getVelocity() - targetRPS) < tolerance;
    }

    public double getCurrent() {
        return shooterMotor.getCurrentDraw();
    }
}
```

### Elevator with Motion Magic

```java
public class Elevator {
    private final TalonFXMotor elevatorMotor;

    public enum Height {
        BOTTOM(0.0),
        MID(50.0),
        TOP(100.0);

        public final double rotations;
        Height(double rotations) { this.rotations = rotations; }
    }

    public Elevator() {
        // Falcon 500 with Motion Magic
        elevatorMotor = new TalonFXMotor(8, false, 40, false);

        // Motion Magic configuration (RPS units)
        elevatorMotor.configureMotionMagic(
            50.0,   // 50 RPS cruise velocity (3000 RPM)
            150.0,  // 150 RPS² acceleration
            500.0   // 500 RPS³ jerk limiting
        );

        // Position PID
        elevatorMotor.setPID(0, 0.1, 0.0, 0.01, 0.0);

        // Brake mode to hold position
        elevatorMotor.setBrakeMode(true);

        // Soft limits for safety
        elevatorMotor.configureSoftLimits(100.0, 0.0, true);

        // Current limiting
        elevatorMotor.configureCurrentLimits(40, 60, 3000);

        // Zero at bottom
        elevatorMotor.setPosition(0.0);
    }

    public void setHeight(Height height) {
        elevatorMotor.set(ControlMode.MOTION_MAGIC, height.rotations);
    }

    public double getHeight() {
        return elevatorMotor.getPosition();
    }
}
```

### Swerve Drive Module

```java
public class SwerveModule {
    private final TalonFXMotor driveMotor;
    private final TalonFXMotor steerMotor;

    public SwerveModule(int driveID, int steerID, boolean onCANivore) {
        // Kraken X60 for drive (FOC enabled by default)
        driveMotor = new TalonFXMotor(driveID, onCANivore, 60, true);
        driveMotor.configure()
            .pid(0.1, 0.0, 0.01, 0.0)
            .currentLimits(60, 80, 5000)
            .brakeMode(true)
            .apply();

        // Falcon 500 for steering (FOC enabled by default)
        steerMotor = new TalonFXMotor(steerID, onCANivore, 40, false);
        steerMotor.configure()
            .pid(0.2, 0.0, 0.02, 0.0)
            .currentLimits(40, 60, 3000)
            .softLimits(360.0, -360.0, false)  // Continuous rotation
            .brakeMode(true)
            .apply();
    }

    public void setDesiredState(double velocityRPS, double angleRotations) {
        driveMotor.set(ControlMode.VELOCITY, velocityRPS);
        steerMotor.set(ControlMode.POSITION, angleRotations);
    }
}
```

### Multi-Motor Drivetrain with Followers

```java
public class Drivetrain {
    private final TalonFXMotor leftLeader;
    private final TalonFXMotor leftFollower;
    private final TalonFXMotor rightLeader;
    private final TalonFXMotor rightFollower;

    public Drivetrain() {
        // Leader motors on CANivore
        leftLeader = new TalonFXMotor(1, true, 60, false);
        rightLeader = new TalonFXMotor(3, true, 60, false);

        // Follower motors
        leftFollower = new TalonFXMotor(2, true, 60, false);
        rightFollower = new TalonFXMotor(4, true, 60, false);

        // Configure followers
        leftFollower.setStrictFollower(1);
        rightFollower.setStrictFollower(3);

        // Configure leaders only (followers mirror, FOC enabled by default)
        for (TalonFXMotor leader : List.of(leftLeader, rightLeader)) {
            leader.configure()
                .currentLimits(60, 80, 5000)
                .brakeMode(false)  // Coast for drive
                .apply();
        }

        // Invert right side
        rightLeader.setInverted(true);
    }

    public void tankDrive(double left, double right) {
        leftLeader.set(left);
        rightLeader.set(right);
    }
}
```

### Kraken Current Control for Intake

```java
public class Intake {
    private final TalonFXMotor intakeMotor;

    public Intake() {
        // Kraken X60 with current control
        intakeMotor = new TalonFXMotor(10, false, 40, true);

        // Check if current control is supported
        if (intakeMotor.supportsControlMode(ControlMode.CURRENT)) {
            System.out.println("Using Kraken current control");
        }
    }

    public void intakeWithCurrentLimit() {
        // Use current control to prevent jamming
        intakeMotor.set(ControlMode.CURRENT, 15.0);  // 15A max
    }

    public void intakeFullSpeed() {
        intakeMotor.set(ControlMode.PERCENT_OUTPUT, 0.8);
    }

    public boolean hasGamePiece() {
        // Detect game piece by current spike
        return intakeMotor.getCurrentDraw() > 10.0;
    }
}
```

## Configuration Examples

### Builder Pattern (Recommended)

```java
// Complete configuration
motor.configure()
    .pid(0.1, 0.0, 0.01, 0.0)
    .motionMagic(50.0, 150.0, 500.0)
    .currentLimits(40, 60, 5000)
    .softLimits(100.0, 0.0, true)
    .inverted(true)
    .brakeMode(true)
    .apply();

// Minimal configuration
motor.configure()
    .currentLimits(40, 60, 5000)
    .brakeMode(true)
    .apply();
```

### Traditional Method Calls

```java
// Position control with Motion Magic
motor.setPID(0, 0.1, 0.0, 0.01, 0.0);
motor.configureMotionMagic(50.0, 150.0, 500.0);
motor.configureSoftLimits(100.0, 0.0, true);
motor.setBrakeMode(true);
motor.configureCurrentLimits(40, 60, 3000);

// Velocity control (FOC enabled by default)
motor.setPID(0, 0.05, 0.0, 0.0, 0.01);
motor.setBrakeMode(false);
motor.configureCurrentLimits(60, 120, 3000);
```

## Feed-Forward Configuration

TalonFX uses arbitrary feed-forward that can be set per control request:

```java
// Set feed-forward value
motor.setFeedForward(0.5);

// Feed-forward is automatically applied to position/velocity/motion magic
motor.set(ControlMode.VELOCITY, 50.0);  // Uses feed-forward 0.5
```

## Extended Feedforward and Gravity Compensation (v2026.2.6+)

TalonFXMotor supports the extended `setPID` with kS, kA, and kG feedforward gains across all 3 PID slots, plus `configureGravity()` for arm/elevator gravity compensation.

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

**Important:** Phoenix 6 voltage compensation works differently than Phoenix 5 or WPILib.

```java
// WRONG - enableVoltageCompensation() doesn't work in Phoenix 6
motor.enableVoltageCompensation(12.0);  // ❌ Logs warning, doesn't work

// CORRECT - Use voltage-based control modes
motor.set(ControlMode.VOLTAGE, 6.0);  // ✓ Uses VoltageOut
motor.set(ControlMode.VELOCITY, 50.0);  // ✓ Uses VelocityVoltage
motor.set(ControlMode.POSITION, 10.0);  // ✓ Uses PositionVoltage
motor.set(ControlMode.MOTION_MAGIC, 25.0);  // ✓ Uses MotionMagicVoltage
```

**Why:** Phoenix 6 voltage-based control modes (VoltageOut, PositionVoltage, VelocityVoltage) inherently compensate for battery voltage. Duty cycle modes (DutyCycleOut) do not.

## PID Tuning Guide

### Starting Values by Application

**Position Control (Elevator, Arm):**
```java
motor.setPID(0, 0.1, 0.0, 0.01, 0.0);
// kP: 0.1 - Adjust for responsiveness
// kI: 0.0 - Usually not needed
// kD: 0.01 - Reduce overshoot
// kV: 0.0 - Not used for position
```

**Velocity Control (Shooter, Drivetrain):**
```java
motor.setPID(0, 0.05, 0.0, 0.0, 0.01);
// kP: 0.05 - Adjust for tracking
// kI: 0.0 - Avoid windup
// kD: 0.0 - Usually not needed
// kV: 0.01 - Feed-forward gain
```

**kV Calculation for Velocity:**
```java
// Falcon 500: 6380 RPM = 106.3 RPS free speed
double maxVelocityRPS = 106.3;
double kV = 1.0 / maxVelocityRPS;  // ≈ 0.0094

// Kraken X60: 6000 RPM = 100 RPS free speed
double maxVelocityRPS = 100.0;
double kV = 1.0 / maxVelocityRPS;  // = 0.01

// Adjusted for efficiency (80%)
double kV_adjusted = kV * 0.8;
```

## CANivore vs Regular CAN

### Regular CAN Bus

```java
TalonFXMotor motor = new TalonFXMotor(1, false, 40, false);
```

**Pros:**
- Simpler wiring
- No additional hardware cost
- Good for small robots (4-8 motors)

**Cons:**
- Shared with RoboRIO
- Lower bandwidth
- Higher latency

### CANivore Bus

```java
TalonFXMotor motor = new TalonFXMotor(1, true, 40, false);
```

The `isOnCANivore` flag uses the `"*"` wildcard internally, which auto-discovers the device on any CANivore bus. This same interface is available on MinionMotor for consistency:

```java
// Consistent CANivore interface across motor types
TalonFXMotor kraken = new TalonFXMotor(5, true, 60, true);
MinionMotor minion = new MinionMotor(10, true);
```

**Pros:**
- Dedicated high-speed bus
- Lower latency
- Better for swerve drive (8+ TalonFX)
- Isolated from RoboRIO CAN bus

**Cons:**
- Additional hardware cost ($150)
- More complex wiring

**When to Use CANivore:**
- Swerve drive (8+ TalonFX)
- High-performance applications
- Need consistent low latency

## Troubleshooting

### Configuration Failures

**Symptom:** DriverStation warnings about config failures

**Causes:**
- CAN bus congestion
- Motor controller not powered
- Brownout during config

**Solution:** Automatic retry handles this
```java
// TalonFXMotor automatically retries up to 3 times
motor.setPID(0, 0.1, 0.0, 0.01, 0.0);
// Check DriverStation for warnings if all retries fail
```

### Motor doesn't move

```java
// Verify motor responds
System.out.println(motor.getMotorType());

// Check current draw
System.out.println("Current: " + motor.getCurrentDraw() + "A");

// Test with open-loop
motor.set(0.25);

// Check limits
System.out.println("Forward limit: " + motor.getForwardLimitSwitch());
System.out.println("Reverse limit: " + motor.getReverseLimitSwitch());
```

### Inconsistent velocity

```java
// Use velocity voltage mode (not duty cycle)
motor.set(ControlMode.VELOCITY, 50.0);  // Uses VelocityVoltage

// Increase current limits
motor.configureCurrentLimits(60, 120, 3000);

// Tune feed-forward
motor.setFeedForward(0.01);
motor.set(ControlMode.VELOCITY, 50.0);
```

### Current limit violations

```java
// Monitor current
double current = motor.getCurrentDraw();
double temperature = motor.getTemperature();
System.out.println("Current: " + current + "A, Temp: " + temperature + "°C");

// Increase limits if needed
motor.configureCurrentLimits(80, 120, 5000);
```

## Comparison: Falcon 500 vs Kraken X60

| Feature | Falcon 500 | Kraken X60 |
|---------|-----------|------------|
| **Stall Torque** | 4.69 Nm | 7.09 Nm (+51%) |
| **Free Speed** | 6380 RPM | 6000 RPM |
| **Stall Current** | 257A | 366A |
| **Current Control** | ⚠ Fallback | ✓ TorqueCurrentFOC |
| **FOC Efficiency** | Good | Excellent |
| **Weight** | 435g | 435g (same) |
| **Price** | Lower | Higher |
| **Best For** | General purpose | High torque, current control |

**When to Use Falcon 500:**
- Cost-sensitive projects
- Standard torque requirements
- Sufficient for most FRC applications

**When to Use Kraken X60:**
- Need maximum torque
- Want current control mode
- High-performance drivetrain
- Premium build quality

## See Also

- **[BaseMotor API](BaseMotor.md)** - Complete interface documentation
- **[Motors Overview](README.md)** - All motor types and patterns
- **[NEOMotor](NEOMotor.md)** - REV NEO alternative
- **[MinionMotor](MinionMotor.md)** - Compact motor option
- **[Actuators Overview](../README.md)** - All actuator types
