# Encoders

Encoders measure rotational position for arms, turrets, and other mechanisms that need precise angle feedback. AdambotsLib supports absolute encoders that maintain their position through power cycles, eliminating the need for homing routines.

## Available Implementations

| Class | Hardware | Connection | Resolution | Features |
|-------|----------|------------|------------|----------|
| [CANCoder](CANCoder.md) | CTRE CANcoder | CAN | 4096 CPR | Magnetic, CAN-based |
| [ThroughBoreEncoder](ThroughBoreEncoder.md) | REV Through Bore | DIO (DutyCycle) | 8192 CPR | Optical, absolute |

## Quick Start

```java
import com.adambots.lib.sensors.*;

// REV Through Bore on DIO port 5
BaseAbsoluteEncoder armEncoder = new ThroughBoreEncoder(5);

// CTRE CANcoder on CAN ID 2
BaseAbsoluteEncoder turretEncoder = new CANCoder(2);

// Get position in degrees
double angleDeg = armEncoder.getAbsolutePositionDegrees();

// Get as Rotation2d for WPILib
Rotation2d rotation = armEncoder.getAbsolutePositionRotation2D();
```

## Interface: BaseAbsoluteEncoder

All encoder implementations follow the `BaseAbsoluteEncoder` interface:

```java
public interface BaseAbsoluteEncoder {
    double getAbsolutePositionDegrees();
    double getAbsolutePositionRadians();
    Rotation2d getAbsolutePositionRotation2D();
}
```

## Key Concepts

### Absolute vs Relative Encoders

**Absolute Encoders** (AdambotsLib):
- Remember position through power cycles
- No homing required on startup
- Know position immediately when robot boots
- Ideal for arms, turrets, swerve modules

**Relative Encoders** (not in AdambotsLib):
- Reset to zero on power cycle
- Require homing routine
- Good for continuous rotation (drivetrain wheels)

### Position Units

All encoders provide multiple unit outputs:

| Method | Units | Use Case |
|--------|-------|----------|
| `getAbsolutePositionDegrees()` | 0-360° | Simple angle calculations |
| `getAbsolutePositionRadians()` | 0-2π | Trigonometry, physics |
| `getAbsolutePositionRotation2D()` | Rotation2d | WPILib kinematics |

```java
// Use degrees for simple logic
double angle = encoder.getAbsolutePositionDegrees();
if (angle > 90 && angle < 180) {
    // Arm in safe zone
}

// Use Rotation2d for WPILib
Rotation2d rotation = encoder.getAbsolutePositionRotation2D();
SwerveModuleState state = new SwerveModuleState(speed, rotation);
```

## Common Patterns

### Mechanism Position Tracking

```java
public class ArmSubsystem extends SubsystemBase {
    private final BaseAbsoluteEncoder m_encoder;
    private final BaseMotor m_motor;

    public ArmSubsystem() {
        m_encoder = new ThroughBoreEncoder(5);
        m_motor = new NEOMotor(3, false, 40, false);
    }

    public final Trigger atSetpoint = new Trigger(() ->
        MathUtil.isNear(
            m_encoder.getAbsolutePositionDegrees(),
            m_targetAngle,
            2.0
        )
    ).debounce(0.1);

    public Command moveToAngle(double targetDeg) {
        return run(() -> {
            double current = m_encoder.getAbsolutePositionDegrees();
            double error = targetDeg - current;
            double output = m_pid.calculate(error);
            m_motor.set(output);
        }).until(atSetpoint);
    }
}
```

### Swerve Module Azimuth

```java
public class SwerveModule {
    private final BaseAbsoluteEncoder m_azimuthEncoder;
    private final BaseMotor m_azimuthMotor;
    private final double m_offset;

    public SwerveModule(int encoderPort, double magnetOffset) {
        m_azimuthEncoder = new CANCoder(encoderPort);
        m_offset = magnetOffset;
    }

    public Rotation2d getAngle() {
        double rawAngle = m_azimuthEncoder.getAbsolutePositionDegrees();
        double adjusted = (rawAngle - m_offset + 360) % 360;
        return Rotation2d.fromDegrees(adjusted);
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            getDrivePosition(),
            getAngle()
        );
    }
}
```

### Position-Based State Machine

```java
public class ElevatorSubsystem extends SubsystemBase {
    private final BaseAbsoluteEncoder m_encoder;

    public enum Position {
        STOWED(0),
        LOW(45),
        MID(90),
        HIGH(135);

        public final double angle;
        Position(double angle) { this.angle = angle; }
    }

    // Triggers for each position
    public final Trigger atStowed = createPositionTrigger(Position.STOWED);
    public final Trigger atLow = createPositionTrigger(Position.LOW);
    public final Trigger atMid = createPositionTrigger(Position.MID);
    public final Trigger atHigh = createPositionTrigger(Position.HIGH);

    private Trigger createPositionTrigger(Position pos) {
        return new Trigger(() ->
            MathUtil.isNear(
                m_encoder.getAbsolutePositionDegrees(),
                pos.angle,
                3.0
            )
        ).debounce(0.15);
    }
}
```

### Soft Limits

```java
public class TurretSubsystem extends SubsystemBase {
    private final BaseAbsoluteEncoder m_encoder;
    private static final double MIN_ANGLE = 0.0;
    private static final double MAX_ANGLE = 270.0;

    public final Trigger atMinLimit = new Trigger(() ->
        m_encoder.getAbsolutePositionDegrees() < MIN_ANGLE + 5.0
    );

    public final Trigger atMaxLimit = new Trigger(() ->
        m_encoder.getAbsolutePositionDegrees() > MAX_ANGLE - 5.0
    );

    public Command rotate(DoubleSupplier speed) {
        return run(() -> {
            double current = m_encoder.getAbsolutePositionDegrees();
            double requestedSpeed = speed.getAsDouble();

            // Prevent movement past limits
            if (current < MIN_ANGLE && requestedSpeed < 0) {
                m_motor.stopMotor();
            } else if (current > MAX_ANGLE && requestedSpeed > 0) {
                m_motor.stopMotor();
            } else {
                m_motor.set(requestedSpeed);
            }
        });
    }
}
```

## Best Practices

### 1. Use Rotation2d for WPILib Integration

```java
// Good: Compatible with WPILib kinematics
public Rotation2d getModuleAngle() {
    return m_encoder.getAbsolutePositionRotation2D();
}

// Avoid: Extra conversion steps
public Rotation2d getModuleAngle() {
    double degrees = m_encoder.getAbsolutePositionDegrees();
    return Rotation2d.fromDegrees(degrees);
}
```

### 2. Cache Encoder Values

```java
private double m_cachedPosition;

@Override
public void periodic() {
    // Read once per iteration
    m_cachedPosition = m_encoder.getAbsolutePositionDegrees();
}

// Use cached value in triggers
public final Trigger atSetpoint = new Trigger(() ->
    MathUtil.isNear(m_cachedPosition, m_target, 2.0)
);
```

### 3. Calibrate Encoder Offsets

```java
// Store offset in constants
public static final double TURRET_ENCODER_OFFSET = 127.3;

public double getCalibratedAngle() {
    double raw = m_encoder.getAbsolutePositionDegrees();
    return (raw - TURRET_ENCODER_OFFSET + 360) % 360;
}
```

### 4. Use MathUtil.isNear() for Comparisons

```java
// Good: Tolerant comparison
public final Trigger atTarget = new Trigger(() ->
    MathUtil.isNear(
        m_encoder.getAbsolutePositionDegrees(),
        m_targetAngle,
        2.0
    )
).debounce(0.1);

// Avoid: Direct equality
public final Trigger atTarget = new Trigger(() ->
    m_encoder.getAbsolutePositionDegrees() == m_targetAngle
);
```

### 5. Handle Angle Wrapping

```java
// For continuous mechanisms that can spin 360°+
public double getShortestPathToTarget(double target) {
    double current = m_encoder.getAbsolutePositionDegrees();
    double error = target - current;

    // Wrap to [-180, 180]
    while (error > 180) error -= 360;
    while (error < -180) error += 360;

    return error;
}
```

## Hardware Comparison

### REV Through Bore Encoder

**Pros:**
- Higher resolution (8192 CPR)
- DutyCycle protocol (no CAN overhead)
- Compact size
- Good for swerve modules

**Cons:**
- Requires DIO port (limited to 10 on RoboRIO 2)
- Shorter cable options
- Must be close to RoboRIO

**Best For:** Swerve azimuth, arms, elevators

### CTRE CANcoder

**Pros:**
- CAN bus (long distances OK)
- Easy daisy-chaining
- Phoenix Tuner configuration
- Robust connections

**Cons:**
- Lower resolution (4096 CPR)
- Uses CAN bandwidth
- Requires CAN ID configuration

**Best For:** Turrets, mechanisms far from RoboRIO, robots with many sensors

## Calibration

### Finding Encoder Offsets

```java
// 1. Move mechanism to known "zero" position
// 2. Read raw encoder value
double rawValue = m_encoder.getAbsolutePositionDegrees();
System.out.println("Raw encoder at zero: " + rawValue);

// 3. Store this as offset constant
public static final double ENCODER_OFFSET = 237.5;  // Value from step 2

// 4. Subtract offset when reading
public double getCalibratedPosition() {
    return (m_encoder.getAbsolutePositionDegrees() - ENCODER_OFFSET + 360) % 360;
}
```

## Troubleshooting

### Encoder Reads Zero or Invalid

**Cause:** Not connected or bad wiring

**Solutions:**
1. Check physical connections
2. Verify port/CAN ID matches code
3. Check encoder LED (if present)
4. Test with vendor tool (Phoenix Tuner, REV Hardware Client)

---

### Position Jumps or Flickers

**Cause:** Magnetic interference or loose connection

**Solutions:**
1. For CANcoder: Check magnet alignment and distance
2. For Through Bore: Verify cable connections
3. Keep encoders away from motors
4. Use cable strain relief

---

### Wrong Direction

**Cause:** Encoder mounted backwards or magnet flipped

**Solutions:**
1. Physically flip encoder or magnet
2. Or invert in software:

```java
public double getPosition() {
    return 360.0 - m_encoder.getAbsolutePositionDegrees();
}
```

---

### Position Doesn't Match Mechanism

**Cause:** Need to calibrate offset

**Solution:** Find and apply offset (see Calibration section)

## See Also

- **[CANCoder](CANCoder.md)** - CTRE CANcoder documentation
- **[ThroughBoreEncoder](ThroughBoreEncoder.md)** - REV Through Bore documentation
- **[Sensors Overview](../README.md)** - All sensor types
