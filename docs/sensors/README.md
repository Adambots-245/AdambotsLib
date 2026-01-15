# Sensors

Sensors are hardware components that measure physical properties and provide feedback about your robot's state. AdambotsLib provides unified, safe interfaces for reading from gyroscopes, encoders, proximity sensors, and distance sensors.

## Overview

All sensor classes in AdambotsLib follow these principles:

### Safety First
- **Input Validation**: All port numbers and CAN IDs are validated at construction time
- **Graceful Degradation**: Invalid inputs produce warnings, not crashes
- **Clear Errors**: DriverStation warnings explain what went wrong and how it was fixed
- **Safe Defaults**: Invalid parameters default to safe values (typically port 0 or CAN ID 1)

### Consistent APIs
- **Interface-Based**: Use `BaseGyro`, `BaseAbsoluteEncoder`, `BaseProximitySensor`, `BaseDistanceSensor` for polymorphism
- **Predictable Behavior**: Similar operations work the same way across different hardware
- **Problem-Domain Questions**: Sensors answer "what" questions, not "how" - expose state, not raw readings

### Hardware Abstraction
- **Swap Components Easily**: Change hardware without rewriting code
- **Vendor-Agnostic**: Works with CTRE, REV, and WPILib components
- **Modern Protocols**: Supports CAN, DIO, Analog, and PWM connections

## Sensor Types

### [Gyroscopes](gyros/README.md)
Measure robot orientation and rotation for navigation and balance.

**Supported Hardware:**
- CTRE Pigeon2 IMU (via CAN)

**Key Features:**
- Continuous yaw tracking (no wrapping)
- Pitch and roll measurement
- Zero and offset capabilities
- Rotation2d output for WPILib integration

**Quick Example:**
```java
BaseGyro gyro = new Gyro(1);  // CAN ID 1
double heading = gyro.getContinuousYawDeg();
gyro.resetYaw();  // Zero the heading
```

[📖 Gyroscopes Documentation →](gyros/README.md)

---

### [Encoders](encoders/README.md)
Measure rotational position for arms, turrets, and mechanisms.

**Supported Hardware:**
- CTRE CANcoder (CAN-based absolute encoder)
- REV Through Bore Encoder (DutyCycle absolute encoder)

**Key Features:**
- Absolute position (maintains position through power cycles)
- Multiple output formats (degrees, radians, Rotation2d)
- 8192 CPR (Through Bore) or 4096 CPR (CANcoder)
- No homing required on startup

**Quick Example:**
```java
BaseAbsoluteEncoder armEncoder = new ThroughBoreEncoder(5);  // DIO port 5
double angleDeg = armEncoder.getAbsolutePositionDegrees();
Rotation2d rotation = armEncoder.getAbsolutePositionRotation2D();
```

[📖 Encoders Documentation →](encoders/README.md)

---

### [Proximity Sensors](proximity/README.md)
Detect object presence for intakes, magazines, and safety systems.

**Supported Hardware:**
- Digital limit switches (mechanical contact switches)
- PhotoEye beam-break sensors (infrared optical sensors)

**Key Features:**
- Binary detection (object present/not present)
- Inverted mode support
- Debouncing recommended for reliability
- Fast response time

**Quick Example:**
```java
// Limit switch for mechanism homing
BaseProximitySensor bottomLimit = new LimitSwitch(0, false);
if (bottomLimit.isDetecting()) {
    motor.stopMotor();
}

// PhotoEye for game piece detection
BaseProximitySensor intakeSensor = new PhotoEye(3, false);
if (intakeSensor.isDetecting()) {
    intake.stop();
}
```

[📖 Proximity Sensors Documentation →](proximity/README.md)

---

### [Distance Sensors](distance/README.md)
Measure range to objects for positioning, collision avoidance, and alignment.

**Supported Hardware:**
- Analog ultrasonic sensors (10cm - 5m typical)
- LIDAR-Lite (PWM-based, 5cm - 40m)
- CTRE CANrange (CAN-based time-of-flight)

**Key Features:**
- Multiple distance units (centimeters, inches, feet)
- Different technologies for different ranges
- Varying accuracy and update rates
- Environmental considerations

**Quick Example:**
```java
// Ultrasonic for close-range detection
BaseDistanceSensor ultrasonic = new UltrasonicSensor(0);  // Analog port 0
double distanceCM = ultrasonic.getDistanceInCentimeters();

// LIDAR for long-range measurement
BaseDistanceSensor lidar = new Lidar(6);  // DIO port 6
double distanceInches = lidar.getDistanceInInches();
```

[📖 Distance Sensors Documentation →](distance/README.md)

---

## Common Patterns

### Interface-Based Design

Use base interfaces to allow hardware swapping:

```java
public class Turret {
    private final BaseAbsoluteEncoder encoder;
    private final BaseGyro gyro;

    public Turret(BaseAbsoluteEncoder encoder, BaseGyro gyro) {
        this.encoder = encoder;  // Works with CANCoder or ThroughBoreEncoder
        this.gyro = gyro;        // Works with any gyro implementation
    }

    public double getAngle() {
        return encoder.getAbsolutePositionDegrees();
    }

    public double getRobotHeading() {
        return gyro.getContinuousYawDeg();
    }
}

// In RobotContainer, choose hardware:
BaseAbsoluteEncoder encoder = new CANCoder(2);  // or ThroughBoreEncoder
BaseGyro gyro = new Gyro(1);
Turret turret = new Turret(encoder, gyro);
```

### Trigger-Based State Exposure

Expose sensor state as triggers for command-based programming:

```java
public class IntakeSubsystem extends SubsystemBase {
    private final BaseProximitySensor beamBreak;

    public IntakeSubsystem() {
        beamBreak = new PhotoEye(3, false);
    }

    // Expose state as problem-domain questions
    public final Trigger hasGamePiece =
        new Trigger(beamBreak::isDetecting)
            .debounce(0.1);  // Prevent flickering
}

// In RobotContainer:
m_intake.hasGamePiece
    .onTrue(m_intake.stopIntake())
    .onTrue(m_led.setPattern(LEDPattern.HAS_GAME_PIECE));
```

### Safe Construction

All constructors validate inputs and provide clear feedback:

```java
// Invalid CAN ID
BaseGyro gyro = new Gyro(100);
// DriverStation error: "Gyro: Invalid CAN ID 100. Valid range: 0-62. Defaulting to 1."
// Gyro uses CAN ID 1 and continues operating

// Invalid DIO port
BaseAbsoluteEncoder encoder = new ThroughBoreEncoder(15);
// DriverStation error: "ThroughBoreEncoder: Invalid DIO port 15. RoboRIO 2 has 10 DIO ports (0-9). Defaulting to 0."
// Encoder uses port 0 and continues operating
```

### Combining Sensors

Use multiple sensors for robust system behavior:

```java
public class Shooter {
    private final BaseGyro gyro;
    private final BaseDistanceSensor rangefinder;
    private final BaseProximitySensor magazineSensor;

    public final Trigger isAimed = new Trigger(() ->
        MathUtil.isNear(gyro.getContinuousYawDeg(), getTargetAngle(), 2.0)
    ).debounce(0.1);

    public final Trigger isInRange = new Trigger(() -> {
        double distance = rangefinder.getDistanceInInches();
        return distance > 60 && distance < 180;
    }).debounce(0.25);

    public final Trigger hasGamePiece =
        new Trigger(magazineSensor::isDetecting)
            .debounce(0.1);

    public final Trigger readyToShoot =
        isAimed.and(isInRange).and(hasGamePiece);
}
```

## Hardware Port Reference

**RoboRIO 2:**
- **DIO Ports**: 0-9 (10 total) - For ThroughBoreEncoder, LimitSwitch, PhotoEye, Lidar
- **Analog Ports**: 0-3 (4 total) - For UltrasonicSensor
- **CAN Bus**: IDs 0-62 - For Gyro (Pigeon2), CANCoder, CANRangeSensor

## Best Practices

### 1. Use Interfaces for Subsystems
```java
// Good: Flexible, testable
public class Arm {
    private final BaseAbsoluteEncoder encoder;
    private final BaseProximitySensor lowerLimit;
}

// Avoid: Tightly coupled to specific hardware
public class Arm {
    private final CANCoder encoder;  // Hard to test or swap hardware
}
```

### 2. Expose State as Triggers, Not Values
```java
// Good: Problem-domain question
public final Trigger atSetpoint = new Trigger(() ->
    MathUtil.isNear(encoder.getAbsolutePositionDegrees(), targetAngle, 2.0)
).debounce(0.1);

// Avoid: Exposing raw values
public double getEncoderPosition() {
    return encoder.getAbsolutePositionDegrees();
}
```

### 3. Use Debouncing for Digital Sensors
```java
// Prevent flickering and false triggers
public final Trigger hasGamePiece =
    new Trigger(photoEye::isDetecting)
        .debounce(0.1, Debouncer.DebounceType.kBoth);
```

### 4. Compare Doubles with MathUtil.isNear()
```java
// Good: Tolerant comparison
if (MathUtil.isNear(gyro.getContinuousYawDeg(), 90.0, 2.0)) {
    // Within 2 degrees of 90
}

// Avoid: Direct equality (will rarely be true)
if (gyro.getContinuousYawDeg() == 90.0) {
    // Rarely true due to floating point precision
}
```

### 5. Cache Expensive Sensor Reads
```java
public class ArmSubsystem extends SubsystemBase {
    private final BaseAbsoluteEncoder encoder;
    private double cachedPosition;

    @Override
    public void periodic() {
        // Read sensor once per iteration
        cachedPosition = encoder.getAbsolutePositionDegrees();
    }

    public final Trigger atSetpoint = new Trigger(() ->
        MathUtil.isNear(cachedPosition, targetAngle, 2.0)
    );
}
```

### 6. Validate Sensor Readings
```java
// Check for invalid or out-of-range readings
double distance = ultrasonic.getDistanceInCentimeters();
if (distance > 0 && distance < 500) {  // Valid range
    // Use the reading
} else {
    // Sensor error or out of range
    Logger.recordOutput("Ultrasonic/Error", true);
}
```

## Migration from WPILib

### Gyroscopes
```java
// WPILib (vendor-specific)
Pigeon2 gyro = new Pigeon2(1);
double yaw = gyro.getYaw().getValueAsDouble();

// AdambotsLib (unified interface with validation)
BaseGyro gyro = new Gyro(1);
double yaw = gyro.getContinuousYawDeg();  // No wrapping, validated
```

### Encoders
```java
// WPILib (vendor-specific)
DutyCycleEncoder encoder = new DutyCycleEncoder(5);
double position = encoder.get() * 360.0;

// AdambotsLib (with validation and units)
BaseAbsoluteEncoder encoder = new ThroughBoreEncoder(5);
double position = encoder.getAbsolutePositionDegrees();  // Direct units
```

### Proximity Sensors
```java
// WPILib
DigitalInput limitSwitch = new DigitalInput(0);
boolean triggered = limitSwitch.get();

// AdambotsLib (with validation and problem-domain interface)
BaseProximitySensor limitSwitch = new LimitSwitch(0, false);
boolean triggered = limitSwitch.isDetecting();  // Clearer semantics
```

### Distance Sensors
```java
// WPILib
AnalogInput ultrasonic = new AnalogInput(0);
double voltage = ultrasonic.getAverageVoltage();
double distance = voltage * scalingFactor;  // Manual conversion

// AdambotsLib (with validation and automatic conversion)
BaseDistanceSensor ultrasonic = new UltrasonicSensor(0);
double distance = ultrasonic.getDistanceInCentimeters();  // Direct units
```

## Troubleshooting

### "DriverStation: Invalid CAN ID"
**Cause**: Constructor received out-of-range CAN ID (valid: 0-62)
**Solution**: Check wiring and update CAN ID in code
**Result**: Sensor uses safe default CAN ID 1, continues operating

### "DriverStation: Invalid DIO port"
**Cause**: Constructor received out-of-range DIO port (valid: 0-9 on RoboRIO 2)
**Solution**: Check wiring and update port number in code
**Result**: Sensor uses port 0, continues operating

### "DriverStation: Invalid analog port"
**Cause**: Constructor received out-of-range analog port (valid: 0-3 on RoboRIO 2)
**Solution**: Check wiring and update port number in code
**Result**: Sensor uses port 0, continues operating

### Sensor Reading is Zero or Invalid
**Cause**: Sensor not connected, wiring issue, or sensor failure
**Solution**:
1. Check physical connections
2. Verify port/CAN ID matches hardware
3. Check sensor LED indicators (if present)
4. Test sensor with vendor tools (Phoenix Tuner, REV Hardware Client)

### Encoder Position Drifts or Jumps
**Cause**: Magnetic interference, loose connection, or encoder calibration
**Solution**:
1. For CANCoder: Verify magnet alignment and distance
2. For Through Bore: Check cable connections
3. Ensure no magnetic interference from motors
4. Recalibrate encoder if needed

### Proximity Sensor Flickering
**Cause**: Sensor at edge of detection range or electrical noise
**Solution**: Use debouncing on triggers
```java
public final Trigger hasGamePiece =
    new Trigger(photoEye::isDetecting)
        .debounce(0.1);  // 100ms debounce
```

### Distance Sensor Returns Maximum Value
**Cause**: Object out of range, sensor obstructed, or reflective surface issues
**Solution**:
- Ultrasonic: Works poorly with soft or angled surfaces
- LIDAR: Struggles with transparent or highly reflective surfaces
- Check sensor specifications for valid target materials

## Performance Considerations

### Sensor Update Rates

Different sensors have different update rates:

| Sensor | Update Rate | Notes |
|--------|-------------|-------|
| Pigeon2 Gyro | 200 Hz | Very fast, minimal latency |
| CANcoder | 100 Hz | Fast, CAN bus dependent |
| Through Bore | 1 kHz | Very fast, DIO based |
| Ultrasonic | ~50 Hz | Slower, speed of sound |
| LIDAR | Up to 100 Hz | Fast but can vary |
| PhotoEye/Limit | Digital | Immediate |

### CAN Bus Usage

Minimize CAN bus traffic for sensors:
- Cache frequently-read values in `periodic()`
- Use appropriate CAN frame periods in Phoenix Tuner
- Consider StatusFrame rates for CANcoders

```java
// Good: Cache in periodic
private double cachedYaw;

@Override
public void periodic() {
    cachedYaw = gyro.getContinuousYawDeg();  // Read once
}

public final Trigger atAngle =
    new Trigger(() -> MathUtil.isNear(cachedYaw, target, 2.0));
```

## See Also

- **[Gyroscopes Documentation](gyros/README.md)** - Complete gyroscope guide
- **[Encoders Documentation](encoders/README.md)** - Complete encoder guide
- **[Proximity Sensors Documentation](proximity/README.md)** - Complete proximity sensor guide
- **[Distance Sensors Documentation](distance/README.md)** - Complete distance sensor guide
- **[Main Documentation](../README.md)** - Library overview
- **[Command Best Practices](../../COMMAND_BEST_PRACTICES.md)** - Trigger-based programming patterns
