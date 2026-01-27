# ThroughBoreEncoder

REV Through Bore Encoder absolute optical encoder using DutyCycle protocol.

## Constructor

```java
public ThroughBoreEncoder(int encoderPort)
```

Creates a Through Bore Encoder on the specified DIO port.

**Parameters:**
- `encoderPort` - DIO port (0-9 on RoboRIO 2)

**Validation:**
- DIO port validated at construction
- Invalid ports default to 0 with DriverStation error

**Example:**
```java
BaseAbsoluteEncoder encoder = new ThroughBoreEncoder(5);  // DIO port 5
```

## Methods

### getPosition()

```java
public Angle getPosition()
```

Returns absolute position as a WPILib `Angle` type (0-360°).

**Returns:** Position as Angle

**Example:**
```java
import static edu.wpi.first.units.Units.*;

Angle position = encoder.getPosition();
double degrees = position.in(Degrees);  // 0.0 to 360.0
double radians = position.in(Radians);  // 0.0 to 2π
```

---

### getPositionRotation2d()

```java
public Rotation2d getPositionRotation2d()
```

Returns absolute position as Rotation2d for WPILib integration.

**Returns:** Rotation2d object

**Example:**
```java
// For swerve module
SwerveModuleState state = new SwerveModuleState(
    speed,
    encoder.getPositionRotation2d()
);
```

## Hardware Details

**Specifications:**
- Resolution: 8192 CPR (13-bit absolute)
- Update rate: Up to 1 kHz
- Connection: DutyCycle via DIO port
- Through-hole diameter: 0.25" (6.35mm)

**Wiring:**
- Red: 5V from RoboRIO
- Black: Ground
- White: Signal to DIO port

**Mounting:**
- Mount encoder on shaft
- Shaft passes through center hole
- Use set screw to secure

## See Also

- [Encoders Overview](README.md)
- [REV Through Bore Docs](https://docs.revrobotics.com/through-bore-encoder/)
