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

### getAbsolutePositionDegrees()

```java
public double getAbsolutePositionDegrees()
```

Returns absolute position in degrees (0-360°).

**Returns:** Position in degrees

**Example:**
```java
double angle = encoder.getAbsolutePositionDegrees();
```

---

### getAbsolutePositionRadians()

```java
public double getAbsolutePositionRadians()
```

Returns absolute position in radians (0-2π).

**Returns:** Position in radians

---

### getAbsolutePositionRotation2D()

```java
public Rotation2d getAbsolutePositionRotation2D()
```

Returns absolute position as Rotation2d for WPILib integration.

**Returns:** Rotation2d object

**Example:**
```java
// For swerve module
SwerveModuleState state = new SwerveModuleState(
    speed,
    encoder.getAbsolutePositionRotation2D()
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
