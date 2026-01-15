# CANCoder

CTRE CANcoder absolute magnetic encoder for CAN bus-based position feedback.

## Constructor

```java
public CANCoder(int port)
```

Creates a CANcoder on the specified CAN ID.

**Parameters:**
- `port` - CAN ID (0-62)

**Validation:**
- CAN ID validated at construction
- Invalid IDs default to 1 with DriverStation error

**Example:**
```java
BaseAbsoluteEncoder encoder = new CANCoder(2);  // CAN ID 2
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
SwerveModulePosition position = new SwerveModulePosition(
    driveDistance,
    encoder.getAbsolutePositionRotation2D()
);
```

## Hardware Details

**Specifications:**
- Resolution: 4096 CPR (12-bit)
- Update rate: Up to 100 Hz
- Accuracy: ±0.088°
- Magnet distance: 3-6mm recommended
- Connection: CAN bus

**Wiring:**
- CAN High (yellow/green) + CAN Low (yellow/black) + Power via CAN

## See Also

- [Encoders Overview](README.md)
- [CTRE CANcoder Docs](https://pro.docs.ctr-electronics.com/en/latest/docs/hardware-reference/cancoder/index.html)
