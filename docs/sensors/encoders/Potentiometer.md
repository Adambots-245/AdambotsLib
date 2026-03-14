# Potentiometer

Analog potentiometer as an absolute position sensor via the RoboRIO's analog input ports. Supports single-turn (360°) and multi-turn (e.g., 10-turn, 3600°) potentiometers.

## Constructor

```java
public Potentiometer(int port, double fullRange, double offset)
public Potentiometer(int port, double fullRange)  // offset defaults to 0
```

Creates a potentiometer on the specified analog input port.

**Parameters:**
- `port` - Analog input port (0-7 on RoboRIO 2; 0-3 onboard, 4-7 on MXP)
- `fullRange` - Full range of the potentiometer in degrees (e.g., 3600 for 10-turn, 360 for single-turn)
- `offset` - Offset in degrees added to the reading (default: 0)

**Validation:**
- Analog port validated at construction
- Invalid ports default to 0 with DriverStation error

**Examples:**
```java
// 10-turn pot on analog port 0, full range 3600°
BaseAbsoluteEncoder pot = new Potentiometer(0, 3600);

// Single-turn pot on port 1 with 45° offset
BaseAbsoluteEncoder pot = new Potentiometer(1, 360, 45);
```

## Methods

### getPosition()

```java
public Angle getPosition()
```

Returns position as a WPILib `Angle` type. For multi-turn pots, the value does **not** wrap at 360°.

**Returns:** Position as Angle

**Example:**
```java
import static edu.wpi.first.units.Units.*;

Angle position = pot.getPosition();
double degrees = position.in(Degrees);  // e.g., 0.0 to 3600.0 for 10-turn
double radians = position.in(Radians);
```

---

### getPositionRotation2d()

```java
public Rotation2d getPositionRotation2d()
```

Returns position as Rotation2d for WPILib integration.

**Returns:** Rotation2d object

## Hardware Details

**Specifications:**
- Resolution: 12-bit (4096 steps over the full range)
- Voltage range: 0-5V
- Resolution per step: fullRange / 4096 (e.g., ~0.88° per step for a 10-turn pot at 3600°)
- Connection: Analog input port

**Wiring:**
- Red: 5V from RoboRIO
- Black: Ground
- White/Signal: To analog input port

**Multi-turn vs Single-turn:**
- **Single-turn (360°):** One rotation covers full range. Good for mechanisms with limited rotation.
- **Multi-turn (e.g., 10-turn, 3600°):** Multiple rotations cover full range. Higher effective resolution per degree of mechanism travel. Position is linear across the full range — no wrapping.

## See Also

- [Encoders Overview](README.md)
- [WPILib AnalogPotentiometer Docs](https://docs.wpilib.org/en/stable/docs/software/hardware-apis/sensors/analog-potentiometers-software.html)
