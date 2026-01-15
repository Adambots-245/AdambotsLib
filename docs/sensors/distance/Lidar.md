# Lidar

LIDAR-Lite distance sensor using laser time-of-flight for precision ranging.

## Constructor

```java
public Lidar(int dioPortNumber)
```

Creates a LIDAR sensor on the specified DIO port (PWM mode).

**Parameters:**
- `dioPortNumber` - DIO port (0-9 on RoboRIO 2)

**Validation:**
- DIO port validated at construction
- Invalid ports default to 0 with DriverStation error

**Example:**
```java
BaseDistanceSensor lidar = new Lidar(6);  // DIO port 6
```

## Methods

### getDistanceInCentimeters()

```java
public double getDistanceInCentimeters()
```

Returns distance in centimeters based on PWM pulse width.

**Returns:** Distance in cm (typically 5-4000cm)

---

### getDistanceInInches()

```java
public double getDistanceInInches()
```

Returns distance in inches.

**Returns:** Distance in inches

---

### getDistanceInFeet()

```java
public double getDistanceInFeet()
```

Returns distance in feet.

**Returns:** Distance in feet

## Hardware Details

**Range:** 5cm - 40m (model dependent)

**Accuracy:** ±2.5cm typical

**Update Rate:** Up to 100 Hz (configurable)

**Measurement Method:** Laser time-of-flight via PWM (10 µs per cm)

**Wiring:**
- Red: 5V from RoboRIO
- Black: Ground
- Yellow/Orange: Signal to DIO port
- Blue: Mode control (leave disconnected for PWM mode)

## Performance Notes

**Works Best On:**
- Non-reflective surfaces
- Solid objects
- Clear line of sight

**Struggles With:**
- Transparent surfaces (glass, clear plastic)
- Highly reflective surfaces (mirrors)
- Bright sunlight (outdoor use)

## See Also

- [Distance Sensors Overview](README.md)
- [UltrasonicSensor](UltrasonicSensor.md)
- [CANRangeSensor](CANRangeSensor.md)
