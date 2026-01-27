# CANRangeSensor

CTRE CANrange time-of-flight distance sensor for CAN bus-based ranging.

## Constructor

```java
public CANRangeSensor(int deviceId, boolean isOnCANivore)
```

Creates a CANrange sensor on the specified CAN ID.

**Parameters:**
- `deviceId` - CAN ID (0-62)
- `isOnCANivore` - True if on CANivore bus, false for RoboRIO CAN bus

**Validation:**
- CAN ID validated at construction
- Invalid IDs default to 1 with DriverStation error

**Example:**
```java
// On RoboRIO CAN bus
BaseDistanceSensor sensor = new CANRangeSensor(10, false);

// On CANivore bus
BaseDistanceSensor sensor = new CANRangeSensor(10, true);
```

## Methods

### getDistance()

```java
public Distance getDistance()
```

Returns distance as a WPILib `Distance` type.

**Returns:** Distance measurement

**Usage:**
```java
import static edu.wpi.first.units.Units.*;

Distance distance = sensor.getDistance();
double cm = distance.in(Centimeters);
double inches = distance.in(Inches);
double feet = distance.in(Feet);
double meters = distance.in(Meters);
```

## Simulation Support

```java
public void setSimulatedDistance(double distanceInMeters)
```

Sets simulated distance for testing (simulation mode only).

**Example:**
```java
if (sensor.isSim()) {
    sensor.setSimulatedDistance(1.5);  // 1.5 meters
}
```

## Hardware Details

**Range:** 4cm - 2m

**Accuracy:** ±2cm

**Connection:** CAN bus

**Wiring:**
- CAN High + CAN Low + Power via CAN

## See Also

- [Distance Sensors Overview](README.md)
- [UltrasonicSensor](UltrasonicSensor.md)
- [Lidar](Lidar.md)
- [CTRE CANrange Docs](https://pro.docs.ctr-electronics.com/en/latest/docs/hardware-reference/canrange/index.html)
