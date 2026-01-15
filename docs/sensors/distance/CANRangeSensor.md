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

### getDistanceInCentimeters()

```java
public double getDistanceInCentimeters()
```

Returns distance in centimeters (converted from meters).

**Returns:** Distance in cm

---

### getDistanceInInches()

```java
public double getDistanceInInches()
```

Returns distance in inches (converted from meters).

**Returns:** Distance in inches

---

### getDistanceInFeet()

```java
public double getDistanceInFeet()
```

Returns distance in feet (converted from meters).

**Returns:** Distance in feet

---

### getRawDistance()

```java
public double getRawDistance()
```

Returns raw distance in meters.

**Returns:** Distance in meters

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
