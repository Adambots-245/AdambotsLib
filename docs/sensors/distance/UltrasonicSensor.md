# UltrasonicSensor

Analog ultrasonic distance sensor using sound waves for range measurement.

## Constructor

```java
public UltrasonicSensor(int portNumber)
```

Creates an ultrasonic sensor on the specified analog port.

**Parameters:**
- `portNumber` - Analog port (0-3 on RoboRIO 2)

**Validation:**
- Analog port validated at construction
- Invalid ports default to 0 with DriverStation error

**Example:**
```java
BaseDistanceSensor sensor = new UltrasonicSensor(0);  // Analog port 0
```

## Methods

### getDistance()

```java
public Distance getDistance()
```

Returns the distance measurement as a WPILib `Distance` type.

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

## Hardware Details

**Range:** Typically 10cm - 5m (model dependent)

**Accuracy:** ±1-3cm

**Update Rate:** ~50 Hz

**Wiring:**
- Power (5V or 12V depending on model)
- Ground
- Signal to analog port

## Configuration

The sensor uses oversampling and averaging for noise reduction:
- Oversampling: 4x (2^2)
- Averaging: 32 samples (2^5)

This balances response time with accuracy.

## See Also

- [Distance Sensors Overview](README.md)
- [Lidar](Lidar.md)
- [CANRangeSensor](CANRangeSensor.md)
