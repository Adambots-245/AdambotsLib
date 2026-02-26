# Distance Sensors

Distance sensors measure range to objects for positioning, collision avoidance, and alignment. Different technologies suit different ranges and accuracy requirements.

## Available Implementations

| Class | Hardware | Connection | Range | Accuracy | Best For |
|-------|----------|------------|-------|----------|----------|
| [UltrasonicSensor](UltrasonicSensor.md) | Analog ultrasonic | Analog (0-3) | 10cm-5m | ±1-3cm | Close range, soft surfaces |
| [Lidar](Lidar.md) | LIDAR-Lite | DIO (PWM) | 5cm-40m | ±2.5cm | Long range, precision |
| [CANRangeSensor](CANRangeSensor.md) | CTRE CANrange | CAN | 4cm-2m | ±2cm | CAN bus, moderate range |
| DummyDistanceSensor | None | N/A | N/A | N/A | No-op null object for disabled subsystems |

### Disabled Subsystems (DummyDistanceSensor)

When no distance sensor is physically present, use `DummyDistanceSensor` instead of `null`. Returns 9999 meters so proximity/range checks evaluate as "nothing detected":

```java
BaseDistanceSensor sensor = isSensorInstalled
    ? new UltrasonicSensor(0)
    : new DummyDistanceSensor();
```

## Quick Start

```java
import com.adambots.lib.sensors.*;
import static edu.wpi.first.units.Units.*;

// Ultrasonic on analog port 0
BaseDistanceSensor ultrasonic = new UltrasonicSensor(0);

// LIDAR on DIO port 6
BaseDistanceSensor lidar = new Lidar(6);

// CANrange on CAN ID 10
BaseDistanceSensor canRange = new CANRangeSensor(10, false);

// Get distance with WPILib units
Distance distance = ultrasonic.getDistance();
double distanceCM = distance.in(Centimeters);
double distanceInches = lidar.getDistance().in(Inches);
double distanceFeet = canRange.getDistance().in(Feet);
```

## Interface: BaseDistanceSensor

```java
import edu.wpi.first.units.measure.Distance;

public interface BaseDistanceSensor {
    Distance getDistance();
}
```

**Unit Conversion:**
```java
import static edu.wpi.first.units.Units.*;

Distance distance = sensor.getDistance();
double cm = distance.in(Centimeters);      // Convert to centimeters
double inches = distance.in(Inches);        // Convert to inches
double feet = distance.in(Feet);            // Convert to feet
double meters = distance.in(Meters);        // Convert to meters
```

## Common Patterns

### Collision Avoidance

```java
import static edu.wpi.first.units.Units.*;

public class DriveSubsystem extends SubsystemBase {
    private final BaseDistanceSensor m_frontSensor;

    public final Trigger obstacleDetected = new Trigger(() ->
        m_frontSensor.getDistance().in(Centimeters) < 30.0
    ).debounce(0.2);
}

// In RobotContainer
m_drive.obstacleDetected
    .whileTrue(m_drive.stop())
    .onTrue(m_led.setRed());
```

### Target Distance Check

```java
import static edu.wpi.first.units.Units.*;

public class ShooterSubsystem extends SubsystemBase {
    private final BaseDistanceSensor m_rangefinder;

    public final Trigger inRange = new Trigger(() -> {
        double distance = m_rangefinder.getDistance().in(Inches);
        return distance > 60 && distance < 180;  // 5-15 feet
    }).debounce(0.25);
}

// Only shoot when in range
m_shooter.isReady
    .and(m_shooter.inRange)
    .and(m_intake.hasGamePiece)
    .whileTrue(m_feeder.feed());
```

### Positioning

```java
import static edu.wpi.first.units.Units.*;

public class IntakeSubsystem extends SubsystemBase {
    private final BaseDistanceSensor m_lidar;

    public Command driveToWall() {
        return run(() -> {
            double distance = m_lidar.getDistance().in(Centimeters);
            double targetDistance = 50.0;  // 50cm from wall

            if (distance > targetDistance + 5) {
                m_drive.drive(0.3, 0, 0);  // Move forward
            } else {
                m_drive.stopMotor();
            }
        });
    }
}
```

## Best Practices

### 1. Validate Sensor Readings

```java
import static edu.wpi.first.units.Units.*;

// Check for valid range
double distance = sensor.getDistance().in(Centimeters);
if (distance > 0 && distance < 500) {  // Valid range
    // Use the reading
} else {
    // Sensor error or out of range
    Logger.recordOutput("Sensor/OutOfRange", true);
}
```

### 2. Use Debouncing

```java
import static edu.wpi.first.units.Units.*;

public final Trigger inRange = new Trigger(() -> {
    double dist = m_sensor.getDistance().in(Centimeters);
    return dist > 20 && dist < 100;
}).debounce(0.25);  // Prevent flickering
```

### 3. Consider Surface Material

```java
// Ultrasonic works poorly on:
// - Soft materials (foam, fabric)
// - Angled surfaces
// - Sound-absorbing materials

// LIDAR works poorly on:
// - Transparent surfaces (glass, clear plastic)
// - Highly reflective surfaces (mirrors)
// - Very dark surfaces
```

### 4. Cache Readings in Periodic

```java
import static edu.wpi.first.units.Units.*;

private double m_cachedDistance;

@Override
public void periodic() {
    m_cachedDistance = m_sensor.getDistance().in(Centimeters);
}

public final Trigger inRange = new Trigger(() ->
    m_cachedDistance > 20 && m_cachedDistance < 100
);
```

## Hardware Comparison

### Ultrasonic Sensors

**Technology:** Sound waves (40 kHz typical)

**Pros:**
- Works in any lighting
- Good with soft surfaces
- Inexpensive
- Wide beam angle

**Cons:**
- Slow update rate (~50 Hz)
- Poor with angled surfaces
- Affected by temperature/humidity
- Limited range

**Best For:** Close-range detection, ground distance

### LIDAR

**Technology:** Laser time-of-flight

**Pros:**
- Long range (up to 40m)
- Fast update rate (up to 100 Hz)
- Accurate (±2.5cm)
- Narrow beam

**Cons:**
- Expensive
- Struggles with transparent/reflective surfaces
- Affected by bright sunlight
- Requires PWM input

**Best For:** Long-range measurement, precision positioning

### CANrange

**Technology:** Time-of-flight (ToF)

**Pros:**
- CAN bus (long cables OK)
- Good accuracy
- Phoenix Tuner configuration
- Moderate range

**Cons:**
- Limited to 2m
- Requires CAN ID
- Uses CAN bandwidth

**Best For:** Moderate distances, CAN bus systems

## Troubleshooting

### Sensor Always Returns Zero

**Causes:**
- Not connected
- Wiring error
- Power issue

**Solutions:**
1. Check connections
2. Verify port/CAN ID
3. Test with vendor tools

---

### Reading Returns Maximum Value

**Causes:**
- Object out of range
- Wrong surface material
- Sensor obstructed

**Solutions:**
1. Move closer to target
2. Change target material
3. Clean sensor lens
4. Check for obstructions

---

### Unstable/Flickering Readings

**Causes:**
- Target at edge of range
- Moving target
- Electrical noise

**Solutions:**
1. Use debouncing
2. Add filtering
3. Average multiple readings

```java
import static edu.wpi.first.units.Units.*;

private double[] readings = new double[5];
private int index = 0;

public double getFilteredDistance() {
    readings[index] = m_sensor.getDistance().in(Centimeters);
    index = (index + 1) % readings.length;

    // Return average
    return Arrays.stream(readings).average().orElse(0);
}
```

## See Also

- [UltrasonicSensor](UltrasonicSensor.md)
- [Lidar](Lidar.md)
- [CANRangeSensor](CANRangeSensor.md)
- [Sensors Overview](../README.md)
