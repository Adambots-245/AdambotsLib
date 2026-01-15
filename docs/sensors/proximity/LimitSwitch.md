# LimitSwitch

Digital limit switch sensor for mechanical contact detection.

## Constructor

```java
public LimitSwitch(int port, boolean inverted)
```

Creates a limit switch on the specified DIO port.

**Parameters:**
- `port` - DIO port (0-9 on RoboRIO 2)
- `inverted` - True for normally-closed (NC), false for normally-open (NO)

**Validation:**
- DIO port validated at construction
- Invalid ports default to 0 with DriverStation error

**Example:**
```java
// Normally-open limit switch
BaseProximitySensor bottomLimit = new LimitSwitch(0, false);

// Normally-closed (fail-safe)
BaseProximitySensor topLimit = new LimitSwitch(1, true);
```

## Methods

### isDetecting()

```java
public boolean isDetecting()
```

Returns true if switch is triggered.

**Returns:** True when limit is triggered

**Example:**
```java
if (limit.isDetecting()) {
    motor.stopMotor();
}
```

## Hardware Details

**Connection:** RoboRIO DIO ports (0-9)

**Wiring:**
- Signal wire to DIO port
- Ground to GND
- Pull-up resistor typically required for NO switches

**Types:**
- **Normally-Open (NO)**: Closed when pressed, use `inverted=false`
- **Normally-Closed (NC)**: Open when pressed, use `inverted=true`

## Usage Patterns

### Homing Sequence

```java
public Command home() {
    return run(() -> m_motor.set(-0.2))
        .until(m_bottomLimit::isDetecting)
        .andThen(runOnce(() -> m_encoder.reset()));
}
```

### Safety Stop

```java
public Command extend() {
    return run(() -> {
        if (!m_upperLimit.isDetecting()) {
            m_motor.set(0.5);
        } else {
            m_motor.stopMotor();
        }
    });
}
```

## See Also

- [Proximity Sensors Overview](README.md)
- [PhotoEye](PhotoEye.md)
