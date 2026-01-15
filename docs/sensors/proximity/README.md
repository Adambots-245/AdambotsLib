# Proximity Sensors

Proximity sensors detect object presence with binary (yes/no) feedback. Use them for game piece detection, homing mechanisms, and safety interlocks.

## Available Implementations

| Class | Hardware | Connection | Range | Response Time |
|-------|----------|------------|-------|---------------|
| [LimitSwitch](LimitSwitch.md) | Mechanical switch | DIO | Contact only | Instant |
| [PhotoEye](PhotoEye.md) | Infrared beam-break | DIO | 0-30cm typical | <1ms |

## Quick Start

```java
import com.adambots.lib.sensors.*;

// Limit switch on DIO port 0
BaseProximitySensor bottomLimit = new LimitSwitch(0, false);

// PhotoEye on DIO port 3
BaseProximitySensor intakeSensor = new PhotoEye(3, false);

// Check detection
if (intakeSensor.isDetecting()) {
    System.out.println("Game piece detected!");
}
```

## Interface: BaseProximitySensor

```java
public interface BaseProximitySensor {
    boolean isDetecting();
}
```

## Common Patterns

### Game Piece Detection

```java
public class IntakeSubsystem extends SubsystemBase {
    private final BaseProximitySensor m_beamBreak;

    public IntakeSubsystem() {
        m_beamBreak = new PhotoEye(3, false);
    }

    // Expose as trigger with debouncing
    public final Trigger hasGamePiece =
        new Trigger(m_beamBreak::isDetecting)
            .debounce(0.1);
}

// In RobotContainer
m_intake.hasGamePiece
    .onTrue(m_intake.stopIntake())
    .onTrue(m_led.setGreen());
```

### Mechanism Homing

```java
public class ElevatorSubsystem extends SubsystemBase {
    private final BaseProximitySensor m_bottomLimit;

    public final Trigger atBottom =
        new Trigger(m_bottomLimit::isDetecting);

    public Command home() {
        return run(() -> m_motor.set(-0.2))  // Move down slowly
            .until(atBottom)
            .andThen(runOnce(() -> m_encoder.reset()));
    }
}
```

### Safety Interlock

```java
public class ClimberSubsystem extends SubsystemBase {
    private final BaseProximitySensor m_upperLimit;
    private final BaseProximitySensor m_lowerLimit;

    public final Trigger atUpperLimit =
        new Trigger(m_upperLimit::isDetecting);

    public final Trigger atLowerLimit =
        new Trigger(m_lowerLimit::isDetecting);

    public Command extend(DoubleSupplier speed) {
        return run(() -> {
            double requestedSpeed = speed.getAsDouble();

            // Stop at limits
            if (requestedSpeed > 0 && atUpperLimit.getAsBoolean()) {
                m_motor.stopMotor();
            } else if (requestedSpeed < 0 && atLowerLimit.getAsBoolean()) {
                m_motor.stopMotor();
            } else {
                m_motor.set(requestedSpeed);
            }
        });
    }
}
```

## Best Practices

### 1. Always Use Debouncing

```java
// Good: Prevents flickering
public final Trigger hasGamePiece =
    new Trigger(photoEye::isDetecting)
        .debounce(0.1);

// Avoid: Can flicker on/off rapidly
public final Trigger hasGamePiece =
    new Trigger(photoEye::isDetecting);
```

### 2. Consider Inverted Mode

```java
// Normally-closed switch (safer - detects wire breaks)
BaseProximitySensor limit = new LimitSwitch(0, true);  // inverted

// Normally-open switch
BaseProximitySensor limit = new LimitSwitch(0, false);
```

### 3. Combine Multiple Sensors

```java
// Wait for both intake and magazine sensors
m_intake.hasGamePiece
    .and(m_magazine.hasGamePiece)
    .onTrue(m_intake.stopAll());
```

## Hardware Comparison

### Limit Switches

**Pros:**
- Extremely reliable
- No power required
- Works in any lighting
- Very cheap

**Cons:**
- Requires physical contact
- Mechanical wear over time
- Can bounce (needs debouncing)

**Best For:** Homing, hard stops, safety limits

### PhotoEyes

**Pros:**
- No contact required
- Fast response
- Long life (no wear)
- Adjustable sensitivity

**Cons:**
- Requires power
- Affected by ambient light
- Can be fooled by transparent objects
- More expensive

**Best For:** Game piece detection, counting, non-contact sensing

## Troubleshooting

### Sensor Always Reads True/False

**Causes:**
- Wiring error (reversed or disconnected)
- Incorrect inverted setting
- Sensor failure

**Solutions:**
1. Check DIO port matches code
2. Verify wiring connections
3. Try toggling inverted parameter
4. Test sensor with different port

---

### Flickering Detection

**Cause:** Object at edge of range or electrical noise

**Solution:** Use debouncing
```java
public final Trigger hasGamePiece =
    new Trigger(photoEye::isDetecting)
        .debounce(0.1);  // 100ms debounce
```

---

### PhotoEye Not Detecting

**Causes:**
- Beam not broken
- Object is transparent
- Sensor dirty or misaligned
- Bright ambient light

**Solutions:**
1. Check LED indicators on sensor
2. Clean sensor lens
3. Adjust sensor alignment
4. Shield from bright lights

## See Also

- [LimitSwitch](LimitSwitch.md)
- [PhotoEye](PhotoEye.md)
- [Sensors Overview](../README.md)
