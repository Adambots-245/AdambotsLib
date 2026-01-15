# PhotoEye

Infrared beam-break sensor for non-contact object detection.

## Constructor

```java
public PhotoEye(int port, boolean inverted)
```

Creates a photo-eye sensor on the specified DIO port.

**Parameters:**
- `port` - DIO port (0-9 on RoboRIO 2)
- `inverted` - True to invert logic (HIGH when detecting)

**Validation:**
- DIO port validated at construction
- Invalid ports default to 0 with DriverStation error

**Example:**
```java
// Normal wiring (LOW when detecting)
BaseProximitySensor intakeSensor = new PhotoEye(3, false);

// Inverted wiring (fail-safe mode)
BaseProximitySensor shooterSensor = new PhotoEye(4, true);
```

## Methods

### isDetecting()

```java
public boolean isDetecting()
```

Returns true when object breaks the beam.

**Returns:** True when beam is broken

**Example:**
```java
if (photoEye.isDetecting()) {
    System.out.println("Game piece detected!");
}
```

## Hardware Details

**Connection:** RoboRIO DIO ports (0-9)

**Range:** Typically 0-30cm depending on model

**Response Time:** < 1ms

**Wiring:**
- Power (typically 5V or 12V)
- Ground
- Signal to DIO port

## Usage Patterns

### Game Piece Detection

```java
public class IntakeSubsystem extends SubsystemBase {
    private final BaseProximitySensor m_photoEye;

    public IntakeSubsystem() {
        m_photoEye = new PhotoEye(3, false);
    }

    public final Trigger hasGamePiece =
        new Trigger(m_photoEye::isDetecting)
            .debounce(0.1);
}

// In RobotContainer
m_intake.hasGamePiece
    .onTrue(m_intake.stopIntake())
    .onTrue(m_led.setPattern(LEDPattern.HAS_GAME_PIECE));
```

### Auto-Feed When Ready

```java
// In RobotContainer
m_shooter.isReady
    .and(m_magazine.hasGamePiece)
    .and(m_vision.canSeeTarget)
    .whileTrue(m_feeder.feed());
```

## Troubleshooting

### Not Detecting Objects

1. Check LED indicators on sensor
2. Verify beam alignment
3. Clean sensor lens
4. Check for bright ambient light interference

### False Triggers

1. Add debouncing (0.05-0.1 seconds)
2. Shield from ambient light
3. Adjust sensitivity (if available)

## See Also

- [Proximity Sensors Overview](README.md)
- [LimitSwitch](LimitSwitch.md)
