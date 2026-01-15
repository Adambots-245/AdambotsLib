# DirectServo

Direct PWM servo control via RoboRIO. Supports both angular and continuous rotation modes.

## Constructors

```java
public DirectServo(int channel, ServoMode mode)
public DirectServo(int channel, ServoMode mode, double minAngle, double maxAngle)
```

**Parameters:**
- `channel` - PWM channel 0-19 on RoboRIO 2
- `mode` - `ServoMode.ANGULAR` or `ServoMode.CONTINUOUS_ROTATION`
- `minAngle` - Minimum angle (default 0.0)
- `maxAngle` - Maximum angle (default 180.0)

**Validation:** Channel range checked, angles/speeds clamped, warnings logged.

## Quick Examples

```java
// Angular servo
BaseServo claw = new DirectServo(0, ServoMode.ANGULAR);
claw.setAngle(90.0);

// Continuous rotation
BaseServo intake = new DirectServo(1, ServoMode.CONTINUOUS_ROTATION);
intake.set(0.5);

// Custom angle range
BaseServo limited = new DirectServo(2, ServoMode.ANGULAR, 45.0, 135.0);
```

## Hardware

- **PWM Ports**: 0-19 (RoboRIO 2)
- **Pulse Width**: 600-2400µs
- **Power**: Via external servo power module
- **Current**: No monitoring
- **Best For**: Simple setups, single servos, standard RC servos

## Common Uses

Single servos, standalone mechanisms, prototyping, standard RC servos.

## See Also
- [BaseServo](BaseServo.md) | [AngularHubServo](AngularHubServo.md) | [CRHubServo](CRHubServo.md) | [Overview](README.md)
