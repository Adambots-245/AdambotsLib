# AngularHubServo

Angular position servo control via REV ServoHub. Supports 0-355° range for Axon Max+ and 0-180° for standard servos.

## Constructor

```java
public AngularHubServo(ServoHub hub, int servoPortNum, double maxAngle)
```

**Parameters:**
- `hub` - ServoHub instance
- `servoPortNum` - Port 0-5 on ServoHub
- `maxAngle` - Maximum angle (355.0 for Axon Max+, 180.0 for standard)

**Validation:** Port range checked, angles clamped, warnings logged.

## Quick Example

```java
ServoHub hub = new ServoHub(10);
BaseServo servo = new AngularHubServo(hub, 0, 355.0);
servo.setAngle(90.0);  // Move to 90 degrees
servo.setAngle(270.0); // Move to 270 degrees
```

## Hardware

- **ServoHub Ports**: 0-5
- **Pulse Width**: 500-2500µs
- **Power**: 6V regulated from ServoHub
- **Current**: Monitored per-channel
- **Best For**: Axon Max+, high-resolution positioning

## Common Uses

Camera pan/tilt, claw positioning, arm joints, deployment mechanisms.

## See Also
- [BaseServo](BaseServo.md) | [CRHubServo](CRHubServo.md) | [DirectServo](DirectServo.md) | [Overview](README.md)
