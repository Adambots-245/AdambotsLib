# CRHubServo

Continuous rotation servo control via REV ServoHub. Variable speed from -1.0 (full CCW) to 1.0 (full CW).

## Constructor

```java
public CRHubServo(ServoHub hub, int servoPortNum)
```

**Parameters:**
- `hub` - ServoHub instance
- `servoPortNum` - Port 0-5 on ServoHub

**Validation:** Port range checked, speed clamped, warnings logged.

## Quick Example

```java
ServoHub hub = new ServoHub(10);
BaseServo roller = new CRHubServo(hub, 1);
roller.set(0.8);   // 80% speed clockwise
roller.set(-0.8);  // 80% speed counterclockwise
roller.stop();     // Stop rotation
```

## Hardware

- **ServoHub Ports**: 0-5
- **Speed Range**: -1.0 to 1.0
- **Power**: 6V regulated from ServoHub
- **Current**: Monitored per-channel
- **Best For**: Intake rollers, ball feeders, continuous rotation needs

## Common Uses

Intake rollers, ball feeders, agitators, spinners, conveyor belts.

## See Also
- [BaseServo](BaseServo.md) | [AngularHubServo](AngularHubServo.md) | [DirectServo](DirectServo.md) | [Overview](README.md)
