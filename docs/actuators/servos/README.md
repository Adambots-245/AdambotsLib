# Servos

Servos control precise angular positions or continuous rotation. AdambotsLib provides a unified interface for REV ServoHub servos and direct PWM servos with built-in safety and validation.

## Quick Start

```java
import com.adambots.lib.actuators.*;
import com.revrobotics.servohub.ServoHub;

// Angular servo (position control)
ServoHub hub = new ServoHub(10);
BaseServo angularServo = new AngularHubServo(hub, 0, 355.0);
angularServo.setAngle(90.0);  // Move to 90 degrees

// Continuous rotation servo (speed control)
BaseServo crServo = new CRHubServo(hub, 1);
crServo.set(0.5);  // Half speed clockwise

// Direct PWM servo
BaseServo pwmServo = new DirectServo(0, ServoMode.ANGULAR);
pwmServo.setAngle(180.0);  // Standard RC servo
```

## Available Implementations

| Class | Hardware | Control Type | Range |
|-------|----------|--------------|-------|
| **[AngularHubServo](AngularHubServo.md)** | REV ServoHub | Position | 0-355° (Axon Max+) |
| **[CRHubServo](CRHubServo.md)** | REV ServoHub | Speed | -1.0 to 1.0 |
| **[DirectServo](DirectServo.md)** | PWM (RoboRIO) | Position or Speed | Configurable |

## Interface: BaseServo

All servo classes implement the `BaseServo` interface:

```java
public interface BaseServo extends BaseActuator {
    enum ServoMode { CONTINUOUS_ROTATION, ANGULAR }
    
    ServoMode getMode();
    void setAngle(double degrees);        // Angular mode only
    void set(double speed);               // CR mode only
    void turnCounterclockwise();
    void turnClockwise();
    void stop();
    void setPulseWidth(int pulseWidth);
    double getCurrent();
    
    // Capability detection
    boolean supportsAngleControl();
    boolean supportsSpeedControl();
    String getServoType();
}
```

**[📖 BaseServo API Reference](BaseServo.md)**

## Choosing the Right Servo

### Angular Servos (AngularHubServo, DirectServo ANGULAR)

**Use when:**
- Need precise position control
- Controlling mechanisms with defined positions
- Limited range of motion required

**Typical Applications:**
- Camera pan/tilt
- Claw positioning
- Arm joints
- Flippers
- Deployment mechanisms with stops

**Example:**
```java
BaseServo cameraServo = new AngularHubServo(hub, 0, 355.0);
cameraServo.setAngle(45.0);   // Point camera at 45°
cameraServo.setAngle(270.0);  // Point camera at 270°
```

### Continuous Rotation Servos (CRHubServo, DirectServo CR)

**Use when:**
- Need continuous rotation
- Variable speed control required
- Direction reversal needed

**Typical Applications:**
- Intake rollers
- Ball feeders
- Conveyor belts
- Agitators
- Spinners

**Example:**
```java
BaseServo intakeRoller = new CRHubServo(hub, 1);
intakeRoller.set(1.0);   // Full speed intake
intakeRoller.set(-1.0);  // Full speed eject
intakeRoller.set(0.0);   // Stop
```

### REV ServoHub vs Direct PWM

| Feature | REV ServoHub | Direct PWM |
|---------|-------------|------------|
| **Connection** | CAN → ServoHub → Servo | Direct to RoboRIO |
| **Ports Available** | 6 per hub | 10 PWM ports |
| **Current Monitoring** | Yes | No |
| **Cable Length** | Longer (CAN) | Shorter (PWM) |
| **Cost** | Higher (hub required) | Lower |
| **Best For** | Multiple servos, current monitoring | Single servos, simple setups |

## Common Patterns

### Camera Gimbal

```java
public class CameraGimbal {
    private final BaseServo panServo;
    private final BaseServo tiltServo;
    
    public CameraGimbal(ServoHub hub) {
        panServo = new AngularHubServo(hub, 0, 355.0);
        tiltServo = new AngularHubServo(hub, 1, 180.0);
    }
    
    public void setPan(double degrees) {
        if (panServo.supportsAngleControl()) {
            panServo.setAngle(degrees);
        }
    }
    
    public void setTilt(double degrees) {
        if (tiltServo.supportsAngleControl()) {
            tiltServo.setAngle(degrees);
        }
    }
    
    public void pointForward() {
        setPan(180.0);
        setTilt(90.0);
    }
    
    public void pointBackward() {
        setPan(0.0);
        setTilt(90.0);
    }
}
```

### Continuous Rotation Intake

```java
public class IntakeRoller {
    private final BaseServo rollerServo;
    private static final double INTAKE_SPEED = 0.8;
    private static final double EJECT_SPEED = -0.8;
    
    public IntakeRoller(ServoHub hub, int port) {
        rollerServo = new CRHubServo(hub, port);
    }
    
    public void intake() {
        if (rollerServo.supportsSpeedControl()) {
            rollerServo.set(INTAKE_SPEED);
        }
    }
    
    public void eject() {
        if (rollerServo.supportsSpeedControl()) {
            rollerServo.set(EJECT_SPEED);
        }
    }
    
    public void stop() {
        rollerServo.stop();
    }
    
    public Command intakeCommand() {
        return Commands.run(this::intake);
    }
    
    public Command ejectCommand() {
        return Commands.run(this::eject);
    }
}
```

### Mixed Servo System

```java
public class Manipulator {
    private final BaseServo wristServo;      // Angular
    private final BaseServo intakeServo;     // CR
    private final BaseServo clawServo;       // Angular
    
    public Manipulator(ServoHub hub) {
        wristServo = new AngularHubServo(hub, 0, 270.0);
        intakeServo = new CRHubServo(hub, 1);
        clawServo = new AngularHubServo(hub, 2, 90.0);
    }
    
    public void setWristAngle(double degrees) {
        wristServo.setAngle(degrees);
    }
    
    public void runIntake(double speed) {
        intakeServo.set(speed);
    }
    
    public void openClaw() {
        clawServo.setAngle(90.0);  // Fully open
    }
    
    public void closeClaw() {
        clawServo.setAngle(0.0);   // Fully closed
    }
}
```

## Safety Features

All servo implementations include:

### Mode-Based Safety
```java
// Calling wrong method for mode doesn't crash - logs warning instead
BaseServo angular = new AngularHubServo(hub, 0, 355.0);
angular.set(0.5);  // ❌ Wrong method for angular mode
// DriverStation warning: "AngularHubServo: Speed control not supported in ANGULAR mode"
// Robot continues running safely

BaseServo cr = new CRHubServo(hub, 1);
cr.setAngle(90.0);  // ❌ Wrong method for CR mode
// DriverStation warning: "CRHubServo: Angle control not supported in CONTINUOUS_ROTATION mode"
// Robot continues running safely
```

### Capability Detection
```java
// Check capabilities before calling methods
if (servo.supportsAngleControl()) {
    servo.setAngle(90.0);  // Safe - checked first
} else if (servo.supportsSpeedControl()) {
    servo.set(0.5);  // Safe - checked first
}
```

### Input Validation
```java
// Port validation
BaseServo bad = new AngularHubServo(hub, 10, 355.0);
// DriverStation error: "AngularHubServo: Invalid port 10. Must be 0-5. Defaulting to port 0."

// Angle clamping
BaseServo servo = new AngularHubServo(hub, 0, 180.0);
servo.setAngle(270.0);  // Out of range
// DriverStation warning: "Angle 270.0 out of range [0, 180.0]. Clamping to 180.0."

// Speed clamping
BaseServo crServo = new CRHubServo(hub, 1);
crServo.set(5.0);  // Out of range
// DriverStation warning: "Speed 5.0 out of range [-1.0, 1.0]. Clamping to 1.0."
```

## Hardware Specifications

### REV ServoHub
- **Ports**: 6 servo channels (0-5)
- **Connection**: CAN bus
- **Power**: 6V regulated output
- **Current**: 10A total (all servos)
- **Current Monitoring**: Per-channel
- **Cable**: Standard 3-wire servo connector
- **Bank Configuration**: Ports 0-2 (Bank 0), 3-5 (Bank 1)

### RoboRIO PWM Ports
- **Ports**: 10 PWM channels (0-9) on RoboRIO 2
- **Voltage**: 5V or 6V (depending on servo)
- **Signal**: Standard PWM (600-2400µs)
- **Current**: Through separate servo power module
- **Cable**: Standard 3-wire servo connector

### Servo Types
- **Standard RC Servo**: 0-180° typical, 1000-2000µs
- **Axon Max+**: 0-355° continuous, 500-2500µs
- **Continuous Rotation**: Full rotation, speed controlled by pulse width

## Best Practices

### 1. Use Capability Detection
```java
// Good: Check before calling
if (servo.supportsAngleControl()) {
    servo.setAngle(targetAngle);
}

// Avoid: Assuming capabilities
servo.setAngle(targetAngle);  // May fail if CR servo
```

### 2. Configure Angle Ranges
```java
// Axon Max+ servos
BaseServo axonServo = new AngularHubServo(hub, 0, 355.0);

// Standard RC servos
BaseServo standardServo = new DirectServo(0, ServoMode.ANGULAR, 0, 180.0);

// Custom range
BaseServo limitedServo = new DirectServo(1, ServoMode.ANGULAR, 45, 135);
```

### 3. Use Constants
```java
public class Constants {
    public static class Servos {
        // ServoHub ports
        public static final int CAMERA_PAN = 0;
        public static final int CAMERA_TILT = 1;
        public static final int INTAKE_ROLLER = 2;
        
        // PWM ports
        public static final int CLAW_SERVO = 0;
        
        // Angle limits
        public static final double AXON_MAX_ANGLE = 355.0;
        public static final double STANDARD_MAX_ANGLE = 180.0;
    }
}
```

### 4. Add Rate Limiting
```java
private double currentAngle = 0.0;
private static final double MAX_RATE = 45.0;  // degrees per 20ms

public void setTargetAngle(double target) {
    double delta = target - currentAngle;
    double maxDelta = MAX_RATE * 0.02;  // 20ms period
    
    if (Math.abs(delta) > maxDelta) {
        currentAngle += Math.signum(delta) * maxDelta;
    } else {
        currentAngle = target;
    }
    
    servo.setAngle(currentAngle);
}
```

## Troubleshooting

### Servo doesn't move
- **Check power**: Verify ServoHub/servo power module connected
- **Check port**: Verify port number in code matches hardware
- **Check mode**: Ensure using correct method (setAngle vs set)
- **Check cable**: Test with different servo/cable
- **Check current**: Monitor current draw (ServoHub only)

### Servo jitters
- **Loose connection**: Check servo cable connections
- **Insufficient power**: Check voltage and current capacity
- **Control frequency**: Ensure not sending commands too fast
- **Mechanical binding**: Check for physical obstructions

### Wrong direction
- **Angle inversion**: Try (maxAngle - desiredAngle)
- **Speed inversion**: Negate speed value
- **Wrong mapping**: Check forward/reverse understanding

### Limited range
- **Max angle too low**: Increase maxAngle parameter
- **Physical stops**: Check mechanical limits
- **Servo type**: Verify servo supports desired range

## Examples

### Complete Camera System
```java
public class VisionCameraControl {
    private final BaseServo panServo;
    private final BaseServo tiltServo;
    private double targetPan = 180.0;
    private double targetTilt = 90.0;
    
    public VisionCameraControl(ServoHub hub) {
        panServo = new AngularHubServo(hub, 0, 355.0);
        tiltServo = new AngularHubServo(hub, 1, 180.0);
    }
    
    public void trackTarget(double targetX, double targetY) {
        // Convert screen coordinates to servo angles
        targetPan = mapRange(targetX, -1, 1, 90, 270);
        targetTilt = mapRange(targetY, -1, 1, 45, 135);
    }
    
    public void periodic() {
        panServo.setAngle(targetPan);
        tiltServo.setAngle(targetTilt);
    }
    
    private double mapRange(double value, double inMin, double inMax, 
                           double outMin, double outMax) {
        return (value - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
    }
}
```

### Game Piece Manipulator
```java
public class GamePieceHandler {
    private final BaseServo clawServo;
    private final BaseServo intakeServo;
    private final BaseServo wristServo;
    
    private enum ClawState { OPEN, CLOSED }
    private ClawState clawState = ClawState.OPEN;
    
    public GamePieceHandler(ServoHub hub) {
        clawServo = new AngularHubServo(hub, 0, 90.0);
        intakeServo = new CRHubServo(hub, 1);
        wristServo = new AngularHubServo(hub, 2, 270.0);
    }
    
    public void openClaw() {
        clawServo.setAngle(90.0);
        clawState = ClawState.OPEN;
    }
    
    public void closeClaw() {
        clawServo.setAngle(0.0);
        clawState = ClawState.CLOSED;
    }
    
    public void runIntake(double speed) {
        intakeServo.set(speed);
    }
    
    public void setWristAngle(double degrees) {
        wristServo.setAngle(degrees);
    }
    
    public Command grabSequence() {
        return Commands.sequence(
            Commands.runOnce(this::openClaw),
            Commands.runOnce(() -> runIntake(0.8)),
            Commands.waitSeconds(0.5),
            Commands.runOnce(this::closeClaw),
            Commands.runOnce(() -> runIntake(0.0))
        );
    }
}
```

## See Also

- **[BaseServo API](BaseServo.md)** - Interface documentation
- **[AngularHubServo](AngularHubServo.md)** - REV ServoHub angular servo
- **[CRHubServo](CRHubServo.md)** - REV ServoHub continuous rotation
- **[DirectServo](DirectServo.md)** - Direct PWM servo control
- **[Actuators Overview](../README.md)** - All actuator types
- **[REV ServoHub](https://docs.revrobotics.com/servo-hub/)** - Hardware documentation
