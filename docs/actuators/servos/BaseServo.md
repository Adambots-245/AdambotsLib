# BaseServo

The `BaseServo` interface provides a unified API for controlling both angular (position) and continuous rotation (speed) servos with mode-based safety.

## Interface Definition

```java
public interface BaseServo extends BaseActuator {
    enum ServoMode { CONTINUOUS_ROTATION, ANGULAR }
    
    ServoMode getMode();
    void turnCounterclockwise();
    void turnClockwise();
    void stop();
    void setPulseWidth(int pulseWidth);
    double getCurrent();
    
    // Mode-specific (with graceful fallbacks)
    default void setAngle(double degrees);    // Angular mode only
    default void set(double speed);           // CR mode only
    
    // Capability detection
    default boolean supportsAngleControl();
    default boolean supportsSpeedControl();
    default String getServoType();
}
```

## Servo Modes

### ANGULAR Mode
**Position control** - Servo moves to specific angles.
- **Range**: 0-355° (Axon Max+) or 0-180° (standard)
- **Methods**: `setAngle()`, `turnClockwise()` (min), `turnCounterclockwise()` (max)
- **Use**: Camera gimbals, arms, claws, flippers

### CONTINUOUS_ROTATION Mode
**Speed control** - Servo rotates continuously at variable speeds.
- **Range**: -1.0 (full CCW) to 1.0 (full CW)
- **Methods**: `set()`, `turnClockwise()` (CW), `turnCounterclockwise()` (CCW)
- **Use**: Intake rollers, feeders, spinners

## Methods

### `getMode()`
Returns the current operating mode.

```java
ServoMode mode = servo.getMode();
if (mode == ServoMode.ANGULAR) {
    // Use angle control
} else {
    // Use speed control
}
```

---

### `setAngle(double degrees)` - Angular Mode Only
Sets the servo to a specific angle.

**Parameters:**
- `degrees` - Target angle (0 to maxAngle)

**Behavior:**
- **Angular servos**: Moves to specified angle
- **CR servos**: Logs warning, call ignored (graceful fallback)

```java
BaseServo angular = new AngularHubServo(hub, 0, 355.0);
angular.setAngle(90.0);  // ✅ Works - angular mode

BaseServo cr = new CRHubServo(hub, 1);
cr.setAngle(90.0);  // ⚠️ Warning logged, call ignored
```

**Best Practice:**
```java
if (servo.supportsAngleControl()) {
    servo.setAngle(90.0);  // Safe - checked first
}
```

---

### `set(double speed)` - CR Mode Only
Sets the servo rotation speed.

**Parameters:**
- `speed` - Speed from -1.0 (full CCW) to 1.0 (full CW), 0.0 = stop

**Behavior:**
- **CR servos**: Rotates at specified speed
- **Angular servos**: Logs warning, call ignored (graceful fallback)

```java
BaseServo cr = new CRHubServo(hub, 1);
cr.set(0.5);  // ✅ Works - CR mode

BaseServo angular = new AngularHubServo(hub, 0, 355.0);
angular.set(0.5);  // ⚠️ Warning logged, call ignored
```

**Best Practice:**
```java
if (servo.supportsSpeedControl()) {
    servo.set(0.5);  // Safe - checked first
}
```

---

### `turnClockwise()`
Turns servo clockwise (behavior depends on mode).

- **Angular mode**: Moves to minimum angle (0°)
- **CR mode**: Full speed clockwise

```java
servo.turnClockwise();
```

---

### `turnCounterclockwise()`
Turns servo counterclockwise (behavior depends on mode).

- **Angular mode**: Moves to maximum angle (355° or 180°)
- **CR mode**: Full speed counterclockwise

```java
servo.turnCounterclockwise();
```

---

### `stop()`
Stops servo motion (behavior depends on mode).

- **Angular mode**: Moves to center position
- **CR mode**: Stops rotation (speed = 0)

```java
servo.stop();
```

---

### `setPulseWidth(int pulseWidth)`
Sets raw PWM pulse width in microseconds.

**Parameters:**
- `pulseWidth` - Pulse width in microseconds (typically 500-2500)

```java
servo.setPulseWidth(1500);  // Center position/stop
```

---

### `getCurrent()`
Gets current draw in amperes (ServoHub only).

**Returns**: Current in amps (0.0 for direct PWM servos)

```java
double current = servo.getCurrent();
System.out.println("Servo drawing: " + current + "A");
```

---

### `supportsAngleControl()`
Checks if servo supports angle control.

**Returns**: `true` if angular mode, `false` if CR mode

```java
if (servo.supportsAngleControl()) {
    servo.setAngle(90.0);
} else {
    System.out.println("This is a CR servo");
}
```

---

### `supportsSpeedControl()`
Checks if servo supports speed control.

**Returns**: `true` if CR mode, `false` if angular mode

```java
if (servo.supportsSpeedControl()) {
    servo.set(0.5);
} else {
    System.out.println("This is an angular servo");
}
```

---

### `getServoType()`
Gets servo implementation name for debugging.

**Returns**: Servo class name (e.g., "AngularHubServo")

```java
System.out.println("Using: " + servo.getServoType());
```

## Usage Examples

### Basic Control

```java
// Angular servo
BaseServo angular = new AngularHubServo(hub, 0, 355.0);
angular.setAngle(90.0);           // Position control
angular.turnCounterclockwise();   // Max angle
angular.stop();                   // Center

// CR servo
BaseServo cr = new CRHubServo(hub, 1);
cr.set(0.8);                  // Speed control
cr.turnClockwise();           // Full CW
cr.stop();                    // Stop rotation
```

### Polymorphic Subsystem

```java
public class ServoSubsystem {
    private final BaseServo servo;
    
    public ServoSubsystem(BaseServo servo) {
        this.servo = servo;  // Works with any servo type
    }
    
    public void setPosition(double value) {
        if (servo.supportsAngleControl()) {
            servo.setAngle(value);
        } else if (servo.supportsSpeedControl()) {
            servo.set(value / 180.0);  // Convert angle to speed
        }
    }
}
```

### Safe Mode Handling

```java
public void controlServo(BaseServo servo, double value) {
    ServoMode mode = servo.getMode();
    
    switch (mode) {
        case ANGULAR:
            servo.setAngle(value);  // Interpret as angle
            break;
        case CONTINUOUS_ROTATION:
            servo.set(value);  // Interpret as speed
            break;
    }
}
```

## Implementations

| Class | Mode | Hardware | Max Angle/Speed |
|-------|------|----------|-----------------|
| **[AngularHubServo](AngularHubServo.md)** | ANGULAR | REV ServoHub | Configurable (0-355°) |
| **[CRHubServo](CRHubServo.md)** | CR | REV ServoHub | -1.0 to 1.0 |
| **[DirectServo](DirectServo.md)** | Both | PWM Direct | Configurable |

## Design Patterns

### Factory Pattern

```java
public class ServoFactory {
    public static BaseServo createCameraServo(ServoHub hub, int port) {
        return new AngularHubServo(hub, port, 355.0);
    }
    
    public static BaseServo createIntakeRoller(ServoHub hub, int port) {
        return new CRHubServo(hub, port);
    }
}
```

### Strategy Pattern

```java
public interface ServoStrategy {
    void execute(BaseServo servo);
}

public class AngleStrategy implements ServoStrategy {
    private final double angle;
    
    public AngleStrategy(double angle) {
        this.angle = angle;
    }
    
    @Override
    public void execute(BaseServo servo) {
        if (servo.supportsAngleControl()) {
            servo.setAngle(angle);
        }
    }
}
```

## Best Practices

### 1. Always Check Capabilities
```java
// Good
if (servo.supportsAngleControl()) {
    servo.setAngle(90.0);
}

// Risky
servo.setAngle(90.0);  // May fail if CR servo
```

### 2. Use Interface for Flexibility
```java
// Good
private final BaseServo servo;

// Avoid
private final AngularHubServo servo;  // Locked to one type
```

### 3. Handle Both Modes
```java
public void setControl(BaseServo servo, double value) {
    if (servo.supportsAngleControl()) {
        servo.setAngle(value);
    } else if (servo.supportsSpeedControl()) {
        servo.set(value / 100.0);
    }
}
```

## See Also

- **[AngularHubServo](AngularHubServo.md)** - Angular implementation
- **[CRHubServo](CRHubServo.md)** - Continuous rotation implementation  
- **[DirectServo](DirectServo.md)** - Direct PWM implementation
- **[Servos Overview](README.md)** - Complete servos guide
