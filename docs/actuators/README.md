# Actuators

Actuators are hardware components that create motion or control physical systems on your robot. AdambotsLib provides unified, safe interfaces for controlling motors, servos, and solenoids.

## Overview

All actuator classes in AdambotsLib follow these principles:

### Safety First
- **Input Validation**: All port numbers and parameters are validated at construction time
- **Graceful Degradation**: Invalid inputs produce warnings, not crashes
- **Bounds Checking**: Values are clamped to safe ranges
- **Clear Errors**: DriverStation warnings explain what went wrong and how it was fixed

### Consistent APIs
- **Interface-Based**: Use `BaseMotor`, `BaseServo`, `BaseSolenoid` for polymorphism
- **Predictable Behavior**: Similar operations work the same way across different hardware
- **Capability Detection**: Check what a device supports before using advanced features

### Hardware Abstraction
- **Swap Components Easily**: Change hardware without rewriting code
- **Vendor-Agnostic**: Works with CTRE, REV, and WPILib components
- **Modern & Legacy**: Supports both current and older hardware

## Actuator Types

### [Motors](motors/README.md)
Control rotational motion with precise speed and position control.

**Supported Hardware:**
- REV NEO / NEO Vortex (via SPARK MAX)
- CTRE Falcon 500 / Kraken X60 (via TalonFX)
- CTRE Minion (via TalonFXS)

**Key Features:**
- Unified velocity units (rotations per second)
- Multiple control modes (position, velocity, voltage, current, motion magic)
- PID control and motion profiling
- Current limiting and soft limits

**Quick Example:**
```java
BaseMotor motor = new NEOMotor(1, false, 40, false);
motor.set(ControlMode.VELOCITY, 50.0);  // 50 RPS
```

[📖 Motors Documentation →](motors/README.md)

---

### [Servos](servos/README.md)
Control precise angular positions or continuous rotation.

**Supported Hardware:**
- REV ServoHub-connected servos (Angular & Continuous Rotation)
- Direct PWM servos (any standard servo)
- Axon Max+ and standard RC servos

**Key Features:**
- Angular mode: 0-355° position control (Axon Max+) or 0-180° (standard)
- Continuous Rotation mode: Variable speed control
- Pulse width control for advanced use cases
- Mode detection to prevent incorrect method calls

**Quick Example:**
```java
// Angular servo
ServoHub hub = new ServoHub(10);
BaseServo angularServo = new AngularHubServo(hub, 0, 355.0);
angularServo.setAngle(90.0);

// Continuous rotation servo
BaseServo crServo = new CRHubServo(hub, 1);
crServo.set(0.5);  // Half speed
```

[📖 Servos Documentation →](servos/README.md)

---

### [Solenoids](solenoids/README.md)
Control pneumatic cylinders and electrical valves.

**Supported Hardware:**
- CTRE Pneumatics Control Module (PCM) - pneumatic double solenoids
- RoboRIO Relay ports - electrical solenoids

**Key Features:**
- Simple enable/disable interface
- Toggle functionality
- State monitoring
- Port validation for safety

**Quick Example:**
```java
// Pneumatic solenoid
BaseSolenoid gripper = new CTREPneumaticSolenoid(0, 1);
gripper.enable();   // Extend
gripper.disable();  // Retract

// Electrical solenoid
BaseSolenoid valve = new ElectricalSolenoid(0);
valve.toggle();  // Switch state
```

[📖 Solenoids Documentation →](solenoids/README.md)

---

## Dummy Implementations for Disabled Subsystems

Every actuator interface has a no-op "dummy" implementation (`DummyMotor`, `DummyServo`, `DummySolenoid`) for use when hardware isn't physically present. Use these instead of `null` to eliminate null checks and prevent Epilogue logging errors:

```java
// In RobotMap — disabled subsystems use dummy objects instead of null
BaseMotor intakeMotor = isIntakeInstalled
    ? new NEOMotor(5, false, 40, false)
    : new DummyMotor();

BaseSolenoid deployer = isIntakeInstalled
    ? new CTREPneumaticSolenoid(0, 1)
    : new DummySolenoid();
```

## Common Patterns

### Interface-Based Design

Use base interfaces to allow hardware swapping:

```java
public class Shooter {
    private final BaseMotor shooterMotor;

    public Shooter(BaseMotor motor) {
        this.shooterMotor = motor;  // Works with NEO, TalonFX, or Minion
    }

    public void setSpeed(double rps) {
        shooterMotor.set(ControlMode.VELOCITY, rps);
    }
}

// In RobotContainer, choose hardware:
BaseMotor motor = new NEOMotor(1, false, 40, false);  // or TalonFXMotor, or MinionMotor
Shooter shooter = new Shooter(motor);
```

### Capability Detection

Check what hardware supports before using advanced features:

```java
// Motors
if (motor.supportsControlMode(ControlMode.MOTION_MAGIC_FOC_TORQUE)) {
    motor.set(ControlMode.MOTION_MAGIC_FOC_TORQUE, targetPosition);
} else {
    motor.set(ControlMode.MOTION_MAGIC, targetPosition);
}

// Servos
if (servo.supportsAngleControl()) {
    servo.setAngle(90.0);
} else if (servo.supportsSpeedControl()) {
    servo.set(0.5);
}
```

### Safe Construction

All constructors validate inputs and provide clear feedback:

```java
// Invalid port numbers are caught immediately
BaseMotor motor = new NEOMotor(-1, false, 40, false);
// DriverStation error: "NEOMotor: Port -1 invalid..."
// Motor defaults to safe configuration

// Invalid servo port
BaseServo servo = new AngularHubServo(hub, 10, 355.0);
// DriverStation error: "AngularHubServo: Invalid port 10. Must be 0-5..."
// Servo defaults to port 0
```

## Best Practices

### 1. Use Interfaces for Subsystems
```java
// Good: Flexible, testable
public class Intake {
    private final BaseMotor motor;
    private final BaseSolenoid deployer;
}

// Avoid: Tightly coupled to specific hardware
public class Intake {
    private final NEOMotor motor;  // Hard to test or swap hardware
}
```

### 2. Configure Once in Constructor
```java
public Shooter(BaseMotor motor) {
    this.motor = motor;

    // Configure at construction time
    motor.setPID(0, 0.1, 0.0, 0.0, 0.0);
    motor.setBrakeMode(true);
    motor.enableVoltageCompensation(12.0);
}
```

### 3. Check Capability Before Use
```java
// Check before using advanced features
if (servo.supportsAngleControl()) {
    servo.setAngle(targetAngle);
}
```

### 4. Handle Velocity Units Carefully
```java
// All motors use RPS (rotations per second)
motor.set(ControlMode.VELOCITY, 50.0);  // 50 RPS (3000 RPM)

// If you have RPM values, convert:
double rpm = 3000;
motor.set(ControlMode.VELOCITY, rpm / 60.0);  // Convert to RPS
```

## Migration from WPILib

### Motors
```java
// WPILib (vendor-specific)
CANSparkMax motor = new CANSparkMax(1, MotorType.kBrushless);
motor.set(0.5);

// AdambotsLib (unified interface)
BaseMotor motor = new NEOMotor(1, false, 40, false);
motor.set(0.5);  // or use ControlMode for advanced control
```

### Servos
```java
// WPILib
Servo servo = new Servo(0);
servo.setAngle(90);

// AdambotsLib (with validation and safety)
BaseServo servo = new DirectServo(0, ServoMode.ANGULAR);
servo.setAngle(90);  // Validates angle range, provides warnings
```

### Solenoids
```java
// WPILib
DoubleSolenoid solenoid = new DoubleSolenoid(
    PneumaticsModuleType.CTREPCM, 0, 1);
solenoid.set(Value.kForward);

// AdambotsLib (with validation)
BaseSolenoid solenoid = new CTREPneumaticSolenoid(0, 1);
solenoid.enable();  // Clearer semantics, validates ports
```

## Troubleshooting

### "DriverStation: Invalid port number"
**Cause**: Constructor received out-of-range port number
**Solution**: Check hardware connections and port numbers in code
**Result**: Component uses safe default, continues operating

### "Control mode not supported"
**Cause**: Tried to use unsupported control mode (e.g., FOC torque on NEO)
**Solution**: Use `supportsControlMode()` to check first, or use fallback mode
**Result**: Warning logged, fallback mode used automatically

### "Angle control not supported in CR mode"
**Cause**: Called `setAngle()` on a continuous rotation servo
**Solution**: Check mode with `supportsAngleControl()` before calling
**Result**: Warning logged, call ignored

## See Also

- **[Motors Documentation](motors/README.md)** - Complete motor control guide
- **[Servos Documentation](servos/README.md)** - Complete servo control guide
- **[Solenoids Documentation](solenoids/README.md)** - Complete solenoid control guide
- **[Main Documentation](../README.md)** - Library overview
