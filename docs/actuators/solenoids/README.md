# Solenoids

Solenoids control pneumatic cylinders and electrical valves on your robot. AdambotsLib provides a unified interface for both pneumatic (PCM-based) and electrical (Relay-based) solenoids.

## Quick Start

```java
import com.adambots.lib.actuators.*;

// Pneumatic solenoid (CTRE PCM)
BaseSolenoid gripper = new CTREPneumaticSolenoid(0, 1);  // Forward: 0, Reverse: 1
gripper.enable();   // Extend (forward position)
gripper.disable();  // Retract (reverse position)
gripper.toggle();   // Switch between positions

// Electrical solenoid (RoboRIO Relay)
BaseSolenoid valve = new ElectricalSolenoid(0);  // Relay port 0
valve.enable();   // Turn on
valve.disable();  // Turn off
```

## Available Implementations

| Class | Hardware | Ports | Use Case |
|-------|----------|-------|----------|
| **[CTREPneumaticSolenoid](CTREPneumaticSolenoid.md)** | CTRE PCM | 0-7 (double) | Pneumatic cylinders (extend/retract) |
| **[ElectricalSolenoid](ElectricalSolenoid.md)** | RoboRIO Relay | 0-3 (single) | Electrical valves, relays |
| **DummySolenoid** | None | N/A | No-op null object for disabled subsystems |

### Disabled Subsystems (DummySolenoid)

When a subsystem's solenoid isn't physically present, use `DummySolenoid` instead of `null`. It tracks internal boolean state for `toggle()`/`get()` consistency but performs no hardware I/O:

```java
BaseSolenoid deployer = isMechanismInstalled
    ? new CTREPneumaticSolenoid(0, 1)
    : new DummySolenoid();
```

## Interface: BaseSolenoid

All solenoid classes implement the `BaseSolenoid` interface:

```java
public interface BaseSolenoid {
    void enable();           // Enable/extend
    void disable();          // Disable/retract
    void toggle();           // Switch state
    boolean get();           // Get current state
    void set(boolean value); // Set state directly
}
```

**[📖 BaseSolenoid API Reference](BaseSolenoid.md)**

## Choosing the Right Solenoid

### Pneumatic Solenoids (CTREPneumaticSolenoid)

**Use when:**
- Controlling pneumatic cylinders
- Need extend/retract motion
- Using CTRE Pneumatics Control Module (PCM)
- Requires compressed air system

**Advantages:**
- High force output
- Fast actuation
- Reliable in dusty environments
- Two stable positions

**Hardware Requirements:**
- CTRE PCM (CAN-connected)
- Compressor
- Air tank
- Pneumatic tubing
- Double solenoid valves

**Example:**
```java
BaseSolenoid intakeDeployer = new CTREPneumaticSolenoid(0, 1);
intakeDeployer.enable();   // Deploy intake
intakeDeployer.disable();  // Retract intake
```

### Electrical Solenoids (ElectricalSolenoid)

**Use when:**
- Controlling electrical valves
- Switching high-current loads via relay
- Direct connection to RoboRIO
- No pneumatics available

**Advantages:**
- Simple wiring (direct to RoboRIO)
- No compressor/air system needed
- Lower weight
- No air leaks

**Hardware Requirements:**
- RoboRIO (Relay ports 0-3)
- Electrical solenoid valve
- Appropriate power supply

**Example:**
```java
BaseSolenoid ballValve = new ElectricalSolenoid(0);
ballValve.enable();   // Open valve
ballValve.disable();  // Close valve
```

## Common Patterns

### Simple Two-Position Actuator

```java
public class IntakeDeployer {
    private final BaseSolenoid deployer;

    public IntakeDeployer(BaseSolenoid solenoid) {
        this.deployer = solenoid;
    }

    public void deploy() {
        deployer.enable();
    }

    public void retract() {
        deployer.disable();
    }

    public void toggle() {
        deployer.toggle();
    }

    public boolean isDeployed() {
        return deployer.get();
    }
}

// In RobotContainer:
BaseSolenoid solenoid = new CTREPneumaticSolenoid(0, 1);
IntakeDeployer deployer = new IntakeDeployer(solenoid);
```

### State Machine Integration

```java
public enum GripperState {
    OPEN, CLOSED
}

public class Gripper {
    private final BaseSolenoid gripperSolenoid;
    private GripperState state = GripperState.OPEN;

    public Gripper(BaseSolenoid solenoid) {
        this.gripperSolenoid = solenoid;
    }

    public void setState(GripperState newState) {
        this.state = newState;
        switch (state) {
            case OPEN:
                gripperSolenoid.disable();
                break;
            case CLOSED:
                gripperSolenoid.enable();
                break;
        }
    }

    public GripperState getState() {
        return state;
    }

    public void toggleState() {
        setState(state == GripperState.OPEN ? GripperState.CLOSED : GripperState.OPEN);
    }
}
```

### Multiple Solenoid Control

```java
public class ClimbingMechanism {
    private final BaseSolenoid leftArm;
    private final BaseSolenoid rightArm;
    private final BaseSolenoid hook;

    public ClimbingMechanism() {
        leftArm = new CTREPneumaticSolenoid(0, 1);
        rightArm = new CTREPneumaticSolenoid(2, 3);
        hook = new CTREPneumaticSolenoid(4, 5);
    }

    public void extendAll() {
        leftArm.enable();
        rightArm.enable();
        hook.enable();
    }

    public void retractAll() {
        leftArm.disable();
        rightArm.disable();
        hook.disable();
    }

    public void setArmState(boolean extended) {
        leftArm.set(extended);
        rightArm.set(extended);
    }
}
```

## Safety Features

All solenoid implementations include:

### Port Validation
```java
// Invalid ports are caught at construction
BaseSolenoid bad = new ElectricalSolenoid(10);
// DriverStation error: "ElectricalSolenoid: Invalid Relay port 10.
//                       RoboRIO 2 has 4 Relay ports (0-3). Defaulting to port 0."
// Solenoid uses port 0 safely
```

### Port Uniqueness (Pneumatic Solenoids)
```java
// Forward and reverse ports must be different
BaseSolenoid bad = new CTREPneumaticSolenoid(2, 2);
// DriverStation error: "CTREPneumaticSolenoid: Forward port (2) and reverse port (2)
//                       must be different. Defaulting to ports 0 and 1."
// Solenoid uses ports 0 and 1 safely
```

### State Monitoring
```java
// Always know the current state
if (solenoid.get()) {
    // Solenoid is enabled/extended
} else {
    // Solenoid is disabled/retracted
}
```

## Hardware Specifications

### CTRE Pneumatics Control Module (PCM)
- **Ports**: 8 solenoid channels (0-7)
- **Connection**: CAN bus
- **Voltage**: 12V nominal
- **Current**: 500mA max per channel
- **Control**: Double solenoids (forward/reverse)
- **Compressor Control**: Integrated
- **Pressure Monitoring**: Via analog input

### RoboRIO Relay Ports
- **Ports**: 4 relay channels (0-3)
- **Connection**: Direct to RoboRIO
- **Voltage**: 12V
- **Current**: 2A continuous, 4A peak per channel
- **Control**: On/off (single direction)
- **Protection**: Overcurrent protection

## Best Practices

### 1. Use Descriptive Names
```java
// Good: Clear purpose
BaseSolenoid intakeDeployer = new CTREPneumaticSolenoid(0, 1);
BaseSolenoid gripperClaw = new CTREPneumaticSolenoid(2, 3);

// Avoid: Generic names
BaseSolenoid solenoid1 = new CTREPneumaticSolenoid(0, 1);
BaseSolenoid s2 = new CTREPneumaticSolenoid(2, 3);
```

### 2. Document Port Assignments
```java
public class Constants {
    // Pneumatic Ports (PCM)
    public static final int INTAKE_DEPLOY_FORWARD = 0;
    public static final int INTAKE_DEPLOY_REVERSE = 1;
    public static final int GRIPPER_FORWARD = 2;
    public static final int GRIPPER_REVERSE = 3;

    // Relay Ports
    public static final int BALL_VALVE = 0;
    public static final int PRESSURE_RELEASE = 1;
}
```

### 3. Handle State Transitions Carefully
```java
// Add delays for mechanical movement
public void deployIntake() {
    deployer.enable();
    Timer.delay(0.2);  // Wait for pneumatics to actuate
}

// Or use Commands with deadlines
public Command deployIntakeCommand() {
    return Commands.runOnce(() -> deployer.enable())
        .withTimeout(0.2);
}
```

### 4. Test in Disabled Mode
```java
// Verify solenoid wiring in disabled mode
public void testSolenoids() {
    System.out.println("Testing solenoid on ports 0,1...");
    solenoid.enable();
    Timer.delay(1.0);
    solenoid.disable();
    Timer.delay(1.0);
    System.out.println("Test complete");
}
```

## Troubleshooting

### Pneumatic Issues

**Problem**: Solenoid doesn't actuate
- Check compressor is running
- Verify air pressure (60-120 PSI)
- Check for air leaks
- Verify port numbers in code match hardware
- Check PCM CAN ID configuration

**Problem**: Slow actuation
- Check air pressure (should be >60 PSI)
- Verify tubing isn't kinked
- Check for air leaks reducing pressure
- Consider larger diameter tubing

**Problem**: Solenoid "chatters" (rapid switching)
- Check control code for tight loops
- Verify only one piece of code controls solenoid
- Add debouncing if triggered by sensor

### Electrical Issues

**Problem**: Relay doesn't switch
- Check 12V power to relay module
- Verify RoboRIO Relay port connection
- Check port number in code (0-3)
- Test with LED or multimeter

**Problem**: Overcurrent trips
- Verify load current < 2A continuous
- Check for short circuits
- Use external relay for higher currents
- Verify correct Relay port wiring

## Examples

### Game Piece Manipulator
```java
public class Manipulator {
    private final BaseSolenoid gripper;
    private final BaseSolenoid wrist;

    public Manipulator() {
        gripper = new CTREPneumaticSolenoid(0, 1);
        wrist = new CTREPneumaticSolenoid(2, 3);
    }

    public void grab() {
        gripper.enable();
    }

    public void release() {
        gripper.disable();
    }

    public void wristUp() {
        wrist.enable();
    }

    public void wristDown() {
        wrist.disable();
    }

    public Command grabCommand() {
        return Commands.runOnce(this::grab);
    }

    public Command releaseCommand() {
        return Commands.runOnce(this::release);
    }
}
```

### Climbing Mechanism
```java
public class Climber {
    private final BaseSolenoid leftClimber;
    private final BaseSolenoid rightClimber;
    private boolean isExtended = false;

    public Climber() {
        leftClimber = new CTREPneumaticSolenoid(4, 5);
        rightClimber = new CTREPneumaticSolenoid(6, 7);
    }

    public void extend() {
        leftClimber.enable();
        rightClimber.enable();
        isExtended = true;
    }

    public void retract() {
        leftClimber.disable();
        rightClimber.disable();
        isExtended = false;
    }

    public void toggle() {
        if (isExtended) {
            retract();
        } else {
            extend();
        }
    }

    public boolean isExtended() {
        return isExtended;
    }
}
```

## See Also

- **[BaseSolenoid API](BaseSolenoid.md)** - Interface documentation
- **[CTREPneumaticSolenoid](CTREPneumaticSolenoid.md)** - Pneumatic solenoid details
- **[ElectricalSolenoid](ElectricalSolenoid.md)** - Electrical solenoid details
- **[Actuators Overview](../README.md)** - All actuator types
- **[CTRE PCM Documentation](https://store.ctr-electronics.com/pcm/)** - Hardware reference
- **[WPILib Pneumatics](https://docs.wpilib.org/en/stable/docs/hardware/hardware-basics/pneumatics-basics.html)** - General pneumatics guide
