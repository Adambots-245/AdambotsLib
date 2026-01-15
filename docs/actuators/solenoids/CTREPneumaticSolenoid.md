# CTREPneumaticSolenoid

Controls pneumatic double solenoids via CTRE Pneumatics Control Module (PCM). Used for pneumatic cylinders that extend and retract.

## Quick Reference

```java
import com.adambots.lib.actuators.CTREPneumaticSolenoid;

// Create pneumatic solenoid (forward port 0, reverse port 1)
BaseSolenoid gripper = new CTREPneumaticSolenoid(0, 1);

gripper.enable();    // Extend (forward position)
gripper.disable();   // Retract (reverse position)
gripper.toggle();    // Switch positions

// Additional methods (beyond BaseSolenoid interface)
((CTREPneumaticSolenoid) gripper).forward();  // Explicit forward
((CTREPneumaticSolenoid) gripper).reverse();  // Explicit reverse
```

## Constructor

```java
public CTREPneumaticSolenoid(int forwardPort, int reversePort)
```

**Parameters:**
- `forwardPort` - PCM port for forward/extend action (0-7)
- `reversePort` - PCM port for reverse/retract action (0-7)

**Validation:**
- Port range validated (0-7)
- Ensures forward ≠ reverse ports
- Invalid configurations default to ports 0 and 1 with DriverStation errors

**Examples:**
```java
// Valid
BaseSolenoid intake = new CTREPneumaticSolenoid(0, 1);   // Ports 0 & 1
BaseSolenoid climber = new CTREPneumaticSolenoid(6, 7);  // Ports 6 & 7

// Invalid port range - automatically corrected
BaseSolenoid bad1 = new CTREPneumaticSolenoid(-1, 10);
// DriverStation errors for both ports, uses 0 and 1

// Same ports - automatically corrected
BaseSolenoid bad2 = new CTREPneumaticSolenoid(2, 2);
// DriverStation error: ports must be different, uses 0 and 1
```

## Methods

### Interface Methods (BaseSolenoid)

```java
void enable()   // Extend cylinder (forward position)
void disable()  // Retract cylinder (reverse position)
void toggle()   // Switch between positions
boolean get()   // Returns true if extended (forward)
void set(boolean value)  // true = extend, false = retract
```

### Additional Methods

```java
public void forward()  // Same as enable() - extend cylinder
public void reverse()  // Same as disable() - retract cylinder
```

These additional methods provide clearer pneumatics semantics. Use `forward()/reverse()` when the extend/retract language is more natural than enable/disable.

## Hardware Specifications

### CTRE Pneumatics Control Module (PCM)
- **Solenoid Ports**: 8 channels (0-7)
- **Connection**: CAN bus
- **Voltage**: 12V nominal
- **Current**: 500mA max per channel
- **Type**: Double solenoid control
- **Compressor**: Integrated control
- **Pressure**: Analog sensor input
- **LED Indicators**: Per-channel status

### Double Solenoid Operation
- **Forward (extend)**: Air to extend side, exhaust retract side
- **Reverse (retract)**: Air to retract side, exhaust extend side
- **Off**: Both sides exhausted (cylinder may move freely)

### Typical Uses
- Pneumatic cylinders (extend/retract)
- Grippers (open/close)
- Deployers (up/down)
- Climbers (extend/retract)
- Shifting mechanisms (high/low gear)

## Usage Examples

### Basic Gripper

```java
public class Gripper {
    private final CTREPneumaticSolenoid gripperSolenoid;

    public Gripper() {
        gripperSolenoid = new CTREPneumaticSolenoid(0, 1);
    }

    public void open() {
        gripperSolenoid.forward();  // or disable()
    }

    public void close() {
        gripperSolenoid.reverse();  // or enable()
    }

    public void toggle() {
        gripperSolenoid.toggle();
    }

    public boolean isClosed() {
        return !gripperSolenoid.get();  // reverse logic
    }
}
```

### Intake Deployer

```java
public class IntakeDeployer {
    private final CTREPneumaticSolenoid deployer;
    private boolean isDeployed = false;

    public IntakeDeployer() {
        deployer = new CTREPneumaticSolenoid(2, 3);
    }

    public void deploy() {
        deployer.enable();  // Extend
        isDeployed = true;
    }

    public void retract() {
        deployer.disable();  // Retract
        isDeployed = false;
    }

    public boolean isDeployed() {
        return isDeployed;
    }

    public Command deployCommand() {
        return Commands.runOnce(this::deploy);
    }

    public Command retractCommand() {
        return Commands.runOnce(this::retract);
    }
}
```

### Climbing Mechanism

```java
public class Climber {
    private final CTREPneumaticSolenoid leftArm;
    private final CTREPneumaticSolenoid rightArm;
    private final CTREPneumaticSolenoid hookRelease;

    public Climber() {
        leftArm = new CTREPneumaticSolenoid(0, 1);
        rightArm = new CTREPneumaticSolenoid(2, 3);
        hookRelease = new CTREPneumaticSolenoid(4, 5);
    }

    public void extendArms() {
        leftArm.forward();
        rightArm.forward();
    }

    public void retractArms() {
        leftArm.reverse();
        rightArm.reverse();
    }

    public void releaseHooks() {
        hookRelease.forward();
    }

    public void lockHooks() {
        hookRelease.reverse();
    }

    public Command climbSequence() {
        return Commands.sequence(
            Commands.runOnce(this::extendArms),
            Commands.waitSeconds(0.5),
            Commands.runOnce(this::releaseHooks),
            Commands.waitSeconds(0.2),
            Commands.runOnce(this::retractArms)
        );
    }
}
```

### Gear Shifter

```java
public class GearShifter {
    private final CTREPneumaticSolenoid shifter;

    public enum Gear {
        HIGH, LOW
    }

    private Gear currentGear = Gear.LOW;

    public GearShifter() {
        shifter = new CTREPneumaticSolenoid(6, 7);
        setGear(Gear.LOW);
    }

    public void setGear(Gear gear) {
        currentGear = gear;
        if (gear == Gear.HIGH) {
            shifter.forward();
        } else {
            shifter.reverse();
        }
    }

    public Gear getGear() {
        return currentGear;
    }

    public void toggleGear() {
        setGear(currentGear == Gear.HIGH ? Gear.LOW : Gear.HIGH);
    }
}
```

## Wiring

### PCM to RoboRIO
```
PCM CAN H → RoboRIO CAN H (Yellow)
PCM CAN L → RoboRIO CAN L (Green)
PCM 12V → PDP 12V
PCM Ground → PDP Ground
```

### Solenoid to PCM
```
Solenoid Forward → PCM Forward Port (0-7)
Solenoid Reverse → PCM Reverse Port (0-7)
Solenoid Common → PCM Common (black wire)
```

### Compressor
```
Compressor Power → PCM Compressor Output
Pressure Switch → PCM Pressure Switch Input
```

## Port Assignment Example

```java
public class Constants {
    public static class Pneumatics {
        // PCM Solenoid Ports (Forward, Reverse pairs)
        public static final int INTAKE_DEPLOY_FWD = 0;
        public static final int INTAKE_DEPLOY_REV = 1;

        public static final int GRIPPER_FWD = 2;
        public static final int GRIPPER_REV = 3;

        public static final int CLIMBER_LEFT_FWD = 4;
        public static final int CLIMBER_LEFT_REV = 5;

        public static final int CLIMBER_RIGHT_FWD = 6;
        public static final int CLIMBER_RIGHT_REV = 7;
    }
}

// Usage:
CTREPneumaticSolenoid intake = new CTREPneumaticSolenoid(
    Constants.Pneumatics.INTAKE_DEPLOY_FWD,
    Constants.Pneumatics.INTAKE_DEPLOY_REV
);
```

## Troubleshooting

### Cylinder doesn't move
- **Check compressor**: Verify it's running
- **Check pressure**: Should be 60-120 PSI
- **Check air leaks**: Listen for hissing sounds
- **Verify ports**: Check forward/reverse match code
- **PCM CAN ID**: Verify PCM configured correctly
- **Check tubing**: Ensure not kinked or disconnected

### Slow actuation
- **Low pressure**: Check compressor and pressure sensor
- **Air leak**: Find and fix leaks
- **Tubing size**: Consider larger diameter tubing
- **Multiple solenoids**: May depressurize system
- **Compressor size**: May be undersized

### Wrong direction
- **Swap ports**: Exchange forward and reverse port numbers in code
- **Or swap tubing**: Exchange forward and reverse tubes at cylinder
- **Check orientation**: Verify cylinder installed correctly

### Solenoid chatters
- **Control code**: Check for rapid enable/disable calls
- **Electrical noise**: Check CAN wiring and termination
- **Low voltage**: Verify 12V stable
- **Bad solenoid**: Replace if mechanical issue

## Best Practices

### 1. Use Descriptive Port Constants
```java
// Good: Clear purpose
public static final int INTAKE_EXTEND_PORT = 0;
public static final int INTAKE_RETRACT_PORT = 1;

// Avoid: Generic names
public static final int PORT_0 = 0;
public static final int PORT_1 = 1;
```

### 2. Choose Meaningful Method Names
```java
// Good: Clear semantics
public void deployIntake() {
    deployer.forward();  // Explicit: forward extends
}

public void retractIntake() {
    deployer.reverse();  // Explicit: reverse retracts
}

// Also good: Using enable/disable
public void openGripper() {
    gripper.enable();  // Enable means "open" for this gripper
}
```

### 3. Add Mechanical Delays
```java
// Wait for pneumatics to complete motion
public void deploy() {
    deployer.forward();
    Timer.delay(0.2);  // 200ms for cylinder to extend
}

// Or use Commands
public Command deployCommand() {
    return Commands.runOnce(() -> deployer.forward())
        .andThen(Commands.waitSeconds(0.2));
}
```

### 4. Track State
```java
private enum IntakeState {
    DEPLOYED, RETRACTED, MOVING
}

private IntakeState state = IntakeState.RETRACTED;

public void deploy() {
    if (state != IntakeState.DEPLOYED) {
        deployer.forward();
        state = IntakeState.MOVING;
        // Set to DEPLOYED after delay or sensor
    }
}
```

## Comparison with ElectricalSolenoid

| Feature | CTREPneumaticSolenoid | ElectricalSolenoid |
|---------|----------------------|-------------------|
| **Connection** | CAN (via PCM) | Direct to RoboRIO |
| **Ports** | 0-7 (8 ports) | 0-3 (4 ports) |
| **Type** | Double solenoid | Single solenoid |
| **Control** | Forward/Reverse | On/Off |
| **Hardware** | Pneumatic cylinder | Electrical valve |
| **Force** | High (pneumatic) | Varies |
| **Speed** | Fast | Varies |
| **Complexity** | Requires air system | Simple wiring |

## See Also

- **[BaseSolenoid](BaseSolenoid.md)** - Interface documentation
- **[ElectricalSolenoid](ElectricalSolenoid.md)** - Electrical alternative
- **[Solenoids Overview](README.md)** - Complete solenoids guide
- **[CTRE PCM](https://store.ctr-electronics.com/pcm/)** - Hardware documentation
- **[WPILib Pneumatics](https://docs.wpilib.org/en/stable/docs/hardware/hardware-basics/pneumatics-basics.html)** - General pneumatics guide
