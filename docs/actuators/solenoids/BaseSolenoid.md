# BaseSolenoid

The `BaseSolenoid` interface provides a unified API for controlling both pneumatic and electrical solenoids.

## Interface Definition

```java
package com.adambots.lib.actuators;

public interface BaseSolenoid {
    void enable();
    void disable();
    void toggle();
    boolean get();
    void set(boolean value);
}
```

## Methods

### `enable()`
Enables the solenoid.
- **Pneumatic solenoids**: Extends cylinder (forward position)
- **Electrical solenoids**: Turns relay on

```java
solenoid.enable();
```

---

### `disable()`
Disables the solenoid.
- **Pneumatic solenoids**: Retracts cylinder (reverse position)
- **Electrical solenoids**: Turns relay off

```java
solenoid.disable();
```

---

### `toggle()`
Toggles the solenoid state - enables if disabled, disables if enabled.

```java
solenoid.toggle();  // Switches to opposite state
```

**Implementation:**
```java
default void toggle() {
    set(!get());
}
```

---

### `get()`
Gets the current state of the solenoid.

**Returns**: `true` if enabled (extended/on), `false` if disabled (retracted/off)

```java
if (solenoid.get()) {
    System.out.println("Solenoid is enabled");
}
```

---

### `set(boolean value)`
Sets the solenoid state directly.

**Parameters:**
- `value` - `true` to enable, `false` to disable

```java
solenoid.set(true);   // Same as enable()
solenoid.set(false);  // Same as disable()
```

## Implementations

| Class | Hardware | Description |
|-------|----------|-------------|
| **[CTREPneumaticSolenoid](CTREPneumaticSolenoid.md)** | CTRE PCM | Double solenoid for pneumatic cylinders |
| **[ElectricalSolenoid](ElectricalSolenoid.md)** | RoboRIO Relay | Electrical solenoid via Relay port |

## Usage Examples

### Basic Control

```java
import com.adambots.lib.actuators.*;

// Create a solenoid (implementation-specific)
BaseSolenoid solenoid = new CTREPneumaticSolenoid(0, 1);

// Enable (extend/on)
solenoid.enable();

// Disable (retract/off)
solenoid.disable();

// Toggle state
solenoid.toggle();

// Check state
if (solenoid.get()) {
    System.out.println("Enabled");
}

// Set directly
solenoid.set(true);  // Enable
solenoid.set(false); // Disable
```

### In a Subsystem

```java
public class GripperSubsystem {
    private final BaseSolenoid gripperSolenoid;

    public GripperSubsystem(BaseSolenoid solenoid) {
        this.gripperSolenoid = solenoid;
    }

    public void open() {
        gripperSolenoid.disable();
    }

    public void close() {
        gripperSolenoid.enable();
    }

    public void toggle() {
        gripperSolenoid.toggle();
    }

    public boolean isClosed() {
        return gripperSolenoid.get();
    }
}

// In RobotContainer:
BaseSolenoid gripperHardware = new CTREPneumaticSolenoid(0, 1);
GripperSubsystem gripper = new GripperSubsystem(gripperHardware);
```

### State Machine Pattern

```java
public enum DeployerState {
    DEPLOYED, RETRACTED
}

public class Deployer {
    private final BaseSolenoid deployer;
    private DeployerState state = DeployerState.RETRACTED;

    public Deployer(BaseSolenoid solenoid) {
        this.deployer = solenoid;
    }

    public void setState(DeployerState newState) {
        this.state = newState;
        deployer.set(state == DeployerState.DEPLOYED);
    }

    public DeployerState getState() {
        return state;
    }

    public void toggleState() {
        setState(state == DeployerState.DEPLOYED ?
                 DeployerState.RETRACTED : DeployerState.DEPLOYED);
    }
}
```

### Command Integration

```java
import edu.wpi.first.wpilibj2.command.*;

public class GripperCommands {
    private final BaseSolenoid gripper;

    public GripperCommands(BaseSolenoid solenoid) {
        this.gripper = solenoid;
    }

    public Command openCommand() {
        return Commands.runOnce(() -> gripper.disable());
    }

    public Command closeCommand() {
        return Commands.runOnce(() -> gripper.enable());
    }

    public Command toggleCommand() {
        return Commands.runOnce(() -> gripper.toggle());
    }

    public Command openAndWaitCommand() {
        return Commands.runOnce(() -> gripper.disable())
            .andThen(Commands.waitSeconds(0.2));
    }
}
```

## Design Patterns

### Polymorphism

Use the interface to allow hardware changes without code changes:

```java
// Robot code doesn't care about implementation
public class IntakeSubsystem {
    private final BaseSolenoid deployer;

    // Accepts any solenoid implementation
    public IntakeSubsystem(BaseSolenoid solenoid) {
        this.deployer = solenoid;
    }

    public void deploy() {
        deployer.enable();
    }
}

// In RobotContainer, choose hardware:
// Option 1: Pneumatic
BaseSolenoid solenoid = new CTREPneumaticSolenoid(0, 1);

// Option 2: Electrical (just change this line!)
BaseSolenoid solenoid = new ElectricalSolenoid(0);

// Subsystem works with either
IntakeSubsystem intake = new IntakeSubsystem(solenoid);
```

### Builder Pattern with Solenoids

```java
public class RobotHardware {
    private BaseSolenoid intakeDeployer;
    private BaseSolenoid gripperClaw;
    private BaseSolenoid climberLeft;
    private BaseSolenoid climberRight;

    public static class Builder {
        private Builder() {}

        public Builder withPneumaticIntake(int forward, int reverse) {
            intakeDeployer = new CTREPneumaticSolenoid(forward, reverse);
            return this;
        }

        public Builder withElectricalIntake(int port) {
            intakeDeployer = new ElectricalSolenoid(port);
            return this;
        }

        public RobotHardware build() {
            return new RobotHardware(this);
        }
    }

    private RobotHardware(Builder builder) {
        this.intakeDeployer = builder.intakeDeployer;
        // ... other components
    }
}
```

## Best Practices

### 1. Use Interface for Subsystem Fields
```java
// Good: Flexible and testable
private final BaseSolenoid solenoid;

// Avoid: Tightly coupled
private final CTREPneumaticSolenoid solenoid;
```

### 2. Meaningful Method Names
```java
// Good: Clear intent
public void deployIntake() {
    deployer.enable();
}

public void retractIntake() {
    deployer.disable();
}

// Avoid: Generic names lose meaning
public void setSolenoid(boolean value) {
    solenoid.set(value);  // Enable or disable? Not clear
}
```

### 3. State Tracking
```java
// Track state for complex logic
private boolean isGripperClosed = false;

public void closeGripper() {
    gripper.enable();
    isGripperClosed = true;
}

public boolean canScore() {
    return isGripperClosed && hasGamePiece();
}
```

### 4. Debouncing
```java
// Prevent rapid toggling
private long lastToggleTime = 0;
private static final long TOGGLE_DEBOUNCE_MS = 200;

public void toggle() {
    long currentTime = System.currentTimeMillis();
    if (currentTime - lastToggleTime > TOGGLE_DEBOUNCE_MS) {
        solenoid.toggle();
        lastToggleTime = currentTime;
    }
}
```

## See Also

- **[CTREPneumaticSolenoid](CTREPneumaticSolenoid.md)** - Pneumatic implementation
- **[ElectricalSolenoid](ElectricalSolenoid.md)** - Electrical implementation
- **[Solenoids Overview](README.md)** - Complete solenoids guide
- **[Actuators](../README.md)** - All actuator types
