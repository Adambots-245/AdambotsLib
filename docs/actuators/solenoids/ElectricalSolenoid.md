# ElectricalSolenoid

Controls electrical solenoids via RoboRIO Relay ports. Used for electrical valves and high-current switching.

## Quick Reference

```java
import com.adambots.lib.actuators.ElectricalSolenoid;

// Create electrical solenoid on Relay port 0
BaseSolenoid valve = new ElectricalSolenoid(0);

valve.enable();   // Turn on (relay energized)
valve.disable();  // Turn off (relay de-energized)
valve.toggle();   // Switch state
```

## Constructor

```java
public ElectricalSolenoid(int port)
```

**Parameters:**
- `port` - RoboRIO Relay port number (0-3 for RoboRIO 2)

**Validation:**
- Port range validated (0-3)
- Invalid ports default to 0 with DriverStation error

**Example:**
```java
// Valid
BaseSolenoid valve1 = new ElectricalSolenoid(0);  // Port 0
BaseSolenoid valve2 = new ElectricalSolenoid(3);  // Port 3

// Invalid - automatically corrected
BaseSolenoid bad = new ElectricalSolenoid(10);
// DriverStation error: "ElectricalSolenoid: Invalid Relay port 10.
//                       RoboRIO 2 has 4 Relay ports (0-3). Defaulting to port 0."
// Uses port 0 instead
```

## Hardware Specifications

### RoboRIO Relay Ports
- **Available Ports**: 0-3 (4 ports total)
- **Voltage**: 12V nominal
- **Current Capacity**:
  - 2A continuous per channel
  - 4A peak per channel
- **Switching**: Solid-state relay
- **Protection**: Overcurrent protection built-in

### Typical Uses
- Solenoid valves
- Pneumatic valve control (non-PCM)
- LED strip control (low current)
- Relay modules
- Custom electrical switching

## Usage Examples

### Basic Valve Control

```java
public class BallValve {
    private final BaseSolenoid valve;

    public BallValve(int relayPort) {
        valve = new ElectricalSolenoid(relayPort);
    }

    public void open() {
        valve.enable();
    }

    public void close() {
        valve.disable();
    }

    public boolean isOpen() {
        return valve.get();
    }
}

// In RobotContainer:
BallValve intakeValve = new BallValve(0);
intakeValve.open();
```

### Timed Control

```java
public class TimedValveController {
    private final BaseSolenoid valve;
    private final Timer timer = new Timer();

    public TimedValveController(int port) {
        valve = new ElectricalSolenoid(port);
    }

    public void openForDuration(double seconds) {
        valve.enable();
        timer.restart();

        // Use this in periodic()
        if (timer.hasElapsed(seconds)) {
            valve.disable();
        }
    }

    public Command openForDurationCommand(double seconds) {
        return Commands.runOnce(() -> valve.enable())
            .andThen(Commands.waitSeconds(seconds))
            .andThen(() -> valve.disable());
    }
}
```

### Multiple Valve System

```java
public class FluidSystem {
    private final BaseSolenoid inletValve;
    private final BaseSolenoid outletValve;
    private final BaseSolenoid pressureRelease;

    public FluidSystem() {
        inletValve = new ElectricalSolenoid(0);
        outletValve = new ElectricalSolenoid(1);
        pressureRelease = new ElectricalSolenoid(2);
    }

    public void fill() {
        inletValve.enable();
        outletValve.disable();
        pressureRelease.disable();
    }

    public void drain() {
        inletValve.disable();
        outletValve.enable();
        pressureRelease.disable();
    }

    public void releasePressure() {
        inletValve.disable();
        outletValve.disable();
        pressureRelease.enable();
    }

    public void closeAll() {
        inletValve.disable();
        outletValve.disable();
        pressureRelease.disable();
    }
}
```

## Wiring

### Basic Connection
```
RoboRIO Relay Port → Electrical Solenoid → Ground
```

### With External Relay Module
```
RoboRIO Relay Port → Relay Module Control
Relay Module Switch → High Current Load
Power Supply → Relay Module → Load
```

### Safety Considerations
1. **Current Limits**: Keep loads under 2A continuous
2. **Inductive Loads**: Add flyback diodes for solenoids/motors
3. **Wire Gauge**: Use appropriate wire for current
4. **Fusing**: Consider inline fuses for protection
5. **Isolation**: Use relay module for high currents (>2A)

## Port Assignment Example

```java
public class Constants {
    public static class Pneumatics {
        // Relay Ports (Electrical Solenoids)
        public static final int INTAKE_VALVE = 0;
        public static final int BALL_RELEASE_VALVE = 1;
        public static final int PRESSURE_VALVE = 2;
        public static final int LED_RELAY = 3;
    }
}

// Usage:
BaseSolenoid intake = new ElectricalSolenoid(Constants.Pneumatics.INTAKE_VALVE);
```

## Troubleshooting

### Solenoid doesn't activate
- **Check power**: Verify 12V at solenoid
- **Check connections**: Ensure relay port wiring correct
- **Test relay**: Use LED to verify relay switches
- **Check port number**: Verify code matches wiring (0-3)

### Intermittent operation
- **Loose connections**: Check all wire connections
- **Voltage drop**: Ensure adequate power supply
- **Code issues**: Check for conflicting control code
- **Relay damage**: Test with multimeter

### Overcurrent trips
- **Load too high**: Verify load <2A continuous
- **Short circuit**: Check for shorts in wiring
- **Use external relay**: For loads >2A
- **Check solenoid**: Test solenoid separately

## Best Practices

### 1. Document Port Assignments
```java
// Good: Clear constant names
public static final int INTAKE_VALVE_RELAY = 0;

// Avoid: Magic numbers
solenoid = new ElectricalSolenoid(0);  // What's on port 0?
```

### 2. Protect Against Errors
```java
// Safe: Validated at construction
try {
    valve = new ElectricalSolenoid(portNumber);
    // Port validation automatic, safe defaults used
} catch (Exception e) {
    // Won't throw - validation handles errors gracefully
}
```

### 3. Add State Tracking
```java
private boolean valveOpen = false;

public void openValve() {
    valve.enable();
    valveOpen = true;
    logStateChange("Valve opened");
}

private void logStateChange(String message) {
    System.out.println("[" + System.currentTimeMillis() + "] " + message);
}
```

### 4. Use Descriptive Names
```java
// Good: Purpose clear
BaseSolenoid ballIntakeValve = new ElectricalSolenoid(0);
BaseSolenoid pressureReleaseValve = new ElectricalSolenoid(1);

// Avoid: Generic names
BaseSolenoid valve1 = new ElectricalSolenoid(0);
BaseSolenoid s2 = new ElectricalSolenoid(1);
```

## Comparison with CTREPneumaticSolenoid

| Feature | ElectricalSolenoid | CTREPneumaticSolenoid |
|---------|-------------------|------------------------|
| **Connection** | Direct to RoboRIO | Via CAN (PCM) |
| **Ports** | 0-3 (4 ports) | 0-7 (8 ports) |
| **Type** | Single solenoid | Double solenoid |
| **Hardware** | Electrical valve | Pneumatic cylinder |
| **Control** | On/Off | Forward/Reverse |
| **Current** | 2A continuous | 500mA per channel |
| **Complexity** | Simple | Requires air system |

## See Also

- **[BaseSolenoid](BaseSolenoid.md)** - Interface documentation
- **[CTREPneumaticSolenoid](CTREPneumaticSolenoid.md)** - Pneumatic alternative
- **[Solenoids Overview](README.md)** - Complete solenoids guide
- **[WPILib Relay](https://docs.wpilib.org/en/stable/docs/software/hardware-apis/misc/relay.html)** - WPILib Relay documentation
