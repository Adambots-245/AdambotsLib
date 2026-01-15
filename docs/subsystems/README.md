# Subsystems

Reusable subsystem implementations for common FRC robot components.

## Overview

This folder contains pre-built subsystem classes that encapsulate common robot functionality. These subsystems follow WPILib command-based patterns and provide clean, tested implementations for common robot components.

## Available Subsystems

### Drivetrain

- **[SwerveSubsystem](SwerveSubsystem.md)** - YAGSL-based swerve drive subsystem
  - JSON-based configuration (no custom SwerveModule class)
  - PhotonVision integration for vision-corrected odometry
  - PathPlanner integration for autonomous
  - 17 command factory methods for all drive operations
  - 8 trigger methods for state-based command composition
  - Support for Kraken X60/X44, Falcon 500, and NEO motors

### Vision & Cameras

- **[CameraSubsystem](CameraSubsystem.md)** - USB camera subsystem for driver viewing
  - Support for 1-2 USB cameras on RoboRIO
  - Automatic streaming to Shuffleboard and Elastic dashboards
  - Pre-configured settings (Driver Optimized, High Detail, Low Bandwidth)
  - Dynamic camera switching with minimal latency
  - Runtime brightness adjustment
  - Connection status monitoring
  - Command factory methods for camera control

### LED Control

- **[CANdleSubsystem](CANdleSubsystem.md)** - CTRE CANdle LED strip controller
  - Solid colors and animations
  - Phoenix 6 API
  - WS2812B-compatible LED strips
  - Robot status indication

## Philosophy

Subsystems in AdambotsLib follow these principles:

### 1. Command-Based Integration
- Extend `SubsystemBase` from WPILib
- Expose state as triggers for command binding
- Provide factory methods for common commands
- Follow problem-domain abstraction patterns

### 2. Safety and Validation
- Input validation on all constructors
- Graceful fallbacks with DriverStation warnings
- Safe defaults for all parameters
- Hardware protection (current limits, etc.)

### 3. Configuration Simplicity
- Sensible defaults for most use cases
- Builder pattern for complex configuration
- Constants integration (pull from Constants.java)
- Clear documentation of hardware requirements

### 4. Hardware Interchangeability
When possible, subsystems use abstraction interfaces to allow hardware swapping without code changes.

## Usage Example

```java
import com.adambots.lib.subsystems.CANdleSubsystem;
import com.adambots.lib.Constants.LEDConstants;

// Create subsystem with CAN ID
CANdleSubsystem leds = new CANdleSubsystem(10);

// Set solid color
leds.setColor(LEDConstants.green);

// Run animation
leds.setAnimation(CANdleSubsystem.AnimationTypes.Rainbow);

// Use in command
addRequirements(leds);
```

## Integration with Robot Code

### Subsystem Registration

Register subsystems in your `RobotContainer`:

```java
public class RobotContainer {
  // Subsystems
  private final CANdleSubsystem m_leds = new CANdleSubsystem(10);

  public RobotContainer() {
    configureBindings();
    configureDefaultCommands();
  }

  private void configureDefaultCommands() {
    // Set default LED color
    m_leds.setDefaultCommand(
      Commands.run(() -> m_leds.setColor(LEDConstants.blue), m_leds)
    );
  }
}
```

### Command Factory Pattern

Create command factories for common operations:

```java
public class LEDCommands {
  public static Command setColorCommand(CANdleSubsystem leds, Color color) {
    return Commands.runOnce(() -> leds.setColor(color), leds);
  }

  public static Command blinkPattern(CANdleSubsystem leds, Color color) {
    return leds.run(() -> leds.setAnimation(AnimationTypes.Strobe))
               .withTimeout(2.0)
               .andThen(() -> leds.setColor(color));
  }
}
```

### State-Based LED Control

Use triggers to change LEDs based on robot state:

```java
// Show green when game piece detected
intakeSubsystem.hasGamePieceTrigger()
  .onTrue(Commands.runOnce(() ->
    m_leds.setColor(LEDConstants.green), m_leds))
  .onFalse(Commands.runOnce(() ->
    m_leds.setColor(LEDConstants.off), m_leds));

// Show alliance color in auto
new Trigger(DriverStation::isAutonomous)
  .whileTrue(Commands.run(() ->
    m_leds.setColor(
      DriverStation.getAlliance()
        .map(a -> a == Alliance.Red ?
          LEDConstants.red : LEDConstants.blue)
        .orElse(LEDConstants.white)
    ), m_leds));
```

## Testing

All subsystems support WPILib simulation:

```java
if (RobotBase.isSimulation()) {
  // Subsystems automatically initialize in sim mode
  // Use Phoenix Tuner or sim GUI to verify behavior
}
```

## Best Practices

### 1. Subsystem Lifecycle
- Initialize in RobotContainer constructor
- Set default commands in configureDefaultCommands()
- Clean up resources in subsystem's close() if needed

### 2. Periodic Updates
- Keep periodic() lightweight
- Avoid blocking operations
- Log critical state to NetworkTables/telemetry

### 3. Command Composition
- Use command factories for reusable operations
- Compose commands with andThen(), alongWith(), etc.
- Leverage triggers for state-based behavior

### 4. Hardware Configuration
- Define all hardware IDs in Constants.java
- Validate configuration in constructor
- Document hardware requirements in class Javadoc

## See Also

- [Actuators Documentation](../actuators/README.md)
- [Sensors Documentation](../sensors/README.md)
- [Command Best Practices](../../COMMAND_BEST_PRACTICES.md)
- [WPILib Command-Based](https://docs.wpilib.org/en/stable/docs/software/commandbased/index.html)

---

**Need Help?** Check the detailed documentation for each subsystem, or refer to the usage examples in USAGE.md.
