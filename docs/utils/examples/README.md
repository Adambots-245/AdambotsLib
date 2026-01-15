# Buttons Usage Examples

Complete, production-ready examples showing how to use the Buttons utility in FRC robot code.

---

## Files

### [Robot.java](Robot.java)

Complete Robot.java implementation showing:
- **Proper initialization sequence** - Buttons.init() BEFORE RobotContainer
- **Controller type configuration** - Mixed Xbox and PS5 controllers
- **Optional rumble feedback** - Haptic feedback when teleop starts
- **Optional rumble cleanup** - Stop rumble when disabled
- **Standard WPILib structure** - All required methods implemented

**Key Takeaway:** Always call `Buttons.init()` in `Robot.robotInit()` before creating `RobotContainer`.

---

### [RobotContainer.java](RobotContainer.java)

Comprehensive RobotContainer showing:
- **Default drive command** with input curves and deadzones
- **70+ button bindings** organized by subsystem
- **Rumble feedback** for important events (game piece acquired, shooter ready, etc.)
- **Trigger combinations** for safety-critical operations (climber deployment)
- **Mixed controller types** - Xbox driver, PS5 operator
- **Input processing examples** - Linear, cubic, and sigmoid curves
- **Auto-aim features** - AprilTag alignment, vision targeting
- **Preset positions** - D-Pad navigation to field positions
- **Emergency stop** - Combined stop all mechanisms
- **Organized binding methods** - Separate methods for driver, operator, and combos

**Key Takeaway:** Use `createForwardSupplier()`, `createStrafeSupplier()`, and `createRotationSupplier()` for clean, configurable drive commands.

---

## Quick Start

### 1. Copy Robot.java

```java
// In your Robot.java robotInit() method
@Override
public void robotInit() {
    // Initialize Buttons FIRST
    Buttons.init(0, 1, ControllerType.XBOX, ControllerType.PS5);

    // Then create RobotContainer
    m_robotContainer = new RobotContainer();
}
```

### 2. Set Up Drive Command

```java
// In RobotContainer.java
private void configureDefaultCommands() {
    swerve.setDefaultCommand(
        swerve.driveCommand(
            Buttons.createForwardSupplier(0.05, InputCurve.CUBIC),
            Buttons.createStrafeSupplier(0.05, InputCurve.CUBIC),
            Buttons.createRotationSupplier(0.1, InputCurve.SIGMOID)
        )
    );
}
```

### 3. Bind Commands to Buttons

```java
// In RobotContainer.java
private void configureButtonBindings() {
    // Xbox A button deploys intake
    Buttons.XboxA.onTrue(intake.deployCommand());

    // PS5 Square spins up shooter
    Buttons.PS5Square.whileTrue(shooter.spinUpCommand());

    // Add rumble feedback
    Buttons.XboxX.whileTrue(
        intake.intakeCommand()
            .until(() -> intake.hasGamePiece())
            .andThen(Commands.runOnce(() ->
                Buttons.rumbleXbox(Buttons.getXboxController(), 300, 0.7)
            ))
    );
}
```

---

## Common Patterns

### Pattern 1: Simple Button Binding

```java
// Press button -> run command once
Buttons.XboxA.onTrue(intake.deployCommand());

// Hold button -> run command while held
Buttons.XboxX.whileTrue(intake.intakeCommand());

// Toggle on/off
Buttons.XboxB.toggleOnTrue(climber.deployCommand());
```

### Pattern 2: Rumble Feedback

```java
// Rumble when action completes
Buttons.XboxA.onTrue(
    intake.deployCommand()
        .andThen(Commands.runOnce(() ->
            Buttons.rumbleXbox(Buttons.getXboxController(), 200, 0.5)
        ))
);

// Rumble when condition met
intake.hasGamePieceTrigger().onTrue(
    Commands.runOnce(() ->
        Buttons.rumbleXbox(Buttons.getXboxController(), 300, 0.7)
    )
);
```

### Pattern 3: Conditional Commands

```java
// Only shoot if shooter is at speed
Buttons.PS5Circle.onTrue(
    Commands.either(
        shooter.shootCommand(),  // If at speed
        Commands.runOnce(() ->   // Otherwise rumble warning
            Buttons.rumblePS5(Buttons.getPS5Controller(), 100, 1.0)
        ),
        shooter::isAtSpeed
    )
);
```

### Pattern 4: Button Combinations

```java
// Require two buttons for safety-critical operation
Buttons.XboxStart.and(Buttons.PS5Triangle).onTrue(
    climber.deploySequenceCommand()
);

// Modifier + button pattern
Buttons.XboxLeftBumper.and(Buttons.XboxA).onTrue(
    intake.advancedModeCommand()
);
```

### Pattern 5: Drive Speed Control

```java
// Slow mode while held
Buttons.XboxLeftBumper.whileTrue(
    swerve.setMaxSpeedCommand(0.3)  // 30% speed
);

// Turbo mode while held
Buttons.XboxRightBumper.whileTrue(
    swerve.setMaxSpeedCommand(1.0)  // 100% speed
);
```

### Pattern 6: Auto-Aim Features

```java
// Auto-align to AprilTag
Buttons.XboxLeftTriggerButton.whileTrue(
    swerve.aimAtAprilTagCommand(4, 2.0)
);

// Auto-aim at vision target
Buttons.XboxRightTriggerButton.whileTrue(
    swerve.aimAtTargetCommand(Cameras.SHOOTER_CAM)
);
```

---

## Controller Type Configurations

### Both Xbox Controllers

```java
// In Robot.java
Buttons.init(0, 1, ControllerType.XBOX, ControllerType.XBOX);

// In RobotContainer.java - driver and operator use Xbox triggers
Buttons.XboxA.onTrue(intake.deployCommand());           // Driver
Buttons.XboxB.onTrue(shooter.spinUpCommand());          // Operator (2nd Xbox)
```

### Both PS5 Controllers

```java
// In Robot.java
Buttons.init(0, 1, ControllerType.PS5, ControllerType.PS5);

// In RobotContainer.java - both use PS5 triggers
Buttons.PS5Cross.onTrue(intake.deployCommand());        // Driver
Buttons.PS5Square.onTrue(shooter.spinUpCommand());      // Operator (2nd PS5)
```

### Xbox Driver + Joystick Operator

```java
// In Robot.java
Buttons.init(0, 1, ControllerType.XBOX, ControllerType.EXTREME_3D_PRO);

// In RobotContainer.java
Buttons.XboxA.onTrue(intake.deployCommand());           // Driver (Xbox)
Buttons.JoystickTrigger.onTrue(shooter.shootCommand()); // Operator (Joystick)
```

### Single Controller (Driver Only)

```java
// In Robot.java
Buttons.init(0, -1, ControllerType.XBOX, ControllerType.NONE);

// In RobotContainer.java - only use driver triggers
Buttons.XboxA.onTrue(intake.deployCommand());
Buttons.XboxB.onTrue(shooter.spinUpCommand());
// Don't use PS5 or Joystick triggers - they won't work
```

---

## Input Curve Selection Guide

![Input Curves Comparison](../Curves.png)

The graph above shows different input response curves. Our implementation uses:
- **Linear (blue)** - Direct 1:1 response
- **Cubic** - Similar to red curve (A=0, B=1), using formula `0.1x + 0.9x³`
- **Sigmoid (green, a=2)** - S-curve for smooth acceleration

---

### Linear Curve - `InputCurve.LINEAR`

**Best for:**
- Experienced drivers who want 1:1 control
- Precise positioning tasks
- Competition play with skilled operators

**Characteristics:**
- Direct relationship: 50% stick = 50% output
- Maximum precision at all speeds
- No smoothing or dampening

```java
Buttons.createForwardSupplier(0.05, InputCurve.LINEAR)
```

---

### Cubic Curve - `InputCurve.CUBIC` (Recommended)

**Best for:**
- General driving (default recommendation)
- New or learning drivers
- Smoother low-speed control

**Characteristics:**
- Finer control at low speeds
- Still responsive at high speeds
- Formula: `0.1 * x + 0.9 * x³`
- 50% stick ≈ 20% output, 100% stick = 100% output

```java
Buttons.createForwardSupplier(0.05, InputCurve.CUBIC)
```

---

### Sigmoid Curve - `InputCurve.SIGMOID`

**Best for:**
- Rotation control (highly recommended)
- Preventing over-rotation
- Mechanisms that need smooth acceleration

**Characteristics:**
- Very smooth acceleration from zero
- Prevents sudden movements
- S-shaped response curve
- Gradual increase, then rapid, then gradual again

```java
Buttons.createRotationSupplier(0.1, InputCurve.SIGMOID)
```

---

## Deadzone Selection Guide

### Small Deadzone (0.03 - 0.05)

**Use when:**
- Controllers are new
- Maximum responsiveness needed
- Precision is critical

```java
Buttons.createForwardSupplier(0.05, InputCurve.CUBIC)
```

---

### Medium Deadzone (0.05 - 0.1)

**Use when:**
- Controllers are moderately worn
- Balance between precision and stability
- **Recommended for most use cases**

```java
Buttons.createForwardSupplier(0.08, InputCurve.CUBIC)
```

---

### Large Deadzone (0.1 - 0.15)

**Use when:**
- Controllers have noticeable drift
- Rotation control (larger is more stable)
- Practice bots with worn controllers

```java
Buttons.createRotationSupplier(0.15, InputCurve.SIGMOID)
```

---

## Best Practices

### ✓ DO

- Initialize Buttons in `Robot.robotInit()` BEFORE creating `RobotContainer`
- Use cubic curves for translation, sigmoid for rotation
- Group bindings by subsystem in separate methods
- Use rumble sparingly for important events only
- Test different deadzones to find what works for your controllers
- Use button combinations for safety-critical operations
- Document what each button does in comments

### ✗ DON'T

- Create RobotContainer before calling `Buttons.init()`
- Use very small deadzones (< 0.03) - controllers will drift
- Use very large deadzones (> 0.2) - hard to start moving
- Rumble on every button press - it's annoying
- Use sigmoid curves for translation - feels sluggish
- Mix up controller types (using Xbox triggers when PS5 configured)
- Forget to call `CommandScheduler.getInstance().run()` in `robotPeriodic()`

---

## Troubleshooting

### "Buttons not initialized" Exception

**Fix:** Call `Buttons.init()` in `Robot.robotInit()` before creating `RobotContainer`

```java
@Override
public void robotInit() {
    Buttons.init(0, 1, ControllerType.XBOX, ControllerType.PS5);  // FIRST
    m_robotContainer = new RobotContainer();                       // THEN
}
```

---

### Controller Not Responding

**Diagnostics:**
```java
System.out.println("Driver type: " + Buttons.getDriverType());
System.out.println("Driver controller: " + Buttons.getDriverController());
```

**Common causes:**
- Wrong USB port number
- Controller not connected to driver station
- Controller type mismatch (configured Xbox but using PS5)

---

### Robot Drifts When Sticks Released

**Fix:** Increase deadzone

```java
// Change from 0.05 to 0.1 or higher
Buttons.createForwardSupplier(0.1, InputCurve.CUBIC)
```

---

### Control Too Sensitive

**Fix:** Use cubic or sigmoid curve instead of linear

```java
// Change from LINEAR to CUBIC
Buttons.createForwardSupplier(0.05, InputCurve.CUBIC)
```

---

### Rumble Not Working

**Common causes:**
- Controller doesn't support rumble (Logitech joystick)
- USB cable is data-only (needs power+data cable)
- Wrong controller type configured

**Test:**
```java
Buttons.rumbleXbox(Buttons.getXboxController(), 1000, 1.0);
```

---

## See Also

- [Buttons Documentation](../Buttons.md) - Complete API reference
- [StateMachine Documentation](../StateMachine.md) - State machine for subsystems
- [Utils Documentation](../README.md) - Overview of all utilities
- [WPILib Command-Based Programming](https://docs.wpilib.org/en/stable/docs/software/commandbased/index.html)

---

**Last Updated:** 2026-01-14
