# CANdleSubsystem - Triggers & Command Factories

Usage guide for CANdleSubsystem's trigger-based state exposure and command factories, following WPILib command-based best practices.

## Overview

CANdleSubsystem follows modern command-based patterns by:
1. **Exposing state as triggers** - Problem-domain questions about LED state
2. **Providing command factories** - Pre-built commands for common operations
3. **Supporting composition** - Commands designed to work with `.andThen()`, `.alongWith()`, etc.

## State Exposure (Triggers)

### Current State Queries

Get the current LED state:

```java
// Get current color
Color currentColor = leds.getCurrentColor();

// Get current animation
AnimationTypes currentAnim = leds.getCurrentAnimation();
```

### Trigger Methods

#### isShowingColorTrigger(Color)

True when LEDs are showing a specific solid color (not animating).

```java
// Trigger when showing green
Trigger showingGreen = leds.isShowingColorTrigger(LEDConstants.green);

// Use for conditional logic
showingGreen.onTrue(Commands.print("LEDs are green!"));

// Bind to button
controller.a().whileTrue(leds.setColorCommand(LEDConstants.blue))
              .onFalse(leds.setColorCommand(LEDConstants.green));
```

#### isOffTrigger()

True when LEDs are off (black).

```java
// Check if LEDs are off
Trigger ledsOff = leds.isOffTrigger();

// Turn on when robot enabled
new Trigger(DriverStation::isEnabled)
  .and(ledsOff)
  .onTrue(leds.allianceColorCommand());
```

#### isAnimatingTrigger()

True when any animation is running.

```java
// Trigger when animating
Trigger animating = leds.isAnimatingTrigger();

// Stop animation when teleop starts
new Trigger(DriverStation::isTeleop)
  .and(animating)
  .onTrue(leds.allianceColorCommand());
```

#### isShowingAnimationTrigger(AnimationTypes)

True when a specific animation is running.

```java
// Check if rainbow is active
Trigger showingRainbow = leds.isShowingAnimationTrigger(AnimationTypes.Rainbow);

// Cancel rainbow on button press
controller.b().and(showingRainbow)
              .onTrue(leds.turnOffCommand());
```

## Command Factories

### Basic Commands

#### setColorCommand(Color)

Set LEDs to a solid color.

```java
// Set to green
Command greenCmd = leds.setColorCommand(LEDConstants.green);

// Bind to button
controller.a().onTrue(greenCmd);

// Use in autonomous
return Commands.sequence(
  leds.setColorCommand(LEDConstants.yellow),
  autoRoutine,
  leds.setColorCommand(LEDConstants.green)
);
```

#### turnOffCommand()

Turn off LEDs.

```java
// Turn off when disabled
new Trigger(DriverStation::isDisabled)
  .onTrue(leds.turnOffCommand());

// Turn off at end of match
Commands.waitUntil(() -> DriverStation.getMatchTime() < 5)
        .andThen(leds.turnOffCommand());
```

#### allianceColorCommand()

Set to alliance color (red or blue, white if unknown).

```java
// Default command to show alliance
leds.setDefaultCommand(leds.allianceColorCommand());

// Show alliance color in auto
new Trigger(DriverStation::isAutonomous)
  .whileTrue(leds.allianceColorCommand());
```

#### setAnimationCommand(AnimationTypes)

Start an animation.

```java
// Show rainbow during teleop init
Command teleopInit = Commands.sequence(
  leds.setAnimationCommand(AnimationTypes.Rainbow),
  Commands.waitSeconds(1.0),
  leds.allianceColorCommand()
);

// Larson when autonomous
new Trigger(DriverStation::isAutonomous)
  .whileTrue(leds.setAnimationCommand(AnimationTypes.Larson));
```

### Pattern Commands

#### blinkCommand(Color, int)

Blink a color N times.

```java
// Blink red 3 times as warning
Command warn = leds.blinkCommand(LEDConstants.red, 3);

// Blink green when scoring
m_intake.hasGamePieceTrigger()
  .onTrue(leds.blinkCommand(LEDConstants.green, 2));

// Blink yellow before match
Commands.waitUntil(() -> DriverStation.getMatchTime() < 15)
        .andThen(leds.blinkCommand(LEDConstants.yellow, 5));
```

#### pulseCommand(Color, double)

Smooth pulse effect for specified duration.

```java
// Pulse blue for 2 seconds
Command pulse = leds.pulseCommand(LEDConstants.blue, 2.0);

// Pulse during intake
Command intakeCmd = Commands.parallel(
  m_intake.runIntakeCommand(),
  leds.pulseCommand(LEDConstants.orange, 3.0)
);

// Pulse while aiming
Command aimAndShoot = Commands.sequence(
  Commands.parallel(
    m_shooter.aimCommand(),
    leds.pulseCommand(LEDConstants.yellow, 1.5)
  ),
  m_shooter.shootCommand()
);
```

#### strobeCommand(Color, double)

Strobe flash for specified duration.

```java
// Strobe red for 1.5 seconds
Command strobe = leds.strobeCommand(LEDConstants.red, 1.5);

// Strobe during climb
Command climb = Commands.parallel(
  m_climber.raiseCommand(),
  leds.strobeCommand(LEDConstants.purple, 3.0)
);
```

### Status Indication Commands

#### celebrateCommand()

Rainbow animation followed by green (2 second celebration).

```java
// Celebrate after scoring
m_scorer.scoredTrigger()
  .onTrue(leds.celebrateCommand());

// Celebrate autonomous success
return autoRoutine.finallyDo(() -> {
  if (autoSucceeded) {
    leds.celebrateCommand().schedule();
  }
});

// Victory sequence
Command victory = Commands.sequence(
  leds.celebrateCommand(),
  leds.celebrateCommand(),  // Double celebration!
  leds.setColorCommand(LEDConstants.green)
);
```

#### warningCommand()

Red blink pattern (5 blinks).

```java
// Warning when temperature high
new Trigger(() -> m_shooter.getTemperature() > 50)
  .onTrue(leds.warningCommand());

// Warning before endgame
Commands.waitUntil(() -> DriverStation.getMatchTime() < 30)
        .andThen(leds.warningCommand());
```

#### errorCommand()

Red strobe (1.5 seconds) for critical errors.

```java
// Error when sensor fails
m_sensor.faultTrigger()
  .onTrue(leds.errorCommand());

// Critical error handling
if (criticalError) {
  leds.errorCommand().schedule();
  DriverStation.reportError("Critical error occurred", false);
}
```

#### readyCommand()

Solid green for ready state.

```java
// Ready when mechanisms at setpoint
new Trigger(() ->
    m_shooter.atSetpoint() &&
    m_intake.hasGamePiece()
  ).onTrue(leds.readyCommand());

// Ready state in teleop
new Trigger(DriverStation::isTeleop)
  .and(() -> !m_intake.isRunning())
  .whileTrue(leds.readyCommand());
```

#### busyCommand()

Orange Larson animation for busy state.

```java
// Busy during initialization
Command initialize = Commands.sequence(
  leds.busyCommand(),
  m_subsystem.initializeCommand(),
  leds.readyCommand()
);

// Busy during long operation
Command longOperation = Commands.parallel(
  leds.busyCommand(),
  m_mechanism.longProcessCommand()
);
```

#### disabledCommand()

Dim yellow for disabled state.

```java
// Show disabled state
new Trigger(DriverStation::isDisabled)
  .onTrue(leds.disabledCommand());
```

## Usage Patterns

### State-Based Control

Use triggers to change LEDs based on robot state:

```java
public void configureBindings() {
  // Alliance color when enabled
  new Trigger(DriverStation::isEnabled)
    .whileTrue(leds.allianceColorCommand());

  // Off when disabled
  new Trigger(DriverStation::isDisabled)
    .whileTrue(leds.disabledCommand());

  // Green when game piece acquired
  m_intake.hasGamePieceTrigger()
    .whileTrue(leds.setColorCommand(LEDConstants.green))
    .onFalse(leds.allianceColorCommand());

  // Warning when temperature high
  new Trigger(() -> m_shooter.getTemperature() > 50)
    .onTrue(leds.warningCommand())
    .onFalse(leds.readyCommand());

  // Celebrate on score
  m_scorer.scoredTrigger()
    .onTrue(leds.celebrateCommand());
}
```

### Command Composition

Combine LED commands with mechanism commands:

```java
// Intake with visual feedback
public Command intakeCommand() {
  return Commands.parallel(
    m_intake.runCommand(),
    leds.pulseCommand(LEDConstants.orange, 2.0)
  ).until(m_intake::hasGamePiece)
   .andThen(leds.blinkCommand(LEDConstants.green, 2));
}

// Shoot sequence with status
public Command shootCommand() {
  return Commands.sequence(
    // Aiming
    Commands.parallel(
      m_shooter.aimCommand(),
      leds.pulseCommand(LEDConstants.yellow, 1.5)
    ),
    // Ready to shoot
    leds.readyCommand(),
    Commands.waitSeconds(0.2),
    // Shooting
    Commands.parallel(
      m_shooter.fireCommand(),
      leds.strobeCommand(LEDConstants.red, 0.5)
    ),
    // Success
    leds.celebrateCommand()
  );
}

// Auto routine with status
public Command autoRoutine() {
  return Commands.sequence(
    leds.busyCommand(),
    // Drive to position
    driveToPositionCommand()
      .deadlineWith(leds.setAnimationCommand(AnimationTypes.Larson)),
    // Score
    scoreCommand()
      .alongWith(leds.pulseCommand(LEDConstants.green, 1.0)),
    // Celebrate
    leds.celebrateCommand()
  );
}
```

### Default Commands

Set default behavior based on robot mode:

```java
// In RobotContainer constructor
private void configureDefaultCommands() {
  // Show alliance color by default in teleop
  leds.setDefaultCommand(
    Commands.either(
      leds.allianceColorCommand(),
      leds.disabledCommand(),
      DriverStation::isEnabled
    )
  );
}

// Or use state machine pattern
private void configureDefaultCommands() {
  leds.setDefaultCommand(
    Commands.run(() -> {
      if (DriverStation.isDisabled()) {
        leds.setColor(new Color(0.3, 0.3, 0));  // Dim yellow
      } else if (m_intake.hasGamePiece()) {
        leds.setColor(LEDConstants.green);
      } else {
        leds.setColor(
          DriverStation.getAlliance()
            .map(a -> a == Alliance.Red ? LEDConstants.red : LEDConstants.blue)
            .orElse(LEDConstants.white)
        );
      }
    }, leds)
  );
}
```

### Complex Sequences

Build multi-step LED sequences:

```java
// Startup sequence
public Command startupSequence() {
  return Commands.sequence(
    leds.setAnimationCommand(AnimationTypes.Rainbow),
    Commands.waitSeconds(1.0),
    leds.blinkCommand(LEDConstants.white, 3),
    leds.allianceColorCommand()
  ).withName("StartupSequence");
}

// Endgame warning sequence
public Command endgameWarning() {
  return Commands.sequence(
    // 30 seconds left
    Commands.waitUntil(() -> DriverStation.getMatchTime() < 30),
    leds.blinkCommand(LEDConstants.yellow, 3),

    // 15 seconds left
    Commands.waitUntil(() -> DriverStation.getMatchTime() < 15),
    leds.blinkCommand(LEDConstants.orange, 5),

    // 5 seconds left
    Commands.waitUntil(() -> DriverStation.getMatchTime() < 5),
    leds.strobeCommand(LEDConstants.red, 5.0)
  ).withName("EndgameWarning");
}

// Victory dance
public Command victorySequence() {
  return Commands.sequence(
    leds.celebrateCommand(),
    leds.setAnimationCommand(AnimationTypes.Rainbow),
    Commands.waitSeconds(3.0),
    leds.setAnimationCommand(AnimationTypes.Twinkle),
    Commands.waitSeconds(2.0),
    leds.setColorCommand(LEDConstants.green)
  ).withName("VictorySequence");
}
```

### Button Bindings

Bind LED controls to operator input:

```java
private void configureButtonBindings() {
  // Driver controller
  m_driverController.a().onTrue(leds.setColorCommand(LEDConstants.green));
  m_driverController.b().onTrue(leds.setColorCommand(LEDConstants.blue));
  m_driverController.x().onTrue(leds.setColorCommand(LEDConstants.yellow));
  m_driverController.y().onTrue(leds.turnOffCommand());

  // Operator controller - animations
  m_operatorController.povUp().onTrue(
    leds.setAnimationCommand(AnimationTypes.Rainbow));
  m_operatorController.povDown().onTrue(
    leds.setAnimationCommand(AnimationTypes.Fire));
  m_operatorController.povLeft().onTrue(
    leds.setAnimationCommand(AnimationTypes.Larson));
  m_operatorController.povRight().onTrue(
    leds.allianceColorCommand());

  // Hold button for strobe
  m_operatorController.rightBumper()
    .whileTrue(leds.setAnimationCommand(AnimationTypes.Strobe))
    .onFalse(leds.allianceColorCommand());
}
```

### Cross-Subsystem Coordination

Coordinate LEDs with other subsystems:

```java
// Show intake status
m_intake.hasGamePieceTrigger()
  .whileTrue(leds.setColorCommand(LEDConstants.green))
  .onFalse(leds.allianceColorCommand());

// Show shooter ready
m_shooter.atSetpointTrigger()
  .and(() -> m_intake.hasGamePiece())
  .whileTrue(leds.readyCommand())
  .onFalse(leds.pulseCommand(LEDConstants.yellow, 999));  // Continuous

// Show climber deployed
m_climber.deployedTrigger()
  .whileTrue(leds.setColorCommand(LEDConstants.purple))
  .onFalse(leds.allianceColorCommand());

// Show vision lock
m_vision.hasTargetTrigger()
  .whileTrue(leds.blinkCommand(LEDConstants.green, 999))  // Continuous
  .onFalse(leds.allianceColorCommand());

// Error states
m_intake.faultTrigger()
  .or(m_shooter.faultTrigger())
  .or(m_climber.faultTrigger())
  .onTrue(leds.errorCommand());
```

## Best Practices

### 1. Use Triggers for State Exposure

✅ **Good - Expose state as triggers:**
```java
// In subsystem
public Trigger hasGamePieceTrigger() {
  return new Trigger(this::hasGamePiece);
}

// In RobotContainer
m_intake.hasGamePieceTrigger()
  .whileTrue(leds.setColorCommand(LEDConstants.green));
```

❌ **Bad - Query state in command:**
```java
// Don't do this
Commands.run(() -> {
  if (m_intake.hasGamePiece()) {
    leds.setColor(LEDConstants.green);
  }
}, leds, m_intake);
```

### 2. Use Command Factories

✅ **Good - Use provided factories:**
```java
leds.celebrateCommand().schedule();
controller.a().onTrue(leds.readyCommand());
```

❌ **Bad - Create commands manually:**
```java
// Don't do this
Commands.runOnce(() -> leds.setAnimation(AnimationTypes.Rainbow), leds)
        .andThen(Commands.waitSeconds(2.0))
        .andThen(Commands.runOnce(() -> leds.setColor(LEDConstants.green), leds));
```

### 3. Name Commands for Debugging

Command factories automatically name commands:
```java
// Already named "SetColor(Green)"
leds.setColorCommand(LEDConstants.green);

// Already named "Celebrate"
leds.celebrateCommand();

// Add names to custom compositions
Commands.sequence(
  leds.readyCommand(),
  shootCommand()
).withName("ReadyAndShoot");
```

### 4. Use `.finallyDo()` for Cleanup

Ensure LEDs return to known state:
```java
// Reset after temporary animation
leds.setAnimationCommand(AnimationTypes.Rainbow)
    .withTimeout(2.0)
    .finallyDo(() -> leds.setColor(LEDConstants.green).schedule());

// Cleanup after command
intakeCommand()
  .finallyDo(() -> leds.allianceColorCommand().schedule());
```

### 5. Avoid Conflicting Commands

Only one command can control LEDs at a time:
```java
// ✅ Good - Sequential
Commands.sequence(
  leds.blinkCommand(LEDConstants.red, 3),
  leds.setColorCommand(LEDConstants.green)
);

// ❌ Bad - Parallel conflict
Commands.parallel(
  leds.blinkCommand(LEDConstants.red, 3),
  leds.setColorCommand(LEDConstants.green)  // Conflicts!
);
```

### 6. Use Default Commands Wisely

Set sensible defaults:
```java
// Good default - shows alliance color when not doing anything else
leds.setDefaultCommand(leds.allianceColorCommand());

// Better default - considers robot state
leds.setDefaultCommand(
  Commands.either(
    leds.allianceColorCommand(),
    leds.disabledCommand(),
    DriverStation::isEnabled
  )
);
```

## Common Patterns

### Pattern: Progressive Warning

Increase urgency as time decreases:
```java
public Command progressiveEndgameWarning() {
  return Commands.sequence(
    Commands.waitUntil(() -> DriverStation.getMatchTime() < 45),
    leds.pulseCommand(LEDConstants.yellow, 2.0),

    Commands.waitUntil(() -> DriverStation.getMatchTime() < 30),
    leds.blinkCommand(LEDConstants.yellow, 3),

    Commands.waitUntil(() -> DriverStation.getMatchTime() < 15),
    leds.blinkCommand(LEDConstants.orange, 5),

    Commands.waitUntil(() -> DriverStation.getMatchTime() < 5),
    leds.strobeCommand(LEDConstants.red, 5.0)
  );
}
```

### Pattern: Status Pipeline

Show status of multi-step operation:
```java
public Command scoringPipeline() {
  return Commands.sequence(
    // Intaking
    Commands.parallel(
      intakeCommand(),
      leds.pulseCommand(LEDConstants.orange, 999)
    ).until(m_intake::hasGamePiece),
    leds.blinkCommand(LEDConstants.green, 2),  // Acquired

    // Aiming
    Commands.parallel(
      aimCommand(),
      leds.pulseCommand(LEDConstants.yellow, 999)
    ).until(m_shooter::atSetpoint),
    leds.readyCommand(),  // Ready

    // Shooting
    Commands.parallel(
      shootCommand(),
      leds.strobeCommand(LEDConstants.red, 1.0)
    ),

    // Success
    leds.celebrateCommand()
  );
}
```

### Pattern: Conditional Feedback

Different feedback based on success:
```java
public Command attemptScore() {
  return scoreCommand()
    .finallyDo((interrupted) -> {
      if (interrupted) {
        leds.errorCommand().schedule();
      } else if (scoringSucceeded()) {
        leds.celebrateCommand().schedule();
      } else {
        leds.warningCommand().schedule();
      }
    });
}
```

## See Also

- [CANdleSubsystem Documentation](CANdleSubsystem.md)
- [Command Best Practices](../../COMMAND_BEST_PRACTICES.md)
- [WPILib Command-Based](https://docs.wpilib.org/en/stable/docs/software/commandbased/index.html)
- [Subsystems Overview](README.md)

---

**Following Best Practices:** This guide demonstrates command-based patterns from COMMAND_BEST_PRACTICES.md, including trigger-based state exposure, command factories, and composition patterns.
