# FRC Command-Based Programming Best Practices

> Based on [bovlb's FRC Tips](https://bovlb.github.io/frc-tips/commands/best-practices.html)

## Core Principles

The three fundamental principles for command-based programming:

1. **Control subsystems using command factories**
2. **Get information from subsystems using triggers**
3. **Coordinate between subsystems by binding commands to triggers**

These practices reduce dependencies between subsystems and centralize cross-subsystem behavior in `RobotContainer`.

---

## 1. Command Factories

### What Are Command Factories?

Command factories are methods that return new command instances representing basic subsystem actions. They should be the **only** way to control a subsystem.

### Best Practices

✅ **DO:**
- Name commands using problem-domain language (`startShooting`, `intakeGamepiece`)
- Express purpose from the subsystem's perspective, not implementation details
- Accept `Supplier` objects for dynamic configuration
- Use `run()` for continuous commands
- Use `runOnce()` for instant commands
- Share implementation using private helper methods
- Keep motors, controllers, and state **private**

❌ **DON'T:**
- Expose hardware details in command names (`setMotorSpeed`, `setPWM`)
- Accept raw values when suppliers would allow dynamic updates
- Make motors or state variables public

### Examples

#### Basic Command Factory Pattern

```java
public class ShooterSubsystem extends SubsystemBase {
    private ShooterMode m_mode = ShooterMode.IDLE;
    private final TalonFX m_shooterMotor;

    // Private helper method for shared implementation
    private Command setState(ShooterMode mode) {
        return runOnce(() -> m_mode = mode).withName(mode.toString());
    }

    // Public command factories
    public Command startShooting() {
        return setState(ShooterMode.SHOOTING);
    }

    public Command stopShooting() {
        return setState(ShooterMode.IDLE);
    }

    public Command ejectGamePiece() {
        return setState(ShooterMode.EJECTING);
    }
}
```

#### Using Suppliers for Dynamic Configuration

```java
public class DriveSubsystem extends SubsystemBase {
    // ✅ GOOD: Accepts suppliers for dynamic updates
    public Command drive(
            DoubleSupplier xSpeed,
            DoubleSupplier ySpeed,
            DoubleSupplier rotation) {
        return run(() -> {
            // Values update every iteration
            m_drive.drive(xSpeed.getAsDouble(),
                         ySpeed.getAsDouble(),
                         rotation.getAsDouble());
        });
    }

    // ❌ BAD: Accepts raw values (won't update dynamically)
    public Command driveFixed(double xSpeed, double ySpeed, double rotation) {
        return run(() -> m_drive.drive(xSpeed, ySpeed, rotation));
    }
}
```

#### Continuous vs Instant Commands

```java
// Continuous command - runs until interrupted
public Command intakeGamePiece() {
    return run(() -> m_motor.set(0.8))
        .withName("IntakeGamePiece");
}

// Instant command - completes immediately
public Command extendArm() {
    return runOnce(() -> m_solenoid.set(true))
        .withName("ExtendArm");
}
```

---

## 2. Triggers

### What Are Triggers?

Triggers are wrappers for `BooleanSupplier` objects that automatically schedule commands when conditions change. They replace polling and manual state checking.

### Best Practices

✅ **DO:**
- Expose state as yes/no questions using problem-domain language
- Ask "what is the system doing?" not "what is the hardware reading?"
- Use `debounce()` to prevent flickering
- Create separate triggers for each possible mode
- Use `MathUtil.isNear()` for comparing doubles

❌ **DON'T:**
- Expose raw sensor values (positions, speeds, voltages)
- Use direct equality checks on doubles
- Create triggers that flicker rapidly

### Examples

#### Declaring Triggers

```java
public class ShooterSubsystem extends SubsystemBase {
    private ShooterMode m_mode = ShooterMode.IDLE;
    private final TalonFX m_shooterMotor;

    // Expose state as problem-domain questions
    public final Trigger isShooting =
        new Trigger(() -> m_mode == ShooterMode.SHOOTING);

    public final Trigger isIdle =
        new Trigger(() -> m_mode == ShooterMode.IDLE);

    // Use debounce to prevent premature termination
    public final Trigger isReady = new Trigger(this::isReady)
        .debounce(0.25, Debouncer.DebounceType.kFalling);

    private boolean isReady() {
        double currentSpeed = m_shooterMotor.getVelocity().getValueAsDouble();
        return MathUtil.isNear(m_targetSpeed, currentSpeed, 50.0);
    }
}
```

#### Using Triggers with Sensors

```java
public class IntakeSubsystem extends SubsystemBase {
    private final DigitalInput m_beamBreak;

    // ✅ GOOD: Problem-domain question
    public final Trigger hasGamePiece =
        new Trigger(() -> !m_beamBreak.get())
            .debounce(0.1);

    // ❌ BAD: Exposing raw sensor reading
    public boolean getBeamBreakValue() {
        return m_beamBreak.get();
    }
}
```

---

## 3. Trigger Binding and Coordination

### Combining Triggers

Use logical operators to create complex conditions:

```java
// In RobotContainer
private void configureBindings() {
    // Combine multiple subsystem triggers
    m_shooter.isShooting
        .and(m_intake.hasGamePiece)
        .and(m_vision.isInRange)
        .and(m_shooter.isReady)
        .whileTrue(m_feeder.feedGamePiece());

    // Use OR for alternative conditions
    m_intake.isJammed
        .or(m_intake.isOverheated)
        .onTrue(m_intake.stopIntake()
            .andThen(new WaitCommand(2.0))
            .andThen(m_intake.ejectGamePiece()));

    // Negate trigger logic
    m_shooter.isReady
        .negate()
        .whileTrue(m_led.setPattern(LEDPattern.NOT_READY));
}
```

### Binding Methods

Choose the right binding method for your use case:

| Method | Use Case | Example |
|--------|----------|---------|
| `onTrue()` | Instant commands that complete immediately | Toggle solenoid, change mode |
| `whileTrue()` | Commands that run while condition is true | Run intake, aim at target |
| `toggleOnTrue()` | Toggle commands on/off (mainly for driver buttons) | Toggle climb lock |

```java
// onTrue: Instant command
m_driver.a()
    .onTrue(m_intake.extendIntake());

// whileTrue: Continuous command
m_driver.rightBumper()
    .whileTrue(m_intake.intakeGamePiece());

// toggleOnTrue: Toggle behavior
m_driver.leftBumper()
    .toggleOnTrue(m_climber.lockClimb());
```

### Cross-Subsystem Coordination Example

```java
public class RobotContainer {
    private final DriveSubsystem m_drive = new DriveSubsystem();
    private final ShooterSubsystem m_shooter = new ShooterSubsystem();
    private final IntakeSubsystem m_intake = new IntakeSubsystem();
    private final VisionSubsystem m_vision = new VisionSubsystem();

    private void configureBindings() {
        // Operator wants to shoot
        m_operator.a()
            .onTrue(m_shooter.startShooting());

        // Stop shooting when idle
        m_shooter.isIdle
            .onTrue(m_shooter.stopShooting());

        // Auto-feed when everything is ready
        m_shooter.isShooting
            .and(m_intake.hasGamePiece)
            .and(m_vision.canSeeTarget)
            .and(m_shooter.isReady)
            .whileTrue(m_intake.feedToShooter());

        // Visual feedback
        m_shooter.isReady
            .and(m_vision.canSeeTarget)
            .whileTrue(m_led.setPattern(LEDPattern.READY_TO_SHOOT))
            .onFalse(m_led.setPattern(LEDPattern.DEFAULT));
    }
}
```

---

## 4. Default Commands

### Best Practices

✅ **DO:**
- Keep default commands simple (stop motors, hold position)
- Use existing command factories
- Pass suppliers from driver controls

❌ **DON'T:**
- Add complex decision logic to default commands
- Create inline commands with lots of logic

### Examples

#### Simple Default Command

```java
public RobotContainer() {
    // Simple: just stop the shooter when not in use
    m_shooter.setDefaultCommand(m_shooter.stopShooting());

    // Drive with joystick suppliers
    m_drive.setDefaultCommand(m_drive.drive(
        () -> -adjustJoystick(m_driver.getLeftY()),
        () -> -adjustJoystick(m_driver.getLeftX()),
        () -> -adjustJoystick(m_driver.getRightX())
    ));
}

private double adjustJoystick(double value) {
    // Apply deadband and squaring
    value = MathUtil.applyDeadband(value, 0.1);
    return Math.copySign(value * value, value);
}
```

---

## 5. Autonomous Commands

### PathPlanner Integration

Use `NamedCommands.registerCommand()` with triggers to determine when commands should complete:

```java
public RobotContainer() {
    // Register named commands for PathPlanner
    NamedCommands.registerCommand("IntakeGamePiece",
        m_intake.intakeGamePiece().asProxy()
            .until(m_intake.hasGamePiece)
            .withTimeout(3.0)
            .withName("IntakeGamePiece"));

    NamedCommands.registerCommand("Shoot",
        m_shooter.startShooting().asProxy()
            .until(m_shooter.isIdle)
            .withName("Shoot"));

    NamedCommands.registerCommand("AimAndShoot",
        m_shooter.startShooting()
            .alongWith(m_drive.aimAtTarget())
            .asProxy()
            .until(m_shooter.isShooting
                .and(m_shooter.isReady)
                .and(m_vision.canSeeTarget))
            .andThen(m_intake.feedToShooter())
            .until(m_intake.hasGamePiece.negate())
            .withName("AimAndShoot"));
}
```

**Important:** Use `asProxy()` to prevent command interruption conflicts during autonomous.

---

## 6. Subsystem Periodic Methods

### Modern Best Practices

Limit `periodic()` to three categories:

1. **Input caching** - Read sensors once per iteration
2. **Odometry updates** - Update pose estimation
3. **Logging** - Send data to dashboards

```java
public class DriveSubsystem extends SubsystemBase {
    private final SwerveDrive m_drive;
    private final PoseEstimator m_poseEstimator;

    // Cache sensor values
    private SwerveModulePosition[] m_modulePositions;
    private Rotation2d m_gyroAngle;

    @Override
    public void periodic() {
        // 1. Input caching
        m_modulePositions = m_drive.getModulePositions();
        m_gyroAngle = m_gyro.getRotation2d();

        // 2. Odometry updates
        m_poseEstimator.update(m_gyroAngle, m_modulePositions);

        // 3. Logging
        SmartDashboard.putNumber("Drive/Heading", m_gyroAngle.getDegrees());
        SmartDashboard.putString("Drive/Pose", m_poseEstimator.getEstimatedPosition().toString());
    }
}
```

**Offload to Commands:** Control loops and decision-making belong in commands and triggers, not periodic methods.

---

## 7. Advanced Patterns

### Exception: Pose Estimation

When multiple subsystems need non-boolean communication, expose problem-space concepts:

```java
public class VisionSubsystem extends SubsystemBase {
    // Public supplier for other subsystems to access
    public final Supplier<Pose2d> pose = this::getEstimatedPose;

    // Public consumer for accepting vision measurements
    public final Consumer<VisionMeasurement> addVisionMeasurement =
        this::addVisionMeasurementInternal;

    private Pose2d getEstimatedPose() {
        return m_poseEstimator.getEstimatedPosition();
    }

    private void addVisionMeasurementInternal(VisionMeasurement measurement) {
        m_poseEstimator.addVisionMeasurement(
            measurement.pose(),
            measurement.timestamp()
        );
    }
}
```

### Performance: Caching Expensive Operations

Cache frequently-accessed values to avoid redundant calculations:

```java
public class ShooterSubsystem extends SubsystemBase {
    private boolean m_cachedIsReady;

    @Override
    public void periodic() {
        // Cache expensive calculation once per iteration
        m_cachedIsReady = calculateIsReady();
    }

    public final Trigger isReady = new Trigger(() -> m_cachedIsReady);

    private boolean calculateIsReady() {
        // Expensive calculation here
        double velocity = m_motor.getVelocity().getValueAsDouble();
        double temperature = m_motor.getDeviceTemp().getValueAsDouble();
        return MathUtil.isNear(velocity, m_targetSpeed, 50.0)
            && temperature < 60.0;
    }
}
```

---

## 8. Incremental Adoption

You don't need to adopt everything at once. Start incrementally:

### Phase 1: Command Factories
Replace direct subsystem control with command factories. This alone improves testability and encapsulation.

```java
// Before: Direct control in RobotContainer
m_driver.a().onTrue(Commands.runOnce(() -> {
    m_shooter.setMode(ShooterMode.SHOOTING);
}));

// After: Command factory
m_driver.a().onTrue(m_shooter.startShooting());
```

### Phase 2: Add Triggers
Decouple subsystems by exposing state as triggers instead of getters.

```java
// Before: Polling in commands
public Command autoShoot() {
    return Commands.waitUntil(() -> m_shooter.getSpeed() > 5000)
        .andThen(m_feeder.feed());
}

// After: Using triggers
public Command autoShoot() {
    return Commands.waitUntil(m_shooter.isReady)
        .andThen(m_feeder.feed());
}
```

### Phase 3: Move Complexity to RobotContainer
Move cross-subsystem coordination from commands to trigger bindings.

```java
// Before: Complex command with subsystem dependencies
public Command shootSequence() {
    return new SequentialCommandGroup(
        m_shooter.spinUp(),
        Commands.waitUntil(() -> m_shooter.isReady() && m_vision.canSee()),
        m_feeder.feed()
    );
}

// After: Trigger binding in RobotContainer
m_shooter.isReady
    .and(m_vision.canSeeTarget)
    .and(m_shooter.isShooting)
    .whileTrue(m_feeder.feed());
```

---

## Quick Reference

### Checklist for Subsystem Design

- [ ] All motors, sensors, and state variables are private
- [ ] Control is only possible through command factories
- [ ] Command factories use problem-domain names
- [ ] Commands accept `Supplier` objects for dynamic values
- [ ] State is exposed through triggers, not getters
- [ ] Triggers use `debounce()` where appropriate
- [ ] Triggers compare doubles with `MathUtil.isNear()`
- [ ] `periodic()` only handles caching, odometry, and logging
- [ ] Cross-subsystem coordination happens in `RobotContainer`

### Common Patterns

```java
// Command factory returning instant command
public Command extendIntake() {
    return runOnce(() -> m_solenoid.set(true))
        .withName("ExtendIntake");
}

// Command factory returning continuous command
public Command intakeGamePiece() {
    return run(() -> m_motor.set(0.8))
        .withName("IntakeGamePiece");
}

// Trigger with debounce
public final Trigger hasGamePiece =
    new Trigger(() -> !m_beamBreak.get())
        .debounce(0.1);

// Complex trigger binding
m_subsystemA.triggerA
    .and(m_subsystemB.triggerB)
    .and(m_subsystemC.triggerC.negate())
    .whileTrue(m_subsystemD.doSomething());

// Named command for auto
NamedCommands.registerCommand("Shoot",
    m_shooter.startShooting().asProxy()
        .until(m_shooter.isIdle)
        .withName("Shoot"));
```

---

## Additional Resources

- [Original Article](https://bovlb.github.io/frc-tips/commands/best-practices.html) by bovlb
- [WPILib Command-Based Programming](https://docs.wpilib.org/en/stable/docs/software/commandbased/index.html)
- [WPILib Trigger Documentation](https://docs.wpilib.org/en/stable/docs/software/commandbased/binding-commands-to-triggers.html)

---

*This document is intended as a reference for Team 245 Adambots. Update and adapt these practices as the team's needs evolve.*
