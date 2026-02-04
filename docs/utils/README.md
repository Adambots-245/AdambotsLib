# Utils

Utility classes and helpers for common FRC programming patterns.

## Overview

This folder contains utility classes that provide reusable solutions for common programming challenges in FRC robotics. These utilities follow best practices and reduce boilerplate code in robot projects.

## Available Utilities

### State Management

- **[StateMachine](StateMachine.md)** - Type-safe state machine for managing subsystem behavior
  - Builder pattern API with automatic mode detection
  - Immediate and conditional transitions
  - State-property maps for cleaner code
  - Transition listeners and invalid transition validation
  - Used for arms, intakes, shooters, and other complex mechanisms

### Controller Input

- **[Buttons](Buttons.md)** - Unified controller input abstraction
  - Xbox, PS5, and Logitech Extreme 3D Pro support
  - Configurable controller types for driver and operator
  - Input curves (linear, cubic, sigmoid) for smooth control
  - Automatic deadzone application
  - All buttons as WPILib Trigger objects
  - Thread-safe rumble with automatic shutoff
  - Pre-built drive input suppliers
  - **Advanced trigger patterns** (double-press, continuous scheduling) *

### Dashboard & Telemetry

- **[Dash](Dash.md)** - Simplified Shuffleboard interface
  - Clean API for adding telemetry data
  - Auto-updating values every robot loop
  - Multi-tab organization for better layout
  - Tunable values for live PID tuning
  - Command buttons for testing subsystems
  - Support for numbers, booleans, strings, and arrays

### PID & Control

- **[PIDAutoTuner](PIDAutoTuner.md)** - Automatic PID tuning utility
  - Relay Feedback Method (Ziegler-Nichols) for automatic gain calculation
  - Simple API with method references (`motor::getPosition`, `motor::setOutput`)
  - Feedforward calculation (kV, kS) via step response analysis
  - Safety features: range limits, timeouts, automatic motor stop
  - Dashboard integration for result display
  - Command-based for easy button binding

### General Utilities

- **[Utils](Utils.md)** - Comprehensive utility functions for common FRC tasks
  - Alliance utilities (check color, mirror poses for red/blue)
  - Math utilities (lerp, map range, percent progress)
  - Angle utilities (normalize, shortest angular difference)
  - Boolean utilities (debounce, rising/falling edge detection)
  - Array utilities (average, min, max, index finding)
  - Time utilities (elapsed checks, conversions)
  - Error reporting (standardized DriverStation messages)
  - **Epsilon comparison** (floating-point tolerance comparisons) *
  - **Geometry conversions** (Pose2d/Transform2d, Twist2d operations) *

### Performance Profiling

- **LoggedTracer** - Performance timing utility for profiling code execution *
  - Measure time between execution points
  - Log durations to NetworkTables for analysis
  - Identify slow subsystems causing loop overruns
  - Debug periodic loop timing issues

### Commands

- **SuppliedWaitCommand** - Dynamic wait command for sequences *
  - Accept duration from DoubleSupplier instead of fixed value
  - Tunable autonomous timing
  - Sensor-based wait durations
  - Game state conditional delays

*Adapted from FRC Team 6328 Mechanical Advantage ([source](https://github.com/Mechanical-Advantage/RobotCode2026Public))

---

## When to Use These Utilities

### Use StateMachine When:
- Your subsystem has multiple distinct operating modes
- You need to manage complex state transitions
- You want to ensure safe transitions between states
- You need logging and debugging of state changes

**Common Use Cases:**
- Arm subsystems with multiple positions
- Intake mechanisms with different modes
- Shooter subsystems with spin-up sequences
- Elevators with height presets

### Use Buttons When:
- You need consistent controller input handling across your robot
- You want to support multiple controller types (Xbox, PS5, joystick)
- You need advanced input processing (curves, deadzones)
- You want centralized trigger definitions for command binding
- You need rumble feedback capabilities
- You want pre-built suppliers for swerve/tank drive

**Common Use Cases:**
- Swerve drive with field-oriented control
- Tank drive with dual joysticks
- Command bindings for all subsystems
- Haptic feedback for driver awareness
- Mixed controller types (Xbox + PS5, Xbox + joystick)

### Use Dash When:
- You need to display telemetry data (speeds, positions, states)
- You want to tune PID constants live during testing
- You need debug buttons for testing subsystems
- You want to organize data across multiple Shuffleboard tabs
- You prefer simple API over direct Shuffleboard/NetworkTables complexity

**Common Use Cases:**
- Display robot speed, position, and heading
- Live PID tuning for shooters, arms, and elevators
- Debug command buttons (reset gyro, deploy intake, etc.)
- Organize telemetry by subsystem across tabs
- Monitor sensor values and motor currents

### Use PIDAutoTuner When:
- You need a starting point for PID gains on a new mechanism
- Manual PID tuning is taking too long
- You want reproducible tuning results
- You're tuning velocity control and need feedforward values

**Common Use Cases:**
- Turret position control
- Arm angle control
- Elevator position control
- Shooter flywheel velocity control
- Conveyor velocity control

### Use Utils When:
- You need alliance-specific logic (mirror poses, check colors)
- You need angle normalization or shortest angular difference
- You need to debounce sensors or detect button edges
- You need array operations (average, min/max)
- You need simple time checks or conversions
- You want standardized error reporting

**Common Use Cases:**
- Mirror field positions for red/blue alliances
- Calculate shortest path for angle control (critical!)
- Debounce noisy game piece sensors
- Find closest target from multiple options
- Check if mechanism is in safe range
- Calculate progress percentage for LED animations
- Detect limit switch activation (rising edge)

---

## Philosophy

Utilities in AdambotsLib follow these principles:

### 1. Reduce Boilerplate
Utilities eliminate repetitive code patterns:
```java
// Without StateMachine - lots of manual state tracking
private ArmState currentState = ArmState.STOWED;
private ArmState targetState = ArmState.STOWED;
private boolean transitioning = false;
// ... lots more manual tracking

// With StateMachine - handled automatically
private final StateMachine<ArmState, ArmProps> sm;
```

### 2. Type Safety
Use Java's type system to catch errors at compile time:
```java
// StateMachine uses generics for type safety
StateMachine<ArmState, ArmProps> sm = new StateMachine<>(...);
sm.to(ArmState.HIGH)  // Compile-time check - must be ArmState
   .withProperties(new ArmProps(...))  // Must be ArmProps
```

### 3. Fail-Fast
Clear error messages when something goes wrong:
```java
sm.addInvalidTransition(State.STOWED, State.ERROR,
    "Cannot transition to ERROR from STOWED");

// Later, attempting this transition gives clear error:
// IllegalStateException: Cannot transition to ERROR from STOWED
```

### 4. Self-Documenting
Code should be readable and clear in intent:
```java
// Old API - what does () -> true mean?
sm.requestTransition(State.HIGH, props, () -> true, action);

// New API - clear and obvious
sm.to(State.HIGH).executing(action).request();
```

---

## Usage Example

### StateMachine in Arm Subsystem

```java
public class ArmSubsystem extends SubsystemBase {
    enum State { STOWED, LOW, MID, HIGH }
    record ArmProps(double angleDeg, double speed) {}

    private final TalonFXMotor motor;
    private final StateMachine<State, ArmProps> sm;

    public ArmSubsystem() {
        motor = new TalonFXMotor(1, false, 40, false);

        Map<State, ArmProps> stateMap = Map.of(
            State.STOWED, new ArmProps(0, 0.5),
            State.LOW, new ArmProps(30, 0.6),
            State.MID, new ArmProps(60, 0.7),
            State.HIGH, new ArmProps(90, 0.8)
        );

        sm = new StateMachine<>(State.STOWED, stateMap, System.out::println);
    }

    public Command goToHigh() {
        return runOnce(() ->
            sm.to(State.HIGH)
              .executing(p -> motor.set(ControlMode.POSITION, p.angleDeg()))
              .request()
        );
    }

    @Override
    public void periodic() {
        sm.periodic();
    }
}
```

---

## Best Practices

### 1. Choose the Right Tool

Not every subsystem needs a state machine:

**Good candidates for StateMachine:**
- Multiple distinct states (3+)
- Complex transition logic
- Safety requirements (invalid transitions)
- Need for state logging/debugging

**Simple subsystems may not need it:**
- Single on/off motor
- Continuous control with no discrete states
- Simple setpoint following

### 2. Document State Meanings

```java
enum ArmState {
    STOWED,       // Inside robot frame
    LOW_GOAL,     // 30° for low scoring
    MID_GOAL,     // 60° for mid scoring
    HIGH_GOAL     // 90° for high scoring
}
```

### 3. Use Transition Listeners

```java
sm.addTransitionListener((from, to) ->
    DataLogManager.log("Arm: " + from + " -> " + to)
);
```

### 4. Publish State to SmartDashboard

```java
@Override
public void periodic() {
    sm.periodic();
    SmartDashboard.putString("Arm State", sm.getCurrentState().toString());
}
```

---

## Integration with Robot Code

### Initialize Buttons in Robot.java

```java
public class Robot extends TimedRobot {
    private RobotContainer m_robotContainer;

    @Override
    public void robotInit() {
        // Initialize Buttons BEFORE RobotContainer
        Buttons.init(0, 1, ControllerType.XBOX, ControllerType.PS5);

        // Now create RobotContainer
        m_robotContainer = new RobotContainer();
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }
}
```

### Use in RobotContainer

```java
public class RobotContainer {
    private final SwerveSubsystem swerve = new SwerveSubsystem();
    private final ArmSubsystem arm = new ArmSubsystem();

    public RobotContainer() {
        configureDefaultCommands();
        configureButtonBindings();
    }

    private void configureDefaultCommands() {
        // Swerve drive with cubic curves
        swerve.setDefaultCommand(
            swerve.driveCommand(
                Buttons.createForwardSupplier(0.05, InputCurve.CUBIC),
                Buttons.createStrafeSupplier(0.05, InputCurve.CUBIC),
                Buttons.createRotationSupplier(0.1, InputCurve.SIGMOID)
            )
        );
    }

    private void configureButtonBindings() {
        // Arm control with state machine
        Buttons.XboxA.onTrue(arm.goToStowed());
        Buttons.XboxB.onTrue(arm.goToLow());
        Buttons.XboxY.onTrue(arm.goToHigh());

        // Rumble when arm reaches target
        arm.isAtTargetTrigger().onTrue(
            Commands.runOnce(() ->
                Buttons.rumbleXbox(Buttons.getXboxController(), 200, 0.5)
            )
        );
    }
}
```

### State-Based LED Feedback

```java
// Show different LED colors based on arm state
arm.isAtTargetTrigger()
    .whileTrue(leds.setColorCommand(Color.kGreen))
    .whileFalse(leds.setColorCommand(Color.kOrange));
```

---

## Testing

Test utilities in simulation or on a practice robot:

```java
if (RobotBase.isSimulation()) {
    // StateMachine works in simulation
    // Use SmartDashboard to verify state transitions
}
```

---

## Utilities from Mechanical Advantage

Several utilities in AdambotsLib are adapted from **FRC Team 6328 Mechanical Advantage**.
These are marked with * in the feature lists above. Licensed under MIT License.

**Source:** [github.com/Mechanical-Advantage/RobotCode2026Public](https://github.com/Mechanical-Advantage/RobotCode2026Public)

### Epsilon Comparison (Utils)

Floating-point comparison with tolerance:

```java
// Compare doubles with default epsilon (1e-9)
if (Utils.epsilonEquals(motorPosition, targetPosition)) {
    // Motor has reached target
}

// Compare with custom epsilon
if (Utils.epsilonEquals(a, b, 0.001)) {
    // Values are within 0.001 of each other
}

// Compare Twist2d objects
if (Utils.epsilonEquals(currentTwist, targetTwist)) {
    // Velocities match
}
```

### Geometry Utilities (Utils)

Pose/Transform conversions and operations:

```java
// Convert between Pose2d and Transform2d
Transform2d transform = Utils.toTransform2d(pose);
Pose2d pose = Utils.toPose2d(transform);

// Compute inverse pose
Pose2d robotPose = odometry.getPose();
Pose2d inversePose = Utils.inverse(robotPose);

// Scale a Twist2d
Twist2d halfVelocity = Utils.multiply(velocity, 0.5);

// Convert ChassisSpeeds to Twist2d
Twist2d twist = Utils.toTwist2d(chassisSpeeds);

// Modify pose components
Pose2d newPose = Utils.withRotation(pose, newRotation);
Pose2d newPose2 = Utils.withTranslation(pose, newTranslation);
```

### LoggedTracer

Profile code execution by measuring time between checkpoints:

```java
public void robotPeriodic() {
    LoggedTracer.reset();

    swerve.periodic();
    LoggedTracer.record("Swerve");  // Logs DrivetrainMS to NetworkTables

    vision.periodic();
    LoggedTracer.record("Vision");  // Logs VisionMS to NetworkTables

    arm.periodic();
    LoggedTracer.record("Arm");     // Logs ArmMS to NetworkTables
}
```

View timing data in AdvantageScope or SmartDashboard under "LoggedTracer/".

### Advanced Trigger Patterns (Buttons)

Double-press detection:

```java
// Require double-press to reset gyro (prevents accidents)
Trigger resetGyro = Buttons.doublePress(Buttons.XboxStartButton);
resetGyro.onTrue(Commands.runOnce(() -> gyro.reset()));

// Double-press with custom timeout (default is 0.4 seconds)
Trigger turboMode = Buttons.doublePress(Buttons.XboxAButton, 0.3);
```

Continuous command scheduling:

```java
// Re-schedule command every loop while held (vs once with whileTrue)
Buttons.whileTrueContinuous(Buttons.XboxAButton, new IntakeCommand(intake));
```

### SuppliedWaitCommand

Wait command with dynamic duration:

```java
// Wait for tunable duration (value can change between executions)
TunableDouble waitTime = new TunableDouble("AutoWait", 2.0);
Command autoSequence = Commands.sequence(
    driveForward(),
    new SuppliedWaitCommand(waitTime::get),
    intake()
);

// Wait based on sensor
Command dynamicWait = new SuppliedWaitCommand(
    () -> distanceSensor.getRange() / 10.0
);

// Conditional wait duration
Command smartWait = new SuppliedWaitCommand(
    () -> DriverStation.isTeleop() ? 0.5 : 1.5
);
```

---

## See Also

- [Actuators Documentation](../actuators/README.md)
- [Sensors Documentation](../sensors/README.md)
- [Subsystems Documentation](../subsystems/README.md)
- [WPILib Command-Based](https://docs.wpilib.org/en/stable/docs/software/commandbased/)
- [Mechanical Advantage RobotCode2026Public](https://github.com/Mechanical-Advantage/RobotCode2026Public)

---

**Need Help?** Check the detailed documentation for each utility, or refer to usage examples in the code.
