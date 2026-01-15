# StateMachine

A robust, type-safe state machine implementation for managing subsystem behavior in FRC robots.

## Table of Contents

- [Overview](#overview)
- [Understanding State Machines](#understanding-state-machines)
- [Getting Started](#getting-started)
- [API Reference](#api-reference)
- [Complete Examples](#complete-examples)
- [Best Practices](#best-practices)
- [Troubleshooting](#troubleshooting)
- [API Comparison](#api-comparison)

---

## Overview

`StateMachine` provides a modern, builder-pattern API for managing complex subsystem behavior. It automatically detects whether transitions should be immediate or conditional, making it intuitive and reducing common errors.

**Key Features:**
- ✅ Automatic mode detection (immediate vs conditional transitions)
- ✅ Builder pattern API - reads like English
- ✅ Type-safe with Java generics
- ✅ Fail-fast with clear error messages
- ✅ Optional state-property maps for cleaner code
- ✅ Transition listeners for debugging and logging
- ✅ Invalid transition validation for safety

**Version:** 2026.2.0+

---

## Understanding State Machines

### What is a State Machine?

A state machine is a model that describes how a system behaves and transitions between different states. Think of it like a flowchart where:
- Each box represents a **state** (a specific condition or mode)
- Each arrow represents a **transition** (how we move between states)
- The system can only be in one state at a time

### Key Concepts

#### 1. States
Represent different modes or conditions of your system.

**Example:** An arm might have states like:
- `STOWED` - Arm is retracted inside robot frame
- `LOW` - Arm positioned for low scoring
- `MID` - Arm positioned for mid scoring
- `HIGH` - Arm positioned for high scoring

```java
enum ArmState {
    STOWED, LOW, MID, HIGH
}
```

#### 2. Transitions
Define how the system moves between states.

**Types:**
- **Immediate Transitions** - Complete on next `periodic()` call
  - Used for position control (PID, Motion Magic)
  - Motor controller handles reaching target
- **Conditional Transitions** - Wait for a condition to become true
  - Used for velocity control, sensor triggers
  - Require explicit condition check

#### 3. Properties
Data associated with each state.

**Example:** Arm properties might include:
- Target position (degrees)
- Speed/power setting
- Human-readable description

```java
record ArmProps(double angleDeg, double speed, String description) {}
```

---

## Getting Started

### Step 1: Define States and Properties

```java
public class ArmSubsystem extends SubsystemBase {
    // Define states as enum
    enum State {
        STOWED, LOW, MID, HIGH
    }

    // Define properties as record (or class)
    record ArmProps(double angleDeg, double speed) {}

    // Hardware
    private final TalonFXMotor motor;

    // State machine
    private final StateMachine<State, ArmProps> sm;
}
```

### Step 2: Create State Machine

**Option A: With State-Property Map (Recommended)**

```java
public ArmSubsystem() {
    motor = new TalonFXMotor(1, false, 40, false);

    // Create state-property map
    Map<State, ArmProps> stateMap = Map.of(
        State.STOWED, new ArmProps(0.0, 0.5),
        State.LOW, new ArmProps(30.0, 0.6),
        State.MID, new ArmProps(60.0, 0.7),
        State.HIGH, new ArmProps(90.0, 0.8)
    );

    // Create state machine
    sm = new StateMachine<>(
        State.STOWED,      // Initial state
        stateMap,          // State-property map
        System.out::println // Logger (optional)
    );
}
```

**Option B: Without State-Property Map**

```java
public ArmSubsystem() {
    motor = new TalonFXMotor(1, false, 40, false);

    // Create state machine
    sm = new StateMachine<>(
        State.STOWED,                    // Initial state
        new ArmProps(0.0, 0.5),          // Initial properties
        System.out::println              // Logger (optional)
    );
}
```

### Step 3: Create Commands

```java
public Command goToHigh() {
    return runOnce(() ->
        sm.to(State.HIGH)
          .executing(p -> motor.set(ControlMode.POSITION, p.angleDeg()))
          .request()
    );
}
```

### Step 4: Call periodic()

**CRITICAL:** You must call `sm.periodic()` in your subsystem's `periodic()` method.

```java
@Override
public void periodic() {
    sm.periodic();  // REQUIRED - handles transition logic
}
```

---

## API Reference

### Basic API Structure

```java
sm.to(State.TARGET)              // Which state to transition to
    .withProperties(props)        // Properties for target state (optional if using state-property map)
    .executing(props -> { })      // What to do during transition (optional)
    .when(condition)              // When to complete transition (optional)
    .request();                   // Execute the transition
```

**Key Insight:** The mode is automatically determined:
- **No `.when()` clause** → Immediate transition (completes on next `periodic()`)
- **With `.when()` clause** → Conditional transition (waits for condition)

---

### Immediate Transitions

**Use Case:** Position control with PID, Motion Magic, or any mode where the motor controller handles reaching the target.

**Syntax:**
```java
sm.to(State.TARGET)
    .executing(props -> {
        // Command motor to target position
        motor.set(ControlMode.POSITION, props.angleDeg());
    })
    .request();  // No .when() = immediate!
```

**Behavior:**
1. `.executing()` action runs immediately
2. State changes on the next `periodic()` call
3. Motor controller handles reaching target

**Example:**
```java
public Command goToHigh() {
    return runOnce(() ->
        sm.to(State.HIGH)
          .executing(p -> motor.set(ControlMode.POSITION, p.angleDeg()))
          .request()
    );
}
```

---

### Conditional Transitions

**Use Case:** Velocity control, sensor triggers, or any time you need to wait for a condition.

**Syntax:**
```java
sm.to(State.TARGET)
    .executing(props -> {
        // Start action (e.g., run motor at speed)
        motor.set(ControlMode.VELOCITY, props.speed());
    })
    .when(() -> condition())  // Wait for this to be true
    .request();
```

**Behavior:**
1. `.executing()` action runs immediately
2. State machine checks `.when()` condition every `periodic()` call
3. State changes when condition returns `true`

**Example:**
```java
public Command intakeUntilGamePiece() {
    return runOnce(() ->
        sm.to(State.INTAKING)
          .withProperties(new IntakeProps(0.8, "Intaking"))
          .executing(p -> intake.setSpeed(p.speed()))
          .when(intake::hasGamePiece)  // Wait for sensor
          .request()
    );
}
```

---

### No Action Transitions

Sometimes you just want to change state without commanding anything:

```java
sm.to(State.IDLE)
    .withProperties(new Props(0, "Idle"))
    .request();  // No .executing() needed
```

---

### State Machine Constructor Options

#### Option 1: With State-Property Map (Recommended)

```java
StateMachine<State, Props> sm = new StateMachine<>(
    initialState,
    stateMap,      // Map<State, Props>
    logger         // Consumer<String> (optional, can be null)
);

// Use without .withProperties()
sm.to(State.HIGH).executing(action).request();
```

#### Option 2: Without State-Property Map

```java
StateMachine<State, Props> sm = new StateMachine<>(
    initialState,
    initialProps,
    logger
);

// Must use .withProperties()
sm.to(State.HIGH).withProperties(props).executing(action).request();
```

---

### Query Methods

```java
// Get current state
State current = sm.getCurrentState();

// Check if transitioning
boolean inTransition = sm.isTransitioning();

// Get current properties
Props props = sm.getCurrentProperties();
```

---

### Advanced Features

#### Add Transition Listener

```java
sm.addTransitionListener((fromState, toState) -> {
    DataLogManager.log("Transitioned: " + fromState + " -> " + toState);
});
```

#### Add Invalid Transitions

Prevent unsafe transitions:

```java
sm.addInvalidTransition(State.STOWED, State.ERROR,
    "Cannot transition to ERROR state from STOWED");

// Attempting this transition will throw IllegalStateException
```

---

## Complete Examples

### Example 1: Arm Subsystem with Position Control

```java
public class ArmSubsystem extends SubsystemBase {
    // States
    enum State {
        STOWED, LOW_GOAL, MID_GOAL, HIGH_GOAL
    }

    // Properties
    record ArmProps(double angleDeg, double speed, String description) {}

    // Hardware
    private final TalonFXMotor motor;
    private final DutyCycleEncoder encoder;

    // State machine
    private final StateMachine<State, ArmProps> sm;

    // Constants
    private static final double TOLERANCE = 2.0;

    public ArmSubsystem() {
        // Initialize hardware
        motor = new TalonFXMotor(1, false, 40, false);
        encoder = new DutyCycleEncoder(0);

        // Create state-property map
        Map<State, ArmProps> stateMap = Map.of(
            State.STOWED, new ArmProps(0, 0.5, "Stowed"),
            State.LOW_GOAL, new ArmProps(30, 0.6, "Low Goal"),
            State.MID_GOAL, new ArmProps(60, 0.7, "Mid Goal"),
            State.HIGH_GOAL, new ArmProps(90, 0.8, "High Goal")
        );

        // Create state machine
        sm = new StateMachine<>(
            State.STOWED,
            stateMap,
            DataLogManager::log
        );

        // Configure motor
        motor.configure()
            .pid(0.1, 0.0, 0.05, 0.0)
            .currentLimits(40, 60, 5000)
            .brakeMode(true)
            .apply();
    }

    // Command methods - Immediate transitions (PID handles position)
    public Command goToStowed() {
        return runOnce(() ->
            sm.to(State.STOWED)
              .executing(p -> motor.set(ControlMode.POSITION, p.angleDeg()))
              .request()
        );
    }

    public Command goToLow() {
        return runOnce(() ->
            sm.to(State.LOW_GOAL)
              .executing(p -> motor.set(ControlMode.POSITION, p.angleDeg()))
              .request()
        );
    }

    public Command goToMid() {
        return runOnce(() ->
            sm.to(State.MID_GOAL)
              .executing(p -> motor.set(ControlMode.POSITION, p.angleDeg()))
              .request()
        );
    }

    public Command goToHigh() {
        return runOnce(() ->
            sm.to(State.HIGH_GOAL)
              .executing(p -> motor.set(ControlMode.POSITION, p.angleDeg()))
              .request()
        );
    }

    @Override
    public void periodic() {
        // CRITICAL: Must call periodic()
        sm.periodic();

        // Update SmartDashboard
        SmartDashboard.putString("Arm State", sm.getCurrentState().toString());
        SmartDashboard.putBoolean("Arm Transitioning", sm.isTransitioning());
        SmartDashboard.putNumber("Arm Angle", encoder.getDistance());
    }

    // Query methods
    public boolean isStowed() {
        return sm.getCurrentState() == State.STOWED;
    }

    public boolean isTransitioning() {
        return sm.isTransitioning();
    }
}
```

---

### Example 2: Intake Subsystem with Conditional Transitions

```java
public class IntakeSubsystem extends SubsystemBase {
    // States
    enum State {
        IDLE, INTAKING, EJECTING, HAS_GAME_PIECE
    }

    // Properties
    record IntakeProps(double speed, String description) {}

    // Hardware
    private final NEOMotor motor;
    private final PhotoEye sensor;

    // State machine
    private final StateMachine<State, IntakeProps> sm;

    public IntakeSubsystem() {
        motor = new NEOMotor(5, false, 20, false);
        sensor = new PhotoEye(3, false);

        // State-property map
        Map<State, IntakeProps> stateMap = Map.of(
            State.IDLE, new IntakeProps(0.0, "Idle"),
            State.INTAKING, new IntakeProps(0.8, "Intaking"),
            State.EJECTING, new IntakeProps(-0.5, "Ejecting"),
            State.HAS_GAME_PIECE, new IntakeProps(0.1, "Holding")
        );

        sm = new StateMachine<>(
            State.IDLE,
            stateMap,
            DataLogManager::log
        );
    }

    // Conditional transition - wait for game piece
    public Command intakeCommand() {
        return runOnce(() ->
            sm.to(State.INTAKING)
              .executing(p -> motor.set(ControlMode.PERCENT_OUTPUT, p.speed()))
              .when(this::hasGamePiece)  // Wait for sensor
              .request()
        ).andThen(runOnce(() ->
            // After transition completes, go to holding state
            sm.to(State.HAS_GAME_PIECE)
              .executing(p -> motor.set(ControlMode.PERCENT_OUTPUT, p.speed()))
              .request()
        ));
    }

    // Immediate transition - just run motor
    public Command ejectCommand() {
        return runOnce(() ->
            sm.to(State.EJECTING)
              .executing(p -> motor.set(ControlMode.PERCENT_OUTPUT, p.speed()))
              .request()
        ).andThen(Commands.waitSeconds(1.0))
         .andThen(stopCommand());
    }

    public Command stopCommand() {
        return runOnce(() ->
            sm.to(State.IDLE)
              .executing(p -> motor.set(ControlMode.PERCENT_OUTPUT, 0.0))
              .request()
        );
    }

    public boolean hasGamePiece() {
        return sensor.isDetecting();
    }

    public Trigger hasGamePieceTrigger() {
        return new Trigger(this::hasGamePiece);
    }

    @Override
    public void periodic() {
        sm.periodic();

        SmartDashboard.putString("Intake State", sm.getCurrentState().toString());
        SmartDashboard.putBoolean("Has Game Piece", hasGamePiece());
    }
}
```

---

### Example 3: Shooter Subsystem with Multiple Transitions

```java
public class ShooterSubsystem extends SubsystemBase {
    enum State {
        IDLE, SPINNING_UP, READY, SHOOTING, COOLING
    }

    record ShooterProps(double speedRPS, String description) {}

    private final TalonFXMotor leftMotor;
    private final TalonFXMotor rightMotor;
    private final StateMachine<State, ShooterProps> sm;

    private static final double READY_TOLERANCE = 5.0;  // RPS

    public ShooterSubsystem() {
        leftMotor = new TalonFXMotor(10, false, true);
        rightMotor = new TalonFXMotor(11, true, true);

        Map<State, ShooterProps> stateMap = Map.of(
            State.IDLE, new ShooterProps(0.0, "Idle"),
            State.SPINNING_UP, new ShooterProps(80.0, "Spinning Up"),
            State.READY, new ShooterProps(80.0, "Ready"),
            State.SHOOTING, new ShooterProps(80.0, "Shooting"),
            State.COOLING, new ShooterProps(10.0, "Cooling")
        );

        sm = new StateMachine<>(State.IDLE, stateMap, DataLogManager::log);
    }

    // Conditional transition - wait for velocity
    public Command spinUpCommand() {
        return runOnce(() ->
            sm.to(State.SPINNING_UP)
              .executing(p -> {
                  leftMotor.set(ControlMode.VELOCITY, p.speedRPS());
                  rightMotor.set(ControlMode.VELOCITY, p.speedRPS());
              })
              .when(this::isAtSpeed)
              .request()
        ).andThen(runOnce(() ->
            // Once at speed, transition to READY
            sm.to(State.READY).request()
        ));
    }

    public Command shootCommand() {
        return runOnce(() ->
            sm.to(State.SHOOTING)
              .request()
        ).andThen(Commands.waitSeconds(0.5))
         .andThen(stopCommand());
    }

    public Command stopCommand() {
        return runOnce(() ->
            sm.to(State.IDLE)
              .executing(p -> {
                  leftMotor.set(ControlMode.PERCENT_OUTPUT, 0.0);
                  rightMotor.set(ControlMode.PERCENT_OUTPUT, 0.0);
              })
              .request()
        );
    }

    private boolean isAtSpeed() {
        double target = sm.getCurrentProperties().speedRPS();
        double leftSpeed = leftMotor.getEncoderVelocity();
        return Math.abs(leftSpeed - target) < READY_TOLERANCE;
    }

    public boolean isReady() {
        return sm.getCurrentState() == State.READY;
    }

    public Trigger isReadyTrigger() {
        return new Trigger(this::isReady);
    }

    @Override
    public void periodic() {
        sm.periodic();

        SmartDashboard.putString("Shooter State", sm.getCurrentState().toString());
        SmartDashboard.putNumber("Shooter Speed", leftMotor.getEncoderVelocity());
        SmartDashboard.putBoolean("Shooter Ready", isReady());
    }
}
```

---

## Best Practices

### 1. Always Call periodic()

The state machine requires `periodic()` to be called every cycle to process transitions.

```java
@Override
public void periodic() {
    sm.periodic();  // REQUIRED!
}
```

**Why:** The state machine checks conditional transitions and completes immediate transitions in `periodic()`.

---

### 2. Use State-Property Maps

State-property maps reduce boilerplate and improve maintainability.

**Good:**
```java
Map<State, Props> stateMap = Map.of(
    State.LOW, new Props(30.0, 0.6),
    State.HIGH, new Props(90.0, 0.8)
);

sm = new StateMachine<>(State.LOW, stateMap, logger);
sm.to(State.HIGH).executing(action).request();  // Clean!
```

**Works but verbose:**
```java
sm = new StateMachine<>(State.LOW, new Props(30.0, 0.6), logger);
sm.to(State.HIGH)
  .withProperties(new Props(90.0, 0.8))  // Must specify every time
  .executing(action)
  .request();
```

---

### 3. Choose the Right Transition Mode

**Use Immediate (no `.when()`):**
- Position control with PID
- Motion Magic
- Any mode where motor controller reaches target automatically

**Use Conditional (with `.when()`):**
- Velocity control
- Waiting for sensors
- Manual positioning
- Any time you need to check a condition

---

### 4. Add Transition Listeners for Debugging

```java
sm.addTransitionListener((from, to) -> {
    DataLogManager.log("State transition: " + from + " -> " + to);
});
```

**Benefits:**
- Easy debugging of state changes
- Automatic logging to DriverStation or file
- No need to manually log in every transition

---

### 5. Use Invalid Transitions for Safety

Prevent dangerous or nonsensical transitions:

```java
sm.addInvalidTransition(State.STOWED, State.MANUAL,
    "Cannot enter manual mode from stowed");

sm.addInvalidTransition(State.ERROR, State.HIGH,
    "Must reset error before moving to high");
```

**Benefits:**
- Fail-fast with clear error messages
- Prevents unsafe operations
- Documents invalid state combinations

---

### 6. Publish State to SmartDashboard

```java
@Override
public void periodic() {
    sm.periodic();

    SmartDashboard.putString("State", sm.getCurrentState().toString());
    SmartDashboard.putBoolean("Transitioning", sm.isTransitioning());
}
```

**Benefits:**
- Real-time state visibility
- Easy debugging during testing
- Driver feedback

---

### 7. Create Trigger Methods for State-Based Logic

```java
public boolean isReady() {
    return sm.getCurrentState() == State.READY;
}

public Trigger isReadyTrigger() {
    return new Trigger(this::isReady);
}

// In RobotContainer
shooter.isReadyTrigger().whileTrue(leds.setColorCommand(Color.kGreen));
```

---

## Troubleshooting

### State Not Changing?

**Symptoms:**
- Transition requested but state stays the same
- SmartDashboard shows old state

**Solutions:**

1. **Did you call `sm.periodic()`?**
   ```java
   @Override
   public void periodic() {
       sm.periodic();  // Must call this!
   }
   ```

2. **Is your condition actually becoming true?**
   ```java
   // Add debug logging
   sm.to(State.TARGET)
     .when(() -> {
         boolean result = condition();
         System.out.println("Condition check: " + result);
         return result;
     })
     .request();
   ```

3. **Check logs for error messages**
   - Look for exceptions or warnings in DriverStation

---

### Getting IllegalStateException?

**Error: "Properties required but not provided"**
- You forgot `.withProperties()` when not using a state-property map
- **Solution:** Add `.withProperties()` or use a state-property map

**Error: "Invalid transition from X to Y"**
- You attempted a transition blocked by `addInvalidTransition()`
- **Solution:** Check if this transition is intentionally blocked for safety

**Error: "Cannot transition while already transitioning"**
- You requested a new transition while a conditional transition is in progress
- **Solution:** Wait for current transition to complete, or cancel it first

---

### Transition Happening Too Soon?

**Symptoms:**
- Immediate transition when you expected conditional
- State changes before condition is met

**Solution:**
- You probably forgot the `.when()` clause
- Immediate transitions complete on the very next `periodic()` call

**Example:**
```java
// Wrong - immediate transition
sm.to(State.INTAKING)
  .executing(p -> intake.setSpeed(p.speed()))
  .request();  // State changes next periodic()

// Right - conditional transition
sm.to(State.INTAKING)
  .executing(p -> intake.setSpeed(p.speed()))
  .when(intake::hasGamePiece)  // Wait for this!
  .request();
```

---

### Transition Not Completing?

**Symptoms:**
- State machine stuck in transitioning state
- Conditional transition never completes

**Solutions:**

1. **Check that condition actually becomes true**
   ```java
   // Add logging
   sm.to(State.TARGET)
     .when(() -> {
         boolean result = condition();
         System.out.println("Waiting for: " + result);
         return result;
     })
     .request();
   ```

2. **Verify sensor readings**
   - Check SmartDashboard for sensor values
   - Ensure sensors are wired and working

3. **Check for logic errors**
   - Is the condition inverted?
   - Are units correct (degrees vs radians)?

---

### Motor Not Responding?

**Symptoms:**
- Transition executes but motor doesn't move
- State changes but no physical action

**Solutions:**

1. **Check that `.executing()` is called**
   ```java
   sm.to(State.HIGH)
     .executing(p -> motor.set(ControlMode.POSITION, p.angle()))  // Add this!
     .request();
   ```

2. **Verify motor command is correct**
   - Check control mode (POSITION, VELOCITY, etc.)
   - Verify units (degrees, RPS, etc.)
   - Test motor with manual command first

3. **Check for CAN errors**
   - Phoenix Tuner or REV Hardware Client
   - DriverStation CAN errors

---

## API Comparison

### Before 2026.2.0 (Old API)

The old API required manually specifying `() -> true` for immediate transitions:

```java
// Immediate transition - confusing pattern
sm.requestTransition(
    ArmState.HIGH_GOAL,
    ArmState.HIGH_GOAL.properties,
    () -> true,  // "Magic" pattern students had to memorize
    props -> motor.set(ControlMode.POSITION, props.angle())
);

// Conditional transition
sm.requestTransition(
    IntakeState.INTAKING,
    IntakeState.INTAKING.properties,
    intake::hasGamePiece,  // Different pattern for conditional
    props -> intake.setSpeed(props.speed())
);
```

**Problems:**
- `() -> true` pattern was not intuitive
- No clear distinction between immediate and conditional
- Required memorizing different patterns
- Not self-documenting

---

### After 2026.2.0 (New API)

The new API automatically detects mode based on presence of `.when()`:

```java
// Immediate transition - automatic detection
sm.to(ArmState.HIGH_GOAL)
    .executing(props -> motor.set(ControlMode.POSITION, props.angle()))
    .request();  // No .when() = immediate!

// Conditional transition - explicit and clear
sm.to(IntakeState.INTAKING)
    .executing(props -> intake.setSpeed(props.speed()))
    .when(intake::hasGamePiece)  // Obvious that we're waiting
    .request();
```

**Benefits:**
- No more `() -> true` magic pattern
- Clear intent with `.when()` clause
- Reads like English
- Same structure for both modes
- Self-documenting code

---

## Additional Resources

- **StateMachine.java** - Full source code with inline documentation
- **StateMachineTest.java** - Comprehensive test examples
- **WPILib Command-Based** - https://docs.wpilib.org/en/stable/docs/software/commandbased/
- **FRC Programming Best Practices** - https://docs.wpilib.org/en/stable/docs/software/basic-programming/

---

## See Also

- [Actuators Documentation](../actuators/README.md) - Motor and servo abstractions
- [Sensors Documentation](../sensors/README.md) - Sensor trigger patterns
- [Subsystems Documentation](../subsystems/README.md) - Complete subsystem examples

---

**Last Updated:** 2026-01-14
**AdambotsLib Version:** 2026.2.0+
