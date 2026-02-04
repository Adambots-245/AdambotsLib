# Epilogue Annotation-Based Logging

AdambotsLib includes built-in support for WPILib Epilogue annotation-based logging. All motor and sensor wrapper classes are annotated with `@Logged`, enabling automatic telemetry logging compatible with AdvantageScope.

## What Gets Logged Automatically

When you use AdambotsLib classes in your `@Logged` subsystems, the following telemetry is automatically captured:

### Motor Classes

| Class | Logged Data |
|-------|-------------|
| **TalonFXMotor** | position, velocity, acceleration, current, temperature, outputPercent, forwardLimitSwitch, reverseLimitSwitch, motorType |
| **NEOMotor** | position, velocity, acceleration, current, temperature, outputPercent, forwardLimitSwitch, reverseLimitSwitch, motorType |
| **MinionMotor** | position, velocity, acceleration, current, temperature, outputPercent, forwardLimitSwitch, reverseLimitSwitch, motorType |

### Sensor Classes

| Class | Logged Data |
|-------|-------------|
| **Gyro** | yaw, pitch, roll, yawRotation2d |
| **CANCoder** | position, positionRotation2d |
| **ThroughBoreEncoder** | position, positionRotation2d |
| **PhotoEye** | isDetecting |
| **LimitSwitch** | isDetecting |
| **UltrasonicSensor** | distance |
| **Lidar** | distance |
| **CANRangeSensor** | distance |

## Setup in Your Robot Project

### 1. Robot.java Configuration

```java
import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.TimedRobot;

@Logged
public class Robot extends TimedRobot {
    private RobotContainer container;

    @Override
    public void robotInit() {
        // Enable disk logging for AdvantageScope
        DataLogManager.start();

        // Create your robot container
        container = new RobotContainer();

        // CRITICAL: Call after all @Logged objects are created
        Epilogue.bind(this);
    }
}
```

### 2. Subsystem Example

```java
import com.adambots.lib.actuators.TalonFXMotor;
import com.adambots.lib.sensors.PhotoEye;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

@Logged
public class ShooterSubsystem extends SubsystemBase {
    // These fields are automatically logged with full telemetry
    private final TalonFXMotor topWheel;
    private final TalonFXMotor bottomWheel;
    private final PhotoEye gamePieceSensor;

    public ShooterSubsystem() {
        topWheel = new TalonFXMotor(15, true, 60, true);
        bottomWheel = new TalonFXMotor(16, true, 60, true);
        gamePieceSensor = new PhotoEye(0, false);
    }
}
```

### 3. AdvantageScope Log Structure

After running your robot, open the log file in AdvantageScope. You'll see a structure like:

```
Robot/
├── container/
│   └── shooterSubsystem/
│       ├── topWheel/
│       │   ├── position: 45.2
│       │   ├── velocity: 85.0 rps
│       │   ├── current: 25.3 A
│       │   ├── temperature: 42.1 C
│       │   └── motorType: "TalonFXMotor (Kraken X60)"
│       ├── bottomWheel/
│       │   └── ...
│       └── gamePieceSensor/
│           └── isDetecting: true
```

## How It Works

AdambotsLib wrapper classes are annotated with `@Logged` at the class level. Private fields that shouldn't be logged (hardware objects, configuration state) are marked with `@NotLogged`. Public getter methods are automatically discovered by Epilogue and logged.

**Key Points:**
- No additional CAN overhead - Epilogue logs cached values from existing getters
- Motors already use optimized update frequencies (50Hz for position/velocity)
- Logging runs every 20ms, offset from the main loop

## Performance Monitoring

If you're concerned about logging performance, monitor these NetworkTables entries:
- `/Epilogue/Stats/Last Run` - Time taken for last logging cycle
- `/Epilogue/Stats/Average Run` - Average logging time

## Troubleshooting

### Logs not appearing in AdvantageScope

1. Ensure `DataLogManager.start()` is called before `Epilogue.bind(this)`
2. Verify your Robot class has `@Logged` annotation
3. Check that `Epilogue.bind(this)` is called after creating all subsystems
4. Make sure the log file path is correct (default: `/home/lvuser/logs/` on roboRIO)

### Missing fields in logs

1. Verify the field is a public getter method (not a field)
2. Check that the class has `@Logged` annotation
3. Ensure the field type is loggable (primitives, WPILib types, or other `@Logged` classes)

## Optional: Custom Loggers

If you need custom logging behavior beyond what Epilogue provides automatically, you can create custom loggers in your robot project. See WPILib documentation for details on creating custom `EpilogueBackend` implementations.

**Note:** Custom loggers must be defined in your robot project, not in libraries like AdambotsLib, due to WPILib annotation processing limitations.
