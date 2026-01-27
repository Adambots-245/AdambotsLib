# Gyroscopes

Gyroscopes (gyros) measure robot orientation and rotation for navigation, balance control, and field-oriented drive. AdambotsLib provides a unified interface for gyro hardware with continuous angle tracking and automatic validation.

## Available Implementations

| Class | Hardware | Connection | Features |
|-------|----------|------------|----------|
| [Gyro](Gyro.md) | CTRE Pigeon2 IMU | CAN | 3-axis, continuous yaw, pitch & roll |

## Quick Start

```java
import com.adambots.lib.sensors.*;
import static edu.wpi.first.units.Units.*;

// Create gyro on CAN ID 1
BaseGyro gyro = new Gyro(1);

// Get heading (continuous, no wrapping)
Angle heading = gyro.getYaw();
double degrees = heading.in(Degrees);
double radians = heading.in(Radians);

// Reset to zero
gyro.resetYaw();

// Get as Rotation2d for kinematics
Rotation2d rotation = gyro.getYawRotation2d();
```

## Interface: BaseGyro

All gyroscope implementations follow the `BaseGyro` interface:

```java
import edu.wpi.first.units.measure.Angle;

public interface BaseGyro {
    // Yaw (heading) - continuous tracking with typed units
    Angle getYaw();                    // Returns WPILib Angle type
    Rotation2d getYawRotation2d();     // For kinematics/odometry

    // Pitch and roll with typed units
    Angle getPitch();
    Angle getRoll();

    // Reset and offset with typed units
    void resetYaw();
    void resetYawToAngle(Angle angle);
    void offsetYawByAngle(Angle offset);
}
```

**Unit Conversion:**
```java
import static edu.wpi.first.units.Units.*;

Angle yaw = gyro.getYaw();
double degrees = yaw.in(Degrees);  // Convert to degrees
double radians = yaw.in(Radians);  // Convert to radians
```

## Key Concepts

### Continuous vs Wrapped Angles

**Continuous angles** track rotation beyond 360°:
- Turning right 450° gives `450.0`, not `90.0`
- Essential for navigation and odometry
- Can be positive or negative
- Used for tracking total robot rotation

**Wrapped angles** stay within 0-360°:
- Not provided by AdambotsLib gyros
- If needed, wrap manually: `angle % 360`

```java
import static edu.wpi.first.units.Units.*;

// Continuous tracking (AdambotsLib default)
gyro.resetYaw();
robot.turnRight(450);  // 1.25 rotations
Angle yaw = gyro.getYaw();
double yawDegrees = yaw.in(Degrees);  // Returns 450.0

// Manual wrapping if needed
double wrapped = yawDegrees % 360;  // Returns 90.0
```

### Yaw, Pitch, and Roll

- **Yaw** (Z-axis): Robot heading/rotation around vertical axis
- **Pitch** (Y-axis): Forward/backward tilt
- **Roll** (X-axis): Side-to-side tilt

**Important**: Pitch and roll values depend on robot orientation and can exchange when the robot rotates.

## Common Patterns

### Field-Oriented Drive

```java
public class DriveSubsystem extends SubsystemBase {
    private final SwerveDrive m_drive;
    private final BaseGyro m_gyro;

    public Command drive(DoubleSupplier xSpeed, DoubleSupplier ySpeed, DoubleSupplier rotation) {
        return run(() -> {
            m_drive.driveFieldOriented(
                xSpeed.getAsDouble(),
                ySpeed.getAsDouble(),
                rotation.getAsDouble(),
                m_gyro.getYawRotation2d()  // Current heading
            );
        });
    }

    public Command resetGyro() {
        return runOnce(() -> m_gyro.resetYaw());
    }
}
```

### Auto Alignment

```java
import static edu.wpi.first.units.Units.*;

public class ShooterSubsystem extends SubsystemBase {
    private final BaseGyro m_gyro;
    private double m_targetAngle = 0.0;

    public final Trigger isAimed = new Trigger(() ->
        MathUtil.isNear(
            m_gyro.getYaw().in(Degrees),
            m_targetAngle,
            2.0  // Within 2 degrees
        )
    ).debounce(0.1);

    public Command aimAt(DoubleSupplier targetAngle) {
        return run(() -> {
            m_targetAngle = targetAngle.getAsDouble();
            double error = m_targetAngle - m_gyro.getYaw().in(Degrees);
            m_drive.rotate(calculatePID(error));
        }).until(isAimed);
    }
}
```

### Balance Controller

```java
import static edu.wpi.first.units.Units.*;

public class BalanceSubsystem extends SubsystemBase {
    private final BaseGyro m_gyro;
    private final DriveSubsystem m_drive;

    // Trigger when robot is level
    public final Trigger isBalanced = new Trigger(() ->
        Math.abs(m_gyro.getPitch().in(Degrees)) < 5.0 &&
        Math.abs(m_gyro.getRoll().in(Degrees)) < 5.0
    ).debounce(0.5);

    public Command balance() {
        return run(() -> {
            double pitch = m_gyro.getPitch().in(Degrees);
            double speed = calculateBalanceSpeed(pitch);
            m_drive.drive(speed, 0, 0);
        }).until(isBalanced);
    }
}
```

### Odometry Integration

```java
public class DriveSubsystem extends SubsystemBase {
    private final SwerveDrive m_drive;
    private final BaseGyro m_gyro;
    private final SwerveDrivePoseEstimator m_poseEstimator;

    @Override
    public void periodic() {
        // Update pose estimator with gyro reading
        m_poseEstimator.update(
            m_gyro.getYawRotation2d(),
            m_drive.getModulePositions()
        );
    }

    public Pose2d getPose() {
        return m_poseEstimator.getEstimatedPosition();
    }
}
```

## Best Practices

### 1. Use Continuous Angles for Navigation

```java
// Good: Continuous for navigation
public Rotation2d getHeading() {
    return m_gyro.getYawRotation2d();
}

// Avoid: Wrapping breaks navigation
public Rotation2d getHeading() {
    double wrapped = m_gyro.getYaw().in(Degrees) % 360;
    return Rotation2d.fromDegrees(wrapped);
}
```

### 2. Reset Gyro at Match Start

```java
import static edu.wpi.first.units.Units.*;

@Override
public void autonomousInit() {
    // Start each mode with known heading
    m_gyro.resetYaw();
}

// Or set to specific angle for auto
@Override
public void autonomousInit() {
    m_gyro.resetYawToAngle(Degrees.of(180));  // Start facing away from driver
}
```

### 3. Use MathUtil.isNear() for Comparisons

```java
import static edu.wpi.first.units.Units.*;

// Good: Tolerant comparison
public final Trigger isAimed = new Trigger(() ->
    MathUtil.isNear(m_gyro.getYaw().in(Degrees), targetAngle, 2.0)
).debounce(0.1);

// Avoid: Direct equality rarely works
public final Trigger isAimed = new Trigger(() ->
    m_gyro.getYaw().in(Degrees) == targetAngle  // Almost never true
);
```

### 4. Cache Gyro Values in Periodic

```java
import static edu.wpi.first.units.Units.*;

private double m_cachedYaw;
private double m_cachedPitch;

@Override
public void periodic() {
    // Read once per iteration
    m_cachedYaw = m_gyro.getYaw().in(Degrees);
    m_cachedPitch = m_gyro.getPitch().in(Degrees);
}

// Use cached values in triggers
public final Trigger isAimed =
    new Trigger(() -> MathUtil.isNear(m_cachedYaw, m_targetAngle, 2.0));
```

### 5. Use Debouncing for Triggers

```java
import static edu.wpi.first.units.Units.*;

// Prevent premature trigger activation
public final Trigger isBalanced = new Trigger(() ->
    Math.abs(m_gyro.getPitch().in(Degrees)) < 5.0
).debounce(0.5, Debouncer.DebounceType.kRising);
```

## Hardware Specifications

### CTRE Pigeon2

**Connection:**
- CAN bus
- Valid CAN IDs: 0-62

**Features:**
- 3-axis gyroscope
- 3-axis accelerometer
- Update rate: 200 Hz (default)
- Temperature compensation
- Mounting orientation configurable

**Orientation:**
- **Yaw**: Rotation around vertical (Z) axis
- **Pitch**: Rotation around forward-back (Y) axis
- **Roll**: Rotation around side-to-side (X) axis

**Important**: Pigeon2 has pitch and roll swapped relative to typical aircraft convention. AdambotsLib corrects this automatically.

## Troubleshooting

### Gyro Reading Drifts Over Time

**Cause**: All MEMS gyros accumulate small errors
**Solution**:
- Use vision or other absolute references to correct drift
- Reset gyro periodically when robot is in known position
- Pigeon2 temperature compensation helps reduce drift

```java
// Correct drift with vision
if (m_vision.hasValidTarget()) {
    double visionAngle = m_vision.getRobotAngle();
    m_gyro.resetYawToAngle(visionAngle);
}
```

### Heading Jumps or Resets Unexpectedly

**Cause**: CAN communication error or gyro power cycle
**Solution**:
- Check CAN bus wiring and termination
- Verify power supply is stable
- Check for CAN bus errors in Phoenix Tuner

### Pitch/Roll Values Seem Wrong

**Cause**: Gyro mounted in non-standard orientation
**Solution**:
- Configure mount pose in Phoenix Tuner
- Or manually transform values in code

### offsetYawByAngle() Wraps Angle

**Old Behavior (Fixed)**: Previously wrapped continuous angles with modulo 360
**Current Behavior**: Correctly adds offset without wrapping
**Solution**: Update to latest AdambotsLib version

## Migration from WPILib

### Direct Pigeon2 Usage

```java
import static edu.wpi.first.units.Units.*;

// WPILib vendor library
Pigeon2 pigeon = new Pigeon2(1);
double yaw = pigeon.getYaw().getValueAsDouble();
pigeon.reset();

// AdambotsLib
BaseGyro gyro = new Gyro(1);
double yaw = gyro.getYaw().in(Degrees);
gyro.resetYaw();
```

### ADXRS450 Gyro

```java
import static edu.wpi.first.units.Units.*;

// WPILib (SPI gyro)
ADXRS450_Gyro gyro = new ADXRS450_Gyro();
double angle = gyro.getAngle();

// AdambotsLib (upgrade to Pigeon2)
BaseGyro gyro = new Gyro(1);  // CAN-based
double angle = gyro.getYaw().in(Degrees);
```

## Performance Considerations

### Update Rates

Pigeon2 can update at 200 Hz by default, which is more than sufficient for FRC applications.

### CAN Bus Impact

Configure appropriate status frame rates in Phoenix Tuner:
- Critical: 10-20 ms (odometry, control)
- Important: 50-100 ms (display, logging)
- Optional: 200-500 ms (diagnostics)

```java
// In code (if needed)
// Most teams can use default settings
```

### Computational Cost

Gyro reads are very fast (microseconds). No special optimization needed.

## See Also

- **[Gyro Implementation](Gyro.md)** - Pigeon2 gyro details
- **[Sensors Overview](../README.md)** - All sensor types
- **[Command Best Practices](../../../COMMAND_BEST_PRACTICES.md)** - Trigger patterns
