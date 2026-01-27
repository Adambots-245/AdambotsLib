# Gyro

CTRE Pigeon2 IMU implementation for measuring robot orientation.

## Constructor

```java
public Gyro(int CANport)
```

Creates a gyro on the specified CAN ID.

**Parameters:**
- `CANport` - CAN ID (0-62)

**Validation:**
- CAN ID validated at construction
- Invalid IDs default to 1 with DriverStation error

**Example:**
```java
BaseGyro gyro = new Gyro(1);  // CAN ID 1
```

## Methods

### getYaw()

```java
public Angle getYaw()
```

Returns the continuous yaw angle. Does not wrap at 360° - continues beyond for multi-rotation tracking.

**Returns:** Continuous yaw as WPILib `Angle` type

**Convention:** Counter-clockwise is positive

**Example:**
```java
import static edu.wpi.first.units.Units.*;

Angle heading = gyro.getYaw();
double degrees = heading.in(Degrees);  // After 1.5 CW rotations: -540.0
double radians = heading.in(Radians);
```

---

### getYawRotation2d()

```java
public Rotation2d getYawRotation2d()
```

Returns the continuous yaw as a WPILib Rotation2d object.

**Returns:** Rotation2d representing current heading

**Example:**
```java
// For swerve drive odometry
m_poseEstimator.update(
    gyro.getYawRotation2d(),
    modulePositions
);
```

---

### resetYaw()

```java
public void resetYaw()
```

Resets the yaw to zero. Use at the start of autonomous or when robot is in a known position.

**Example:**
```java
@Override
public void autonomousInit() {
    gyro.resetYaw();  // Start at 0°
}
```

---

### resetYawToAngle()

```java
public void resetYawToAngle(Angle angle)
```

Sets the yaw to a specific angle without physically moving the robot.

**Parameters:**
- `angle` - Angle to set as current heading (WPILib Angle type)

**Example:**
```java
import static edu.wpi.first.units.Units.*;

// Start auto facing away from driver station
gyro.resetYawToAngle(Degrees.of(180));
```

---

### offsetYawByAngle()

```java
public void offsetYawByAngle(Angle offset)
```

Offsets the current yaw by adding the specified angle. Maintains continuous tracking (does not wrap).

**Parameters:**
- `offset` - Angle to add to current heading (WPILib Angle type)

**Example:**
```java
import static edu.wpi.first.units.Units.*;

// Correct for 5° drift detected by vision
gyro.offsetYawByAngle(Degrees.of(5));
```

---

### getPitch()

```java
public Angle getPitch()
```

Returns the measured pitch (forward/backward tilt).

**Returns:** Pitch as WPILib `Angle` type

**Note:** Pitch and roll swap when robot rotates 90°

**Example:**
```java
import static edu.wpi.first.units.Units.*;

public final Trigger isPitched = new Trigger(() ->
    Math.abs(gyro.getPitch().in(Degrees)) > 15.0
);
```

---

### getRoll()

```java
public Angle getRoll()
```

Returns the measured roll (side-to-side tilt).

**Returns:** Roll as WPILib `Angle` type

**Note:** Pitch and roll swap when robot rotates 90°

**Example:**
```java
import static edu.wpi.first.units.Units.*;

public Command balance() {
    return run(() -> {
        double pitch = gyro.getPitch().in(Degrees);
        double correction = balancePID.calculate(pitch, 0.0);
        m_drive.drive(correction, 0, 0);
    });
}
```

## Hardware Details

### CTRE Pigeon2 Specifications

**Physical:**
- Dimensions: 24mm × 24mm × 14mm
- Weight: 4.5g
- Mounting: M2.5 holes

**Electrical:**
- Power: 12V via CAN
- Current: ~40mA typical
- CAN termination: Not required (internal)

**Performance:**
- Gyro range: ±2000 °/s
- Accelerometer range: ±8g
- Update rate: 200 Hz (default)
- Temperature range: -40°C to 85°C

**Orientation:**
- Mount with PCB flat
- Arrow indicates forward direction
- Can be rotated in software

### Wiring

Connect to CAN bus:
1. CAN High (yellow/green) → CAN High
2. CAN Low (yellow/yellow or black) → CAN Low
3. Power automatically provided via CAN

**CAN Bus Configuration:**
- Set unique CAN ID (0-62) using Phoenix Tuner
- CAN bus termination: 120Ω resistors at each end of bus

## Usage Patterns

### Field-Oriented Drive

```java
public class DriveSubsystem extends SubsystemBase {
    private final BaseGyro m_gyro;
    private final SwerveDrive m_drive;

    public Command driveFieldOriented(
            DoubleSupplier xSpeed,
            DoubleSupplier ySpeed,
            DoubleSupplier rotation) {
        return run(() -> {
            // Convert joystick to field-oriented
            ChassisSpeeds fieldSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                xSpeed.getAsDouble(),
                ySpeed.getAsDouble(),
                rotation.getAsDouble(),
                m_gyro.getYawRotation2d()
            );

            m_drive.drive(fieldSpeeds);
        });
    }
}
```

### Auto Rotation

```java
import static edu.wpi.first.units.Units.*;

public class AutoRotate extends Command {
    private final DriveSubsystem m_drive;
    private final BaseGyro m_gyro;
    private final double m_targetAngle;
    private final PIDController m_pid;

    public AutoRotate(DriveSubsystem drive, BaseGyro gyro, double targetAngle) {
        m_drive = drive;
        m_gyro = gyro;
        m_targetAngle = targetAngle;
        m_pid = new PIDController(0.05, 0, 0.003);
        m_pid.enableContinuousInput(-180, 180);
    }

    @Override
    public void execute() {
        double current = m_gyro.getYaw().in(Degrees);
        double output = m_pid.calculate(current, m_targetAngle);
        m_drive.drive(0, 0, output);
    }

    @Override
    public boolean isFinished() {
        return MathUtil.isNear(
            m_gyro.getYaw().in(Degrees),
            m_targetAngle,
            2.0
        );
    }
}
```

### Vision Correction

```java
import static edu.wpi.first.units.Units.*;

public class DriveSubsystem extends SubsystemBase {
    private final BaseGyro m_gyro;
    private final VisionSubsystem m_vision;

    @Override
    public void periodic() {
        // Periodically correct gyro drift with vision
        if (m_vision.hasTarget() && m_vision.isConfident()) {
            double visionAngle = m_vision.getRobotAngle();
            double gyroAngle = m_gyro.getYaw().in(Degrees);

            // If difference is large, trust vision
            if (Math.abs(visionAngle - gyroAngle) > 5.0) {
                m_gyro.resetYawToAngle(Degrees.of(visionAngle));
                Logger.recordOutput("Gyro/Corrected", true);
            }
        }
    }
}
```

### Balance Controller

```java
import static edu.wpi.first.units.Units.*;

public class BalanceCommand extends Command {
    private final DriveSubsystem m_drive;
    private final BaseGyro m_gyro;
    private final PIDController m_pid;

    public BalanceCommand(DriveSubsystem drive, BaseGyro gyro) {
        m_drive = drive;
        m_gyro = gyro;
        m_pid = new PIDController(0.03, 0, 0.001);
        addRequirements(drive);
    }

    @Override
    public void execute() {
        double pitch = m_gyro.getPitch().in(Degrees);
        double correction = m_pid.calculate(pitch, 0.0);

        // Drive forward/back to balance
        m_drive.drive(correction, 0, 0);
    }

    @Override
    public boolean isFinished() {
        // Balanced when pitch is near zero
        return Math.abs(m_gyro.getPitch().in(Degrees)) < 2.5;
    }
}
```

## Troubleshooting

### Gyro Reads Zero or NaN

**Cause:** Gyro not connected or CAN communication failure

**Solutions:**
1. Check CAN wiring (High, Low, power)
2. Verify CAN ID matches code
3. Check Phoenix Tuner shows device
4. Verify bus termination (120Ω at each end)

---

### Heading Drifts Over Time

**Cause:** Normal gyro behavior - all MEMS gyros accumulate error

**Solutions:**
1. Use vision to periodically correct
2. Reset at known positions (against wall, etc.)
3. Enable temperature compensation in Phoenix Tuner
4. Consider using vision for absolute heading

```java
import static edu.wpi.first.units.Units.*;

// Correct with vision
if (m_vision.canSeeTarget()) {
    gyro.resetYawToAngle(Degrees.of(m_vision.getRobotHeading()));
}
```

---

### Pitch/Roll Values Swap or Wrong

**Cause:** Gyro mounted in non-standard orientation

**Solutions:**
1. Configure mount pose in Phoenix Tuner
2. Or manually transform in code:

```java
public double getForwardTilt() {
    // If gyro rotated 90°, pitch becomes roll
    return m_gyro.getRoll();
}
```

---

### Heading Jumps During Match

**Cause:** CAN bus error or brownout

**Solutions:**
1. Check battery connections
2. Verify CAN bus wiring quality
3. Add StatusFrame monitoring
4. Consider using last-known-good value

```java
import static edu.wpi.first.units.Units.*;

private double m_lastValidYaw = 0;

public double getYawDegrees() {
    double yaw = m_gyro.getYaw().in(Degrees);
    if (!Double.isNaN(yaw)) {
        m_lastValidYaw = yaw;
        return yaw;
    }
    return m_lastValidYaw;  // Use cached value
}
```

## Configuration Tips

### Phoenix Tuner Configuration

**Status Frame Rates:**
- **Yaw (odometry)**: 10-20ms for critical applications
- **Pitch/Roll**: 50-100ms for balance
- **Diagnostics**: 200ms or slower

**Mount Orientation:**
Configure if gyro is not mounted flat with forward arrow pointing forward.

**Temperature Compensation:**
Enable for best drift performance.

## Performance Notes

**Update Rate:** 200 Hz default is excellent for all FRC applications

**Latency:** ~5ms typical

**CAN Bandwidth:** Minimal impact with proper StatusFrame configuration

**CPU Usage:** Negligible - hardware handles all calculations

## See Also

- **[Gyroscopes Overview](README.md)** - Gyro concepts and patterns
- **[Sensors Overview](../README.md)** - All sensor types
- **[BaseGyro Interface](BaseGyro.md)** - Interface documentation
- **[CTRE Pigeon2 Docs](https://pro.docs.ctr-electronics.com/en/latest/docs/hardware-reference/pigeon2/index.html)** - Vendor documentation
