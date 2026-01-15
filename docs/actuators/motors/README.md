# Motors

Motor control with unified API for NEO, Falcon 500, Kraken X60, and Minion motors. Seamlessly switch hardware without code changes.

## Quick Start

```java
import com.adambots.lib.actuators.*;

// REV NEO motor
BaseMotor neo = new NEOMotor(1, false, 40, false);

// CTRE Falcon 500 / Kraken X60
BaseMotor talon = new TalonFXMotor(2, false, 40, false, false);

// CTRE Minion (TalonFXS)
BaseMotor minion = new MinionMotor(3, false, 40, false);

// All use same API
neo.set(ControlMode.VELOCITY, 50.0);  // 50 RPS
talon.set(ControlMode.POSITION, 10.0);  // 10 rotations
minion.set(ControlMode.PERCENT_OUTPUT, 0.5);  // 50% power
```

## Available Implementations

| Motor | Controller | Key Features |
|-------|-----------|--------------|
| **[NEOMotor](NEOMotor.md)** | SPARK MAX | Brushless NEO, current limiting, REVLib |
| **[TalonFXMotor](TalonFXMotor.md)** | TalonFX | Falcon 500, Kraken X60, FOC, Phoenix 6 |
| **[MinionMotor](MinionMotor.md)** | TalonFXS | Compact, lightweight, Phoenix 6 |

## Control Modes

All motors support these modes (with hardware-specific fallbacks):

- **PERCENT_OUTPUT**: -1.0 to 1.0 duty cycle
- **POSITION**: Closed-loop position (rotations)
- **VELOCITY**: Closed-loop velocity (RPS)
- **VOLTAGE**: Direct voltage control
- **CURRENT**: Current limiting (Kraken, NEO only)
- **MOTION_MAGIC**: Trapezoidal motion profiling
- **MOTION_MAGIC_FOC_TORQUE**: FOC torque mode (TalonFX only)
- **FOLLOWER**: Follow another motor

## Key Features

### Motor Interchangeability
```java
// Same code works for all motors
public class Shooter {
    private final BaseMotor motor;
    
    public Shooter(BaseMotor motor) {
        this.motor = motor;  // NEO, TalonFX, or Minion
    }
    
    public void setSpeed(double rps) {
        motor.set(ControlMode.VELOCITY, rps);
    }
}
```

### Velocity Units: RPS (Rotations Per Second)
```java
// All motors use RPS
motor.set(ControlMode.VELOCITY, 83.33);  // 83.33 RPS = 5000 RPM
double currentRPS = motor.getVelocity();  // Returns RPS
```

### Capability Detection
```java
if (motor.supportsControlMode(ControlMode.CURRENT)) {
    motor.set(ControlMode.CURRENT, 20.0);  // 20A limit
} else {
    motor.set(ControlMode.PERCENT_OUTPUT, 0.5);  // Fallback
}
```

## Hardware Comparison

| Feature | NEO | Falcon 500 | Kraken X60 | Minion |
|---------|-----|------------|------------|--------|
| **Free Speed** | 5880 RPM | 6380 RPM | 6000 RPM | 5800 RPM |
| **Stall Torque** | 3.36 Nm | 4.69 Nm | 7.09 Nm | 1.26 Nm |
| **Weight** | 453g | 435g | 435g | 150g |
| **FOC** | No | Yes | Yes | No |
| **Current Control** | Yes | No | Yes | No |
| **Encoder** | Integrated | Integrated | Integrated | Integrated |
| **Controller** | SPARK MAX | TalonFX | TalonFX | TalonFXS |

## Common Patterns

### Shooter Wheel

```java
public class Shooter {
    private final BaseMotor shooterMotor;
    
    public Shooter(BaseMotor motor) {
        this.shooterMotor = motor;
        
        // Configure PID for velocity control
        shooterMotor.setPID(0, 0.0001, 0.0, 0.0, 0.00017);
        shooterMotor.setBrakeMode(false);  // Coast mode
        shooterMotor.enableVoltageCompensation(12.0);
    }
    
    public void setVelocity(double rps) {
        shooterMotor.set(ControlMode.VELOCITY, rps);
    }
    
    public boolean atSpeed(double targetRPS, double tolerance) {
        return Math.abs(shooterMotor.getVelocity() - targetRPS) < tolerance;
    }
}
```

### Elevator with Position Control

```java
public class Elevator {
    private final BaseMotor elevatorMotor;
    
    public enum Height {
        BOTTOM(0.0),
        MID(50.0),
        TOP(100.0);
        
        public final double rotations;
        Height(double rotations) { this.rotations = rotations; }
    }
    
    public Elevator(BaseMotor motor) {
        this.elevatorMotor = motor;
        
        // Configure for position control
        elevatorMotor.setPID(0, 0.1, 0.0, 0.01, 0.0);
        elevatorMotor.setBrakeMode(true);
        elevatorMotor.configureSoftLimits(100.0, 0.0, true);
        elevatorMotor.setPosition(0.0);  // Zero at bottom
    }
    
    public void setHeight(Height height) {
        elevatorMotor.set(ControlMode.POSITION, height.rotations);
    }
    
    public double getHeight() {
        return elevatorMotor.getPosition();
    }
}
```

### Drive Motor with Motion Magic

```java
public class DriveModule {
    private final BaseMotor driveMotor;
    
    public DriveModule(BaseMotor motor) {
        this.driveMotor = motor;
        
        // Configure motion profiling
        driveMotor.setPID(0, 0.05, 0.0, 0.0, 0.0);
        driveMotor.configureMotionMagic(80.0, 160.0, 0.0);  // RPS units
        driveMotor.setBrakeMode(true);
    }
    
    public void setTargetPosition(double rotations) {
        driveMotor.set(ControlMode.MOTION_MAGIC, rotations);
    }
    
    public boolean atTarget(double tolerance) {
        double error = Math.abs(driveMotor.getPosition() - getTargetPosition());
        return error < tolerance;
    }
}
```

## Configuration Best Practices

### 1. Configure Once in Constructor
```java
public Intake(BaseMotor motor) {
    this.motor = motor;
    
    // All configuration happens here
    motor.setPID(0, 0.0001, 0.0, 0.0, 0.0);
    motor.setBrakeMode(false);
    motor.enableVoltageCompensation(12.0);
    motor.configureCurrentLimits(40, 60, 5500);
}
```

### 2. Use RPS for Velocity
```java
// Good: RPS (consistent across all motors)
motor.set(ControlMode.VELOCITY, 50.0);  // 50 RPS

// If you have RPM:
double rpm = 3000;
motor.set(ControlMode.VELOCITY, rpm / 60.0);  // Convert to RPS
```

### 3. Check Capabilities
```java
// Safe: Check before using
if (motor.supportsControlMode(ControlMode.MOTION_MAGIC_FOC_TORQUE)) {
    motor.set(ControlMode.MOTION_MAGIC_FOC_TORQUE, target);
} else {
    motor.set(ControlMode.MOTION_MAGIC, target);  // Fallback
}
```

### 4. Use Constants
```java
public class Constants {
    public static class Shooter {
        public static final int MOTOR_ID = 10;
        public static final double kP = 0.0001;
        public static final double kV = 0.00017;
        public static final double TARGET_VELOCITY_RPS = 83.33;  // 5000 RPM
    }
}
```

## Safety Features

### Current Limiting
```java
// All motors support current limiting
motor.configureCurrentLimits(40, 60, 5500);
// 40A continuous, 60A peak, 5500 RPM threshold
```

### Soft Limits
```java
// Prevent mechanism damage
motor.configureSoftLimits(100.0, 0.0, true);
// Forward limit: 100 rotations, Reverse limit: 0
```

### Voltage Compensation
```java
// Consistent behavior regardless of battery voltage
motor.enableVoltageCompensation(12.0);
```

## Troubleshooting

### Motor doesn't move
- Check CAN ID matches code
- Verify motor controller has power
- Check brake mode (may need coast)
- Verify control mode is appropriate

### Inconsistent velocity
- Add voltage compensation
- Tune PID constants
- Check for mechanical binding
- Verify current limits aren't triggering

### Position drifts
- Enable brake mode
- Tune PID (increase kP, add kI)
- Check encoder connection
- Verify no mechanical slippage

## Migration Guide

### Breaking Change: RPS Units (v2026.2.0)

NEOMotor now uses RPS instead of RPM:

```java
// Old (before v2026.2.0)
neoMotor.set(ControlMode.VELOCITY, 5000);  // RPM

// New (v2026.2.0+)
neoMotor.set(ControlMode.VELOCITY, 83.33);  // RPS (5000 RPM)
```

TalonFX and Minion already used RPS (no changes needed).

## See Also

- **[BaseMotor API](BaseMotor.md)** - Interface documentation
- **[NEOMotor](NEOMotor.md)** - REV NEO motor details
- **[TalonFXMotor](TalonFXMotor.md)** - Falcon/Kraken motor details
- **[MinionMotor](MinionMotor.md)** - Minion motor details
- **[Actuators Overview](../README.md)** - All actuator types
