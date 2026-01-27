# AdambotsLib Documentation

Welcome to the AdambotsLib documentation! This library provides reusable, production-ready components for FRC robotics teams.

## 📚 Table of Contents

### Actuators
Control hardware components with consistent, safe APIs:

- **[Actuators Overview](actuators/README.md)** - Introduction to actuator abstractions
- **[Motors](actuators/motors/README.md)** - Motor control (NEO, TalonFX/Falcon/Kraken, Minion)
- **[Servos](actuators/servos/README.md)** - Servo control (Angular, Continuous Rotation, Direct PWM)
- **[Solenoids](actuators/solenoids/README.md)** - Pneumatic and electrical solenoid control

### Sensors
Measure physical properties with validated, reliable interfaces:

- **[Sensors Overview](sensors/README.md)** - Introduction to sensor abstractions
- **[Gyroscopes](sensors/gyros/README.md)** - Orientation and rotation (Pigeon2)
- **[Encoders](sensors/encoders/README.md)** - Absolute position (CANcoder, Through Bore)
- **[Proximity Sensors](sensors/proximity/README.md)** - Object detection (Limit Switch, PhotoEye)
- **[Distance Sensors](sensors/distance/README.md)** - Range measurement (Ultrasonic, LIDAR, CANrange)

### Subsystems
Pre-built command-based subsystems for common robot components:

- **[Subsystems Overview](subsystems/README.md)** - Introduction to subsystem implementations
- **[SwerveSubsystem](subsystems/SwerveSubsystem.md)** - YAGSL-based swerve drive with PhotonVision and PathPlanner
- **[CANdleSubsystem](subsystems/CANdleSubsystem.md)** - CTRE CANdle LED strip controller (Phoenix 6)
- **[CANdleSubsystem Commands](subsystems/CANdleSubsystem-CommandFactory.md)** - Command factory methods for CANdle
- **[CameraSubsystem](subsystems/CameraSubsystem.md)** - PhotonVision camera management
- **[Custom Commands](subsystems/CustomCommands.md)** - Reusable command patterns

### Vision
Computer vision for AprilTag detection and pose estimation:

- **[Vision Overview](vision/README.md)** - Introduction to vision processing
- **[PhotonVision](vision/PhotonVision.md)** - PhotonVision integration guide
- **[Vision Configuration](vision/VisionConfiguration.md)** - Camera and pipeline setup

### Utilities
Utility classes and helpers for common programming patterns:

- **[Utils Overview](utils/README.md)** - Introduction to utility classes
- **[Utils](utils/Utils.md)** - General utility functions (alliance, math, angles, geometry)
- **[Buttons](utils/Buttons.md)** - Controller input abstraction (Xbox, PS5, joystick)
- **[Dash](utils/Dash.md)** - Simplified Shuffleboard interface
- **[StateMachine](utils/StateMachine.md)** - Type-safe state machine for managing subsystem behavior

## 🚀 Quick Start

### Installation

Add AdambotsLib to your `build.gradle`:

```gradle
repositories {
    maven {
        url = uri("https://adambots.github.io/AdambotsLib/maven")
    }
}

dependencies {
    implementation "com.adambots:AdambotsLib:2026.2.0"
}
```

### Basic Example

```java
import com.adambots.lib.actuators.*;
import com.adambots.lib.sensors.*;
import com.adambots.lib.subsystems.*;
import static edu.wpi.first.units.Units.*;

// Create a motor (NEO, TalonFX, or Minion)
BaseMotor motor = new NEOMotor(1, false, 40, false);

// Set velocity in rotations per second
motor.set(ControlMode.VELOCITY, 50.0);  // 50 RPS

// Configure PID
motor.setPID(0, 0.1, 0.0, 0.0, 0.0);

// Create sensors
BaseGyro gyro = new Gyro(1);  // Pigeon2 on CAN ID 1
BaseAbsoluteEncoder encoder = new ThroughBoreEncoder(5);  // DIO port 5
BaseProximitySensor photoEye = new PhotoEye(3, false);  // DIO port 3

// Create subsystems
CANdleSubsystem leds = new CANdleSubsystem(10);  // CAN ID 10

// Read sensor values with WPILib units
Angle heading = gyro.getYaw();
double headingDegrees = heading.in(Degrees);

Angle armAngle = encoder.getPosition();
double armDegrees = armAngle.in(Degrees);

boolean hasGamePiece = photoEye.isDetecting();

// Control LEDs
leds.setColor(LEDConstants.green);
leds.setAnimation(AnimationTypes.Rainbow);
```

## 🎯 Key Features

### Safety First
- **Input validation** on all constructors (actuators and sensors)
- **Graceful fallbacks** instead of exceptions
- **DriverStation warnings** for debugging
- **Bounds checking** on all parameters
- **Port range validation** (DIO, Analog, CAN, PWM)

### Hardware Interchangeability
Switch between different hardware without code changes:
- **Motors**: NEO, Falcon, Kraken, Minion with consistent velocity units (RPS)
- **Encoders**: CANcoder, Through Bore with unified position API
- **Sensors**: Problem-domain interfaces for all sensor types

### Trigger-Based Sensors
Sensors expose state as triggers for command-based programming:
- Boolean conditions, not raw values
- Debouncing support
- Integration with command bindings

### Comprehensive Documentation
- Hardware specifications
- Usage examples
- Best practices
- Common pitfalls

## 📖 Documentation Structure

```
docs/
├── README.md (this file)
├── actuators/
│   ├── README.md (actuators overview)
│   ├── motors/
│   │   ├── README.md
│   │   ├── BaseMotor.md
│   │   ├── NEOMotor.md
│   │   ├── TalonFXMotor.md
│   │   └── MinionMotor.md
│   ├── servos/
│   │   ├── README.md
│   │   ├── BaseServo.md
│   │   ├── AngularHubServo.md
│   │   ├── CRHubServo.md
│   │   └── DirectServo.md
│   └── solenoids/
│       ├── README.md
│       ├── BaseSolenoid.md
│       ├── ElectricalSolenoid.md
│       └── CTREPneumaticSolenoid.md
├── sensors/
│   ├── README.md (sensors overview)
│   ├── gyros/
│   │   ├── README.md
│   │   └── Gyro.md
│   ├── encoders/
│   │   ├── README.md
│   │   ├── CANCoder.md
│   │   └── ThroughBoreEncoder.md
│   ├── proximity/
│   │   ├── README.md
│   │   ├── LimitSwitch.md
│   │   └── PhotoEye.md
│   └── distance/
│       ├── README.md
│       ├── UltrasonicSensor.md
│       ├── Lidar.md
│       └── CANRangeSensor.md
├── subsystems/
│   ├── README.md (subsystems overview)
│   ├── SwerveSubsystem.md
│   ├── CANdleSubsystem.md
│   ├── CANdleSubsystem-CommandFactory.md
│   ├── CameraSubsystem.md
│   └── CustomCommands.md
├── vision/
│   ├── README.md (vision overview)
│   ├── PhotonVision.md
│   └── VisionConfiguration.md
└── utils/
    ├── README.md (utils overview)
    ├── Utils.md
    ├── Buttons.md
    ├── Dash.md
    └── StateMachine.md
```

## 🤝 Contributing

See [MAINTAINER.md](../MAINTAINER.md) for development guidelines.

## 📄 License

See [WPILib-License.md](../WPILib-License.md) for licensing information.

## 🔗 Additional Resources

- **[Repository](https://github.com/Adambots/AdambotsLib)**
- **[Usage Guide](../USAGE.md)** - Quick reference
- **[Maintainer Guide](../MAINTAINER.md)** - Development guidelines

---

**Need Help?** Check the detailed documentation for each component, or refer to the code examples in the usage guides.
