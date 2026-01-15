# AdambotsLib

Reusable components library for FRC Team 245 (Adambots).

## Documentation

📖 **[Complete API Documentation](docs/README.md)** - Comprehensive guides with examples for all components

**Quick References:**
- **[USAGE.md](USAGE.md)** - How to add and use AdambotsLib in your robot projects
- **[MAINTAINER.md](MAINTAINER.md)** - How to build, publish, and maintain the library

**Component Documentation:**
- **[Actuators](docs/actuators/README.md)** - Motors, servos, and solenoids
  - [Motors](docs/actuators/motors/README.md) - NEO, TalonFX/Falcon/Kraken, Minion
  - [Servos](docs/actuators/servos/README.md) - Angular, CR, Direct PWM
  - [Solenoids](docs/actuators/solenoids/README.md) - Pneumatic and electrical

## Quick Start

### Using AdambotsLib in Your Robot Project

In VS Code, press `Ctrl+Shift+P` (or `Cmd+Shift+P` on Mac), then:

1. Type: "WPILib: Manage Vendor Libraries"
2. Select: "Install new library (online)"
3. Enter: `https://raw.githubusercontent.com/Adambots-245/AdambotsLib/main/AdambotsLib.json`

See [USAGE.md](USAGE.md) for complete instructions and examples.

### Publishing a New Version (Maintainers)

```bash
# 1. Update version in gradle.properties
# 2. Build and publish
./publish-to-maven.sh

# 3. Commit and push
git add AdambotsLib.json maven/ gradle.properties
git commit -m "Release v2026.1.0"
git push
```

See [MAINTAINER.md](MAINTAINER.md) for complete publishing guide.

## Library Components

**Actuators:**
- NEOMotor - REV Robotics NEO/NEO 550 motor controller wrapper
- TalonFXMotor - CTRE Falcon 500/Kraken X60 motor controller wrapper
- MinionMotor - CTRE TalonFXS motor controller wrapper
- AngularHubServo - REV Robotics servo control

**Subsystems:**
- SwerveSubsystem - Complete swerve drive subsystem using YAGSL

**Commands:**
- DriveCommands - Pre-built swerve drive commands

**Vision:**
- PhotonVision - Camera and vision processing integration

## Information

**Current Version**: 2026.1.0
**FRC Year**: 2026
**WPILib Version**: 2026.1.1

**Repository**: https://github.com/Adambots-245/AdambotsLib
**Maven**: https://adambots-245.github.io/AdambotsLib/maven
**Vendordep**: https://raw.githubusercontent.com/Adambots-245/AdambotsLib/main/AdambotsLib.json

## Dependencies

AdambotsLib automatically includes:
- Phoenix6 (26.1.0)
- Phoenix5 (5.36.0)
- REVLib (2026.0.1)
- ReduxLib (2026.1.1)
- YAGSL (2026.1.12)
- PathplannerLib (2026.1.2)
- PhotonLib (v2025.3.1)
- And other vendor libraries

## Building

To build the library locally:

```bash
./gradlew build
```

To test locally before publishing:

```bash
./gradlew publishToMavenLocal
```

## License

This library is maintained by FRC Team 245 (Adambots) for use in FRC competitions.
