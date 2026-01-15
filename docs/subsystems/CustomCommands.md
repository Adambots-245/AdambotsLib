# Adding Custom Commands to AdambotsLib Subsystems

Guide for teams to add game-specific commands without modifying AdambotsLib code.

## Table of Contents

- [Overview](#overview)
- [Recommended Approach](#recommended-approach)
- [Command Factory Pattern](#command-factory-pattern)
- [Inline Command Composition](#inline-command-composition)
- [Complete Example](#complete-example)
- [Common Patterns by Subsystem](#common-patterns-by-subsystem)
- [Best Practices](#best-practices)

---

## Overview

AdambotsLib provides robust, tested subsystems (SwerveSubsystem, CameraSubsystem, CANdleSubsystem) that work across seasons. However, each year's game requires custom commands specific to that competition.

**The Problem:**
- Teams need to add game-specific functionality
- AdambotsLib code should remain unchanged
- Updates to AdambotsLib shouldn't break team code

**The Solution:**
Use **Command Factory Pattern** and **Inline Composition** to add custom commands in your team's codebase without modifying the library.

**What NOT to Do:**
- ❌ Don't subclass AdambotsLib subsystems
- ❌ Don't modify AdambotsLib source code
- ❌ Don't copy-paste library code into your repo

**What TO Do:**
- ✅ Create command factory classes in your codebase
- ✅ Use inline command composition for simple commands
- ✅ Build on AdambotsLib's public API
- ✅ Keep game-specific code separate

---

## Recommended Approach

### Two Main Patterns

#### 1. Command Factory Pattern (Most Commands)
Create factory classes that use subsystem public APIs to build game-specific commands.

**Use When:**
- Command will be reused multiple times
- Command involves complex logic
- Command combines multiple subsystems
- You want testable, maintainable code

#### 2. Inline Command Composition (Simple Commands)
Compose commands directly in RobotContainer using WPILib's command composition.

**Use When:**
- Command is used only once
- Logic is straightforward (1-3 steps)
- Command doesn't need testing in isolation
- Quick prototyping

---

## Command Factory Pattern

### Structure

Create a command factory class in your team's codebase that accepts subsystems and returns Commands.

```java
// In your team's code: frc/robot/commands/CustomDriveCommands.java
package frc.robot.commands;

import com.adambots.lib.subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class CustomDriveCommands {
    private final SwerveSubsystem swerve;

    public CustomDriveCommands(SwerveSubsystem swerve) {
        this.swerve = swerve;
    }

    // Add your custom commands here
}
```

### REBUILT 2026 Examples

#### Example 1: Auto-Collect Fuel from Depot

```java
public class CustomDriveCommands {
    private final SwerveSubsystem swerve;
    private final IntakeSubsystem intake;

    public CustomDriveCommands(SwerveSubsystem swerve, IntakeSubsystem intake) {
        this.swerve = swerve;
        this.intake = intake;
    }

    /**
     * REBUILT 2026: Navigate to depot and collect fuel.
     * Uses vision to align with depot AprilTag, then collects fuel.
     */
    public Command collectFuelFromDepot() {
        return Commands.sequence(
            // Drive to depot position
            swerve.driveToPoseCommand(FieldPositions.DEPOT_APPROACH),

            // Align with depot AprilTag (assuming tag ID 5)
            swerve.aimAtAprilTagCommand(5, 2.0),

            // Deploy intake and collect fuel
            Commands.parallel(
                intake.deployCommand(),
                swerve.driveToDistanceCommand(0.5, 0.3)  // Approach slowly
            ),

            // Wait for fuel detection
            Commands.waitUntil(intake::hasFuel),

            // Retract intake
            intake.stowCommand()
        ).withName("CollectFuelFromDepot");
    }
}
```

#### Example 2: Score Fuel in Hub

```java
/**
 * REBUILT 2026: Drive to hub and score fuel.
 * Approaches hub using vision alignment and shoots when positioned.
 */
public Command scoreFuelInHub() {
    return Commands.sequence(
        // Navigate to hub approach position
        swerve.driveToPoseCommand(FieldPositions.HUB_SCORING_POSITION),

        // Align with hub using vision (assuming hub has AprilTag)
        swerve.aimAtAprilTagCommand(FieldPositions.HUB_TAG_ID, 1.5),

        // Fine-tune distance
        Commands.run(() -> {
            double distance = swerve.getVision().getDistanceFromAprilTag(
                FieldPositions.HUB_TAG_ID
            );
            double error = FieldPositions.HUB_TARGET_DISTANCE - distance;
            swerve.drive(
                new Translation2d(error * 0.5, 0),  // Proportional approach
                0,
                true
            );
        }, swerve)
        .until(() -> {
            double distance = swerve.getVision().getDistanceFromAprilTag(
                FieldPositions.HUB_TAG_ID
            );
            return Math.abs(distance - FieldPositions.HUB_TARGET_DISTANCE) < 0.1;
        }),

        // Spin up shooter and shoot
        shooter.spinUpCommand(),
        Commands.waitSeconds(0.5),
        shooter.shootCommand()
    ).withName("ScoreFuelInHub");
}
```

#### Example 3: Navigate Trench with Vision

```java
/**
 * REBUILT 2026: Navigate through trench using AprilTags for guidance.
 * Follows trench markers to safely traverse obstacle.
 */
public Command navigateTrenchWithVision() {
    return Commands.sequence(
        // Align with first trench marker
        swerve.aimAtAprilTagCommand(FieldPositions.TRENCH_ENTRY_TAG, 2.0),

        // Drive through trench following markers
        Commands.run(() -> {
            // Check which trench tag is visible
            int visibleTag = swerve.getVision().hasID(
                FieldPositions.TRENCH_TAG_IDS
            );

            if (visibleTag != -1) {
                // Calculate drive direction toward visible marker
                Rotation2d yaw = swerve.getVision().getYawToAprilTag(visibleTag);
                double forwardSpeed = 1.0;
                double rotationSpeed = yaw.getRadians() * 0.5;  // Proportional steering

                swerve.drive(
                    new Translation2d(forwardSpeed, 0),
                    rotationSpeed,
                    true
                );
            } else {
                // No marker visible, stop
                swerve.lock();
            }
        }, swerve)
        .until(() -> {
            // Exit when we see the exit marker
            return swerve.getVision().isTagVisible(FieldPositions.TRENCH_EXIT_TAG);
        }),

        // Final alignment at trench exit
        swerve.aimAtAprilTagCommand(FieldPositions.TRENCH_EXIT_TAG, 2.0)
    ).withName("NavigateTrenchWithVision");
}
```

#### Example 4: Autonomous Fuel Cycle

```java
/**
 * REBUILT 2026: Complete fuel collection and scoring cycle.
 * Collects fuel from field, scores in hub, repeats.
 */
public Command autonomousFuelCycle(int numCycles) {
    return Commands.sequence(
        // Start with pre-loaded fuel
        scoreFuelInHub(),

        // Repeat collection and scoring
        Commands.repeating(
            Commands.sequence(
                collectFuelFromDepot(),
                scoreFuelInHub()
            )
        ).withTimeout(numCycles * 10.0),  // Estimate 10 seconds per cycle

        // End game: Drive to tower
        swerve.driveToPoseCommand(FieldPositions.TOWER_APPROACH)
    ).withName("AutonomousFuelCycle");
}
```

#### Example 5: Multi-Subsystem Coordination

```java
public class RobotCommands {
    private final SwerveSubsystem swerve;
    private final IntakeSubsystem intake;
    private final ShooterSubsystem shooter;
    private final ClimberSubsystem climber;
    private final CANdleSubsystem leds;

    public RobotCommands(
        SwerveSubsystem swerve,
        IntakeSubsystem intake,
        ShooterSubsystem shooter,
        ClimberSubsystem climber,
        CANdleSubsystem leds
    ) {
        this.swerve = swerve;
        this.intake = intake;
        this.shooter = shooter;
        this.climber = climber;
        this.leds = leds;
    }

    /**
     * REBUILT 2026: Complete match sequence with LED feedback.
     */
    public Command completeMatchSequence() {
        return Commands.sequence(
            // Autonomous: Score pre-loaded fuel
            Commands.parallel(
                scoreFuelInHub(),
                leds.setColorCommand(Color.kBlue)
            ),

            // Teleop indicator
            Commands.waitUntil(DriverStation::isTeleop),
            leds.animationCommand(CANdleSubsystem.AnimationTypes.Rainbow)
                .withTimeout(2.0),

            // Teleop: Human-driven with LED status
            Commands.parallel(
                // Drive status LEDs
                Commands.run(() -> {
                    if (intake.hasFuel()) {
                        leds.setColor(Color.kGreen);
                    } else if (shooter.isSpunUp()) {
                        leds.setColor(Color.kYellow);
                    } else {
                        leds.setColor(Color.kWhite);
                    }
                }),

                // Auto-stow intake when moving fast
                Commands.run(() -> {
                    if (swerve.getRobotVelocity() > 3.0 && intake.isDeployed()) {
                        intake.stow();
                    }
                })
            ).until(() -> DriverStation.getMatchTime() < 30),

            // End game: Climb tower
            Commands.sequence(
                swerve.driveToPoseCommand(FieldPositions.TOWER_APPROACH),
                leds.setColorCommand(Color.kPurple),
                climber.climbSequenceCommand()
            )
        ).withName("CompleteMatchSequence");
    }
}
```

### Field Positions Helper Class

Create a constants class for game-specific positions:

```java
// In your team's code: frc/robot/FieldPositions.java
package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

/**
 * REBUILT 2026 field positions and constants.
 */
public class FieldPositions {
    // Hub scoring
    public static final int HUB_TAG_ID = 7;
    public static final double HUB_TARGET_DISTANCE = 1.5;  // meters
    public static final Pose2d HUB_SCORING_POSITION =
        new Pose2d(8.27, 4.11, Rotation2d.fromDegrees(0));

    // Depot positions
    public static final Pose2d DEPOT_APPROACH =
        new Pose2d(2.5, 7.5, Rotation2d.fromDegrees(90));

    // Trench navigation
    public static final int TRENCH_ENTRY_TAG = 10;
    public static final int TRENCH_EXIT_TAG = 11;
    public static final int[] TRENCH_TAG_IDS = {10, 11, 12};

    // Tower climbing
    public static final Pose2d TOWER_APPROACH =
        new Pose2d(5.5, 2.5, Rotation2d.fromDegrees(180));

    // Alliance-specific helpers
    public static Pose2d mirrorForAlliance(Pose2d pose) {
        return Utils.mirrorPoseForAlliance(pose, 16.54);  // Field length
    }
}
```

---

## Inline Command Composition

For simple, one-off commands, compose directly in RobotContainer.

### REBUILT 2026 Examples

#### Example 1: Quick Fuel Eject

```java
public class RobotContainer {
    private void configureButtonBindings() {
        // REBUILT 2026: Emergency fuel eject
        emergencyButton.onTrue(
            Commands.parallel(
                intake.reverseCommand(),
                shooter.reverseCommand(),
                leds.setColorCommand(Color.kRed)
            ).withTimeout(1.0)
            .withName("EmergencyEject")
        );
    }
}
```

#### Example 2: Backup and Realign

```java
private void configureButtonBindings() {
    // REBUILT 2026: Backup from hub if shot missed
    missedShotButton.onTrue(
        Commands.sequence(
            swerve.driveToDistanceCommand(-1.0, 0.5),  // Back up 1 meter
            Commands.waitSeconds(0.5),
            swerve.aimAtAprilTagCommand(FieldPositions.HUB_TAG_ID, 2.0)
        ).withName("BackupAndRealign")
    );
}
```

#### Example 3: Quick Collection

```java
private void configureButtonBindings() {
    // REBUILT 2026: Quick fuel collection from ground
    collectButton.whileTrue(
        Commands.parallel(
            intake.deployCommand(),
            Commands.run(() ->
                swerve.drive(new Translation2d(0.5, 0), 0, true),
                swerve
            )
        ).finallyDo(() -> {
            if (intake.hasFuel()) {
                intake.stow();
            }
        })
    );
}
```

#### Example 4: Camera Switching Based on Location

```java
private void configureAutoSwitching() {
    // REBUILT 2026: Auto-switch cameras based on field zone

    // Near depot = rear camera
    new Trigger(() -> {
        Pose2d pose = swerve.getPose();
        return pose.getX() < 3.0;  // Near alliance wall (depot area)
    }).onTrue(camera.switchToCamera2Command())
      .onFalse(camera.switchToCamera1Command());

    // Near hub = front camera for scoring
    new Trigger(() -> {
        double distance = swerve.getVision().getDistanceFromAprilTag(
            FieldPositions.HUB_TAG_ID
        );
        return distance > 0 && distance < 3.0;
    }).onTrue(camera.switchToCamera1Command());
}
```

#### Example 5: LED Status Indicators

```java
private void configureLEDIndicators() {
    // REBUILT 2026: Visual feedback for drivers

    // Has fuel = green
    intake.hasFuelTrigger()
        .whileTrue(leds.setColorCommand(Color.kGreen))
        .whileFalse(leds.setColorCommand(Color.kWhite));

    // Shooter ready = yellow
    shooter.isReadyTrigger()
        .whileTrue(leds.setColorCommand(Color.kYellow));

    // At scoring position = purple
    new Trigger(() -> {
        double distance = swerve.getVision().getDistanceFromAprilTag(
            FieldPositions.HUB_TAG_ID
        );
        return Math.abs(distance - FieldPositions.HUB_TARGET_DISTANCE) < 0.2;
    }).whileTrue(leds.setColorCommand(Color.kPurple));

    // Alliance color during auto
    new Trigger(DriverStation::isAutonomous)
        .whileTrue(Commands.run(() -> {
            Optional<Alliance> alliance = DriverStation.getAlliance();
            if (alliance.isPresent()) {
                leds.setColor(alliance.get() == Alliance.Red ?
                    Color.kRed : Color.kBlue);
            }
        }, leds));
}
```

---

## Complete Example

Here's a complete robot code structure using AdambotsLib subsystems with REBUILT 2026 game logic:

### Project Structure

```
frc/robot/
├── Robot.java
├── RobotContainer.java
├── Constants.java
├── FieldPositions.java              // REBUILT 2026 field positions
├── subsystems/
│   ├── IntakeSubsystem.java         // Team-specific
│   ├── ShooterSubsystem.java        // Team-specific
│   └── ClimberSubsystem.java        // Team-specific
└── commands/
    ├── CustomDriveCommands.java     // Drive command factory
    ├── AutoCommands.java             // Auto routines
    └── RobotCommands.java            // Multi-subsystem commands
```

### RobotContainer.java

```java
package frc.robot;

import com.adambots.lib.subsystems.SwerveSubsystem;
import com.adambots.lib.subsystems.CameraSubsystem;
import com.adambots.lib.subsystems.CANdleSubsystem;
import com.adambots.lib.utils.Buttons;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

public class RobotContainer {
    // AdambotsLib subsystems (no modification needed)
    private final SwerveSubsystem swerve;
    private final CameraSubsystem camera;
    private final CANdleSubsystem leds;

    // Team-specific subsystems
    private final IntakeSubsystem intake;
    private final ShooterSubsystem shooter;
    private final ClimberSubsystem climber;

    // Command factories
    private final CustomDriveCommands customDrive;
    private final AutoCommands autoCommands;
    private final RobotCommands robotCommands;

    public RobotContainer() {
        // Initialize AdambotsLib subsystems
        swerve = new SwerveSubsystem(
            Constants.Swerve.MAX_SPEED,
            Constants.Swerve.DRIVE_BASE_RADIUS,
            Constants.Swerve.CONTROLLER_PROPERTIES
        );

        camera = new CameraSubsystem(
            CameraSubsystem.CameraConfig.DRIVER_OPTIMIZED,
            true,   // Front camera
            true    // Rear camera
        );

        leds = new CANdleSubsystem(Constants.CANdle.CAN_ID);

        // Initialize team subsystems
        intake = new IntakeSubsystem();
        shooter = new ShooterSubsystem();
        climber = new ClimberSubsystem();

        // Initialize command factories with dependencies
        customDrive = new CustomDriveCommands(swerve, intake, shooter);
        autoCommands = new AutoCommands(swerve, intake, shooter, leds);
        robotCommands = new RobotCommands(swerve, intake, shooter, climber, leds);

        configureDefaultCommands();
        configureButtonBindings();
        configureLEDIndicators();
        configureAutoSwitching();
    }

    private void configureDefaultCommands() {
        // Swerve default: Teleop drive with field-oriented control
        swerve.setDefaultCommand(
            swerve.driveCommand(
                Buttons.createForwardSupplier(0.05, Buttons.InputCurve.CUBIC),
                Buttons.createStrafeSupplier(0.05, Buttons.InputCurve.CUBIC),
                Buttons.createRotationSupplier(0.1, Buttons.InputCurve.SIGMOID)
            )
        );

        // LEDs default: Alliance color
        leds.setDefaultCommand(
            Commands.run(() -> {
                Optional<Alliance> alliance = DriverStation.getAlliance();
                if (alliance.isPresent()) {
                    leds.setColor(alliance.get() == Alliance.Red ?
                        Color.kRed : Color.kBlue);
                }
            }, leds)
        );
    }

    private void configureButtonBindings() {
        // ===== REBUILT 2026 GAME COMMANDS =====

        // A Button: Collect fuel from depot
        Buttons.XboxA.onTrue(customDrive.collectFuelFromDepot());

        // B Button: Score fuel in hub
        Buttons.XboxB.onTrue(customDrive.scoreFuelInHub());

        // X Button: Navigate trench
        Buttons.XboxX.onTrue(customDrive.navigateTrenchWithVision());

        // Y Button: Emergency eject fuel (inline composition)
        Buttons.XboxY.onTrue(
            Commands.parallel(
                intake.reverseCommand(),
                shooter.reverseCommand()
            ).withTimeout(1.0)
            .withName("EmergencyEject")
        );

        // Left Bumper: Deploy intake
        Buttons.XboxLeftBumper.whileTrue(intake.deployCommand());

        // Right Bumper: Shoot
        Buttons.XboxRightBumper.onTrue(shooter.shootCommand());

        // Start: Reset gyro
        Buttons.XboxStart.onTrue(swerve.resetGyroCommand());

        // Back: Lock wheels
        Buttons.XboxBack.onTrue(swerve.lockCommand());

        // D-Pad Up: Increase camera brightness
        Buttons.DPadUp.onTrue(camera.increaseBrightnessCommand());

        // D-Pad Down: Decrease camera brightness
        Buttons.DPadDown.onTrue(camera.decreaseBrightnessCommand());

        // Left Trigger: Switch to rear camera
        Buttons.XboxLeftTrigger.onTrue(camera.switchToCamera2Command());

        // Right Trigger: Switch to front camera
        Buttons.XboxRightTrigger.onTrue(camera.switchToCamera1Command());
    }

    private void configureLEDIndicators() {
        // REBUILT 2026: Visual feedback
        intake.hasFuelTrigger()
            .whileTrue(leds.setColorCommand(Color.kGreen));

        shooter.isReadyTrigger()
            .whileTrue(leds.setColorCommand(Color.kYellow));
    }

    private void configureAutoSwitching() {
        // Auto-switch cameras based on field zone
        new Trigger(() -> swerve.getPose().getX() < 3.0)
            .onTrue(camera.switchToCamera2Command())
            .onFalse(camera.switchToCamera1Command());
    }

    public Command getAutonomousCommand() {
        // REBUILT 2026: Run 3-fuel autonomous
        return autoCommands.threeFuelAuto();
    }
}
```

### CustomDriveCommands.java

```java
package frc.robot.commands;

import com.adambots.lib.subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.FieldPositions;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class CustomDriveCommands {
    private final SwerveSubsystem swerve;
    private final IntakeSubsystem intake;
    private final ShooterSubsystem shooter;

    public CustomDriveCommands(
        SwerveSubsystem swerve,
        IntakeSubsystem intake,
        ShooterSubsystem shooter
    ) {
        this.swerve = swerve;
        this.intake = intake;
        this.shooter = shooter;
    }

    // REBUILT 2026: Collect fuel from depot
    public Command collectFuelFromDepot() {
        return Commands.sequence(
            swerve.driveToPoseCommand(FieldPositions.DEPOT_APPROACH),
            swerve.aimAtAprilTagCommand(5, 2.0),
            Commands.parallel(
                intake.deployCommand(),
                swerve.driveToDistanceCommand(0.5, 0.3)
            ),
            Commands.waitUntil(intake::hasFuel),
            intake.stowCommand()
        ).withName("CollectFuelFromDepot");
    }

    // REBUILT 2026: Score fuel in hub
    public Command scoreFuelInHub() {
        return Commands.sequence(
            swerve.driveToPoseCommand(FieldPositions.HUB_SCORING_POSITION),
            swerve.aimAtAprilTagCommand(FieldPositions.HUB_TAG_ID, 1.5),
            shooter.spinUpCommand(),
            Commands.waitSeconds(0.5),
            shooter.shootCommand()
        ).withName("ScoreFuelInHub");
    }

    // REBUILT 2026: Navigate trench with vision
    public Command navigateTrenchWithVision() {
        return Commands.sequence(
            swerve.aimAtAprilTagCommand(FieldPositions.TRENCH_ENTRY_TAG, 2.0),
            Commands.run(() -> {
                int visibleTag = swerve.getVision().hasID(
                    FieldPositions.TRENCH_TAG_IDS
                );
                if (visibleTag != -1) {
                    Rotation2d yaw = swerve.getVision().getYawToAprilTag(visibleTag);
                    swerve.drive(
                        new Translation2d(1.0, 0),
                        yaw.getRadians() * 0.5,
                        true
                    );
                } else {
                    swerve.lock();
                }
            }, swerve)
            .until(() -> swerve.getVision().isTagVisible(
                FieldPositions.TRENCH_EXIT_TAG
            )),
            swerve.aimAtAprilTagCommand(FieldPositions.TRENCH_EXIT_TAG, 2.0)
        ).withName("NavigateTrenchWithVision");
    }
}
```

### AutoCommands.java

```java
package frc.robot.commands;

import com.adambots.lib.subsystems.SwerveSubsystem;
import com.adambots.lib.subsystems.CANdleSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class AutoCommands {
    private final SwerveSubsystem swerve;
    private final IntakeSubsystem intake;
    private final ShooterSubsystem shooter;
    private final CANdleSubsystem leds;
    private final CustomDriveCommands customDrive;

    public AutoCommands(
        SwerveSubsystem swerve,
        IntakeSubsystem intake,
        ShooterSubsystem shooter,
        CANdleSubsystem leds
    ) {
        this.swerve = swerve;
        this.intake = intake;
        this.shooter = shooter;
        this.leds = leds;
        this.customDrive = new CustomDriveCommands(swerve, intake, shooter);
    }

    /**
     * REBUILT 2026: Score pre-loaded fuel and collect/score two more.
     */
    public Command threeFuelAuto() {
        return Commands.sequence(
            // Score pre-loaded fuel
            leds.setColorCommand(Color.kBlue),
            customDrive.scoreFuelInHub(),

            // Collect and score fuel 1
            leds.setColorCommand(Color.kGreen),
            customDrive.collectFuelFromDepot(),
            customDrive.scoreFuelInHub(),

            // Collect and score fuel 2
            customDrive.collectFuelFromDepot(),
            customDrive.scoreFuelInHub(),

            // Drive to tower approach
            leds.setColorCommand(Color.kPurple),
            swerve.driveToPoseCommand(FieldPositions.TOWER_APPROACH)
        ).withName("ThreeFuelAuto");
    }

    /**
     * REBUILT 2026: Navigate trench and score.
     */
    public Command trenchNavigationAuto() {
        return Commands.sequence(
            leds.setColorCommand(Color.kYellow),
            customDrive.navigateTrenchWithVision(),
            customDrive.collectFuelFromDepot(),
            customDrive.scoreFuelInHub()
        ).withName("TrenchNavigationAuto");
    }
}
```

---

## Common Patterns by Subsystem

### SwerveSubsystem Patterns

#### Pattern 1: Vision-Assisted Approach
```java
// Approach target using vision alignment
public Command approachTarget(int tagID, double targetDistance) {
    return Commands.sequence(
        swerve.aimAtAprilTagCommand(tagID, 2.0),
        Commands.run(() -> {
            double distance = swerve.getVision().getDistanceFromAprilTag(tagID);
            double error = targetDistance - distance;
            swerve.drive(new Translation2d(error * 0.5, 0), 0, true);
        }, swerve)
        .until(() -> {
            double distance = swerve.getVision().getDistanceFromAprilTag(tagID);
            return Math.abs(distance - targetDistance) < 0.1;
        })
    );
}
```

#### Pattern 2: Path Following with Callbacks
```java
// Drive path and execute action at waypoints
public Command drivePathWithActions(Pose2d[] waypoints, Runnable[] actions) {
    Command[] commands = new Command[waypoints.length];
    for (int i = 0; i < waypoints.length; i++) {
        int index = i;
        commands[i] = Commands.sequence(
            swerve.driveToPoseCommand(waypoints[i]),
            Commands.runOnce(actions[index])
        );
    }
    return Commands.sequence(commands);
}
```

#### Pattern 3: Dynamic Target Selection
```java
// Choose closest target and drive to it
public Command driveToClosestTarget(Pose2d[] targets) {
    return Commands.defer(() -> {
        Pose2d closest = findClosestPose(targets, swerve.getPose());
        return swerve.driveToPoseCommand(closest);
    }, Set.of(swerve));
}

private Pose2d findClosestPose(Pose2d[] poses, Pose2d current) {
    Pose2d closest = poses[0];
    double minDistance = Double.MAX_VALUE;
    for (Pose2d pose : poses) {
        double distance = current.getTranslation()
            .getDistance(pose.getTranslation());
        if (distance < minDistance) {
            minDistance = distance;
            closest = pose;
        }
    }
    return closest;
}
```

### CameraSubsystem Patterns

#### Pattern 1: Zone-Based Auto-Switching
```java
// Switch cameras based on robot zone
private void configureZoneSwitching() {
    new Trigger(() -> isInScoringZone())
        .onTrue(camera.switchToCamera1Command());

    new Trigger(() -> isInLoadingZone())
        .onTrue(camera.switchToCamera2Command());
}

private boolean isInScoringZone() {
    return swerve.getPose().getX() > 8.0;
}

private boolean isInLoadingZone() {
    return swerve.getPose().getX() < 3.0;
}
```

#### Pattern 2: Brightness Auto-Adjustment
```java
// Adjust brightness based on time of day (for outdoor regionals)
private void configureAutoBrightness() {
    new Trigger(() -> {
        LocalTime now = LocalTime.now();
        return now.isAfter(LocalTime.of(17, 0));  // After 5 PM
    }).onTrue(camera.setBrightnessCommand(120))   // Darker
      .onFalse(camera.setBrightnessCommand(80));  // Brighter
}
```

### CANdleSubsystem Patterns

#### Pattern 1: State-Based LED Feedback
```java
// LEDs show robot state
private void configureStateLEDs() {
    // Idle = white
    leds.setDefaultCommand(leds.setColorCommand(Color.kWhite));

    // Has game piece = green
    intake.hasFuelTrigger().whileTrue(leds.setColorCommand(Color.kGreen));

    // Ready to shoot = yellow
    shooter.isReadyTrigger().whileTrue(leds.setColorCommand(Color.kYellow));

    // Climbing = purple
    climber.isClimbingTrigger().whileTrue(leds.setColorCommand(Color.kPurple));

    // Error = red blinking
    new Trigger(() -> DriverStation.isFMSAttached() && !swerve.isAnyCameraConnected())
        .whileTrue(leds.animationCommand(CANdleSubsystem.AnimationTypes.Strobe));
}
```

#### Pattern 2: Match Timer Feedback
```java
// LEDs indicate match time remaining
private void configureTimerLEDs() {
    new Trigger(() -> DriverStation.getMatchTime() < 30)
        .onTrue(leds.animationCommand(CANdleSubsystem.AnimationTypes.Fire));

    new Trigger(() -> DriverStation.getMatchTime() < 10)
        .onTrue(leds.animationCommand(CANdleSubsystem.AnimationTypes.Strobe));
}
```

---

## Best Practices

### 1. Keep AdambotsLib Code Unchanged

**DO:**
```java
// Use public API in your command factory
public Command scoreFuel() {
    return Commands.sequence(
        swerve.driveToPoseCommand(scoringPose),
        shooter.shootCommand()
    );
}
```

**DON'T:**
```java
// Don't modify AdambotsLib source
public class MySwerve extends SwerveSubsystem {
    // ❌ Avoid subclassing
}
```

### 2. Use Descriptive Command Names

**DO:**
```java
return customDrive.collectFuelFromDepot()
    .withName("CollectFuelFromDepot");
```

**DON'T:**
```java
return someCommand().withName("command1");
```

### 3. Organize by Game Elements

```
commands/
├── CustomDriveCommands.java    // Drive-related
├── FuelCommands.java            // Fuel collection/scoring
├── TowerCommands.java           // Climbing
└── AutoCommands.java            // Complete auto routines
```

### 4. Document Game-Specific Logic

```java
/**
 * REBUILT 2026: Navigate through trench using vision markers.
 *
 * Uses AprilTags 10-12 to guide through obstacle.
 * Trench is 3 meters long with markers every meter.
 *
 * @return Command to navigate trench
 */
public Command navigateTrench() {
    // ...
}
```

### 5. Make Commands Reusable

**DO:**
```java
// Parameterized for reusability
public Command scoreFuelInHub(int hubTagID) {
    return Commands.sequence(
        swerve.aimAtAprilTagCommand(hubTagID, 2.0),
        shooter.shootCommand()
    );
}
```

**DON'T:**
```java
// Hardcoded, not reusable
public Command scoreFuelInRedHub() {
    return Commands.sequence(
        swerve.aimAtAprilTagCommand(7, 2.0),  // Red hub only
        shooter.shootCommand()
    );
}
```

### 6. Use Inline Composition for Simple Commands

**When to use inline:**
- Used only once
- 1-3 steps
- No complex logic

```java
button.onTrue(
    Commands.sequence(
        intake.deploy(),
        Commands.waitSeconds(0.5),
        intake.collect()
    )
);
```

**When to use factory:**
- Reused multiple times
- Complex logic
- Needs testing
- Combines multiple subsystems

### 7. Test Commands Independently

Create unit tests for command factories:

```java
@Test
public void testCollectFuelFromDepot() {
    Command command = customDrive.collectFuelFromDepot();
    assertNotNull(command);
    assertEquals("CollectFuelFromDepot", command.getName());
}
```

### 8. Use Triggers for Reactive Behavior

```java
// React to game state changes
intake.hasFuelTrigger()
    .onTrue(leds.setColorCommand(Color.kGreen))
    .onFalse(leds.setColorCommand(Color.kWhite));
```

### 9. Handle Edge Cases

```java
// Check for vision target before using
public Command approachHub() {
    return Commands.either(
        // If hub visible, approach it
        swerve.aimAtAprilTagCommand(HUB_TAG_ID, 2.0),
        // Otherwise, drive to known position first
        Commands.sequence(
            swerve.driveToPoseCommand(HUB_APPROACH_POSE),
            swerve.aimAtAprilTagCommand(HUB_TAG_ID, 2.0)
        ),
        () -> swerve.getVision().isTagVisible(HUB_TAG_ID)
    );
}
```

### 10. Version Control Game-Specific Code

```java
/**
 * Command factory for REBUILT 2026 season.
 *
 * @version 2026.1.0
 * @author Team 245 - AdamBots
 */
public class CustomDriveCommands {
    // ...
}
```

---

## Summary

### Key Takeaways

1. **Never modify AdambotsLib code** - Use command factories and composition
2. **Command Factory Pattern** - Primary approach for reusable commands
3. **Inline Composition** - Quick approach for simple one-off commands
4. **No Inheritance** - Don't subclass library subsystems
5. **Organize by Game Elements** - Keep code structure logical
6. **Document Game Context** - Make it clear what's game-specific
7. **Test Independently** - Command factories should be unit testable

### Recommended File Structure

```
frc/robot/
├── Robot.java
├── RobotContainer.java
├── Constants.java
├── FieldPositions.java          // REBUILT 2026 field positions
├── subsystems/
│   └── (team-specific subsystems)
└── commands/
    ├── CustomDriveCommands.java  // Drive command factory
    ├── FuelCommands.java          // Fuel handling
    ├── AutoCommands.java          // Autonomous routines
    └── RobotCommands.java         // Multi-subsystem coordination
```

### Resources

- [WPILib Command-Based Programming](https://docs.wpilib.org/en/stable/docs/software/commandbased/index.html)
- [REBUILT 2026 Game Manual](https://www.firstinspires.org/resource-library/frc/competition-manual-qa-system)
- [AdambotsLib Documentation](../README.md)

---

**Need Help?** Ask your programming mentor or post in the Discord #programming channel with specific examples from this guide.
