# Mechanisms

AdambotsLib's mechanism classes wrap [YAMS](https://github.com/Yet-Another-Software-Suite/YAMS)
positional and velocity mechanisms behind a stable, AdambotsLib-native API. Your
team code deals only with `BaseMotor` + `AdambotsArm` / `AdambotsElevator` /
`AdambotsFlyWheel` / `AdambotsPivot` — YAMS types never leak into user code.

## Available mechanisms

| Class              | YAMS backing                         | Use for                                    |
| ------------------ | ------------------------------------ | ------------------------------------------ |
| `AdambotsArm`      | `yams.mechanisms.positional.Arm`     | Cantilevered arms (gravity matters)        |
| `AdambotsPivot`    | `yams.mechanisms.positional.Pivot`   | Turrets, wrists (rotation without gravity) |
| `AdambotsElevator` | `yams.mechanisms.positional.Elevator`| Linear stages, cascading lifts             |
| `AdambotsFlyWheel` | `yams.mechanisms.velocity.FlyWheel`  | Shooters, intakes, any velocity target     |

## Design

- **Composition over inheritance** — each `Adambots*` class *contains* a YAMS
  mechanism. You extend the wrapper to add team behavior; YAMS internals stay
  behind the curtain.
- **Single public motor type** — mechanisms accept `BaseMotor`. Internally,
  `MotorBridge` translates to YAMS's `SmartMotorController`. You never touch
  YAMS directly.
- **Phoenix Pro features are opt-in and safe** — `withFOC(true)` and
  `withMotionMagicExpo(...)` log a `DriverStation` warning if applied to a
  non-Phoenix motor (e.g. a NEO) and fall back to the trapezoid profile.
- **Visualizer wiring is automatic** — call `withVisualizer(mv, index)` on the
  config and the mechanism's `Pose3d` auto-publishes to the supplied
  `MechanismVisualizer` each update.

## Example: ArmSubsystem

```java
package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.adambots.lib.actuators.BaseMotor;
import com.adambots.lib.actuators.TalonFXMotor;
import com.adambots.lib.mechanisms.AdambotsArm;
import com.adambots.lib.mechanisms.config.AdambotsArmConfig;
import com.adambots.lib.visualization.MechanismVisualizer;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;

public class ArmSubsystem extends SubsystemBase {

    private final AdambotsArm arm;

    public ArmSubsystem(MechanismVisualizer visualizer) {
        // Hardware: Kraken X60 on CAN ID 21, inverted, 40A limit
        BaseMotor motor = new TalonFXMotor(21, true, 40, true);

        AdambotsArmConfig config = new AdambotsArmConfig()
            // Control tuning
            .withPID(2.5, 0.0, 0.1)
            .withMotionMagic(/*cruiseRPS*/ 2.0, /*accelRPSps*/ 4.0)
            .withFeedforward(/*kS*/ 0.15, /*kG*/ 0.45, /*kV*/ 1.2, /*kA*/ 0.02)
            .withCurrentLimits(/*stator*/ 60, /*supply*/ 30)
            .withVoltageCompensation(Volts.of(12))
            .withTolerance(Degrees.of(1.0))

            // Phoenix Pro (safe — warn-only on non-Phoenix motors)
            .withFOC(true)
            .withMotionMagicExpo(/*kV*/ 0.9, /*kA*/ 0.04)

            // Mechanism geometry
            .withGearRatio(125.0)
            .withLength(Inches.of(24))
            .withMass(Kilograms.of(5.0))
            .withAngleRange(Degrees.of(-15), Degrees.of(110))
            .withStartingAngle(Degrees.of(0))
            .withHorizontalZero(Degrees.of(0))
            .withMomentOfInertia(0.3)

            // AdambotsLib integration
            .withSubsystem(this)
            .withTelemetry("Arm", TelemetryVerbosity.HIGH)
            .withVisualizer(visualizer, /*index*/ 0,
                new Translation3d(0.28, 0.0, 0.42));

        this.arm = new AdambotsArm(motor, config);
    }

    // ----- Commands -----
    public Command stow()  { return arm.moveToAngle(Degrees.of(0)); }
    public Command score() { return arm.moveToAngle(Degrees.of(95)); }
    public Command moveTo(Angle target) { return arm.moveToAngle(target); }

    // ----- Triggers -----
    public Trigger atTarget(Angle target) { return arm.atAngle(target, Degrees.of(2)); }
    public Trigger nearMax() { return arm.atMax(); }

    // ----- Queries -----
    public Angle getAngle() { return arm.getAngle(); }

    // ----- Periodic -----
    @Override
    public void periodic() {
        arm.updateTelemetry();
        arm.visualizationUpdate();
    }

    @Override
    public void simulationPeriodic() {
        arm.simulationPeriodic();
    }
}
```

### Wiring the visualizer

Create one `MechanismVisualizer` in `RobotContainer` and share it across every
mechanism subsystem. Index numbers map to `model_0.glb`, `model_1.glb`, … in
your AdvantageScope 3D layout.

```java
public class RobotContainer {
    private final MechanismVisualizer visualizer = new MechanismVisualizer("MyRobot", 4);
    private final ArmSubsystem arm      = new ArmSubsystem(visualizer);
    // private final ElevatorSubsystem elev = new ElevatorSubsystem(visualizer);
    // ...
}
```

## Configuration reference

Each `Adambots*Config` exposes a fluent builder. Groups of related methods:

- **Closed-loop** — `withPID`, `withPIDSlot`, `withTolerance`, `withFeedforward`
- **Motion profile** — `withMotionMagic` (trapezoid), `withMotionMagicExpo` (Pro)
- **Actuation** — `withCurrentLimits`, `withBrakeMode`, `withVoltageCompensation`
- **Geometry** — `withGearRatio`, plus mechanism-specific
  (`withLength`/`withMass`/`withAngleRange` for arms, `withDrumRadius` for
  elevators, `withWheelDiameter` for flywheels, etc.)
- **Phoenix Pro** — `withFOC`, `withMotionMagicExpo`, `withContinuousWrap`
  (warns on non-Phoenix motors)
- **AdambotsLib** — `withSubsystem`, `withTelemetry`, `withVisualizer`,
  `withSimMotor` (DCMotor override for simulation)

See the Javadocs on each `Adambots*Config` class for the full API.

## Why a wrapper over YAMS?

- **API stability** — YAMS is a young project; the wrapper insulates team code
  from upstream breaking changes.
- **Single motor abstraction** — teams already use `BaseMotor`; we don't want
  two motor types in the same codebase.
- **Safe defaults for our hardware mix** — we run a mix of Phoenix and REV
  motors, so Pro-only features need warn-on-misuse semantics.
- **Visualizer integration** — our `MechanismVisualizer` emits the pose format
  our AdvantageScope layouts expect out of the box.
