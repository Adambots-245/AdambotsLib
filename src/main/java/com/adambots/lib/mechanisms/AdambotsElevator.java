package com.adambots.lib.mechanisms;

import static edu.wpi.first.units.Units.Meters;

import java.util.function.Supplier;

import com.adambots.lib.actuators.BaseMotor;
import com.adambots.lib.actuators.MotorBridge;
import com.adambots.lib.mechanisms.config.AdambotsElevatorConfig;
import com.adambots.lib.visualization.MechanismVisualizer;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import yams.mechanisms.positional.Elevator;
import yams.motorcontrollers.SmartMotorController;

/**
 * AdambotsLib elevator mechanism. Composition wrapper over YAMS {@code Elevator}.
 *
 * <p><strong>Usage:</strong>
 * <pre>{@code
 * AdambotsElevator elevator = new AdambotsElevator(motor,
 *     new AdambotsElevatorConfig()
 *         .withPID(1.0, 0, 0.1)
 *         .withMotionMagic(100, 200)
 *         .withFeedforward(0.3, 0.5, 0.12, 0.0)
 *         .withCurrentLimits(80, 40)
 *         .withGearRatio(12.0)
 *         .withDrumRadius(Inches.of(1.0))
 *         .withMass(5.0)
 *         .withHeightRange(Meters.of(0), Meters.of(1.5))
 *         .withSubsystem(this));
 *
 * elevator.moveToHeight(Inches.of(36)).schedule();
 * }</pre>
 */
public final class AdambotsElevator {

    private final Elevator inner;

    public AdambotsElevator(BaseMotor motor, AdambotsElevatorConfig config) {
        config.validateAgainst(motor);

        SmartMotorController smart = MotorBridge.toSmart(
            motor, config.buildMotorConfig(), config.getSimMotorOverride());

        this.inner = new Elevator(config.buildYamsConfig(smart));

        MechanismVisualizer viz = config.getVisualizer();
        int vizIndex = config.getVisualizerIndex();
        if (viz != null && vizIndex >= 0) {
            final Translation3d base = config.getVisualizerBase();
            viz.register(vizIndex, () -> new Pose3d(
                new Translation3d(
                    base.getX(),
                    base.getY(),
                    base.getZ() + inner.getHeight().in(Meters)),
                new Rotation3d()));
        }
    }

    // ----- Queries -----
    public Distance getHeight() { return inner.getHeight(); }
    public LinearVelocity getVelocity() { return inner.getVelocity(); }

    // ----- Commands -----
    public Command moveToHeight(Distance height) { return inner.setHeight(height); }
    public Command moveToHeight(Supplier<Distance> height) { return inner.setHeight(height); }
    public Command holdHeight(Distance height) { return inner.run(height); }
    public Command holdHeight(Supplier<Distance> height) { return inner.run(height); }
    public Command runTo(Distance height, Distance tolerance) { return inner.runTo(height, tolerance); }

    // ----- Triggers -----
    public Trigger atHeight(Distance target, Distance tolerance) { return inner.isNear(target, tolerance); }
    public Trigger atMax() { return inner.max(); }
    public Trigger atMin() { return inner.min(); }
    public Trigger above(Distance height) { return inner.gte(height); }
    public Trigger below(Distance height) { return inner.lte(height); }
    public Trigger between(Distance start, Distance end) { return inner.between(start, end); }

    // ----- Periodic hooks -----
    public void updateTelemetry() { inner.updateTelemetry(); }
    public void simulationPeriodic() { inner.simIterate(); }
    public void visualizationUpdate() { inner.visualizationUpdate(); }
}
