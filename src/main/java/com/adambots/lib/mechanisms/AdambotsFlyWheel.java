package com.adambots.lib.mechanisms;

import java.util.function.Supplier;

import com.adambots.lib.actuators.BaseMotor;
import com.adambots.lib.actuators.MotorBridge;
import com.adambots.lib.mechanisms.config.AdambotsFlyWheelConfig;
import com.adambots.lib.visualization.MechanismVisualizer;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;

/**
 * AdambotsLib flywheel mechanism. Composition wrapper over YAMS {@code FlyWheel}.
 *
 * <p><strong>Usage:</strong>
 * <pre>{@code
 * AdambotsFlyWheel shooter = new AdambotsFlyWheel(motor,
 *     new AdambotsFlyWheelConfig()
 *         .withPID(0.001, 0, 0)
 *         .withFeedforward(0.1, 0.01, 0.001)
 *         .withCurrentLimits(80, 40)
 *         .withGearRatio(2.0)
 *         .withWheelDiameter(Inches.of(4.0))
 *         .withMomentOfInertia(0.001)
 *         .withFOC(true)
 *         .withSubsystem(this));
 *
 * shooter.spinUpTo(RotationsPerSecond.of(80)).schedule();
 * }</pre>
 */
public final class AdambotsFlyWheel {

    private final FlyWheel inner;

    public AdambotsFlyWheel(BaseMotor motor, AdambotsFlyWheelConfig config) {
        config.validateAgainst(motor);

        SmartMotorController smart = MotorBridge.toSmart(
            motor, config.buildMotorConfig(), config.getSimMotorOverride(), config.isFocEnabled());

        this.inner = new FlyWheel(config.buildYamsConfig(smart));

        MechanismVisualizer viz = config.getVisualizer();
        int vizIndex = config.getVisualizerIndex();
        if (viz != null && vizIndex >= 0) {
            final Translation3d offset = config.getVisualizerOffset();
            // For visualization, represent the flywheel by static position;
            // teams can override via generic visualizer.register() if they need spin animation.
            viz.register(vizIndex, () -> new Pose3d(offset, new Rotation3d()));
        }
    }

    // ----- Queries -----
    public AngularVelocity getSpeed() { return inner.getSpeed(); }

    /** Linear (surface) velocity derived from angular velocity × wheel radius. */
    public LinearVelocity getLinearVelocity() { return inner.getLinearVelocity(); }

    // ----- Commands -----
    /** Command that spins up to target velocity and finishes within tolerance. */
    public Command spinUpTo(AngularVelocity velocity) { return inner.setSpeed(velocity); }
    public Command spinUpTo(Supplier<AngularVelocity> velocity) { return inner.setSpeed(velocity); }

    /** Command that runs at target velocity indefinitely (never finishes). */
    public Command runAt(AngularVelocity velocity) { return inner.run(velocity); }
    public Command runAtLinear(LinearVelocity velocity) { return inner.run(velocity); }

    /** Command that drives to target velocity with a custom tolerance. */
    public Command runTo(AngularVelocity velocity, AngularVelocity tolerance) {
        return inner.runTo(velocity, tolerance);
    }

    public Command runTo(LinearVelocity velocity, LinearVelocity tolerance) {
        return inner.runTo(velocity, tolerance);
    }

    // ----- Triggers -----
    public Trigger atSpeed(AngularVelocity target, AngularVelocity tolerance) {
        return inner.isNear(target, tolerance);
    }
    public Trigger atMax() { return inner.max(); }
    public Trigger atMin() { return inner.min(); }
    public Trigger above(AngularVelocity speed) { return inner.gte(speed); }
    public Trigger below(AngularVelocity speed) { return inner.lte(speed); }
    public Trigger between(AngularVelocity start, AngularVelocity end) {
        return inner.between(start, end);
    }

    // ----- Periodic hooks -----
    public void updateTelemetry() { inner.updateTelemetry(); }
    public void simulationPeriodic() { inner.simIterate(); }
    public void visualizationUpdate() { inner.visualizationUpdate(); }
}
