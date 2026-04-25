package com.adambots.lib.mechanisms;

import static edu.wpi.first.units.Units.Radians;

import java.util.function.Supplier;

import com.adambots.lib.actuators.BaseMotor;
import com.adambots.lib.actuators.MotorBridge;
import com.adambots.lib.mechanisms.config.AdambotsPivotConfig;
import com.adambots.lib.visualization.MechanismVisualizer;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import yams.mechanisms.positional.Pivot;
import yams.motorcontrollers.SmartMotorController;

/**
 * AdambotsLib pivot mechanism. Composition wrapper over YAMS {@code Pivot}.
 *
 * <p>Pivot differs from {@link AdambotsArm}: no arm length or mass — used for
 * turrets, wrist pivots, and rotational axes where you care about angle but not
 * cantilevered gravitational torque. For an arm with gravity compensation, use
 * {@link AdambotsArm}.
 *
 * <p><strong>Usage (turret):</strong>
 * <pre>{@code
 * AdambotsPivot turret = new AdambotsPivot(motor,
 *     new AdambotsPivotConfig()
 *         .withPID(1.5, 0, 0.1)
 *         .withMotionMagic(30, 60)
 *         .withCurrentLimits(40, 20)
 *         .withGearRatio(100.0)
 *         .withContinuousWrap(Degrees.of(-180), Degrees.of(180))
 *         .withSubsystem(this));
 *
 * turret.moveToAngle(Degrees.of(135)).schedule();
 * }</pre>
 */
public final class AdambotsPivot {

    private final Pivot inner;

    public AdambotsPivot(BaseMotor motor, AdambotsPivotConfig config) {
        config.validateAgainst(motor);

        SmartMotorController smart = MotorBridge.toSmart(
            motor, config.buildMotorConfig(), config.getSimMotorOverride(), config.isFocEnabled());

        this.inner = new Pivot(config.buildYamsConfig(smart));

        MechanismVisualizer viz = config.getVisualizer();
        int vizIndex = config.getVisualizerIndex();
        if (viz != null && vizIndex >= 0) {
            final Translation3d pivot = config.getVisualizerPivot();
            // Pivot rotates around Z by default (turret yaw). Teams can override via viz.register()
            // to use a different axis if needed.
            viz.register(vizIndex, () -> new Pose3d(
                pivot,
                new Rotation3d(0, 0, inner.getAngle().in(Radians))));
        }
    }

    // ----- Queries -----
    public Angle getAngle() { return inner.getAngle(); }

    // ----- Commands -----
    public Command moveToAngle(Angle angle) { return inner.setAngle(angle); }
    public Command moveToAngle(Supplier<Angle> angle) { return inner.setAngle(angle); }
    public Command holdAngle(Angle angle) { return inner.run(angle); }
    public Command holdAngle(Supplier<Angle> angle) { return inner.run(angle); }
    public Command runTo(Angle angle, Angle tolerance) { return inner.runTo(angle, tolerance); }

    // ----- Triggers -----
    public Trigger atAngle(Angle target, Angle tolerance) { return inner.isNear(target, tolerance); }
    public Trigger atMax() { return inner.max(); }
    public Trigger atMin() { return inner.min(); }
    public Trigger above(Angle angle) { return inner.gte(angle); }
    public Trigger below(Angle angle) { return inner.lte(angle); }
    public Trigger between(Angle start, Angle end) { return inner.between(start, end); }

    // ----- Periodic hooks -----
    public void updateTelemetry() { inner.updateTelemetry(); }
    public void simulationPeriodic() { inner.simIterate(); }
    public void visualizationUpdate() { inner.visualizationUpdate(); }
}
