package com.adambots.lib.mechanisms;

import static edu.wpi.first.units.Units.Radians;

import java.util.function.Supplier;

import com.adambots.lib.actuators.BaseMotor;
import com.adambots.lib.actuators.MotorBridge;
import com.adambots.lib.mechanisms.config.AdambotsArmConfig;
import com.adambots.lib.visualization.MechanismVisualizer;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import yams.mechanisms.positional.Arm;
import yams.motorcontrollers.SmartMotorController;

/**
 * AdambotsLib arm mechanism. Thin composition wrapper over YAMS {@code Arm}
 * that exposes only WPILib + AdambotsLib types.
 *
 * <p><strong>Usage:</strong>
 * <pre>{@code
 * BaseMotor motor = new TalonFXMotor(1, true);
 * AdambotsArm arm = new AdambotsArm(motor,
 *     new AdambotsArmConfig()
 *         .withPID(0.5, 0, 0.1)
 *         .withMotionMagic(50, 100)
 *         .withFeedforward(0.1, 0.3, 0.12, 0.01)
 *         .withCurrentLimits(80, 40)
 *         .withGearRatio(50.0)
 *         .withAngleRange(Degrees.of(0), Degrees.of(90))
 *         .withLength(Meters.of(0.5))
 *         .withMass(2.0)
 *         .withFOC(true)                   // Phoenix Pro
 *         .withMotionMagicExpo(0.12, 0.01) // Phoenix Pro
 *         .withSubsystem(this)
 *         .withVisualizer(viz, 0));
 *
 * arm.moveToAngle(Degrees.of(45)).schedule();
 * }</pre>
 */
public final class AdambotsArm {

    private final Arm inner;

    public AdambotsArm(BaseMotor motor, AdambotsArmConfig config) {
        config.validateAgainst(motor);

        SmartMotorController smart = MotorBridge.toSmart(
            motor, config.buildMotorConfig(), config.getSimMotorOverride());

        this.inner = new Arm(config.buildYamsConfig(smart));

        // Auto-wire visualizer if configured
        MechanismVisualizer viz = config.getVisualizer();
        int vizIndex = config.getVisualizerIndex();
        if (viz != null && vizIndex >= 0) {
            final Translation3d pivot = config.getVisualizerPivot();
            viz.register(vizIndex, () -> new Pose3d(
                pivot,
                new Rotation3d(0, inner.getAngle().in(Radians), 0)));
        }
    }

    // ----- Position queries -----
    public Angle getAngle() { return inner.getAngle(); }

    // ----- Commands -----
    /** Command that drives to the target angle and finishes at the tolerance. */
    public Command moveToAngle(Angle angle) { return inner.setAngle(angle); }

    /** Command that drives to the target angle and finishes at the tolerance (supplier variant). */
    public Command moveToAngle(Supplier<Angle> angle) { return inner.setAngle(angle); }

    /** Command that holds the target angle indefinitely (never finishes). */
    public Command holdAngle(Angle angle) { return inner.run(angle); }
    public Command holdAngle(Supplier<Angle> angle) { return inner.run(angle); }

    /** Command that drives to the target angle and finishes within a custom tolerance. */
    public Command runTo(Angle angle, Angle tolerance) { return inner.runTo(angle, tolerance); }

    // ----- Triggers -----
    public Trigger atAngle(Angle target, Angle tolerance) { return inner.isNear(target, tolerance); }
    public Trigger atMax() { return inner.max(); }
    public Trigger atMin() { return inner.min(); }
    public Trigger above(Angle angle) { return inner.gte(angle); }
    public Trigger below(Angle angle) { return inner.lte(angle); }
    public Trigger between(Angle start, Angle end) { return inner.between(start, end); }

    // ----- SysId -----
    public Command sysId(Voltage maxVoltage, Velocity<edu.wpi.first.units.VoltageUnit> step, Time duration) {
        return inner.sysId(maxVoltage, step, duration);
    }

    // ----- Periodic hooks (call from subsystem) -----
    public void updateTelemetry() { inner.updateTelemetry(); }
    public void simulationPeriodic() { inner.simIterate(); }
    public void visualizationUpdate() { inner.visualizationUpdate(); }
}
