package com.adambots.lib.actuators;

import edu.wpi.first.math.system.plant.DCMotor;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.local.SparkWrapper;
import yams.motorcontrollers.remote.TalonFXSWrapper;
import yams.motorcontrollers.remote.TalonFXWrapper;

/**
 * Internal adapter that bridges AdambotsLib's {@link BaseMotor} into a YAMS
 * {@link SmartMotorController}. Used by mechanism wrappers (AdambotsArm,
 * AdambotsElevator, AdambotsFlyWheel, AdambotsPivot) to obtain the
 * vendor-specific YAMS wrapper for a given {@code BaseMotor}.
 *
 * <p><strong>This class is an implementation detail.</strong> It is public so
 * mechanism wrappers in sibling packages can call it, but it is not part of the
 * AdambotsLib public API. Robot code should never reference it directly.
 *
 * <p>Uses pattern matching on the sealed {@code BaseMotor} hierarchy so the
 * compiler enforces that every supported motor type has a translation.
 */
public final class MotorBridge {

    private MotorBridge() {
        throw new UnsupportedOperationException("Utility class — not instantiable");
    }

    /**
     * Wraps a BaseMotor into a YAMS SmartMotorController using the provided config.
     *
     * <p>If {@code dcMotorOverride} is non-null, that DCMotor model is used instead
     * of the default inferred from the motor type. This lets teams override the
     * sim physics model (e.g., use Minion-specific params when available).
     *
     * @param motor           the BaseMotor to bridge
     * @param smartConfig     the YAMS SmartMotorControllerConfig to apply
     * @param dcMotorOverride optional DCMotor model override; null uses the default
     * @return a configured YAMS SmartMotorController
     * @throws IllegalArgumentException if the motor type cannot be bridged (DummyMotor)
     */
    public static SmartMotorController toSmart(
            BaseMotor motor,
            SmartMotorControllerConfig smartConfig,
            DCMotor dcMotorOverride) {

        if (motor instanceof TalonFXMotor t) {
            return new TalonFXWrapper(
                t.rawMotor(),
                dcMotorOverride != null ? dcMotorOverride : t.dcMotorModel(),
                smartConfig);
        }
        if (motor instanceof MinionMotor m) {
            return new TalonFXSWrapper(
                m.rawMotor(),
                dcMotorOverride != null ? dcMotorOverride : m.dcMotorModel(),
                smartConfig);
        }
        if (motor instanceof NEOMotor n) {
            return new SparkWrapper(
                n.rawMotor(),
                dcMotorOverride != null ? dcMotorOverride : n.dcMotorModel(),
                smartConfig);
        }
        if (motor instanceof DummyMotor) {
            throw new IllegalArgumentException(
                "DummyMotor cannot be used inside a mechanism. Use a real motor " +
                "or avoid constructing the mechanism when hardware is not present.");
        }
        // Sealed interface guarantees this is unreachable, but defensive for future subtypes.
        throw new IllegalArgumentException("Unsupported BaseMotor type: " + motor.getClass().getName());
    }

    /**
     * Wraps a BaseMotor into a YAMS SmartMotorController with the motor's
     * default DCMotor model.
     */
    public static SmartMotorController toSmart(
            BaseMotor motor,
            SmartMotorControllerConfig smartConfig) {
        return toSmart(motor, smartConfig, null);
    }

    /**
     * Returns true if the underlying motor is a Phoenix (CTRE) motor that
     * supports Pro features. Used by mechanism configs to decide whether
     * Pro-only options are applicable or should log a warning.
     */
    public static boolean isPhoenix(BaseMotor motor) {
        return motor instanceof TalonFXMotor || motor instanceof MinionMotor;
    }

    /**
     * Returns true if the underlying motor is a Kraken X60 TalonFX (the only
     * motor that fully supports FOC TorqueCurrent control modes).
     */
    public static boolean isKrakenX60(BaseMotor motor) {
        return motor instanceof TalonFXMotor t && t.isKrakenX60();
    }
}
