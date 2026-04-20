package com.adambots.lib.mechanisms.config;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import com.adambots.lib.actuators.BaseMotor;
import com.adambots.lib.actuators.MotorBridge;

import edu.wpi.first.wpilibj.DriverStation;

import yams.motorcontrollers.SmartMotorController.ClosedLoopControllerSlot;

/**
 * Shared helpers for mechanism configs — Pro feature validation, slot conversion,
 * and common builder operations.
 *
 * <p>Internal — not part of the public API.
 */
public final class MotorConfigHelpers {

    private MotorConfigHelpers() {
        throw new UnsupportedOperationException("Utility class");
    }

    /**
     * Logs a warning if a Pro feature is configured against a non-Phoenix motor.
     *
     * @param motor        the motor the mechanism will use
     * @param mechanismTag identifying tag for the log message (e.g., "AdambotsArm[shoulder]")
     * @param focRequested true if the caller wants FOC
     * @param expoRequested true if the caller wants Motion Magic Expo
     */
    public static void warnProFeaturesOnNonPhoenix(BaseMotor motor, String mechanismTag,
                                                    boolean focRequested, boolean expoRequested) {
        boolean isPhoenix = MotorBridge.isPhoenix(motor);
        if (focRequested && !isPhoenix) {
            DriverStation.reportWarning(
                mechanismTag + ": withFOC(true) requested on non-Phoenix motor (" +
                motor.getMotorType() + "). FOC is ignored.", false);
        }
        if (expoRequested && !isPhoenix) {
            DriverStation.reportWarning(
                mechanismTag + ": withMotionMagicExpo() requested on non-Phoenix motor (" +
                motor.getMotorType() + "). Falling back to trapezoid.", false);
        }
    }

    /** Converts an integer slot (0-2) to YAMS {@link ClosedLoopControllerSlot}. */
    public static ClosedLoopControllerSlot slotOf(int i) {
        return switch (i) {
            case 0 -> ClosedLoopControllerSlot.SLOT_0;
            case 1 -> ClosedLoopControllerSlot.SLOT_1;
            case 2 -> ClosedLoopControllerSlot.SLOT_2;
            default -> throw new IllegalArgumentException("Invalid PID slot (must be 0-2): " + i);
        };
    }

    /** Convenience re-exports so configs don't all need to import static themselves. */
    public static final class Units {
        public static edu.wpi.first.units.measure.Current amps(double a) { return Amps.of(a); }
        public static edu.wpi.first.units.measure.AngularVelocity rps(double v) { return RotationsPerSecond.of(v); }
        public static edu.wpi.first.units.measure.AngularAcceleration rpsps(double a) {
            return RotationsPerSecondPerSecond.of(a);
        }
        private Units() {}
    }
}
