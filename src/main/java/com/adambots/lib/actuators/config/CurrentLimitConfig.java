package com.adambots.lib.actuators.config;

import static edu.wpi.first.units.Units.*;
import edu.wpi.first.units.measure.*;

/**
 * Configuration record for motor current limiting.
 *
 * <p>Current limits protect the motor and battery from excessive current draw.
 * Modern motor controllers use "smart" current limiting with separate limits for
 * stalled and free-spinning conditions.
 *
 * <p><strong>Typical Values:</strong>
 * <ul>
 *   <li><strong>Drive motors:</strong> stall=40-60A, free=80-120A</li>
 *   <li><strong>Mechanism motors:</strong> stall=20-40A, free=40-80A</li>
 *   <li><strong>Small actuators:</strong> stall=10-20A, free=20-40A</li>
 * </ul>
 *
 * <p><strong>Usage Example:</strong>
 * <pre>{@code
 * // Conservative limits for NEO on drive motor
 * CurrentLimitConfig driveLimits = new CurrentLimitConfig(
 *     Amps.of(40), Amps.of(60), 5000);
 *
 * // Allow higher current for shooter motor
 * CurrentLimitConfig shooterLimits = new CurrentLimitConfig(
 *     Amps.of(60), Amps.of(120), 3000);
 * }</pre>
 *
 * @param stallLimit Current limit when motor is under heavy load
 * @param freeLimit Current limit when motor is spinning freely
 * @param limitRpmThreshold RPM threshold below which stall limit applies
 */
public record CurrentLimitConfig(
    Current stallLimit,
    Current freeLimit,
    int limitRpmThreshold
) {
    /**
     * Creates a CurrentLimitConfig using raw ampere values for backwards compatibility.
     *
     * @param stallLimitAmps Current limit when motor is under heavy load (amperes)
     * @param freeLimitAmps Current limit when motor is spinning freely (amperes)
     * @param limitRpmThreshold RPM threshold below which stall limit applies
     * @return A new CurrentLimitConfig instance
     */
    public static CurrentLimitConfig fromAmps(double stallLimitAmps, double freeLimitAmps, int limitRpmThreshold) {
        return new CurrentLimitConfig(
            Amps.of(stallLimitAmps),
            Amps.of(freeLimitAmps),
            limitRpmThreshold
        );
    }

    /**
     * Gets the stall limit in amperes for motor controller APIs.
     * @return Stall limit in amperes
     */
    public double stallLimitAmps() {
        return stallLimit.in(Amps);
    }

    /**
     * Gets the free limit in amperes for motor controller APIs.
     * @return Free limit in amperes
     */
    public double freeLimitAmps() {
        return freeLimit.in(Amps);
    }
}
