package com.adambots.lib.actuators.config;

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
 * CurrentLimitConfig driveLimits = new CurrentLimitConfig(40, 60, 5000);
 *
 * // Allow higher current for shooter motor
 * CurrentLimitConfig shooterLimits = new CurrentLimitConfig(60, 120, 3000);
 * }</pre>
 *
 * @param stallLimitAmps Current limit when motor is under heavy load (amperes)
 * @param freeLimitAmps Current limit when motor is spinning freely (amperes)
 * @param limitRpmThreshold RPM threshold below which stall limit applies
 */
public record CurrentLimitConfig(
    int stallLimitAmps,
    int freeLimitAmps,
    int limitRpmThreshold
) {}
