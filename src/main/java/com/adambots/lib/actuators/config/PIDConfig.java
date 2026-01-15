package com.adambots.lib.actuators.config;

/**
 * Configuration record for PID gains.
 *
 * <p>This immutable record stores PID gain values for closed-loop motor control.
 * Most motor controllers support multiple PID slots (0-2 or 0-3) for different
 * control scenarios.
 *
 * <p><strong>Usage Example:</strong>
 * <pre>{@code
 * // Position control gains
 * PIDConfig positionPID = new PIDConfig(0, 0.1, 0.0, 0.05, 0.0);
 *
 * // Velocity control gains with feed-forward
 * PIDConfig velocityPID = new PIDConfig(1, 0.0001, 0, 0.00005, 0.000176);
 * }</pre>
 *
 * @param slot PID gain slot index (typically 0-2)
 * @param kP Proportional gain coefficient
 * @param kI Integral gain coefficient
 * @param kD Derivative gain coefficient
 * @param kF Feed-forward gain coefficient
 */
public record PIDConfig(
    int slot,
    double kP,
    double kI,
    double kD,
    double kF
) {}
