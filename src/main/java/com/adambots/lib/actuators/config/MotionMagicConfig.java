package com.adambots.lib.actuators.config;

/**
 * Configuration record for Motion Magic motion profiling.
 *
 * <p>Motion Magic generates smooth trapezoidal motion profiles that respect maximum
 * velocity and acceleration constraints, preventing jerky motion and reducing mechanical
 * stress.
 *
 * <p><strong>Parameter Guidance:</strong>
 * <ul>
 *   <li><strong>cruiseVelocityRPS:</strong> Set to 70-80% of mechanism's maximum speed</li>
 *   <li><strong>accelerationRPSPerSec:</strong> Typically 2-4x cruise velocity</li>
 *   <li><strong>jerkRPSPerSecPerSec:</strong> Optional smoothing (0 = disabled, or 5-10x acceleration)</li>
 * </ul>
 *
 * <p><strong>Usage Example:</strong>
 * <pre>{@code
 * // Elevator motion profile
 * MotionMagicConfig elevatorProfile = new MotionMagicConfig(
 *     50.0,    // 50 RPS cruise velocity
 *     150.0,   // 150 RPS² acceleration
 *     500.0    // 500 RPS³ jerk limiting
 * );
 *
 * // Arm motion profile (slower, smoother)
 * MotionMagicConfig armProfile = new MotionMagicConfig(30.0, 90.0, 300.0);
 * }</pre>
 *
 * @param cruiseVelocityRPS Maximum velocity during motion in rotations per second
 * @param accelerationRPSPerSec Acceleration rate in rotations per second²
 * @param jerkRPSPerSecPerSec Jerk (rate of acceleration change) in rotations per second³ (0 to disable)
 */
public record MotionMagicConfig(
    double cruiseVelocityRPS,
    double accelerationRPSPerSec,
    double jerkRPSPerSecPerSec
) {}
