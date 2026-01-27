package com.adambots.lib.actuators.config;

import static edu.wpi.first.units.Units.*;
import edu.wpi.first.units.measure.*;

/**
 * Configuration record for Motion Magic motion profiling.
 *
 * <p>Motion Magic generates smooth trapezoidal motion profiles that respect maximum
 * velocity and acceleration constraints, preventing jerky motion and reducing mechanical
 * stress.
 *
 * <p><strong>Parameter Guidance:</strong>
 * <ul>
 *   <li><strong>cruiseVelocity:</strong> Set to 70-80% of mechanism's maximum speed</li>
 *   <li><strong>acceleration:</strong> Typically 2-4x cruise velocity</li>
 *   <li><strong>jerkRPSPerSecPerSec:</strong> Optional smoothing (0 = disabled, or 5-10x acceleration)</li>
 * </ul>
 *
 * <p><strong>Usage Example:</strong>
 * <pre>{@code
 * // Elevator motion profile
 * MotionMagicConfig elevatorProfile = new MotionMagicConfig(
 *     RotationsPerSecond.of(50),    // 50 RPS cruise velocity
 *     RotationsPerSecondPerSecond.of(150),   // 150 RPS² acceleration
 *     500.0    // 500 RPS³ jerk limiting
 * );
 *
 * // Arm motion profile (slower, smoother)
 * MotionMagicConfig armProfile = new MotionMagicConfig(
 *     RotationsPerSecond.of(30),
 *     RotationsPerSecondPerSecond.of(90),
 *     300.0
 * );
 * }</pre>
 *
 * @param cruiseVelocity Maximum velocity during motion
 * @param acceleration Acceleration rate
 * @param jerkRPSPerSecPerSec Jerk (rate of acceleration change) in rotations per second³ (0 to disable)
 */
public record MotionMagicConfig(
    AngularVelocity cruiseVelocity,
    AngularAcceleration acceleration,
    double jerkRPSPerSecPerSec
) {
    /**
     * Creates a MotionMagicConfig using raw RPS values for backwards compatibility.
     *
     * @param cruiseVelocityRPS Maximum velocity during motion in rotations per second
     * @param accelerationRPSPerSec Acceleration rate in rotations per second²
     * @param jerkRPSPerSecPerSec Jerk (rate of acceleration change) in rotations per second³ (0 to disable)
     * @return A new MotionMagicConfig instance
     */
    public static MotionMagicConfig fromRPS(double cruiseVelocityRPS, double accelerationRPSPerSec, double jerkRPSPerSecPerSec) {
        return new MotionMagicConfig(
            RotationsPerSecond.of(cruiseVelocityRPS),
            RotationsPerSecondPerSecond.of(accelerationRPSPerSec),
            jerkRPSPerSecPerSec
        );
    }

    /**
     * Gets the cruise velocity in rotations per second for motor controller APIs.
     * @return Cruise velocity in RPS
     */
    public double cruiseVelocityRPS() {
        return cruiseVelocity.in(RotationsPerSecond);
    }

    /**
     * Gets the acceleration in rotations per second² for motor controller APIs.
     * @return Acceleration in RPS²
     */
    public double accelerationRPSPerSec() {
        return acceleration.in(RotationsPerSecondPerSecond);
    }
}
