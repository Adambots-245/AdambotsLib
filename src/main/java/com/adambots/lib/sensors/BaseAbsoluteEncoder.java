package com.adambots.lib.sensors;

import edu.wpi.first.math.geometry.Rotation2d;

/**
 * Base interface for absolute position encoders.
 *
 * <p>Absolute encoders maintain position information through power cycles,
 * unlike relative encoders which reset to zero. This makes them ideal for
 * mechanisms that need to know their position immediately on robot startup
 * without homing sequences.
 *
 * <p><strong>Common Implementations:</strong>
 * <ul>
 *   <li>{@link ThroughBoreEncoder} - REV Through Bore (8192 CPR, DutyCycle)</li>
 *   <li>{@link CANCoder} - CTRE CANcoder (4096 CPR, CAN)</li>
 * </ul>
 *
 * <p><strong>Position Units:</strong>
 * Position is reported as degrees (0.0 to 360.0) or radians (0.0 to 2π).
 * Values wrap at the boundaries for absolute position tracking.
 *
 * <p><strong>Usage Example:</strong>
 * <pre>{@code
 * BaseAbsoluteEncoder encoder = new ThroughBoreEncoder(5);
 *
 * // Get current position (degrees)
 * double position = encoder.getAbsolutePositionDegrees();
 *
 * // Get position in radians
 * double radians = encoder.getAbsolutePositionRadians();
 *
 * // Get as Rotation2d for kinematics
 * Rotation2d rotation = encoder.getAbsolutePositionRotation2D();
 * }</pre>
 *
 * @see BaseGyro
 */
public interface BaseAbsoluteEncoder {

    /**
     * Returns the discrete (does not continue past 360) value of the encoder in degrees
     * @return Discrete value of encoder in degrees
     */
    double getAbsolutePositionDegrees();

    /**
     * Returns the discrete (does not continue past 2pi) value of the encoder in radians
     * @return Discrete value of encoder in radians
     */
    double getAbsolutePositionRadians();

    /**
     * Returns the discrete (does not continue past 2pi) value of the encoder in radians
     * @return Discrete value of encoder in radians
     */
    Rotation2d getAbsolutePositionRotation2D();

}