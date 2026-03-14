package com.adambots.lib.sensors;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;

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
 *   <li>{@link Potentiometer} - Analog potentiometer (12-bit, analog input)</li>
 * </ul>
 *
 * <p><strong>Position Units:</strong>
 * Position is returned as a WPILib {@link Angle} type. Use the {@code .in()} method
 * to convert to degrees or radians.
 *
 * <p><strong>Usage Example:</strong>
 * <pre>{@code
 * import static edu.wpi.first.units.Units.*;
 *
 * BaseAbsoluteEncoder encoder = new CANCoder(5);
 *
 * // Get current position (various units)
 * Angle position = encoder.getPosition();
 * double degrees = position.in(Degrees);
 * double radians = position.in(Radians);
 *
 * // Get as Rotation2d for kinematics
 * Rotation2d rotation = encoder.getPositionRotation2d();
 * }</pre>
 *
 * @see BaseGyro
 */
@Logged
public interface BaseAbsoluteEncoder {

    /**
     * Returns the absolute position of the encoder.
     *
     * <p>Position values wrap at 360° (one full rotation).
     * Use {@code .in(Degrees)} or {@code .in(Radians)} to convert:
     * <pre>{@code
     * Angle pos = encoder.getPosition();
     * double degrees = pos.in(Degrees);  // 0.0 to 360.0
     * double radians = pos.in(Radians);  // 0.0 to 2π
     * }</pre>
     *
     * @return Absolute position (wraps at 360°)
     */
    Angle getPosition();

    /**
     * Returns the absolute position as a Rotation2d.
     *
     * <p>Useful for WPILib kinematics and geometry classes.
     *
     * @return Absolute position as Rotation2d
     */
    Rotation2d getPositionRotation2d();

}
