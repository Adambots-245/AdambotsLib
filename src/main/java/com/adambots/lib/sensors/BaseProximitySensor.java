package com.adambots.lib.sensors;

import edu.wpi.first.epilogue.Logged;

/**
 * Base interface for proximity sensors that detect object presence.
 *
 * <p>Proximity sensors provide binary (triggered/not triggered) feedback
 * when objects enter their detection zone. Common implementations include
 * limit switches, beam-break sensors (photo-eyes), and inductive sensors.
 *
 * <p><strong>Common Implementations:</strong>
 * <ul>
 *   <li>{@link LimitSwitch} - Mechanical contact switch</li>
 *   <li>{@link PhotoEye} - Optical beam-break sensor</li>
 * </ul>
 *
 * <p><strong>Usage Example:</strong>
 * <pre>{@code
 * BaseProximitySensor gamePieceDetector = new PhotoEye(3, false);
 *
 * // Check if object is present
 * if (gamePieceDetector.isDetecting()) {
 *     intake.stop();
 *     System.out.println("Game piece acquired!");
 * }
 * }</pre>
 *
 * @see LimitSwitch
 * @see PhotoEye
 */
@Logged
public interface BaseProximitySensor {

    /**
     * Returns whether the sensor is currently detecting an object.
     *
     * <p>Implementations should account for inverted configurations automatically,
     * so this method always returns true when an object is present, regardless
     * of the sensor's wiring.
     *
     * @return True if an object is detected, false otherwise
     */
    boolean isDetecting();

}