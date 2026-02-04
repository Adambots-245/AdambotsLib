package com.adambots.lib.sensors;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Distance;

/**
 * Base interface for distance measurement sensors.
 *
 * <p>Distance sensors measure the range to objects using various technologies
 * including ultrasonic, LIDAR, and time-of-flight. Each implementation has
 * different range capabilities, accuracy, and update rates.
 *
 * <p><strong>Common Implementations:</strong>
 * <ul>
 *   <li>{@link UltrasonicSensor} - Analog ultrasonic (10cm - 5m typical)</li>
 *   <li>{@link Lidar} - LIDAR-Lite PWM (5cm - 40m)</li>
 *   <li>{@link CANRangeSensor} - CTRE CANrange (CAN-based)</li>
 * </ul>
 *
 * <p><strong>Distance Units:</strong>
 * Distance is returned as a WPILib {@link Distance} type. Use the {@code .in()} method
 * to convert to your desired unit (Centimeters, Inches, Meters, etc.).
 *
 * <p><strong>Usage Example:</strong>
 * <pre>{@code
 * import static edu.wpi.first.units.Units.*;
 *
 * BaseDistanceSensor rangefinder = new UltrasonicSensor(0);
 *
 * // Get distance in various units
 * Distance distance = rangefinder.getDistance();
 * double cm = distance.in(Centimeters);
 * double inches = distance.in(Inches);
 * double meters = distance.in(Meters);
 *
 * // Check if object is within range
 * if (distance.in(Centimeters) < 30.0) {
 *     System.out.println("Object detected within 30cm");
 * }
 * }</pre>
 *
 * <p><strong>Accuracy Considerations:</strong>
 * <ul>
 *   <li>Ultrasonic sensors: ±1-3cm accuracy, affected by surface texture</li>
 *   <li>LIDAR: ±2cm accuracy, more consistent across surfaces</li>
 *   <li>All sensors: may return invalid readings for very close or distant objects</li>
 * </ul>
 */
@Logged
public interface BaseDistanceSensor {

    /**
     * Take a measurement and return the distance.
     *
     * <p>Returns a WPILib {@link Distance} type that can be converted to any unit:
     * <pre>{@code
     * Distance d = sensor.getDistance();
     * double cm = d.in(Centimeters);
     * double inches = d.in(Inches);
     * double meters = d.in(Meters);
     * }</pre>
     *
     * @return Distance measurement
     */
    Distance getDistance();

}
