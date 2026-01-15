package com.adambots.lib.sensors;

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
 * Distance can be reported in centimeters, inches, or feet. Implementations
 * must provide all three measurement units.
 *
 * <p><strong>Usage Example:</strong>
 * <pre>{@code
 * BaseDistanceSensor rangefinder = new UltrasonicSensor(0);
 *
 * // Get distance in centimeters
 * double distanceCM = rangefinder.getDistanceInCentimeters();
 *
 * // Check if object is within range
 * if (distanceCM < 30.0) {
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
public interface BaseDistanceSensor {

    /**
     * Take a measurement and return the distance in cm
     * 
     * @return Distance in cm
     */
    double getDistanceInCentimeters();

    /**
     * Take a measurement and return the distance in inches
     * 
     * @return Distance in inches
     */
    double getDistanceInInches();

    /**
     * Take a measurement and return the distance in feet
     *
     * @return Distance in feet
     */
    double getDistanceInFeet();

}