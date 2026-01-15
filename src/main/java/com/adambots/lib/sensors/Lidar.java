/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.adambots.lib.sensors;

import com.adambots.lib.utils.Utils;

import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.DigitalInput;

/**
 * LIDAR distance sensor wrapper for FRC robotics.
 *
 * <p>This class provides a simple interface to LIDAR-Lite or similar pulse-based
 * distance sensors. The sensor outputs a PWM signal where the pulse width represents
 * the measured distance (10 microseconds per centimeter).
 *
 * <p><strong>Typical Range:</strong> 0-500 cm (depends on sensor model and conditions)
 *
 * <p><strong>Usage Example:</strong>
 * <pre>{@code
 * Lidar sensor = new Lidar(0);  // DIO port 0
 * double distanceCm = sensor.getDistanceInCentimeters();
 * double distanceIn = sensor.getDistanceInInches();
 * }</pre>
 *
 * @see BaseDistanceSensor
 * @see edu.wpi.first.wpilibj.Counter
 */
public class Lidar implements BaseDistanceSensor {
    private Counter counter;
    private DigitalInput source;

    /**
     * Creates a new LIDAR sensor on the specified digital input port.
     *
     * <p>The sensor is configured to measure pulse width in semi-period mode,
     * which is appropriate for PWM-based distance sensors like LIDAR-Lite.
     *
     * @param dioPortNumber Digital I/O port number (0-9 on roboRIO)
     */
	public Lidar(int dioPortNumber) {
        // Validate DIO port range (RoboRIO 2 has 10 DIO ports: 0-9)
        if (dioPortNumber < 0 || dioPortNumber > 9) {
            Utils.reportError("Lidar: Invalid DIO port " + dioPortNumber +
                ". RoboRIO 2 has 10 DIO ports (0-9). Defaulting to 0.");
            dioPortNumber = 0;
        }

        source = new DigitalInput(dioPortNumber);

		counter = new Counter(source);

	    counter.setMaxPeriod(1.0);
	    // Configure for measuring rising to falling pulses
	    counter.setSemiPeriodMode(true);
	    counter.reset();
    }
    
	/**
	 * Gets the current distance measurement in centimeters.
	 *
	 * <p>The measurement is calculated from the PWM pulse width, where the sensor
	 * outputs 10 microseconds per centimeter of distance. Returns 0 if no valid
	 * reading is available.
	 *
	 * <p><strong>Technical Details:</strong> Converts the counter period (seconds)
	 * to microseconds, then divides by 10 to get centimeters.
	 *
	 * @return Distance in centimeters (typically 0-500 cm range)
	 */
	@Override
	public double getDistanceInCentimeters() {
		double cm;

		// getPeriod returns time in seconds. The hardware resolution is microseconds.
		// The LIDAR-Lite unit sends a high signal for 10 microseconds per cm of distance.
		cm = (counter.getPeriod() * 1000000.0 / 10.0);
		return cm;
	}

	/**
	 * Gets the current distance measurement in inches.
	 *
	 * <p>Convenience method that converts the centimeter measurement to inches.
     *
	 * @return Distance in inches (centimeters / 2.54)
	 */
	@Override
	public double getDistanceInInches() {
		return getDistanceInCentimeters() / 2.54;
    }

	/**
	 * Gets the current distance measurement in feet.
	 *
	 * <p>Convenience method that converts the inch measurement to feet.
	 *
	 * @return Distance in feet (inches / 12.0)
	 */
	@Override
	public double getDistanceInFeet() {
		return getDistanceInInches() / 12.0;
	}

	
}
