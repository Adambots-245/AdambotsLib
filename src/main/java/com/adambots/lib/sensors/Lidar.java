/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.adambots.lib.sensors;

import com.adambots.lib.utils.Utils;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.units.measure.Distance;
import static edu.wpi.first.units.Units.*;

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
 * import static edu.wpi.first.units.Units.*;
 *
 * Lidar sensor = new Lidar(0);  // DIO port 0
 * Distance distance = sensor.getDistance();
 * double cm = distance.in(Centimeters);
 * double inches = distance.in(Inches);
 * }</pre>
 *
 * @see BaseDistanceSensor
 * @see edu.wpi.first.wpilibj.Counter
 */
@Logged
public class Lidar implements BaseDistanceSensor {
    @NotLogged
    private Counter counter;

    @NotLogged
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
	 * Gets the current distance measurement.
	 *
	 * <p>The measurement is calculated from the PWM pulse width, where the sensor
	 * outputs 10 microseconds per centimeter of distance. Returns 0 if no valid
	 * reading is available.
	 *
	 * <p><strong>Technical Details:</strong> Converts the counter period (seconds)
	 * to microseconds, then divides by 10 to get centimeters.
	 *
	 * @return Distance measurement (use .in(Centimeters), .in(Inches), etc. to convert)
	 */
	@Override
	public Distance getDistance() {
		// getPeriod returns time in seconds. The hardware resolution is microseconds.
		// The LIDAR-Lite unit sends a high signal for 10 microseconds per cm of distance.
		double cm = (counter.getPeriod() * 1000000.0 / 10.0);
		return Centimeters.of(cm);
	}
}
