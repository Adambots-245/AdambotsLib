// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.lib.sensors;

import com.adambots.lib.utils.Utils;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.units.measure.Distance;
import static edu.wpi.first.units.Units.*;

/**
 * Generic UltrasonicSensor sensor to hide actual implementation and perform range finding
 */
public class UltrasonicSensor implements BaseDistanceSensor {
    // The handle to access the sensor
    private final AnalogInput rangefinder;

    // The scaling factor:  distance in cm = volts returned * SCALING_FACTOR
    // 5120 mm per 5V or 512 cm per 5V. The 24/23 is a correction factor to adjust the reading
    private final double SCALING_FACTOR = 512/5*24/23;

    /** Creates a new ultrasonic sensor hooked up to <code>portNumber</code> on the analog breakout.
     * @params portNumber The port number on the breakout.
     */
    public UltrasonicSensor(int portNumber){
        // Validate analog port range (RoboRIO 2 has 4 analog ports: 0-3)
        if (portNumber < 0 || portNumber > 3) {
            Utils.reportError("UltrasonicSensor: Invalid analog port " + portNumber +
                ". RoboRIO 2 has 4 analog ports (0-3). Defaulting to 0.");
            portNumber = 0;
        }

        rangefinder = new AnalogInput(portNumber);

        // Configure oversampling and averaging for noise reduction
        // Values chosen for balance between response time and accuracy
        rangefinder.setOversampleBits(2);  // 4x oversampling (2^2 = 4 samples)
        rangefinder.setAverageBits(5);     // Average over 32 samples (2^5 = 32)
    }

    /**
     * Take a measurement and return the distance.
     *
     * @return Distance measurement (use .in(Centimeters), .in(Inches), etc. to convert)
     */
    @Override
    public Distance getDistance() {
        double cm = rangefinder.getAverageVoltage() * SCALING_FACTOR;
        return Centimeters.of(cm);
    }
}
