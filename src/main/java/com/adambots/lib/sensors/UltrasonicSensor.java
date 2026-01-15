// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.lib.sensors;

import com.adambots.lib.utils.Utils;

import edu.wpi.first.wpilibj.AnalogInput;

/**
 * Generic UltrasonicSensor sensor to hide actual implementation and perform range finding
 */
public class UltrasonicSensor implements BaseDistanceSensor {
    // The handle to access the sensor
    private final AnalogInput rangefinder;
    
    // The scaling factor:  distance in inches = volts returned / SCALING_FACTOR
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

    /** Returns the distance measured in inches.  */
    public double getDistanceInCentimeters(){
        return rangefinder.getAverageVoltage() * SCALING_FACTOR;
    }
    
    /** Returns the distance measured in inches.  */
    @Override
    public double getDistanceInInches(){
        return getDistanceInCentimeters() / 2.54;
    }
    
    /** Returns the distance measured in feet.  */
    @Override
    public double getDistanceInFeet(){
        return getDistanceInInches() / 12.0;
    }
}
