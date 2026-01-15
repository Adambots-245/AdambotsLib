// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.lib.sensors;

import com.adambots.lib.utils.Utils;

import edu.wpi.first.wpilibj.DigitalInput;

/**
 * Digital limit switch sensor implementation.
 *
 * <p>Use this class for mechanical limit switches that provide digital on/off
 * feedback when a mechanism reaches a physical limit. Commonly used for homing,
 * position verification, and safety interlocks.
 *
 * <p><strong>Hardware:</strong>
 * <ul>
 *   <li>RoboRIO 2: 10 DIO ports (0-9)</li>
 *   <li>Normally-open or normally-closed switches supported via inverted parameter</li>
 *   <li>Pull-up resistor typically required for normally-open switches</li>
 * </ul>
 *
 * <p><strong>Usage Example:</strong>
 * <pre>{@code
 * // Normally-closed limit switch on DIO 0
 * BaseProximitySensor bottomLimit = new LimitSwitch(0, false);
 *
 * // Normally-open limit switch on DIO 1 (inverted)
 * BaseProximitySensor topLimit = new LimitSwitch(1, true);
 *
 * // Check if limit is triggered
 * if (bottomLimit.isDetecting()) {
 *     motor.stopMotor();
 * }
 * }</pre>
 *
 * <p><strong>Safety:</strong> Port range is validated on construction.
 * Invalid ports default to port 0 with a DriverStation error.
 *
 * @see BaseProximitySensor
 */
public class LimitSwitch implements BaseProximitySensor {
    private DigitalInput sensor;
    private boolean inverted;

    public LimitSwitch(int port, boolean inverted) {
        // Validate DIO port range (RoboRIO 2 has 10 DIO ports: 0-9)
        if (port < 0 || port > 9) {
            Utils.reportError("LimitSwitch: Invalid DIO port " + port +
                ". RoboRIO 2 has 10 DIO ports (0-9). Defaulting to 0.");
            port = 0;
        }
        this.sensor = new DigitalInput(port);
        this.inverted = inverted;
    }

    @Override
    public boolean isDetecting() {
        if (inverted) return !sensor.get();
        return sensor.get(); 
    }
}
