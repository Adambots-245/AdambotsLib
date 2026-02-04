
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
import edu.wpi.first.wpilibj.DigitalInput;

/**
 * Photo-eye beam break sensor or limit switch wrapper for FRC robotics.
 *
 * <p>This class provides a simple interface to digital proximity sensors such as
 * photo-eyes (infrared beam sensors) or limit switches. Supports inverted mode
 * for fail-safe wiring configurations.
 *
 * <p><strong>Safety Note:</strong> Digital inputs read HIGH (true) when disconnected.
 * If wiring the sensor to be normally-HIGH (true when not detecting), use inverted=true
 * to maintain expected behavior and ensure a non-critical failure mode.
 *
 * <p><strong>Usage Examples:</strong>
 * <pre>{@code
 * // Normal wiring (LOW when detecting)
 * PhotoEye intake = new PhotoEye(0, false);
 * if (intake.isDetecting()) {
 *     // Game piece detected
 * }
 *
 * // Inverted wiring (HIGH when detecting - fail-safe mode)
 * PhotoEye shooter = new PhotoEye(1, true);
 * }</pre>
 *
 * @see BaseProximitySensor
 * @see edu.wpi.first.wpilibj.DigitalInput
 */
@Logged
public class PhotoEye implements BaseProximitySensor {
    @NotLogged
    private DigitalInput sensor;

    @NotLogged
    private boolean inverted;

    /**
     * Creates a new photo-eye or limit switch sensor on the specified digital input port.
     *
     * <p>If the sensor is wired to return HIGH (true) when NOT detecting (fail-safe mode),
     * set inverted=true to maintain the expected behavior where {@link #isDetecting()}
     * returns true when an object is present.
     *
     * <p><strong>Wiring Note:</strong> Digital inputs return true when disconnected.
     * Using inverted wiring ensures a broken wire is detected as a fault rather than
     * falsely indicating detection.
     *
     * @param port Digital I/O port number (0-9 on roboRIO)
     * @param inverted True to invert the sensor reading (useful for fail-safe wiring)
     */
    public PhotoEye (int port, boolean inverted){
        // Validate DIO port range (RoboRIO 2 has 10 DIO ports: 0-9)
        if (port < 0 || port > 9) {
            Utils.reportError("PhotoEye: Invalid DIO port " + port +
                ". RoboRIO 2 has 10 DIO ports (0-9). Defaulting to 0.");
            port = 0;
        }
        this.sensor = new DigitalInput(port);
        this.inverted = inverted;
    }

    /**
     * Checks if the sensor is currently detecting an object.
     *
     * <p>This method automatically accounts for the inverted flag set in the constructor.
     * It will always return true when an object is present, regardless of the sensor's
     * actual wiring configuration.
     *
     * @return True if an object is detected in the sensor's beam/switch, false otherwise
     */
    public boolean isDetecting(){
        if (inverted) return !sensor.get();
        return sensor.get();
    }
}