// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.lib.sensors;

import com.adambots.lib.utils.Utils;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

/**
 * REV Through Bore Encoder implementation using DutyCycle encoding.
 *
 * <p>The REV Through Bore Encoder is an absolute encoder that uses a duty cycle
 * signal to report position. It provides 8192 counts per revolution (CPR) and
 * maintains position through power cycles.
 *
 * <p><strong>Hardware:</strong>
 * <ul>
 *   <li>Connection: DIO port on RoboRIO (uses DutyCycle protocol)</li>
 *   <li>Resolution: 8192 CPR (13-bit absolute)</li>
 *   <li>Operating Range: 0-1 rotation (absolute), continuous for relative</li>
 *   <li>RoboRIO 2: 10 DIO ports (0-9)</li>
 * </ul>
 *
 * <p><strong>Usage Example:</strong>
 * <pre>{@code
 * BaseAbsoluteEncoder armEncoder = new ThroughBoreEncoder(5);  // DIO port 5
 *
 * // Get absolute position (0-1 rotation)
 * double position = armEncoder.getAbsolutePositionDegrees();
 *
 * // Get position in radians
 * double radians = armEncoder.getAbsolutePositionRadians();
 *
 * // Get as Rotation2d for use with WPILib kinematics
 * Rotation2d rotation = armEncoder.getAbsolutePositionRotation2D();
 * }</pre>
 *
 * <p><strong>Wiring:</strong>
 * <ul>
 *   <li>Red: 5V from RoboRIO</li>
 *   <li>Black: Ground</li>
 *   <li>White: Signal to DIO port</li>
 * </ul>
 *
 * <p><strong>Safety:</strong> Port range is validated on construction.
 * Invalid ports default to port 0 with a DriverStation error.
 *
 * @see BaseAbsoluteEncoder
 * @see edu.wpi.first.wpilibj.DutyCycleEncoder
 */
public class ThroughBoreEncoder implements BaseAbsoluteEncoder {
    private DutyCycleEncoder encoder;

    public ThroughBoreEncoder(int encoderPort) {
        // Validate DIO port range (RoboRIO 2 has 10 DIO ports: 0-9)
        if (encoderPort < 0 || encoderPort > 9) {
            Utils.reportError("ThroughBoreEncoder: Invalid DIO port " + encoderPort +
                ". RoboRIO 2 has 10 DIO ports (0-9). Defaulting to 0.");
            encoderPort = 0;
        }
        encoder = new DutyCycleEncoder(encoderPort);
    }

    @Override
    public double getAbsolutePositionDegrees() {
        return encoder.get() * 360.0;
    }

    @Override
    public double getAbsolutePositionRadians() {
        return Math.toRadians(getAbsolutePositionDegrees());
    }

    @Override
    public Rotation2d getAbsolutePositionRotation2D() {
        return new Rotation2d(getAbsolutePositionRadians());
    }
}
