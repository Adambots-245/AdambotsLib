// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.lib.sensors;

import com.adambots.lib.utils.Utils;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.units.measure.Angle;
import static edu.wpi.first.units.Units.*;

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
 * import static edu.wpi.first.units.Units.*;
 *
 * BaseAbsoluteEncoder armEncoder = new ThroughBoreEncoder(5);  // DIO port 5
 *
 * // Get absolute position
 * Angle position = armEncoder.getPosition();
 * double degrees = position.in(Degrees);
 * double radians = position.in(Radians);
 *
 * // Get as Rotation2d for use with WPILib kinematics
 * Rotation2d rotation = armEncoder.getPositionRotation2d();
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
@Logged
public class ThroughBoreEncoder implements BaseAbsoluteEncoder {
    @NotLogged
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

    /**
     * Returns the absolute position of the encoder.
     *
     * @return Absolute position (use .in(Degrees) or .in(Radians) to convert)
     */
    @Override
    public Angle getPosition() {
        return Degrees.of(encoder.get() * 360.0);
    }

    /**
     * Returns the absolute position as a Rotation2d.
     *
     * @return Absolute position as Rotation2d
     */
    @Override
    public Rotation2d getPositionRotation2d() {
        return Rotation2d.fromDegrees(encoder.get() * 360.0);
    }
}
