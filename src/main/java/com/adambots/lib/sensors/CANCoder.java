/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.adambots.lib.sensors;

import com.adambots.lib.utils.Utils;
import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import static edu.wpi.first.units.Units.*;

/**
 * Generic Absolute Encoder sensor to hide actual implementation and ensure uniform values across subsystems.
 * This implementation uses the CTRE CANcoder.
 *
 * <p><strong>Usage Example:</strong>
 * <pre>{@code
 * import static edu.wpi.first.units.Units.*;
 *
 * BaseAbsoluteEncoder encoder = new CANCoder(5);
 *
 * Angle position = encoder.getPosition();
 * double degrees = position.in(Degrees);
 *
 * Rotation2d rotation = encoder.getPositionRotation2d();
 * }</pre>
 */
public class CANCoder implements BaseAbsoluteEncoder {
    private CANcoder encoder;

    public CANCoder (int port){
        // Validate CAN ID range
        if (port < 0 || port > 62) {
            Utils.reportError("CANCoder: Invalid CAN ID " + port +
                ". Valid range: 0-62. Defaulting to 1.");
            port = 1;
        }
        this.encoder = new CANcoder(port); //Defining the encoder using the port passed in
    }

    /**
     * Returns the absolute position of the encoder.
     *
     * @return Absolute position (use .in(Degrees) or .in(Radians) to convert)
     */
    @Override
    public Angle getPosition() {
        // CANcoder returns position in rotations, convert to degrees
        return Degrees.of(encoder.getAbsolutePosition().getValueAsDouble() * 360);
    }

    /**
     * Returns the absolute position as a Rotation2d.
     *
     * @return Absolute position as Rotation2d
     */
    @Override
    public Rotation2d getPositionRotation2d() {
        return Rotation2d.fromDegrees(encoder.getAbsolutePosition().getValueAsDouble() * 360);
    }
}
