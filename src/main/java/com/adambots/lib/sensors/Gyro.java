/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.adambots.lib.sensors;

import com.adambots.lib.utils.Utils;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import static edu.wpi.first.units.Units.*;

/**
 * Generic gyro sensor to hide actual implementation and ensure uniform values across subsystems.
 * This implementation uses the Pigeon2 IMU.
 *
 * <p><strong>Usage Example:</strong>
 * <pre>{@code
 * import static edu.wpi.first.units.Units.*;
 *
 * BaseGyro gyro = new Gyro(1);
 * gyro.resetYaw();  // Zero the heading
 *
 * Angle heading = gyro.getYaw();
 * double degrees = heading.in(Degrees);
 *
 * Rotation2d angle = gyro.getYawRotation2d();  // For WPILib geometry
 * }</pre>
 */
@Logged
public class Gyro implements BaseGyro {
    @NotLogged
    private Pigeon2 gyro;

    public Gyro (int CANport){
        // Validate CAN ID range (typically 0-62 for CTRE devices)
        if (CANport < 0 || CANport > 62) {
            Utils.reportError("Gyro: Invalid CAN ID " + CANport +
                ". Valid range: 0-62. Defaulting to 1.");
            CANport = 1;
        }
        this.gyro = new Pigeon2(CANport); //Defining the gyroscope using the configured CAN ID
    }

    /**
     * Returns the continuous yaw angle.
     * <p>
     * Ensure CCW is a positive value change.
     *
     * @return Continuous yaw angle (use .in(Degrees) or .in(Radians) to convert)
     */
    @Override
    public Angle getYaw() {
        return Degrees.of(-gyro.getYaw().getValueAsDouble());
    }

    /**
     * Returns the continuous yaw angle as a Rotation2d.
     * <p>
     * Ensure CCW is a positive value change.
     *
     * @return Continuous yaw as Rotation2d
     */
    @Override
    public Rotation2d getYawRotation2d() {
        return Rotation2d.fromDegrees(-gyro.getYaw().getValueAsDouble());
    }

    /**
     * Zeros gyroscope yaw.
     */
    @Override
    public void resetYaw() {
        gyro.reset();
    }

    /**
     * Resets the yaw of the gyroscope to the specified value.
     *
     * @param angle The angle to reset to
     */
    @Override
    public void resetYawToAngle(Angle angle) {
        gyro.setYaw(angle.in(Degrees));
    }

    /**
     * Offsets the current yaw of the gyroscope by a specified angle.
     *
     * @param offset The angle offset to add
     */
    @Override
    public void offsetYawByAngle(Angle offset) {
        // Do NOT wrap continuous angles - they need to track beyond 360 for navigation
        double currentYaw = -gyro.getYaw().getValueAsDouble();
        gyro.setYaw(currentYaw + offset.in(Degrees));
    }

    /**
     * Returns the measured pitch value of the gyroscope.
     * <p>
     * Keep in mind roll and pitch will change depending on robot rotation.
     *
     * @return Pitch angle (use .in(Degrees) or .in(Radians) to convert)
     */
    @Override
    public Angle getPitch() {
        return Degrees.of(gyro.getRoll().getValueAsDouble()); //returns roll as pigeon has roll defined about conventional pitch axis
    }

    /**
     * Returns the measured roll value of the gyroscope.
     * <p>
     * Keep in mind roll and pitch will change depending on robot rotation.
     *
     * @return Roll angle (use .in(Degrees) or .in(Radians) to convert)
     */
    @Override
    public Angle getRoll() {
        return Degrees.of(gyro.getPitch().getValueAsDouble()); //returns pitch as pigeon has pitch defined about conventional roll axis
    }
}
