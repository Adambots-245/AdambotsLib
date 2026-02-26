package com.adambots.lib.sensors;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;

/**
 * No-op gyroscope implementation for disabled subsystems.
 *
 * <p>Use this instead of {@code null} when a subsystem's gyro is not physically present.
 * All methods are safe to call — setters do nothing, getters return zero/default values.
 */
@Logged
public class DummyGyro implements BaseGyro {

    @Override
    public Angle getYaw() { return Degrees.of(0); }

    @Override
    public Rotation2d getYawRotation2d() { return new Rotation2d(); }

    @Override
    public void offsetYawByAngle(Angle offset) {}

    @Override
    public void resetYaw() {}

    @Override
    public void resetYawToAngle(Angle angle) {}

    @Override
    public Angle getPitch() { return Degrees.of(0); }

    @Override
    public Angle getRoll() { return Degrees.of(0); }
}
