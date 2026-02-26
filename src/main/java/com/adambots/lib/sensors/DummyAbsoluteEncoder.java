package com.adambots.lib.sensors;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;

/**
 * No-op absolute encoder implementation for disabled subsystems.
 *
 * <p>Use this instead of {@code null} when a subsystem's encoder is not physically present.
 * All methods are safe to call and return zero-position values.
 */
@Logged
public class DummyAbsoluteEncoder implements BaseAbsoluteEncoder {

    @Override
    public Angle getPosition() { return Degrees.of(0); }

    @Override
    public Rotation2d getPositionRotation2d() { return new Rotation2d(); }
}
