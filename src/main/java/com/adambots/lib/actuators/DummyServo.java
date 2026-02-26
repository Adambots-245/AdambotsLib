package com.adambots.lib.actuators;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Current;

/**
 * No-op servo implementation for disabled subsystems.
 *
 * <p>Use this instead of {@code null} when a subsystem's servo is not physically present.
 * All methods are safe to call — setters do nothing, getters return zero/default values.
 */
public class DummyServo implements BaseServo {

    @Override
    public void set(double speed) {}

    @Override
    public ServoMode getMode() { return ServoMode.ANGULAR; }

    @Override
    public void turnCounterclockwise() {}

    @Override
    public void turnClockwise() {}

    @Override
    public void stop() {}

    @Override
    public void setPulseWidth(int pulseWidth) {}

    @Override
    public Current getCurrent() { return Amps.of(0); }
}
