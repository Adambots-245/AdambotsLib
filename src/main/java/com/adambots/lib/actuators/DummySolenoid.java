package com.adambots.lib.actuators;

/**
 * No-op solenoid implementation for disabled subsystems.
 *
 * <p>Use this instead of {@code null} when a subsystem's solenoid is not physically present.
 * Tracks internal state for {@link #toggle()} / {@link #get()} consistency, but performs
 * no hardware I/O.
 */
public class DummySolenoid implements BaseSolenoid {

    private boolean state = false;

    @Override
    public void enable() { state = true; }

    @Override
    public void disable() { state = false; }

    @Override
    public void toggle() { state = !state; }

    @Override
    public boolean get() { return state; }

    @Override
    public void set(boolean value) { state = value; }
}
