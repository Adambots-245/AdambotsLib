package com.adambots.lib.sensors;

import edu.wpi.first.epilogue.Logged;

/**
 * No-op proximity sensor implementation for disabled subsystems.
 *
 * <p>Use this instead of {@code null} when a subsystem's proximity sensor is not physically present.
 * Always returns {@code false} (nothing detected) as a safe default.
 */
@Logged
public class DummyProximitySensor implements BaseProximitySensor {

    @Override
    public boolean isDetecting() { return false; }
}
