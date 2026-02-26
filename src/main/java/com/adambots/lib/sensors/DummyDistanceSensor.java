package com.adambots.lib.sensors;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Distance;

/**
 * No-op distance sensor implementation for disabled subsystems.
 *
 * <p>Use this instead of {@code null} when a subsystem's distance sensor is not physically present.
 * Returns a large distance (9999 meters) so that any proximity/range checks evaluate as "nothing detected."
 */
@Logged
public class DummyDistanceSensor implements BaseDistanceSensor {

    @Override
    public Distance getDistance() { return Meters.of(9999); }
}
