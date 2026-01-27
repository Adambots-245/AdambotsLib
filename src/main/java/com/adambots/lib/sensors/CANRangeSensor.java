package com.adambots.lib.sensors;

import com.adambots.lib.utils.Utils;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.sim.CANrangeSimState;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.units.measure.Distance;
import static edu.wpi.first.units.Units.*;

/**
 * Class to represent the CANRange sensor that measures distance from an object.
 * Implements the BaseDistanceSensor interface.
 *
 * <p><strong>Usage Example:</strong>
 * <pre>{@code
 * import static edu.wpi.first.units.Units.*;
 *
 * CANRangeSensor sensor = new CANRangeSensor(1, true);
 * Distance distance = sensor.getDistance();
 * double cm = distance.in(Centimeters);
 * double meters = distance.in(Meters);
 * }</pre>
 */
public class CANRangeSensor implements BaseDistanceSensor {

    private CANrange canRangeSensor;
    private CANrangeSimState simState;
    private final boolean isSim;

    /**
     * Constructor for CANRangeSensor.
     * @param deviceId The device ID of the CANRange sensor.
     * @param isOnCANivore Whether the sensor is on a CANivore bus.
     */
    public CANRangeSensor(int deviceId, boolean isOnCANivore) {
        // Validate CAN ID range
        if (deviceId < 0 || deviceId > 62) {
            Utils.reportError("CANRangeSensor: Invalid CAN ID " + deviceId +
                ". Valid range: 0-62. Defaulting to 1.");
            deviceId = 1;
        }

        if (isOnCANivore) {
            canRangeSensor = new CANrange(deviceId, new CANBus("*"));
        } else {
            canRangeSensor = new CANrange(deviceId);
        }
        isSim = RobotBase.isSimulation();

        if (isSim) {
            simState = canRangeSensor.getSimState();
            // Set default simulation values
            simState.setDistance(0.0);  // Start at 0 meters
        }
    }

    /**
     * Gets the current distance measurement.
     *
     * @return Distance measurement (use .in(Centimeters), .in(Meters), etc. to convert)
     */
    @Override
    public Distance getDistance() {
        // CANrange returns distance in meters
        double meters = canRangeSensor.getDistance().getValueAsDouble();
        return Meters.of(meters);
    }

    /**
     * Sets the simulated distance for testing.
     * Only works in simulation mode.
     * @param distance The distance to simulate
     */
    public void setSimulatedDistance(Distance distance) {
        if (isSim) {
            simState.setDistance(distance.in(Meters));
        }
    }

    /**
     * Returns whether we are running in simulation mode
     * @return true if in simulation
     */
    public boolean isSim() {
        return isSim;
    }

    /**
     * Gets the raw distance in meters.
     * Useful for testing.
     * @return Distance in meters
     */
    public double getRawDistanceMeters() {
        return canRangeSensor.getDistance().getValueAsDouble();
    }
}
