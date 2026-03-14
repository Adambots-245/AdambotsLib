package com.adambots.lib.sensors;

import com.adambots.lib.utils.Utils;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import static edu.wpi.first.units.Units.*;

/**
 * Potentiometer implementation of {@link BaseAbsoluteEncoder} using an analog input port.
 *
 * <p>Supports single-turn and multi-turn potentiometers (e.g., 10-turn pots) connected
 * to the RoboRIO's analog input ports. The potentiometer's voltage is linearly mapped
 * to a position in degrees based on the configured full range.
 *
 * <p><strong>Hardware:</strong>
 * <ul>
 *   <li>Connection: Analog input port on RoboRIO (0-3 onboard, 4-7 on MXP)</li>
 *   <li>Resolution: 12-bit (4096 steps over the full range)</li>
 *   <li>Voltage range: 0-5V</li>
 *   <li>RoboRIO 2: 8 analog input ports (0-7)</li>
 * </ul>
 *
 * <p><strong>Note:</strong> Unlike single-rotation encoders, the position does NOT wrap
 * at 360°. A 10-turn pot with {@code fullRange=3600} returns values from 0 to 3600°.
 *
 * <p><strong>Usage Example:</strong>
 * <pre>{@code
 * import static edu.wpi.first.units.Units.*;
 *
 * // 10-turn pot on analog port 0, full range 3600°
 * BaseAbsoluteEncoder pot = new Potentiometer(0, 3600);
 *
 * // Single-turn pot with offset
 * BaseAbsoluteEncoder pot2 = new Potentiometer(1, 360, 45);
 *
 * Angle position = pot.getPosition();
 * double degrees = position.in(Degrees);
 *
 * Rotation2d rotation = pot.getPositionRotation2d();
 * }</pre>
 *
 * <p><strong>Wiring:</strong>
 * <ul>
 *   <li>Red: 5V from RoboRIO</li>
 *   <li>Black: Ground</li>
 *   <li>White: Signal to analog input port</li>
 * </ul>
 *
 * <p><strong>Safety:</strong> Port range is validated on construction.
 * Invalid ports default to port 0 with a DriverStation error.
 *
 * @see BaseAbsoluteEncoder
 * @see edu.wpi.first.wpilibj.AnalogPotentiometer
 */
@Logged
public class Potentiometer implements BaseAbsoluteEncoder {
    @NotLogged
    private final AnalogPotentiometer potentiometer;

    /**
     * Creates a potentiometer on an analog input port.
     *
     * @param port      Analog input port (0-7 on RoboRIO 2)
     * @param fullRange Full range of the potentiometer in degrees
     *                  (e.g., 3600 for a 10-turn pot, 360 for single-turn)
     * @param offset    Offset in degrees added to the reading
     */
    public Potentiometer(int port, double fullRange, double offset) {
        // Validate analog port range (RoboRIO 2 has 8 analog input ports: 0-7)
        if (port < 0 || port > 7) {
            Utils.reportError("Potentiometer: Invalid analog port " + port +
                ". RoboRIO 2 has 8 analog input ports (0-7). Defaulting to 0.");
            port = 0;
        }
        potentiometer = new AnalogPotentiometer(port, fullRange, offset);
    }

    /**
     * Creates a potentiometer with zero offset.
     *
     * @param port      Analog input port (0-7 on RoboRIO 2)
     * @param fullRange Full range of the potentiometer in degrees
     *                  (e.g., 3600 for a 10-turn pot, 360 for single-turn)
     */
    public Potentiometer(int port, double fullRange) {
        this(port, fullRange, 0);
    }

    /**
     * Returns the position of the potentiometer.
     *
     * <p>The value is in degrees based on the configured full range.
     * For multi-turn pots, this does NOT wrap at 360°.
     *
     * @return Position as Angle (use .in(Degrees) or .in(Radians) to convert)
     */
    @Override
    public Angle getPosition() {
        return Degrees.of(potentiometer.get());
    }

    /**
     * Returns the position as a Rotation2d.
     *
     * @return Position as Rotation2d
     */
    @Override
    public Rotation2d getPositionRotation2d() {
        return Rotation2d.fromDegrees(potentiometer.get());
    }
}
