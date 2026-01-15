// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.lib.actuators;

import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.Relay.Value;

/**
 * Electrical solenoid implementation using RoboRIO Relay ports.
 *
 * <p>Use this class for electrical solenoids (not pneumatic) that are
 * controlled via the Relay output ports on the RoboRIO. These are typically
 * used for high-current devices like solenoid valves that control fluid flow.
 *
 * <p><strong>Hardware:</strong>
 * <ul>
 *   <li>RoboRIO 1/2: 4 Relay ports (0-3)</li>
 *   <li>Each port can source 2A continuous, 4A peak</li>
 *   <li>Use appropriate relay module for higher currents</li>
 * </ul>
 *
 * <p><strong>Usage Example:</strong>
 * <pre>{@code
 * BaseSolenoid solenoid = new ElectricalSolenoid(0);  // Relay port 0
 * solenoid.enable();   // Turn on
 * solenoid.disable();  // Turn off
 * solenoid.toggle();   // Toggle state
 * }</pre>
 *
 * <p><strong>Safety:</strong> Port range is validated on construction.
 * Invalid ports default to port 0 with a DriverStation error.
 */
public class ElectricalSolenoid implements BaseSolenoid {

    // Electrical solenoids are typically connected to the Relay port on the Rio
    private Relay relay;

    public ElectricalSolenoid(int port){
        // Validate Relay port range (RoboRIO 2 has 4 ports: 0-3)
        if (port < 0 || port > 3) {
            edu.wpi.first.wpilibj.DriverStation.reportError(
                "ElectricalSolenoid: Invalid Relay port " + port +
                ". RoboRIO 2 has 4 Relay ports (0-3). Defaulting to port 0.", false);
            port = 0;
        }
        relay = new Relay(port);
    }

    @Override
    public void enable() {
        relay.set(Value.kOn);
    }

    @Override
    public void disable() {
        relay.set(Value.kOff);
    }

    @Override
    public void toggle() {
        set(!get());
    }

    @Override
    public boolean get() {
        return (relay.get() == Value.kOn);
    }

    @Override
    public void set(boolean value) {
        relay.set(value ? Value.kOn : Value.kOff);
    }
}
