// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.lib.actuators;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

/**
 * CTRE Pneumatics Control Module (PCM) double solenoid implementation.
 *
 * <p>Use this class for pneumatic double solenoids connected to a CTRE PCM.
 * Double solenoids have two ports (forward and reverse) that control
 * a two-position pneumatic cylinder.
 *
 * <p><strong>Hardware:</strong>
 * <ul>
 *   <li>CTRE PCM: 8 solenoid ports (0-7)</li>
 *   <li>CAN ID must be configured on the PCM</li>
 *   <li>Requires compressor and pneumatic system</li>
 * </ul>
 *
 * <p><strong>Usage Example:</strong>
 * <pre>{@code
 * BaseSolenoid gripper = new CTREPneumaticSolenoid(0, 1);  // Forward: 0, Reverse: 1
 * gripper.enable();   // Extend (forward)
 * gripper.disable();  // Retract (reverse)
 * gripper.toggle();   // Toggle position
 * }</pre>
 *
 * <p><strong>Mapping:</strong>
 * <ul>
 *   <li>{@code enable()} → Forward position (port 0 in example)</li>
 *   <li>{@code disable()} → Reverse position (port 1 in example)</li>
 * </ul>
 *
 * <p><strong>Safety:</strong> Port ranges and port uniqueness are validated
 * on construction. Invalid ports default to 0 and 1 with DriverStation errors.
 */
public class CTREPneumaticSolenoid implements BaseSolenoid{
    private DoubleSolenoid solenoid;

    public CTREPneumaticSolenoid(int forwardPort, int reversePort){
        // Validate PCM port range (0-7)
        if (forwardPort < 0 || forwardPort > 7) {
            edu.wpi.first.wpilibj.DriverStation.reportError(
                "CTREPneumaticSolenoid: Invalid forward port " + forwardPort +
                ". PCM has 8 ports (0-7). Defaulting to port 0.", false);
            forwardPort = 0;
        }

        if (reversePort < 0 || reversePort > 7) {
            edu.wpi.first.wpilibj.DriverStation.reportError(
                "CTREPneumaticSolenoid: Invalid reverse port " + reversePort +
                ". PCM has 8 ports (0-7). Defaulting to port 1.", false);
            reversePort = 1;
        }

        // Validate that forward and reverse ports are different
        if (forwardPort == reversePort) {
            edu.wpi.first.wpilibj.DriverStation.reportError(
                "CTREPneumaticSolenoid: Forward port (" + forwardPort +
                ") and reverse port (" + reversePort + ") must be different. " +
                "Defaulting to ports 0 and 1.", false);
            forwardPort = 0;
            reversePort = 1;
        }

        solenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, forwardPort, reversePort);
    }

    /**
     * Set solenoid to forward position (extends cylinder).
     * Same as calling {@link #enable()}.
     */
    public void forward(){
        solenoid.set(Value.kForward);
    }

    /**
     * Set solenoid to reverse position (retracts cylinder).
     * Same as calling {@link #disable()}.
     */
    public void reverse(){
        solenoid.set(Value.kReverse);
    }

    @Override
    public void enable() {
        forward();
    }

    @Override
    public void disable() {
        reverse();
    }

    @Override
    public void toggle() {
        set(!get());
    }

    @Override
    public boolean get() {
        return (solenoid.get() == Value.kForward);
    }

    @Override
    public void set(boolean value) {
        solenoid.set(value ? Value.kForward : Value.kReverse);
    }


}
