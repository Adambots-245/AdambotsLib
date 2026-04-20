package com.adambots.lib.actuators;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.*;

/**
 * No-op motor implementation for disabled subsystems.
 *
 * <p>Use this instead of {@code null} when a subsystem's motor is not physically present.
 * All methods are safe to call — setters do nothing, getters return zero/default values.
 * This prevents null checks throughout robot code and eliminates Epilogue logging errors.
 */
@Logged
public final class DummyMotor implements BaseMotor {

    @Override
    public void set(double speed) {}

    @Override
    public void set(ControlMode mode, double value) {}

    @Override
    public void setPID(int slotIdx, double kP, double kI, double kD, double kF) {}

    @Override
    public void configureMotionMagic(AngularVelocity cruiseVelocity, AngularAcceleration acceleration, double jerkRPSPerSecPerSec) {}

    @Override
    public void configureCurrentLimits(Current stallLimit, Current freeLimit, double limitRpmThreshold) {}

    @Override
    public void configureSoftLimits(double forwardLimitRotations, double reverseLimitRotations, boolean enable) {}

    @Override
    public void enableSoftLimits(boolean enable) {}

    @Override
    public void setInverted(boolean inverted) {}

    @Override
    public void setBrakeMode(boolean brake) {}

    @Override
    public void setPosition(double rotations) {}

    @Override
    public void enableVoltageCompensation(Voltage nominalVoltage) {}

    @Override
    public double getPosition() { return 0.0; }

    @Override
    public AngularVelocity getVelocity() { return RotationsPerSecond.of(0); }

    @Override
    public AngularAcceleration getAcceleration() { return RotationsPerSecondPerSecond.of(0); }

    @Override
    public Current getCurrentDraw() { return Amps.of(0); }

    @Override
    public double getOutputPercent() { return 0.0; }

    @Override
    public double getTemperature() { return 0.0; }

    @Override
    public boolean getForwardLimitSwitch() { return false; }

    @Override
    public boolean getReverseLimitSwitch() { return false; }

    @Override
    public void setStrictFollower(int leaderDeviceID) {}

    @Override
    public void configureHardLimits(boolean enableForward, boolean enableReverse,
                                    double forwardResetValueRotations, double reverseResetValueRotations) {}

    @Override
    public String getMotorType() { return "DummyMotor"; }

    @Override
    public boolean supportsControlMode(ControlMode mode) { return false; }
}
