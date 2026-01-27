package com.adambots.lib.actuators;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;

import static edu.wpi.first.units.Units.*;
import edu.wpi.first.units.measure.*;

/**
 * A wrapper class for the CTRE TalonFX motor controller that implements the
 * BaseMotor interface.
 * This class provides a standardized interface for controlling TalonFX motors,
 * including
 * Falcon 500 and Kraken X60 motors, with support for various control modes,
 * configuration options,
 * and status reporting.
 * 
 * <p>
 * Features include:
 * <ul>
 * <li>Multiple control modes (Percent Output, Position, Velocity, etc.)</li>
 * <li>PID configuration with multiple slots</li>
 * <li>Motion Magic configuration</li>
 * <li>Current limiting</li>
 * <li>Soft and hard limit switch support</li>
 * <li>Voltage compensation</li>
 * <li>Motor following capabilities</li>
 * </ul>
 * 
 * <p>
 * The class automatically optimizes CAN bus usage by configuring appropriate
 * update frequencies for different status signals and includes special handling
 * for Kraken X60 motors when using current control mode.
 * 
 * <p>
 * Example usage:
 * 
 * <pre>
 * TalonFXMotor motor = new TalonFXMotor(1, true, 40, false);
 * motor.setBrakeMode(true);
 * motor.set(ControlMode.PERCENT_OUTPUT, 0.5);
 * </pre>
 * 
 * @see BaseMotor
 * @see com.ctre.phoenix6.hardware.TalonFX
 */
public class TalonFXMotor implements BaseMotor {
    private final TalonFX motor;
    private final boolean isKraken;
    private boolean focFlag = false;
    private double feedForward = 0.0;
    private final int maxRetries = 3; // Maximum retries for configuration

    // Reusable control request objects to avoid allocation overhead
    private final VoltageOut voltageRequest = new VoltageOut(0);
    private final PositionDutyCycle positionRequest = new PositionDutyCycle(0);
    private final VelocityDutyCycle velocityRequest = new VelocityDutyCycle(0);
    private final MotionMagicDutyCycle motionMagicRequest = new MotionMagicDutyCycle(0);

    /**
     * Functional interface for applying a configuration and returning a StatusCode.
     */
    @FunctionalInterface
    interface ConfigApplyAction {
        StatusCode apply();
    }

    /**
     * Constructs a TalonFXMotor instance.
     *
     * @param portNum            The port number to which the motor is connected.
     * @param isOnCANivore       A boolean indicating if the motor is on a CANivore
     *                           bus.
     * @param supplyCurrentLimit The supply current limit for the motor.
     * @param isKraken           A boolean indicating if the motor is part of the
     *                           Kraken subsystem.
     */
    public TalonFXMotor(int portNum, boolean isOnCANivore, double supplyCurrentLimit, boolean isKraken) {
        this.isKraken = isKraken;

        // Initialize motor on either CANivore or regular CAN bus
        if (isOnCANivore) {
            motor = new TalonFX(portNum, new CANBus("*"));
        } else {
            motor = new TalonFX(portNum);
        }

        // Configure default current limits
        var currentLimits = new CurrentLimitsConfigs();

        // CRITICAL: Refresh before apply to avoid factory defaulting other config fields
        StatusCode refreshStatus = motor.getConfigurator().refresh(currentLimits);
        if (!refreshStatus.isOK()) {
            edu.wpi.first.wpilibj.DriverStation.reportWarning(
                "TalonFXMotor: Failed to refresh current limits config (Status: " + refreshStatus +
                "). Configuration may be factory defaulted!", true);
        }

        currentLimits.withSupplyCurrentLimit(supplyCurrentLimit)
                .withSupplyCurrentLimitEnable(true);

        boolean success = applyConfigWithRetry(() -> motor.getConfigurator().apply(currentLimits));
        if (!success) {
            edu.wpi.first.wpilibj.DriverStation.reportError(
                "TalonFXMotor: Failed to apply current limits after retries", false);
        }

        // Configure status frame periods for efficiency
        motor.getVelocity().setUpdateFrequency(50);
        motor.getPosition().setUpdateFrequency(50);
        motor.getForwardLimit().setUpdateFrequency(25);
        motor.getReverseLimit().setUpdateFrequency(25);

        // Optimize CAN bus usage
        motor.optimizeBusUtilization();
    }

    /**
     * Sets the control mode and value for the motor.
     * 
     * @param mode  The ControlMode to set the motor to (PERCENT_OUTPUT, POSITION,
     *              VELOCITY,
     *              VOLTAGE, CURRENT, MOTION_MAGIC, or FOLLOWER)
     * @param value The target value to set:
     *              - For PERCENT_OUTPUT: Output value between -1.0 and 1.0
     *              - For POSITION: Target position in sensor units
     *              - For VELOCITY: Target velocity in sensor units per 100ms
     *              - For VOLTAGE: Target voltage
     *              - For CURRENT: Target current in amps
     *              - For MOTION_MAGIC: Target position in sensor units
     *              - For FOLLOWER: Device ID of the motor to follow
     */
    @Override
    public void set(ControlMode mode, double value) {
        switch (mode) {
            case PERCENT_OUTPUT:
                motor.setControl(new DutyCycleOut(value).withEnableFOC(focFlag));
                break;
            case POSITION:
                // Apply feed-forward if set
                if (feedForward != 0) {
                    motor.setControl(positionRequest.withPosition(value)
                            .withSlot(0).withEnableFOC(focFlag).withFeedForward(feedForward));
                } else {
                    motor.setControl(positionRequest.withPosition(value)
                            .withSlot(0).withEnableFOC(focFlag));
                }
                break;
            case VELOCITY:
                // Apply feed-forward if set
                if (feedForward != 0) {
                    motor.setControl(velocityRequest.withVelocity(value)
                            .withSlot(0).withEnableFOC(focFlag).withFeedForward(feedForward));
                } else {
                    motor.setControl(velocityRequest.withVelocity(value)
                            .withSlot(0).withEnableFOC(focFlag));
                }
                break;
            case VOLTAGE:
                // Use VoltageOut for proper voltage control, not DutyCycleOut
                motor.setControl(voltageRequest.withOutput(value).withEnableFOC(focFlag));
                break;
            case CURRENT:
                if (isKraken) {
                    // Kraken X60 supports FOC (Field Oriented Control) for better current control
                    motor.setControl(new TorqueCurrentFOC(value));
                } else {
                    // For standard TalonFX, use duty cycle control since direct current control
                    // isn't available
                    motor.setControl(new DutyCycleOut(value).withEnableFOC(focFlag));
                }
                break;
            case MOTION_MAGIC:
                // Apply feed-forward if set
                if (feedForward != 0) {
                    motor.setControl(motionMagicRequest.withPosition(value)
                            .withSlot(0).withEnableFOC(focFlag).withFeedForward(feedForward));
                } else {
                    motor.setControl(motionMagicRequest.withPosition(value)
                            .withSlot(0).withEnableFOC(focFlag));
                }
                break;
            case MOTION_MAGIC_FOC_TORQUE:
                motor.setControl(new MotionMagicTorqueCurrentFOC(value).withSlot(0).withFeedForward(feedForward));
                break;
            case FOLLOWER:
                // Follow another Talon FX controller
                int deviceID = (int) value;
                motor.setControl(new com.ctre.phoenix6.controls.Follower(deviceID, MotorAlignmentValue.Aligned));
                break;
        }
    }

    public void enableFOC() {
        focFlag = true;
    }

    public void setFeedForward(double value) {
        feedForward = value;
    }

    /**
     * Sets the speed of the TalonFX motor using duty cycle output control.
     * 
     * @param speed The speed to set the motor to, ranging from -1.0 to 1.0.
     *              Positive values indicate forward rotation, negative values
     *              indicate reverse rotation,
     *              and 0.0 represents stopped.
     */
    @Override
    public void set(double speed) {
        motor.setControl(new DutyCycleOut(speed));
    }

    /**
     * Sets the PID (Proportional, Integral, Derivative) and Feed Forward control
     * constants for the motor controller.
     * The motor controller supports multiple PID slot configurations (0-2) that can
     * be used for different control modes.
     *
     * @param slotIdx The PID slot index to configure (0-2)
     * @param kP      The Proportional gain constant
     * @param kI      The Integral gain constant
     * @param kD      The Derivative gain constant
     * @param kF      The Feed Forward gain constant
     * @throws IllegalArgumentException if slotIdx is not between 0-2
     */
    @Override
    public void setPID(int slotIdx, double kP, double kI, double kD, double kF) {
        switch (slotIdx) {
            case 0:
                var slot0Config = new Slot0Configs();

                // CRITICAL: Refresh before apply to avoid factory defaulting other config fields
                StatusCode refreshStatus0 = motor.getConfigurator().refresh(slot0Config);
                if (!refreshStatus0.isOK()) {
                    edu.wpi.first.wpilibj.DriverStation.reportWarning(
                        "TalonFXMotor: Failed to refresh slot 0 PID config (Status: " + refreshStatus0 +
                        "). Configuration may be factory defaulted!", true);
                }

                slot0Config.withKP(kP).withKI(kI).withKD(kD).withKV(kF);

                boolean success0 = applyConfigWithRetry(() -> motor.getConfigurator().apply(slot0Config, slotIdx));
                if (!success0) {
                    edu.wpi.first.wpilibj.DriverStation.reportError(
                        "TalonFXMotor: Failed to apply slot 0 PID after retries", false);
                }
                break;

            case 1:
                var slot1Config = new Slot1Configs();

                // CRITICAL: Refresh before apply to avoid factory defaulting other config fields
                StatusCode refreshStatus1 = motor.getConfigurator().refresh(slot1Config);
                if (!refreshStatus1.isOK()) {
                    edu.wpi.first.wpilibj.DriverStation.reportWarning(
                        "TalonFXMotor: Failed to refresh slot 1 PID config (Status: " + refreshStatus1 +
                        "). Configuration may be factory defaulted!", true);
                }

                slot1Config.withKP(kP).withKI(kI).withKD(kD).withKV(kF);

                boolean success1 = applyConfigWithRetry(() -> motor.getConfigurator().apply(slot1Config, slotIdx));
                if (!success1) {
                    edu.wpi.first.wpilibj.DriverStation.reportError(
                        "TalonFXMotor: Failed to apply slot 1 PID after retries", false);
                }
                break;

            case 2:
                var slot2Config = new Slot2Configs();

                // CRITICAL: Refresh before apply to avoid factory defaulting other config fields
                StatusCode refreshStatus2 = motor.getConfigurator().refresh(slot2Config);
                if (!refreshStatus2.isOK()) {
                    edu.wpi.first.wpilibj.DriverStation.reportWarning(
                        "TalonFXMotor: Failed to refresh slot 2 PID config (Status: " + refreshStatus2 +
                        "). Configuration may be factory defaulted!", true);
                }

                slot2Config.withKP(kP).withKI(kI).withKD(kD).withKV(kF);

                boolean success2 = applyConfigWithRetry(() -> motor.getConfigurator().apply(slot2Config, slotIdx));
                if (!success2) {
                    edu.wpi.first.wpilibj.DriverStation.reportError(
                        "TalonFXMotor: Failed to apply slot 2 PID after retries", false);
                }
                break;

            default:
                throw new IllegalArgumentException("Invalid slot index. Must be between 0 and 3.");
        }
    }

    /**
     * Configures Motion Magic motion profiling parameters.
     *
     * <p>Motion Magic generates smooth trapezoidal motion profiles for position control.
     *
     * @param cruiseVelocity Maximum velocity during motion
     * @param acceleration Acceleration rate
     * @param jerkRPSPerSecPerSec Jerk (rate of acceleration change) in rotations per second³
     */
    @Override
    public void configureMotionMagic(AngularVelocity cruiseVelocity, AngularAcceleration acceleration, double jerkRPSPerSecPerSec) {
        var config = new MotionMagicConfigs();

        // CRITICAL: Refresh before apply to avoid factory defaulting other config fields
        StatusCode refreshStatus = motor.getConfigurator().refresh(config);
        if (!refreshStatus.isOK()) {
            edu.wpi.first.wpilibj.DriverStation.reportWarning(
                "TalonFXMotor: Failed to refresh Motion Magic config (Status: " + refreshStatus +
                "). Configuration may be factory defaulted!", true);
        }

        config.withMotionMagicCruiseVelocity(cruiseVelocity.in(RotationsPerSecond))
                .withMotionMagicAcceleration(acceleration.in(RotationsPerSecondPerSecond))
                .withMotionMagicJerk(jerkRPSPerSecPerSec);

        boolean success = applyConfigWithRetry(() -> motor.getConfigurator().apply(config));
        if (!success) {
            edu.wpi.first.wpilibj.DriverStation.reportError(
                "TalonFXMotor: Failed to apply Motion Magic config after retries", false);
        }
    }

    /**
     * Configures current limiting to protect the motor and battery.
     *
     * @param stallLimit Stator current limit when motor is under heavy load
     * @param freeLimit Supply current limit when motor is spinning freely
     * @param limitRpmThreshold RPM threshold below which stall limit applies (not used by TalonFX)
     */
    @Override
    public void configureCurrentLimits(Current stallLimit, Current freeLimit, double limitRpmThreshold) {
        var config = new CurrentLimitsConfigs();

        // CRITICAL: Refresh before apply to avoid factory defaulting other config fields
        StatusCode refreshStatus = motor.getConfigurator().refresh(config);
        if (!refreshStatus.isOK()) {
            edu.wpi.first.wpilibj.DriverStation.reportWarning(
                "TalonFXMotor: Failed to refresh current limits config (Status: " + refreshStatus +
                "). Configuration may be factory defaulted!", true);
        }

        config.withStatorCurrentLimit(stallLimit.in(Amps))
                .withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLimit(freeLimit.in(Amps))
                .withSupplyCurrentLimitEnable(true);

        boolean success = applyConfigWithRetry(() -> motor.getConfigurator().apply(config));
        if (!success) {
            edu.wpi.first.wpilibj.DriverStation.reportError(
                "TalonFXMotor: Failed to apply current limits after retries", false);
        }
    }

    /**
     * Configures software position limits to prevent mechanism damage.
     *
     * @param forwardLimitRotations Forward soft limit position in rotations from zero
     * @param reverseLimitRotations Reverse soft limit position in rotations from zero
     * @param enable True to enable soft limits, false to disable
     */
    @Override
    public void configureSoftLimits(double forwardLimitRotations, double reverseLimitRotations, boolean enable) {
        var config = new SoftwareLimitSwitchConfigs();

        // CRITICAL: Refresh before apply to avoid factory defaulting other config fields
        StatusCode refreshStatus = motor.getConfigurator().refresh(config);
        if (!refreshStatus.isOK()) {
            edu.wpi.first.wpilibj.DriverStation.reportWarning(
                "TalonFXMotor: Failed to refresh soft limits config (Status: " + refreshStatus +
                "). Configuration may be factory defaulted!", true);
        }

        config.withForwardSoftLimitThreshold(forwardLimitRotations)
                .withReverseSoftLimitThreshold(reverseLimitRotations)
                .withForwardSoftLimitEnable(enable)
                .withReverseSoftLimitEnable(enable);

        boolean success = applyConfigWithRetry(() -> motor.getConfigurator().apply(config));
        if (!success) {
            edu.wpi.first.wpilibj.DriverStation.reportError(
                "TalonFXMotor: Failed to apply soft limits after retries", false);
        }
    }

    /**
     * Enables or disables software limit switches for both forward and reverse
     * directions.
     * Software limits prevent the motor from moving beyond specified positions.
     * 
     * @param enable true to enable software limits, false to disable them
     */
    @Override
    public void enableSoftLimits(boolean enable) {
        var config = new SoftwareLimitSwitchConfigs();

        // CRITICAL: Refresh before apply to avoid factory defaulting other config fields
        StatusCode refreshStatus = motor.getConfigurator().refresh(config);
        if (!refreshStatus.isOK()) {
            edu.wpi.first.wpilibj.DriverStation.reportWarning(
                "TalonFXMotor: Failed to refresh soft limits enable config (Status: " + refreshStatus +
                "). Configuration may be factory defaulted!", true);
        }

        config.withForwardSoftLimitEnable(enable)
                .withReverseSoftLimitEnable(enable);

        boolean success = applyConfigWithRetry(() -> motor.getConfigurator().apply(config));
        if (!success) {
            edu.wpi.first.wpilibj.DriverStation.reportError(
                "TalonFXMotor: Failed to apply soft limits enable after retries", false);
        }
    }

    /**
     * Sets the inversion state of the motor.
     * 
     * @param inverted true to invert the motor (Clockwise is positive),
     *                 false for normal operation (CounterClockwise is positive)
     */
    @Override
    public void setInverted(boolean inverted) {
        var config = new MotorOutputConfigs();

        // CRITICAL: Refresh before apply to avoid factory defaulting other config fields
        StatusCode refreshStatus = motor.getConfigurator().refresh(config);
        if (!refreshStatus.isOK()) {
            edu.wpi.first.wpilibj.DriverStation.reportWarning(
                "TalonFXMotor: Failed to refresh motor output config (Status: " + refreshStatus +
                "). Configuration may be factory defaulted!", true);
        }

        config.withInverted(inverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive);

        boolean success = applyConfigWithRetry(() -> motor.getConfigurator().apply(config));
        if (!success) {
            edu.wpi.first.wpilibj.DriverStation.reportError(
                "TalonFXMotor: Failed to apply motor inversion after retries", false);
        }
    }

    /**
     * Sets the neutral mode of the motor to either brake or coast.
     * In brake mode, the motor actively resists motion when not driven.
     * In coast mode, the motor spins freely when not driven.
     *
     * @param brake true to enable brake mode, false for coast mode
     */
    @Override
    public void setBrakeMode(boolean brake) {
        var config = new MotorOutputConfigs();

        // CRITICAL: Refresh before apply to avoid factory defaulting other config fields
        StatusCode refreshStatus = motor.getConfigurator().refresh(config);
        if (!refreshStatus.isOK()) {
            edu.wpi.first.wpilibj.DriverStation.reportWarning(
                "TalonFXMotor: Failed to refresh brake mode config (Status: " + refreshStatus +
                "). Configuration may be factory defaulted!", true);
        }

        config.withNeutralMode(brake ? NeutralModeValue.Brake : NeutralModeValue.Coast);

        boolean success = applyConfigWithRetry(() -> motor.getConfigurator().apply(config));
        if (!success) {
            edu.wpi.first.wpilibj.DriverStation.reportError(
                "TalonFXMotor: Failed to apply brake mode after retries", false);
        }
    }

    /**
     * Sets the position of the motor in rotations.
     * 
     * @param rotations The target position in rotations for the motor to move to
     */
    @Override
    public void setPosition(double rotations) {
        motor.setPosition(rotations);
        // set(ControlMode.POSITION, rotations);
    }

    /**
     * Set Position with an Arbitrary Feed Forward
     * 
     * @param activePidSlot
     * @param rotations
     * @param arbFeedForward
     */
    public void setPositionWithArbFeedForward(double rotations, double arbFeedForward) {
        int activePidSlot = 0;
        motor.setControl(new PositionDutyCycle(rotations)
                .withSlot(activePidSlot)
                .withFeedForward(arbFeedForward));
    }

    /**
     * Enables voltage compensation for the motor.
     * This helps maintain consistent motor performance regardless of battery
     * voltage fluctuations.
     *
     * @param nominalVoltage The voltage to compensate to.
     *                The motor will scale its output to maintain consistent
     *                behavior at this voltage level.
     */
    @Override
    public void enableVoltageCompensation(Voltage nominalVoltage) {
        // NOTE: This method is deprecated and does NOT provide proper voltage compensation in Phoenix 6.
        // Setting PeakForwardVoltage/PeakReverseVoltage just caps the voltage output,
        // it does NOT compensate for battery droop like WPILib voltage compensation does.
        //
        // CORRECT APPROACH for Phoenix 6:
        // - Use voltage-based control modes (VoltageOut, PositionVoltage, VelocityVoltage, MotionMagicVoltage)
        // - FOC control modes (MotionMagicFOCTorque, PositionFOCTorque, VelocityFOCTorque) also use voltage
        // - These explicitly request a voltage output and handle compensation internally
        // - Duty cycle based controls (DutyCycleOut, DutyCycleFOC) can exceed the peak voltage cap
        //
        // This method is retained for interface compliance but logs a warning.

        edu.wpi.first.wpilibj.DriverStation.reportWarning(
            "TalonFXMotor.enableVoltageCompensation() does NOT work as expected in Phoenix 6. " +
            "Use voltage-based control modes (VoltageOut, PositionVoltage, VelocityVoltage, MotionMagicVoltage) " +
            "instead of duty cycle control modes. See Phoenix 6 documentation.", false);

        // DO NOT apply configuration - this would incorrectly cap voltage
    }

    /**
     * Gets the current position of the motor.
     * 
     * @return The current position of the motor in rotations (double)
     */
    @Override
    public double getPosition() {
        return motor.getPosition().getValueAsDouble();
    }

    /**
     * Gets the current velocity of the motor.
     *
     * @return The current velocity
     */
    @Override
    public AngularVelocity getVelocity() {
        return RotationsPerSecond.of(motor.getVelocity().getValueAsDouble());
    }

    /**
     * Gets the current acceleration of the motor.
     *
     * @return The current acceleration
     */
    @Override
    public AngularAcceleration getAcceleration() {
        return RotationsPerSecondPerSecond.of(motor.getAcceleration().getValueAsDouble());
    }

    /**
     * Gets the current draw of the motor.
     *
     * @return The current draw
     */
    @Override
    public Current getCurrentDraw() {
        return Amps.of(motor.getStatorCurrent().getValueAsDouble());
    }

    /**
     * Gets the current output percentage of the motor.
     * 
     * @return The current output percentage as a double between -1.0 and 1.0
     */
    @Override
    public double getOutputPercent() {
        return motor.getDutyCycle().getValueAsDouble();
    }

    /**
     * Gets the current temperature of the motor.
     * 
     * @return The current temperature in degrees Celsius
     */
    @Override
    public double getTemperature() {
        return motor.getDeviceTemp().getValueAsDouble();
    }

    /**
     * Gets the state of the forward limit switch.
     * 
     * @return True if the forward limit switch is closed, false otherwise
     */
    @Override
    public boolean getForwardLimitSwitch() {
        // return motor.getForwardLimit().getValueAsDouble() == 1;
        return motor.getForwardLimit().getValue() == ForwardLimitValue.ClosedToGround;
    }

    /**
     * Gets the state of the reverse limit switch.
     * 
     * @return True if the reverse limit switch is closed, false otherwise
     */
    @Override
    public boolean getReverseLimitSwitch() {
        // return motor.getReverseLimit().getValueAsDouble() == 1;
        return motor.getReverseLimit().getValue() == ReverseLimitValue.ClosedToGround;
    }

    /**
     * Sets this motor as a strict follower of another motor controller.
     * 
     * @param deviceID The ID of the motor controller to follow
     */
    @Override
    public void setStrictFollower(int deviceID) {
        // Set this motor as a follower using the Follower control request
        motor.setControl(new com.ctre.phoenix6.controls.Follower(deviceID, MotorAlignmentValue.Aligned)); // Aligned = don't oppose master
                                                                                    // direction
    }

    /**
     * Configures hard limits for the TalonFX motor.
     * 
     * @param enableForward True to enable the forward hard limit, false otherwise
     * @param enableReverse True to enable the reverse hard limit, false otherwise
     */
    @Override
    public void configureHardLimits(boolean enableForward, boolean enableReverse, double forwardValue,
            double reverseValue) {
        var limitSwitchConfigs = new HardwareLimitSwitchConfigs();

        // CRITICAL: Refresh before apply to avoid factory defaulting other config fields
        StatusCode refreshStatus = motor.getConfigurator().refresh(limitSwitchConfigs);
        if (!refreshStatus.isOK()) {
            edu.wpi.first.wpilibj.DriverStation.reportWarning(
                "TalonFXMotor: Failed to refresh hard limits config (Status: " + refreshStatus +
                "). Configuration may be factory defaulted!", true);
        }

        limitSwitchConfigs.withForwardLimitEnable(enableForward)
                .withForwardLimitAutosetPositionEnable(enableForward)
                .withForwardLimitAutosetPositionValue(forwardValue)
                .withForwardLimitType(ForwardLimitTypeValue.NormallyOpen)
                .withReverseLimitEnable(enableReverse)
                .withReverseLimitAutosetPositionEnable(enableReverse)
                .withReverseLimitAutosetPositionValue(reverseValue)
                .withReverseLimitType(ReverseLimitTypeValue.NormallyOpen);

        boolean success = applyConfigWithRetry(() -> motor.getConfigurator().apply(limitSwitchConfigs));
        if (!success) {
            edu.wpi.first.wpilibj.DriverStation.reportError(
                "TalonFXMotor: Failed to apply hard limits after retries", false);
        }

        // For simulation/feedback
        motor.getForwardLimit().setUpdateFrequency(50);
        motor.getReverseLimit().setUpdateFrequency(50);

        // Set up simulation state
        var simState = motor.getSimState();
        simState.setForwardLimit(enableForward); // Set the initial state
        simState.setReverseLimit(enableReverse);
    }

    /**
     * Applies a configuration with automatic retries.
     *
     * <p>This method attempts to apply a configuration multiple times if initial attempts fail.
     * Motor controllers sometimes reject configurations during startup or high CAN bus traffic.
     *
     * @param applyAction The configuration action to apply
     * @return True if configuration was applied successfully, false after all retries exhausted
     */
    public boolean applyConfigWithRetry(ConfigApplyAction applyAction) {
        for (int i = 0; i < maxRetries; i++) {
            StatusCode status = applyAction.apply();

            if (status.isOK()) {
                if (i > 0) {
                    // Only log if we had to retry
                    edu.wpi.first.wpilibj.DataLogManager.log(
                        "TalonFX configuration succeeded on attempt " + (i + 1));
                }
                return true; // Configuration successful
            } else if (i < maxRetries - 1) {
                // Not the last attempt - log retry
                edu.wpi.first.wpilibj.DataLogManager.log(
                    "TalonFX configuration failed (Status: " + status + "), retrying... (" +
                    (i + 1) + "/" + maxRetries + ")");
                // No Thread.sleep - retry immediately
            }
        }

        // All retries exhausted - report warning to driver station
        edu.wpi.first.wpilibj.DriverStation.reportWarning(
            "TalonFX configuration failed after " + maxRetries + " attempts", false);
        return false; // Configuration failed after retries
    }

    @Override
    public boolean supportsControlMode(ControlMode mode) {
        // TalonFXMotor supports all modes, but CURRENT only works with Kraken
        if (mode == ControlMode.CURRENT && !isKraken) {
            return false;
        }
        return true;
    }

    @Override
    public String getMotorType() {
        return isKraken ? "TalonFXMotor (Kraken X60)" : "TalonFXMotor (Falcon 500)";
    }
}