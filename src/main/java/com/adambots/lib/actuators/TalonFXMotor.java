package com.adambots.lib.actuators;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
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
@Logged
public class TalonFXMotor implements BaseMotor {
    @NotLogged
    private final TalonFX motor;

    @NotLogged
    private final boolean isKraken;

    @NotLogged
    private boolean focFlag = false;

    @NotLogged
    private double feedForward = 0.0;

    @NotLogged
    private boolean isInverted = false; // Track inversion state for follower mode

    @NotLogged
    private final int maxRetries = 3; // Maximum retries for configuration

    // Reusable control request objects to avoid allocation overhead
    @NotLogged
    private final VoltageOut voltageRequest = new VoltageOut(0);

    @NotLogged
    private final PositionVoltage positionVoltageRequest = new PositionVoltage(0);

    @NotLogged
    private final VelocityVoltage velocityVoltageRequest = new VelocityVoltage(0);

    @NotLogged
    private final MotionMagicVoltage motionMagicVoltageRequest = new MotionMagicVoltage(0);

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

        currentLimits.SupplyCurrentLimit = supplyCurrentLimit;
        currentLimits.SupplyCurrentLimitEnable = true;

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
                // Use Voltage-based control for better precision (Phoenix 6 recommendation)
                if (feedForward != 0) {
                    motor.setControl(positionVoltageRequest.withPosition(value)
                            .withSlot(0).withEnableFOC(focFlag).withFeedForward(feedForward));
                } else {
                    motor.setControl(positionVoltageRequest.withPosition(value)
                            .withSlot(0).withEnableFOC(focFlag));
                }
                break;
            case VELOCITY:
                // Use Voltage-based control for better precision (Phoenix 6 recommendation)
                if (feedForward != 0) {
                    motor.setControl(velocityVoltageRequest.withVelocity(value)
                            .withSlot(0).withEnableFOC(focFlag).withFeedForward(feedForward));
                } else {
                    motor.setControl(velocityVoltageRequest.withVelocity(value)
                            .withSlot(0).withEnableFOC(focFlag));
                }
                break;
            case VOLTAGE:
                // Use VoltageOut for proper voltage control, not DutyCycleOut
                motor.setControl(voltageRequest.withOutput(value).withEnableFOC(focFlag));
                break;
            case CURRENT:
                // TorqueCurrentFOC works on all TalonFX (requires Phoenix Pro license)
                motor.setControl(new TorqueCurrentFOC(value));
                break;
            case MOTION_MAGIC:
                // Use Voltage-based control for better precision (Phoenix 6 recommendation)
                if (feedForward != 0) {
                    motor.setControl(motionMagicVoltageRequest.withPosition(value)
                            .withSlot(0).withEnableFOC(focFlag).withFeedForward(feedForward));
                } else {
                    motor.setControl(motionMagicVoltageRequest.withPosition(value)
                            .withSlot(0).withEnableFOC(focFlag));
                }
                break;
            case MOTION_MAGIC_FOC_TORQUE:
                motor.setControl(new MotionMagicTorqueCurrentFOC(value).withSlot(0).withFeedForward(feedForward));
                break;
            case FOLLOWER:
                // Follow another Talon FX controller, respecting isInverted flag
                int deviceID = (int) value;
                motor.setControl(new com.ctre.phoenix6.controls.Follower(deviceID,
                        isInverted ? MotorAlignmentValue.Opposed : MotorAlignmentValue.Aligned));
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

                slot0Config.kP = kP;
                slot0Config.kI = kI;
                slot0Config.kD = kD;
                slot0Config.kV = kF;

                boolean success0 = applyConfigWithRetry(() -> motor.getConfigurator().apply(slot0Config));
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

                slot1Config.kP = kP;
                slot1Config.kI = kI;
                slot1Config.kD = kD;
                slot1Config.kV = kF;

                boolean success1 = applyConfigWithRetry(() -> motor.getConfigurator().apply(slot1Config));
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

                slot2Config.kP = kP;
                slot2Config.kI = kI;
                slot2Config.kD = kD;
                slot2Config.kV = kF;

                boolean success2 = applyConfigWithRetry(() -> motor.getConfigurator().apply(slot2Config));
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
     * Sets PID and feedforward gains with full Phoenix 6 support.
     *
     * <p>Phoenix 6 provides separate feedforward gains for different control scenarios:
     * <ul>
     *   <li>kS - Static feedforward to overcome friction</li>
     *   <li>kV - Velocity feedforward (output per unit velocity)</li>
     *   <li>kA - Acceleration feedforward (output per unit acceleration)</li>
     *   <li>kG - Gravity feedforward (for elevator/arm mechanisms)</li>
     * </ul>
     *
     * @param slotIdx The PID slot index to configure (0-2)
     * @param kP Proportional gain
     * @param kI Integral gain
     * @param kD Derivative gain
     * @param kV Velocity feedforward (output per unit velocity)
     * @param kS Static feedforward (overcomes friction)
     * @param kA Acceleration feedforward
     * @param kG Gravity feedforward
     * @throws IllegalArgumentException if slotIdx is not between 0-2
     */
    public void setPID(int slotIdx, double kP, double kI, double kD,
                       double kV, double kS, double kA, double kG) {
        switch (slotIdx) {
            case 0:
                var slot0Config = new Slot0Configs();

                StatusCode refreshStatus0 = motor.getConfigurator().refresh(slot0Config);
                if (!refreshStatus0.isOK()) {
                    edu.wpi.first.wpilibj.DriverStation.reportWarning(
                        "TalonFXMotor: Failed to refresh slot 0 PID config (Status: " + refreshStatus0 +
                        "). Configuration may be factory defaulted!", true);
                }

                slot0Config.kP = kP;
                slot0Config.kI = kI;
                slot0Config.kD = kD;
                slot0Config.kV = kV;
                slot0Config.kS = kS;
                slot0Config.kA = kA;
                slot0Config.kG = kG;

                boolean success0 = applyConfigWithRetry(() -> motor.getConfigurator().apply(slot0Config));
                if (!success0) {
                    edu.wpi.first.wpilibj.DriverStation.reportError(
                        "TalonFXMotor: Failed to apply slot 0 PID after retries", false);
                }
                break;

            case 1:
                var slot1Config = new Slot1Configs();

                StatusCode refreshStatus1 = motor.getConfigurator().refresh(slot1Config);
                if (!refreshStatus1.isOK()) {
                    edu.wpi.first.wpilibj.DriverStation.reportWarning(
                        "TalonFXMotor: Failed to refresh slot 1 PID config (Status: " + refreshStatus1 +
                        "). Configuration may be factory defaulted!", true);
                }

                slot1Config.kP = kP;
                slot1Config.kI = kI;
                slot1Config.kD = kD;
                slot1Config.kV = kV;
                slot1Config.kS = kS;
                slot1Config.kA = kA;
                slot1Config.kG = kG;

                boolean success1 = applyConfigWithRetry(() -> motor.getConfigurator().apply(slot1Config));
                if (!success1) {
                    edu.wpi.first.wpilibj.DriverStation.reportError(
                        "TalonFXMotor: Failed to apply slot 1 PID after retries", false);
                }
                break;

            case 2:
                var slot2Config = new Slot2Configs();

                StatusCode refreshStatus2 = motor.getConfigurator().refresh(slot2Config);
                if (!refreshStatus2.isOK()) {
                    edu.wpi.first.wpilibj.DriverStation.reportWarning(
                        "TalonFXMotor: Failed to refresh slot 2 PID config (Status: " + refreshStatus2 +
                        "). Configuration may be factory defaulted!", true);
                }

                slot2Config.kP = kP;
                slot2Config.kI = kI;
                slot2Config.kD = kD;
                slot2Config.kV = kV;
                slot2Config.kS = kS;
                slot2Config.kA = kA;
                slot2Config.kG = kG;

                boolean success2 = applyConfigWithRetry(() -> motor.getConfigurator().apply(slot2Config));
                if (!success2) {
                    edu.wpi.first.wpilibj.DriverStation.reportError(
                        "TalonFXMotor: Failed to apply slot 2 PID after retries", false);
                }
                break;

            default:
                throw new IllegalArgumentException("Invalid slot index. Must be between 0 and 2.");
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

        config.MotionMagicCruiseVelocity = cruiseVelocity.in(RotationsPerSecond);
        config.MotionMagicAcceleration = acceleration.in(RotationsPerSecondPerSecond);
        config.MotionMagicJerk = jerkRPSPerSecPerSec;

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

        config.StatorCurrentLimit = stallLimit.in(Amps);
        config.StatorCurrentLimitEnable = true;
        config.SupplyCurrentLimit = freeLimit.in(Amps);
        config.SupplyCurrentLimitEnable = true;

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

        config.ForwardSoftLimitThreshold = forwardLimitRotations;
        config.ReverseSoftLimitThreshold = reverseLimitRotations;
        config.ForwardSoftLimitEnable = enable;
        config.ReverseSoftLimitEnable = enable;

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

        config.ForwardSoftLimitEnable = enable;
        config.ReverseSoftLimitEnable = enable;

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
        this.isInverted = inverted;
        var config = new MotorOutputConfigs();

        // CRITICAL: Refresh before apply to avoid factory defaulting other config fields
        StatusCode refreshStatus = motor.getConfigurator().refresh(config);
        if (!refreshStatus.isOK()) {
            edu.wpi.first.wpilibj.DriverStation.reportWarning(
                "TalonFXMotor: Failed to refresh motor output config (Status: " + refreshStatus +
                "). Configuration may be factory defaulted!", true);
        }

        config.Inverted = inverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;

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

        config.NeutralMode = brake ? NeutralModeValue.Brake : NeutralModeValue.Coast;

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
        motor.setControl(new PositionVoltage(rotations)
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
     * Uses the inversion state set via {@link #setInverted(boolean)} to determine alignment.
     *
     * <p>Note: In follower mode, the motor's own inversion config is ignored. The alignment
     * is determined by the {@code isInverted} flag: if true, uses Opposed alignment to make
     * the follower spin opposite to the leader's output.
     *
     * <p>For explicit control over alignment, use {@link #setStrictFollower(int, boolean)}.
     *
     * @param deviceID The CAN ID of the motor controller to follow
     */
    @Override
    public void setStrictFollower(int deviceID) {
        setStrictFollower(deviceID, isInverted);
    }

    /**
     * Sets this motor as a strict follower of another motor controller with explicit alignment control.
     *
     * <p>Use this when you need direct control over whether the follower matches or opposes
     * the leader's output direction. This is useful for mechanisms like shooters where motors
     * face each other and need to spin opposite directions.
     *
     * <p>Example:
     * <pre>
     * // Shooter with motors facing each other
     * leftMotor.set(ControlMode.PERCENT_OUTPUT, 0.5);  // leader
     * rightMotor.setStrictFollower(leftMotorId, true); // oppose leader
     * </pre>
     *
     * @param deviceID The CAN ID of the motor controller to follow
     * @param opposeMaster true to spin opposite to the leader (Opposed),
     *                     false to spin same direction as leader (Aligned)
     */
    @Override
    public void setStrictFollower(int deviceID, boolean opposeMaster) {
        // Set this motor as a follower using the Follower control request
        // Follower control ignores motor inversion config, so we must use MotorAlignment
        MotorAlignmentValue alignment = opposeMaster ? MotorAlignmentValue.Opposed : MotorAlignmentValue.Aligned;
        motor.setControl(new com.ctre.phoenix6.controls.Follower(deviceID, alignment));
    }

    /**
     * Configures hardware limit switches for the TalonFX motor.
     *
     * <p><strong>Note:</strong> Kraken X60 motors do not support hardware limit switches.
     * For Kraken, use software limits via {@link #configureSoftLimits} or a CANcoder
     * as a remote limit switch. Falcon 500 motors support hardware limit switches
     * via the 4-pin JST connector on the TalonFX controller.
     *
     * @param enableForward True to enable the forward hard limit, false otherwise
     * @param enableReverse True to enable the reverse hard limit, false otherwise
     * @param forwardValue Position value to auto-set when forward limit is triggered
     * @param reverseValue Position value to auto-set when reverse limit is triggered
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

        limitSwitchConfigs.ForwardLimitEnable = enableForward;
        limitSwitchConfigs.ForwardLimitAutosetPositionEnable = enableForward;
        limitSwitchConfigs.ForwardLimitAutosetPositionValue = forwardValue;
        limitSwitchConfigs.ForwardLimitType = ForwardLimitTypeValue.NormallyOpen;
        limitSwitchConfigs.ReverseLimitEnable = enableReverse;
        limitSwitchConfigs.ReverseLimitAutosetPositionEnable = enableReverse;
        limitSwitchConfigs.ReverseLimitAutosetPositionValue = reverseValue;
        limitSwitchConfigs.ReverseLimitType = ReverseLimitTypeValue.NormallyOpen;

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

    @Override
    public void configureGravity(GravityType type) {
        var slot0Configs = new Slot0Configs();

        StatusCode refreshStatus = motor.getConfigurator().refresh(slot0Configs);
        if (!refreshStatus.isOK()) {
            edu.wpi.first.wpilibj.DriverStation.reportWarning(
                "TalonFXMotor: Failed to refresh config before configureGravity (Status: " + refreshStatus +
                "). Configuration may be factory defaulted!", true);
        }

        switch (type) {
            case ARM_COSINE:
                slot0Configs.GravityType = GravityTypeValue.Arm_Cosine;
                break;
            case ELEVATOR_STATIC:
                slot0Configs.GravityType = GravityTypeValue.Elevator_Static;
                break;
            case NONE:
            default:
                slot0Configs.kG = 0;
                break;
        }

        boolean success = applyConfigWithRetry(() -> motor.getConfigurator().apply(slot0Configs));
        if (!success) {
            edu.wpi.first.wpilibj.DriverStation.reportError(
                "TalonFXMotor: Failed to apply gravity configuration after retries", false);
        }
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
        // TalonFXMotor supports all modes (CURRENT requires Phoenix Pro license)
        return true;
    }

    @Override
    public String getMotorType() {
        return isKraken ? "TalonFXMotor (Kraken X60)" : "TalonFXMotor (Falcon 500)";
    }
}