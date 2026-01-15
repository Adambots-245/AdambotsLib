package com.adambots.lib.actuators;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.ForwardLimitValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.ReverseLimitValue;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.HardwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXSConfigurator;
import com.ctre.phoenix6.configs.VoltageConfigs;

/**
 * Implementation of BaseMotor for Minion Motor using TalonFXS motor controller.
 * Note: Applying the configuration is a blocking call and may take some time.
 * This implementation includes retry logic for configuration application.
 * Ensure that any configuration setting is done only once during initialization
 * to avoid unnecessary delays during operation.
 */
public class MinionMotor implements BaseMotor {
    private final TalonFXS motor;
    private final TalonFXSConfigurator configurator;
    private final MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0);
    private final PositionVoltage positionRequest = new PositionVoltage(0);
    private final VelocityVoltage velocityRequest = new VelocityVoltage(0);
    private final VoltageOut voltageRequest = new VoltageOut(0);
    private final DutyCycleOut percentRequest = new DutyCycleOut(0);
    private final int maxRetries = 3; // Maximum retries for configuration
    
    /**
     * Functional interface for applying a configuration and returning a StatusCode.
     */
    @FunctionalInterface
    interface ConfigApplyAction {
        StatusCode apply();
    }

    /**
     * Constructor for MinionMotor using TalonFXS
     * 
     * @param deviceID The CAN ID of the motor controller
     */
    public MinionMotor(int deviceID) {
        motor = new TalonFXS(deviceID);
        configurator = motor.getConfigurator();
    }
    
    /**
     * Constructor for MinionMotor using TalonFXS with specific CAN bus
     *
     * @param deviceID The CAN ID of the motor controller
     * @param canBus The name of the CAN bus
     */
    public MinionMotor(int deviceID, String canBus) {
        motor = new TalonFXS(deviceID, new CANBus(canBus));
        configurator = motor.getConfigurator();
    }

    @Override
    public void set(ControlMode mode, double value) {
        switch (mode) {
            case PERCENT_OUTPUT:
                motor.setControl(percentRequest.withOutput(value));
                break;
            case POSITION:
                motor.setControl(positionRequest.withPosition(value));
                break;
            case VELOCITY:
                motor.setControl(velocityRequest.withVelocity(value));
                break;
            case VOLTAGE:
                motor.setControl(voltageRequest.withOutput(value));
                break;
            case CURRENT:
                // TalonFXS doesn't have a direct current control mode - fallback to percent output
                edu.wpi.first.wpilibj.DriverStation.reportWarning(
                    "MinionMotor: CURRENT mode not supported. Falling back to PERCENT_OUTPUT.", false);
                motor.setControl(percentRequest.withOutput(value));
                break;
            case MOTION_MAGIC:
                motor.setControl(motionMagicRequest.withPosition(value));
                break;
            case FOLLOWER:
                // In Phoenix 6, Follower requires device ID
                // value here is assumed to be the device ID to follow
                motor.setControl(new Follower((int)value, MotorAlignmentValue.Aligned));
                break;
            case MOTION_MAGIC_FOC_TORQUE:
                // TalonFXS doesn't support FOC torque mode - fallback to regular Motion Magic
                edu.wpi.first.wpilibj.DriverStation.reportWarning(
                    "MinionMotor: MOTION_MAGIC_FOC_TORQUE not supported. Falling back to MOTION_MAGIC.", false);
                motor.setControl(motionMagicRequest.withPosition(value));
                break;
            default:
                throw new IllegalArgumentException("Unsupported control mode: " + mode);
        }
    }

    @Override
    public void setPID(int slotIdx, double kP, double kI, double kD, double kF) {
        // In Phoenix 6, slot configuration is done differently
        Slot0Configs slot0Configs = new Slot0Configs();

        // CRITICAL: Check if refresh fails - failed refresh causes config apply to factory default
        StatusCode refreshStatus = configurator.refresh(slot0Configs);
        if (!refreshStatus.isOK()) {
            edu.wpi.first.wpilibj.DriverStation.reportWarning(
                "MinionMotor: Failed to refresh config before setPID (Status: " + refreshStatus +
                "). Configuration may be factory defaulted!", true);
            // Continue anyway to attempt configuration, but user is warned
        }

        // Update PID values
        slot0Configs.kP = kP;
        slot0Configs.kI = kI;
        slot0Configs.kD = kD;
        slot0Configs.kV = kF;  // kV is the feed forward in Phoenix 6

        // Apply configuration - check return value
        boolean success = applyConfigWithRetry(() -> configurator.apply(slot0Configs));
        if (!success) {
            edu.wpi.first.wpilibj.DriverStation.reportError(
                "MinionMotor: Failed to apply PID configuration after retries", false);
        }
    }

    @Override
    public void configureMotionMagic(double cruiseVelocityRPS, double accelerationRPSPerSec, double jerkRPSPerSecPerSec) {
        MotionMagicConfigs configs = new MotionMagicConfigs();

        // CRITICAL: Check if refresh fails
        StatusCode refreshStatus = configurator.refresh(configs);
        if (!refreshStatus.isOK()) {
            edu.wpi.first.wpilibj.DriverStation.reportWarning(
                "MinionMotor: Failed to refresh config before configureMotionMagic (Status: " + refreshStatus +
                "). Configuration may be factory defaulted!", true);
        }

        // Set new values
        configs.MotionMagicCruiseVelocity = cruiseVelocityRPS;
        configs.MotionMagicAcceleration = accelerationRPSPerSec;
        configs.MotionMagicJerk = jerkRPSPerSecPerSec;

        // Apply configuration - check return value
        boolean success = applyConfigWithRetry(() -> configurator.apply(configs));
        if (!success) {
            edu.wpi.first.wpilibj.DriverStation.reportError(
                "MinionMotor: Failed to apply Motion Magic configuration after retries", false);
        }
    }

    @Override
    public void configureCurrentLimits(double stallLimitAmps, double freeLimitAmps, double limitRpmThreshold) {
        CurrentLimitsConfigs configs = new CurrentLimitsConfigs();

        // CRITICAL: Check if refresh fails
        StatusCode refreshStatus = configurator.refresh(configs);
        if (!refreshStatus.isOK()) {
            edu.wpi.first.wpilibj.DriverStation.reportWarning(
                "MinionMotor: Failed to refresh config before configureCurrentLimits (Status: " + refreshStatus +
                "). Configuration may be factory defaulted!", true);
        }

        // Set new values - note that Phoenix 6 has different terminology
        configs.StatorCurrentLimit = stallLimitAmps;
        configs.StatorCurrentLimitEnable = true;
        configs.SupplyCurrentLimit = freeLimitAmps;
        configs.SupplyCurrentLimitEnable = true;
        configs.SupplyCurrentLowerLimit = freeLimitAmps * 0.8; // Lower limit after time threshold
        configs.SupplyCurrentLowerTime = 0.1; // Time to exceed threshold before lowering limit

        // Apply configuration - check return value
        boolean success = applyConfigWithRetry(() -> configurator.apply(configs));
        if (!success) {
            edu.wpi.first.wpilibj.DriverStation.reportError(
                "MinionMotor: Failed to apply current limit configuration after retries", false);
        }
    }

    @Override
    public void configureSoftLimits(double forwardLimitRotations, double reverseLimitRotations, boolean enable) {
        SoftwareLimitSwitchConfigs configs = new SoftwareLimitSwitchConfigs();

        // CRITICAL: Check if refresh fails
        StatusCode refreshStatus = configurator.refresh(configs);
        if (!refreshStatus.isOK()) {
            edu.wpi.first.wpilibj.DriverStation.reportWarning(
                "MinionMotor: Failed to refresh config before configureSoftLimits (Status: " + refreshStatus +
                "). Configuration may be factory defaulted!", true);
        }

        // Set new values
        configs.ForwardSoftLimitThreshold = forwardLimitRotations;
        configs.ForwardSoftLimitEnable = enable;
        configs.ReverseSoftLimitThreshold = reverseLimitRotations;
        configs.ReverseSoftLimitEnable = enable;

        // Apply configuration - check return value
        boolean success = applyConfigWithRetry(() -> configurator.apply(configs));
        if (!success) {
            edu.wpi.first.wpilibj.DriverStation.reportError(
                "MinionMotor: Failed to apply soft limit configuration after retries", false);
        }
    }

    @Override
    public void enableSoftLimits(boolean enable) {
        SoftwareLimitSwitchConfigs configs = new SoftwareLimitSwitchConfigs();

        // CRITICAL: Check if refresh fails
        StatusCode refreshStatus = configurator.refresh(configs);
        if (!refreshStatus.isOK()) {
            edu.wpi.first.wpilibj.DriverStation.reportWarning(
                "MinionMotor: Failed to refresh config before enableSoftLimits (Status: " + refreshStatus +
                "). Configuration may be factory defaulted!", true);
        }

        // Update enable values only
        configs.ForwardSoftLimitEnable = enable;
        configs.ReverseSoftLimitEnable = enable;

        // Apply configuration - check return value
        boolean success = applyConfigWithRetry(() -> configurator.apply(configs));
        if (!success) {
            edu.wpi.first.wpilibj.DriverStation.reportError(
                "MinionMotor: Failed to apply soft limit enable configuration after retries", false);
        }
    }

    @Override
    public void setInverted(boolean inverted) {
        MotorOutputConfigs configs = new MotorOutputConfigs();

        // CRITICAL: Check if refresh fails
        StatusCode refreshStatus = configurator.refresh(configs);
        if (!refreshStatus.isOK()) {
            edu.wpi.first.wpilibj.DriverStation.reportWarning(
                "MinionMotor: Failed to refresh config before setInverted (Status: " + refreshStatus +
                "). Configuration may be factory defaulted!", true);
        }

        // Update inversion setting
        configs.Inverted = inverted ? com.ctre.phoenix6.signals.InvertedValue.Clockwise_Positive :
                                     com.ctre.phoenix6.signals.InvertedValue.CounterClockwise_Positive;

        // Apply configuration - check return value
        boolean success = applyConfigWithRetry(() -> configurator.apply(configs));
        if (!success) {
            edu.wpi.first.wpilibj.DriverStation.reportError(
                "MinionMotor: Failed to apply inversion configuration after retries", false);
        }
    }

    @Override
    public void setBrakeMode(boolean brake) {
        MotorOutputConfigs configs = new MotorOutputConfigs();

        // CRITICAL: Check if refresh fails
        StatusCode refreshStatus = configurator.refresh(configs);
        if (!refreshStatus.isOK()) {
            edu.wpi.first.wpilibj.DriverStation.reportWarning(
                "MinionMotor: Failed to refresh config before setBrakeMode (Status: " + refreshStatus +
                "). Configuration may be factory defaulted!", true);
        }

        // Update neutral mode setting
        configs.NeutralMode = brake ? com.ctre.phoenix6.signals.NeutralModeValue.Brake :
                                     com.ctre.phoenix6.signals.NeutralModeValue.Coast;

        // Apply configuration - check return value
        boolean success = applyConfigWithRetry(() -> configurator.apply(configs));
        if (!success) {
            edu.wpi.first.wpilibj.DriverStation.reportError(
                "MinionMotor: Failed to apply brake mode configuration after retries", false);
        }
    }

    @Override
    public void setPosition(double rotations) {
        // In Phoenix 6, we set the position through the configurator
        motor.setPosition(rotations);
    }

    @Override
    public void enableVoltageCompensation(double voltage) {
        // NOTE: This method is deprecated and does NOT provide proper voltage compensation in Phoenix 6.
        // Setting PeakForwardVoltage/PeakReverseVoltage just caps the voltage output,
        // it does NOT compensate for battery droop like WPILib voltage compensation does.
        //
        // CORRECT APPROACH for Phoenix 6:
        // - Use voltage-based control modes (VoltageOut, PositionVoltage, VelocityVoltage, MotionMagicVoltage)
        // - These explicitly request a voltage output and handle compensation internally
        // - Duty cycle based controls (DutyCycleOut) can exceed the peak voltage cap
        //
        // This method is retained for interface compliance but logs a warning.

        edu.wpi.first.wpilibj.DriverStation.reportWarning(
            "MinionMotor.enableVoltageCompensation() does NOT work as expected in Phoenix 6. " +
            "Use voltage-based control modes (VoltageOut, PositionVoltage, VelocityVoltage, MotionMagicVoltage) " +
            "instead of duty cycle control modes. See Phoenix 6 documentation.", false);

        // DO NOT apply configuration - this would incorrectly cap voltage
    }

    @Override
    public double getPosition() {
        // In Phoenix 6, getPosition returns a StatusSignal<Double> representing rotations
        var positionSignal = motor.getPosition();
        positionSignal.refresh(); // Make sure we have the latest value
        return positionSignal.getValue().magnitude(); // Value is in rotations which matches our interface
    }

    @Override
    public double getVelocity() {
        // In Phoenix 6, getVelocity returns rotations per second
        var velocitySignal = motor.getVelocity();
        velocitySignal.refresh();
        return velocitySignal.getValue().magnitude(); // Value is in RPS
    }

    @Override
    public double getAcceleration() {
        var accelerationSignal = motor.getAcceleration();
        accelerationSignal.refresh();
        return accelerationSignal.getValue().magnitude(); // Value is in RPS/s
    }

    @Override
    public double getCurrentDraw() {
        var currentSignal = motor.getStatorCurrent();
        currentSignal.refresh();
        return currentSignal.getValue().magnitude(); // Value is in amperes
    }

    @Override
    public double getOutputPercent() {
        var outputSignal = motor.getDutyCycle();
        outputSignal.refresh();
        return outputSignal.getValue().doubleValue(); // Value is from -1.0 to 1.0
    }

    @Override
    public double getTemperature() {
        var tempSignal = motor.getDeviceTemp();
        tempSignal.refresh();
        return tempSignal.getValue().magnitude(); // Value is in Celsius
    }

    @Override
    public boolean getForwardLimitSwitch() {
        var limitSignal = motor.getForwardLimit();
        limitSignal.refresh();
        return limitSignal.getValue().equals(ForwardLimitValue.ClosedToGround); // Returns true if switch is closed
    }

    @Override
    public boolean getReverseLimitSwitch() {
        var limitSignal = motor.getReverseLimit();
        limitSignal.refresh();
        return limitSignal.getValue().equals(ReverseLimitValue.ClosedToGround); // Returns true if switch is closed
    }

    @Override
    public void setStrictFollower(int deviceID) {
        // Set this motor to follow another device
        motor.setControl(new Follower(deviceID, MotorAlignmentValue.Aligned));
    }

    @Override
    public void configureHardLimits(boolean enableForward, boolean enableReverse,
                                   double forwardResetValueRotations, double reverseResetValueRotations) {
        HardwareLimitSwitchConfigs configs = new HardwareLimitSwitchConfigs();

        // CRITICAL: Check if refresh fails
        StatusCode refreshStatus = configurator.refresh(configs);
        if (!refreshStatus.isOK()) {
            edu.wpi.first.wpilibj.DriverStation.reportWarning(
                "MinionMotor: Failed to refresh config before configureHardLimits (Status: " + refreshStatus +
                "). Configuration may be factory defaulted!", true);
        }

        // Configure limits
        configs.ForwardLimitEnable = enableForward;
        configs.ForwardLimitAutosetPositionEnable = enableForward;
        configs.ForwardLimitAutosetPositionValue = forwardResetValueRotations;

        configs.ReverseLimitEnable = enableReverse;
        configs.ReverseLimitAutosetPositionEnable = enableReverse;
        configs.ReverseLimitAutosetPositionValue = reverseResetValueRotations;

        // Apply configuration - check return value
        boolean success = applyConfigWithRetry(() -> configurator.apply(configs));
        if (!success) {
            edu.wpi.first.wpilibj.DriverStation.reportError(
                "MinionMotor: Failed to apply hard limit configuration after retries", false);
        }
    }

    @Override
    public void set(double speed) {
        set(ControlMode.PERCENT_OUTPUT, speed);
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
                        "MinionMotor configuration succeeded on attempt " + (i + 1));
                }
                return true; // Configuration successful
            } else if (i < maxRetries - 1) {
                // Not the last attempt - log retry
                edu.wpi.first.wpilibj.DataLogManager.log(
                    "MinionMotor configuration failed (Status: " + status + "), retrying... (" +
                    (i + 1) + "/" + maxRetries + ")");
                // No Thread.sleep - retry immediately
            }
        }

        // All retries exhausted - report warning to driver station
        edu.wpi.first.wpilibj.DriverStation.reportWarning(
            "MinionMotor configuration failed after " + maxRetries + " attempts", false);
        return false; // Configuration failed after retries
    }

    @Override
    public boolean supportsControlMode(ControlMode mode) {
        // MinionMotor doesn't support CURRENT or MOTION_MAGIC_FOC_TORQUE natively
        return mode != ControlMode.CURRENT && mode != ControlMode.MOTION_MAGIC_FOC_TORQUE;
    }

    @Override
    public String getMotorType() {
        return "MinionMotor (TalonFXS)";
    }

}