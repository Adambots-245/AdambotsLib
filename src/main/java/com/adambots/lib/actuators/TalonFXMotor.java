package com.adambots.lib.actuators;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;

import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.wpilibj.RobotBase;
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
    private boolean focFlag = true; // Safe without Pro license — ignored if unlicensed, enables FOC if licensed

    @NotLogged
    private double feedForward = 0.0;

    @NotLogged
    private boolean isInverted = false; // Track inversion state for follower mode

    @NotLogged
    private boolean isBrakeMode = false; // Track brake mode for atomic config apply

    @NotLogged
    private TalonFXSimState simState; // null when not in sim

    @NotLogged
    private final int maxRetries = 3; // Maximum retries for configuration

    // Local tracking for Slot0Configs — single source of truth (eliminates refresh-failure risk)
    @NotLogged
    private double slot0_kP = 0, slot0_kI = 0, slot0_kD = 0, slot0_kV = 0;
    @NotLogged
    private double slot0_kS = 0, slot0_kA = 0, slot0_kG = 0;
    @NotLogged
    private GravityTypeValue slot0_gravityType = GravityTypeValue.Elevator_Static;

    // Local tracking for Slot1Configs
    @NotLogged
    private double slot1_kP = 0, slot1_kI = 0, slot1_kD = 0, slot1_kV = 0;
    @NotLogged
    private double slot1_kS = 0, slot1_kA = 0, slot1_kG = 0;

    // Local tracking for Slot2Configs
    @NotLogged
    private double slot2_kP = 0, slot2_kI = 0, slot2_kD = 0, slot2_kV = 0;
    @NotLogged
    private double slot2_kS = 0, slot2_kA = 0, slot2_kG = 0;

    // Local tracking for soft limits — single source of truth (eliminates refresh-failure risk)
    @NotLogged
    private double softLimitForwardThreshold = 0, softLimitReverseThreshold = 0;
    @NotLogged
    private boolean softLimitForwardEnabled = false, softLimitReverseEnabled = false;

    // Local tracking for MotionMagicConfigs
    @NotLogged
    private double mmCruiseVelocity = 0, mmAcceleration = 0, mmJerk = 0;
    @NotLogged
    private double mmExpoKV = 0, mmExpoKA = 0;

    // Local tracking for FeedbackConfigs
    @NotLogged
    private FeedbackSensorSourceValue feedbackSource = FeedbackSensorSourceValue.RotorSensor;
    @NotLogged
    private int feedbackRemoteSensorID = 0;
    @NotLogged
    private double feedbackSensorToMechRatio = 1.0, feedbackRotorToSensorRatio = 1.0;

    // Reusable control request objects to avoid allocation overhead
    @NotLogged
    private final VoltageOut voltageRequest = new VoltageOut(0);

    @NotLogged
    private final PositionVoltage positionVoltageRequest = new PositionVoltage(0);

    @NotLogged
    private final VelocityVoltage velocityVoltageRequest = new VelocityVoltage(0);

    @NotLogged
    private final MotionMagicVoltage motionMagicVoltageRequest = new MotionMagicVoltage(0);

    @NotLogged
    private final PositionTorqueCurrentFOC positionTorqueRequest = new PositionTorqueCurrentFOC(0);

    @NotLogged
    private final VelocityTorqueCurrentFOC velocityTorqueRequest = new VelocityTorqueCurrentFOC(0);

    @NotLogged
    private final MotionMagicExpoVoltage motionMagicExpoVoltageRequest = new MotionMagicExpoVoltage(0);

    @NotLogged
    private final MotionMagicExpoTorqueCurrentFOC motionMagicExpoTorqueRequest = new MotionMagicExpoTorqueCurrentFOC(0);

    @NotLogged
    private final DutyCycleOut dutyCycleRequest = new DutyCycleOut(0);

    @NotLogged
    private final TorqueCurrentFOC torqueCurrentRequest = new TorqueCurrentFOC(0);

    @NotLogged
    private final MotionMagicTorqueCurrentFOC motionMagicTorqueRequest = new MotionMagicTorqueCurrentFOC(0);

    @NotLogged
    private final Follower followerRequest = new Follower(0, MotorAlignmentValue.Aligned);

    // Cached StatusSignal objects — initialized in constructor to avoid repeated HashMap lookups
    @NotLogged
    private StatusSignal<Angle> positionSignal;
    @NotLogged
    private StatusSignal<AngularVelocity> velocitySignal;
    @NotLogged
    private StatusSignal<AngularAcceleration> accelerationSignal;
    @NotLogged
    private StatusSignal<Current> statorCurrentSignal;
    @NotLogged
    private StatusSignal<Double> dutyCycleSignal;
    @NotLogged
    private StatusSignal<Temperature> temperatureSignal;
    @NotLogged
    private StatusSignal<ForwardLimitValue> forwardLimitSignal;
    @NotLogged
    private StatusSignal<ReverseLimitValue> reverseLimitSignal;
    @NotLogged
    private StatusSignal<Angle> rotorPositionSignal;

    /**
     * Functional interface for applying a configuration and returning a StatusCode.
     */
    @FunctionalInterface
    interface ConfigApplyAction {
        StatusCode apply();
    }

    /**
     * Constructs a TalonFXMotor with default settings.
     *
     * <p>Defaults: regular CAN bus, 40A supply current limit.
     * Current limits can be further configured via the mechanism config's
     * {@code withCurrentLimits()} method.
     *
     * @param portNum  The CAN ID for the motor controller (0-62).
     * @param isKraken True for Kraken X60, false for Falcon 500.
     */
    public TalonFXMotor(int portNum, boolean isKraken) {
        this(portNum, false, 40, isKraken);
    }

    /**
     * Constructs a TalonFXMotor on a specific CAN bus with a default 40A supply current limit.
     *
     * @param portNum      The CAN ID for the motor controller (0-62).
     * @param isOnCANivore True if the motor is on a CANivore bus.
     * @param isKraken     True for Kraken X60, false for Falcon 500.
     */
    public TalonFXMotor(int portNum, boolean isOnCANivore, boolean isKraken) {
        this(portNum, isOnCANivore, 40, isKraken);
    }

    /**
     * Constructs a TalonFXMotor instance.
     *
     * @param portNum            The CAN ID for the motor controller (0-62).
     * @param isOnCANivore       True if the motor is on a CANivore bus.
     * @param supplyCurrentLimit The supply current limit in amps.
     * @param isKraken           True for Kraken X60, false for Falcon 500.
     */
    public TalonFXMotor(int portNum, boolean isOnCANivore, double supplyCurrentLimit, boolean isKraken) {
        this.isKraken = isKraken;

        // Initialize motor on either CANivore or regular CAN bus
        if (isOnCANivore) {
            motor = new TalonFX(portNum, new CANBus("*"));
        } else {
            motor = new TalonFX(portNum);
        }

        // Factory reset to ensure a clean baseline — TalonFX persists configs in flash,
        // so stale settings from previous code deploys or Phoenix Tuner can cause issues
        motor.getConfigurator().apply(new TalonFXConfiguration(), 0.050);

        // Configure default current limits
        var currentLimits = new CurrentLimitsConfigs();
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

        // Cache StatusSignal references to avoid repeated HashMap lookups in getters
        positionSignal = motor.getPosition();
        velocitySignal = motor.getVelocity();
        accelerationSignal = motor.getAcceleration();
        statorCurrentSignal = motor.getStatorCurrent();
        dutyCycleSignal = motor.getDutyCycle();
        temperatureSignal = motor.getDeviceTemp();
        forwardLimitSignal = motor.getForwardLimit();
        reverseLimitSignal = motor.getReverseLimit();
        rotorPositionSignal = motor.getRotorPosition();

        // Cache sim state for simulation support
        if (RobotBase.isSimulation()) {
            simState = motor.getSimState();
        }
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
                motor.setControl(dutyCycleRequest.withOutput(value).withEnableFOC(focFlag));
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
                motor.setControl(torqueCurrentRequest.withOutput(value));
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
                motor.setControl(motionMagicTorqueRequest.withPosition(value).withSlot(0).withFeedForward(feedForward));
                break;
            case POSITION_FOC_TORQUE:
                // Torque current-based position control — requires Phoenix Pro license
                motor.setControl(positionTorqueRequest.withPosition(value)
                        .withSlot(0).withFeedForward(feedForward));
                break;
            case VELOCITY_FOC_TORQUE:
                // Torque current-based velocity control — requires Phoenix Pro license
                motor.setControl(velocityTorqueRequest.withVelocity(value)
                        .withSlot(0).withFeedForward(feedForward));
                break;
            case MOTION_MAGIC_EXPO:
                // Exponential motion profile (voltage) — smoother than trapezoidal
                if (feedForward != 0) {
                    motor.setControl(motionMagicExpoVoltageRequest.withPosition(value)
                            .withSlot(0).withEnableFOC(focFlag).withFeedForward(feedForward));
                } else {
                    motor.setControl(motionMagicExpoVoltageRequest.withPosition(value)
                            .withSlot(0).withEnableFOC(focFlag));
                }
                break;
            case MOTION_MAGIC_EXPO_TORQUE:
                // Exponential motion profile with torque current — requires Phoenix Pro license
                motor.setControl(motionMagicExpoTorqueRequest.withPosition(value)
                        .withSlot(0).withFeedForward(feedForward));
                break;
            case FOLLOWER:
                // Follow another Talon FX controller, respecting isInverted flag
                motor.setControl(followerRequest.withLeaderID((int) value)
                        .withMotorAlignment(isInverted ? MotorAlignmentValue.Opposed : MotorAlignmentValue.Aligned));
                break;
        }
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
        motor.setControl(dutyCycleRequest.withOutput(speed).withEnableFOC(focFlag));
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
                slot0_kP = kP; slot0_kI = kI; slot0_kD = kD; slot0_kV = kF;
                applySlot0();
                break;
            case 1:
                slot1_kP = kP; slot1_kI = kI; slot1_kD = kD; slot1_kV = kF;
                applySlot1();
                break;
            case 2:
                slot2_kP = kP; slot2_kI = kI; slot2_kD = kD; slot2_kV = kF;
                applySlot2();
                break;
            default:
                throw new IllegalArgumentException("Invalid slot index. Must be between 0 and 2.");
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
                slot0_kP = kP; slot0_kI = kI; slot0_kD = kD;
                slot0_kV = kV; slot0_kS = kS; slot0_kA = kA; slot0_kG = kG;
                applySlot0();
                break;
            case 1:
                slot1_kP = kP; slot1_kI = kI; slot1_kD = kD;
                slot1_kV = kV; slot1_kS = kS; slot1_kA = kA; slot1_kG = kG;
                applySlot1();
                break;
            case 2:
                slot2_kP = kP; slot2_kI = kI; slot2_kD = kD;
                slot2_kV = kV; slot2_kS = kS; slot2_kA = kA; slot2_kG = kG;
                applySlot2();
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
        mmCruiseVelocity = cruiseVelocity.in(RotationsPerSecond);
        mmAcceleration = acceleration.in(RotationsPerSecondPerSecond);
        mmJerk = jerkRPSPerSecPerSec;
        applyMotionMagic();
    }

    /**
     * Configures Motion Magic Expo profile parameters.
     *
     * <p>Expo profiles approach the target asymptotically, producing smoother motion
     * than standard trapezoidal profiles. The profile shape is determined by kV and kA
     * parameters (always in Volts, regardless of slot gain units).
     *
     * <p>Use with {@link ControlMode#MOTION_MAGIC_EXPO} or {@link ControlMode#MOTION_MAGIC_EXPO_TORQUE}.
     *
     * @param expoKV Voltage per RPS — determines cruise velocity (V/RPS). Set to 0 for max velocity.
     * @param expoKA Voltage per RPS² — determines acceleration (V/RPS²)
     * @param cruiseVelocity Maximum cruise velocity. Set to {@code RotationsPerSecond.of(0)} for no limit.
     */
    public void configureMotionMagicExpo(double expoKV, double expoKA, AngularVelocity cruiseVelocity) {
        mmExpoKV = expoKV;
        mmExpoKA = expoKA;
        mmCruiseVelocity = cruiseVelocity.in(RotationsPerSecond);
        applyMotionMagic();
    }

    /**
     * Builds MotionMagicConfigs from local state and applies it.
     * No refresh needed — we own the full state.
     */
    private void applyMotionMagic() {
        var config = new MotionMagicConfigs();
        config.MotionMagicCruiseVelocity = mmCruiseVelocity;
        config.MotionMagicAcceleration = mmAcceleration;
        config.MotionMagicJerk = mmJerk;
        config.MotionMagicExpo_kV = mmExpoKV;
        config.MotionMagicExpo_kA = mmExpoKA;

        boolean success = applyConfigWithRetry(() -> motor.getConfigurator().apply(config));
        if (!success) {
            edu.wpi.first.wpilibj.DriverStation.reportError(
                "TalonFXMotor: Failed to apply Motion Magic Expo config after retries", false);
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

        // Enable stator current reporting — suppressed by optimizeBusUtilization() in constructor
        statorCurrentSignal.setUpdateFrequency(10);
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
        softLimitForwardThreshold = forwardLimitRotations;
        softLimitReverseThreshold = reverseLimitRotations;
        softLimitForwardEnabled = enable;
        softLimitReverseEnabled = enable;
        applySoftLimits();
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
        softLimitForwardEnabled = enable;
        softLimitReverseEnabled = enable;
        applySoftLimits();
    }

    /**
     * Builds SoftwareLimitSwitchConfigs from local state and applies it.
     * No refresh needed — we own the full state.
     */
    private void applySoftLimits() {
        var config = new SoftwareLimitSwitchConfigs();
        config.ForwardSoftLimitThreshold = softLimitForwardThreshold;
        config.ReverseSoftLimitThreshold = softLimitReverseThreshold;
        config.ForwardSoftLimitEnable = softLimitForwardEnabled;
        config.ReverseSoftLimitEnable = softLimitReverseEnabled;

        boolean success = applyConfigWithRetry(() -> motor.getConfigurator().apply(config));
        if (!success) {
            edu.wpi.first.wpilibj.DriverStation.reportError(
                "TalonFXMotor: Failed to apply soft limits after retries", false);
        }
    }

    /**
     * Sets the inversion state of the motor.
     *
     * <p>Uses the Phoenix 6 MotorOutputConfigs to configure motor direction.
     * This matches the CTRE quickstart pattern of direct config apply without refresh.
     *
     * @param inverted true to invert the motor (Clockwise is positive),
     *                 false for normal operation (CounterClockwise is positive)
     */
    @Override
    public void setInverted(boolean inverted) {
        this.isInverted = inverted;
        applyMotorOutput();
    }

    /**
     * Sets the positive direction of the motor using Phoenix 6 InvertedValue.
     *
     * <p>Maps directly to CTRE's {@code InvertedValue.Clockwise_Positive} and
     * {@code InvertedValue.CounterClockwise_Positive} for explicit CW/CCW control.
     *
     * @param direction The direction that counts as positive output
     */
    @Override
    public void setDirection(MotorDirection direction) {
        this.isInverted = (direction == MotorDirection.CLOCKWISE_POSITIVE);
        applyMotorOutput();
    }

    /**
     * Builds MotorOutputConfigs from local state and applies it.
     * No refresh needed — we own the full state (Inverted + NeutralMode).
     * Note: PeakForwardVoltage/PeakReverseVoltage use factory defaults (16.0/-16.0)
     * since the library does not expose methods to modify them.
     */
    private void applyMotorOutput() {
        var config = new MotorOutputConfigs();
        config.withInverted(isInverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive)
              .withNeutralMode(isBrakeMode ? NeutralModeValue.Brake : NeutralModeValue.Coast);

        boolean success = applyConfigWithRetry(() -> motor.getConfigurator().apply(config));
        if (!success) {
            edu.wpi.first.wpilibj.DriverStation.reportError(
                "TalonFXMotor: Failed to apply motor output config after retries", false);
        }
    }

    /**
     * Sets the neutral mode of the motor to either brake or coast.
     * In brake mode, the motor actively resists motion when not driven.
     * In coast mode, the motor spins freely when not driven.
     *
     * <p>Uses the TalonFX direct {@code setNeutralMode()} method, which is independent
     * of MotorOutputConfigs and avoids any interaction with the Inverted config.
     *
     * @param brake true to enable brake mode, false for coast mode
     */
    @Override
    public void setBrakeMode(boolean brake) {
        this.isBrakeMode = brake;
        motor.setNeutralMode(brake ? NeutralModeValue.Brake : NeutralModeValue.Coast);
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
        motor.setControl(positionVoltageRequest.withPosition(rotations)
                .withSlot(activePidSlot)
                .withEnableFOC(focFlag)
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
        return positionSignal.getValueAsDouble();
    }

    @Override
    public double getRotorPosition() {
        return rotorPositionSignal.getValueAsDouble();
    }

    /**
     * Gets the current velocity of the motor.
     *
     * @return The current velocity
     */
    @Override
    public AngularVelocity getVelocity() {
        return RotationsPerSecond.of(velocitySignal.getValueAsDouble());
    }

    /**
     * Gets the current acceleration of the motor.
     *
     * @return The current acceleration
     */
    @Override
    public AngularAcceleration getAcceleration() {
        return RotationsPerSecondPerSecond.of(accelerationSignal.getValueAsDouble());
    }

    /**
     * Gets the current draw of the motor.
     *
     * @return The current draw
     */
    @Override
    public Current getCurrentDraw() {
        return Amps.of(statorCurrentSignal.getValueAsDouble());
    }

    /**
     * Gets the current output percentage of the motor.
     *
     * @return The current output percentage as a double between -1.0 and 1.0
     */
    @Override
    public double getOutputPercent() {
        return dutyCycleSignal.getValueAsDouble();
    }

    /**
     * Gets the current temperature of the motor.
     *
     * @return The current temperature in degrees Celsius
     */
    @Override
    public double getTemperature() {
        return temperatureSignal.getValueAsDouble();
    }

    /**
     * Gets the state of the forward limit switch.
     *
     * @return True if the forward limit switch is closed, false otherwise
     */
    @Override
    public boolean getForwardLimitSwitch() {
        return forwardLimitSignal.getValue() == ForwardLimitValue.ClosedToGround;
    }

    /**
     * Gets the state of the reverse limit switch.
     *
     * @return True if the reverse limit switch is closed, false otherwise
     */
    @Override
    public boolean getReverseLimitSwitch() {
        return reverseLimitSignal.getValue() == ReverseLimitValue.ClosedToGround;
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
        motor.setControl(followerRequest.withLeaderID(deviceID).withMotorAlignment(alignment));
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

        // Re-enable limit signals at higher rate for active limit monitoring
        forwardLimitSignal.setUpdateFrequency(50);
        reverseLimitSignal.setUpdateFrequency(50);

        // Set up simulation state
        var simState = motor.getSimState();
        simState.setForwardLimit(enableForward); // Set the initial state
        simState.setReverseLimit(enableReverse);
    }

    @Override
    public void configureGravity(GravityType type) {
        switch (type) {
            case ARM_COSINE:
                slot0_gravityType = GravityTypeValue.Arm_Cosine;
                break;
            case ELEVATOR_STATIC:
                slot0_gravityType = GravityTypeValue.Elevator_Static;
                break;
            case NONE:
            default:
                slot0_gravityType = GravityTypeValue.Elevator_Static;
                slot0_kG = 0;
                break;
        }
        applySlot0();
    }

    @Override
    public void configureSensorToMechanismRatio(double ratio) {
        feedbackSensorToMechRatio = ratio;
        applyFeedbackConfig();
    }

    /**
     * Configures a remote CANcoder as the feedback sensor.
     *
     * @param cancoderId             CAN ID of the CANcoder
     * @param sensorToMechanismRatio Gear ratio from sensor to mechanism output
     */
    @Override
    public void configureRemoteCANcoder(int cancoderId, double sensorToMechanismRatio) {
        feedbackSource = FeedbackSensorSourceValue.RemoteCANcoder;
        feedbackRemoteSensorID = cancoderId;
        feedbackSensorToMechRatio = sensorToMechanismRatio;
        applyFeedbackConfig();
    }

    /**
     * Configures a fused CANcoder as the feedback sensor (requires Phoenix Pro).
     *
     * <p>Fuses the CANcoder with the internal rotor sensor for higher bandwidth position
     * and velocity feedback. Best for swerve azimuth and high-accuracy mechanisms.
     *
     * @param cancoderId             CAN ID of the CANcoder
     * @param sensorToMechanismRatio Gear ratio from sensor to mechanism output
     * @param rotorToSensorRatio     Gear ratio from rotor to the CANcoder
     */
    @Override
    public void configureFusedCANcoder(int cancoderId, double sensorToMechanismRatio, double rotorToSensorRatio) {
        feedbackSource = FeedbackSensorSourceValue.FusedCANcoder;
        feedbackRemoteSensorID = cancoderId;
        feedbackSensorToMechRatio = sensorToMechanismRatio;
        feedbackRotorToSensorRatio = rotorToSensorRatio;
        applyFeedbackConfig();
    }

    /**
     * Configures a sync CANcoder as the feedback sensor (requires Phoenix Pro).
     *
     * <p>Synchronizes the internal rotor position against the CANcoder, then continues
     * using the rotor sensor for closed-loop control. Reports if positions diverge.
     *
     * @param cancoderId             CAN ID of the CANcoder
     * @param sensorToMechanismRatio Gear ratio from sensor to mechanism output
     * @param rotorToSensorRatio     Gear ratio from rotor to the CANcoder
     */
    @Override
    public void configureSyncCANcoder(int cancoderId, double sensorToMechanismRatio, double rotorToSensorRatio) {
        feedbackSource = FeedbackSensorSourceValue.SyncCANcoder;
        feedbackRemoteSensorID = cancoderId;
        feedbackSensorToMechRatio = sensorToMechanismRatio;
        feedbackRotorToSensorRatio = rotorToSensorRatio;
        applyFeedbackConfig();
    }

    /**
     * Builds FeedbackConfigs from local state and applies it.
     * No refresh needed — we own the full state.
     */
    private void applyFeedbackConfig() {
        var config = new FeedbackConfigs();
        config.FeedbackSensorSource = feedbackSource;
        config.FeedbackRemoteSensorID = feedbackRemoteSensorID;
        config.SensorToMechanismRatio = feedbackSensorToMechRatio;
        config.RotorToSensorRatio = feedbackRotorToSensorRatio;

        boolean success = applyConfigWithRetry(() -> motor.getConfigurator().apply(config));
        if (!success) {
            edu.wpi.first.wpilibj.DriverStation.reportError(
                "TalonFXMotor: Failed to apply FeedbackConfigs after retries", false);
        }

        // Re-cache signals — feedback source changed, signal routing may differ
        positionSignal = motor.getPosition();
        velocitySignal = motor.getVelocity();
        rotorPositionSignal = motor.getRotorPosition();

        // Re-enable signals for the new feedback source
        positionSignal.setUpdateFrequency(50);
        velocitySignal.setUpdateFrequency(50);
        rotorPositionSignal.setUpdateFrequency(50);
    }

    /**
     * Builds Slot0Configs from local state and applies it. No refresh needed — we own the full state.
     */
    private void applySlot0() {
        var config = new Slot0Configs();
        config.kP = slot0_kP;
        config.kI = slot0_kI;
        config.kD = slot0_kD;
        config.kV = slot0_kV;
        config.kS = slot0_kS;
        config.kA = slot0_kA;
        config.kG = slot0_kG;
        config.GravityType = slot0_gravityType;
        boolean success = applyConfigWithRetry(() -> motor.getConfigurator().apply(config));
        if (!success) {
            edu.wpi.first.wpilibj.DriverStation.reportError(
                "TalonFXMotor: Failed to apply Slot0 config after retries", false);
        }
    }

    /**
     * Builds Slot1Configs from local state and applies it. No refresh needed — we own the full state.
     */
    private void applySlot1() {
        var config = new Slot1Configs();
        config.kP = slot1_kP;
        config.kI = slot1_kI;
        config.kD = slot1_kD;
        config.kV = slot1_kV;
        config.kS = slot1_kS;
        config.kA = slot1_kA;
        config.kG = slot1_kG;
        boolean success = applyConfigWithRetry(() -> motor.getConfigurator().apply(config));
        if (!success) {
            edu.wpi.first.wpilibj.DriverStation.reportError(
                "TalonFXMotor: Failed to apply Slot1 config after retries", false);
        }
    }

    /**
     * Builds Slot2Configs from local state and applies it. No refresh needed — we own the full state.
     */
    private void applySlot2() {
        var config = new Slot2Configs();
        config.kP = slot2_kP;
        config.kI = slot2_kI;
        config.kD = slot2_kD;
        config.kV = slot2_kV;
        config.kS = slot2_kS;
        config.kA = slot2_kA;
        config.kG = slot2_kG;
        boolean success = applyConfigWithRetry(() -> motor.getConfigurator().apply(config));
        if (!success) {
            edu.wpi.first.wpilibj.DriverStation.reportError(
                "TalonFXMotor: Failed to apply Slot2 config after retries", false);
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

    @Override
    public double getSimMotorVoltage() {
        return simState != null ? simState.getMotorVoltage() : 0.0;
    }

    @Override
    public void setSimPosition(double rotorRotations) {
        if (simState != null) simState.setRawRotorPosition(rotorRotations);
    }

    @Override
    public void setSimVelocity(double rotorRPS) {
        if (simState != null) simState.setRotorVelocity(rotorRPS);
    }

    @Override
    public void setSimSupplyVoltage(double volts) {
        if (simState != null) simState.setSupplyVoltage(volts);
    }
}