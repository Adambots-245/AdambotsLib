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
import com.ctre.phoenix6.configs.CommutationConfigs;
import com.ctre.phoenix6.configs.ExternalFeedbackConfigs;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfigurator;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;

import com.ctre.phoenix6.sim.TalonFXSSimState;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.wpilibj.RobotBase;
import static edu.wpi.first.units.Units.*;
import edu.wpi.first.units.measure.*;

/**
 * Implementation of BaseMotor for Minion Motor using TalonFXS motor controller.
 * Note: Applying the configuration is a blocking call and may take some time.
 * This implementation includes retry logic for configuration application.
 * Ensure that any configuration setting is done only once during initialization
 * to avoid unnecessary delays during operation.
 */
@Logged
public class MinionMotor implements BaseMotor {
    @NotLogged
    private final TalonFXS motor;

    @NotLogged
    private final TalonFXSConfigurator configurator;

    @NotLogged
    private final MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0);

    @NotLogged
    private final PositionVoltage positionRequest = new PositionVoltage(0);

    @NotLogged
    private final VelocityVoltage velocityRequest = new VelocityVoltage(0);

    @NotLogged
    private final VoltageOut voltageRequest = new VoltageOut(0);

    @NotLogged
    private final DutyCycleOut percentRequest = new DutyCycleOut(0);

    @NotLogged
    private final int maxRetries = 3; // Maximum retries for configuration

    @NotLogged
    private boolean focFlag = true; // Safe without Pro license — ignored if unlicensed, enables FOC if licensed

    @NotLogged
    private boolean isInverted = false; // Track inversion state for follower mode

    @NotLogged
    private boolean isBrakeMode = false; // Track brake mode for atomic config apply

    @NotLogged
    private TalonFXSSimState simState; // null when not in sim

    // Local tracking for Slot0Configs — single source of truth (eliminates refresh-failure risk)
    @NotLogged
    private double slot0_kP = 0, slot0_kI = 0, slot0_kD = 0, slot0_kV = 0;
    @NotLogged
    private double slot0_kS = 0, slot0_kA = 0, slot0_kG = 0;
    @NotLogged
    private GravityTypeValue slot0_gravityType = GravityTypeValue.Elevator_Static;

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

        // Factory reset to ensure a clean baseline — TalonFXS persists configs in flash,
        // so stale settings from previous code deploys or Phoenix Tuner can cause issues
        configurator.apply(new TalonFXSConfiguration(), 0.050);

        // Tell the TalonFXS a Minion motor is connected via JST
        CommutationConfigs commutationConfigs = new CommutationConfigs();
        commutationConfigs.MotorArrangement = MotorArrangementValue.Minion_JST;
        configurator.apply(commutationConfigs);

        // Configure status frame periods for efficiency
        motor.getVelocity().setUpdateFrequency(50);
        motor.getPosition().setUpdateFrequency(50);
        motor.getForwardLimit().setUpdateFrequency(25);
        motor.getReverseLimit().setUpdateFrequency(25);

        // Optimize CAN bus usage
        motor.optimizeBusUtilization();

        // Cache sim state for simulation support
        if (RobotBase.isSimulation()) {
            simState = motor.getSimState();
        }
    }

    /**
     * Constructor for MinionMotor using TalonFXS on a CANivore bus.
     * Uses the "*" wildcard to auto-discover the device on any CANivore bus,
     * matching the TalonFXMotor interface.
     *
     * @param deviceID The CAN ID of the motor controller
     * @param isOnCANivore true if the motor is on a CANivore bus, false for RIO CAN bus
     */
    public MinionMotor(int deviceID, boolean isOnCANivore) {
        this(deviceID, isOnCANivore ? "*" : "");
    }

    /**
     * Constructor for MinionMotor using TalonFXS with a specific CAN bus name.
     * Use this when you need to target a specific named CAN bus rather than
     * auto-discovering via the "*" wildcard.
     *
     * @param deviceID The CAN ID of the motor controller
     * @param canBus The name of the CAN bus (use "*" for auto-discovery, "" for RIO bus)
     */
    public MinionMotor(int deviceID, String canBus) {
        motor = new TalonFXS(deviceID, new CANBus(canBus));
        configurator = motor.getConfigurator();

        // Factory reset to ensure a clean baseline — TalonFXS persists configs in flash,
        // so stale settings from previous code deploys or Phoenix Tuner can cause issues
        configurator.apply(new TalonFXSConfiguration(), 0.050);

        // Tell the TalonFXS a Minion motor is connected via JST
        CommutationConfigs commutationConfigs = new CommutationConfigs();
        commutationConfigs.MotorArrangement = MotorArrangementValue.Minion_JST;
        configurator.apply(commutationConfigs);

        // Configure status frame periods for efficiency
        motor.getVelocity().setUpdateFrequency(50);
        motor.getPosition().setUpdateFrequency(50);
        motor.getForwardLimit().setUpdateFrequency(25);
        motor.getReverseLimit().setUpdateFrequency(25);

        // Optimize CAN bus usage
        motor.optimizeBusUtilization();

        // Cache sim state for simulation support
        if (RobotBase.isSimulation()) {
            simState = motor.getSimState();
        }
    }

    @Override
    public void set(ControlMode mode, double value) {
        switch (mode) {
            case PERCENT_OUTPUT:
                motor.setControl(percentRequest.withOutput(value).withEnableFOC(focFlag));
                break;
            case POSITION:
                motor.setControl(positionRequest.withPosition(value).withEnableFOC(focFlag));
                break;
            case VELOCITY:
                motor.setControl(velocityRequest.withVelocity(value).withEnableFOC(focFlag));
                break;
            case VOLTAGE:
                motor.setControl(voltageRequest.withOutput(value).withEnableFOC(focFlag));
                break;
            case CURRENT:
                // TalonFXS doesn't have a direct current control mode - fallback to percent output
                edu.wpi.first.wpilibj.DriverStation.reportWarning(
                    "MinionMotor: CURRENT mode not supported. Falling back to PERCENT_OUTPUT.", false);
                motor.setControl(percentRequest.withOutput(value));
                break;
            case MOTION_MAGIC:
                motor.setControl(motionMagicRequest.withPosition(value).withEnableFOC(focFlag));
                break;
            case FOLLOWER:
                // In Phoenix 6, Follower requires device ID
                // value here is assumed to be the device ID to follow
                // Use isInverted to determine alignment (like TalonFXMotor)
                motor.setControl(new Follower((int)value,
                        isInverted ? MotorAlignmentValue.Opposed : MotorAlignmentValue.Aligned));
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
        slot0_kP = kP; slot0_kI = kI; slot0_kD = kD; slot0_kV = kF;
        applySlot0();
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
     * <p><strong>Important:</strong> When using kG, you must also call
     * {@link #configureGravity(GravityType)} to set the gravity compensation type.
     * Without it, kG defaults to Elevator_Static behavior.
     *
     * @param slotIdx The PID slot index to configure (0 only for MinionMotor)
     * @param kP Proportional gain
     * @param kI Integral gain
     * @param kD Derivative gain
     * @param kV Velocity feedforward
     * @param kS Static feedforward (overcomes friction)
     * @param kA Acceleration feedforward
     * @param kG Gravity feedforward
     */
    public void setPID(int slotIdx, double kP, double kI, double kD,
                       double kV, double kS, double kA, double kG) {
        slot0_kP = kP; slot0_kI = kI; slot0_kD = kD;
        slot0_kV = kV; slot0_kS = kS; slot0_kA = kA; slot0_kG = kG;
        applySlot0();
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
        var config = new ExternalFeedbackConfigs();

        StatusCode refreshStatus = configurator.refresh(config);
        if (!refreshStatus.isOK()) {
            edu.wpi.first.wpilibj.DriverStation.reportWarning(
                "MinionMotor: Failed to refresh ExternalFeedbackConfigs (Status: " + refreshStatus +
                "). Other feedback fields may be factory defaulted.", false);
        }

        config.SensorToMechanismRatio = ratio;

        boolean success = applyConfigWithRetry(() -> configurator.apply(config));
        if (!success) {
            edu.wpi.first.wpilibj.DriverStation.reportError(
                "MinionMotor: Failed to apply SensorToMechanismRatio after retries", false);
        }
    }

    @Override
    public void configureMotionMagic(AngularVelocity cruiseVelocity, AngularAcceleration acceleration, double jerkRPSPerSecPerSec) {
        MotionMagicConfigs configs = new MotionMagicConfigs();

        // CRITICAL: Check if refresh fails
        StatusCode refreshStatus = configurator.refresh(configs);
        if (!refreshStatus.isOK()) {
            edu.wpi.first.wpilibj.DriverStation.reportWarning(
                "MinionMotor: Failed to refresh config before configureMotionMagic (Status: " + refreshStatus +
                "). Configuration may be factory defaulted!", true);
        }

        // Set new values
        configs.MotionMagicCruiseVelocity = cruiseVelocity.in(RotationsPerSecond);
        configs.MotionMagicAcceleration = acceleration.in(RotationsPerSecondPerSecond);
        configs.MotionMagicJerk = jerkRPSPerSecPerSec;

        // Apply configuration - check return value
        boolean success = applyConfigWithRetry(() -> configurator.apply(configs));
        if (!success) {
            edu.wpi.first.wpilibj.DriverStation.reportError(
                "MinionMotor: Failed to apply Motion Magic configuration after retries", false);
        }
    }

    @Override
    public void configureCurrentLimits(Current stallLimit, Current freeLimit, double limitRpmThreshold) {
        CurrentLimitsConfigs configs = new CurrentLimitsConfigs();

        // CRITICAL: Check if refresh fails
        StatusCode refreshStatus = configurator.refresh(configs);
        if (!refreshStatus.isOK()) {
            edu.wpi.first.wpilibj.DriverStation.reportWarning(
                "MinionMotor: Failed to refresh config before configureCurrentLimits (Status: " + refreshStatus +
                "). Configuration may be factory defaulted!", true);
        }

        double stallLimitAmps = stallLimit.in(Amps);
        double freeLimitAmps = freeLimit.in(Amps);

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

    /**
     * Sets the inversion state of the motor.
     *
     * <p>Uses the Phoenix 6 MotorOutputConfigs to configure motor direction.
     * Applies directly without refresh to match the CTRE quickstart pattern.
     *
     * @param inverted true to invert the motor (Clockwise is positive),
     *                 false for normal operation (CounterClockwise is positive)
     */
    @Override
    public void setInverted(boolean inverted) {
        this.isInverted = inverted;

        var config = new MotorOutputConfigs();

        // Refresh to preserve PeakVoltage and other MotorOutputConfigs fields
        StatusCode refreshStatus = configurator.refresh(config);
        if (!refreshStatus.isOK()) {
            edu.wpi.first.wpilibj.DriverStation.reportWarning(
                "MinionMotor: Failed to refresh MotorOutputConfigs (Status: " + refreshStatus +
                "). Factory defaults will be used for PeakVoltage fields.", false);
        }

        config.withInverted(isInverted ? com.ctre.phoenix6.signals.InvertedValue.Clockwise_Positive :
                                          com.ctre.phoenix6.signals.InvertedValue.CounterClockwise_Positive)
              .withNeutralMode(isBrakeMode ? com.ctre.phoenix6.signals.NeutralModeValue.Brake :
                                              com.ctre.phoenix6.signals.NeutralModeValue.Coast);

        boolean success = applyConfigWithRetry(() -> configurator.apply(config));
        if (!success) {
            edu.wpi.first.wpilibj.DriverStation.reportError(
                "MinionMotor: Failed to apply motor inversion after retries", false);
        }
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

        var config = new MotorOutputConfigs();

        StatusCode refreshStatus = configurator.refresh(config);
        if (!refreshStatus.isOK()) {
            edu.wpi.first.wpilibj.DriverStation.reportWarning(
                "MinionMotor: Failed to refresh MotorOutputConfigs (Status: " + refreshStatus +
                "). Factory defaults will be used for PeakVoltage fields.", false);
        }

        config.withInverted(direction == MotorDirection.CLOCKWISE_POSITIVE
                ? com.ctre.phoenix6.signals.InvertedValue.Clockwise_Positive
                : com.ctre.phoenix6.signals.InvertedValue.CounterClockwise_Positive)
              .withNeutralMode(isBrakeMode ? com.ctre.phoenix6.signals.NeutralModeValue.Brake
                                           : com.ctre.phoenix6.signals.NeutralModeValue.Coast);

        boolean success = applyConfigWithRetry(() -> configurator.apply(config));
        if (!success) {
            edu.wpi.first.wpilibj.DriverStation.reportError(
                "MinionMotor: Failed to apply motor direction after retries", false);
        }
    }

    /**
     * Sets the neutral mode of the motor to either brake or coast.
     * In brake mode, the motor actively resists motion when not driven.
     * In coast mode, the motor spins freely when not driven.
     *
     * <p>Uses the TalonFXS direct {@code setNeutralMode()} method, which is independent
     * of MotorOutputConfigs and avoids any interaction with the Inverted config.
     *
     * @param brake true to enable brake mode, false for coast mode
     */
    @Override
    public void setBrakeMode(boolean brake) {
        this.isBrakeMode = brake;
        motor.setNeutralMode(brake ? com.ctre.phoenix6.signals.NeutralModeValue.Brake :
                                    com.ctre.phoenix6.signals.NeutralModeValue.Coast);
    }

    @Override
    public void setPosition(double rotations) {
        // In Phoenix 6, we set the position through the configurator
        motor.setPosition(rotations);
    }

    @Override
    public void enableVoltageCompensation(Voltage nominalVoltage) {
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
    public AngularVelocity getVelocity() {
        // In Phoenix 6, getVelocity returns rotations per second
        var velocitySignal = motor.getVelocity();
        velocitySignal.refresh();
        return RotationsPerSecond.of(velocitySignal.getValue().magnitude());
    }

    @Override
    public AngularAcceleration getAcceleration() {
        var accelerationSignal = motor.getAcceleration();
        accelerationSignal.refresh();
        return RotationsPerSecondPerSecond.of(accelerationSignal.getValue().magnitude());
    }

    @Override
    public Current getCurrentDraw() {
        var currentSignal = motor.getStatorCurrent();
        currentSignal.refresh();
        return Amps.of(currentSignal.getValue().magnitude());
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
        MotorAlignmentValue alignment = opposeMaster ? MotorAlignmentValue.Opposed : MotorAlignmentValue.Aligned;
        motor.setControl(new Follower(deviceID, alignment));
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
        boolean success = applyConfigWithRetry(() -> configurator.apply(config));
        if (!success) {
            edu.wpi.first.wpilibj.DriverStation.reportError(
                "MinionMotor: Failed to apply Slot0 config after retries", false);
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