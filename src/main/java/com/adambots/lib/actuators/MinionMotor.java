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
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.ClosedLoopGeneralConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.HardwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.Slot2Configs;
import com.ctre.phoenix6.configs.CommutationConfigs;
import com.ctre.phoenix6.configs.ExternalFeedbackConfigs;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfigurator;
import com.ctre.phoenix6.signals.ExternalFeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.SensorPhaseValue;

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
    private final Follower followerRequest = new Follower(0, MotorAlignmentValue.Aligned);

    // Cached StatusSignal objects — initialized in constructor to avoid repeated HashMap lookups.
    // IMPORTANT: Phoenix 6 StatusSignals do NOT auto-update. Every getter MUST call
    // signal.refresh() before reading. refresh() is non-blocking (reads from local CAN
    // cache, not a CAN round-trip). Without refresh(), values go stale after the first read.
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

    // Local tracking for CurrentLimitsConfigs — defaults match Phoenix 6 factory defaults
    @NotLogged
    private double currentStatorLimit = 120, currentSupplyLimit = 70;
    @NotLogged
    private boolean currentStatorEnabled = true, currentSupplyEnabled = true;
    @NotLogged
    private double currentSupplyLowerLimit = 0, currentSupplyLowerTime = 0.1;

    // Local tracking for HardwareLimitSwitchConfigs
    @NotLogged
    private boolean hardLimitForwardEnabled = false, hardLimitReverseEnabled = false;
    @NotLogged
    private boolean hardLimitForwardAutoset = false, hardLimitReverseAutoset = false;
    @NotLogged
    private double hardLimitForwardValue = 0, hardLimitReverseValue = 0;

    // Local tracking for soft limits — single source of truth (eliminates refresh-failure risk)
    @NotLogged
    private double softLimitForwardThreshold = 0, softLimitReverseThreshold = 0;
    @NotLogged
    private boolean softLimitForwardEnabled = false, softLimitReverseEnabled = false;

    // Local tracking for MotionMagicConfigs
    @NotLogged
    private double mmCruiseVelocity = 0, mmAcceleration = 0, mmJerk = 0;

    // Local tracking for ExternalFeedbackConfigs
    @NotLogged
    private ExternalFeedbackSensorSourceValue extFeedbackSource = ExternalFeedbackSensorSourceValue.Commutation;
    @NotLogged
    private int extFeedbackRemoteSensorID = 0;
    @NotLogged
    private double extSensorToMechRatio = 1.0, extRotorToSensorRatio = 1.0;
    @NotLogged
    private double extAbsoluteSensorOffset = 0, extDiscontinuityPoint = 1.0;
    @NotLogged
    private int extQuadratureEdgesPerRotation = 4096;
    @NotLogged
    private SensorPhaseValue extSensorPhase = SensorPhaseValue.Aligned;

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
        configurator.apply(new TalonFXSConfiguration(), 0.100);

        // Tell the TalonFXS a Minion motor is connected via JST
        CommutationConfigs commutationConfigs = new CommutationConfigs();
        commutationConfigs.MotorArrangement = MotorArrangementValue.Minion_JST;
        configurator.apply(commutationConfigs);

        // Configure status frame periods for all signals used by getters
        motor.getPosition().setUpdateFrequency(50);
        motor.getVelocity().setUpdateFrequency(50);
        motor.getAcceleration().setUpdateFrequency(50);
        motor.getRotorPosition().setUpdateFrequency(50);
        motor.getStatorCurrent().setUpdateFrequency(10);
        motor.getDutyCycle().setUpdateFrequency(10);
        motor.getDeviceTemp().setUpdateFrequency(4);
        motor.getForwardLimit().setUpdateFrequency(25);
        motor.getReverseLimit().setUpdateFrequency(25);

        // Optimize CAN bus usage — suppresses signals not listed above
        motor.optimizeBusUtilization();

        // Cache StatusSignal references to avoid repeated HashMap lookups in getters
        cacheStatusSignals();

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
        configurator.apply(new TalonFXSConfiguration(), 0.100);

        // Tell the TalonFXS a Minion motor is connected via JST
        CommutationConfigs commutationConfigs = new CommutationConfigs();
        commutationConfigs.MotorArrangement = MotorArrangementValue.Minion_JST;
        configurator.apply(commutationConfigs);

        // Configure status frame periods for all signals used by getters
        motor.getPosition().setUpdateFrequency(50);
        motor.getVelocity().setUpdateFrequency(50);
        motor.getAcceleration().setUpdateFrequency(50);
        motor.getRotorPosition().setUpdateFrequency(50);
        motor.getStatorCurrent().setUpdateFrequency(10);
        motor.getDutyCycle().setUpdateFrequency(10);
        motor.getDeviceTemp().setUpdateFrequency(4);
        motor.getForwardLimit().setUpdateFrequency(25);
        motor.getReverseLimit().setUpdateFrequency(25);

        // Optimize CAN bus usage — suppresses signals not listed above
        motor.optimizeBusUtilization();

        // Cache StatusSignal references to avoid repeated HashMap lookups in getters
        cacheStatusSignals();

        // Cache sim state for simulation support
        if (RobotBase.isSimulation()) {
            simState = motor.getSimState();
        }
    }

    private void cacheStatusSignals() {
        positionSignal = motor.getPosition();
        velocitySignal = motor.getVelocity();
        accelerationSignal = motor.getAcceleration();
        statorCurrentSignal = motor.getStatorCurrent();
        dutyCycleSignal = motor.getDutyCycle();
        temperatureSignal = motor.getDeviceTemp();
        forwardLimitSignal = motor.getForwardLimit();
        reverseLimitSignal = motor.getReverseLimit();
        rotorPositionSignal = motor.getRotorPosition();
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
                motor.setControl(followerRequest.withLeaderID((int) value)
                        .withMotorAlignment(isInverted ? MotorAlignmentValue.Opposed : MotorAlignmentValue.Aligned));
                break;
            case MOTION_MAGIC_FOC_TORQUE:
                // TalonFXS doesn't support FOC torque mode - fallback to regular Motion Magic
                edu.wpi.first.wpilibj.DriverStation.reportWarning(
                    "MinionMotor: MOTION_MAGIC_FOC_TORQUE not supported. Falling back to MOTION_MAGIC.", false);
                motor.setControl(motionMagicRequest.withPosition(value).withEnableFOC(focFlag));
                break;
            case POSITION_FOC_TORQUE:
                edu.wpi.first.wpilibj.DriverStation.reportWarning(
                    "MinionMotor: POSITION_FOC_TORQUE not supported. Falling back to POSITION.", false);
                motor.setControl(positionRequest.withPosition(value).withEnableFOC(focFlag));
                break;
            case VELOCITY_FOC_TORQUE:
                edu.wpi.first.wpilibj.DriverStation.reportWarning(
                    "MinionMotor: VELOCITY_FOC_TORQUE not supported. Falling back to VELOCITY.", false);
                motor.setControl(velocityRequest.withVelocity(value).withEnableFOC(focFlag));
                break;
            case MOTION_MAGIC_EXPO:
                edu.wpi.first.wpilibj.DriverStation.reportWarning(
                    "MinionMotor: MOTION_MAGIC_EXPO not supported. Falling back to MOTION_MAGIC.", false);
                motor.setControl(motionMagicRequest.withPosition(value).withEnableFOC(focFlag));
                break;
            case MOTION_MAGIC_EXPO_TORQUE:
                edu.wpi.first.wpilibj.DriverStation.reportWarning(
                    "MinionMotor: MOTION_MAGIC_EXPO_TORQUE not supported. Falling back to MOTION_MAGIC.", false);
                motor.setControl(motionMagicRequest.withPosition(value).withEnableFOC(focFlag));
                break;
            default:
                throw new IllegalArgumentException("Unsupported control mode: " + mode);
        }
    }

    /**
     * Sets PID gains for closed-loop control.
     *
     * @param slotIdx PID slot index (0-2)
     * @param kP Proportional gain
     * @param kI Integral gain
     * @param kD Derivative gain
     * @param kF Feed-forward gain (maps to kV)
     */
    @Override
    public void setPID(int slotIdx, double kP, double kI, double kD, double kF) {
        switch (slotIdx) {
            case 0 -> { slot0_kP = kP; slot0_kI = kI; slot0_kD = kD; slot0_kV = kF; applySlot0(); }
            case 1 -> { slot1_kP = kP; slot1_kI = kI; slot1_kD = kD; slot1_kV = kF; applySlot1(); }
            case 2 -> { slot2_kP = kP; slot2_kI = kI; slot2_kD = kD; slot2_kV = kF; applySlot2(); }
            default -> edu.wpi.first.wpilibj.DriverStation.reportWarning(
                "MinionMotor: Invalid PID slot " + slotIdx + ". Valid range: 0-2.", false);
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
     * <p><strong>Important:</strong> When using kG, you must also call
     * {@link #configureGravity(GravityType)} to set the gravity compensation type.
     * Without it, kG defaults to Elevator_Static behavior.
     *
     * @param slotIdx The PID slot index to configure (0-2)
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
        switch (slotIdx) {
            case 0 -> {
                slot0_kP = kP; slot0_kI = kI; slot0_kD = kD;
                slot0_kV = kV; slot0_kS = kS; slot0_kA = kA; slot0_kG = kG;
                applySlot0();
            }
            case 1 -> {
                slot1_kP = kP; slot1_kI = kI; slot1_kD = kD;
                slot1_kV = kV; slot1_kS = kS; slot1_kA = kA; slot1_kG = kG;
                applySlot1();
            }
            case 2 -> {
                slot2_kP = kP; slot2_kI = kI; slot2_kD = kD;
                slot2_kV = kV; slot2_kS = kS; slot2_kA = kA; slot2_kG = kG;
                applySlot2();
            }
            default -> edu.wpi.first.wpilibj.DriverStation.reportWarning(
                "MinionMotor: Invalid PID slot " + slotIdx + ". Valid range: 0-2.", false);
        }
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
        extSensorToMechRatio = ratio;
        applyExternalFeedbackConfig();
    }

    /**
     * Configures a pulse-width absolute encoder connected to the TalonFXS data port.
     *
     * <p>Use for REV Through Bore encoders or PWM-output potentiometers wired directly
     * to the motor controller's data port.
     *
     * @param sensorToMechanismRatio Gear ratio from sensor to mechanism output
     * @param absoluteSensorOffset   Offset in rotations (0 to 1) to zero the sensor
     * @param discontinuityPoint     Wrap point in rotations (1.0 for full wrap, 0.5 for ±180°)
     */
    @Override
    public void configureExternalPulseWidthSensor(double sensorToMechanismRatio,
            double absoluteSensorOffset, double discontinuityPoint) {
        configureExternalPulseWidthSensor(sensorToMechanismRatio, absoluteSensorOffset, discontinuityPoint, false);
    }

    /**
     * Configures a pulse-width absolute encoder with explicit sensor phase.
     *
     * <p>Sensor phase controls whether the sensor counts in the same direction as the motor.
     * To determine phase: drive the motor positive with {@code opposeMotor=false}. If sensor
     * velocity is positive, the phase is correct. If negative, set {@code opposeMotor=true}.
     *
     * <p>Note: Phoenix 6 automatically inverts sensor direction when motor invert changes,
     * so this parameter only needs to reflect the mechanical sensor-to-shaft relationship.
     *
     * @param sensorToMechanismRatio Gear ratio from sensor to mechanism output
     * @param absoluteSensorOffset   Offset in rotations (0 to 1) to zero the sensor
     * @param discontinuityPoint     Wrap point in rotations (1.0 for full wrap, 0.5 for ±180°)
     * @param opposeMotor            true if the sensor counts opposite to motor rotation
     */
    @Override
    public void configureExternalPulseWidthSensor(double sensorToMechanismRatio,
            double absoluteSensorOffset, double discontinuityPoint, boolean opposeMotor) {
        extFeedbackSource = ExternalFeedbackSensorSourceValue.PulseWidth;
        extSensorToMechRatio = sensorToMechanismRatio;
        extAbsoluteSensorOffset = absoluteSensorOffset;
        extDiscontinuityPoint = discontinuityPoint;
        extSensorPhase = opposeMotor ? SensorPhaseValue.Opposed : SensorPhaseValue.Aligned;
        applyExternalFeedbackConfig();
    }

    /**
     * {@inheritDoc}
     *
     * <p>On TalonFXS, this is applied via {@link ClosedLoopGeneralConfigs}.
     * Other closed-loop general configs on the device are left untouched.
     */
    @Override
    public void configureContinuousWrap(boolean enabled) {
        var config = new ClosedLoopGeneralConfigs();
        config.ContinuousWrap = enabled;

        boolean success = applyConfigWithRetry(() -> configurator.apply(config));
        if (!success) {
            edu.wpi.first.wpilibj.DriverStation.reportError(
                "MinionMotor: Failed to apply ClosedLoopGeneralConfigs after retries", false);
        }
    }

    /**
     * Configures a quadrature encoder connected to the TalonFXS data port.
     *
     * @param sensorToMechanismRatio Gear ratio from sensor to mechanism output
     * @param edgesPerRotation       Quadrature edges per rotation (e.g., 8192 for REV Through Bore)
     */
    @Override
    public void configureExternalQuadratureSensor(double sensorToMechanismRatio, int edgesPerRotation) {
        configureExternalQuadratureSensor(sensorToMechanismRatio, edgesPerRotation, false);
    }

    /**
     * Configures a quadrature encoder with explicit sensor phase.
     *
     * @param sensorToMechanismRatio Gear ratio from sensor to mechanism output
     * @param edgesPerRotation       Quadrature edges per rotation
     * @param opposeMotor            true if the sensor counts opposite to motor rotation
     */
    @Override
    public void configureExternalQuadratureSensor(double sensorToMechanismRatio, int edgesPerRotation, boolean opposeMotor) {
        extFeedbackSource = ExternalFeedbackSensorSourceValue.Quadrature;
        extSensorToMechRatio = sensorToMechanismRatio;
        extQuadratureEdgesPerRotation = edgesPerRotation;
        extSensorPhase = opposeMotor ? SensorPhaseValue.Opposed : SensorPhaseValue.Aligned;
        applyExternalFeedbackConfig();
    }

    /**
     * Configures a remote CANcoder as the feedback sensor.
     *
     * @param cancoderId             CAN ID of the CANcoder
     * @param sensorToMechanismRatio Gear ratio from sensor to mechanism output
     */
    @Override
    public void configureRemoteCANcoder(int cancoderId, double sensorToMechanismRatio) {
        extFeedbackSource = ExternalFeedbackSensorSourceValue.RemoteCANcoder;
        extFeedbackRemoteSensorID = cancoderId;
        extSensorToMechRatio = sensorToMechanismRatio;
        applyExternalFeedbackConfig();
    }

    /**
     * Configures a fused CANcoder as the feedback sensor (requires Phoenix Pro).
     *
     * <p>Fuses the CANcoder with the commutation sensor for higher bandwidth position
     * and velocity feedback. Best for mechanisms requiring high-accuracy closed-loop control.
     *
     * @param cancoderId             CAN ID of the CANcoder
     * @param sensorToMechanismRatio Gear ratio from sensor to mechanism output
     * @param rotorToSensorRatio     Gear ratio from rotor to the CANcoder
     */
    @Override
    public void configureFusedCANcoder(int cancoderId, double sensorToMechanismRatio, double rotorToSensorRatio) {
        extFeedbackSource = ExternalFeedbackSensorSourceValue.FusedCANcoder;
        extFeedbackRemoteSensorID = cancoderId;
        extSensorToMechRatio = sensorToMechanismRatio;
        extRotorToSensorRatio = rotorToSensorRatio;
        applyExternalFeedbackConfig();
    }

    /**
     * Builds ExternalFeedbackConfigs from local state and applies it.
     * No refresh needed — we own the full state.
     */
    private void applyExternalFeedbackConfig() {
        var config = new ExternalFeedbackConfigs();
        config.ExternalFeedbackSensorSource = extFeedbackSource;
        config.FeedbackRemoteSensorID = extFeedbackRemoteSensorID;
        config.SensorToMechanismRatio = extSensorToMechRatio;
        config.RotorToSensorRatio = extRotorToSensorRatio;
        config.AbsoluteSensorOffset = extAbsoluteSensorOffset;
        config.AbsoluteSensorDiscontinuityPoint = extDiscontinuityPoint;
        config.QuadratureEdgesPerRotation = extQuadratureEdgesPerRotation;
        config.SensorPhase = extSensorPhase;

        boolean success = applyConfigWithRetry(() -> configurator.apply(config));
        if (!success) {
            edu.wpi.first.wpilibj.DriverStation.reportError(
                "MinionMotor: Failed to apply ExternalFeedbackConfigs after retries", false);
        }

        // Re-cache signals — feedback source changed, signal routing may differ
        positionSignal = motor.getPosition();
        velocitySignal = motor.getVelocity();
        rotorPositionSignal = motor.getRotorPosition();

        // Re-enable signals for the new feedback source
        positionSignal.setUpdateFrequency(50);
        velocitySignal.setUpdateFrequency(50);
        rotorPositionSignal.setUpdateFrequency(50);
        motor.optimizeBusUtilization();
    }

    @Override
    public void configureMotionMagic(AngularVelocity cruiseVelocity, AngularAcceleration acceleration, double jerkRPSPerSecPerSec) {
        mmCruiseVelocity = cruiseVelocity.in(RotationsPerSecond);
        mmAcceleration = acceleration.in(RotationsPerSecondPerSecond);
        mmJerk = jerkRPSPerSecPerSec;
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

        boolean success = applyConfigWithRetry(() -> configurator.apply(config));
        if (!success) {
            edu.wpi.first.wpilibj.DriverStation.reportError(
                "MinionMotor: Failed to apply Motion Magic config after retries", false);
        }
    }

    @Override
    public void configureCurrentLimits(Current stallLimit, Current freeLimit, double limitRpmThreshold) {
        double freeLimitAmps = freeLimit.in(Amps);
        currentStatorLimit = stallLimit.in(Amps);
        currentStatorEnabled = true;
        currentSupplyLimit = freeLimitAmps;
        currentSupplyEnabled = true;
        currentSupplyLowerLimit = freeLimitAmps * 0.8;
        currentSupplyLowerTime = 0.1;
        applyCurrentLimits();
    }

    /**
     * Builds CurrentLimitsConfigs from local state and applies it.
     * No refresh needed — we own the full state.
     */
    private void applyCurrentLimits() {
        var config = new CurrentLimitsConfigs();
        config.StatorCurrentLimit = currentStatorLimit;
        config.StatorCurrentLimitEnable = currentStatorEnabled;
        config.SupplyCurrentLimit = currentSupplyLimit;
        config.SupplyCurrentLimitEnable = currentSupplyEnabled;
        config.SupplyCurrentLowerLimit = currentSupplyLowerLimit;
        config.SupplyCurrentLowerTime = currentSupplyLowerTime;

        boolean success = applyConfigWithRetry(() -> configurator.apply(config));
        if (!success) {
            edu.wpi.first.wpilibj.DriverStation.reportError(
                "MinionMotor: Failed to apply current limits after retries", false);
        }
    }

    @Override
    public void configureSoftLimits(double forwardLimitRotations, double reverseLimitRotations, boolean enable) {
        softLimitForwardThreshold = forwardLimitRotations;
        softLimitReverseThreshold = reverseLimitRotations;
        softLimitForwardEnabled = enable;
        softLimitReverseEnabled = enable;
        applySoftLimits();
    }

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

        boolean success = applyConfigWithRetry(() -> configurator.apply(config));
        if (!success) {
            edu.wpi.first.wpilibj.DriverStation.reportError(
                "MinionMotor: Failed to apply soft limits after retries", false);
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
        config.withInverted(isInverted ? com.ctre.phoenix6.signals.InvertedValue.Clockwise_Positive :
                                          com.ctre.phoenix6.signals.InvertedValue.CounterClockwise_Positive)
              .withNeutralMode(isBrakeMode ? com.ctre.phoenix6.signals.NeutralModeValue.Brake :
                                              com.ctre.phoenix6.signals.NeutralModeValue.Coast);

        boolean success = applyConfigWithRetry(() -> configurator.apply(config));
        if (!success) {
            edu.wpi.first.wpilibj.DriverStation.reportError(
                "MinionMotor: Failed to apply motor output config after retries", false);
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

    // --- StatusSignal Getters ---
    // Every getter calls signal.refresh() to fetch the latest value from the local CAN cache.
    // DO NOT remove refresh() calls — without them, signals return stale startup values.

    @Override
    public double getPosition() {
        return positionSignal.refresh().getValueAsDouble();
    }

    @Override
    public double getRotorPosition() {
        return rotorPositionSignal.refresh().getValueAsDouble();
    }

    @Override
    public AngularVelocity getVelocity() {
        return RotationsPerSecond.of(velocitySignal.refresh().getValueAsDouble());
    }

    @Override
    public AngularAcceleration getAcceleration() {
        return RotationsPerSecondPerSecond.of(accelerationSignal.refresh().getValueAsDouble());
    }

    @Override
    public Current getCurrentDraw() {
        return Amps.of(statorCurrentSignal.refresh().getValueAsDouble());
    }

    @Override
    public double getOutputPercent() {
        return dutyCycleSignal.refresh().getValueAsDouble();
    }

    @Override
    public double getTemperature() {
        return temperatureSignal.refresh().getValueAsDouble();
    }

    @Override
    public boolean getForwardLimitSwitch() {
        return forwardLimitSignal.refresh().getValue() == ForwardLimitValue.ClosedToGround;
    }

    @Override
    public boolean getReverseLimitSwitch() {
        return reverseLimitSignal.refresh().getValue() == ReverseLimitValue.ClosedToGround;
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
        motor.setControl(followerRequest.withLeaderID(deviceID).withMotorAlignment(alignment));
    }

    @Override
    public void configureHardLimits(boolean enableForward, boolean enableReverse,
                                   double forwardResetValueRotations, double reverseResetValueRotations) {
        hardLimitForwardEnabled = enableForward;
        hardLimitReverseEnabled = enableReverse;
        hardLimitForwardAutoset = enableForward;
        hardLimitReverseAutoset = enableReverse;
        hardLimitForwardValue = forwardResetValueRotations;
        hardLimitReverseValue = reverseResetValueRotations;
        applyHardLimits();

        // Set up simulation state
        if (simState != null) {
            simState.setForwardLimit(enableForward);
            simState.setReverseLimit(enableReverse);
        }
    }

    /**
     * Builds HardwareLimitSwitchConfigs from local state and applies it.
     * No refresh needed — we own the full state.
     */
    private void applyHardLimits() {
        var config = new HardwareLimitSwitchConfigs();
        config.ForwardLimitEnable = hardLimitForwardEnabled;
        config.ForwardLimitAutosetPositionEnable = hardLimitForwardAutoset;
        config.ForwardLimitAutosetPositionValue = hardLimitForwardValue;
        config.ForwardLimitType = com.ctre.phoenix6.signals.ForwardLimitTypeValue.NormallyOpen;
        config.ReverseLimitEnable = hardLimitReverseEnabled;
        config.ReverseLimitAutosetPositionEnable = hardLimitReverseAutoset;
        config.ReverseLimitAutosetPositionValue = hardLimitReverseValue;
        config.ReverseLimitType = com.ctre.phoenix6.signals.ReverseLimitTypeValue.NormallyOpen;

        boolean success = applyConfigWithRetry(() -> configurator.apply(config));
        if (!success) {
            edu.wpi.first.wpilibj.DriverStation.reportError(
                "MinionMotor: Failed to apply hard limits after retries", false);
        }

        // Boost limit signal rate for active monitoring
        forwardLimitSignal.setUpdateFrequency(50);
        reverseLimitSignal.setUpdateFrequency(50);
    }

    @Override
    public void setPositionWithVelocityFF(double position, double velocity) {
        motor.setControl(positionRequest
            .withPosition(position)
            .withVelocity(velocity)
            .withEnableFOC(focFlag));
    }

    @Override
    public void setPositionWithVelocityFF(double position, double velocity, int slot) {
        motor.setControl(positionRequest
            .withPosition(position)
            .withVelocity(velocity)
            .withSlot(slot)
            .withEnableFOC(focFlag));
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
     * Builds Slot1Configs from local state and applies it.
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
        boolean success = applyConfigWithRetry(() -> configurator.apply(config));
        if (!success) {
            edu.wpi.first.wpilibj.DriverStation.reportError(
                "MinionMotor: Failed to apply Slot1 config after retries", false);
        }
    }

    /**
     * Builds Slot2Configs from local state and applies it.
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
        boolean success = applyConfigWithRetry(() -> configurator.apply(config));
        if (!success) {
            edu.wpi.first.wpilibj.DriverStation.reportError(
                "MinionMotor: Failed to apply Slot2 config after retries", false);
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
    public void refreshAllSignals() {
        BaseStatusSignal.refreshAll(
            positionSignal, velocitySignal, accelerationSignal,
            statorCurrentSignal, dutyCycleSignal, temperatureSignal,
            forwardLimitSignal, reverseLimitSignal, rotorPositionSignal
        );
    }

    @Override
    public boolean supportsControlMode(ControlMode mode) {
        // MinionMotor doesn't support CURRENT or Pro-only FOC torque/expo modes
        return mode != ControlMode.CURRENT
            && mode != ControlMode.MOTION_MAGIC_FOC_TORQUE
            && mode != ControlMode.POSITION_FOC_TORQUE
            && mode != ControlMode.VELOCITY_FOC_TORQUE
            && mode != ControlMode.MOTION_MAGIC_EXPO
            && mode != ControlMode.MOTION_MAGIC_EXPO_TORQUE;
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