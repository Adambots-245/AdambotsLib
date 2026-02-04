package com.adambots.lib.actuators;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import static edu.wpi.first.units.Units.*;
import edu.wpi.first.units.measure.*;

/**
 * Implementation for REV NEO and NEO Vortex motors using SPARK MAX controller
 */
@Logged
public class NEOMotor implements BaseMotor {
    @NotLogged
    private final SparkMax motor;

    @NotLogged
    private final RelativeEncoder encoder;

    @NotLogged
    private final SparkClosedLoopController closedLoopController;

    @NotLogged
    private final SparkMaxConfig config;

    @NotLogged
    private double feedForward = 0.0;

    // Conversion constants for velocity units (NEO uses RPM, BaseMotor interface uses RPS)
    private static final double RPM_TO_RPS = 1.0 / 60.0;
    private static final double RPS_TO_RPM = 60.0;

    /**
     * Constructs a NEOMotor object with default current limiting.
     *
     * <p><strong>NOTE:</strong> This constructor uses a default current limit of 40A.
     * For production code, use {@link #NEOMotor(int, boolean, int, boolean)} to specify
     * appropriate current limits for your application.
     *
     * @param portNum The port number for the motor.
     * @param brushed True if the motor is brushed, false if brushless.
     * @deprecated Use {@link #NEOMotor(int, boolean, int, boolean)} to explicitly specify
     *             current limits and inversion. This constructor exists for backwards compatibility.
     */
    @Deprecated
    public NEOMotor(int portNum, boolean brushed) {
        this(portNum, brushed, 40, false); // Default: 40A current limit, not inverted
    }

    /**
     * Constructs a NEOMotor object.
     *
     * @param portNum The port number for the motor.
     * @param brushed True if the motor is brushed, false if brushless.
     * @param supplyCurrentLimit The supply current limit for the motor (amperes).
     *                           CRITICAL: Current limiting is essential for NEO motors
     *                           due to their low internal resistance.
     * @param inverted True to invert the motor direction, false for normal operation.
     */
    public NEOMotor(int portNum, boolean brushed, int supplyCurrentLimit, boolean inverted) {
        motor = new SparkMax(portNum, brushed ? MotorType.kBrushed : MotorType.kBrushless);
        config = new SparkMaxConfig();

        // Get the built-in encoder
        encoder = motor.getEncoder();

        // Get the closed loop controller
        closedLoopController = motor.getClosedLoopController();

        // Default configuration with CRITICAL current limiting for NEO safety
        config.voltageCompensation(12.0);
        config.smartCurrentLimit(supplyCurrentLimit);
        config.inverted(inverted);

        // PERSIST on initial setup only (survive brownouts)
        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    /**
     * Sets the motor control mode and value.
     *
     * @param mode  The control mode to set.
     * @param value The value to set for the control mode.
     */
    @Override
    public void set(ControlMode mode, double value) {
        switch (mode) {
            case PERCENT_OUTPUT:
                motor.set(value);
                break;
            case POSITION:
                closedLoopController.setSetpoint(value, SparkBase.ControlType.kPosition);
                break;
            case VELOCITY:
                // Convert RPS to RPM for NEO controller
                // BREAKING CHANGE (v2026.2.0): Now accepts RPS instead of RPM
                double velocityRPM = value * RPS_TO_RPM;
                closedLoopController.setSetpoint(velocityRPM, SparkBase.ControlType.kVelocity);
                break;
            case VOLTAGE:
                motor.setVoltage(value);
                break;
            case CURRENT:
                closedLoopController.setSetpoint(value, SparkBase.ControlType.kCurrent);
                break;
            case MOTION_MAGIC:
                // Use MAXMotion Position Control for motion profiling
                closedLoopController.setSetpoint(value, SparkBase.ControlType.kMAXMotionPositionControl);
                break;
            case MOTION_MAGIC_FOC_TORQUE:
                // NEO motors don't support FOC torque mode - fallback to MOTION_MAGIC
                edu.wpi.first.wpilibj.DriverStation.reportWarning(
                    "NEOMotor: MOTION_MAGIC_FOC_TORQUE not supported. Falling back to MOTION_MAGIC.", false);
                closedLoopController.setSetpoint(value, SparkBase.ControlType.kMAXMotionPositionControl);
                break;
            case FOLLOWER:
                config.follow((int) value);
                // CRITICAL FIX: Must call configure() to apply follower
                // PERSIST because follower is one-time setup
                motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
                break;
        }
    }

    /**
     * Sets the motor speed.
     *
     * @param speed The speed to set for the motor.
     */
    @Override
    public void set(double speed) {
        motor.set(speed);
    }

    /**
     * Sets the feed-forward value for closed-loop control modes.
     *
     * @param value The feed-forward value to use in closed-loop control.
     */
    public void setFeedForward(double value) {
        this.feedForward = value;
    }

    /**
     * Sets the PIDF constants for the motor's closed loop control.
     *
     * @param slotIdx The slot index for the PIDF constants.
     * @param kP      The proportional gain.
     * @param kI      The integral gain.
     * @param kD      The derivative gain.
     * @param kF      The feedforward gain.
     */
    @Override
    public void setPID(int slotIdx, double kP, double kI, double kD, double kF) {
        config.closedLoop.pid(kP, kI, kD);
        config.closedLoop.feedForward.kV(kF);
        // NO PERSIST during runtime - avoid blocking flash writes
        motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    /**
     * Configures motion magic settings for motion profiling.
     *
     * @param cruiseVelocity Maximum velocity during motion
     * @param acceleration Acceleration rate
     * @param jerkRPSPerSecPerSec Jerk (rate of acceleration change) in rotations per second³
     */
    @Override
    public void configureMotionMagic(AngularVelocity cruiseVelocity, AngularAcceleration acceleration, double jerkRPSPerSecPerSec) {
        // Convert to RPM for NEO controller
        config.closedLoop.maxMotion
                .cruiseVelocity(cruiseVelocity.in(RPM))
                .maxAcceleration(acceleration.in(RotationsPerSecondPerSecond) * RPS_TO_RPM);
        // NO PERSIST during runtime - avoid blocking flash writes
        motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    /**
     * Configures current limits for the motor.
     *
     * @param stallLimit The stall current limit.
     * @param freeLimit  The free current limit.
     * @param limitRpmThreshold The RPM limit for current limiting.
     */
    @Override
    public void configureCurrentLimits(Current stallLimit, Current freeLimit, double limitRpmThreshold) {
        config.smartCurrentLimit((int) stallLimit.in(Amps), (int) freeLimit.in(Amps), (int) limitRpmThreshold);
        // NO PERSIST during runtime - avoid blocking flash writes
        motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    /**
     * Configures soft limits for the motor.
     *
     * @param forwardLimit  The forward soft limit value.
     * @param reverseLimit  The reverse soft limit value.
     * @param enable        True to enable soft limits, false to disable.
     */
    @Override
    public void configureSoftLimits(double forwardLimitRotations, double reverseLimitRotations, boolean enable) {
        config.softLimit.forwardSoftLimitEnabled(enable).forwardSoftLimit(forwardLimitRotations);
        config.softLimit.reverseSoftLimitEnabled(enable).reverseSoftLimit(reverseLimitRotations);
        // NO PERSIST during runtime - avoid blocking flash writes
        motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    /**
     * Enables or disables soft limits for the motor.
     *
     * @param enable True to enable soft limits, false to disable.
     */
    @Override
    public void enableSoftLimits(boolean enable) {
        config.softLimit.forwardSoftLimitEnabled(enable);
        config.softLimit.reverseSoftLimitEnabled(enable);
        // NO PERSIST during runtime - avoid blocking flash writes
        motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    /**
     * Sets the inversion of the motor.
     *
     * @param inverted True to invert the motor, false to not invert.
     */
    @Override
    public void setInverted(boolean inverted) {
        config.inverted(inverted);
        // NO PERSIST during runtime - avoid blocking flash writes
        motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    /**
     * Sets the brake mode of the motor.
     *
     * @param brake True to set brake mode, false to set coast mode.
     */
    @Override
    public void setBrakeMode(boolean brake) {
        config.idleMode(brake ? IdleMode.kBrake : IdleMode.kCoast);
        // NO PERSIST during runtime - avoid blocking flash writes
        motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    /**
     * Sets the encoder position to a specific value in rotations.
     *
     * @param rotations The position to set the encoder to in rotations.
     */
    @Override
    public void setPosition(double rotations) {
        encoder.setPosition(rotations);
    }

    /**
     * Enables voltage compensation for the motor.
     *
     * @param nominalVoltage The voltage to compensate to.
     */
    @Override
    public void enableVoltageCompensation(Voltage nominalVoltage) {
        config.voltageCompensation(nominalVoltage.in(Volts));
        // NO PERSIST during runtime - avoid blocking flash writes
        motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    /**
     * Gets the current position of the motor's encoder in rotations.
     *
     * @return The current position of the encoder in rotations.
     */
    @Override
    public double getPosition() {
        return encoder.getPosition();
    }

    /**
     * Gets the current velocity of the motor.
     *
     * @return The current velocity of the motor
     */
    @Override
    public AngularVelocity getVelocity() {
        // Convert RPM to RPS for consistent interface with Phoenix 6 motors
        return RotationsPerSecond.of(encoder.getVelocity() * RPM_TO_RPS);
    }

    /**
     * Gets the current acceleration of the motor.
     * <p>
     * Note: NEO motors do not directly support acceleration measurement.
     *
     * @return Zero acceleration (not supported)
     */
    @Override
    public AngularAcceleration getAcceleration() {
        // NEO does not directly support acceleration measurement
        return RotationsPerSecondPerSecond.of(0.0);
    }

    /**
     * Gets the current draw of the motor.
     *
     * @return The current draw of the motor
     */
    @Override
    public Current getCurrentDraw() {
        return Amps.of(motor.getOutputCurrent());
    }

    /**
     * Gets the output percentage of the motor controller.
     *
     * @return The output percentage as a decimal value between -1 and 1.
     */
    @Override
    public double getOutputPercent() {
        return motor.get();
    }

    /**
     * Gets the temperature of the motor in degrees Celsius.
     *
     * @return The temperature of the motor in degrees Celsius.
     */
    @Override
    public double getTemperature() {
        return motor.getMotorTemperature();
    }

    /**
     * Gets the state of the forward limit switch.
     *
     * @return True if the forward limit switch is pressed, false otherwise.
     */
    @Override
    public boolean getForwardLimitSwitch() {
        return motor.getForwardLimitSwitch().isPressed();
    }

    /**
     * Gets the state of the reverse limit switch.
     *
     * @return True if the reverse limit switch is pressed, false otherwise.
     */
    @Override
    public boolean getReverseLimitSwitch() {
        return motor.getReverseLimitSwitch().isPressed();
    }

    /**
     * Sets the motor to strictly follow another motor controller.
     *
     * @param deviceID The device ID of the motor controller to follow.
     */
    @Override
    public void setStrictFollower(int deviceID) {
        config.follow(deviceID);
        // PERSIST because follower is one-time setup that should survive brownouts
        motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    }

    /**
     * Configures hard limits using limit switches.
     *
     * @param enableForward  True to enable the forward limit switch, false otherwise.
     * @param enableReverse  True to enable the reverse limit switch, false otherwise.
     */
    @Override
    public void configureHardLimits(boolean enableForward, boolean enableReverse,
                                   double forwardResetValueRotations, double reverseResetValueRotations) {
        config.limitSwitch.setSparkMaxDataPortConfig()
                .forwardLimitSwitchEnabled(enableForward)
                .reverseLimitSwitchEnabled(enableReverse);

        if (enableForward) {
            config.limitSwitch.forwardLimitSwitchType(Type.kNormallyClosed);
        }
        if (enableReverse) {
            config.limitSwitch.reverseLimitSwitchType(Type.kNormallyClosed);
        }
        // NO PERSIST during runtime - avoid blocking flash writes
        motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    @Override
    public boolean supportsControlMode(ControlMode mode) {
        // NEOMotor doesn't support MOTION_MAGIC_FOC_TORQUE
        return mode != ControlMode.MOTION_MAGIC_FOC_TORQUE;
    }

    @Override
    public String getMotorType() {
        return "NEOMotor (SPARK MAX)";
    }
}