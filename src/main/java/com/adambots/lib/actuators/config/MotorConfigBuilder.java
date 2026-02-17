package com.adambots.lib.actuators.config;

import com.adambots.lib.actuators.BaseMotor;

import static edu.wpi.first.units.Units.*;
import edu.wpi.first.units.measure.*;

/**
 * Fluent builder for motor configuration.
 *
 * <p>This builder provides a clean, readable API for configuring motors with multiple
 * settings in a single chain. It's similar to the StateMachine builder pattern and
 * makes motor configuration less error-prone.
 *
 * <p><strong>Usage Example:</strong>
 * <pre>{@code
 * // Configure a drive motor with all common settings
 * motor.configure()
 *     .pid(0.1, 0.0, 0.05, 0.0)
 *     .currentLimits(40, 60, 5000)
 *     .motionMagic(50.0, 100.0, 200.0)
 *     .brakeMode(true)
 *     .voltageCompensation(Volts.of(12.0))
 *     .apply();
 *
 * // Minimal configuration for a simple mechanism
 * motor.configure()
 *     .currentLimits(20, 40, 3000)
 *     .brakeMode(true)
 *     .apply();
 * }</pre>
 *
 * <p><strong>Benefits:</strong>
 * <ul>
 *   <li>Reads like English - easy to understand</li>
 *   <li>All parameters have clear names with units</li>
 *   <li>Optional parameters - only configure what you need</li>
 *   <li>Type-safe - compiler catches errors</li>
 *   <li>Self-documenting - IDE shows what each parameter means</li>
 * </ul>
 *
 * @see BaseMotor#configure()
 */
public class MotorConfigBuilder {
    private final BaseMotor motor;
    private PIDConfig pidConfig;
    private CurrentLimitConfig currentLimitConfig;
    private MotionMagicConfig motionMagicConfig;
    private Boolean brakeMode;
    private Boolean voltageCompensation;
    private double voltageCompensationValue = 12.0;
    private BaseMotor.GravityType gravityType;
    private Boolean inverted;

    /**
     * Creates a new motor configuration builder.
     *
     * <p>Typically not called directly - use {@link BaseMotor#configure()} instead.
     *
     * @param motor The motor to configure
     */
    public MotorConfigBuilder(BaseMotor motor) {
        this.motor = motor;
    }

    /**
     * Configures PID gains for closed-loop control.
     *
     * <p>Uses slot 0 by default. For multiple PID slots, call this method multiple
     * times with different slot indices via {@link #pid(int, double, double, double, double)}.
     *
     * <p><strong>Quick Start Values:</strong>
     * <ul>
     *   <li><strong>Position:</strong> kP=0.1, kI=0, kD=0.05, kF=0</li>
     *   <li><strong>Velocity:</strong> kP=0.0001, kI=0, kD=0.00005, kF=0.000176</li>
     * </ul>
     *
     * @param kP Proportional gain
     * @param kI Integral gain
     * @param kD Derivative gain
     * @param kF Feed-forward gain
     * @return This builder for chaining
     */
    public MotorConfigBuilder pid(double kP, double kI, double kD, double kF) {
        this.pidConfig = new PIDConfig(0, kP, kI, kD, kF);
        return this;
    }

    /**
     * Configures PID gains for a specific slot.
     *
     * @param slot PID gain slot index (0-2 or 0-3 depending on motor controller)
     * @param kP Proportional gain
     * @param kI Integral gain
     * @param kD Derivative gain
     * @param kF Feed-forward gain
     * @return This builder for chaining
     */
    public MotorConfigBuilder pid(int slot, double kP, double kI, double kD, double kF) {
        this.pidConfig = new PIDConfig(slot, kP, kI, kD, kF);
        return this;
    }

    /**
     * Configures current limits to protect motor and battery.
     *
     * <p><strong>Typical Values:</strong>
     * <ul>
     *   <li><strong>Drive motors:</strong> stallAmps=40-60, freeAmps=80-120</li>
     *   <li><strong>Mechanism motors:</strong> stallAmps=20-40, freeAmps=40-80</li>
     * </ul>
     *
     * @param stallLimit Current limit when motor is under heavy load
     * @param freeLimit Current limit when motor is spinning freely
     * @param limitRpm RPM threshold below which stall limit applies
     * @return This builder for chaining
     */
    public MotorConfigBuilder currentLimits(Current stallLimit, Current freeLimit, int limitRpm) {
        this.currentLimitConfig = new CurrentLimitConfig(stallLimit, freeLimit, limitRpm);
        return this;
    }

    /**
     * Configures current limits to protect motor and battery (backwards compatible overload).
     *
     * @param stallAmps Current limit when motor is under heavy load (amperes)
     * @param freeAmps Current limit when motor is spinning freely (amperes)
     * @param limitRpm RPM threshold below which stall limit applies
     * @return This builder for chaining
     */
    public MotorConfigBuilder currentLimits(double stallAmps, double freeAmps, int limitRpm) {
        this.currentLimitConfig = CurrentLimitConfig.fromAmps(stallAmps, freeAmps, limitRpm);
        return this;
    }

    /**
     * Configures Motion Magic motion profiling for smooth position control.
     *
     * <p><strong>Parameter Guidance:</strong>
     * <ul>
     *   <li><strong>cruiseVelocity:</strong> 70-80% of max speed</li>
     *   <li><strong>acceleration:</strong> 2-4x cruise velocity</li>
     *   <li><strong>jerkRPSPerSecPerSec:</strong> 5-10x acceleration, or 0 to disable</li>
     * </ul>
     *
     * @param cruiseVelocity Maximum velocity
     * @param acceleration Acceleration rate
     * @param jerkRPSPerSecPerSec Jerk in rotations per second³ (0 to disable)
     * @return This builder for chaining
     */
    public MotorConfigBuilder motionMagic(AngularVelocity cruiseVelocity, AngularAcceleration acceleration, double jerkRPSPerSecPerSec) {
        this.motionMagicConfig = new MotionMagicConfig(cruiseVelocity, acceleration, jerkRPSPerSecPerSec);
        return this;
    }

    /**
     * Configures Motion Magic motion profiling for smooth position control (backwards compatible overload).
     *
     * @param cruiseVelRPS Maximum velocity in rotations per second
     * @param accelRPSPerSec Acceleration in rotations per second²
     * @param jerkRPSPerSecPerSec Jerk in rotations per second³ (0 to disable)
     * @return This builder for chaining
     */
    public MotorConfigBuilder motionMagic(double cruiseVelRPS, double accelRPSPerSec, double jerkRPSPerSecPerSec) {
        this.motionMagicConfig = MotionMagicConfig.fromRPS(cruiseVelRPS, accelRPSPerSec, jerkRPSPerSecPerSec);
        return this;
    }

    /**
     * Configures motor neutral mode (brake or coast).
     *
     * <p><strong>When to use:</strong>
     * <ul>
     *   <li><strong>Brake mode (true):</strong> Mechanisms that hold position (arms, elevators)</li>
     *   <li><strong>Coast mode (false):</strong> Drive trains that should coast to stop</li>
     * </ul>
     *
     * @param brake True for brake mode, false for coast mode
     * @return This builder for chaining
     */
    public MotorConfigBuilder brakeMode(boolean brake) {
        this.brakeMode = brake;
        return this;
    }

    /**
     * Enables voltage compensation for consistent motor performance.
     *
     * <p><strong>IMPORTANT - Phoenix 6 Motors (TalonFX, Minion):</strong>
     * This method does NOT work correctly for Phoenix 6 motors. For Phoenix 6:
     * <ul>
     *   <li>Use voltage-based control modes (VoltageOut, PositionVoltage, VelocityVoltage, MotionMagicVoltage)</li>
     *   <li>These modes explicitly request voltage output and handle compensation internally</li>
     *   <li>DO NOT use duty cycle modes (DutyCycleOut) if you want voltage compensation</li>
     * </ul>
     *
     * <p><strong>REV Motors (NEO, NEO 550):</strong>
     * This method works correctly for REV motors and properly compensates for battery droop.
     *
     * @param voltage Nominal voltage to compensate to (typically 12V)
     * @return This builder for chaining
     */
    public MotorConfigBuilder voltageCompensation(Voltage voltage) {
        this.voltageCompensation = true;
        this.voltageCompensationValue = voltage.in(Volts);
        return this;
    }

    /**
     * Configures gravity compensation type.
     *
     * <p>Must be used with kG set via the extended setPID method for the
     * compensation to take effect.
     *
     * <p><strong>Example:</strong>
     * <pre>{@code
     * motor.configure()
     *     .gravity(BaseMotor.GravityType.ARM_COSINE)
     *     .brakeMode(true)
     *     .apply();
     * }</pre>
     *
     * @param type ARM_COSINE for arms, ELEVATOR_STATIC for elevators
     * @return This builder for chaining
     */
    /**
     * Configures motor direction inversion.
     *
     * <p><strong>When to use:</strong>
     * <ul>
     *   <li>When the motor needs to spin in the opposite direction from default</li>
     *   <li>To match motor direction with mechanism movement direction</li>
     * </ul>
     *
     * <p><strong>Example:</strong>
     * <pre>{@code
     * motor.configure()
     *     .inverted(true)
     *     .brakeMode(true)
     *     .apply();
     * }</pre>
     *
     * @param inverted True to invert motor direction, false for default direction
     * @return This builder for chaining
     */
    public MotorConfigBuilder inverted(boolean inverted) {
        this.inverted = inverted;
        return this;
    }

    public MotorConfigBuilder gravity(BaseMotor.GravityType type) {
        this.gravityType = type;
        return this;
    }

    /**
     * Applies all configured settings to the motor.
     *
     * <p>This method should be called last in the builder chain. It applies all the
     * configurations that were set via the builder methods.
     *
     * <p><strong>Example:</strong>
     * <pre>{@code
     * motor.configure()
     *     .pid(0.1, 0, 0.05, 0)
     *     .currentLimits(40, 60, 5000)
     *     .brakeMode(true)
     *     .apply();  // Applies all settings
     * }</pre>
     */
    public void apply() {
        if (pidConfig != null) {
            motor.setPID(pidConfig.slot(), pidConfig.kP(), pidConfig.kI(), pidConfig.kD(), pidConfig.kF());
        }
        if (currentLimitConfig != null) {
            motor.configureCurrentLimits(
                currentLimitConfig.stallLimit(),
                currentLimitConfig.freeLimit(),
                currentLimitConfig.limitRpmThreshold()
            );
        }
        if (motionMagicConfig != null) {
            motor.configureMotionMagic(
                motionMagicConfig.cruiseVelocity(),
                motionMagicConfig.acceleration(),
                motionMagicConfig.jerkRPSPerSecPerSec()
            );
        }
        if (inverted != null) {
            motor.setInverted(inverted);
        }
        if (brakeMode != null) {
            motor.setBrakeMode(brakeMode);
        }
        if (voltageCompensation != null && voltageCompensation) {
            motor.enableVoltageCompensation(Volts.of(voltageCompensationValue));
        }
        if (gravityType != null) {
            motor.configureGravity(gravityType);
        }
    }
}
