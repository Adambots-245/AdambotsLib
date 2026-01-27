package com.adambots.lib.actuators;

import static edu.wpi.first.units.Units.*;
import edu.wpi.first.units.measure.*;

/**
 * Base interface for motor controllers in FRC robotics.
 *
 * <p>This interface provides a unified API for controlling different motor types
 * (NEO/SparkMax, Falcon/TalonFX, Kraken/TalonFXS) with consistent methods for
 * configuration and control.
 *
 * <p><strong>Supported Features:</strong>
 * <ul>
 *   <li>Multiple control modes (open-loop, closed-loop position/velocity, motion profiling)</li>
 *   <li>PID control with configurable gains</li>
 *   <li>Motion Magic for smooth trapezoidal motion profiles</li>
 *   <li>Current limiting and voltage compensation</li>
 *   <li>Software and hardware limit switches</li>
 *   <li>Follower mode for synchronized multi-motor control</li>
 * </ul>
 *
 * <p><strong>Usage Example:</strong>
 * <pre>{@code
 * BaseMotor shooterMotor = new NEOMotor(5, false, 40, false);
 * shooterMotor.setPID(0, 0.0001, 0, 0.00005, 0.000176);
 * shooterMotor.set(ControlMode.VELOCITY, 83.33);  // 83.33 RPS (5000 RPM)
 * }</pre>
 *
 * <p><strong>BREAKING CHANGE (v2026.2.0):</strong> Velocity units standardized to RPS.
 * <p>All motor types now use rotations per second (RPS) for velocity instead of RPM.
 * If you used NEOMotor with velocity control, divide velocity values by 60:
 * <ul>
 *   <li><strong>Old:</strong> {@code motor.set(ControlMode.VELOCITY, 5000)} // 5000 RPM</li>
 *   <li><strong>New:</strong> {@code motor.set(ControlMode.VELOCITY, 83.33)} // 83.33 RPS</li>
 *   <li><strong>Old:</strong> {@code motor.getVelocity()} returned 5000 (RPM)</li>
 *   <li><strong>New:</strong> {@code motor.getVelocity()} returns 83.33 (RPS)</li>
 * </ul>
 *
 * <p><strong>Motor Interchangeability:</strong>
 * <p>You can now swap motor types (NEO, Falcon, Kraken, Minion) without code changes:
 * <ul>
 *   <li>All motors use consistent velocity units (RPS)</li>
 *   <li>Unsupported control modes fall back gracefully with warnings</li>
 *   <li>Use {@link #supportsControlMode(ControlMode)} to check capabilities at runtime</li>
 * </ul>
 *
 * @see BaseActuator
 * @see NEOMotor
 * @see TalonFXMotor
 * @see MinionMotor
 */
public interface BaseMotor extends BaseActuator{

    /**
     * Control modes for motor operation.
     *
     * <p>Each mode interprets the setpoint value differently:
     * <ul>
     *   <li><strong>PERCENT_OUTPUT:</strong> Open-loop duty cycle control (-1.0 to 1.0)</li>
     *   <li><strong>POSITION:</strong> Closed-loop position control (rotations)</li>
     *   <li><strong>VELOCITY:</strong> Closed-loop velocity control (rotations per second)</li>
     *   <li><strong>VOLTAGE:</strong> Direct voltage output (-12.0 to 12.0 volts)</li>
     *   <li><strong>CURRENT:</strong> Torque current control (amperes, if supported)</li>
     *   <li><strong>MOTION_MAGIC:</strong> Position control with automatic motion profiling</li>
     *   <li><strong>MOTION_MAGIC_FOC_TORQUE:</strong> Motion Magic using field-oriented
     *       control (TalonFX/Minion only)</li>
     *   <li><strong>FOLLOWER:</strong> Follow another motor's output</li>
     * </ul>
     *
     * <p><strong>Compatibility Matrix:</strong>
     * <table border="1" cellpadding="3">
     *   <tr><th>Mode</th><th>NEO</th><th>TalonFX (Falcon)</th><th>TalonFX (Kraken)</th><th>Minion</th></tr>
     *   <tr><td>PERCENT_OUTPUT</td><td>✓</td><td>✓</td><td>✓</td><td>✓</td></tr>
     *   <tr><td>POSITION</td><td>✓</td><td>✓</td><td>✓</td><td>✓</td></tr>
     *   <tr><td>VELOCITY (RPS)</td><td>✓</td><td>✓</td><td>✓</td><td>✓</td></tr>
     *   <tr><td>VOLTAGE</td><td>✓</td><td>✓</td><td>✓</td><td>✓</td></tr>
     *   <tr><td>CURRENT</td><td>✓</td><td>⚠ Fallback</td><td>✓</td><td>⚠ Fallback</td></tr>
     *   <tr><td>MOTION_MAGIC</td><td>✓</td><td>✓</td><td>✓</td><td>✓</td></tr>
     *   <tr><td>MOTION_MAGIC_FOC_TORQUE</td><td>⚠ Fallback</td><td>✓</td><td>✓</td><td>⚠ Fallback</td></tr>
     *   <tr><td>FOLLOWER</td><td>✓</td><td>✓</td><td>✓</td><td>✓</td></tr>
     * </table>
     * <p>✓ = Fully supported | ⚠ = Supported with fallback (logs warning, uses alternate mode)
     *
     * <p><strong>BREAKING CHANGE (v2026.2.0):</strong> Velocity units standardized to
     * rotations per second (RPS) for all motor types. Previously NEOMotor used RPM.
     *
     * @see #set(ControlMode, double)
     * @see #supportsControlMode(ControlMode)
     */
    public enum ControlMode {
        PERCENT_OUTPUT,
        POSITION,
        VELOCITY,
        VOLTAGE,
        CURRENT,
        MOTION_MAGIC,
        MOTION_MAGIC_FOC_TORQUE,
        FOLLOWER
    }

    /**
     * Sets the motor control mode and output value.
     *
     * <p>The interpretation of the value parameter depends on the selected control mode:
     * <ul>
     *   <li><strong>PERCENT_OUTPUT:</strong> -1.0 to 1.0 (percent of max voltage)</li>
     *   <li><strong>POSITION:</strong> Target position in rotations</li>
     *   <li><strong>VELOCITY:</strong> Target velocity in rotations per second (RPS)</li>
     *   <li><strong>VOLTAGE:</strong> Output voltage in volts (-12.0 to 12.0)</li>
     *   <li><strong>CURRENT:</strong> Target current in amperes (if supported by motor)</li>
     *   <li><strong>MOTION_MAGIC:</strong> Target position in rotations with automatic profiling</li>
     *   <li><strong>MOTION_MAGIC_FOC_TORQUE:</strong> Target position in rotations using FOC</li>
     *   <li><strong>FOLLOWER:</strong> Device ID of motor to follow (not a setpoint value)</li>
     * </ul>
     *
     * <p><strong>Examples:</strong>
     * <pre>{@code
     * motor.set(ControlMode.PERCENT_OUTPUT, 0.5);    // 50% forward
     * motor.set(ControlMode.POSITION, 10.0);         // Move to 10 rotations
     * motor.set(ControlMode.VELOCITY, 83.33);        // 83.33 RPS (equivalent to 5000 RPM)
     * motor.set(ControlMode.MOTION_MAGIC, 25.5);     // Smooth move to 25.5 rotations
     * }</pre>
     *
     * @param mode Control mode to use
     * @param value Setpoint value (interpretation depends on mode - see above)
     * @throws IllegalArgumentException if mode is not supported by this motor implementation
     */
    void set(ControlMode mode, double value);


    /**
     * Configures PID gains for closed-loop control.
     *
     * <p>Most motor controllers support multiple PID gain slots (0-2 or 0-3) for different
     * control scenarios. For example, use slot 0 for position control and slot 1 for
     * velocity control with different tuning.
     *
     * <p><strong>PID Tuning Guide:</strong>
     * <ul>
     *   <li><strong>kP:</strong> Proportional gain - primary error correction (start with 0.1-1.0)</li>
     *   <li><strong>kI:</strong> Integral gain - eliminates steady-state error (usually 0 or very small)</li>
     *   <li><strong>kD:</strong> Derivative gain - dampens oscillation (typically 0.1-10x kP)</li>
     *   <li><strong>kF:</strong> Feed-forward gain - compensates for known system dynamics (velocity: ~0.001-0.01)</li>
     * </ul>
     *
     * <p><strong>Example:</strong>
     * <pre>{@code
     * // Position control gains
     * motor.setPID(0, 0.1, 0.0, 0.05, 0.0);
     *
     * // Velocity control gains with feed-forward
     * motor.setPID(1, 0.0001, 0, 0.00005, 0.000176);
     * }</pre>
     *
     * @param slotIdx PID gain slot index (typically 0-2, varies by motor controller)
     * @param kP Proportional gain coefficient
     * @param kI Integral gain coefficient
     * @param kD Derivative gain coefficient
     * @param kF Feed-forward gain coefficient
     */
    void setPID(int slotIdx, double kP, double kI, double kD, double kF);

    /**
     * Configures Motion Magic motion profiling parameters.
     *
     * <p>Motion Magic generates smooth trapezoidal motion profiles that respect maximum
     * velocity and acceleration constraints. This prevents jerky motion and reduces mechanical
     * stress on the robot.
     *
     * <p><strong>Parameter Guidance:</strong>
     * <ul>
     *   <li><strong>cruiseVelocity:</strong> Set to 70-80% of mechanism's maximum speed</li>
     *   <li><strong>acceleration:</strong> Typically 2-4x cruise velocity</li>
     *   <li><strong>jerkRPSPerSecPerSec:</strong> Optional smoothing (0 = disabled, or 5-10x acceleration)</li>
     * </ul>
     *
     * <p><strong>Example:</strong>
     * <pre>{@code
     * import static edu.wpi.first.units.Units.*;
     *
     * // Elevator motion profile
     * motor.configureMotionMagic(
     *     RotationsPerSecond.of(50),    // 50 RPS cruise velocity
     *     RotationsPerSecondPerSecond.of(150),   // 150 RPS² acceleration
     *     500.0    // 500 RPS³ jerk limiting
     * );
     *
     * // Then command to position:
     * motor.set(ControlMode.MOTION_MAGIC, 25.0);  // Smooth move to 25 rotations
     * }</pre>
     *
     * @param cruiseVelocity Maximum velocity during motion
     * @param acceleration Acceleration rate
     * @param jerkRPSPerSecPerSec Jerk (rate of acceleration change) in rotations per second³ (0 to disable)
     */
    void configureMotionMagic(AngularVelocity cruiseVelocity, AngularAcceleration acceleration, double jerkRPSPerSecPerSec);

    /**
     * Configures current limiting to protect the motor and battery.
     *
     * <p>Current limits prevent motor overheating and excessive battery drain. Modern motor
     * controllers use "smart" current limiting with separate limits for stalled and free-spinning
     * conditions.
     *
     * <p><strong>Typical Values:</strong>
     * <ul>
     *   <li><strong>Drive motors:</strong> stall=40-60A, free=80-120A</li>
     *   <li><strong>Mechanism motors:</strong> stall=20-40A, free=40-80A</li>
     *   <li><strong>Small actuators:</strong> stall=10-20A, free=20-40A</li>
     * </ul>
     *
     * <p><strong>Example:</strong>
     * <pre>{@code
     * // NEO on drive motor - conservative limits
     * motor.configureCurrentLimits(40, 60, 5000);
     *
     * // TalonFX on shooter - allow higher free-spin current
     * motor.configureCurrentLimits(60, 120, 3000);
     * }</pre>
     *
     * @param stallLimitAmps Current limit when motor is under heavy load (amperes)
     * @param freeLimitAmps Current limit when motor is spinning freely (amperes)
     * @param limitRpmThreshold RPM threshold below which stall limit applies
     */
    void configureCurrentLimits(double stallLimitAmps, double freeLimitAmps, double limitRpmThreshold);

    /**
     * Configures software (virtual) position limits to prevent mechanism damage.
     *
     * <p>Soft limits prevent the motor from commanding positions beyond safe mechanical
     * bounds without requiring physical limit switches. The motor will automatically
     * stop when reaching these limits.
     *
     * <p><strong>Best Practices:</strong>
     * <ul>
     *   <li>Set limits slightly inside mechanical hard stops to avoid crashes</li>
     *   <li>Use positive values for forward limit, negative for reverse</li>
     *   <li>Test limits carefully during mechanism development</li>
     *   <li>Consider using hardware limit switches as backup safety</li>
     * </ul>
     *
     * <p><strong>Example:</strong>
     * <pre>{@code
     * // Elevator with 100 rotation range
     * motor.configureSoftLimits(
     *     98.0,    // Forward limit at 98 rotations (2 rotation safety margin)
     *     -2.0,    // Reverse limit at -2 rotations (allow slight negative)
     *     true     // Enable limits
     * );
     * }</pre>
     *
     * @param forwardLimitRotations Forward soft limit position in rotations from zero
     * @param reverseLimitRotations Reverse soft limit position in rotations from zero
     * @param enable True to enable soft limits, false to disable
     */
    void configureSoftLimits(double forwardLimitRotations, double reverseLimitRotations, boolean enable);

    /**
     * Enables or disables software position limits.
     *
     * <p>This method allows you to toggle soft limits on/off without reconfiguring
     * the limit values. Useful for setup/testing modes where you need to move beyond
     * normal limits temporarily.
     *
     * @param enable True to enable soft limits, false to disable
     * @see #configureSoftLimits(double, double, boolean)
     */
    void enableSoftLimits(boolean enable);

    /**
     * Sets the motor output direction inversion.
     *
     * <p>When inverted, positive commands result in reverse motor rotation and vice versa.
     * Use this to correct for mechanical mounting orientation without changing code logic.
     *
     * <p><strong>Example:</strong> If your left and right drive motors face opposite directions,
     * invert one side so both spin "forward" with positive commands.
     *
     * @param inverted True to invert motor direction, false for normal direction
     */
    void setInverted(boolean inverted);

    /**
     * Configures the motor's neutral mode behavior.
     *
     * <p>This determines how the motor behaves when commanded to zero output:
     * <ul>
     *   <li><strong>Brake mode (true):</strong> Motor actively resists motion, providing
     *       immediate stopping and position holding. Use for mechanisms that need to stay
     *       in place (arms, elevators, intakes).</li>
     *   <li><strong>Coast mode (false):</strong> Motor freewheels when stopped, allowing
     *       easy manual backdriving. Use for drive trains or mechanisms that should coast
     *       to a stop.</li>
     * </ul>
     *
     * <p><strong>Example:</strong>
     * <pre>{@code
     * driveMotor.setBrakeMode(false);    // Coast mode for drivetrain
     * elevatorMotor.setBrakeMode(true);  // Brake mode to hold position
     * }</pre>
     *
     * @param brake True for brake mode (resist motion), false for coast mode (freewheel)
     */
    void setBrakeMode(boolean brake);

    /**
     * Resets the motor's position sensor to a specified value.
     *
     * <p>This sets the current position reading without physically moving the motor.
     * Commonly used during robot initialization or when homing to a known position.
     *
     * <p><strong>Example:</strong>
     * <pre>{@code
     * // Reset elevator to bottom position
     * motor.setPosition(0.0);
     *
     * // Set arm to known calibration angle
     * motor.setPosition(15.5);  // 15.5 rotations = calibrated starting position
     * }</pre>
     *
     * @param rotations Position value to set in rotations
     */
    void setPosition(double rotations);

    /**
     * Enables voltage compensation for consistent motor performance.
     *
     * <p>Voltage compensation adjusts motor output to maintain consistent behavior regardless
     * of battery voltage. As battery depletes from 12.6V to 10V during a match, this feature
     * keeps motor output proportional to commanded values.
     *
     * <p><strong>Benefits:</strong>
     * <ul>
     *   <li>Consistent autonomous routines throughout the match</li>
     *   <li>More predictable PID tuning</li>
     *   <li>Repeatable robot behavior from match to match</li>
     * </ul>
     *
     * <p><strong>Typical value:</strong> 12.0 volts (compensate to full battery voltage)
     *
     * <p><strong>Example:</strong>
     * <pre>{@code
     * motor.enableVoltageCompensation(12.0);  // Compensate to 12V nominal
     * }</pre>
     *
     * @param voltageNominal Nominal voltage to compensate to, typically 12.0 volts
     */
    void enableVoltageCompensation(double voltageNominal);

    /**
     * Gets the current motor position from the encoder.
     *
     * <p>Position is measured in rotations from the last reset point.
     *
     * @return Current position in rotations
     */
    double getPosition();

    /**
     * Gets the current motor velocity.
     *
     * <p><strong>Returns typed AngularVelocity</strong> - use {@code .in(RotationsPerSecond)}
     * or {@code .in(RPM)} to convert to raw values.
     *
     * <p><strong>Example:</strong>
     * <pre>{@code
     * import static edu.wpi.first.units.Units.*;
     *
     * AngularVelocity velocity = motor.getVelocity();
     * double rps = velocity.in(RotationsPerSecond);  // rotations per second
     * double rpm = velocity.in(RPM);                  // revolutions per minute
     * }</pre>
     *
     * @return Current velocity as AngularVelocity
     */
    AngularVelocity getVelocity();

    /**
     * Gets the current motor acceleration.
     *
     * <p><strong>Note:</strong> Acceleration is calculated by the motor controller
     * and may not be available on all hardware.
     *
     * @return Current acceleration (returns zero if not supported)
     */
    AngularAcceleration getAcceleration();

    /**
     * Gets the current draw of the motor.
     *
     * <p>Monitor this value to detect stalls, overloads, or current limit violations.
     *
     * <p><strong>Example:</strong>
     * <pre>{@code
     * import static edu.wpi.first.units.Units.*;
     *
     * Current current = motor.getCurrentDraw();
     * double amps = current.in(Amps);
     * }</pre>
     *
     * @return Current draw
     */
    Current getCurrentDraw();

    /**
     * Gets the current motor output as a percentage.
     *
     * <p>This represents the actual duty cycle being applied to the motor,
     * regardless of control mode.
     *
     * @return Output percentage (-1.0 to 1.0, where 1.0 = full forward)
     */
    double getOutputPercent();

    /**
     * Gets the internal temperature of the motor controller.
     *
     * <p>Monitor this value to prevent thermal shutdown. Most motor controllers
     * will automatically reduce output or shut down if temperature exceeds safe limits
     * (typically 80-100°C depending on hardware).
     *
     * @return Temperature in degrees Celsius
     */
    double getTemperature();

    /**
     * Gets the state of the forward hardware limit switch.
     *
     * <p>Hardware limit switches are physical switches that trigger when a mechanism
     * reaches its travel limits. They provide a fail-safe backup to software limits.
     *
     * @return True if forward limit switch is currently pressed/triggered, false otherwise
     */
    boolean getForwardLimitSwitch();

    /**
     * Gets the state of the reverse hardware limit switch.
     *
     * <p>Hardware limit switches are physical switches that trigger when a mechanism
     * reaches its travel limits. They provide a fail-safe backup to software limits.
     *
     * @return True if reverse limit switch is currently pressed/triggered, false otherwise
     */
    boolean getReverseLimitSwitch();

    /**
     * Configures this motor to follow another motor's output.
     *
     * <p>In follower mode, this motor will exactly mirror the output of the leader motor.
     * This is commonly used for multi-motor drivetrains or mechanisms where multiple
     * motors need to move in perfect synchronization.
     *
     * <p><strong>Important:</strong> The follower motor must be the same type and on the
     * same CAN bus as the leader motor.
     *
     * <p><strong>Example:</strong>
     * <pre>{@code
     * BaseMotor leftLeader = new NEOMotor(1, false, 40, false);
     * BaseMotor leftFollower = new NEOMotor(2, false, 40, false);
     * leftFollower.setStrictFollower(1);  // Follow motor with device ID 1
     * }</pre>
     *
     * @param leaderDeviceID CAN device ID of the motor to follow
     */
    void setStrictFollower(int leaderDeviceID);

    /**
     * Configures hardware limit switches for automatic position zeroing.
     *
     * <p>Hardware limit switches can automatically reset the encoder position when triggered,
     * providing repeatable homing for mechanisms like elevators or arms.
     *
     * <p><strong>Example:</strong>
     * <pre>{@code
     * // Elevator that zeros when hitting bottom limit switch
     * motor.configureHardLimits(
     *     false,  // Don't enable forward limit
     *     true,   // Enable reverse (bottom) limit
     *     0.0,    // Not used (forward disabled)
     *     0.0     // Reset position to 0 when hitting bottom
     * );
     * }</pre>
     *
     * @param enableForward True to enable forward limit switch
     * @param enableReverse True to enable reverse limit switch
     * @param forwardResetValueRotations Position to set when forward limit is hit (rotations)
     * @param reverseResetValueRotations Position to set when reverse limit is hit (rotations)
     */
    void configureHardLimits(boolean enableForward, boolean enableReverse,
                            double forwardResetValueRotations, double reverseResetValueRotations);

    /**
     * Creates a configuration builder for this motor.
     *
     * <p>The builder pattern provides a fluent API for configuring motors with multiple
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
     *     .voltageCompensation(12.0)
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
     * @return MotorConfigBuilder for fluent configuration
     * @see com.adambots.lib.actuators.config.MotorConfigBuilder
     */
    default com.adambots.lib.actuators.config.MotorConfigBuilder configure() {
        return new com.adambots.lib.actuators.config.MotorConfigBuilder(this);
    }

    /**
     * Checks if this motor supports a specific control mode.
     *
     * <p>This method allows runtime checking of control mode capabilities before using them.
     * Useful for writing motor-agnostic code that can work with different motor types.
     *
     * <p><strong>Default implementation:</strong> Supports all modes except MOTION_MAGIC_FOC_TORQUE and CURRENT.
     * Motor implementations override this to specify their actual capabilities.
     *
     * <p><strong>Example:</strong>
     * <pre>{@code
     * if (motor.supportsControlMode(ControlMode.CURRENT)) {
     *     motor.set(ControlMode.CURRENT, 10.0);
     * } else {
     *     motor.set(ControlMode.PERCENT_OUTPUT, 0.5);
     * }
     * }</pre>
     *
     * @param mode Control mode to check
     * @return True if this motor fully supports the control mode, false otherwise
     */
    default boolean supportsControlMode(ControlMode mode) {
        // Default: support all modes except FOC torque and current
        return mode != ControlMode.MOTION_MAGIC_FOC_TORQUE && mode != ControlMode.CURRENT;
    }

    /**
     * Gets the motor type name for logging and debugging.
     *
     * <p>This method helps identify which motor implementation is being used,
     * useful for diagnostics and error messages.
     *
     * <p><strong>Example:</strong>
     * <pre>{@code
     * System.out.println("Motor type: " + motor.getMotorType());
     * // Output: "Motor type: TalonFXMotor (Kraken X60)"
     * }</pre>
     *
     * @return Motor type name (e.g., "NEOMotor", "TalonFXMotor", "MinionMotor")
     */
    default String getMotorType() {
        return this.getClass().getSimpleName();
    }

}