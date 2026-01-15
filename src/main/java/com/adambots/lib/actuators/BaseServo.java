package com.adambots.lib.actuators;

/**
 * Base interface for servo controllers in FRC robotics.
 *
 * <p>This interface provides a unified API for controlling different servo types
 * with support for both Angular (position control) and Continuous Rotation (speed control) modes.
 *
 * <p><strong>Supported Servo Types:</strong>
 * <ul>
 *   <li>REV ServoHub-connected servos (AngularHubServo, CRHubServo)</li>
 *   <li>Direct PWM-connected servos (DirectServo)</li>
 *   <li>Angular servos (0-355° typical for Axon Max+)</li>
 *   <li>Continuous rotation servos (variable speed)</li>
 * </ul>
 *
 * <p><strong>Usage Example:</strong>
 * <pre>{@code
 * // Angular servo on ServoHub
 * ServoHub hub = new ServoHub(10);
 * BaseServo angularServo = new AngularHubServo(hub, 0, 355.0);
 * angularServo.setAngle(90.0);  // Move to 90 degrees
 *
 * // Continuous rotation servo
 * BaseServo crServo = new CRHubServo(hub, 1);
 * crServo.set(0.5);  // Half speed clockwise
 * }</pre>
 *
 * <p><strong>Capability Detection:</strong>
 * <p>Use capability methods to check what a servo supports before calling mode-specific methods:
 * <pre>{@code
 * if (servo.supportsAngleControl()) {
 *     servo.setAngle(180.0);
 * } else if (servo.supportsSpeedControl()) {
 *     servo.set(1.0);
 * }
 * }</pre>
 *
 * @see AngularHubServo
 * @see CRHubServo
 * @see DirectServo
 */
public interface BaseServo extends BaseActuator{
    /**
     * Operating mode of the servo.
     *
     * <p><strong>Mode Details:</strong>
     * <ul>
     *   <li><strong>ANGULAR:</strong> Position control mode (0-355° typical)</li>
     *   <li><strong>CONTINUOUS_ROTATION:</strong> Speed control mode (-1.0 to 1.0)</li>
     * </ul>
     */
    enum ServoMode {
        CONTINUOUS_ROTATION,
        ANGULAR
    }

    /**
     * Gets the current operating mode of the servo.
     * @return The current ServoMode
     */
    ServoMode getMode();

    /**
     * For CR mode: Turns the servo counterclockwise
     * For Angular mode: Sets position to maximum angle
     */
    void turnCounterclockwise();

    /**
     * For CR mode: Turns the servo clockwise
     * For Angular mode: Sets position to minimum angle
     */
    void turnClockwise();

    /**
     * For CR mode: Stops rotation
     * For Angular mode: Sets to center position
     */
    void stop();

    /**
     * Sets the raw pulse width of the servo.
     * @param pulseWidth The pulse width in microseconds
     */
    void setPulseWidth(int pulseWidth);

    /**
     * Gets the current being drawn by the servo.
     * @return The current in amps
     */
    double getCurrent();

    /**
     * For Angular mode: Sets the angle of the servo.
     * For CR mode: Logs a warning and does nothing (graceful fallback).
     *
     * <p><strong>Best Practice:</strong> Use {@link #supportsAngleControl()} to check
     * if this servo supports angle control before calling this method.
     *
     * <p><strong>Example:</strong>
     * <pre>{@code
     * if (servo.supportsAngleControl()) {
     *     servo.setAngle(90.0);
     * }
     * }</pre>
     *
     * @param degrees The angle in degrees (0-355 for Axon Max+, 0-180 for standard servos)
     */
    default void setAngle(double degrees) {
        if (!supportsAngleControl()) {
            edu.wpi.first.wpilibj.DriverStation.reportWarning(
                getServoType() + ": Angle control not supported in " + getMode() + " mode. Call ignored.", false);
        }
    }

    /**
     * For CR mode: Sets the speed of rotation.
     * For Angular mode: Logs a warning and does nothing (graceful fallback).
     *
     * <p><strong>Best Practice:</strong> Use {@link #supportsSpeedControl()} to check
     * if this servo supports speed control before calling this method.
     *
     * <p><strong>Example:</strong>
     * <pre>{@code
     * if (servo.supportsSpeedControl()) {
     *     servo.set(0.5);  // Half speed
     * }
     * }</pre>
     *
     * @param speed Speed from -1.0 (full CCW) to 1.0 (full CW)
     */
    @Override
    default void set(double speed) {
        if (!supportsSpeedControl()) {
            edu.wpi.first.wpilibj.DriverStation.reportWarning(
                getServoType() + ": Speed control not supported in " + getMode() + " mode. Call ignored.", false);
        }
    }

    /**
     * Checks if this servo supports angle control.
     *
     * <p>Use this method to determine if {@link #setAngle(double)} is supported
     * before calling it to avoid unnecessary warnings.
     *
     * @return True if this servo supports angle control (Angular mode), false otherwise
     */
    default boolean supportsAngleControl() {
        return getMode() == ServoMode.ANGULAR;
    }

    /**
     * Checks if this servo supports speed control.
     *
     * <p>Use this method to determine if {@link #set(double)} is supported
     * before calling it to avoid unnecessary warnings.
     *
     * @return True if this servo supports speed control (CR mode), false otherwise
     */
    default boolean supportsSpeedControl() {
        return getMode() == ServoMode.CONTINUOUS_ROTATION;
    }

    /**
     * Gets the servo type name for logging and debugging.
     *
     * <p>This helps identify which servo implementation is being used,
     * useful for diagnostics and error messages.
     *
     * <p><strong>Example:</strong>
     * <pre>{@code
     * System.out.println("Servo type: " + servo.getServoType());
     * // Output: "Servo type: AngularHubServo"
     * }</pre>
     *
     * @return Servo type name (e.g., "AngularHubServo", "CRHubServo", "DirectServo")
     */
    default String getServoType() {
        return this.getClass().getSimpleName();
    }
}
