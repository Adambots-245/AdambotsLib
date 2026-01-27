package com.adambots.lib.actuators;

import edu.wpi.first.wpilibj.Servo;

import static edu.wpi.first.units.Units.*;
import edu.wpi.first.units.measure.*;

public class DirectServo implements BaseServo {
    private final Servo servo;
    private final ServoMode mode;

    // Pulse width constants (microseconds)
    private static final double SERVO_MIN_PULSE_WIDTH_US = 600.0;
    private static final double SERVO_MAX_PULSE_WIDTH_US = 2400.0;

    // Configurable angle range
    private double minAngle = 0.0;
    private double maxAngle = 180.0;  // Default for standard servos
    private double centerAngle = 90.0;

    /**
     * Creates a new DirectServo connected to a PWM port
     * @param channel The PWM channel this servo is connected to (0-19 on RoboRIO 2)
     * @param mode The servo mode (CONTINUOUS_ROTATION or ANGULAR)
     */
    public DirectServo(int channel, ServoMode mode) {
        // Validate PWM channel range (0-19 for RoboRIO 2)
        if (channel < 0 || channel > 19) {
            edu.wpi.first.wpilibj.DriverStation.reportWarning(
                "DirectServo: PWM channel " + channel + " may be out of range [0-19]. Verify hardware configuration.", false);
        }
        this.servo = new Servo(channel);
        this.mode = mode;
        updateCenterAngle();
    }

    /**
     * Creates a new DirectServo with custom angle range
     * @param channel The PWM channel this servo is connected to (0-19 on RoboRIO 2)
     * @param mode The servo mode (CONTINUOUS_ROTATION or ANGULAR)
     * @param minAngle Minimum angle
     * @param maxAngle Maximum angle
     */
    public DirectServo(int channel, ServoMode mode, Angle minAngle, Angle maxAngle) {
        // Validate PWM channel range (0-19 for RoboRIO 2)
        if (channel < 0 || channel > 19) {
            edu.wpi.first.wpilibj.DriverStation.reportWarning(
                "DirectServo: PWM channel " + channel + " may be out of range [0-19]. Verify hardware configuration.", false);
        }
        this.servo = new Servo(channel);
        this.mode = mode;
        setAngleRange(minAngle, maxAngle);
    }

    /**
     * Sets the angle range for the servo
     * @param minAngle Minimum angle
     * @param maxAngle Maximum angle
     */
    public void setAngleRange(Angle minAngle, Angle maxAngle) {
        double minDegrees = minAngle.in(Degrees);
        double maxDegrees = maxAngle.in(Degrees);
        if (maxDegrees <= minDegrees) {
            edu.wpi.first.wpilibj.DriverStation.reportWarning(
                "DirectServo: maxAngle (" + maxDegrees + "°) must be greater than minAngle (" +
                minDegrees + "°). Ignoring setAngleRange() call.", false);
            return;
        }
        this.minAngle = minDegrees;
        this.maxAngle = maxDegrees;
        updateCenterAngle();
    }

    private void updateCenterAngle() {
        this.centerAngle = (maxAngle + minAngle) / 2.0;
    }

    @Override
    public ServoMode getMode() {
        return mode;
    }

    @Override
    public void turnCounterclockwise() {
        if (mode == ServoMode.CONTINUOUS_ROTATION) {
            servo.set(1.0);  // Full speed CCW
        } else {
            servo.setAngle(maxAngle);  // Max angle
        }
    }

    @Override
    public void turnClockwise() {
        if (mode == ServoMode.CONTINUOUS_ROTATION) {
            servo.set(0.0);  // Full speed CW
        } else {
            servo.setAngle(minAngle);  // Min angle
        }
    }

    @Override
    public void stop() {
        if (mode == ServoMode.CONTINUOUS_ROTATION) {
            servo.set(0.5);  // Center position stops rotation
        } else {
            servo.setAngle(centerAngle);  // Center position
        }
    }

    @Override
    public void set(double speed) {
        if (mode == ServoMode.CONTINUOUS_ROTATION) {
            // Validate speed bounds
            double originalSpeed = speed;
            speed = Math.min(1.0, Math.max(-1.0, speed));

            if (speed != originalSpeed) {
                edu.wpi.first.wpilibj.DriverStation.reportWarning(
                    "DirectServo: Speed " + originalSpeed + " out of range [-1.0, 1.0]. Clamping to " + speed + ".", false);
            }

            // Map -1.0 to 1.0 to 0.0 to 1.0 range for WPILib Servo
            double mappedSpeed = (speed + 1.0) / 2.0;
            servo.set(mappedSpeed);
        } else {
            // Use BaseServo's graceful fallback
            BaseServo.super.set(speed);
        }
    }

    @Override
    public void setPulseWidth(int pulseWidth) {
        // Validate pulse width bounds
        if (pulseWidth < SERVO_MIN_PULSE_WIDTH_US || pulseWidth > SERVO_MAX_PULSE_WIDTH_US) {
            edu.wpi.first.wpilibj.DriverStation.reportWarning(
                "DirectServo: Pulse width " + pulseWidth + " µs out of range [" +
                (int)SERVO_MIN_PULSE_WIDTH_US + ", " + (int)SERVO_MAX_PULSE_WIDTH_US + "]. Clamping.", false);
        }

        // Convert microseconds to position (0-1 range)
        // WPILib expects 0.6ms-2.4ms range mapped to 0-1
        double position = (pulseWidth - SERVO_MIN_PULSE_WIDTH_US) / (SERVO_MAX_PULSE_WIDTH_US - SERVO_MIN_PULSE_WIDTH_US);
        position = Math.min(1.0, Math.max(0.0, position));
        servo.set(position);
    }

    @Override
    public Current getCurrent() {
        // WPILib Servo class doesn't provide current monitoring
        return Amps.of(0.0);
    }

    @Override
    public void setAngle(Angle angle) {
        if (mode == ServoMode.ANGULAR) {
            double degrees = angle.in(Degrees);
            // Validate and clamp to valid range
            double originalDegrees = degrees;
            degrees = Math.min(maxAngle, Math.max(minAngle, degrees));

            if (degrees != originalDegrees) {
                edu.wpi.first.wpilibj.DriverStation.reportWarning(
                    "DirectServo: Angle " + originalDegrees + "° out of range [" +
                    minAngle + "°, " + maxAngle + "°]. Clamping to " + degrees + "°.", false);
            }

            // Map the custom angle range to WPILib's 0-180 range
            double normalizedAngle = (degrees - minAngle) / (maxAngle - minAngle) * 180.0;
            servo.setAngle(normalizedAngle);
        } else {
            // Use BaseServo's graceful fallback
            BaseServo.super.setAngle(angle);
        }
    }

    /**
     * Gets the raw position (0 to 1 range)
     * @return The current position
     */
    public double getPosition() {
        return servo.get();
    }

    /**
     * Gets the current angle (only valid in ANGULAR mode)
     * @return The current angle in the configured range, or 0° if in CR mode
     */
    public Angle getAngle() {
        if (mode == ServoMode.ANGULAR) {
            // Convert from WPILib's 0-180 range back to our custom range
            double wpiLibAngle = servo.getAngle();
            return Degrees.of(minAngle + (wpiLibAngle / 180.0) * (maxAngle - minAngle));
        }
        edu.wpi.first.wpilibj.DriverStation.reportWarning(
            "DirectServo: Angle reading not supported in CR mode. Returning 0°.", false);
        return Degrees.of(0.0);
    }

    /**
     * Gets the minimum configured angle
     * @return The minimum angle
     */
    public Angle getMinAngle() {
        return Degrees.of(minAngle);
    }

    /**
     * Gets the maximum configured angle
     * @return The maximum angle
     */
    public Angle getMaxAngle() {
        return Degrees.of(maxAngle);
    }

    /**
     * Gets the center angle
     * @return The center angle
     */
    public Angle getCenterAngle() {
        return Degrees.of(centerAngle);
    }
}