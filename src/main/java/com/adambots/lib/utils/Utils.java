// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.lib.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.util.Color;

/**
 * General utility functions for FRC robot code.
 *
 * <p>This class provides commonly-needed utility functions that complement WPILib's
 * built-in utilities. For standard math operations like clamp and deadband, use
 * {@link edu.wpi.first.math.MathUtil} instead.
 *
 * <p><strong>Categories:</strong>
 * <ul>
 *   <li>Alliance Utilities - Check alliance color, mirror poses</li>
 *   <li>Math Utilities - Linear interpolation, range mapping</li>
 *   <li>Angle Utilities - Normalize angles, compute shortest difference</li>
 *   <li>Boolean Utilities - Debouncing, edge detection</li>
 *   <li>Array Utilities - Average, min, max for arrays</li>
 *   <li>Time Utilities - Time conversion and elapsed checks</li>
 *   <li>Error Reporting - Standardized error/warning messages</li>
 * </ul>
 *
 * <p><strong>Note:</strong> For standard WPILib utilities, use:
 * <ul>
 *   <li>{@link edu.wpi.first.math.MathUtil#clamp} - Clamp value to range</li>
 *   <li>{@link edu.wpi.first.math.MathUtil#applyDeadband} - Apply deadband</li>
 *   <li>{@link edu.wpi.first.math.MathUtil#interpolate} - Linear interpolation</li>
 *   <li>{@link edu.wpi.first.wpilibj.util.Color} - Color operations</li>
 * </ul>
 *
 * @see edu.wpi.first.math.MathUtil
 * @see edu.wpi.first.wpilibj.DriverStation
 */
public class Utils {

    /**
     * Private constructor to prevent instantiation.
     *
     * @throws UnsupportedOperationException always, as this is a utility class
     */
    private Utils() {
        throw new UnsupportedOperationException("Not meant to be instantiated. Utility class");
    }

    // ======================== ALLIANCE UTILITIES ========================

    /**
     * Returns {@code true} if the robot is on the red alliance.
     *
     * <p><strong>Example:</strong>
     * <pre>{@code
     * if (Utils.isOnRedAlliance()) {
     *     targetPose = FieldPositions.RED_SPEAKER;
     * } else {
     *     targetPose = FieldPositions.BLUE_SPEAKER;
     * }
     * }</pre>
     *
     * @return {@code true} if on red alliance, {@code false} otherwise (including if alliance unknown)
     */
    public static boolean isOnRedAlliance() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
        }
        return false;
    }

    /**
     * Returns {@code true} if the robot is on the blue alliance.
     *
     * <p><strong>Example:</strong>
     * <pre>{@code
     * if (Utils.isOnBlueAlliance()) {
     *     targetPose = FieldPositions.BLUE_SPEAKER;
     * }
     * }</pre>
     *
     * @return {@code true} if on blue alliance, {@code false} otherwise (including if alliance unknown)
     */
    public static boolean isOnBlueAlliance() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Blue;
        }
        return false;
    }

    /**
     * Returns the alliance color as a WPILib Color object.
     *
     * <p>Returns {@link Color#kRed} for red alliance, {@link Color#kBlue} for blue alliance,
     * or {@link Color#kBlack} if alliance is unknown (e.g., before connected to FMS).
     *
     * <p><strong>Example:</strong>
     * <pre>{@code
     * // Set LED color to alliance color
     * leds.setColor(Utils.getAllianceColor());
     * }</pre>
     *
     * @return Alliance color (red, blue, or black if unknown)
     */
    public static Color getAllianceColor() {
        if (isOnRedAlliance()) {
            return Color.kRed;
        } else if (isOnBlueAlliance()) {
            return Color.kBlue;
        }
        return Color.kBlack;
    }

    /**
     * Mirrors an X coordinate for the current alliance.
     *
     * <p>FRC fields are symmetric, but paths and positions are typically defined for
     * one alliance (usually blue). This method mirrors an X coordinate so it works
     * for both alliances.
     *
     * <p><strong>Example:</strong>
     * <pre>{@code
     * // Blue alliance: x = 2.0, Red alliance: x = 14.54
     * double targetX = Utils.mirrorXForAlliance(2.0, 16.54);
     * Pose2d target = new Pose2d(targetX, 5.5, Rotation2d.fromDegrees(0));
     * }</pre>
     *
     * @param x X coordinate in blue alliance frame
     * @param fieldLength Total field length in meters (typically 16.54m)
     * @return Mirrored X coordinate if on red alliance, original X if on blue alliance
     */
    public static double mirrorXForAlliance(double x, double fieldLength) {
        if (isOnRedAlliance()) {
            return fieldLength - x;
        }
        return x;
    }

    /**
     * Mirrors a Translation2d for the current alliance.
     *
     * <p>Mirrors the X coordinate and keeps Y the same.
     *
     * <p><strong>Example:</strong>
     * <pre>{@code
     * Translation2d blueSpeaker = new Translation2d(0.5, 5.5);
     * Translation2d speaker = Utils.mirrorTranslationForAlliance(blueSpeaker, 16.54);
     * }</pre>
     *
     * @param translation Translation in blue alliance frame
     * @param fieldLength Total field length in meters (typically 16.54m)
     * @return Mirrored translation if on red alliance, original if on blue alliance
     */
    public static Translation2d mirrorTranslationForAlliance(Translation2d translation, double fieldLength) {
        return new Translation2d(
            mirrorXForAlliance(translation.getX(), fieldLength),
            translation.getY()
        );
    }

    /**
     * Mirrors a Pose2d for the current alliance.
     *
     * <p>Mirrors the X coordinate and rotates the heading 180 degrees.
     *
     * <p><strong>Example:</strong>
     * <pre>{@code
     * Pose2d bluePose = new Pose2d(2.0, 5.5, Rotation2d.fromDegrees(0));
     * Pose2d alliancePose = Utils.mirrorPoseForAlliance(bluePose, 16.54);
     * }</pre>
     *
     * @param pose Pose in blue alliance frame
     * @param fieldLength Total field length in meters (typically 16.54m)
     * @return Mirrored pose if on red alliance, original pose if on blue alliance
     */
    public static Pose2d mirrorPoseForAlliance(Pose2d pose, double fieldLength) {
        if (isOnRedAlliance()) {
            return new Pose2d(
                mirrorXForAlliance(pose.getX(), fieldLength),
                pose.getY(),
                pose.getRotation().rotateBy(Rotation2d.fromDegrees(180))
            );
        }
        return pose;
    }

    // ======================== MATH UTILITIES ========================

    /**
     * Linearly interpolates between two values.
     *
     * <p><strong>Formula:</strong> {@code start + (end - start) * t}
     *
     * <p><strong>Note:</strong> For standard interpolation, consider using
     * {@link edu.wpi.first.math.MathUtil#interpolate} instead.
     *
     * <p><strong>Example:</strong>
     * <pre>{@code
     * // Interpolate from 0 to 100
     * double value = Utils.lerp(0, 100, 0.5);  // Returns 50
     * double value2 = Utils.lerp(0, 100, 0.25); // Returns 25
     * }</pre>
     *
     * @param start Start value (at t=0)
     * @param end End value (at t=1)
     * @param t Interpolation factor (typically 0.0 to 1.0, but can be outside for extrapolation)
     * @return Interpolated value
     */
    public static double lerp(double start, double end, double t) {
        return start + (end - start) * t;
    }

    /**
     * Maps a value from one range to another.
     *
     * <p>This is useful for converting sensor values, scaling inputs, or normalizing data.
     *
     * <p><strong>Example:</strong>
     * <pre>{@code
     * // Map joystick input (-1 to 1) to motor speed (0 to 100)
     * double motorSpeed = Utils.mapRange(joystick.getY(), -1, 1, 0, 100);
     *
     * // Map encoder (0 to 4096) to angle (0 to 360)
     * double angleDeg = Utils.mapRange(encoder.getPosition(), 0, 4096, 0, 360);
     * }</pre>
     *
     * @param value Input value
     * @param inMin Minimum of input range
     * @param inMax Maximum of input range
     * @param outMin Minimum of output range
     * @param outMax Maximum of output range
     * @return Value mapped to output range
     */
    public static double mapRange(double value, double inMin, double inMax, double outMin, double outMax) {
        return (value - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
    }

    /**
     * Checks if a value is within a range (inclusive).
     *
     * <p><strong>Example:</strong>
     * <pre>{@code
     * if (Utils.isInRange(encoder.getPosition(), 100, 200)) {
     *     // Encoder is in safe zone
     * }
     * }</pre>
     *
     * @param value Value to check
     * @param min Minimum of range (inclusive)
     * @param max Maximum of range (inclusive)
     * @return {@code true} if value is in range [min, max]
     */
    public static boolean isInRange(double value, double min, double max) {
        return value >= min && value <= max;
    }

    /**
     * Calculates the percentage of progress from start to end.
     *
     * <p><strong>Example:</strong>
     * <pre>{@code
     * // Calculate arm progress from stowed (0) to deployed (90)
     * double progress = Utils.percentProgress(arm.getAngle(), 0, 90);
     * // If angle = 45, progress = 0.5 (50%)
     * }</pre>
     *
     * @param current Current value
     * @param start Start value
     * @param end End value
     * @return Progress as a value from 0.0 to 1.0
     */
    public static double percentProgress(double current, double start, double end) {
        if (Math.abs(end - start) < 1e-6) {
            return 1.0;  // Already at target
        }
        return (current - start) / (end - start);
    }

    // ======================== ANGLE UTILITIES ========================

    /**
     * Normalizes an angle to the range [-180, 180] degrees.
     *
     * <p><strong>Example:</strong>
     * <pre>{@code
     * double angle = Utils.wrapAngleDeg(270);   // Returns -90
     * double angle2 = Utils.wrapAngleDeg(-190); // Returns 170
     * }</pre>
     *
     * @param angleDeg Angle in degrees
     * @return Normalized angle in range [-180, 180]
     */
    public static double wrapAngleDeg(double angleDeg) {
        double angle = angleDeg % 360;
        if (angle > 180) {
            angle -= 360;
        } else if (angle < -180) {
            angle += 360;
        }
        return angle;
    }

    /**
     * Normalizes an angle to the range [-π, π] radians.
     *
     * <p><strong>Example:</strong>
     * <pre>{@code
     * double angle = Utils.wrapAngleRad(Math.PI * 1.5);  // Returns -π/2
     * }</pre>
     *
     * @param angleRad Angle in radians
     * @return Normalized angle in range [-π, π]
     */
    public static double wrapAngleRad(double angleRad) {
        double angle = angleRad % (2 * Math.PI);
        if (angle > Math.PI) {
            angle -= 2 * Math.PI;
        } else if (angle < -Math.PI) {
            angle += 2 * Math.PI;
        }
        return angle;
    }

    /**
     * Calculates the shortest angular difference from current to target angle in degrees.
     *
     * <p>This is critical for angle control - always rotate the shortest direction.
     *
     * <p><strong>Example:</strong>
     * <pre>{@code
     * // Current = 10°, Target = 350°
     * // Normal subtraction: 340° (wrong!)
     * // Shortest difference: -20° (correct - rotate left 20°)
     * double error = Utils.shortestAngleDifferenceDeg(10, 350);  // Returns -20
     * }</pre>
     *
     * @param currentDeg Current angle in degrees
     * @param targetDeg Target angle in degrees
     * @return Shortest angular difference in range [-180, 180]
     */
    public static double shortestAngleDifferenceDeg(double currentDeg, double targetDeg) {
        return wrapAngleDeg(targetDeg - currentDeg);
    }

    /**
     * Calculates the shortest angular difference from current to target angle in radians.
     *
     * <p><strong>Example:</strong>
     * <pre>{@code
     * double error = Utils.shortestAngleDifferenceRad(current, target);
     * motorOutput = pidController.calculate(0, error);
     * }</pre>
     *
     * @param currentRad Current angle in radians
     * @param targetRad Target angle in radians
     * @return Shortest angular difference in range [-π, π]
     */
    public static double shortestAngleDifferenceRad(double currentRad, double targetRad) {
        return wrapAngleRad(targetRad - currentRad);
    }

    // ======================== BOOLEAN UTILITIES ========================

    /**
     * Debounces a boolean value to prevent rapid toggling.
     *
     * <p>The value must be stable for the specified time before changing state.
     * This prevents false triggers from noisy sensors or buttons.
     *
     * <p><strong>Example:</strong>
     * <pre>{@code
     * private boolean lastSensorValue = false;
     * private long lastChangeTime = 0;
     *
     * public boolean hasGamePiece() {
     *     boolean currentValue = sensor.get();
     *     boolean debounced = Utils.debounce(
     *         currentValue, lastSensorValue, lastChangeTime, 100
     *     );
     *
     *     if (currentValue != lastSensorValue) {
     *         lastChangeTime = System.currentTimeMillis();
     *         lastSensorValue = currentValue;
     *     }
     *
     *     return debounced;
     * }
     * }</pre>
     *
     * @param currentValue Current sensor/button state
     * @param lastValue Previous sensor/button state
     * @param lastChangeTimeMs Time of last state change (from System.currentTimeMillis())
     * @param debounceTimeMs Minimum time value must be stable (milliseconds)
     * @return Debounced value
     */
    public static boolean debounce(boolean currentValue, boolean lastValue,
                                   long lastChangeTimeMs, long debounceTimeMs) {
        if (currentValue == lastValue) {
            // Value is stable
            return currentValue;
        } else {
            // Value changed - check if enough time has passed
            long elapsed = System.currentTimeMillis() - lastChangeTimeMs;
            return elapsed >= debounceTimeMs ? currentValue : lastValue;
        }
    }

    /**
     * Detects a rising edge (false to true transition).
     *
     * <p><strong>Example:</strong>
     * <pre>{@code
     * private boolean lastButtonState = false;
     *
     * public void periodic() {
     *     boolean buttonPressed = button.get();
     *     if (Utils.risingEdge(buttonPressed, lastButtonState)) {
     *         // Button was just pressed
     *         doAction();
     *     }
     *     lastButtonState = buttonPressed;
     * }
     * }</pre>
     *
     * @param currentValue Current value
     * @param lastValue Previous value
     * @return {@code true} if transitioned from false to true
     */
    public static boolean risingEdge(boolean currentValue, boolean lastValue) {
        return currentValue && !lastValue;
    }

    /**
     * Detects a falling edge (true to false transition).
     *
     * <p><strong>Example:</strong>
     * <pre>{@code
     * if (Utils.fallingEdge(limitSwitch.get(), lastLimitState)) {
     *     // Limit switch was just released
     * }
     * }</pre>
     *
     * @param currentValue Current value
     * @param lastValue Previous value
     * @return {@code true} if transitioned from true to false
     */
    public static boolean fallingEdge(boolean currentValue, boolean lastValue) {
        return !currentValue && lastValue;
    }

    // ======================== ARRAY UTILITIES ========================

    /**
     * Calculates the average of an array of doubles.
     *
     * <p><strong>Example:</strong>
     * <pre>{@code
     * double[] motorCurrents = {12.5, 13.2, 11.8, 12.9};
     * double avgCurrent = Utils.average(motorCurrents);  // Returns 12.6
     * }</pre>
     *
     * @param values Array of values
     * @return Average value, or 0.0 if array is empty
     */
    public static double average(double[] values) {
        if (values.length == 0) {
            return 0.0;
        }
        double sum = 0;
        for (double value : values) {
            sum += value;
        }
        return sum / values.length;
    }

    /**
     * Finds the maximum value in an array of doubles.
     *
     * <p><strong>Example:</strong>
     * <pre>{@code
     * double[] temps = {45.2, 52.1, 48.7, 51.3};
     * double maxTemp = Utils.max(temps);  // Returns 52.1
     * }</pre>
     *
     * @param values Array of values
     * @return Maximum value, or Double.NEGATIVE_INFINITY if array is empty
     */
    public static double max(double[] values) {
        if (values.length == 0) {
            return Double.NEGATIVE_INFINITY;
        }
        double max = values[0];
        for (int i = 1; i < values.length; i++) {
            if (values[i] > max) {
                max = values[i];
            }
        }
        return max;
    }

    /**
     * Finds the minimum value in an array of doubles.
     *
     * <p><strong>Example:</strong>
     * <pre>{@code
     * double[] distances = {2.5, 1.2, 3.1, 1.8};
     * double minDist = Utils.min(distances);  // Returns 1.2
     * }</pre>
     *
     * @param values Array of values
     * @return Minimum value, or Double.POSITIVE_INFINITY if array is empty
     */
    public static double min(double[] values) {
        if (values.length == 0) {
            return Double.POSITIVE_INFINITY;
        }
        double min = values[0];
        for (int i = 1; i < values.length; i++) {
            if (values[i] < min) {
                min = values[i];
            }
        }
        return min;
    }

    /**
     * Finds the index of the minimum value in an array.
     *
     * <p><strong>Example:</strong>
     * <pre>{@code
     * double[] distances = {2.5, 1.2, 3.1, 1.8};
     * int closestIndex = Utils.minIndex(distances);  // Returns 1
     * }</pre>
     *
     * @param values Array of values
     * @return Index of minimum value, or -1 if array is empty
     */
    public static int minIndex(double[] values) {
        if (values.length == 0) {
            return -1;
        }
        int minIndex = 0;
        double min = values[0];
        for (int i = 1; i < values.length; i++) {
            if (values[i] < min) {
                min = values[i];
                minIndex = i;
            }
        }
        return minIndex;
    }

    /**
     * Finds the index of the maximum value in an array.
     *
     * <p><strong>Example:</strong>
     * <pre>{@code
     * double[] speeds = {2.5, 4.2, 3.1, 1.8};
     * int fastestIndex = Utils.maxIndex(speeds);  // Returns 1
     * }</pre>
     *
     * @param values Array of values
     * @return Index of maximum value, or -1 if array is empty
     */
    public static int maxIndex(double[] values) {
        if (values.length == 0) {
            return -1;
        }
        int maxIndex = 0;
        double max = values[0];
        for (int i = 1; i < values.length; i++) {
            if (values[i] > max) {
                max = values[i];
                maxIndex = i;
            }
        }
        return maxIndex;
    }

    // ======================== TIME UTILITIES ========================

    /**
     * Checks if a specified amount of time has elapsed.
     *
     * <p><strong>Example:</strong>
     * <pre>{@code
     * private long actionStartTime;
     *
     * public void startAction() {
     *     actionStartTime = System.currentTimeMillis();
     * }
     *
     * public boolean isActionComplete() {
     *     return Utils.hasElapsed(actionStartTime, 2000);  // 2 seconds
     * }
     * }</pre>
     *
     * @param startTimeMs Start time from System.currentTimeMillis()
     * @param durationMs Duration to check in milliseconds
     * @return {@code true} if duration has elapsed
     */
    public static boolean hasElapsed(long startTimeMs, long durationMs) {
        return (System.currentTimeMillis() - startTimeMs) >= durationMs;
    }

    /**
     * Converts seconds to milliseconds.
     *
     * <p><strong>Example:</strong>
     * <pre>{@code
     * long timeoutMs = Utils.secondsToMillis(2.5);  // Returns 2500
     * }</pre>
     *
     * @param seconds Time in seconds
     * @return Time in milliseconds
     */
    public static long secondsToMillis(double seconds) {
        return (long) (seconds * 1000);
    }

    /**
     * Converts milliseconds to seconds.
     *
     * <p><strong>Example:</strong>
     * <pre>{@code
     * double seconds = Utils.millisToSeconds(2500);  // Returns 2.5
     * }</pre>
     *
     * @param millis Time in milliseconds
     * @return Time in seconds
     */
    public static double millisToSeconds(long millis) {
        return millis / 1000.0;
    }

    // ======================== ERROR REPORTING ========================

    /**
     * Reports an error to the DriverStation with a standardized format.
     *
     * <p>Use this method to report validation errors, initialization failures,
     * or other error conditions that should be visible to the drive team.
     *
     * <p><strong>Example:</strong>
     * <pre>{@code
     * if (!gyro.isConnected()) {
     *     Utils.reportError("Gyro not connected - check CAN bus");
     * }
     * }</pre>
     *
     * @param message The error message to display
     */
    public static void reportError(String message) {
        DriverStation.reportError(message, false);
    }

    /**
     * Reports a warning to the DriverStation with a standardized format.
     *
     * <p>Use this method to report non-critical issues that the drive team
     * should be aware of but don't prevent operation.
     *
     * <p><strong>Example:</strong>
     * <pre>{@code
     * if (battery.getVoltage() < 11.5) {
     *     Utils.reportWarning("Battery voltage low: " + battery.getVoltage());
     * }
     * }</pre>
     *
     * @param message The warning message to display
     */
    public static void reportWarning(String message) {
        DriverStation.reportWarning(message, false);
    }

    /**
     * Reports an info message to the DriverStation console.
     *
     * <p>Use this for non-critical informational messages.
     *
     * <p><strong>Example:</strong>
     * <pre>{@code
     * Utils.reportInfo("Autonomous path selected: " + pathName);
     * }</pre>
     *
     * @param message The info message to display
     */
    public static void reportInfo(String message) {
        System.out.println("[INFO] " + message);
    }
}
