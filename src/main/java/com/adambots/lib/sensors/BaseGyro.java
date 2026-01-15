package com.adambots.lib.sensors;

import edu.wpi.first.math.geometry.Rotation2d;

/**
 * Base interface for gyroscope sensors used in FRC robotics.
 *
 * <p>This interface provides a common API for all gyroscope implementations (e.g., Pigeon2, NavX).
 * Gyroscopes measure rotational motion and orientation in 3D space.
 *
 * <p><strong>Coordinate System:</strong> Counter-clockwise (CCW) rotation is positive,
 * following the right-hand rule with Z-axis pointing up.
 *
 * <p><strong>Usage Example:</strong>
 * <pre>{@code
 * BaseGyro gyro = new Pigeon2(1);
 * gyro.resetYaw();  // Zero the heading
 *
 * double heading = gyro.getContinuousYawDeg();  // Get current rotation
 * Rotation2d angle = gyro.getContinuousYawRotation2d();  // For WPILib geometry
 * }</pre>
 *
 * @see Rotation2d
 */
public interface BaseGyro {

    /**
     * Gets the continuous yaw angle in degrees.
     *
     * <p>Unlike wrapped angles (0-360°), continuous angles increase/decrease without
     * bounds, allowing for tracking total rotation beyond one revolution. For example,
     * rotating 450° clockwise returns -450, not 270.
     *
     * <p><strong>Direction:</strong> Counter-clockwise (CCW) is positive.
     *
     * @return Continuous yaw angle in degrees (unbounded, typically -∞ to +∞)
     */
    double getContinuousYawDeg();

    /**
     * Gets the continuous yaw angle in radians.
     *
     * <p>Same as {@link #getContinuousYawDeg()} but in radians for mathematical operations.
     *
     * <p><strong>Direction:</strong> Counter-clockwise (CCW) is positive.
     *
     * @return Continuous yaw angle in radians (unbounded, typically -∞ to +∞)
     */
    double getContinuousYawRad();

    /**
     * Gets the continuous yaw angle as a Rotation2d object.
     *
     * <p>Returns the yaw in WPILib's Rotation2d format for use with geometry classes
     * like Pose2d, Transform2d, etc. This is the preferred method for drivetrain code.
     *
     * @return Continuous yaw as Rotation2d object
     * @see Rotation2d
     * @see edu.wpi.first.math.geometry.Pose2d
     */
    Rotation2d getContinuousYawRotation2d();

    /**
     * Offsets the current yaw reading by a specified angle.
     *
     * <p>This adds an offset to all future yaw readings without changing the underlying
     * sensor value. Useful for field-relative driving where you want to set a new "forward"
     * direction without physically resetting the gyro.
     *
     * <p><strong>Example:</strong> If current yaw is 90° and you call offsetYawByAngle(10),
     * the yaw will now read 100°.
     *
     * @param offsetDeg Angle offset to add in degrees (positive CCW)
     */
    void offsetYawByAngle (double offsetDeg);

    /**
     * Resets the yaw angle to zero.
     *
     * <p>This sets the current orientation as the new zero reference point.
     * Commonly called during autonomous init or when resetting field-relative driving.
     */
    void resetYaw();

    /**
     * Resets the yaw angle to a specific value.
     *
     * <p>This sets the current orientation to read as the specified angle.
     * Useful when you know the robot's starting orientation (e.g., facing 180°
     * when starting against the opposite alliance wall).
     *
     * @param offsetDeg The angle value to set as current yaw in degrees
     */
    void resetYawToAngle(double offsetDeg);

    /**
     * Gets the current pitch angle in degrees.
     *
     * <p>Pitch represents forward/backward tilt. Positive pitch means the front of the
     * robot is tilted upward.
     *
     * <p><strong>Important:</strong> Pitch and roll measurements change relative to the
     * robot's yaw orientation. A 90° yaw rotation swaps pitch and roll axes.
     *
     * @return Pitch angle in degrees (typically -180 to +180)
     */
    double getPitch();

    /**
     * Gets the current roll angle in degrees.
     *
     * <p>Roll represents left/right tilt. Positive roll means the right side of the
     * robot is tilted downward.
     *
     * <p><strong>Important:</strong> Pitch and roll measurements change relative to the
     * robot's yaw orientation. A 90° yaw rotation swaps pitch and roll axes.
     *
     * @return Roll angle in degrees (typically -180 to +180)
     */
    double getRoll();

}