package com.adambots.lib.sensors;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;

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
 * import static edu.wpi.first.units.Units.*;
 *
 * BaseGyro gyro = new Gyro(1);
 * gyro.resetYaw();  // Zero the heading
 *
 * Angle heading = gyro.getYaw();  // Get current rotation
 * double degrees = heading.in(Degrees);
 * double radians = heading.in(Radians);
 *
 * Rotation2d angle = gyro.getYawRotation2d();  // For WPILib geometry
 * }</pre>
 *
 * @see Rotation2d
 */
@Logged
public interface BaseGyro {

    /**
     * Gets the continuous yaw angle.
     *
     * <p>Unlike wrapped angles (0-360°), continuous angles increase/decrease without
     * bounds, allowing for tracking total rotation beyond one revolution. For example,
     * rotating 450° clockwise returns -450°, not 270°.
     *
     * <p><strong>Direction:</strong> Counter-clockwise (CCW) is positive.
     *
     * <p>Use {@code .in(Degrees)} or {@code .in(Radians)} to convert:
     * <pre>{@code
     * Angle yaw = gyro.getYaw();
     * double degrees = yaw.in(Degrees);
     * double radians = yaw.in(Radians);
     * }</pre>
     *
     * @return Continuous yaw angle (unbounded)
     */
    Angle getYaw();

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
    Rotation2d getYawRotation2d();

    /**
     * Offsets the current yaw reading by a specified angle.
     *
     * <p>This adds an offset to all future yaw readings without changing the underlying
     * sensor value. Useful for field-relative driving where you want to set a new "forward"
     * direction without physically resetting the gyro.
     *
     * <p><strong>Example:</strong> If current yaw is 90° and you call offsetYawByAngle(Degrees.of(10)),
     * the yaw will now read 100°.
     *
     * @param offset Angle offset to add (positive CCW)
     */
    void offsetYawByAngle(Angle offset);

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
     * @param angle The angle value to set as current yaw
     */
    void resetYawToAngle(Angle angle);

    /**
     * Gets the current pitch angle.
     *
     * <p>Pitch represents forward/backward tilt. Positive pitch means the front of the
     * robot is tilted upward.
     *
     * <p><strong>Important:</strong> Pitch and roll measurements change relative to the
     * robot's yaw orientation. A 90° yaw rotation swaps pitch and roll axes.
     *
     * @return Pitch angle
     */
    Angle getPitch();

    /**
     * Gets the current roll angle.
     *
     * <p>Roll represents left/right tilt. Positive roll means the right side of the
     * robot is tilted downward.
     *
     * <p><strong>Important:</strong> Pitch and roll measurements change relative to the
     * robot's yaw orientation. A 90° yaw rotation swaps pitch and roll axes.
     *
     * @return Roll angle
     */
    Angle getRoll();

}
