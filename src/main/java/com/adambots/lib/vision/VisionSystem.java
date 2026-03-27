// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.lib.vision;

import java.util.List;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * Interface representing a complete vision system for pose estimation and target tracking.
 *
 * <p>This interface provides an abstraction layer so that the swerve subsystem can work
 * with any vision system (PhotonVision, Limelight, or custom implementations) without
 * being tied to specific vendor types.
 *
 * <p><strong>Implementations:</strong>
 * <ul>
 *   <li>{@code PhotonVision} - PhotonVision-based implementation</li>
 *   <li>Custom Limelight implementation (see documentation for example)</li>
 * </ul>
 *
 * <p><strong>Usage Example:</strong>
 * <pre>{@code
 * // In RobotContainer - circular dependency handling
 * // 1. Create swerve first (can run basic odometry without vision)
 * SwerveSubsystem swerve = new SwerveSubsystem(directory, config);
 *
 * // 2. Create vision with reference to swerve
 * VisionSystem vision = new PhotonVision(visionConfig, swerve::getPose, swerve.getField());
 *
 * // 3. Pass vision back to swerve
 * swerve.setupVision(vision);
 *
 * // 4. In periodic(), update pose estimation
 * vision.updatePoseEstimation(
 *     swerveDrive::addVisionMeasurement,
 *     swerveDrive::getSimulationDriveTrainPose
 * );
 * }</pre>
 *
 * @see VisionCameraInterface
 * @see VisionTarget
 * @see PhotonVision
 */
public interface VisionSystem {

    // ==================== POSE ESTIMATION ====================

    /**
     * Updates pose estimation by processing camera data and feeding measurements
     * to the provided consumer.
     *
     * <p>This method should be called periodically (typically in the subsystem's
     * periodic() method) to add vision measurements to the pose estimator.
     *
     * <p><strong>Usage Example:</strong>
     * <pre>{@code
     * vision.updatePoseEstimation(
     *     swerveDrive::addVisionMeasurement,
     *     swerveDrive::getSimulationDriveTrainPose
     * );
     * }</pre>
     *
     * @param visionConsumer Accepts (Pose2d pose, double timestampSeconds, Matrix stdDevs)
     * @param simPose Supplies the simulated drivetrain pose (for sim only), empty if not in sim
     */
    void updatePoseEstimation(
        VisionMeasurementConsumer visionConsumer,
        Supplier<Optional<Pose2d>> simPose
    );

    // ==================== TARGET DETECTION ====================

    /**
     * Checks if any target is currently visible in any enabled camera.
     *
     * @return true if at least one target is visible
     */
    boolean hasTarget();

    /**
     * Checks if a specific AprilTag is currently visible in any camera.
     *
     * @param tagID The AprilTag ID to check
     * @return true if the tag is visible
     */
    boolean isTagVisible(int tagID);

    /**
     * Gets the ID of the closest visible AprilTag.
     *
     * @return The closest tag ID, or -1 if no tags are visible
     */
    int getClosestVisibleTag();

    // ==================== TAG QUERIES ====================

    /**
     * Gets the distance from the robot to a specific AprilTag.
     *
     * @param tagID The AprilTag ID
     * @return Distance in meters, or -1.0 if the tag doesn't exist in the field layout
     */
    double getDistanceFromAprilTag(int tagID);

    /**
     * Gets the transform from the robot to a specific AprilTag.
     *
     * @param tagID The AprilTag ID
     * @return Transform2d from robot to tag
     */
    Transform2d getTransformToAprilTag(int tagID);

    /**
     * Gets the yaw angle from the robot to a specific AprilTag.
     *
     * <p>Positive values indicate the tag is to the left of the robot's heading,
     * negative values indicate the tag is to the right.
     *
     * @param tagID The AprilTag ID
     * @return Rotation2d representing yaw to the tag, or null if the tag doesn't exist
     */
    Rotation2d getYawToAprilTag(int tagID);

    // ==================== GEOMETRY HELPERS ====================

    /**
     * Computes the average position of a group of AprilTags from the field layout.
     *
     * <p>Useful for computing the center of a target structure composed of
     * multiple tags (e.g., speaker, amp). Uses static field layout positions,
     * not live detections.
     *
     * @param tagIds Array of AprilTag IDs to average
     * @return Average Translation2d of the tag positions, or (0, 0) if no tags found
     */
    Translation2d getTagGroupCenter(int[] tagIds);

    /**
     * Gets the distance from the robot's current pose to an arbitrary field point.
     *
     * @param target The field point to measure distance to
     * @return Distance in meters
     */
    double getDistanceToPoint(Translation2d target);

    /**
     * Gets the relative yaw angle from the robot's current heading to an arbitrary field point.
     *
     * <p>Positive values indicate the target is to the left of the robot's heading,
     * negative values indicate the target is to the right.
     *
     * @param target The field point to aim at
     * @return Rotation2d representing relative yaw to the target
     */
    Rotation2d getYawToPoint(Translation2d target);

    /**
     * Counts how many tags from a filtered set are currently visible with
     * acceptable ambiguity.
     *
     * <p>Each tag ID is counted at most once, even if seen by multiple cameras.
     *
     * @param filterIds Array of AprilTag IDs to look for
     * @param maxAmbiguity Maximum pose ambiguity threshold (0-1); tags above this are ignored
     * @return Number of unique matching tags currently visible
     */
    int getVisibleTagCount(int[] filterIds, double maxAmbiguity);

    // ==================== RUNTIME TUNING ====================

    /**
     * Sets a dynamic scaler for vision standard deviations.
     *
     * <p>The scaler is evaluated each cycle and multiplied into the computed std devs
     * before passing to the pose estimator. Use this to reduce vision trust at high
     * robot speeds (motion blur, latency).
     *
     * <p><strong>Example:</strong>
     * <pre>{@code
     * vision.setStdDevScaler(() -> {
     *     double speed = Math.hypot(
     *         swerve.getRobotVelocity().vxMetersPerSecond,
     *         swerve.getRobotVelocity().vyMetersPerSecond);
     *     return 1.0 + speed;  // trust vision less at higher speeds
     * });
     * }</pre>
     *
     * @param scaler Supplier returning the scale factor (1.0 = no change). Null resets to 1.0.
     */
    default void setStdDevScaler(DoubleSupplier scaler) {}

    /**
     * Sets a runtime tag filter restricting which AprilTags are used for pose estimation.
     *
     * <p>Pass {@code null} or an empty array to restore the config-time tag filter
     * (or allow all tags if no config-time filter was set).
     *
     * <p><strong>Example:</strong>
     * <pre>{@code
     * // During shooting — only use hub tags:
     * vision.setTagFilter(new int[]{1, 2, 3, 4});
     *
     * // After shooting — restore default filtering:
     * vision.setTagFilter(null);
     * }</pre>
     *
     * @param allowedTagIds Array of allowed tag IDs, or null/empty to restore defaults
     */
    default void setTagFilter(int[] allowedTagIds) {}

    // ==================== CAMERA MANAGEMENT ====================

    /**
     * Gets all configured cameras.
     *
     * @return List of VisionCameraInterface instances
     */
    List<? extends VisionCameraInterface> getCameras();

    /**
     * Gets a specific camera by name.
     *
     * @param name The camera name
     * @return The camera, or null if not found
     */
    VisionCameraInterface getCamera(String name);

    /**
     * Enables all cameras for pose estimation.
     */
    void enableAllCameras();

    /**
     * Disables all cameras from pose estimation.
     *
     * <p>When disabled, no vision measurements will be added to odometry.
     */
    void disableAllCameras();

    // ==================== TARGET QUERIES ====================

    /**
     * Gets the best (lowest ambiguity) target from a specific camera.
     *
     * @param cameraName Name of the camera
     * @return The best target wrapped as VisionTarget, or empty if no targets
     */
    Optional<? extends VisionTarget> getBestTargetFromCamera(String cameraName);
}
