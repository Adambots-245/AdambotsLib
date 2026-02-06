// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.lib.vision;

import java.util.List;
import java.util.Optional;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import swervelib.SwerveDrive;

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
 * }</pre>
 *
 * @see VisionCameraInterface
 * @see VisionTarget
 * @see PhotonVision
 */
public interface VisionSystem {

    // ==================== POSE ESTIMATION ====================

    /**
     * Updates the pose estimation for the swerve drive.
     *
     * <p>This method should be called periodically (typically in the subsystem's
     * periodic() method) to add vision measurements to the pose estimator.
     *
     * @param swerveDrive The SwerveDrive instance to update
     */
    void updatePoseEstimation(SwerveDrive swerveDrive);

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
