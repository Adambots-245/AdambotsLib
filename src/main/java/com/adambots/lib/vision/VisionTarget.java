// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.lib.vision;

import edu.wpi.first.math.geometry.Transform3d;

/**
 * Interface representing a tracked vision target.
 *
 * <p>This interface provides an abstraction layer for vision targets, allowing
 * different vision systems (PhotonVision, Limelight, etc.) to be used interchangeably
 * with the swerve subsystem.
 *
 * <p>For PhotonVision, this wraps {@code PhotonTrackedTarget}.
 *
 * @see VisionResult
 * @see VisionSystem
 */
public interface VisionTarget {

    /**
     * Gets the fiducial (AprilTag) ID of this target.
     *
     * @return The fiducial ID, or -1 if not an AprilTag target
     */
    int getFiducialId();

    /**
     * Gets the yaw (horizontal angle) to the target.
     *
     * <p>Positive values indicate the target is to the left of center,
     * negative values indicate the target is to the right.
     *
     * @return Yaw angle in degrees
     */
    double getYaw();

    /**
     * Gets the pitch (vertical angle) to the target.
     *
     * <p>Positive values indicate the target is above center,
     * negative values indicate the target is below.
     *
     * @return Pitch angle in degrees
     */
    double getPitch();

    /**
     * Gets the area of the target as a percentage of the image.
     *
     * @return Target area (0.0 to 100.0)
     */
    double getArea();

    /**
     * Gets the pose ambiguity of the target.
     *
     * <p>Lower values indicate a more confident pose estimate.
     * Values near 0 are highly confident, values near 1 are ambiguous.
     * A value of -1 typically indicates pose estimation is not available.
     *
     * @return Pose ambiguity (0.0 to 1.0), or -1 if not available
     */
    double getPoseAmbiguity();

    /**
     * Gets the best camera-to-target transform.
     *
     * <p>This transform represents the 3D position and rotation of the target
     * relative to the camera. The "best" transform is selected based on
     * pose ambiguity when multiple solutions exist.
     *
     * @return The Transform3d from camera to target
     */
    Transform3d getBestCameraToTarget();
}
