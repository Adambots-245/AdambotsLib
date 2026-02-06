// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.lib.vision;

import java.util.Optional;

/**
 * Interface representing a vision camera.
 *
 * <p>This interface provides an abstraction layer for vision cameras, allowing
 * different vision systems (PhotonVision, Limelight, etc.) to be used interchangeably
 * with the swerve subsystem.
 *
 * <p>For PhotonVision, this is implemented by {@code VisionCamera}.
 *
 * @see VisionResult
 * @see VisionSystem
 */
public interface VisionCameraInterface {

    /**
     * Gets the name of this camera.
     *
     * @return The camera name
     */
    String getName();

    /**
     * Checks if this camera is currently enabled.
     *
     * <p>Disabled cameras do not contribute to pose estimation.
     *
     * @return true if the camera is enabled
     */
    boolean isEnabled();

    /**
     * Enables this camera for pose estimation.
     */
    void enable();

    /**
     * Disables this camera from pose estimation.
     */
    void disable();

    /**
     * Checks if this camera currently has a target in view.
     *
     * @return true if at least one target is visible
     */
    boolean hasTarget();

    /**
     * Gets the result with the best (lowest ambiguity) target from the cache.
     *
     * <p>This may not be the most recent result if an older result had a
     * better target.
     *
     * @return The best result, or empty if no valid results
     */
    Optional<? extends VisionResult> getBestResult();

    /**
     * Gets the most recent result from this camera.
     *
     * @return The latest result, or empty if no results available
     */
    Optional<? extends VisionResult> getLatestResult();
}
