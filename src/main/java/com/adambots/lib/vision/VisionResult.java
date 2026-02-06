// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.lib.vision;

import java.util.List;

/**
 * Interface representing a vision pipeline result.
 *
 * <p>This interface provides an abstraction layer for vision results, allowing
 * different vision systems (PhotonVision, Limelight, etc.) to be used interchangeably
 * with the swerve subsystem.
 *
 * <p>For PhotonVision, this wraps {@code PhotonPipelineResult}.
 *
 * @see VisionTarget
 * @see VisionCameraInterface
 */
public interface VisionResult {

    /**
     * Checks if this result contains any detected targets.
     *
     * @return true if at least one target was detected
     */
    boolean hasTargets();

    /**
     * Gets the best target from this result.
     *
     * <p>The "best" target is typically the one with the lowest pose ambiguity
     * or highest confidence.
     *
     * @return The best VisionTarget, or null if no targets were detected
     */
    VisionTarget getBestTarget();

    /**
     * Gets all targets detected in this result.
     *
     * @return List of all detected targets, may be empty
     */
    List<? extends VisionTarget> getTargets();

    /**
     * Gets the timestamp of when this result was captured.
     *
     * <p>This timestamp is used for latency compensation when adding
     * vision measurements to the pose estimator.
     *
     * @return Timestamp in seconds (FPGA time)
     */
    double getTimestampSeconds();
}
