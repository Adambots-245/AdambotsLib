// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.lib.vision.config;

import java.util.List;

import com.adambots.lib.vision.config.VisionCameraConfig.CameraPurpose;

/**
 * Configuration record for the complete vision system.
 *
 * <p>This record holds all configuration for a PhotonVision-based vision system,
 * including all camera configurations and system-wide parameters.
 *
 * <p><strong>Usage Example:</strong>
 * <pre>{@code
 * VisionSystemConfig config = VisionConfigBuilder.create()
 *     .addCamera("Left")
 *         .positionInches(15, 11.75, 8)
 *         .rotationDegrees(0, 0, -30)
 *         .purpose(CameraPurpose.ODOMETRY)
 *         .allowedTags(6, 7, 8, 9, 10, 11)
 *         .done()
 *     .addCamera("Right")
 *         .positionInches(15, -11.75, 8)
 *         .rotationDegrees(0, 0, 30)
 *         .purpose(CameraPurpose.ODOMETRY)
 *         .done()
 *     .ambiguityThreshold(0.25)
 *     .build();
 *
 * // Use the config
 * PhotonVision vision = new PhotonVision(config, poseSupplier, field);
 * }</pre>
 *
 * @param cameras List of camera configurations
 * @param ambiguityThreshold Maximum ambiguity for pose estimates (0-1, lower = stricter)
 * @param maxPoseJumpMeters Maximum allowed pose jump in meters (filters outliers)
 *
 * @see VisionCameraConfig
 * @see VisionConfigBuilder
 */
public record VisionSystemConfig(
    List<VisionCameraConfig> cameras,
    double ambiguityThreshold,
    double maxPoseJumpMeters
) {

    /**
     * Default ambiguity threshold for filtering pose estimates.
     * <p>Values between 0-1, where lower values are more strict.
     * A value of 0.25 means pose estimates with ambiguity > 0.25 are rejected.
     */
    public static final double DEFAULT_AMBIGUITY = 0.25;

    /**
     * Default maximum pose jump in meters.
     * <p>Pose estimates that would move the robot more than this distance
     * in a single frame are likely outliers and should be filtered.
     */
    public static final double DEFAULT_MAX_POSE_JUMP = 10.0;

    /**
     * Gets the number of configured cameras.
     *
     * @return Number of cameras
     */
    public int getCameraCount() {
        return cameras.size();
    }

    /**
     * Gets a camera configuration by name.
     *
     * @param name The camera name to find
     * @return The camera config, or null if not found
     */
    public VisionCameraConfig getCamera(String name) {
        for (VisionCameraConfig camera : cameras) {
            if (camera.name().equals(name)) {
                return camera;
            }
        }
        return null;
    }

    /**
     * Gets all cameras with a specific purpose.
     *
     * @param purpose The purpose to filter by
     * @return List of cameras matching the purpose
     */
    public List<VisionCameraConfig> getCamerasWithPurpose(CameraPurpose purpose) {
        return cameras.stream()
            .filter(c -> c.purpose() == purpose || c.purpose() == CameraPurpose.BOTH)
            .toList();
    }

    /**
     * Gets all cameras configured for odometry/pose estimation.
     *
     * @return List of cameras for odometry
     */
    public List<VisionCameraConfig> getOdometryCameras() {
        return cameras.stream()
            .filter(VisionCameraConfig::isForOdometry)
            .toList();
    }

    /**
     * Gets all cameras configured for alignment/targeting.
     *
     * @return List of cameras for alignment
     */
    public List<VisionCameraConfig> getAlignmentCameras() {
        return cameras.stream()
            .filter(VisionCameraConfig::isForAlignment)
            .toList();
    }

    /**
     * Checks if a camera name exists in the configuration.
     *
     * @param name Camera name to check
     * @return true if a camera with that name exists
     */
    public boolean hasCamera(String name) {
        return getCamera(name) != null;
    }
}
