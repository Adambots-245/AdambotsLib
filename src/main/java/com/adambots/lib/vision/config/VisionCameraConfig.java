// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.lib.vision.config;

import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;

/**
 * Configuration record for a PhotonVision camera.
 *
 * <p>This record holds all configuration necessary to set up a vision camera
 * for pose estimation and target tracking. Each camera can be configured
 * independently with its own position, rotation, standard deviations, and
 * tag filtering.
 *
 * <p><strong>Camera Purpose:</strong>
 * <ul>
 *   <li>{@link CameraPurpose#ODOMETRY} - Camera used only for pose estimation</li>
 *   <li>{@link CameraPurpose#ALIGNMENT} - Camera used only for targeting/alignment</li>
 *   <li>{@link CameraPurpose#BOTH} - Camera used for both purposes</li>
 * </ul>
 *
 * <p><strong>Coordinate System (Top-Down View):</strong>
 * <pre>
 *               FRONT OF ROBOT
 *                     |
 *                     | +X
 *                     |
 *         +Y &lt;--------+--------&gt; -Y
 *                     |
 *            (Robot   |
 *             Center) |
 *                     | -X
 *                BACK OF ROBOT
 * </pre>
 *
 * <p><strong>Height (Side View):</strong>
 * <pre>
 *     ^ +Z (height from floor)
 *     |
 *     |    [CAM] &lt;-- Camera at Z height
 *     |      |
 *     |   [ROBOT]
 * ----+------------- Floor (Z=0)
 * </pre>
 *
 * <p><strong>Rotation:</strong>
 * <ul>
 *   <li>Roll: Rotation around X axis (side-to-side tilt)</li>
 *   <li>Pitch: Rotation around Y axis (positive = tilted up)</li>
 *   <li>Yaw: Rotation around Z axis (positive = rotated left, negative = right)</li>
 * </ul>
 *
 * <p><strong>Usage Example:</strong>
 * <pre>{@code
 * VisionCameraConfig config = new VisionCameraConfig(
 *     "LeftCam",
 *     CameraPurpose.ODOMETRY,
 *     new Translation3d(0.381, 0.298, 0.203),  // Position in meters
 *     new Rotation3d(0, 0, Math.toRadians(-30)),  // Rotation
 *     VisionStdDevs.DEFAULT_SINGLE_TAG,
 *     VisionStdDevs.DEFAULT_MULTI_TAG,
 *     new int[]{6, 7, 8, 9, 10, 11},  // Allowed tags
 *     4.0,  // Max tag distance (meters) - use default
 *     0.0,  // Min tag area percent (0.0 = disabled)
 *     PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR  // Pose strategy
 * );
 * }</pre>
 *
 * @param name PhotonVision camera name (must match name in PhotonVision UI)
 * @param purpose Camera purpose (ODOMETRY, ALIGNMENT, or BOTH)
 * @param robotToCamTranslation Position of camera relative to robot center in meters
 * @param robotToCamRotation Rotation of camera relative to robot forward direction
 * @param singleTagStdDevs Standard deviations for single-tag pose estimates
 * @param multiTagStdDevs Standard deviations for multi-tag pose estimates
 * @param allowedTagIDs Array of AprilTag IDs this camera should process (empty = all tags)
 * @param maxTagDistanceMeters Maximum distance (meters) for single-tag pose estimation (default 4.0)
 * @param minTagAreaPercent Minimum tag image area percentage to accept (0.0 = disabled)
 * @param poseStrategy PhotonVision pose estimation strategy (default MULTI_TAG_PNP_ON_COPROCESSOR)
 *
 * @see VisionStdDevs
 * @see VisionConfigBuilder
 */
public record VisionCameraConfig(
    String name,
    CameraPurpose purpose,
    Translation3d robotToCamTranslation,
    Rotation3d robotToCamRotation,
    VisionStdDevs singleTagStdDevs,
    VisionStdDevs multiTagStdDevs,
    int[] allowedTagIDs,
    double maxTagDistanceMeters,
    double minTagAreaPercent,
    PoseStrategy poseStrategy
) {

    /**
     * Default maximum tag distance in meters for single-tag pose estimation.
     * Tags beyond this distance are rejected when only one tag is visible.
     */
    public static final double DEFAULT_MAX_TAG_DISTANCE = 4.0;

    /**
     * Default pose estimation strategy.
     * MULTI_TAG_PNP_ON_COPROCESSOR is more robust than AVERAGE_BEST_TARGETS,
     * which naively averages headings and breaks near +/-180 degrees.
     */
    public static final PoseStrategy DEFAULT_POSE_STRATEGY = PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR;

    /**
     * Defines the purpose of a vision camera.
     *
     * <p>This allows filtering cameras based on their intended use:
     * <ul>
     *   <li><strong>ODOMETRY</strong> - Camera contributes to robot pose estimation.
     *       These cameras should have good field coverage and see AprilTags
     *       for general localization.</li>
     *   <li><strong>ALIGNMENT</strong> - Camera used for precise targeting/alignment.
     *       These cameras might be optimized for seeing specific game elements
     *       but not used for general pose estimation.</li>
     *   <li><strong>BOTH</strong> - Camera serves both purposes.</li>
     * </ul>
     */
    public enum CameraPurpose {
        /**
         * Camera used for pose estimation/odometry only.
         * Vision measurements from this camera will be added to the pose estimator.
         */
        ODOMETRY,

        /**
         * Camera used for targeting/alignment only.
         * Vision measurements from this camera will NOT be added to pose estimation,
         * but can be used for precise alignment to game elements.
         */
        ALIGNMENT,

        /**
         * Camera used for both pose estimation and alignment.
         */
        BOTH
    }

    /**
     * Checks if this camera should be used for odometry/pose estimation.
     *
     * @return true if camera purpose is ODOMETRY or BOTH
     */
    public boolean isForOdometry() {
        return purpose == CameraPurpose.ODOMETRY || purpose == CameraPurpose.BOTH;
    }

    /**
     * Checks if this camera should be used for alignment/targeting.
     *
     * @return true if camera purpose is ALIGNMENT or BOTH
     */
    public boolean isForAlignment() {
        return purpose == CameraPurpose.ALIGNMENT || purpose == CameraPurpose.BOTH;
    }

    /**
     * Checks if this camera has tag filtering enabled.
     *
     * @return true if allowedTagIDs is non-null and non-empty
     */
    public boolean hasTagFilter() {
        return allowedTagIDs != null && allowedTagIDs.length > 0;
    }

    /**
     * Checks if a specific tag ID is allowed by this camera's filter.
     *
     * @param tagID The AprilTag ID to check
     * @return true if the tag is allowed (or if no filter is set)
     */
    public boolean isTagAllowed(int tagID) {
        if (!hasTagFilter()) {
            return true;  // No filter = all tags allowed
        }
        for (int allowed : allowedTagIDs) {
            if (allowed == tagID) {
                return true;
            }
        }
        return false;
    }
}
