// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.lib.vision.config;

import java.util.ArrayList;
import java.util.List;

import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import com.adambots.lib.vision.config.VisionCameraConfig.CameraPurpose;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

/**
 * Fluent builder for creating VisionSystemConfig objects.
 *
 * <p>This builder provides a clean, readable way to configure vision systems
 * year-over-year without modifying AdambotsLib source code.
 *
 * <p><strong>Usage Example:</strong>
 * <pre>{@code
 * // Define game-specific tag groups in your Constants file
 * public static final int[] SCORING_TAGS = {6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22};
 * public static final int[] INTAKE_TAGS = {1, 2, 4, 5, 12, 13, 14, 15};
 *
 * // Build the vision configuration
 * VisionSystemConfig config = VisionConfigBuilder.create()
 *     .addCamera("Left")
 *         .position(Inches.of(15), Inches.of(11.75), Inches.of(8))
 *         .rotation(Degrees.of(0), Degrees.of(0), Degrees.of(-30))
 *         .purpose(CameraPurpose.ODOMETRY)
 *         .allowedTags(SCORING_TAGS)
 *         .singleTagStdDevs(Meters.of(0.5), Meters.of(0.5), Radians.of(0.5))
 *         .multiTagStdDevs(Meters.of(0.5), Meters.of(0.5), Radians.of(1.0))
 *         .done()
 *     .addCamera("Right")
 *         .position(Inches.of(15), Inches.of(-11.75), Inches.of(8))
 *         .rotation(Degrees.of(0), Degrees.of(0), Degrees.of(30))
 *         .purpose(CameraPurpose.ODOMETRY)
 *         .allowedTags(SCORING_TAGS)
 *         .done()
 *     .addCamera("Middle")
 *         .position(Inches.of(8), Inches.of(0), Inches.of(41))
 *         .rotation(Degrees.of(0), Degrees.of(-43), Degrees.of(177))
 *         .purpose(CameraPurpose.ALIGNMENT)
 *         .allowedTags(INTAKE_TAGS)
 *         .maxTagDistance(Meters.of(3.0))  // Limit turret camera range
 *         .done()
 *     .ambiguityThreshold(0.25)
 *     .build();
 * }</pre>
 *
 * @see VisionSystemConfig
 * @see VisionCameraConfig
 * @see CameraBuilder
 */
public class VisionConfigBuilder {

    private final List<VisionCameraConfig> cameras = new ArrayList<>();
    private double ambiguityThreshold = VisionSystemConfig.DEFAULT_AMBIGUITY;

    /**
     * Private constructor - use {@link #create()} to start building.
     */
    private VisionConfigBuilder() {}

    /**
     * Creates a new VisionConfigBuilder instance.
     *
     * @return A new builder instance
     */
    public static VisionConfigBuilder create() {
        return new VisionConfigBuilder();
    }

    /**
     * Starts building a new camera configuration.
     *
     * @param name The PhotonVision camera name (must match name in PV UI)
     * @return A CameraBuilder for configuring the camera
     */
    public CameraBuilder addCamera(String name) {
        return new CameraBuilder(this, name);
    }

    /**
     * Sets the ambiguity threshold for pose estimates.
     * <p>Pose estimates with ambiguity higher than this value will be rejected.
     *
     * @param threshold Ambiguity threshold (0-1, lower = stricter)
     * @return This builder for chaining
     */
    public VisionConfigBuilder ambiguityThreshold(double threshold) {
        this.ambiguityThreshold = threshold;
        return this;
    }

    /**
     * No-op. Pose jump filtering has been removed; the WPILib Kalman filter
     * handles outlier weighting via standard deviations.
     *
     * @param distance Ignored
     * @return This builder for chaining
     * @deprecated Hard pose-jump filtering has been removed. This method is a no-op.
     */
    @Deprecated
    public VisionConfigBuilder maxPoseJump(Distance distance) {
        return this;
    }

    /**
     * No-op. Heading jump filtering has been removed; the WPILib Kalman filter
     * handles outlier weighting via standard deviations.
     *
     * @param angle Ignored
     * @return This builder for chaining
     * @deprecated Hard heading-jump filtering has been removed. This method is a no-op.
     */
    @Deprecated
    public VisionConfigBuilder maxHeadingJump(Angle angle) {
        return this;
    }

    /**
     * Builds the final VisionSystemConfig.
     *
     * @return The configured VisionSystemConfig
     * @throws IllegalStateException if no cameras have been added
     */
    public VisionSystemConfig build() {
        if (cameras.isEmpty()) {
            throw new IllegalStateException("At least one camera must be configured");
        }
        return new VisionSystemConfig(
            List.copyOf(cameras),
            ambiguityThreshold
        );
    }

    /**
     * Internal method to add a completed camera configuration.
     * Called by CameraBuilder.done().
     */
    void addCameraConfig(VisionCameraConfig config) {
        cameras.add(config);
    }

    /**
     * Builder for configuring individual cameras.
     *
     * <p>Use this builder to configure camera position, rotation, standard
     * deviations, and tag filtering. Call {@link #done()} when finished
     * to return to the parent VisionConfigBuilder.
     *
     * <p><strong>Required settings:</strong>
     * <ul>
     *   <li>Position (via {@link #position(Distance, Distance, Distance)})</li>
     *   <li>Rotation (via {@link #rotation(Angle, Angle, Angle)})</li>
     * </ul>
     *
     * <p><strong>Optional settings (with defaults):</strong>
     * <ul>
     *   <li>Purpose - defaults to BOTH</li>
     *   <li>Single tag std devs - defaults to DEFAULT_SINGLE_TAG</li>
     *   <li>Multi tag std devs - defaults to DEFAULT_MULTI_TAG</li>
     *   <li>Allowed tags - defaults to all tags (no filtering)</li>
     * </ul>
     */
    public static class CameraBuilder {
        private final VisionConfigBuilder parent;
        private final String name;

        private Translation3d translation = null;
        private Rotation3d rotation = null;
        private CameraPurpose purpose = CameraPurpose.BOTH;
        private VisionStdDevs singleTagStdDevs = VisionStdDevs.DEFAULT_SINGLE_TAG;
        private VisionStdDevs multiTagStdDevs = VisionStdDevs.DEFAULT_MULTI_TAG;
        private int[] allowedTagIDs = new int[0];
        private double maxTagDistanceMeters = VisionCameraConfig.DEFAULT_MAX_TAG_DISTANCE;
        private PoseStrategy poseStrategy = VisionCameraConfig.DEFAULT_POSE_STRATEGY;

        /**
         * Creates a new CameraBuilder.
         *
         * @param parent The parent VisionConfigBuilder
         * @param name The camera name
         */
        CameraBuilder(VisionConfigBuilder parent, String name) {
            this.parent = parent;
            this.name = name;
        }

        /**
         * Sets the camera position relative to robot center.
         *
         * <p><strong>Coordinate System (Top-Down View):</strong>
         * <pre>
         *               FRONT OF ROBOT
         *                     |
         *                     | +X
         *         +Y &lt;--------+--------&gt; -Y
         *                     |
         *            (Robot   | -X
         *             Center)
         *                BACK OF ROBOT
         *
         *     Example: position(Inches.of(15), Inches.of(11.75), Inches.of(8))
         *              = 15" forward, 11.75" left, 8" high
         * </pre>
         *
         * @param x Forward from robot center (positive = front)
         * @param y Left from robot center (positive = left)
         * @param z Height from floor
         * @return This builder for chaining
         */
        public CameraBuilder position(Distance x, Distance y, Distance z) {
            this.translation = new Translation3d(x.in(Meters), y.in(Meters), z.in(Meters));
            return this;
        }

        /**
         * Sets the camera rotation.
         *
         * <p><strong>Yaw Direction (Top-Down View):</strong>
         * <pre>
         *         Robot Forward (0 deg)
         *                |
         *                |
         *     &lt;----------+----------&gt;
         *    +Yaw        |        -Yaw
         *    (left)      |       (right)
         *
         *     Example: rotation(Degrees.of(0), Degrees.of(0), Degrees.of(-30))
         *              = camera faces 30 deg to the RIGHT
         * </pre>
         *
         * @param roll Rotation around X axis (side-to-side tilt)
         * @param pitch Rotation around Y axis (up/down angle, positive = tilted up)
         * @param yaw Rotation around Z axis (left/right, positive = rotated left)
         * @return This builder for chaining
         */
        public CameraBuilder rotation(Angle roll, Angle pitch, Angle yaw) {
            this.rotation = new Rotation3d(roll.in(Radians), pitch.in(Radians), yaw.in(Radians));
            return this;
        }

        /**
         * Sets the camera purpose.
         *
         * @param purpose ODOMETRY, ALIGNMENT, or BOTH
         * @return This builder for chaining
         */
        public CameraBuilder purpose(CameraPurpose purpose) {
            this.purpose = purpose;
            return this;
        }

        /**
         * Sets the standard deviations for single-tag pose estimates.
         *
         * @param x X position uncertainty
         * @param y Y position uncertainty
         * @param theta Rotation uncertainty
         * @return This builder for chaining
         */
        public CameraBuilder singleTagStdDevs(Distance x, Distance y, Angle theta) {
            this.singleTagStdDevs = new VisionStdDevs(x, y, theta);
            return this;
        }

        /**
         * Sets the standard deviations for single-tag pose estimates.
         *
         * @param stdDevs VisionStdDevs configuration
         * @return This builder for chaining
         */
        public CameraBuilder singleTagStdDevs(VisionStdDevs stdDevs) {
            this.singleTagStdDevs = stdDevs;
            return this;
        }

        /**
         * Sets the standard deviations for multi-tag pose estimates.
         *
         * @param x X position uncertainty
         * @param y Y position uncertainty
         * @param theta Rotation uncertainty
         * @return This builder for chaining
         */
        public CameraBuilder multiTagStdDevs(Distance x, Distance y, Angle theta) {
            this.multiTagStdDevs = new VisionStdDevs(x, y, theta);
            return this;
        }

        /**
         * Sets the standard deviations for multi-tag pose estimates.
         *
         * @param stdDevs VisionStdDevs configuration
         * @return This builder for chaining
         */
        public CameraBuilder multiTagStdDevs(VisionStdDevs stdDevs) {
            this.multiTagStdDevs = stdDevs;
            return this;
        }

        /**
         * Sets the allowed AprilTag IDs for this camera.
         * <p>Only tags in this list will be processed by this camera.
         * Pass no arguments or an empty array to allow all tags.
         *
         * @param tagIDs The tag IDs to allow
         * @return This builder for chaining
         */
        public CameraBuilder allowedTags(int... tagIDs) {
            this.allowedTagIDs = tagIDs != null ? tagIDs : new int[0];
            return this;
        }

        /**
         * Sets the maximum distance for single-tag pose estimation.
         * <p>When only one AprilTag is visible, poses calculated from tags
         * beyond this distance will be rejected as unreliable.
         * <p>Default is 4.0 meters.
         *
         * @param distance Maximum distance for single-tag estimation
         * @return This builder for chaining
         */
        public CameraBuilder maxTagDistance(Distance distance) {
            this.maxTagDistanceMeters = distance.in(Meters);
            return this;
        }

        /**
         * Sets the pose estimation strategy for this camera.
         * <p>Default is {@link PoseStrategy#MULTI_TAG_PNP_ON_COPROCESSOR}.
         *
         * @param strategy The pose estimation strategy
         * @return This builder for chaining
         */
        public CameraBuilder poseStrategy(PoseStrategy strategy) {
            this.poseStrategy = strategy;
            return this;
        }

        /**
         * Completes this camera configuration and returns to the parent builder.
         *
         * @return The parent VisionConfigBuilder for chaining
         * @throws IllegalStateException if position or rotation hasn't been set
         */
        public VisionConfigBuilder done() {
            if (translation == null) {
                throw new IllegalStateException(
                    "Camera '" + name + "' position must be set using position()"
                );
            }
            if (rotation == null) {
                throw new IllegalStateException(
                    "Camera '" + name + "' rotation must be set using rotation()"
                );
            }

            VisionCameraConfig config = new VisionCameraConfig(
                name,
                purpose,
                translation,
                rotation,
                singleTagStdDevs,
                multiTagStdDevs,
                allowedTagIDs,
                maxTagDistanceMeters,
                poseStrategy
            );

            parent.addCameraConfig(config);
            return parent;
        }
    }
}
