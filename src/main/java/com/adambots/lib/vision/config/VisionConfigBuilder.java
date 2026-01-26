// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.lib.vision.config;

import java.util.ArrayList;
import java.util.List;

import com.adambots.lib.vision.config.VisionCameraConfig.CameraPurpose;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

/**
 * Fluent builder for creating VisionSystemConfig objects.
 *
 * <p>This builder provides a clean, readable way to configure vision systems
 * year-over-year without modifying AdambotsLib source code.
 *
 * <p><strong>Usage Example:</strong>
 * <pre>{@code
 * // Define game-specific tag groups in your Constants file
 * public static final int[] REEF_TAGS = {6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22};
 * public static final int[] HP_TAGS = {1, 2, 4, 5, 12, 13, 14, 15};
 *
 * // Build the vision configuration
 * VisionSystemConfig config = VisionConfigBuilder.create()
 *     .addCamera("Left")
 *         .positionInches(15, 11.75, 8)
 *         .rotationDegrees(0, 0, -30)
 *         .purpose(CameraPurpose.ODOMETRY)
 *         .allowedTags(REEF_TAGS)
 *         .singleTagStdDevs(0.5, 0.5, 0.5)
 *         .multiTagStdDevs(0.5, 0.5, 1.0)
 *         .done()
 *     .addCamera("Right")
 *         .positionInches(15, -11.75, 8)
 *         .rotationDegrees(0, 0, 30)
 *         .purpose(CameraPurpose.ODOMETRY)
 *         .allowedTags(REEF_TAGS)
 *         .done()
 *     .addCamera("Middle")
 *         .positionInches(8, 0, 41)
 *         .rotationDegrees(0, -43, 177)
 *         .purpose(CameraPurpose.ALIGNMENT)
 *         .allowedTags(HP_TAGS)
 *         .done()
 *     .ambiguityThreshold(0.25)
 *     .maxPoseJump(10.0)
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
    private double maxPoseJumpMeters = VisionSystemConfig.DEFAULT_MAX_POSE_JUMP;

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
     * Sets the maximum allowed pose jump in meters.
     * <p>Pose estimates that would move the robot more than this distance
     * in a single frame are filtered as outliers.
     *
     * @param meters Maximum pose jump in meters
     * @return This builder for chaining
     */
    public VisionConfigBuilder maxPoseJump(double meters) {
        this.maxPoseJumpMeters = meters;
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
            ambiguityThreshold,
            maxPoseJumpMeters
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
     *   <li>Position (via positionMeters or positionInches)</li>
     *   <li>Rotation (via rotationRadians or rotationDegrees)</li>
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
         * Sets the camera position in meters relative to robot center.
         *
         * @param x Forward from robot center (positive = front)
         * @param y Left from robot center (positive = left)
         * @param z Height from floor
         * @return This builder for chaining
         */
        public CameraBuilder positionMeters(double x, double y, double z) {
            this.translation = new Translation3d(x, y, z);
            return this;
        }

        /**
         * Sets the camera position in inches relative to robot center.
         * <p>Values are automatically converted to meters.
         *
         * @param x Forward from robot center (positive = front)
         * @param y Left from robot center (positive = left)
         * @param z Height from floor
         * @return This builder for chaining
         */
        public CameraBuilder positionInches(double x, double y, double z) {
            this.translation = new Translation3d(
                Units.inchesToMeters(x),
                Units.inchesToMeters(y),
                Units.inchesToMeters(z)
            );
            return this;
        }

        /**
         * Sets the camera rotation in radians.
         *
         * @param roll Rotation around X axis (side-to-side tilt)
         * @param pitch Rotation around Y axis (up/down angle, positive = tilted up)
         * @param yaw Rotation around Z axis (left/right, positive = rotated left)
         * @return This builder for chaining
         */
        public CameraBuilder rotationRadians(double roll, double pitch, double yaw) {
            this.rotation = new Rotation3d(roll, pitch, yaw);
            return this;
        }

        /**
         * Sets the camera rotation in degrees.
         * <p>Values are automatically converted to radians.
         *
         * @param roll Rotation around X axis (side-to-side tilt)
         * @param pitch Rotation around Y axis (up/down angle, positive = tilted up)
         * @param yaw Rotation around Z axis (left/right, positive = rotated left)
         * @return This builder for chaining
         */
        public CameraBuilder rotationDegrees(double roll, double pitch, double yaw) {
            this.rotation = new Rotation3d(
                Units.degreesToRadians(roll),
                Units.degreesToRadians(pitch),
                Units.degreesToRadians(yaw)
            );
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
         * @param x X position uncertainty in meters
         * @param y Y position uncertainty in meters
         * @param theta Rotation uncertainty in radians
         * @return This builder for chaining
         */
        public CameraBuilder singleTagStdDevs(double x, double y, double theta) {
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
         * @param x X position uncertainty in meters
         * @param y Y position uncertainty in meters
         * @param theta Rotation uncertainty in radians
         * @return This builder for chaining
         */
        public CameraBuilder multiTagStdDevs(double x, double y, double theta) {
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
         * Completes this camera configuration and returns to the parent builder.
         *
         * @return The parent VisionConfigBuilder for chaining
         * @throws IllegalStateException if position or rotation hasn't been set
         */
        public VisionConfigBuilder done() {
            if (translation == null) {
                throw new IllegalStateException(
                    "Camera '" + name + "' position must be set using positionMeters() or positionInches()"
                );
            }
            if (rotation == null) {
                throw new IllegalStateException(
                    "Camera '" + name + "' rotation must be set using rotationRadians() or rotationDegrees()"
                );
            }

            VisionCameraConfig config = new VisionCameraConfig(
                name,
                purpose,
                translation,
                rotation,
                singleTagStdDevs,
                multiTagStdDevs,
                allowedTagIDs
            );

            parent.addCameraConfig(config);
            return parent;
        }
    }
}
