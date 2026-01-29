// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.lib.targets;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform2d;

import java.util.Optional;

/**
 * Represents a game-specific target location defined relative to AprilTags.
 *
 * <p>A GameTarget maps a semantic name (e.g., "tower-blue", "hub-red") to:
 * <ul>
 *   <li>One or more AprilTag IDs that define the target's reference position</li>
 *   <li>An offset from the tag position where the robot should position</li>
 *   <li>Optional tolerance values for alignment checking</li>
 * </ul>
 *
 * <p>Tag positions come from WPILib's AprilTagFieldLayout (official field data
 * or custom scans from WPICal/PractiCal). This class only stores the mapping
 * and offset, not the actual tag positions.
 *
 * <p><strong>Usage Example:</strong>
 * <pre>{@code
 * // Create a target 0.5m in front of tag 15, facing 180 degrees
 * GameTarget tower = new GameTarget(
 *     "tower-blue",
 *     new int[]{15, 16},
 *     new Transform2d(
 *         new Translation2d(-0.5, 0),
 *         Rotation2d.fromDegrees(180)
 *     ),
 *     new GameTarget.Tolerance(0.05, 2.0)
 * );
 *
 * // Get the actual pose using field layout
 * Pose2d targetPose = tower.getTargetPose(fieldLayout);
 * }</pre>
 *
 * @see GameTargetConfig
 * @see GameTargetConfigBuilder
 */
public class GameTarget {

    private final String name;
    private final int[] tagIds;
    private final Transform2d offset;
    private final Tolerance tolerance;

    /**
     * Tolerance values for target alignment checking.
     */
    public static class Tolerance {
        private final double positionMeters;
        private final double rotationDegrees;

        /**
         * Creates a new Tolerance with specified values.
         *
         * @param positionMeters Position tolerance in meters
         * @param rotationDegrees Rotation tolerance in degrees
         */
        public Tolerance(double positionMeters, double rotationDegrees) {
            this.positionMeters = positionMeters;
            this.rotationDegrees = rotationDegrees;
        }

        /**
         * Creates a default tolerance (0.1m position, 5 degrees rotation).
         *
         * @return Default tolerance values
         */
        public static Tolerance defaults() {
            return new Tolerance(0.1, 5.0);
        }

        public double getPositionMeters() {
            return positionMeters;
        }

        public double getRotationDegrees() {
            return rotationDegrees;
        }
    }

    /**
     * Creates a new GameTarget with all parameters.
     *
     * @param name Unique name for this target (e.g., "tower-blue")
     * @param tagIds Array of AprilTag IDs that define this target
     * @param offset Transform from tag pose to robot target pose
     * @param tolerance Optional tolerance for alignment checking (null for defaults)
     */
    public GameTarget(String name, int[] tagIds, Transform2d offset, Tolerance tolerance) {
        this.name = name;
        this.tagIds = tagIds;
        this.offset = offset;
        this.tolerance = tolerance != null ? tolerance : Tolerance.defaults();
    }

    /**
     * Creates a new GameTarget with default tolerance.
     *
     * @param name Unique name for this target
     * @param tagIds Array of AprilTag IDs that define this target
     * @param offset Transform from tag pose to robot target pose
     */
    public GameTarget(String name, int[] tagIds, Transform2d offset) {
        this(name, tagIds, offset, null);
    }

    /**
     * Gets the target name.
     *
     * @return Target name
     */
    public String getName() {
        return name;
    }

    /**
     * Gets the AprilTag IDs associated with this target.
     *
     * @return Array of tag IDs
     */
    public int[] getTagIds() {
        return tagIds;
    }

    /**
     * Gets the primary (first) tag ID for this target.
     *
     * @return Primary tag ID
     */
    public int getPrimaryTagId() {
        return tagIds[0];
    }

    /**
     * Gets the offset transform from tag to target position.
     *
     * @return Offset transform
     */
    public Transform2d getOffset() {
        return offset;
    }

    /**
     * Gets the tolerance values for this target.
     *
     * @return Tolerance values
     */
    public Tolerance getTolerance() {
        return tolerance;
    }

    /**
     * Calculates the target pose by applying the offset to the primary tag's position.
     *
     * <p>The tag position is retrieved from the provided AprilTagFieldLayout,
     * which may contain official field data or custom positions from WPICal/PractiCal.
     *
     * @param fieldLayout AprilTag field layout with tag positions
     * @return Target pose, or empty if primary tag not found in layout
     */
    public Optional<Pose2d> getTargetPose(AprilTagFieldLayout fieldLayout) {
        Optional<Pose3d> tagPose = fieldLayout.getTagPose(getPrimaryTagId());
        if (tagPose.isEmpty()) {
            return Optional.empty();
        }

        Pose2d tagPose2d = tagPose.get().toPose2d();
        return Optional.of(tagPose2d.transformBy(offset));
    }

    /**
     * Gets the target pose, throwing if the tag is not found.
     *
     * @param fieldLayout AprilTag field layout with tag positions
     * @return Target pose
     * @throws IllegalStateException if primary tag not found in layout
     */
    public Pose2d getTargetPoseOrThrow(AprilTagFieldLayout fieldLayout) {
        return getTargetPose(fieldLayout)
            .orElseThrow(() -> new IllegalStateException(
                "Tag " + getPrimaryTagId() + " not found in field layout for target '" + name + "'"));
    }

    /**
     * Checks if a pose is within tolerance of this target.
     *
     * @param fieldLayout AprilTag field layout with tag positions
     * @param pose Pose to check
     * @return true if within tolerance
     */
    public boolean isAtTarget(AprilTagFieldLayout fieldLayout, Pose2d pose) {
        Optional<Pose2d> targetPose = getTargetPose(fieldLayout);
        if (targetPose.isEmpty()) {
            return false;
        }

        double distance = pose.getTranslation().getDistance(targetPose.get().getTranslation());
        double rotationError = Math.abs(pose.getRotation().minus(targetPose.get().getRotation()).getDegrees());

        return distance <= tolerance.positionMeters && rotationError <= tolerance.rotationDegrees;
    }

    /**
     * Gets the distance from a pose to this target.
     *
     * @param fieldLayout AprilTag field layout with tag positions
     * @param pose Pose to measure from
     * @return Distance in meters, or -1 if tag not found
     */
    public double getDistance(AprilTagFieldLayout fieldLayout, Pose2d pose) {
        Optional<Pose2d> targetPose = getTargetPose(fieldLayout);
        if (targetPose.isEmpty()) {
            return -1;
        }
        return pose.getTranslation().getDistance(targetPose.get().getTranslation());
    }

    @Override
    public String toString() {
        return "GameTarget{name='" + name + "', tags=" + java.util.Arrays.toString(tagIds) + "}";
    }
}
