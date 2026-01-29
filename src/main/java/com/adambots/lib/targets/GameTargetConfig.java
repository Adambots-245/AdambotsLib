// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.lib.targets;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;

import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;

/**
 * Configuration holding all game targets for a specific game year.
 *
 * <p>This class provides lookup methods to find targets by name and
 * alliance-aware helpers for automatic target selection.
 *
 * <p><strong>Target Naming Convention:</strong>
 * Alliance-specific targets should use the naming convention:
 * <ul>
 *   <li>{@code "targetname-blue"} for blue alliance targets</li>
 *   <li>{@code "targetname-red"} for red alliance targets</li>
 * </ul>
 *
 * <p>This allows using {@link #getAllianceTarget(String, boolean)} to
 * automatically select the correct target based on alliance color.
 *
 * <p><strong>Usage Example:</strong>
 * <pre>{@code
 * // Load from JSON
 * GameTargetConfig config = GameTargetLoader.load("gametargets.json");
 *
 * // Or use builder
 * GameTargetConfig config = GameTargetConfigBuilder.create()
 *     .addTarget("tower-blue").tagIds(15, 16).offsetMeters(-0.5, 0).rotation(180).done()
 *     .addTarget("tower-red").tagIds(31, 32).offsetMeters(-0.5, 0).rotation(0).done()
 *     .build();
 *
 * // Get target by name
 * GameTarget tower = config.getTarget("tower-blue").get();
 *
 * // Get alliance-appropriate target
 * GameTarget allianceTower = config.getAllianceTarget("tower", isRed).get();
 *
 * // Get target pose
 * Pose2d targetPose = tower.getTargetPose(fieldLayout).get();
 * }</pre>
 *
 * @see GameTarget
 * @see GameTargetConfigBuilder
 * @see GameTargetLoader
 */
public class GameTargetConfig {

    private final int gameYear;
    private final String gameName;
    private final Map<String, GameTarget> targets;

    /**
     * Creates a new GameTargetConfig.
     *
     * @param gameYear FRC game year (e.g., 2026)
     * @param gameName FRC game name (e.g., "REBUILT")
     * @param targets List of game targets
     */
    public GameTargetConfig(int gameYear, String gameName, List<GameTarget> targets) {
        this.gameYear = gameYear;
        this.gameName = gameName;
        this.targets = new HashMap<>();
        for (GameTarget target : targets) {
            this.targets.put(target.getName(), target);
        }
    }

    /**
     * Gets the FRC game year.
     *
     * @return Game year (e.g., 2026)
     */
    public int getGameYear() {
        return gameYear;
    }

    /**
     * Gets the FRC game name.
     *
     * @return Game name (e.g., "REBUILT")
     */
    public String getGameName() {
        return gameName;
    }

    /**
     * Gets all targets in this configuration.
     *
     * @return Unmodifiable map of target name to GameTarget
     */
    public Map<String, GameTarget> getAllTargets() {
        return Collections.unmodifiableMap(targets);
    }

    /**
     * Gets a target by name.
     *
     * @param name Target name
     * @return The target, or empty if not found
     */
    public Optional<GameTarget> getTarget(String name) {
        return Optional.ofNullable(targets.get(name));
    }

    /**
     * Gets a target by name, throwing if not found.
     *
     * @param name Target name
     * @return The target
     * @throws IllegalArgumentException if target not found
     */
    public GameTarget getTargetOrThrow(String name) {
        return getTarget(name)
            .orElseThrow(() -> new IllegalArgumentException("Target not found: " + name));
    }

    /**
     * Gets the alliance-specific target based on the base name and alliance.
     *
     * <p>Appends "-red" or "-blue" to the base name depending on alliance.
     *
     * @param baseName Base target name (without alliance suffix)
     * @param isRedAlliance True if on red alliance
     * @return The alliance-specific target, or empty if not found
     */
    public Optional<GameTarget> getAllianceTarget(String baseName, boolean isRedAlliance) {
        String suffix = isRedAlliance ? "-red" : "-blue";
        return getTarget(baseName + suffix);
    }

    /**
     * Gets the alliance-specific target, throwing if not found.
     *
     * @param baseName Base target name (without alliance suffix)
     * @param isRedAlliance True if on red alliance
     * @return The alliance-specific target
     * @throws IllegalArgumentException if target not found
     */
    public GameTarget getAllianceTargetOrThrow(String baseName, boolean isRedAlliance) {
        return getAllianceTarget(baseName, isRedAlliance)
            .orElseThrow(() -> new IllegalArgumentException(
                "Alliance target not found: " + baseName + (isRedAlliance ? "-red" : "-blue")));
    }

    /**
     * Gets the target pose by name.
     *
     * @param fieldLayout AprilTag field layout
     * @param targetName Target name
     * @return Target pose, or empty if target or tag not found
     */
    public Optional<Pose2d> getTargetPose(AprilTagFieldLayout fieldLayout, String targetName) {
        return getTarget(targetName).flatMap(t -> t.getTargetPose(fieldLayout));
    }

    /**
     * Gets the alliance-specific target pose.
     *
     * @param fieldLayout AprilTag field layout
     * @param baseName Base target name (without alliance suffix)
     * @param isRedAlliance True if on red alliance
     * @return Target pose, or empty if target or tag not found
     */
    public Optional<Pose2d> getAllianceTargetPose(AprilTagFieldLayout fieldLayout, String baseName, boolean isRedAlliance) {
        return getAllianceTarget(baseName, isRedAlliance).flatMap(t -> t.getTargetPose(fieldLayout));
    }

    /**
     * Checks if a target with the given name exists.
     *
     * @param name Target name
     * @return true if target exists
     */
    public boolean hasTarget(String name) {
        return targets.containsKey(name);
    }

    /**
     * Gets the number of targets in this configuration.
     *
     * @return Number of targets
     */
    public int size() {
        return targets.size();
    }

    @Override
    public String toString() {
        return "GameTargetConfig{year=" + gameYear + ", game='" + gameName + "', targets=" + targets.size() + "}";
    }
}
