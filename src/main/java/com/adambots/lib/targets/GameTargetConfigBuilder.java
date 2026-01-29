// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.lib.targets;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

import java.util.ArrayList;
import java.util.List;

/**
 * Fluent builder for creating GameTargetConfig programmatically.
 *
 * <p>This builder provides a code-based alternative to JSON configuration,
 * allowing teams to define game targets directly in Java with full type safety.
 *
 * <p><strong>Usage Example:</strong>
 * <pre>{@code
 * GameTargetConfig config = GameTargetConfigBuilder.create()
 *     .gameYear(2026)
 *     .gameName("REBUILT")
 *     .addTarget("tower-blue")
 *         .tagIds(15, 16)
 *         .offset(Meters.of(-0.5), Meters.of(0), Degrees.of(180))
 *         .tolerance(0.05, 2.0)
 *         .done()
 *     .addTarget("tower-red")
 *         .tagIds(31, 32)
 *         .offset(Meters.of(-0.5), Meters.of(0), Degrees.of(0))
 *         .tolerance(0.05, 2.0)
 *         .done()
 *     .addTarget("hub-blue")
 *         .tagIds(1, 2, 3)
 *         .offset(Meters.of(-0.3), Meters.of(0), Degrees.of(90))
 *         .done()
 *     .build();
 * }</pre>
 *
 * @see GameTargetConfig
 * @see GameTarget
 */
public class GameTargetConfigBuilder {

    private int gameYear = 2026;
    private String gameName = "Unknown";
    private final List<GameTarget> targets = new ArrayList<>();

    private GameTargetConfigBuilder() {}

    /**
     * Creates a new builder instance.
     *
     * @return New builder
     */
    public static GameTargetConfigBuilder create() {
        return new GameTargetConfigBuilder();
    }

    /**
     * Sets the FRC game year.
     *
     * @param year Game year (e.g., 2026)
     * @return This builder for chaining
     */
    public GameTargetConfigBuilder gameYear(int year) {
        this.gameYear = year;
        return this;
    }

    /**
     * Sets the FRC game name.
     *
     * @param name Game name (e.g., "REBUILT")
     * @return This builder for chaining
     */
    public GameTargetConfigBuilder gameName(String name) {
        this.gameName = name;
        return this;
    }

    /**
     * Starts building a new target with the given name.
     *
     * @param name Target name (e.g., "tower-blue")
     * @return TargetBuilder for configuring the target
     */
    public TargetBuilder addTarget(String name) {
        return new TargetBuilder(this, name);
    }

    /**
     * Adds a pre-built GameTarget to the configuration.
     *
     * @param target GameTarget to add
     * @return This builder for chaining
     */
    public GameTargetConfigBuilder withTarget(GameTarget target) {
        targets.add(target);
        return this;
    }

    /**
     * Builds the GameTargetConfig.
     *
     * @return Configured GameTargetConfig
     */
    public GameTargetConfig build() {
        return new GameTargetConfig(gameYear, gameName, targets);
    }

    /**
     * Nested builder for configuring individual targets.
     */
    public static class TargetBuilder {
        private final GameTargetConfigBuilder parent;
        private final String name;
        private int[] tagIds = new int[0];
        private Transform2d offset = new Transform2d();
        private GameTarget.Tolerance tolerance = null;

        private TargetBuilder(GameTargetConfigBuilder parent, String name) {
            this.parent = parent;
            this.name = name;
        }

        /**
         * Sets the AprilTag IDs for this target.
         *
         * @param ids Tag IDs
         * @return This builder for chaining
         */
        public TargetBuilder tagIds(int... ids) {
            this.tagIds = ids;
            return this;
        }

        /**
         * Sets the offset using WPILib units (Distance and Angle).
         *
         * <p>The offset is relative to the primary tag's pose:
         * <ul>
         *   <li>Positive X is forward from the tag (in the tag's facing direction)</li>
         *   <li>Positive Y is left from the tag's perspective</li>
         *   <li>Rotation is the desired robot heading relative to the tag's rotation</li>
         * </ul>
         *
         * @param x X offset (positive = forward from tag)
         * @param y Y offset (positive = left from tag)
         * @param rotation Target heading relative to tag rotation
         * @return This builder for chaining
         */
        public TargetBuilder offset(Distance x, Distance y, Angle rotation) {
            this.offset = new Transform2d(
                new Translation2d(x.in(Meters), y.in(Meters)),
                Rotation2d.fromDegrees(rotation.in(Degrees))
            );
            return this;
        }

        /**
         * Sets the offset using raw values (meters and degrees).
         *
         * @param xMeters X offset in meters
         * @param yMeters Y offset in meters
         * @param rotationDegrees Rotation in degrees
         * @return This builder for chaining
         */
        public TargetBuilder offsetMeters(double xMeters, double yMeters, double rotationDegrees) {
            this.offset = new Transform2d(
                new Translation2d(xMeters, yMeters),
                Rotation2d.fromDegrees(rotationDegrees)
            );
            return this;
        }

        /**
         * Sets only the translation offset (rotation defaults to 0).
         *
         * @param x X offset
         * @param y Y offset
         * @return This builder for chaining
         */
        public TargetBuilder offset(Distance x, Distance y) {
            return offset(x, y, Degrees.of(0));
        }

        /**
         * Sets only the rotation offset (translation defaults to 0,0).
         *
         * @param rotation Target heading relative to tag rotation
         * @return This builder for chaining
         */
        public TargetBuilder rotation(Angle rotation) {
            return offset(Meters.of(0), Meters.of(0), rotation);
        }

        /**
         * Sets only the rotation offset using degrees.
         *
         * @param degrees Rotation in degrees
         * @return This builder for chaining
         */
        public TargetBuilder rotationDegrees(double degrees) {
            return offset(Meters.of(0), Meters.of(0), Degrees.of(degrees));
        }

        /**
         * Sets the tolerance values for alignment checking.
         *
         * @param positionMeters Position tolerance in meters
         * @param rotationDegrees Rotation tolerance in degrees
         * @return This builder for chaining
         */
        public TargetBuilder tolerance(double positionMeters, double rotationDegrees) {
            this.tolerance = new GameTarget.Tolerance(positionMeters, rotationDegrees);
            return this;
        }

        /**
         * Sets the tolerance using WPILib units.
         *
         * @param position Position tolerance
         * @param rotation Rotation tolerance
         * @return This builder for chaining
         */
        public TargetBuilder tolerance(Distance position, Angle rotation) {
            this.tolerance = new GameTarget.Tolerance(position.in(Meters), rotation.in(Degrees));
            return this;
        }

        /**
         * Completes this target and returns to the parent builder.
         *
         * @return Parent builder for chaining
         */
        public GameTargetConfigBuilder done() {
            GameTarget target = new GameTarget(name, tagIds, offset, tolerance);
            parent.targets.add(target);
            return parent;
        }
    }
}
