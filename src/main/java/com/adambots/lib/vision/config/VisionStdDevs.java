// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.lib.vision.config;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

/**
 * Configuration record for vision measurement standard deviations.
 *
 * <p>Standard deviations represent the uncertainty in vision measurements.
 * Lower values indicate higher confidence/trust in the measurement.
 *
 * <p><strong>Values:</strong>
 * <ul>
 *   <li>{@code x} - X position uncertainty</li>
 *   <li>{@code y} - Y position uncertainty</li>
 *   <li>{@code theta} - Rotation uncertainty</li>
 * </ul>
 *
 * <p><strong>Usage Example:</strong>
 * <pre>{@code
 * // Create custom std devs
 * VisionStdDevs custom = new VisionStdDevs(Meters.of(0.3), Meters.of(0.3), Radians.of(0.2));
 *
 * // Use presets
 * VisionStdDevs single = VisionStdDevs.DEFAULT_SINGLE_TAG;
 * VisionStdDevs multi = VisionStdDevs.DEFAULT_MULTI_TAG;
 *
 * // Convert to matrix for use with pose estimator
 * Matrix<N3, N1> matrix = custom.toMatrix();
 * }</pre>
 *
 * @param x X position uncertainty
 * @param y Y position uncertainty
 * @param theta Rotation uncertainty
 */
public record VisionStdDevs(Distance x, Distance y, Angle theta) {

    /**
     * Default standard deviations for single AprilTag pose estimates.
     * <p>Conservative values with moderate trust in vision.
     */
    public static final VisionStdDevs DEFAULT_SINGLE_TAG =
        new VisionStdDevs(Meters.of(0.5), Meters.of(0.5), Radians.of(0.5));

    /**
     * Default standard deviations for multi-AprilTag pose estimates.
     * <p>Higher rotation uncertainty because multi-tag averaging can introduce rotation errors.
     */
    public static final VisionStdDevs DEFAULT_MULTI_TAG =
        new VisionStdDevs(Meters.of(0.5), Meters.of(0.5), Radians.of(1.0));

    /**
     * High confidence standard deviations for very reliable measurements.
     * <p>Use when camera is close to tags and seeing multiple tags clearly.
     */
    public static final VisionStdDevs HIGH_CONFIDENCE =
        new VisionStdDevs(Meters.of(0.1), Meters.of(0.1), Radians.of(0.1));

    /**
     * Low confidence standard deviations for less reliable measurements.
     * <p>Use for cameras that are far from tags or have poor visibility.
     */
    public static final VisionStdDevs LOW_CONFIDENCE =
        new VisionStdDevs(Meters.of(1.0), Meters.of(1.0), Radians.of(1.0));

    /**
     * Very low confidence standard deviations.
     * <p>Use when vision measurements should have minimal impact on pose estimation.
     */
    public static final VisionStdDevs VERY_LOW_CONFIDENCE =
        new VisionStdDevs(Meters.of(2.0), Meters.of(2.0), Radians.of(2.0));

    /**
     * Converts this standard deviation configuration to a WPILib Matrix.
     *
     * @return Matrix<N3, N1> suitable for use with pose estimators
     */
    public Matrix<N3, N1> toMatrix() {
        return VecBuilder.fill(x.in(Meters), y.in(Meters), theta.in(Radians));
    }

    /**
     * Creates a VisionStdDevs from a WPILib Matrix.
     *
     * @param matrix The matrix to convert (must be N3 x N1)
     * @return VisionStdDevs with values from the matrix
     */
    public static VisionStdDevs fromMatrix(Matrix<N3, N1> matrix) {
        return new VisionStdDevs(
            Meters.of(matrix.get(0, 0)),
            Meters.of(matrix.get(1, 0)),
            Radians.of(matrix.get(2, 0))
        );
    }

    /**
     * Creates a uniform VisionStdDevs where all components have the same value.
     * <p>The distance value in meters is also used as the theta value in radians.
     *
     * @param value The value to use for x, y (and theta in radians)
     * @return VisionStdDevs with uniform uncertainty
     */
    public static VisionStdDevs uniform(Distance value) {
        return new VisionStdDevs(value, value, Radians.of(value.in(Meters)));
    }

    /**
     * Scales this standard deviation by a factor.
     * <p>Useful for dynamic adjustment based on distance or confidence.
     *
     * @param factor Scale factor (>1 increases uncertainty, <1 decreases)
     * @return New VisionStdDevs with scaled values
     */
    public VisionStdDevs scaled(double factor) {
        return new VisionStdDevs(
            Meters.of(x.in(Meters) * factor),
            Meters.of(y.in(Meters) * factor),
            Radians.of(theta.in(Radians) * factor)
        );
    }
}
