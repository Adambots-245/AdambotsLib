// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.lib.vision.config;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

/**
 * Configuration record for vision measurement standard deviations.
 *
 * <p>Standard deviations represent the uncertainty in vision measurements.
 * Lower values indicate higher confidence/trust in the measurement.
 *
 * <p><strong>Values:</strong>
 * <ul>
 *   <li>{@code x} - X position uncertainty in meters</li>
 *   <li>{@code y} - Y position uncertainty in meters</li>
 *   <li>{@code theta} - Rotation uncertainty in radians</li>
 * </ul>
 *
 * <p><strong>Usage Example:</strong>
 * <pre>{@code
 * // Create custom std devs
 * VisionStdDevs custom = new VisionStdDevs(0.3, 0.3, 0.2);
 *
 * // Use presets
 * VisionStdDevs single = VisionStdDevs.DEFAULT_SINGLE_TAG;
 * VisionStdDevs multi = VisionStdDevs.DEFAULT_MULTI_TAG;
 *
 * // Convert to matrix for use with pose estimator
 * Matrix<N3, N1> matrix = custom.toMatrix();
 * }</pre>
 *
 * @param x X position uncertainty in meters
 * @param y Y position uncertainty in meters
 * @param theta Rotation uncertainty in radians
 */
public record VisionStdDevs(double x, double y, double theta) {

    /**
     * Default standard deviations for single AprilTag pose estimates.
     * <p>Conservative values with moderate trust in vision.
     */
    public static final VisionStdDevs DEFAULT_SINGLE_TAG = new VisionStdDevs(0.5, 0.5, 0.5);

    /**
     * Default standard deviations for multi-AprilTag pose estimates.
     * <p>Higher rotation uncertainty because multi-tag averaging can introduce rotation errors.
     */
    public static final VisionStdDevs DEFAULT_MULTI_TAG = new VisionStdDevs(0.5, 0.5, 1.0);

    /**
     * High confidence standard deviations for very reliable measurements.
     * <p>Use when camera is close to tags and seeing multiple tags clearly.
     */
    public static final VisionStdDevs HIGH_CONFIDENCE = new VisionStdDevs(0.1, 0.1, 0.1);

    /**
     * Low confidence standard deviations for less reliable measurements.
     * <p>Use for cameras that are far from tags or have poor visibility.
     */
    public static final VisionStdDevs LOW_CONFIDENCE = new VisionStdDevs(1.0, 1.0, 1.0);

    /**
     * Very low confidence standard deviations.
     * <p>Use when vision measurements should have minimal impact on pose estimation.
     */
    public static final VisionStdDevs VERY_LOW_CONFIDENCE = new VisionStdDevs(2.0, 2.0, 2.0);

    /**
     * Converts this standard deviation configuration to a WPILib Matrix.
     *
     * @return Matrix<N3, N1> suitable for use with pose estimators
     */
    public Matrix<N3, N1> toMatrix() {
        return VecBuilder.fill(x, y, theta);
    }

    /**
     * Creates a VisionStdDevs from a WPILib Matrix.
     *
     * @param matrix The matrix to convert (must be N3 x N1)
     * @return VisionStdDevs with values from the matrix
     */
    public static VisionStdDevs fromMatrix(Matrix<N3, N1> matrix) {
        return new VisionStdDevs(matrix.get(0, 0), matrix.get(1, 0), matrix.get(2, 0));
    }

    /**
     * Creates a uniform VisionStdDevs where all components have the same value.
     *
     * @param value The value to use for x, y, and theta
     * @return VisionStdDevs with uniform uncertainty
     */
    public static VisionStdDevs uniform(double value) {
        return new VisionStdDevs(value, value, value);
    }

    /**
     * Scales this standard deviation by a factor.
     * <p>Useful for dynamic adjustment based on distance or confidence.
     *
     * @param factor Scale factor (>1 increases uncertainty, <1 decreases)
     * @return New VisionStdDevs with scaled values
     */
    public VisionStdDevs scaled(double factor) {
        return new VisionStdDevs(x * factor, y * factor, theta * factor);
    }
}
