package com.adambots.lib.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

/**
 * Functional interface for consuming vision pose measurements.
 *
 * <p>This decouples vision from any specific drivetrain implementation.
 * The typical usage is to pass a method reference from the drivetrain:
 * <pre>{@code
 * vision.updatePoseEstimation(
 *     swerveDrive::addVisionMeasurement,
 *     swerveDrive::getSimulationDriveTrainPose
 * );
 * }</pre>
 *
 * @see VisionSystem#updatePoseEstimation(VisionMeasurementConsumer, java.util.function.Supplier)
 */
@FunctionalInterface
public interface VisionMeasurementConsumer {
    /**
     * Accepts a vision pose measurement with timestamp and standard deviations.
     *
     * @param pose The estimated robot pose from vision
     * @param timestampSeconds The timestamp of the measurement in seconds
     * @param stdDevs The standard deviations of the measurement (3x1 matrix: x, y, theta)
     */
    void accept(Pose2d pose, double timestampSeconds, Matrix<N3, N1> stdDevs);
}
