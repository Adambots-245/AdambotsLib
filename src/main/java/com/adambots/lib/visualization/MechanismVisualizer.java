package com.adambots.lib.visualization;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

/**
 * Manages a {@code Pose3d[]} array for AdvantageScope 3D component visualization.
 *
 * <p>AdvantageScope maps array index N to {@code model_N.glb} in the robot's asset folder.
 * Each mechanism updates its assigned index via a {@link Supplier}, and the visualizer
 * publishes the full array each cycle.
 *
 * <p><strong>Usage:</strong>
 * <pre>{@code
 * // Create visualizer with 3 component slots
 * var viz = new MechanismVisualizer("Robot/Components", 3);
 *
 * // Register mechanisms at their component indices
 * viz.register(0, () -> new Pose3d(
 *     new Translation3d(0, 0, elevator.getHeight().in(Meters)),
 *     new Rotation3d()
 * ));
 *
 * viz.register(1, () -> {
 *     double angleRad = arm.getAngle().in(Radians);
 *     return new Pose3d(Translation3d.kZero, new Rotation3d(0, angleRad, 0));
 * });
 *
 * // Call in subsystem periodic()
 * viz.update();
 * }</pre>
 */
public class MechanismVisualizer {

    private final Pose3d[] poses;
    private final List<ComponentEntry> entries = new ArrayList<>();
    private final StructArrayPublisher<Pose3d> publisher;

    /**
     * Creates a mechanism visualizer.
     *
     * @param ntKey NetworkTables key for the Pose3d[] (e.g., "Robot/Components")
     * @param numComponents number of 3D component slots
     */
    public MechanismVisualizer(String ntKey, int numComponents) {
        this.poses = new Pose3d[numComponents];
        for (int i = 0; i < numComponents; i++) {
            poses[i] = new Pose3d();
        }

        publisher = NetworkTableInstance.getDefault()
            .getStructArrayTopic(ntKey, Pose3d.struct)
            .publish();
    }

    /**
     * Registers a pose supplier at a specific component index.
     *
     * @param index the array index (maps to model_N.glb)
     * @param poseSupplier supplies the current Pose3d for this component
     */
    public void register(int index, Supplier<Pose3d> poseSupplier) {
        entries.add(new ComponentEntry(index, poseSupplier));
    }

    /**
     * Updates all registered poses and publishes to NetworkTables.
     * Call this from your subsystem's {@code periodic()} method.
     */
    public void update() {
        for (ComponentEntry entry : entries) {
            poses[entry.index] = entry.poseSupplier.get();
        }
        publisher.set(poses);
    }

    /**
     * Closes the NetworkTables publisher.
     */
    public void close() {
        publisher.close();
    }

    private record ComponentEntry(int index, Supplier<Pose3d> poseSupplier) {}
}
