// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.lib.utils;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import java.util.HashMap;
import java.util.Map;

/**
 * Performance timing utility for profiling code execution.
 *
 * <p>LoggedTracer enables measurement of time between execution points in robot code.
 * Each measurement is logged to NetworkTables for analysis in AdvantageScope or
 * SmartDashboard.
 *
 * <p>Adapted from FRC Team 6328 Mechanical Advantage.
 * Licensed under MIT License - Copyright (c) 2025-2026 Littleton Robotics.
 *
 * @see <a href="https://github.com/Mechanical-Advantage/RobotCode2026Public">Original Source</a>
 *
 * <p><strong>Usage Example:</strong>
 * <pre>{@code
 * public class Robot extends TimedRobot {
 *     public void robotPeriodic() {
 *         LoggedTracer.reset();
 *
 *         runDrivetrain();
 *         LoggedTracer.record("Drivetrain");
 *
 *         runVision();
 *         LoggedTracer.record("Vision");
 *
 *         runArm();
 *         LoggedTracer.record("Arm");
 *     }
 * }
 * // In NetworkTables/Logs:
 * // LoggedTracer/DrivetrainMS = 2.3
 * // LoggedTracer/VisionMS = 5.1
 * // LoggedTracer/ArmMS = 1.2
 * }</pre>
 *
 * <p><strong>Use Cases:</strong>
 * <ul>
 *   <li>Identify slow subsystems causing loop overruns</li>
 *   <li>Profile vision processing latency</li>
 *   <li>Track pose estimation calculation time</li>
 *   <li>Debug periodic loop timing issues</li>
 * </ul>
 */
public class LoggedTracer {

    /** NetworkTable for publishing tracer outputs. */
    private static final NetworkTable table =
        NetworkTableInstance.getDefault().getTable("LoggedTracer");

    /** Cache of publishers to avoid creating new ones each cycle. */
    private static final Map<String, DoublePublisher> publishers = new HashMap<>();

    /** Start time for the current measurement period. */
    private static double startTime = -1.0;

    /**
     * Private constructor to prevent instantiation.
     *
     * @throws UnsupportedOperationException always, as this is a utility class
     */
    private LoggedTracer() {
        throw new UnsupportedOperationException("Not meant to be instantiated. Utility class");
    }

    /**
     * Resets the tracer start time.
     *
     * <p>Call this at the beginning of each periodic loop to establish
     * a baseline for timing measurements.
     *
     * <p><strong>Example:</strong>
     * <pre>{@code
     * public void robotPeriodic() {
     *     LoggedTracer.reset();
     *     // ... rest of periodic code with record() calls
     * }
     * }</pre>
     */
    public static void reset() {
        startTime = Timer.getFPGATimestamp();
    }

    /**
     * Records the elapsed time since the last reset/record call.
     *
     * <p>Logs the duration in milliseconds to NetworkTables under
     * "LoggedTracer/{epochName}MS". After recording, the start time
     * is automatically reset for the next measurement.
     *
     * <p><strong>Example:</strong>
     * <pre>{@code
     * LoggedTracer.reset();
     * swerve.periodic();
     * LoggedTracer.record("Swerve");  // Logs time for swerve.periodic()
     *
     * vision.periodic();
     * LoggedTracer.record("Vision");  // Logs time for vision.periodic()
     * }</pre>
     *
     * @param epochName Name for this timing checkpoint (used in log key)
     */
    public static void record(String epochName) {
        double now = Timer.getFPGATimestamp();
        double elapsedMs = (now - startTime) * 1000.0;

        // Get or create publisher for this epoch
        DoublePublisher publisher = publishers.computeIfAbsent(
            epochName,
            name -> table.getDoubleTopic(name + "MS").publish()
        );

        publisher.set(elapsedMs);
        startTime = now;
    }

    /**
     * Records elapsed time with a custom NetworkTable path prefix.
     *
     * <p>Useful for organizing measurements into categories.
     *
     * <p><strong>Example:</strong>
     * <pre>{@code
     * LoggedTracer.reset();
     * drive.calculateOdometry();
     * LoggedTracer.record("Odometry", "Drive");  // Logs to Drive/OdometryMS
     * }</pre>
     *
     * @param epochName Name for this timing checkpoint
     * @param category Category prefix for organization
     */
    public static void record(String epochName, String category) {
        record(category + "/" + epochName);
    }
}
