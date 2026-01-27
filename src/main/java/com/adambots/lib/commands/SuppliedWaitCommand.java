// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.lib.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * A wait command that accepts a dynamic duration supplier.
 *
 * <p>Unlike WPILib's standard {@link edu.wpi.first.wpilibj2.command.WaitCommand},
 * which requires a fixed duration at construction time, this command evaluates
 * the duration when the command is initialized. This allows wait durations to
 * be determined by tunable values, sensor readings, or other dynamic sources.
 *
 * <p>Adapted from FRC Team 6328 Mechanical Advantage.
 * Licensed under MIT License - Copyright (c) 2025-2026 Littleton Robotics.
 *
 * @see <a href="https://github.com/Mechanical-Advantage/RobotCode2026Public">Original Source</a>
 *
 * <p><strong>Usage Examples:</strong>
 * <pre>{@code
 * // Wait for a tunable duration
 * TunableDouble waitTime = new TunableDouble("AutoWait", 2.0);
 * Command autoSequence = Commands.sequence(
 *     driveForward(),
 *     new SuppliedWaitCommand(waitTime::get),  // Duration determined at runtime
 *     intake()
 * );
 *
 * // Wait based on sensor distance
 * Command dynamicWait = new SuppliedWaitCommand(
 *     () -> distanceSensor.getRange() / 10.0  // Longer wait when further away
 * );
 *
 * // Wait based on game state
 * Command smartWait = new SuppliedWaitCommand(
 *     () -> DriverStation.isTeleop() ? 0.5 : 1.5
 * );
 * }</pre>
 *
 * <p><strong>When to Use:</strong>
 * <ul>
 *   <li>Autonomous routines with tunable timing</li>
 *   <li>Delays that depend on sensor values</li>
 *   <li>Conditional wait durations based on game state</li>
 *   <li>Subsystem sequences with variable timing</li>
 * </ul>
 *
 * @see edu.wpi.first.wpilibj2.command.WaitCommand
 */
public class SuppliedWaitCommand extends Command {

    /** Supplier providing the wait duration in seconds. */
    private final DoubleSupplier durationSupplier;

    /** Timer for tracking elapsed time. */
    private final Timer timer = new Timer();

    /** The duration for the current execution, captured at initialize(). */
    private double duration;

    /**
     * Creates a new SuppliedWaitCommand.
     *
     * <p>The duration supplier is evaluated when the command is initialized,
     * not when it is constructed. This allows the wait duration to be
     * determined dynamically based on runtime conditions.
     *
     * <p><strong>Example:</strong>
     * <pre>{@code
     * // Duration from NetworkTables tunable
     * new SuppliedWaitCommand(() -> SmartDashboard.getNumber("WaitTime", 1.0));
     *
     * // Duration from subsystem state
     * new SuppliedWaitCommand(() -> shooter.isSpunUp() ? 0.1 : 0.5);
     * }</pre>
     *
     * @param durationSupplier A supplier that provides the wait duration in seconds.
     *                         Evaluated once when the command initializes.
     */
    public SuppliedWaitCommand(DoubleSupplier durationSupplier) {
        this.durationSupplier = durationSupplier;
    }

    @Override
    public void initialize() {
        duration = durationSupplier.getAsDouble();
        timer.restart();
    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(duration);
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }

    /**
     * Returns the duration that was captured when the command was last initialized.
     *
     * <p>Useful for debugging or logging the actual wait duration used.
     *
     * @return The wait duration in seconds, or 0.0 if never initialized
     */
    public double getDuration() {
        return duration;
    }

    /**
     * Returns the elapsed time since the command started.
     *
     * <p>Useful for progress tracking or debugging.
     *
     * @return Elapsed time in seconds, or 0.0 if not running
     */
    public double getElapsedTime() {
        return timer.get();
    }
}
