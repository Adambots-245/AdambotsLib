// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.lib.utils;

import java.util.HashMap;
import java.util.Map;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.LongSupplier;
import java.util.function.Supplier;

import com.adambots.lib.Constants;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Dashboard utility for adding values to Shuffleboard with automatic updates.
 *
 * <p>Dash provides a simple, clean API for adding telemetry data to Shuffleboard
 * without dealing with NetworkTables or Shuffleboard API complexity. All values
 * added with suppliers auto-update every robot loop.
 *
 * <p><strong>Basic Usage:</strong>
 * <pre>{@code
 * // In subsystem constructor or Robot.robotInit()
 * Dash.add("Robot Speed", () -> swerve.getSpeed());
 * Dash.add("Intake Deployed", () -> intake.isDeployed());
 * Dash.add("Match Time", () -> DriverStation.getMatchTime());
 * }</pre>
 *
 * <p><strong>Custom Tabs:</strong>
 * <pre>{@code
 * // Use custom tab for organization
 * Dash.useTab("Swerve");
 * Dash.add("FL Velocity", () -> frontLeft.getVelocity());
 * Dash.add("FR Velocity", () -> frontRight.getVelocity());
 *
 * // Switch back to default
 * Dash.useDefaultTab();
 * }</pre>
 *
 * <p><strong>Tunable Values (PID, etc.):</strong>
 * <pre>{@code
 * // Add tunable number - returns entry you can read
 * GenericEntry kP = Dash.addTunable("kP", 0.5);
 * GenericEntry kD = Dash.addTunable("kD", 0.1);
 *
 * // Read in periodic()
 * public void periodic() {
 *     double p = kP.getDouble(0.5);
 *     double d = kD.getDouble(0.1);
 *     controller.setPID(p, 0, d);
 * }
 * }</pre>
 *
 * <p><strong>Commands:</strong>
 * <pre>{@code
 * // Add command buttons to Shuffleboard
 * Dash.addCommand("Reset Gyro", swerve.resetGyroCommand());
 * Dash.addCommand("Deploy Intake", intake.deployCommand());
 * }</pre>
 *
 * @see edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
 * @see edu.wpi.first.networktables.NetworkTable
 */
public class Dash {
    /** Current active Shuffleboard tab. */
    private static ShuffleboardTab currentTab;

    /** Default tab name from Constants. */
    private static final String DEFAULT_TAB_NAME = Constants.kDefaultShuffleboardTab;

    /** Cache of tabs by name for quick access. */
    private static final Map<String, ShuffleboardTab> tabCache = new HashMap<>();

    // Static initializer to set default tab
    static {
        currentTab = Shuffleboard.getTab(DEFAULT_TAB_NAME);
        tabCache.put(DEFAULT_TAB_NAME, currentTab);
    }

    /**
     * Private constructor to prevent instantiation.
     *
     * @throws UnsupportedOperationException always, as this is a utility class
     */
    private Dash() {
        throw new UnsupportedOperationException("Not meant to be instantiated. Utility class");
    }

    // ======================== TAB MANAGEMENT ========================

    /**
     * Switch to a different Shuffleboard tab.
     *
     * <p>All subsequent {@code add()} calls will add to this tab until changed.
     *
     * <p><strong>Example:</strong>
     * <pre>{@code
     * Dash.useTab("Swerve");
     * Dash.add("FL Angle", () -> frontLeft.getAngle());
     * Dash.add("FR Angle", () -> frontRight.getAngle());
     * }</pre>
     *
     * @param tabName Name of the tab to use
     */
    public static void useTab(String tabName) {
        if (!tabCache.containsKey(tabName)) {
            tabCache.put(tabName, Shuffleboard.getTab(tabName));
        }
        currentTab = tabCache.get(tabName);
    }

    /**
     * Switch back to the default tab defined in Constants.
     *
     * <p><strong>Example:</strong>
     * <pre>{@code
     * Dash.useTab("Swerve");
     * // ... add swerve values ...
     * Dash.useDefaultTab();  // Back to "debug" tab
     * }</pre>
     */
    public static void useDefaultTab() {
        currentTab = tabCache.get(DEFAULT_TAB_NAME);
    }

    /**
     * Get the currently active tab.
     *
     * @return Current ShuffleboardTab
     */
    public static ShuffleboardTab getCurrentTab() {
        return currentTab;
    }

    // ======================== ADD VALUES (AUTO-UPDATE) ========================

    /**
     * Add a double value that auto-updates every robot loop.
     *
     * <p><strong>Example:</strong>
     * <pre>{@code
     * Dash.add("Robot Speed", () -> swerve.getSpeed());
     * Dash.add("Gyro Angle", () -> gyro.getAngle());
     * }</pre>
     *
     * @param name Display name on Shuffleboard
     * @param supplier Supplier that provides the current value
     */
    public static void add(String name, DoubleSupplier supplier) {
        currentTab.addDouble(name, supplier);
    }

    /**
     * Add a long/integer value that auto-updates every robot loop.
     *
     * <p><strong>Example:</strong>
     * <pre>{@code
     * Dash.add("Loop Count", () -> loopCounter);
     * Dash.add("Match Time", () -> (long) DriverStation.getMatchTime());
     * }</pre>
     *
     * @param name Display name on Shuffleboard
     * @param supplier Supplier that provides the current value
     */
    public static void add(String name, LongSupplier supplier) {
        currentTab.addInteger(name, supplier);
    }

    /**
     * Add a boolean value that auto-updates every robot loop.
     *
     * <p><strong>Example:</strong>
     * <pre>{@code
     * Dash.add("Intake Deployed", () -> intake.isDeployed());
     * Dash.add("At Target", () -> elevator.isAtTarget());
     * }</pre>
     *
     * @param name Display name on Shuffleboard
     * @param supplier Supplier that provides the current value
     */
    public static void add(String name, BooleanSupplier supplier) {
        currentTab.addBoolean(name, supplier);
    }

    /**
     * Add a String value that auto-updates every robot loop.
     *
     * <p><strong>Example:</strong>
     * <pre>{@code
     * Dash.add("Current State", () -> stateMachine.getCurrentState().toString());
     * Dash.add("Alliance", () -> DriverStation.getAlliance().toString());
     * }</pre>
     *
     * @param name Display name on Shuffleboard
     * @param supplier Supplier that provides the current string
     */
    public static void add(String name, Supplier<String> supplier) {
        currentTab.addString(name, supplier);
    }

    /**
     * Add a double array that auto-updates every robot loop.
     *
     * <p><strong>Example:</strong>
     * <pre>{@code
     * Dash.addDoubleArray("Module States", () -> new double[] {
     *     module1.getVelocity(),
     *     module2.getVelocity(),
     *     module3.getVelocity(),
     *     module4.getVelocity()
     * });
     * }</pre>
     *
     * @param name Display name on Shuffleboard
     * @param supplier Supplier that provides the current array
     */
    public static void addDoubleArray(String name, Supplier<double[]> supplier) {
        currentTab.addDoubleArray(name, supplier);
    }

    /**
     * Add a String array that auto-updates every robot loop.
     *
     * <p><strong>Example:</strong>
     * <pre>{@code
     * Dash.add("Subsystem States", () -> new String[] {
     *     intake.getState().toString(),
     *     shooter.getState().toString(),
     *     climber.getState().toString()
     * });
     * }</pre>
     *
     * @param name Display name on Shuffleboard
     * @param supplier Supplier that provides the current array
     */
    public static void addStringArray(String name, Supplier<String[]> supplier) {
        currentTab.addStringArray(name, supplier);
    }

    // ======================== TUNABLE VALUES ========================

    /**
     * Add a tunable number (editable on Shuffleboard).
     *
     * <p>Use this for PID constants, speed limits, or other values you want to
     * tune during testing. The returned {@link GenericEntry} can be read to get
     * the current value.
     *
     * <p><strong>Example:</strong>
     * <pre>{@code
     * // In subsystem constructor
     * private GenericEntry kP = Dash.addTunable("Arm kP", 0.5);
     * private GenericEntry kD = Dash.addTunable("Arm kD", 0.1);
     *
     * // In periodic()
     * public void periodic() {
     *     double p = kP.getDouble(0.5);
     *     double d = kD.getDouble(0.1);
     *     armController.setPID(p, 0, d);
     * }
     * }</pre>
     *
     * @param name Display name on Shuffleboard
     * @param defaultValue Initial/default value
     * @return GenericEntry that can be read to get the current value
     */
    public static GenericEntry addTunable(String name, double defaultValue) {
        return currentTab.add(name, defaultValue)
            .withWidget(BuiltInWidgets.kTextView)
            .getEntry();
    }

    /**
     * Add a tunable boolean (checkbox on Shuffleboard).
     *
     * <p><strong>Example:</strong>
     * <pre>{@code
     * private GenericEntry enableVision = Dash.addTunable("Enable Vision", true);
     *
     * public void periodic() {
     *     if (enableVision.getBoolean(true)) {
     *         updateVisionPose();
     *     }
     * }
     * }</pre>
     *
     * @param name Display name on Shuffleboard
     * @param defaultValue Initial/default value
     * @return GenericEntry that can be read to get the current value
     */
    public static GenericEntry addTunable(String name, boolean defaultValue) {
        return currentTab.add(name, defaultValue)
            .withWidget(BuiltInWidgets.kToggleButton)
            .getEntry();
    }

    /**
     * Add a tunable String (text field on Shuffleboard).
     *
     * <p><strong>Example:</strong>
     * <pre>{@code
     * private GenericEntry autoPath = Dash.addTunable("Auto Path", "4PieceAuto");
     *
     * public Command getAutonomousCommand() {
     *     String pathName = autoPath.getString("4PieceAuto");
     *     return swerve.getAutonomousCommand(pathName);
     * }
     * }</pre>
     *
     * @param name Display name on Shuffleboard
     * @param defaultValue Initial/default value
     * @return GenericEntry that can be read to get the current value
     */
    public static GenericEntry addTunable(String name, String defaultValue) {
        return currentTab.add(name, defaultValue)
            .withWidget(BuiltInWidgets.kTextView)
            .getEntry();
    }

    // ======================== COMMANDS ========================

    /**
     * Add a command button to Shuffleboard.
     *
     * <p>Creates a button that runs the command when pressed. Useful for testing
     * and debugging subsystems.
     *
     * <p><strong>Example:</strong>
     * <pre>{@code
     * // In RobotContainer or subsystem
     * Dash.addCommand("Reset Gyro", swerve.resetGyroCommand());
     * Dash.addCommand("Deploy Intake", intake.deployCommand());
     * Dash.addCommand("Spin Up Shooter", shooter.spinUpCommand());
     * }</pre>
     *
     * @param name Button label on Shuffleboard
     * @param command Command to run when button is pressed
     */
    public static void addCommand(String name, Command command) {
        currentTab.add(name, command);
    }

    // ======================== UTILITY METHODS ========================

    /**
     * Remove all widgets from the current tab.
     *
     * <p><strong>Warning:</strong> This clears ALL widgets from the current tab,
     * including those not created by Dash.
     *
     * <p><strong>Example:</strong>
     * <pre>{@code
     * // Clear debug tab when switching to competition mode
     * Dash.useTab("Debug");
     * Dash.clearTab();
     * Dash.useDefaultTab();
     * }</pre>
     */
    public static void clearTab() {
        // Note: Shuffleboard doesn't provide a clear() method,
        // so we need to recreate the tab
        String tabName = getCurrentTabName();
        tabCache.remove(tabName);
        Shuffleboard.selectTab(tabName);
        currentTab = Shuffleboard.getTab(tabName);
        tabCache.put(tabName, currentTab);
    }

    /**
     * Get the name of the current tab.
     *
     * @return Current tab name
     */
    private static String getCurrentTabName() {
        for (Map.Entry<String, ShuffleboardTab> entry : tabCache.entrySet()) {
            if (entry.getValue() == currentTab) {
                return entry.getKey();
            }
        }
        return DEFAULT_TAB_NAME;
    }

    /**
     * Select (focus) the current tab in the Shuffleboard UI.
     *
     * <p>This brings the tab to the front in the Shuffleboard display.
     *
     * <p><strong>Example:</strong>
     * <pre>{@code
     * // During autonomous, show autonomous tab
     * Dash.useTab("Autonomous");
     * Dash.selectCurrentTab();
     * }</pre>
     */
    public static void selectCurrentTab() {
        Shuffleboard.selectTab(getCurrentTabName());
    }
}
