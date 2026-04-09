// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.lib.utils;

import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.LongSupplier;
import java.util.function.Supplier;

import com.adambots.lib.Constants;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
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

    /** Widget metadata for Elastic layout export. */
    private static final List<WidgetInfo> widgets = new ArrayList<>();

    /** Widget metadata record for tracking what was added to the dashboard. */
    private record WidgetInfo(
        String tabName, String name, String dataType,
        boolean tunable, int column, int row
    ) {}

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

    /** Records widget metadata for Elastic export. */
    private static void trackWidget(String name, String dataType, boolean tunable, int column, int row) {
        widgets.add(new WidgetInfo(getCurrentTabName(), name, dataType, tunable, column, row));
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
        trackWidget(name, "double", false, -1, -1);
    }

    /**
     * Add a double value at a specific grid position.
     *
     * @param name Display name on Shuffleboard
     * @param supplier Supplier that provides the current value
     * @param column Column position (0-indexed from left)
     * @param row Row position (0-indexed from top)
     */
    public static void add(String name, DoubleSupplier supplier, int column, int row) {
        currentTab.addDouble(name, supplier).withPosition(column, row);
        trackWidget(name, "double", false, column, row);
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
        trackWidget(name, "integer", false, -1, -1);
    }

    /**
     * Add a long/integer value at a specific grid position.
     *
     * @param name Display name on Shuffleboard
     * @param supplier Supplier that provides the current value
     * @param column Column position (0-indexed from left)
     * @param row Row position (0-indexed from top)
     */
    public static void add(String name, LongSupplier supplier, int column, int row) {
        currentTab.addInteger(name, supplier).withPosition(column, row);
        trackWidget(name, "integer", false, column, row);
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
        trackWidget(name, "boolean", false, -1, -1);
    }

    /**
     * Add a boolean value at a specific grid position.
     *
     * @param name Display name on Shuffleboard
     * @param supplier Supplier that provides the current value
     * @param column Column position (0-indexed from left)
     * @param row Row position (0-indexed from top)
     */
    public static void add(String name, BooleanSupplier supplier, int column, int row) {
        currentTab.addBoolean(name, supplier).withPosition(column, row);
        trackWidget(name, "boolean", false, column, row);
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
        trackWidget(name, "string", false, -1, -1);
    }

    /**
     * Add a String value at a specific grid position.
     *
     * @param name Display name on Shuffleboard
     * @param supplier Supplier that provides the current value
     * @param column Column position (0-indexed from left)
     * @param row Row position (0-indexed from top)
     */
    public static void add(String name, Supplier<String> supplier, int column, int row) {
        currentTab.addString(name, supplier).withPosition(column, row);
        trackWidget(name, "string", false, column, row);
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
        trackWidget(name, "doubleArray", false, -1, -1);
    }

    /**
     * Add a double array at a specific grid position.
     *
     * @param name Display name on Shuffleboard
     * @param supplier Supplier that provides the current array
     * @param column Column position (0-indexed from left)
     * @param row Row position (0-indexed from top)
     */
    public static void addDoubleArray(String name, Supplier<double[]> supplier, int column, int row) {
        currentTab.addDoubleArray(name, supplier).withPosition(column, row);
        trackWidget(name, "doubleArray", false, column, row);
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
        trackWidget(name, "stringArray", false, -1, -1);
    }

    /**
     * Add a String array at a specific grid position.
     *
     * @param name Display name on Shuffleboard
     * @param supplier Supplier that provides the current array
     * @param column Column position (0-indexed from left)
     * @param row Row position (0-indexed from top)
     */
    public static void addStringArray(String name, Supplier<String[]> supplier, int column, int row) {
        currentTab.addStringArray(name, supplier).withPosition(column, row);
        trackWidget(name, "stringArray", false, column, row);
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
        GenericEntry entry = currentTab.add(name, defaultValue)
            .withWidget(BuiltInWidgets.kTextView)
            .getEntry();
        trackWidget(name, "double", true, -1, -1);
        return entry;
    }

    /**
     * Add a tunable number at a specific grid position.
     *
     * @param name Display name on Shuffleboard
     * @param defaultValue Initial/default value
     * @param column Column position (0-indexed from left)
     * @param row Row position (0-indexed from top)
     * @return GenericEntry that can be read to get the current value
     */
    public static GenericEntry addTunable(String name, double defaultValue, int column, int row) {
        GenericEntry entry = currentTab.add(name, defaultValue)
            .withWidget(BuiltInWidgets.kTextView)
            .withPosition(column, row)
            .getEntry();
        trackWidget(name, "double", true, column, row);
        return entry;
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
        GenericEntry entry = currentTab.add(name, defaultValue)
            .withWidget(BuiltInWidgets.kToggleButton)
            .getEntry();
        trackWidget(name, "boolean", true, -1, -1);
        return entry;
    }

    /**
     * Add a tunable boolean at a specific grid position.
     *
     * @param name Display name on Shuffleboard
     * @param defaultValue Initial/default value
     * @param column Column position (0-indexed from left)
     * @param row Row position (0-indexed from top)
     * @return GenericEntry that can be read to get the current value
     */
    public static GenericEntry addTunable(String name, boolean defaultValue, int column, int row) {
        GenericEntry entry = currentTab.add(name, defaultValue)
            .withWidget(BuiltInWidgets.kToggleButton)
            .withPosition(column, row)
            .getEntry();
        trackWidget(name, "boolean", true, column, row);
        return entry;
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
        GenericEntry entry = currentTab.add(name, defaultValue)
            .withWidget(BuiltInWidgets.kTextView)
            .getEntry();
        trackWidget(name, "string", true, -1, -1);
        return entry;
    }

    /**
     * Add a tunable String at a specific grid position.
     *
     * @param name Display name on Shuffleboard
     * @param defaultValue Initial/default value
     * @param column Column position (0-indexed from left)
     * @param row Row position (0-indexed from top)
     * @return GenericEntry that can be read to get the current value
     */
    public static GenericEntry addTunable(String name, String defaultValue, int column, int row) {
        GenericEntry entry = currentTab.add(name, defaultValue)
            .withWidget(BuiltInWidgets.kTextView)
            .withPosition(column, row)
            .getEntry();
        trackWidget(name, "string", true, column, row);
        return entry;
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
        trackWidget(name, "command", false, -1, -1);
    }

    /**
     * Add a command button at a specific grid position.
     *
     * @param name Button label on Shuffleboard
     * @param command Command to run when button is pressed
     * @param column Column position (0-indexed from left)
     * @param row Row position (0-indexed from top)
     */
    public static void addCommand(String name, Command command, int column, int row) {
        currentTab.add(name, command).withPosition(column, row);
        trackWidget(name, "command", false, column, row);
    }

    // ======================== ELASTIC DASHBOARD EXPORT ========================

    /** Elastic Dashboard grid cell size in pixels (default from Elastic source). */
    private static final int ELASTIC_GRID_SIZE = 128;

    /** Default widget width in grid cells. */
    private static final int DEFAULT_WIDGET_CELLS_W = 2;

    /** Default widget height in grid cells. */
    private static final int DEFAULT_WIDGET_CELLS_H = 1;

    /** Grid columns used for auto-layout wrapping. */
    private static final int AUTO_LAYOUT_COLS = 8;

    /**
     * Exports the current dashboard layout as an Elastic Dashboard JSON file.
     *
     * <p>Call this after all {@code Dash.add()} calls are complete (e.g., at the end
     * of {@code robotInit()}). The file is written to the robot's deploy directory.
     *
     * <p><strong>Loading in Elastic:</strong> Elastic can load the exported file via
     * File → Open Layout (after copying it to your laptop), or via remote layout
     * download if enabled in Elastic settings.
     *
     * <p><strong>Usage:</strong>
     * <pre>{@code
     * // At end of robotInit() or RobotContainer constructor
     * Dash.exportElasticLayout();
     * }</pre>
     *
     * <p><strong>Thread safety:</strong> Not thread-safe. Call from the main robot
     * thread only, after all widgets have been added.
     */
    public static void exportElasticLayout() {
        exportElasticLayout(
            Filesystem.getDeployDirectory().toPath().resolve("elastic-layout.json").toString()
        );
    }

    /**
     * Exports the current dashboard layout as an Elastic Dashboard JSON file
     * to a custom path. See {@link #exportElasticLayout()} for details.
     *
     * @param filePath Path to write the JSON file
     */
    public static void exportElasticLayout(String filePath) {
        // Group widgets by tab, preserving insertion order
        Map<String, List<WidgetInfo>> tabWidgets = new LinkedHashMap<>();
        for (WidgetInfo w : widgets) {
            tabWidgets.computeIfAbsent(w.tabName(), k -> new ArrayList<>()).add(w);
        }

        StringBuilder json = new StringBuilder();
        json.append("{\n");
        json.append("  \"version\": 1.0,\n");
        json.append("  \"grid_size\": ").append(ELASTIC_GRID_SIZE).append(",\n");
        json.append("  \"tabs\": [\n");

        boolean firstTab = true;
        for (var entry : tabWidgets.entrySet()) {
            if (!firstTab) json.append(",\n");
            firstTab = false;

            String tabName = entry.getKey();
            List<WidgetInfo> tabItems = entry.getValue();

            json.append("    {\n");
            json.append("      \"name\": \"").append(escapeJson(tabName)).append("\",\n");
            json.append("      \"grid_layout\": {\n");
            json.append("        \"layouts\": [],\n");
            json.append("        \"containers\": [\n");

            // Collect occupied cells from explicit positions (to avoid auto-layout collisions)
            java.util.Set<Long> occupiedCells = new java.util.HashSet<>();
            for (WidgetInfo w : tabItems) {
                if (w.column() >= 0 && w.row() >= 0) {
                    for (int dx = 0; dx < DEFAULT_WIDGET_CELLS_W; dx++) {
                        for (int dy = 0; dy < DEFAULT_WIDGET_CELLS_H; dy++) {
                            occupiedCells.add(cellKey(w.column() + dx, w.row() + dy));
                        }
                    }
                }
            }

            int autoCol = 0, autoRow = 0;
            boolean firstWidget = true;
            for (WidgetInfo w : tabItems) {
                if (!firstWidget) json.append(",\n");
                firstWidget = false;

                int col, row;
                if (w.column() >= 0 && w.row() >= 0) {
                    col = w.column();
                    row = w.row();
                } else {
                    // Find next free cell, skipping occupied ones
                    while (isOccupied(occupiedCells, autoCol, autoRow)) {
                        autoCol += DEFAULT_WIDGET_CELLS_W;
                        if (autoCol + DEFAULT_WIDGET_CELLS_W > AUTO_LAYOUT_COLS) {
                            autoCol = 0;
                            autoRow++;
                        }
                    }
                    col = autoCol;
                    row = autoRow;
                    // Mark this cell as occupied and advance
                    for (int dx = 0; dx < DEFAULT_WIDGET_CELLS_W; dx++) {
                        for (int dy = 0; dy < DEFAULT_WIDGET_CELLS_H; dy++) {
                            occupiedCells.add(cellKey(col + dx, row + dy));
                        }
                    }
                    autoCol += DEFAULT_WIDGET_CELLS_W;
                    if (autoCol + DEFAULT_WIDGET_CELLS_W > AUTO_LAYOUT_COLS) {
                        autoCol = 0;
                        autoRow++;
                    }
                }

                // Convert cell positions to pixels
                double xPixels = (double) col * ELASTIC_GRID_SIZE;
                double yPixels = (double) row * ELASTIC_GRID_SIZE;
                double widthPixels = (double) DEFAULT_WIDGET_CELLS_W * ELASTIC_GRID_SIZE;
                double heightPixels = (double) DEFAULT_WIDGET_CELLS_H * ELASTIC_GRID_SIZE;

                String widgetType = mapToElasticType(w.dataType(), w.tunable());
                String topic = "/Shuffleboard/" + tabName + "/" + w.name();
                String dataType = mapToElasticDataType(w.dataType());

                json.append("          {\n");
                json.append("            \"title\": \"").append(escapeJson(w.name())).append("\",\n");
                json.append("            \"x\": ").append(formatDouble(xPixels)).append(",\n");
                json.append("            \"y\": ").append(formatDouble(yPixels)).append(",\n");
                json.append("            \"width\": ").append(formatDouble(widthPixels)).append(",\n");
                json.append("            \"height\": ").append(formatDouble(heightPixels)).append(",\n");
                json.append("            \"type\": \"").append(widgetType).append("\",\n");
                json.append("            \"properties\": {\n");
                json.append("              \"topic\": \"").append(escapeJson(topic)).append("\",\n");
                json.append("              \"period\": 0.06");

                if (dataType != null) {
                    json.append(",\n              \"data_type\": \"").append(dataType).append("\"");
                }
                if (w.tunable() && !"boolean".equals(w.dataType())) {
                    json.append(",\n              \"show_submit_button\": true");
                }
                if ("boolean".equals(w.dataType()) && !w.tunable()) {
                    json.append(",\n              \"true_color\": \"0xFF00FF00\"");
                    json.append(",\n              \"false_color\": \"0xFFFF0000\"");
                }

                json.append("\n            }\n");
                json.append("          }");
            }

            json.append("\n        ]\n");
            json.append("      }\n");
            json.append("    }");
        }

        json.append("\n  ]\n");
        json.append("}\n");

        try (FileWriter writer = new FileWriter(filePath)) {
            writer.write(json.toString());
            DriverStation.reportWarning("Dash: Exported Elastic layout to " + filePath +
                " (" + widgets.size() + " widgets)", false);
        } catch (IOException e) {
            DriverStation.reportError("Dash: Failed to export Elastic layout to " + filePath +
                ": " + e.getMessage(), false);
        }
    }

    /** Encodes a (col, row) cell as a packed long for HashSet lookup. */
    private static long cellKey(int col, int row) {
        return ((long) col << 32) | (row & 0xFFFFFFFFL);
    }

    /** Checks if any cell in a 2x1 widget region is occupied. */
    private static boolean isOccupied(java.util.Set<Long> occupied, int col, int row) {
        for (int dx = 0; dx < DEFAULT_WIDGET_CELLS_W; dx++) {
            for (int dy = 0; dy < DEFAULT_WIDGET_CELLS_H; dy++) {
                if (occupied.contains(cellKey(col + dx, row + dy))) {
                    return true;
                }
            }
        }
        return false;
    }

    /** Maps Dash data types to Elastic widget type names. */
    private static String mapToElasticType(String dataType, boolean tunable) {
        if ("boolean".equals(dataType)) {
            return tunable ? "Toggle Switch" : "Boolean Box";
        }
        if ("command".equals(dataType)) {
            return "Command";
        }
        return "Text Display";
    }

    /** Maps Dash data types to Elastic data_type strings, or null for multi-topic widgets. */
    private static String mapToElasticDataType(String dataType) {
        return switch (dataType) {
            case "double" -> "double";
            case "integer" -> "int";
            case "boolean" -> "boolean";
            case "string" -> "string";
            case "doubleArray" -> "double[]";
            case "stringArray" -> "string[]";
            default -> null; // command has no data_type
        };
    }

    /** Formats a double with a fractional part for JSON (e.g., 256.0, not 256). */
    private static String formatDouble(double d) {
        if (d == Math.floor(d) && !Double.isInfinite(d)) {
            return String.format("%.1f", d);
        }
        return Double.toString(d);
    }

    /** Escapes special characters for JSON string values. */
    private static String escapeJson(String s) {
        StringBuilder sb = new StringBuilder(s.length() + 8);
        for (int i = 0; i < s.length(); i++) {
            char c = s.charAt(i);
            switch (c) {
                case '\\' -> sb.append("\\\\");
                case '"' -> sb.append("\\\"");
                case '\n' -> sb.append("\\n");
                case '\r' -> sb.append("\\r");
                case '\t' -> sb.append("\\t");
                case '\b' -> sb.append("\\b");
                case '\f' -> sb.append("\\f");
                default -> {
                    if (c < 0x20) {
                        sb.append(String.format("\\u%04x", (int) c));
                    } else {
                        sb.append(c);
                    }
                }
            }
        }
        return sb.toString();
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
        // Purge widget tracking entries for this tab
        widgets.removeIf(w -> w.tabName().equals(tabName));
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
