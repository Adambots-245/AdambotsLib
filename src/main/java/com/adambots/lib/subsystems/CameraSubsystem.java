// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.lib.subsystems;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.MjpegServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoMode;
import edu.wpi.first.cscore.VideoSource;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Camera subsystem for driver viewing on Shuffleboard or Elastic dashboard.
 *
 * <p>Supports up to two USB cameras connected to the RoboRIO for field viewing
 * and driver awareness. Cameras are optimized for low latency and bandwidth
 * efficiency while providing clear images for teleoperation.
 *
 * <p><strong>Features:</strong>
 * <ul>
 *   <li>Support for 1-2 USB cameras on RoboRIO</li>
 *   <li>Dynamic camera switching with minimal latency</li>
 *   <li>Configurable resolution, FPS, and compression</li>
 *   <li>Automatic dashboard integration (Shuffleboard/Elastic)</li>
 *   <li>Connection status monitoring</li>
 *   <li>Runtime brightness adjustment</li>
 *   <li>Command factory methods for camera control</li>
 * </ul>
 *
 * <p><strong>Hardware Setup:</strong>
 * <ul>
 *   <li>Camera 1: USB port 0 on RoboRIO (typically front camera)</li>
 *   <li>Camera 2: USB port 1 on RoboRIO (typically rear camera)</li>
 *   <li>Recommended: Microsoft LifeCam or Logitech C270/C920</li>
 * </ul>
 *
 * <p><strong>Network Bandwidth:</strong>
 * <ul>
 *   <li>Single camera (320x240, 15 FPS): ~2 Mbps</li>
 *   <li>Two cameras (both active): ~4 Mbps (may strain field network)</li>
 *   <li>Recommended: Use camera switching to keep only one active</li>
 * </ul>
 *
 * <p><strong>Basic Usage:</strong>
 * <pre>{@code
 * // Single camera
 * CameraSubsystem cameras = new CameraSubsystem(
 *     CameraSubsystem.CameraConfig.DRIVER_OPTIMIZED,
 *     true,   // Enable camera 1
 *     false   // Disable camera 2
 * );
 *
 * // Two cameras with switching
 * CameraSubsystem cameras = new CameraSubsystem(
 *     CameraSubsystem.CameraConfig.DRIVER_OPTIMIZED,
 *     true,   // Enable camera 1
 *     true    // Enable camera 2
 * );
 *
 * // Switch cameras with command
 * button.onTrue(cameras.switchCameraCommand(CameraSubsystem.ActiveCamera.CAMERA_TWO));
 * }</pre>
 *
 * <p><strong>Dashboard Integration:</strong>
 * <ul>
 *   <li>Shuffleboard: Cameras appear under "CameraServer" in Sources</li>
 *   <li>Elastic: Add "Camera Stream" widget and select camera</li>
 *   <li>Stream URLs: http://roborio-TEAM-frc.local:1181 (camera 1), 1182 (camera 2)</li>
 * </ul>
 *
 * @see <a href="https://docs.wpilib.org/en/stable/docs/software/vision-processing/roborio/using-the-cameraserver-on-the-roborio.html">WPILib CameraServer</a>
 */
public class CameraSubsystem extends SubsystemBase {

    /**
     * Camera selection enum for switching between cameras.
     */
    public enum ActiveCamera {
        /** Front/primary camera (USB port 0) */
        CAMERA_ONE,
        /** Rear/secondary camera (USB port 1) */
        CAMERA_TWO
    }

    /**
     * Pre-configured camera settings optimized for different use cases.
     */
    public enum CameraConfig {
        /**
         * Driver optimized - balanced latency and bandwidth.
         * Resolution: 320x240, FPS: 15, Compression: 30
         * Bandwidth: ~2 Mbps, Latency: ~200ms
         */
        DRIVER_OPTIMIZED(320, 240, 15, 30),

        /**
         * High detail - better image quality, higher bandwidth.
         * Resolution: 640x480, FPS: 10, Compression: 30
         * Bandwidth: ~3 Mbps, Latency: ~250ms
         */
        HIGH_DETAIL(640, 480, 10, 30),

        /**
         * Low bandwidth - minimal network usage.
         * Resolution: 160x120, FPS: 10, Compression: 20
         * Bandwidth: ~0.5 Mbps, Latency: ~300ms
         */
        LOW_BANDWIDTH(160, 120, 10, 20),

        /**
         * Balanced - good compromise for most situations.
         * Resolution: 480x360, FPS: 12, Compression: 25
         * Bandwidth: ~2.5 Mbps, Latency: ~220ms
         */
        BALANCED(480, 360, 12, 25);

        public final int width;
        public final int height;
        public final int fps;
        public final int compression;

        CameraConfig(int width, int height, int fps, int compression) {
            this.width = width;
            this.height = height;
            this.fps = fps;
            this.compression = compression;
        }
    }

    // Camera hardware
    private final UsbCamera camera1;
    private final UsbCamera camera2;
    private final MjpegServer server1;
    private final MjpegServer server2;

    // Configuration
    private final CameraConfig config;
    private final boolean camera1Enabled;
    private final boolean camera2Enabled;

    // State tracking
    private ActiveCamera activeCamera = ActiveCamera.CAMERA_ONE;
    private int brightness = 85;

    // NetworkTables
    private final NetworkTable cameraTable;

    /**
     * Create a camera subsystem with two cameras.
     *
     * @param config Camera configuration preset
     * @param enableCamera1 Enable camera 1 (USB port 0)
     * @param enableCamera2 Enable camera 2 (USB port 1)
     */
    public CameraSubsystem(CameraConfig config, boolean enableCamera1, boolean enableCamera2) {
        this.config = config;
        this.camera1Enabled = enableCamera1;
        this.camera2Enabled = enableCamera2;

        // Get NetworkTable reference
        cameraTable = NetworkTableInstance.getDefault().getTable("CameraServer");

        // Initialize cameras
        if (camera1Enabled) {
            camera1 = initializeCamera("Camera 1", 0);
            server1 = CameraServer.addServer("Camera 1 Server", 1181);
            server1.setSource(camera1);
            configureServer(server1);
        } else {
            camera1 = null;
            server1 = null;
        }

        if (camera2Enabled) {
            camera2 = initializeCamera("Camera 2", 1);
            server2 = CameraServer.addServer("Camera 2 Server", 1182);
            server2.setSource(camera2);
            configureServer(server2);

            // Keep connection open to avoid reconnection delay when switching
            if (camera1Enabled) {
                camera2.setConnectionStrategy(VideoSource.ConnectionStrategy.kKeepOpen);
            }
        } else {
            camera2 = null;
            server2 = null;
        }

        // Keep camera 1 connection open if we have two cameras
        if (camera1Enabled && camera2Enabled && camera1 != null) {
            camera1.setConnectionStrategy(VideoSource.ConnectionStrategy.kKeepOpen);
        }

        System.out.println("CameraSubsystem initialized:");
        System.out.println("  Camera 1: " + (camera1Enabled ? "ENABLED" : "DISABLED"));
        System.out.println("  Camera 2: " + (camera2Enabled ? "ENABLED" : "DISABLED"));
        System.out.println("  Resolution: " + config.width + "x" + config.height);
        System.out.println("  FPS: " + config.fps);
        System.out.println("  Compression: " + config.compression);
    }

    /**
     * Create a camera subsystem with a single camera.
     *
     * @param config Camera configuration preset
     */
    public CameraSubsystem(CameraConfig config) {
        this(config, true, false);
    }

    /**
     * Initialize a USB camera with configured settings.
     *
     * @param name Camera name for NetworkTables
     * @param port USB port (0 or 1)
     * @return Configured UsbCamera
     */
    private UsbCamera initializeCamera(String name, int port) {
        UsbCamera camera = new UsbCamera(name, port);

        // Set video mode (MJPEG format)
        camera.setResolution(config.width, config.height);
        camera.setFPS(config.fps);

        // Configure camera properties
        try {
            camera.setBrightness(brightness);
            // Note: Auto-exposure is typically enabled by default
        } catch (Exception e) {
            System.err.println("Failed to configure " + name + " properties: " + e.getMessage());
        }

        return camera;
    }

    /**
     * Configure MJPEG server settings.
     *
     * @param server Server to configure
     */
    private void configureServer(MjpegServer server) {
        server.setCompression(config.compression);
        server.setResolution(config.width, config.height);
        server.setFPS(config.fps);
    }

    @Override
    public void periodic() {
        // Monitor connection status
        if (camera1Enabled && camera1 != null) {
            SmartDashboard.putBoolean("Camera 1/Connected", camera1.isConnected());
        }

        if (camera2Enabled && camera2 != null) {
            SmartDashboard.putBoolean("Camera 2/Connected", camera2.isConnected());
        }

        // Show active camera
        if (camera1Enabled && camera2Enabled) {
            SmartDashboard.putString("Active Camera", activeCamera.toString());
        }

        // Report overall status
        boolean anyConnected = false;
        if (camera1Enabled && camera1 != null && camera1.isConnected()) anyConnected = true;
        if (camera2Enabled && camera2 != null && camera2.isConnected()) anyConnected = true;

        SmartDashboard.putString("Camera/Status", anyConnected ? "Online" : "Offline");
    }

    /**
     * Switch to the specified camera.
     * Only effective if both cameras are enabled.
     *
     * @param camera Camera to switch to
     */
    public void switchCamera(ActiveCamera camera) {
        if (!camera1Enabled || !camera2Enabled) {
            System.err.println("Cannot switch cameras - both cameras must be enabled");
            return;
        }

        activeCamera = camera;
        System.out.println("Switched to " + camera);
    }

    /**
     * Get the currently active camera.
     *
     * @return Active camera
     */
    public ActiveCamera getActiveCamera() {
        return activeCamera;
    }

    /**
     * Set brightness for all enabled cameras.
     *
     * @param brightness Brightness value (0-255)
     */
    public void setBrightness(int brightness) {
        this.brightness = Math.min(255, Math.max(0, brightness));

        try {
            if (camera1Enabled && camera1 != null) {
                camera1.setBrightness(this.brightness);
            }
            if (camera2Enabled && camera2 != null) {
                camera2.setBrightness(this.brightness);
            }
        } catch (Exception e) {
            System.err.println("Failed to set brightness: " + e.getMessage());
        }
    }

    /**
     * Increase brightness by 10.
     */
    public void increaseBrightness() {
        setBrightness(brightness + 10);
    }

    /**
     * Decrease brightness by 10.
     */
    public void decreaseBrightness() {
        setBrightness(brightness - 10);
    }

    /**
     * Get current brightness setting.
     *
     * @return Brightness value (0-255)
     */
    public int getBrightness() {
        return brightness;
    }

    /**
     * Check if camera 1 is connected.
     *
     * @return true if camera 1 is connected
     */
    public boolean isCamera1Connected() {
        return camera1Enabled && camera1 != null && camera1.isConnected();
    }

    /**
     * Check if camera 2 is connected.
     *
     * @return true if camera 2 is connected
     */
    public boolean isCamera2Connected() {
        return camera2Enabled && camera2 != null && camera2.isConnected();
    }

    /**
     * Check if any camera is connected.
     *
     * @return true if at least one camera is connected
     */
    public boolean isAnyCameraConnected() {
        return isCamera1Connected() || isCamera2Connected();
    }

    /**
     * Get the streaming URL for camera 1.
     *
     * @param teamNumber Team number for URL generation
     * @return Stream URL for camera 1
     */
    public String getCamera1StreamUrl(int teamNumber) {
        return "http://roborio-" + teamNumber + "-frc.local:1181/?action=stream";
    }

    /**
     * Get the streaming URL for camera 2.
     *
     * @param teamNumber Team number for URL generation
     * @return Stream URL for camera 2
     */
    public String getCamera2StreamUrl(int teamNumber) {
        return "http://roborio-" + teamNumber + "-frc.local:1182/?action=stream";
    }

    // ==================== COMMAND FACTORIES ====================

    /**
     * Command to switch to the specified camera.
     *
     * @param camera Camera to switch to
     * @return Command that switches cameras
     */
    public Command switchCameraCommand(ActiveCamera camera) {
        return Commands.runOnce(() -> switchCamera(camera), this)
            .withName("SwitchTo" + camera);
    }

    /**
     * Command to switch to camera 1.
     *
     * @return Command that switches to camera 1
     */
    public Command switchToCamera1Command() {
        return switchCameraCommand(ActiveCamera.CAMERA_ONE);
    }

    /**
     * Command to switch to camera 2.
     *
     * @return Command that switches to camera 2
     */
    public Command switchToCamera2Command() {
        return switchCameraCommand(ActiveCamera.CAMERA_TWO);
    }

    /**
     * Command to toggle between cameras.
     * Only works if both cameras are enabled.
     *
     * @return Command that toggles cameras
     */
    public Command toggleCameraCommand() {
        return Commands.runOnce(() -> {
            if (activeCamera == ActiveCamera.CAMERA_ONE) {
                switchCamera(ActiveCamera.CAMERA_TWO);
            } else {
                switchCamera(ActiveCamera.CAMERA_ONE);
            }
        }, this).withName("ToggleCamera");
    }

    /**
     * Command to increase brightness.
     *
     * @return Command that increases brightness
     */
    public Command increaseBrightnessCommand() {
        return Commands.runOnce(this::increaseBrightness, this)
            .withName("IncreaseBrightness");
    }

    /**
     * Command to decrease brightness.
     *
     * @return Command that decreases brightness
     */
    public Command decreaseBrightnessCommand() {
        return Commands.runOnce(this::decreaseBrightness, this)
            .withName("DecreaseBrightness");
    }

    /**
     * Command to set brightness to a specific value.
     *
     * @param brightness Brightness value (0-255)
     * @return Command that sets brightness
     */
    public Command setBrightnessCommand(int brightness) {
        return Commands.runOnce(() -> setBrightness(brightness), this)
            .withName("SetBrightness(" + brightness + ")");
    }

    /**
     * Command to reset brightness to default (85).
     *
     * @return Command that resets brightness
     */
    public Command resetBrightnessCommand() {
        return setBrightnessCommand(85);
    }
}
