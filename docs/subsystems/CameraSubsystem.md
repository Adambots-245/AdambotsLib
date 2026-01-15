# CameraSubsystem

USB camera subsystem for driver viewing on Shuffleboard or Elastic dashboard.

## Table of Contents

- [Overview](#overview)
- [Quick Start](#quick-start)
- [Hardware Setup](#hardware-setup)
- [Camera Configurations](#camera-configurations)
- [Camera Switching](#camera-switching)
- [Dashboard Integration](#dashboard-integration)
- [Real-World Examples](#real-world-examples)
- [API Reference](#api-reference)
- [Best Practices](#best-practices)
- [Troubleshooting](#troubleshooting)

---

## Overview

The `CameraSubsystem` provides easy integration of USB cameras for driver field viewing. It supports up to two cameras connected directly to the RoboRIO's USB ports, with automatic streaming to Shuffleboard or Elastic dashboards.

**Key Features:**
- **1-2 USB cameras** - Front and rear camera support
- **Pre-configured settings** - Optimized presets for different scenarios
- **Dynamic switching** - Toggle between cameras with minimal latency
- **Automatic dashboard integration** - Works with Shuffleboard and Elastic
- **Runtime brightness control** - Adjust for field lighting conditions
- **Connection monitoring** - Track camera status in real-time
- **Command-based control** - Full command factory support

**This is NOT for AprilTag vision** - Use [PhotonVision](../vision/PhotonVision.md) for AprilTag pose estimation. This subsystem is purely for driver field viewing.

---

## Quick Start

### Single Camera Setup

```java
public class RobotContainer {
    // Create camera subsystem with driver-optimized settings
    private final CameraSubsystem camera = new CameraSubsystem(
        CameraSubsystem.CameraConfig.DRIVER_OPTIMIZED
    );

    public RobotContainer() {
        // Camera automatically streams to dashboard
        // View in Shuffleboard under "CameraServer" sources
    }
}
```

### Two Camera Setup with Switching

```java
public class RobotContainer {
    // Enable both cameras
    private final CameraSubsystem cameras = new CameraSubsystem(
        CameraSubsystem.CameraConfig.DRIVER_OPTIMIZED,
        true,  // Enable camera 1 (front)
        true   // Enable camera 2 (rear)
    );

    private void configureButtonBindings() {
        // Toggle between cameras with button
        new JoystickButton(driver, 5).onTrue(
            cameras.toggleCameraCommand()
        );

        // Or switch to specific camera
        new JoystickButton(driver, 3).onTrue(
            cameras.switchToCamera1Command()  // Front
        );
        new JoystickButton(driver, 4).onTrue(
            cameras.switchToCamera2Command()  // Rear
        );
    }
}
```

### View on Dashboard

**Shuffleboard:**
1. Open Shuffleboard
2. Click "Sources" in the left panel
3. Find "CameraServer" → "Camera 1"
4. Drag camera stream to a tab

**Elastic:**
1. Open Elastic Dashboard
2. Add widget → "Camera Stream"
3. Select "Camera 1" or "Camera 2"

---

## Hardware Setup

### Camera Connections

| USB Port | Typical Use | Cable |
|----------|-------------|-------|
| **USB 0** | Front camera (camera 1) | USB 2.0/3.0 cable |
| **USB 1** | Rear camera (camera 2) | USB 2.0/3.0 cable |

**Recommended Cameras:**
- **Microsoft LifeCam HD-3000** - $25, proven FRC reliability
- **Logitech C270** - $30, good low-light performance
- **Logitech C920** - $60, higher quality, may use more bandwidth

### Physical Mounting

**Front Camera:**
- Mount high for long-range view
- Angle slightly downward to see close objects
- Protect from game pieces
- Ensure USB cable has strain relief

**Rear Camera:**
- Mount facing backward/upward
- Position to see human player station
- Protect from impacts during reversing
- Consider quick-disconnect for servicing

### Cable Management

- Use **USB 2.0 cables** (3.0 not required for cameras)
- Maximum cable length: **5 meters** (16 feet)
- Use cable ties and strain relief
- Avoid routing near motor controllers (EMI)
- Test camera connection before securing cables

---

## Camera Configurations

### Pre-Configured Presets

The subsystem provides four optimized configurations:

#### DRIVER_OPTIMIZED (Recommended)

Best balance of image quality, latency, and bandwidth for driver viewing.

```java
CameraSubsystem.CameraConfig.DRIVER_OPTIMIZED
```

**Specifications:**
- Resolution: **320x240**
- FPS: **15**
- Compression: **30**
- Bandwidth: **~2 Mbps**
- Latency: **~200ms**

**Use when:** Normal driving, most competitions

---

#### HIGH_DETAIL

Better image quality for precise maneuvering, higher bandwidth usage.

```java
CameraSubsystem.CameraConfig.HIGH_DETAIL
```

**Specifications:**
- Resolution: **640x480**
- FPS: **10**
- Compression: **30**
- Bandwidth: **~3 Mbps**
- Latency: **~250ms**

**Use when:** Need to see fine details, low-traffic network

---

#### LOW_BANDWIDTH

Minimal network usage for bandwidth-constrained situations.

```java
CameraSubsystem.CameraConfig.LOW_BANDWIDTH
```

**Specifications:**
- Resolution: **160x120**
- FPS: **10**
- Compression: **20**
- Bandwidth: **~0.5 Mbps**
- Latency: **~300ms**

**Use when:** Network congestion, multiple data streams active

---

#### BALANCED

Good compromise for most situations.

```java
CameraSubsystem.CameraConfig.BALANCED
```

**Specifications:**
- Resolution: **480x360**
- FPS: **12**
- Compression: **25**
- Bandwidth: **~2.5 Mbps**
- Latency: **~220ms**

**Use when:** Need slightly better quality than DRIVER_OPTIMIZED

---

### Configuration Comparison

| Config | Resolution | FPS | Bandwidth | Best For |
|--------|-----------|-----|-----------|----------|
| **DRIVER_OPTIMIZED** | 320x240 | 15 | 2 Mbps | Most competitions |
| **HIGH_DETAIL** | 640x480 | 10 | 3 Mbps | Precise driving |
| **LOW_BANDWIDTH** | 160x120 | 10 | 0.5 Mbps | Network issues |
| **BALANCED** | 480x360 | 12 | 2.5 Mbps | General use |

---

## Camera Switching

### Why Switch Cameras?

- **Forward driving** - Use front camera
- **Reverse driving** - Use rear camera
- **Human player station** - Use rear camera to see loading
- **Bandwidth management** - Only one camera active at a time

### Switching Methods

#### Toggle Between Cameras

```java
// Switch back and forth with single button
button.onTrue(cameras.toggleCameraCommand());
```

#### Switch to Specific Camera

```java
// Front camera button
frontButton.onTrue(cameras.switchToCamera1Command());

// Rear camera button
rearButton.onTrue(cameras.switchToCamera2Command());
```

#### Programmatic Switching

```java
// Switch based on drive direction
if (drivetrain.isMovingBackward()) {
    cameras.switchCamera(CameraSubsystem.ActiveCamera.CAMERA_TWO);
} else {
    cameras.switchCamera(CameraSubsystem.ActiveCamera.CAMERA_ONE);
}
```

### Latency Considerations

**With `kConnectionKeepOpen` (used by default):**
- Switching latency: **Instant** (~0ms)
- Both cameras consume bandwidth simultaneously
- Recommended for competitions

**Without `kConnectionKeepOpen`:**
- Switching latency: **1-2 seconds** (reconnection delay)
- Only active camera consumes bandwidth
- Not recommended for real-time switching

---

## Dashboard Integration

### Shuffleboard Setup

1. **Find Camera Streams:**
   - Click "Sources" in left panel
   - Expand "CameraServer"
   - Look for "Camera 1" and "Camera 2"

2. **Add to Dashboard:**
   - Drag camera to a tab
   - Or right-click → "Show as: Camera Stream"

3. **Configure Widget:**
   - Adjust size by dragging corners
   - Right-click → Properties to adjust settings
   - Can show both cameras side-by-side

### Elastic Dashboard Setup

1. **Add Camera Widget:**
   - Click "+" to add widget
   - Select "Camera Stream"

2. **Configure Stream:**
   - Choose "Camera 1" or "Camera 2" from dropdown
   - Adjust widget size as needed

3. **Layout Tips:**
   - Place camera prominently for driver
   - Consider full-screen mode for driver station
   - Can create multiple tabs with different cameras

### Stream URLs

If using custom dashboard or browser viewing:

```
Camera 1: http://roborio-TEAM-frc.local:1181/?action=stream
Camera 2: http://roborio-TEAM-frc.local:1182/?action=stream
```

Replace `TEAM` with your team number (e.g., `roborio-1234-frc.local`).

---

## Real-World Examples

### Example 1: Basic Single Camera

```java
public class RobotContainer {
    // Single front-facing camera for driver
    private final CameraSubsystem camera = new CameraSubsystem(
        CameraSubsystem.CameraConfig.DRIVER_OPTIMIZED
    );

    public RobotContainer() {
        // Camera automatically starts streaming
        System.out.println("Camera stream: " + camera.getCamera1StreamUrl(1234));
    }
}
```

### Example 2: Two Cameras with Button Switching

```java
public class RobotContainer {
    private final XboxController driver = new XboxController(0);
    private final CameraSubsystem cameras = new CameraSubsystem(
        CameraSubsystem.CameraConfig.DRIVER_OPTIMIZED,
        true,  // Front camera
        true   // Rear camera
    );

    private void configureButtonBindings() {
        // Left bumper = front camera
        new JoystickButton(driver, XboxController.Button.kLeftBumper.value)
            .onTrue(cameras.switchToCamera1Command());

        // Right bumper = rear camera
        new JoystickButton(driver, XboxController.Button.kRightBumper.value)
            .onTrue(cameras.switchToCamera2Command());

        // Back button = toggle
        new JoystickButton(driver, XboxController.Button.kBack.value)
            .onTrue(cameras.toggleCameraCommand());
    }
}
```

### Example 3: Auto-Switch Based on Drive Direction

```java
public class SwerveSubsystem extends SubsystemBase {
    private final CameraSubsystem cameras;

    public SwerveSubsystem(CameraSubsystem cameras) {
        this.cameras = cameras;
    }

    @Override
    public void periodic() {
        // Get robot velocity
        ChassisSpeeds speeds = swerveDrive.getFieldVelocity();

        // Auto-switch camera based on direction
        if (speeds.vxMetersPerSecond < -0.5) {
            // Moving backward significantly
            cameras.switchCamera(CameraSubsystem.ActiveCamera.CAMERA_TWO);
        } else {
            // Moving forward or stationary
            cameras.switchCamera(CameraSubsystem.ActiveCamera.CAMERA_ONE);
        }
    }
}
```

### Example 4: Brightness Control

```java
public class RobotContainer {
    private final XboxController operator = new XboxController(1);
    private final CameraSubsystem cameras = new CameraSubsystem(
        CameraSubsystem.CameraConfig.DRIVER_OPTIMIZED
    );

    private void configureButtonBindings() {
        // D-pad up = increase brightness
        new POVButton(operator, 0)
            .onTrue(cameras.increaseBrightnessCommand());

        // D-pad down = decrease brightness
        new POVButton(operator, 180)
            .onTrue(cameras.decreaseBrightnessCommand());

        // Start button = reset to default brightness
        new JoystickButton(operator, XboxController.Button.kStart.value)
            .onTrue(cameras.resetBrightnessCommand());
    }
}
```

### Example 5: Low-Bandwidth Configuration for Events

```java
public class RobotContainer {
    // Use low bandwidth config at crowded events
    private final CameraSubsystem camera = new CameraSubsystem(
        CameraSubsystem.CameraConfig.LOW_BANDWIDTH
    );

    public RobotContainer() {
        // Monitor connection in periodic
        SmartDashboard.putBoolean("Camera OK", camera.isCamera1Connected());
    }
}
```

### Example 6: Conditional Camera Based on Game State

```java
public class RobotContainer {
    private final CameraSubsystem cameras = new CameraSubsystem(
        CameraSubsystem.CameraConfig.DRIVER_OPTIMIZED,
        true, true
    );

    // Create triggers for auto-switching
    private void configureAutoSwitching() {
        // At human player station = rear camera
        new Trigger(() -> {
            Pose2d pose = swerve.getPose();
            return pose.getX() < 2.0;  // Near alliance wall
        }).onTrue(cameras.switchToCamera2Command());

        // On field = front camera
        new Trigger(() -> {
            Pose2d pose = swerve.getPose();
            return pose.getX() > 2.0;
        }).onTrue(cameras.switchToCamera1Command());
    }
}
```

---

## API Reference

### Constructors

#### `CameraSubsystem(CameraConfig config, boolean enableCamera1, boolean enableCamera2)`

Create camera subsystem with two cameras.

**Parameters:**
- `config` - Camera configuration preset
- `enableCamera1` - Enable camera 1 (USB port 0)
- `enableCamera2` - Enable camera 2 (USB port 1)

**Example:**
```java
new CameraSubsystem(CameraConfig.DRIVER_OPTIMIZED, true, true);
```

#### `CameraSubsystem(CameraConfig config)`

Create camera subsystem with single camera (camera 1 only).

**Parameters:**
- `config` - Camera configuration preset

**Example:**
```java
new CameraSubsystem(CameraConfig.DRIVER_OPTIMIZED);
```

---

### Camera Switching Methods

#### `switchCamera(ActiveCamera camera)`

Switch to the specified camera.

**Parameters:**
- `camera` - Camera to switch to (`CAMERA_ONE` or `CAMERA_TWO`)

**Example:**
```java
cameras.switchCamera(ActiveCamera.CAMERA_TWO);
```

#### `getActiveCamera()`

Get the currently active camera.

**Returns:** `ActiveCamera` - Current active camera

**Example:**
```java
ActiveCamera current = cameras.getActiveCamera();
```

---

### Brightness Methods

#### `setBrightness(int brightness)`

Set brightness for all enabled cameras.

**Parameters:**
- `brightness` - Brightness value (0-255)

**Example:**
```java
cameras.setBrightness(100);
```

#### `increaseBrightness()`

Increase brightness by 10.

**Example:**
```java
cameras.increaseBrightness();
```

#### `decreaseBrightness()`

Decrease brightness by 10.

**Example:**
```java
cameras.decreaseBrightness();
```

#### `getBrightness()`

Get current brightness setting.

**Returns:** `int` - Brightness value (0-255)

**Example:**
```java
int brightness = cameras.getBrightness();
```

---

### Connection Status Methods

#### `isCamera1Connected()`

Check if camera 1 is connected.

**Returns:** `boolean` - true if connected

**Example:**
```java
if (cameras.isCamera1Connected()) {
    System.out.println("Front camera online");
}
```

#### `isCamera2Connected()`

Check if camera 2 is connected.

**Returns:** `boolean` - true if connected

**Example:**
```java
if (cameras.isCamera2Connected()) {
    System.out.println("Rear camera online");
}
```

#### `isAnyCameraConnected()`

Check if any camera is connected.

**Returns:** `boolean` - true if at least one camera connected

**Example:**
```java
if (!cameras.isAnyCameraConnected()) {
    DriverStation.reportWarning("No cameras connected!", false);
}
```

---

### Stream URL Methods

#### `getCamera1StreamUrl(int teamNumber)`

Get the streaming URL for camera 1.

**Parameters:**
- `teamNumber` - Team number for URL

**Returns:** `String` - Stream URL

**Example:**
```java
String url = cameras.getCamera1StreamUrl(1234);
// "http://roborio-1234-frc.local:1181/?action=stream"
```

#### `getCamera2StreamUrl(int teamNumber)`

Get the streaming URL for camera 2.

**Parameters:**
- `teamNumber` - Team number for URL

**Returns:** `String` - Stream URL

**Example:**
```java
String url = cameras.getCamera2StreamUrl(1234);
```

---

### Command Factory Methods

#### `switchCameraCommand(ActiveCamera camera)`

Command to switch to specified camera.

**Parameters:**
- `camera` - Camera to switch to

**Returns:** `Command`

**Example:**
```java
button.onTrue(cameras.switchCameraCommand(ActiveCamera.CAMERA_TWO));
```

#### `switchToCamera1Command()`

Command to switch to camera 1.

**Returns:** `Command`

**Example:**
```java
frontButton.onTrue(cameras.switchToCamera1Command());
```

#### `switchToCamera2Command()`

Command to switch to camera 2.

**Returns:** `Command`

**Example:**
```java
rearButton.onTrue(cameras.switchToCamera2Command());
```

#### `toggleCameraCommand()`

Command to toggle between cameras.

**Returns:** `Command`

**Example:**
```java
toggleButton.onTrue(cameras.toggleCameraCommand());
```

#### `increaseBrightnessCommand()`

Command to increase brightness.

**Returns:** `Command`

**Example:**
```java
dpadUp.onTrue(cameras.increaseBrightnessCommand());
```

#### `decreaseBrightnessCommand()`

Command to decrease brightness.

**Returns:** `Command`

**Example:**
```java
dpadDown.onTrue(cameras.decreaseBrightnessCommand());
```

#### `setBrightnessCommand(int brightness)`

Command to set brightness to specific value.

**Parameters:**
- `brightness` - Brightness value (0-255)

**Returns:** `Command`

**Example:**
```java
button.onTrue(cameras.setBrightnessCommand(150));
```

#### `resetBrightnessCommand()`

Command to reset brightness to default (85).

**Returns:** `Command`

**Example:**
```java
resetButton.onTrue(cameras.resetBrightnessCommand());
```

---

## Best Practices

### 1. Choose the Right Configuration

**For Most Competitions:**
```java
CameraConfig.DRIVER_OPTIMIZED  // 320x240, 15 FPS
```

**For Crowded Events (High Network Traffic):**
```java
CameraConfig.LOW_BANDWIDTH  // 160x120, 10 FPS
```

**For Precision Maneuvering:**
```java
CameraConfig.HIGH_DETAIL  // 640x480, 10 FPS
```

### 2. Enable Only Needed Cameras

```java
// DON'T (wastes bandwidth with unused camera)
new CameraSubsystem(config, true, true);  // Both always streaming

// DO (only enable what you need)
new CameraSubsystem(config, true, false);  // Single camera only
```

If you need two cameras, switching is better than running both simultaneously.

### 3. Monitor Camera Status

```java
@Override
public void periodic() {
    if (!cameras.isAnyCameraConnected()) {
        DriverStation.reportWarning("Camera disconnected!", false);
    }
}
```

### 4. Test on Practice Robot First

- Verify camera compatibility
- Test switching latency
- Measure actual bandwidth usage
- Adjust brightness for field lighting

### 5. Cable Management

- Use strain relief on USB connections
- Secure cables away from moving mechanisms
- Test cable routing under robot movement
- Have spare USB cables at competition

### 6. Bandwidth Budgeting

**Total FRC Field Allocation:** ~4.5 Mbps

**Example Budget:**
- Telemetry (RoboRIO ↔ DS): 0.5 Mbps
- Camera (DRIVER_OPTIMIZED): 2.0 Mbps
- Safety margin: 2.0 Mbps
- **Total:** 4.5 Mbps ✓

**With Two Cameras Active:**
- Telemetry: 0.5 Mbps
- Camera 1: 2.0 Mbps
- Camera 2: 2.0 Mbps
- **Total:** 4.5 Mbps (at limit) ⚠

### 7. Dashboard Layout

**Driver Station:**
- Make camera widget large and prominent
- Position near center of driver screen
- Use full-screen camera during critical maneuvers
- Test visibility from driver position

**Pit Display:**
- Can show both cameras side-by-side
- Add camera status indicators
- Display active camera name

### 8. Brightness Adjustment

```java
// Adjust for field lighting during setup
cameras.setBrightness(100);  // Bright field
cameras.setBrightness(70);   // Dim field
```

Test brightness at event and save preferred setting.

---

## Troubleshooting

### Camera Not Appearing on Dashboard

**Symptoms:** No camera stream in Shuffleboard/Elastic

**Solutions:**
1. Check USB connection to RoboRIO
2. Verify camera is enabled in constructor
3. Check robot code is deployed and running
4. Restart Shuffleboard/Elastic
5. Check "CameraServer" in NetworkTables Viewer

---

### High Latency (>300ms)

**Symptoms:** Delayed camera feed, hard to drive

**Solutions:**
1. Switch to `LOW_BANDWIDTH` configuration
2. Reduce FPS: Lower frame rate can improve latency
3. Check for network congestion
4. Disable unused cameras
5. Verify field network is functioning

---

### USB Bandwidth Error

**Symptoms:** Error message: "could not start streaming due to USB bandwidth limitations"

**Solutions:**
1. Use `DRIVER_OPTIMIZED` or `LOW_BANDWIDTH` config
2. Reduce resolution to 320x240 or lower
3. Lower FPS to 10
4. Switch pixel format to MJPEG (already default)
5. Don't run both cameras at high resolution

---

### Camera Disconnects Randomly

**Symptoms:** Connection status flaps between online/offline

**Solutions:**
1. Check USB cable quality
2. Replace USB cable
3. Try different USB port on RoboRIO
4. Verify camera power requirements
5. Check for EMI near camera cables

---

### Image Too Dark or Bright

**Symptoms:** Can't see field clearly

**Solutions:**
1. Adjust brightness: `cameras.setBrightness(value)`
2. Test different brightness values (50-150)
3. Enable auto-exposure (already default)
4. Reposition camera for better lighting
5. Different camera may have better low-light performance

---

### Slow Camera Switching

**Symptoms:** 1-2 second delay when switching cameras

**Solutions:**
1. Verify code uses `kConnectionKeepOpen` (already default in this subsystem)
2. Check that both cameras are enabled
3. This is normal without `kConnectionKeepOpen`
4. Consider bandwidth trade-off of keeping both cameras active

---

### Camera Works in Simulation, Not on Robot

**Symptoms:** Camera streams in sim but not on real robot

**Solutions:**
1. Verify camera is connected to RoboRIO (not driver station)
2. Check USB cable is good quality
3. Try different USB port
4. Verify camera is supported (Microsoft LifeCam, Logitech recommended)
5. Check roboRIO logs for error messages

---

## Network Bandwidth Reference

### Configuration Bandwidth Usage

| Config | Resolution | FPS | Estimated Bandwidth |
|--------|-----------|-----|---------------------|
| DRIVER_OPTIMIZED | 320x240 | 15 | 2.0 Mbps |
| HIGH_DETAIL | 640x480 | 10 | 3.0 Mbps |
| LOW_BANDWIDTH | 160x120 | 10 | 0.5 Mbps |
| BALANCED | 480x360 | 12 | 2.5 Mbps |

### Field Network Limits

- **FRC Field Allocation:** ~4.5 Mbps per team
- **RoboRIO to Driver Station:** Must include telemetry + camera
- **Recommended:** Leave 1-2 Mbps safety margin

---

## See Also

- [PhotonVision](../vision/PhotonVision.md) - AprilTag vision for pose estimation
- [SwerveSubsystem](SwerveSubsystem.md) - Swerve drive integration
- [WPILib CameraServer Documentation](https://docs.wpilib.org/en/stable/docs/software/vision-processing/roborio/using-the-cameraserver-on-the-roborio.html)
- [Shuffleboard Camera Streams](https://docs.wpilib.org/en/stable/docs/software/dashboards/shuffleboard/getting-started/shuffleboard-displaying-camera.html)

---

**Need Help?** Check the WPILib documentation or ask in the FRC Discord #programming channel.
