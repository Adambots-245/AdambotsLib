# CANdleSubsystem

CTRE CANdle LED strip controller subsystem using Phoenix 6 API.

## Overview

`CANdleSubsystem` provides a command-based interface for controlling WS2812B-compatible LED strips via CTRE's CANdle device. It supports both solid colors and animations for robot status indication.

**Phoenix 6 Migration:** This subsystem has been fully migrated to Phoenix 6 API and uses the modern control request pattern for LED control.

## Constructor

### CANdleSubsystem(int canID)

Creates a CANdle LED subsystem with the specified CAN ID.

```java
CANdleSubsystem leds = new CANdleSubsystem(10);  // CAN ID 10
```

**Parameters:**
- `canID` - CAN ID for the CANdle (0-62)

**Validation:**
- CAN ID validated at construction
- Invalid IDs default to 1 with DriverStation error

**Configuration Applied:**
- LED strip type from Constants (GRB for BTF-Lighting)
- Brightness scalar: 1.0 (100%)
- Status LED disabled when active
- 5V rail enabled
- VBat output disabled

### CANdleSubsystem(CANdle candleDevice)

**Deprecated:** Use `CANdleSubsystem(int)` instead.

Creates a CANdle subsystem with a pre-configured CANdle device.

## Methods

### setColor(Color color)

Sets all LEDs to a solid color using WPILib Color object.

```java
leds.setColor(LEDConstants.green);
leds.setColor(new Color(255, 0, 0));  // Red
```

**Parameters:**
- `color` - WPILib Color object (RGB values 0.0-1.0)

**Behavior:**
- Clears all animations
- Sets entire LED strip to specified color
- Values clamped to 0-255 range

---

### setColor(int r, int g, int b)

Sets all LEDs to a solid color using RGB values.

```java
leds.setColor(255, 0, 0);    // Red
leds.setColor(0, 255, 0);    // Green
leds.setColor(0, 0, 255);    // Blue
```

**Parameters:**
- `r` - Red (0-255)
- `g` - Green (0-255)
- `b` - Blue (0-255)

**Behavior:**
- Clears all animations
- Values automatically clamped to 0-255

---

### setLEDs(int r, int g, int b, int startIdx, int numOfLEDs)

Sets a specific range of LEDs to a color.

```java
// Set first 20 LEDs to red
leds.setLEDs(255, 0, 0, 8, 20);  // Start at index 8 (external strip)

// Set middle section to blue
leds.setLEDs(0, 0, 255, 30, 10);
```

**Parameters:**
- `r` - Red (0-255)
- `g` - Green (0-255)
- `b` - Blue (0-255)
- `startIdx` - Starting LED index (0-based)
- `numOfLEDs` - Number of LEDs to set

**LED Indexing:**
- Indices 0-7: Onboard CANdle LEDs
- Indices 8-399: External LED strip (8 is first LED)

**Validation:**
- All values clamped to valid ranges
- Invalid indices default to safe values

---

### setAnimation(AnimationTypes toChange)

Sets the active animation pattern.

```java
leds.setAnimation(AnimationTypes.Rainbow);
leds.setAnimation(AnimationTypes.Larson);
leds.setAnimation(AnimationTypes.Fire);
```

**Parameters:**
- `toChange` - Animation type from `AnimationTypes` enum

**Available Animations:**

| Animation | Description | Parameters |
|-----------|-------------|------------|
| `ColorFlow` | Gradient flow effect | Orange color, forward direction |
| `Fire` | Flickering flame effect | 50% brightness, sparking/cooling |
| `Larson` | Bouncing "Knight Rider" effect | Orange color, 30 LED size |
| `Rainbow` | Cycling rainbow pattern | 100% brightness, forward |
| `RgbFade` | Fade between RGB colors | 70% brightness |
| `SingleFade` | Fade in/out single color | Purple color (50, 2, 200) |
| `Strobe` | Rapid flashing | Blue color |
| `Twinkle` | Random twinkling | Green color, 100% max LEDs |
| `TwinkleOff` | Twinkle then all off | Purple/blue color, 76% max LEDs |
| `SetAll` | Solid color (use with setColor) | No animation |
| `Empty` | Clear all animations | Clears all 8 animation slots |

**Behavior:**
- Clears previous animations before setting new one
- Each animation uses a specific slot (0-8)
- Animations run automatically (Phoenix 6)

---

### clearAllAnims()

Clears all animation slots using EmptyAnimation.

```java
leds.clearAllAnims();  // Stop all animations
leds.setColor(Color.kBlack);  // Turn off LEDs
```

**Behavior:**
- Sends EmptyAnimation to all 8 slots
- Does NOT turn off LEDs (use setColor for that)
- Called automatically by setColor() and setAnimation()

---

### getCANdle()

Gets the underlying CANdle hardware device for advanced control.

```java
CANdle candle = leds.getCANdle();
// Use for advanced Phoenix 6 features
```

**Returns:** The CANdle hardware device

**Use Cases:**
- Custom animations not in AnimationTypes
- Direct StatusCode checking
- Advanced Phoenix 6 configuration

## Animation Types Enum

```java
public enum AnimationTypes {
    ColorFlow,      // Flowing gradient
    Fire,           // Flame effect
    Larson,         // Knight Rider bounce
    Rainbow,        // Rainbow cycle
    RgbFade,        // RGB fade
    SingleFade,     // Single color fade
    Strobe,         // Strobe flash
    Twinkle,        // Random twinkle
    TwinkleOff,     // Twinkle then off
    SetAll,         // Solid color
    Empty           // Clear animations
}
```

## Hardware Details

### CANdle Specifications

**Supported LED Types:**
- WS2812B (NeoPixel-compatible)
- Configurable strip types: GRB, RGB, BRG, GRBW, RGBW, BRGW

**LED Capacity:**
- 8 onboard LEDs (indices 0-7)
- Up to 392 external LEDs (indices 8-399)
- Total: 400 LEDs maximum

**Power Requirements:**
- 5V rail for LED control
- VBat output for high-power strips (optional)
- Current limit considerations for long strips

**Connection:**
- CAN bus (roboRIO or CANivore)
- 3-pin LED connector (GND, 5V, Data)

### Wiring

**CANdle to RoboRIO:**
- CAN High (Yellow) → CAN H
- CAN Low (Green) → CAN L
- Power (12V) → Robot power distribution

**CANdle to LED Strip:**
- GND → LED Strip GND
- 5V → LED Strip 5V (for signal, not power)
- Data Out → LED Strip Data In

**External Power:**
For strips >50 LEDs, use external 5V power supply:
- Connect strip 5V to external supply
- Share GND between CANdle and supply
- Data signal only from CANdle

## Configuration

### Constants Setup

Define LED constants in `Constants.java`:

```java
public static final class LEDConstants {
    public static final int LEDS_IN_STRIP = 100;
    public static final StripTypeValue LED_STRIP_TYPE = StripTypeValue.GRB;

    // Color definitions
    public static final Color off = new Color(0, 0, 0);
    public static final Color yellow = new Color(255, 255, 0);
    public static final Color blue = new Color(0, 0, 255);
    public static final Color green = new Color(0, 255, 0);
    public static final Color red = new Color(255, 0, 0);
}
```

### Strip Type Selection

Choose based on your LED strip model:

| Strip Type | When to Use |
|------------|-------------|
| `GRB` | BTF-Lighting, most WS2812B strips |
| `RGB` | Some generic LED strips |
| `BRG` | Uncommon, check datasheet |
| `GRBW` | RGBW strips (4-wire) with GRB order |
| `RGBW` | RGBW strips with RGB order |
| `BRGW` | RGBW strips with BRG order |

**Testing:** If colors appear wrong, try different strip types.

## Usage Examples

### Basic Color Control

```java
public class RobotContainer {
  private final CANdleSubsystem m_leds = new CANdleSubsystem(10);

  public RobotContainer() {
    // Default to alliance color
    m_leds.setDefaultCommand(Commands.run(() ->
      m_leds.setColor(
        DriverStation.getAlliance()
          .map(a -> a == Alliance.Red ? LEDConstants.red : LEDConstants.blue)
          .orElse(LEDConstants.white)
      ), m_leds
    ));
  }
}
```

### State-Based LEDs

```java
// Show status based on robot state
public void configureBindings() {
  // Green when game piece acquired
  m_intake.hasGamePieceTrigger()
    .onTrue(Commands.runOnce(() ->
      m_leds.setColor(LEDConstants.green), m_leds));

  // Red when overheating
  new Trigger(() -> m_shooter.getTemperature() > 50)
    .whileTrue(Commands.run(() ->
      m_leds.setAnimation(AnimationTypes.Strobe), m_leds));

  // Yellow during autonomous
  new Trigger(DriverStation::isAutonomous)
    .whileTrue(Commands.run(() ->
      m_leds.setColor(LEDConstants.yellow), m_leds));
}
```

### Animation Sequences

```java
// Celebrate scoring
public Command celebrateCommand() {
  return Commands.sequence(
    Commands.runOnce(() ->
      m_leds.setAnimation(AnimationTypes.Rainbow), m_leds),
    Commands.waitSeconds(2.0),
    Commands.runOnce(() ->
      m_leds.setColor(LEDConstants.green), m_leds)
  );
}

// Warning pattern
public Command warningPattern() {
  return Commands.repeatingSequence(
    Commands.runOnce(() -> m_leds.setColor(LEDConstants.red), m_leds),
    Commands.waitSeconds(0.5),
    Commands.runOnce(() -> m_leds.setColor(LEDConstants.off), m_leds),
    Commands.waitSeconds(0.5)
  ).withTimeout(3.0);
}
```

### Zone-Based Control

```java
// Different colors for different zones
public void setZoneLEDs() {
  // First 20 LEDs: intake status (green/off)
  m_leds.setLEDs(
    m_intake.hasGamePiece() ? 0 : 0,
    m_intake.hasGamePiece() ? 255 : 0,
    0,
    8,   // Start at first external LED
    20   // 20 LEDs
  );

  // Next 20 LEDs: shooter status (red when ready)
  m_leds.setLEDs(
    m_shooter.atSetpoint() ? 255 : 50,
    0,
    0,
    28,  // Start after intake zone
    20
  );

  // Remaining LEDs: alliance color
  Color allianceColor = DriverStation.getAlliance()
    .map(a -> a == Alliance.Red ? LEDConstants.red : LEDConstants.blue)
    .orElse(LEDConstants.white);

  m_leds.setLEDs(
    (int)(allianceColor.red * 255),
    (int)(allianceColor.green * 255),
    (int)(allianceColor.blue * 255),
    48,  // Start after shooter zone
    60   // Rest of strip
  );
}
```

### Command Factory Pattern

```java
public class LEDCommands {
  // Solid color command
  public static Command solidColor(CANdleSubsystem leds, Color color) {
    return Commands.runOnce(() -> leds.setColor(color), leds);
  }

  // Timed animation
  public static Command animation(
      CANdleSubsystem leds,
      AnimationTypes type,
      double seconds) {
    return Commands.runOnce(() -> leds.setAnimation(type), leds)
      .andThen(Commands.waitSeconds(seconds));
  }

  // Blink pattern
  public static Command blink(
      CANdleSubsystem leds,
      Color color,
      int times) {
    Command blinkOnce = Commands.sequence(
      Commands.runOnce(() -> leds.setColor(color), leds),
      Commands.waitSeconds(0.3),
      Commands.runOnce(() -> leds.setColor(LEDConstants.off), leds),
      Commands.waitSeconds(0.3)
    );
    return blinkOnce.repeatedly().withTimeout(times * 0.6);
  }

  // Gradient based on sensor value
  public static Command gradientFromSensor(
      CANdleSubsystem leds,
      DoubleSupplier sensorValue,
      double minValue,
      double maxValue) {
    return Commands.run(() -> {
      double value = sensorValue.getAsDouble();
      double normalized = (value - minValue) / (maxValue - minValue);
      normalized = MathUtil.clamp(normalized, 0.0, 1.0);

      // Green (0) to Red (1)
      int red = (int)(normalized * 255);
      int green = (int)((1.0 - normalized) * 255);

      leds.setColor(red, green, 0);
    }, leds);
  }
}
```

## Phoenix 6 Features

### Control Request Pattern

Phoenix 6 uses a modern control request pattern:

```java
// Solid color
var solidColor = new SolidColor(8, 107);  // LED range
solidColor.Color = new RGBWColor(255, 0, 0, 0);  // RGBW
candle.setControl(solidColor);

// Animation
var rainbow = new RainbowAnimation(8, 107);
rainbow.Brightness = 1.0;
rainbow.FrameRate = 70;
rainbow.Slot = 0;
candle.setControl(rainbow);
```

### Animation Configuration

All animations support:
- `Slot`: Animation slot (0-7)
- `LEDStartIndex`: First LED in range
- `LEDEndIndex`: Last LED in range
- `FrameRate`: Speed in Hz (2-1000)

Animation-specific properties:
- `Color`: RGBWColor for color-based animations
- `Brightness`: 0.0-1.0 for brightness-based animations
- `Direction`: Forward/Backward for directional animations
- `Size`: LED count for pocket animations (Larson)

### StatusCode Checking

Check operation success:

```java
CANdle candle = leds.getCANdle();
StatusCode status = candle.setControl(animation);

if (status != StatusCode.OK) {
  DriverStation.reportWarning(
    "CANdle animation failed: " + status, false);
}
```

## Troubleshooting

### Colors Appear Wrong

**Problem:** LEDs show incorrect colors (e.g., red when expecting green)

**Solution:**
1. Check strip type in Constants: Try `GRB`, `RGB`, or `BRG`
2. Verify strip datasheet for color order
3. Test with `setColor(255, 0, 0)` - should be red

### LEDs Don't Light Up

**Problem:** No LED output

**Checklist:**
1. Verify CAN connection (use Phoenix Tuner)
2. Check 5V rail enabled: `config.CANdleFeatures.Enable5VRail = Enabled`
3. Verify LED strip power connection
4. Test with solid color first (not animation)
5. Check LED strip data pin connection

### Animations Choppy/Slow

**Problem:** Animations run at wrong speed

**Solution:**
1. Adjust FrameRate property (default values in setAnimation())
2. Check CAN bus utilization (reduce traffic)
3. Ensure periodic() is being called
4. Verify Phoenix 6 firmware version

### Strip Partially Lights

**Problem:** Only first N LEDs work

**Solutions:**
1. Check `LEDS_IN_STRIP` constant matches physical strip
2. Verify power supply capacity for full strip
3. Check data signal integrity (long runs may need buffer)
4. Test smaller range to isolate power vs. signal issue

### CAN ID Conflicts

**Problem:** CANdle not responding

**Solution:**
1. Use Phoenix Tuner to scan CAN bus
2. Verify CAN ID doesn't conflict with other devices
3. Check CAN termination if on long bus
4. Validate CAN High/Low wiring

## Performance Considerations

### Update Rate

- Animations run automatically (no periodic() calls needed)
- Solid color changes are instant
- Command scheduling overhead minimal

### CAN Bus Load

- Animations generate periodic CAN traffic
- Multiple simultaneous animations increase load
- Use fewer animation slots for lower bandwidth

### CPU Usage

- Subsystem is lightweight (no heavy computations)
- Safe for high-frequency command switching
- Phoenix 6 handles animation generation

## Migration from Phoenix 5

If migrating from Phoenix 5 CANdle code:

**API Changes:**
- `animate()` → `setControl()`
- Nested animation classes → Separate classes in `controls` package
- Configuration objects now nested (`config.LED.StripType`)
- Animation constructors take LED range parameters

**Behavior Changes:**
- Animations start immediately (no `periodic()` needed)
- Slots managed automatically
- StatusCode returns from all operations

**Dependencies:**
- Phoenix 6 required (26.1.0+)
- Phoenix 5 may coexist (for YAGSL/other libs)

## Command Factories & Triggers

CANdleSubsystem includes built-in **triggers** and **command factories** following WPILib command-based best practices:

### Triggers (State Exposure)
- `isShowingColorTrigger(Color)` - True when showing specific color
- `isOffTrigger()` - True when LEDs are off
- `isAnimatingTrigger()` - True when animation running
- `isShowingAnimationTrigger(AnimationType)` - True for specific animation

### Command Factories
**Basic:** `setColorCommand()`, `turnOffCommand()`, `allianceColorCommand()`, `setAnimationCommand()`

**Patterns:** `blinkCommand()`, `pulseCommand()`, `strobeCommand()`

**Status:** `celebrateCommand()`, `warningCommand()`, `errorCommand()`, `readyCommand()`, `busyCommand()`, `disabledCommand()`

**For detailed usage examples and patterns, see [Command Factory Guide](CANdleSubsystem-CommandFactory.md)**

## See Also

- [Command Factory & Triggers Guide](CANdleSubsystem-CommandFactory.md) - **Detailed usage examples**
- [Subsystems Overview](README.md)
- [Phoenix 6 Documentation](https://pro.docs.ctr-electronics.com/en/latest/docs/api-reference/api-usage/index.html)
- [CANdle Hardware Guide](https://pro.docs.ctr-electronics.com/en/latest/docs/hardware-reference/candle/index.html)
- [Command Best Practices](../../COMMAND_BEST_PRACTICES.md)
- [WPILib Color Class](https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/wpilibj/util/Color.html)

---

**Phoenix 6 Migration:** This subsystem has been fully migrated to Phoenix 6 and follows modern CTRE best practices. No deprecated Phoenix 5 APIs are used.
