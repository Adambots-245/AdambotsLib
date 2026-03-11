// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.lib.subsystems;

import com.adambots.lib.Constants.LEDConstants;
import com.adambots.lib.utils.Utils;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.signals.VBatOutputModeValue;
import com.ctre.phoenix6.signals.AnimationDirectionValue;
import com.ctre.phoenix6.signals.LarsonBounceValue;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.signals.StatusLedWhenActiveValue;
import com.ctre.phoenix6.signals.Enable5VRailValue;
import com.ctre.phoenix6.controls.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.util.function.DoubleSupplier;

/**
 * CANdle LED subsystem using Phoenix 6 API.
 *
 * <p>Controls WS2812B-compatible LED strips via CTRE CANdle device.
 * Supports solid colors and animations for robot status indication.
 */
public class CANdleSubsystem extends SubsystemBase {
  private static final Color ADAMBOTS_YELLOW = new Color(255, 216, 2);
  private static final int LED_START_INDEX = 8;  // Onboard LEDs are 0-7, external strip starts at 8

  private final CANdle candle;
  private final int ledsInStrip;

  // State tracking for triggers
  private Color currentColor;
  private AnimationTypes currentAnimation = AnimationTypes.SetAll;
  private boolean isAnimating = false;

  /**
   * Creates a CANdle LED subsystem on the default CAN bus.
   *
   * @param canID CAN ID for the CANdle (0-62)
   * @param ledsInStrip Number of LEDs in the external strip
   */
  public CANdleSubsystem(int canID, int ledsInStrip) {
    this(canID, ledsInStrip, false);
  }

  /**
   * Creates a CANdle LED subsystem.
   *
   * @param canID CAN ID for the CANdle (0-62)
   * @param ledsInStrip Number of LEDs in the external strip
   * @param isOnCANivore True if the CANdle is on a CANivore bus
   */
  public CANdleSubsystem(int canID, int ledsInStrip, boolean isOnCANivore) {
    this(canID, ledsInStrip, isOnCANivore, ADAMBOTS_YELLOW);
  }

  /**
   * Creates a CANdle LED subsystem with full configuration.
   *
   * @param canID CAN ID for the CANdle (0-62)
   * @param ledsInStrip Number of LEDs in the external strip
   * @param isOnCANivore True if the CANdle is on a CANivore bus
   * @param defaultColor Default color to set on startup
   */
  public CANdleSubsystem(int canID, int ledsInStrip, boolean isOnCANivore, Color defaultColor) {
    // Validate CAN ID
    if (canID < 0 || canID > 62) {
      Utils.reportError("CANdleSubsystem: Invalid CAN ID " + canID +
          ". Valid range: 0-62. Defaulting to 1.");
      canID = 1;
    }

    this.ledsInStrip = ledsInStrip;
    this.currentColor = defaultColor;

    this.candle = isOnCANivore
        ? new CANdle(canID, new CANBus("*"))
        : new CANdle(canID);
    configureCANdle();

    // Set default color
    setColor(defaultColor);
  }

  private void configureCANdle() {
    CANdleConfiguration config = new CANdleConfiguration();

    // Configure LED strip settings
    config.LED.StripType = LEDConstants.LED_STRIP_TYPE;
    config.LED.BrightnessScalar = 1.0;

    // Configure CANdle features
    config.CANdleFeatures.StatusLedWhenActive = StatusLedWhenActiveValue.Disabled;
    config.CANdleFeatures.Enable5VRail = Enable5VRailValue.Enabled;
    config.CANdleFeatures.VBatOutputMode = VBatOutputModeValue.Off;

    // Apply configuration
    candle.getConfigurator().apply(config);
  }

  @Override
  public void periodic() {
    // Phoenix 6 animations run automatically after being set
    // No need to call animate() in periodic
  }

  /**
   * Animation types supported by CANdle.
   */
  public enum AnimationTypes {
    ColorFlow,
    Fire,
    Larson,
    Rainbow,
    RgbFade,
    SingleFade,
    Strobe,
    Twinkle,
    TwinkleOff,
    SetAll,
    Empty
  }

  /**
   * Set the color for all LEDs using Color object.
   *
   * @param color WPILib Color object (RGB values 0.0-1.0)
   */
  public void setColor(Color color) {
    setColor((int)(color.red*255), (int)(color.green*255), (int)(color.blue*255));
    this.currentColor = color;
    this.isAnimating = false;
    this.currentAnimation = AnimationTypes.SetAll;
  }

  /**
   * Set the colors for the LED Strip.
   *
   * @param r Red (0 to 255)
   * @param g Green (0 to 255)
   * @param b Blue (0 to 255)
   */
  public void setColor(int r, int g, int b) {
    // Clear all animations first
    clearAllAnims();

    // Clamp values to valid range
    r = MathUtil.clamp(r, 0, 255);
    g = MathUtil.clamp(g, 0, 255);
    b = MathUtil.clamp(b, 0, 255);

    // Phoenix 6: Use SolidColor control request
    var solidColor = new SolidColor(LED_START_INDEX, ledEndIndex());
    solidColor.Color = new RGBWColor(r, g, b, 0);
    candle.setControl(solidColor);

    // Update state
    this.currentColor = new Color(r / 255.0, g / 255.0, b / 255.0);
    this.isAnimating = false;
    this.currentAnimation = AnimationTypes.SetAll;
  }

  /**
   * Set an individual LED or range of LEDs to a specified color.
   * Indices are 0-based from the start of the external strip.
   *
   * @param r Red (0 to 255)
   * @param g Green (0 to 255)
   * @param b Blue (0 to 255)
   * @param startIdx 0-based index from the start of the external strip
   * @param numOfLEDs Number of LEDs to light up with this color
   */
  public void setLEDs(int r, int g, int b, int startIdx, int numOfLEDs){
    // Clamp color values
    r = MathUtil.clamp(r, 0, 255);
    g = MathUtil.clamp(g, 0, 255);
    b = MathUtil.clamp(b, 0, 255);

    // Validate index and count
    if (startIdx < 0 || startIdx > ledsInStrip)
      startIdx = 0;

    if (numOfLEDs < 0 || numOfLEDs > ledsInStrip)
      numOfLEDs = ledsInStrip;

    setLEDRange(new RGBWColor(r, g, b, 0), startIdx, numOfLEDs);
  }

  /**
   * Set a range of LEDs on the external strip to a color.
   * Handles the internal offset from strip-relative to CANdle-absolute indices.
   *
   * @param color RGBWColor to set
   * @param stripStart 0-based index from the start of the external strip
   * @param count Number of LEDs to set
   */
  private void setLEDRange(RGBWColor color, int stripStart, int count) {
    if (count <= 0) return;
    int absStart = LED_START_INDEX + stripStart;
    int absEnd = absStart + count - 1;
    var solidColor = new SolidColor(absStart, absEnd);
    solidColor.Color = color;
    candle.setControl(solidColor);
  }

  /**
   * Set the active animation pattern.
   *
   * @param toChange Animation type to activate
   */
  public void setAnimation(AnimationTypes toChange) {
    // Clear previous animations
    clearAllAnims();

    // Update state
    this.currentAnimation = toChange;
    this.isAnimating = (toChange != AnimationTypes.SetAll && toChange != AnimationTypes.Empty);

    switch (toChange) {
      case ColorFlow -> {
        var anim = new ColorFlowAnimation(LED_START_INDEX, ledEndIndex());
        anim.Color = new RGBWColor(255, 150, 0, 0);
        anim.Slot = 0;
        anim.Direction = AnimationDirectionValue.Forward;
        anim.FrameRate = 50;
        candle.setControl(anim);
      }

      case Fire -> {
        var anim = new FireAnimation(LED_START_INDEX, ledEndIndex());
        anim.Brightness = 0.5;
        anim.Slot = 1;
        anim.Sparking = 0.8;
        anim.Cooling = 0.5;
        anim.FrameRate = 35;
        candle.setControl(anim);
      }

      case Larson -> {
        var anim = new LarsonAnimation(LED_START_INDEX, ledEndIndex());
        anim.Color = new RGBWColor(255, 150, 0, 0);
        anim.Slot = 2;
        anim.Size = 30;
        anim.BounceMode = LarsonBounceValue.Front;
        anim.FrameRate = 25;
        candle.setControl(anim);
      }

      case Rainbow -> {
        var anim = new RainbowAnimation(LED_START_INDEX, ledEndIndex());
        anim.Brightness = 1.0;
        anim.Slot = 3;
        anim.Direction = AnimationDirectionValue.Forward;
        anim.FrameRate = 70;
        candle.setControl(anim);
      }

      case RgbFade -> {
        var anim = new RgbFadeAnimation(LED_START_INDEX, ledEndIndex());
        anim.Brightness = 0.7;
        anim.Slot = 4;
        anim.FrameRate = 40;
        candle.setControl(anim);
      }

      case SingleFade -> {
        var anim = new SingleFadeAnimation(LED_START_INDEX, ledEndIndex());
        anim.Color = new RGBWColor(50, 2, 200, 0);
        anim.Slot = 5;
        anim.FrameRate = 50;
        candle.setControl(anim);
      }

      case Strobe -> {
        var anim = new StrobeAnimation(LED_START_INDEX, ledEndIndex());
        anim.Color = new RGBWColor(0, 0, 255, 0);  // Blue
        anim.Slot = 6;
        anim.FrameRate = 50;
        candle.setControl(anim);
      }

      case Twinkle -> {
        var anim = new TwinkleAnimation(LED_START_INDEX, ledEndIndex());
        anim.Color = new RGBWColor(0, 255, 0, 0);  // Green
        anim.Slot = 7;
        anim.MaxLEDsOnProportion = 1.0;  // 100% of LEDs can be on
        anim.FrameRate = 50;
        candle.setControl(anim);
      }

      case TwinkleOff -> {
        var anim = new TwinkleOffAnimation(LED_START_INDEX, ledEndIndex());
        anim.Color = new RGBWColor(70, 90, 175, 0);
        anim.Slot = 8;
        anim.MaxLEDsOnProportion = 0.76;  // 76% of LEDs can be on
        anim.FrameRate = 50;
        candle.setControl(anim);
      }

      case Empty -> {
        // Use EmptyAnimation to clear
        for (int slot = 0; slot < 8; slot++) {
          var emptyAnim = new EmptyAnimation(slot);
          candle.setControl(emptyAnim);
        }
      }

      case SetAll -> {
        // No animation, solid color is handled by setColor()
      }

      default -> {
        // Default to Rainbow
        var anim = new RainbowAnimation(LED_START_INDEX, ledEndIndex());
        anim.Brightness = 1.0;
        anim.Slot = 0;
        anim.Direction = AnimationDirectionValue.Forward;
        anim.FrameRate = 70;
        candle.setControl(anim);
      }
    }
  }

  /**
   * Clear all animation slots using EmptyAnimation.
   */
  public void clearAllAnims() {
    for (int slot = 0; slot < 8; slot++) {
      var emptyAnim = new EmptyAnimation(slot);
      candle.setControl(emptyAnim);
    }
  }

  /**
   * Get the CANdle device for advanced configuration.
   *
   * @return The CANdle hardware device
   */
  public CANdle getCANdle() {
    return candle;
  }

  // ===== State Exposure (Triggers) =====

  /**
   * Gets the current LED color.
   *
   * @return Current color (may not reflect actual state if animation is running)
   */
  public Color getCurrentColor() {
    return currentColor;
  }

  /**
   * Gets the current animation type.
   *
   * @return Current animation type
   */
  public AnimationTypes getCurrentAnimation() {
    return currentAnimation;
  }

  /**
   * Trigger that is true when LEDs are showing a specific color.
   *
   * @param color Color to check for
   * @return Trigger for color matching
   */
  public Trigger isShowingColorTrigger(Color color) {
    return new Trigger(() ->
      !isAnimating &&
      Math.abs(currentColor.red - color.red) < 0.01 &&
      Math.abs(currentColor.green - color.green) < 0.01 &&
      Math.abs(currentColor.blue - color.blue) < 0.01
    );
  }

  /**
   * Trigger that is true when LEDs are off (black).
   *
   * @return Trigger for LEDs off state
   */
  public Trigger isOffTrigger() {
    return isShowingColorTrigger(LEDConstants.off);
  }

  /**
   * Trigger that is true when an animation is running.
   *
   * @return Trigger for animation state
   */
  public Trigger isAnimatingTrigger() {
    return new Trigger(() -> isAnimating);
  }

  /**
   * Trigger that is true when a specific animation is running.
   *
   * @param animation Animation type to check for
   * @return Trigger for specific animation
   */
  public Trigger isShowingAnimationTrigger(AnimationTypes animation) {
    return new Trigger(() -> currentAnimation == animation && isAnimating);
  }

  // ===== Command Factories =====

  /**
   * Command to set LEDs to a solid color.
   *
   * @param color Color to set
   * @return Command that sets the color
   */
  public Command setColorCommand(Color color) {
    return Commands.runOnce(() -> setColor(color), this)
      .withName("SetColor(" + colorToString(color) + ")");
  }

  /**
   * Command to turn off LEDs (set to black).
   *
   * @return Command that turns off LEDs
   */
  public Command turnOffCommand() {
    return setColorCommand(LEDConstants.off)
      .withName("TurnOffLEDs");
  }

  /**
   * Command to set LEDs to alliance color.
   *
   * @return Command that sets alliance color (red or blue)
   */
  public Command allianceColorCommand() {
    return Commands.runOnce(() -> {
      Color allianceColor = DriverStation.getAlliance()
        .map(a -> a == Alliance.Red ? LEDConstants.red : LEDConstants.blue)
        .orElse(LEDConstants.white);
      setColor(allianceColor);
    }, this).withName("AllianceColor");
  }

  /**
   * Command to set an animation.
   *
   * @param animation Animation type to run
   * @return Command that starts the animation
   */
  public Command setAnimationCommand(AnimationTypes animation) {
    return Commands.runOnce(() -> setAnimation(animation), this)
      .withName("SetAnimation(" + animation + ")");
  }

  /**
   * Command to blink a color for a specified number of times.
   *
   * @param color Color to blink
   * @param times Number of times to blink
   * @return Command that blinks the color
   */
  public Command blinkCommand(Color color, int times) {
    return Commands.sequence(
      Commands.runOnce(() -> setColor(color), this),
      Commands.waitSeconds(0.3),
      Commands.runOnce(() -> setColor(LEDConstants.off), this),
      Commands.waitSeconds(0.3)
    ).repeatedly()
     .withTimeout(times * 0.6)
     .finallyDo(() -> setColor(LEDConstants.off))
     .withName("Blink(" + colorToString(color) + "x" + times + ")");
  }

  /**
   * Command to pulse a color for a specified duration.
   * Uses SingleFade animation for smooth pulsing effect.
   *
   * @param color Color to pulse
   * @param seconds Duration in seconds
   * @return Command that pulses the color
   */
  public Command pulseCommand(Color color, double seconds) {
    return Commands.sequence(
      Commands.runOnce(() -> {
        // Temporarily set animation with custom color
        clearAllAnims();
        var anim = new SingleFadeAnimation(LED_START_INDEX, ledEndIndex());
        anim.Color = new RGBWColor(
          (int)(color.red * 255),
          (int)(color.green * 255),
          (int)(color.blue * 255),
          0
        );
        anim.Slot = 5;
        anim.FrameRate = 50;
        candle.setControl(anim);
        currentAnimation = AnimationTypes.SingleFade;
        isAnimating = true;
      }, this),
      Commands.waitSeconds(seconds),
      Commands.runOnce(() -> setColor(LEDConstants.off), this)
    ).withName("Pulse(" + colorToString(color) + ")");
  }

  /**
   * Command to strobe a color for a specified duration.
   *
   * @param color Color to strobe
   * @param seconds Duration in seconds
   * @return Command that strobes the color
   */
  public Command strobeCommand(Color color, double seconds) {
    return Commands.sequence(
      Commands.runOnce(() -> {
        clearAllAnims();
        var anim = new StrobeAnimation(LED_START_INDEX, ledEndIndex());
        anim.Color = new RGBWColor(
          (int)(color.red * 255),
          (int)(color.green * 255),
          (int)(color.blue * 255),
          0
        );
        anim.Slot = 6;
        anim.FrameRate = 50;
        candle.setControl(anim);
        currentAnimation = AnimationTypes.Strobe;
        isAnimating = true;
      }, this),
      Commands.waitSeconds(seconds),
      Commands.runOnce(() -> setColor(LEDConstants.off), this)
    ).withName("Strobe(" + colorToString(color) + ")");
  }

  /**
   * Command sequence for celebration (rainbow animation followed by green).
   *
   * @return Command that runs celebration sequence
   */
  public Command celebrateCommand() {
    return Commands.sequence(
      setAnimationCommand(AnimationTypes.Rainbow),
      Commands.waitSeconds(2.0),
      setColorCommand(LEDConstants.green)
    ).withName("Celebrate");
  }

  /**
   * Command sequence for warning pattern (red blinking).
   *
   * @return Command that runs warning pattern
   */
  public Command warningCommand() {
    return blinkCommand(LEDConstants.red, 5)
      .withName("Warning");
  }

  /**
   * Command sequence for error indication (strobe red).
   *
   * @return Command that runs error indication
   */
  public Command errorCommand() {
    return strobeCommand(LEDConstants.red, 1.5)
      .withName("Error");
  }

  /**
   * Command to show ready status (solid green).
   *
   * @return Command that shows ready status
   */
  public Command readyCommand() {
    return setColorCommand(LEDConstants.green)
      .withName("Ready");
  }

  /**
   * Command to show busy status (orange larson animation).
   *
   * @return Command that shows busy status
   */
  public Command busyCommand() {
    return setAnimationCommand(AnimationTypes.Larson)
      .withName("Busy");
  }

  /**
   * Command to show disabled status (dim yellow).
   *
   * @return Command that shows disabled status
   */
  public Command disabledCommand() {
    return setColorCommand(new Color(0.3, 0.3, 0))
      .withName("Disabled");
  }

  // ===== Progress Bar =====

  /**
   * Command that fills LEDs proportionally based on a progress value.
   * Useful for countdown timers, charge indicators, etc.
   *
   * @param color Color for the filled portion
   * @param progress Supplier returning 0.0 (empty) to 1.0 (full)
   * @return Command that continuously updates the fill level
   */
  public Command progressCommand(Color color, DoubleSupplier progress) {
    return progressCommand(color, 0, ledsInStrip, progress);
  }

  /**
   * Command that fills LEDs proportionally based on a progress value.
   * Useful for countdown timers, charge indicators, etc.
   *
   * @param color Color for the filled portion
   * @param startIdx First LED index in the range (0-based from strip start)
   * @param numLEDs Total LEDs in the range
   * @param progress Supplier returning 0.0 (empty) to 1.0 (full)
   * @return Command that continuously updates the fill level
   */
  public Command progressCommand(Color color, int startIdx, int numLEDs, DoubleSupplier progress) {
    RGBWColor onColor = new RGBWColor(
      (int)(color.red * 255),
      (int)(color.green * 255),
      (int)(color.blue * 255),
      0
    );
    RGBWColor offColor = new RGBWColor(0, 0, 0, 0);

    return run(() -> {
      double clamped = MathUtil.clamp(progress.getAsDouble(), 0.0, 1.0);
      int litCount = (int)(clamped * numLEDs);
      if (litCount > 0) {
        setLEDRange(onColor, startIdx, litCount);
      }
      int remaining = numLEDs - litCount;
      if (remaining > 0) {
        setLEDRange(offColor, startIdx + litCount, remaining);
      }
    }).withName("Progress(" + colorToString(color) + ")");
  }

  // ===== Helper Methods =====

  /**
   * Computes the last LED index (absolute) for the full strip.
   */
  private int ledEndIndex() {
    return LED_START_INDEX + ledsInStrip - 1;
  }

  /**
   * Converts Color to human-readable string for command names.
   *
   * @param color Color to convert
   * @return String representation
   */
  private String colorToString(Color color) {
    // Check for common colors
    if (color.equals(LEDConstants.red)) return "Red";
    if (color.equals(LEDConstants.green)) return "Green";
    if (color.equals(LEDConstants.blue)) return "Blue";
    if (color.equals(LEDConstants.yellow)) return "Yellow";
    if (color.equals(LEDConstants.orange)) return "Orange";
    if (color.equals(LEDConstants.purple)) return "Purple";
    if (color.equals(LEDConstants.white)) return "White";
    if (color.equals(LEDConstants.off)) return "Off";

    // Generic RGB string
    return String.format("RGB(%d,%d,%d)",
      (int)(color.red * 255),
      (int)(color.green * 255),
      (int)(color.blue * 255)
    );
  }
}
