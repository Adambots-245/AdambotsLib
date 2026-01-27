package com.adambots.lib.utils;

import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.ScheduledFuture;
import java.util.concurrent.TimeUnit;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * Centralized controller button and axis mapping for FRC robots.
 *
 * <p>This class provides static access to controller triggers for command binding.
 * Supports Xbox, PS5, and joystick controllers with configurable controller types.
 * Must be initialized in Robot.robotInit() before use.
 *
 * <p><strong>Features:</strong>
 * <ul>
 *   <li>Support for Xbox, PS5, and Extreme 3D Pro controllers</li>
 *   <li>Configurable controller types per port</li>
 *   <li>Comprehensive button/trigger mappings</li>
 *   <li>Input processing utilities (deadzone, curves)</li>
 *   <li>Controller rumble with automatic shutoff</li>
 * </ul>
 *
 * <p><strong>Usage Example:</strong>
 * <pre>{@code
 * // In Robot.robotInit():
 * Buttons.init(0, 1, ControllerType.XBOX, ControllerType.EXTREME_3D_PRO);
 *
 * // In RobotContainer:
 * Buttons.XboxAButton.onTrue(intakeCommand);
 * Buttons.XboxBButton.onTrue(shootCommand);
 * Buttons.JoystickButton1.whileTrue(aimCommand);
 *
 * // Or with PS5:
 * Buttons.init(0, 1, ControllerType.PS5, ControllerType.XBOX);
 * Buttons.PS5Cross.onTrue(intakeCommand);
 * }</pre>
 *
 * @see edu.wpi.first.wpilibj2.command.button.CommandXboxController
 * @see edu.wpi.first.wpilibj2.command.button.CommandPS5Controller
 * @see edu.wpi.first.wpilibj2.command.button.CommandJoystick
 */
public class Buttons {

    // ======================== CONSTANTS ========================

    /** Default threshold for analog triggers to register as button press. */
    public static final double DEFAULT_TRIGGER_THRESHOLD = 0.3;

    /** Default threshold for joystick stick positions to register as directional input. */
    public static final double DEFAULT_STICK_THRESHOLD = 0.8;

    /** Linear component weight for cubic input curve (must sum to 1.0 with CUBIC_CUBIC_COMPONENT). */
    public static final double CUBIC_LINEAR_COMPONENT = 0.1;

    /** Cubic component weight for cubic input curve (must sum to 1.0 with CUBIC_LINEAR_COMPONENT). */
    public static final double CUBIC_CUBIC_COMPONENT = 0.9;

    /** Steepness parameter for sigmoid smoothing curve. */
    public static final double SIGMOID_STEEPNESS = 2.0;

    /** Maximum allowed rumble duration in milliseconds (5 seconds). */
    public static final int MAX_RUMBLE_DURATION_MS = 5000;

    // ======================== CONTROLLER TYPES ========================

    /**
     * Supported controller types for initialization.
     */
    public enum ControllerType {
        /** Xbox controller (Xbox One or Xbox Series). */
        XBOX,
        /** PlayStation 5 DualSense controller. */
        PS5,
        /** Logitech Extreme 3D Pro joystick. */
        EXTREME_3D_PRO,
        /** No controller assigned to this port. */
        NONE
    }

    /**
     * Input curve types for joystick processing.
     */
    public enum InputCurve {
        /** No curve applied - direct linear mapping. */
        LINEAR,
        /** Cubic curve for precise low-speed control. */
        CUBIC,
        /** Sigmoid S-curve for smooth gradual response. */
        SIGMOID
    }

    // ======================== CONTROLLER INSTANCES ========================

    /** Driver controller (port 0). Type determined by initialization. */
    private static Object driverController;

    /** Operator controller (port 1). Type determined by initialization. */
    private static Object operatorController;

    /** Driver controller type. */
    private static ControllerType driverType = ControllerType.NONE;

    /** Operator controller type. */
    private static ControllerType operatorType = ControllerType.NONE;

    /** Tracks whether init() has been called. */
    private static boolean initialized = false;

    // ======================== XBOX CONTROLLER BUTTONS ========================

    /** Xbox controller Back button trigger. */
    public static Trigger XboxBackButton;

    /** Xbox controller Start button trigger. */
    public static Trigger XboxStartButton;

    /** Xbox controller X button trigger (typically used for secondary actions). */
    public static Trigger XboxXButton;

    /** Xbox controller Y button trigger (typically used for high/upper mechanisms). */
    public static Trigger XboxYButton;

    /** Xbox controller B button trigger (typically used for cancel or low actions). */
    public static Trigger XboxBButton;

    /** Xbox controller A button trigger (typically used for primary/confirm actions). */
    public static Trigger XboxAButton;

    /** Xbox controller left bumper trigger. */
    public static Trigger XboxLeftBumper;

    /** Xbox controller right bumper trigger. */
    public static Trigger XboxRightBumper;

    /** Xbox controller left analog stick click button trigger. */
    public static Trigger XboxLeftStickButton;

    /** Xbox controller right analog stick click button trigger. */
    public static Trigger XboxRightStickButton;

    // ======================== XBOX CONTROLLER TRIGGERS ========================

    /** Xbox controller left trigger as button (analog trigger > DEFAULT_TRIGGER_THRESHOLD). */
    public static Trigger XboxLeftTriggerButton;

    /** Xbox controller right trigger as button (analog trigger > DEFAULT_TRIGGER_THRESHOLD). */
    public static Trigger XboxRightTriggerButton;

    // ======================== XBOX CONTROLLER STICK DIRECTIONS ========================

    /** Xbox controller right stick pushed up trigger (Y axis < -DEFAULT_STICK_THRESHOLD). */
    public static Trigger XboxRightStickUp;

    /** Xbox controller right stick pushed down trigger (Y axis > DEFAULT_STICK_THRESHOLD). */
    public static Trigger XboxRightStickDown;

    /** Xbox controller left stick pushed up trigger (Y axis < -DEFAULT_STICK_THRESHOLD). */
    public static Trigger XboxLeftStickUp;

    /** Xbox controller left stick pushed down trigger (Y axis > DEFAULT_STICK_THRESHOLD). */
    public static Trigger XboxLeftStickDown;

    // ======================== XBOX CONTROLLER D-PAD ========================

    /** Xbox controller D-Pad North (0°) trigger. */
    public static Trigger XboxDPadN;

    /** Xbox controller D-Pad Northeast (45°) trigger. */
    public static Trigger XboxDPadNE;

    /** Xbox controller D-Pad East (90°) trigger. */
    public static Trigger XboxDPadE;

    /** Xbox controller D-Pad Southeast (135°) trigger. */
    public static Trigger XboxDPadSE;

    /** Xbox controller D-Pad South (180°) trigger. */
    public static Trigger XboxDPadS;

    /** Xbox controller D-Pad Southwest (225°) trigger. */
    public static Trigger XboxDPadSW;

    /** Xbox controller D-Pad West (270°) trigger. */
    public static Trigger XboxDPadW;

    /** Xbox controller D-Pad Northwest (315°) trigger. */
    public static Trigger XboxDPadNW;

    // ======================== PS5 CONTROLLER BUTTONS ========================

    /** PS5 controller Create button (equivalent to Xbox Back). */
    public static Trigger PS5Create;

    /** PS5 controller Options button (equivalent to Xbox Start). */
    public static Trigger PS5Options;

    /** PS5 controller Square button. */
    public static Trigger PS5Square;

    /** PS5 controller Triangle button. */
    public static Trigger PS5Triangle;

    /** PS5 controller Circle button. */
    public static Trigger PS5Circle;

    /** PS5 controller Cross button (X). */
    public static Trigger PS5Cross;

    /** PS5 controller L1 button (left bumper). */
    public static Trigger PS5L1;

    /** PS5 controller R1 button (right bumper). */
    public static Trigger PS5R1;

    /** PS5 controller L3 button (left stick click). */
    public static Trigger PS5L3;

    /** PS5 controller R3 button (right stick click). */
    public static Trigger PS5R3;

    /** PS5 controller PlayStation button. */
    public static Trigger PS5PS;

    /** PS5 controller touchpad button. */
    public static Trigger PS5Touchpad;

    // ======================== PS5 CONTROLLER TRIGGERS ========================

    /** PS5 controller L2 trigger as button (analog trigger > DEFAULT_TRIGGER_THRESHOLD). */
    public static Trigger PS5L2Button;

    /** PS5 controller R2 trigger as button (analog trigger > DEFAULT_TRIGGER_THRESHOLD). */
    public static Trigger PS5R2Button;

    // ======================== PS5 CONTROLLER STICK DIRECTIONS ========================

    /** PS5 controller right stick pushed up trigger. */
    public static Trigger PS5RightStickUp;

    /** PS5 controller right stick pushed down trigger. */
    public static Trigger PS5RightStickDown;

    /** PS5 controller left stick pushed up trigger. */
    public static Trigger PS5LeftStickUp;

    /** PS5 controller left stick pushed down trigger. */
    public static Trigger PS5LeftStickDown;

    // ======================== PS5 CONTROLLER D-PAD ========================

    /** PS5 controller D-Pad up trigger. */
    public static Trigger PS5DPadUp;

    /** PS5 controller D-Pad down trigger. */
    public static Trigger PS5DPadDown;

    /** PS5 controller D-Pad left trigger. */
    public static Trigger PS5DPadLeft;

    /** PS5 controller D-Pad right trigger. */
    public static Trigger PS5DPadRight;

    // ======================== JOYSTICK BUTTONS ========================

    /** Joystick trigger button (button 1 on Extreme 3D Pro). */
    public static Trigger JoystickButton1;

    /** Joystick thumb button (button 2 on Extreme 3D Pro). */
    public static Trigger JoystickButton2;

    /** Joystick button 3 (bottom left on Extreme 3D Pro base). */
    public static Trigger JoystickButton3;

    /** Joystick button 4 (bottom right on Extreme 3D Pro base). */
    public static Trigger JoystickButton4;

    /** Joystick button 5 (top left on Extreme 3D Pro base). */
    public static Trigger JoystickButton5;

    /** Joystick button 6 (top right on Extreme 3D Pro base). */
    public static Trigger JoystickButton6;

    /** Joystick button 7 (left cluster on Extreme 3D Pro base). */
    public static Trigger JoystickButton7;

    /** Joystick button 8 (left cluster on Extreme 3D Pro base). */
    public static Trigger JoystickButton8;

    /** Joystick button 9 (left cluster on Extreme 3D Pro base). */
    public static Trigger JoystickButton9;

    /** Joystick button 10 (right cluster on Extreme 3D Pro base). */
    public static Trigger JoystickButton10;

    /** Joystick button 11 (right cluster on Extreme 3D Pro base). */
    public static Trigger JoystickButton11;

    /** Joystick button 12 (right cluster on Extreme 3D Pro base). */
    public static Trigger JoystickButton12;

    // ======================== JOYSTICK POV HAT ========================

    /** Joystick POV hat up trigger. */
    public static Trigger JoystickPOVUp;

    /** Joystick POV hat down trigger. */
    public static Trigger JoystickPOVDown;

    /** Joystick POV hat left trigger. */
    public static Trigger JoystickPOVLeft;

    /** Joystick POV hat right trigger. */
    public static Trigger JoystickPOVRight;

    /** Joystick POV hat up-left diagonal trigger. */
    public static Trigger JoystickPOVUpLeft;

    /** Joystick POV hat up-right diagonal trigger. */
    public static Trigger JoystickPOVUpRight;

    /** Joystick POV hat down-left diagonal trigger. */
    public static Trigger JoystickPOVDownLeft;

    /** Joystick POV hat down-right diagonal trigger. */
    public static Trigger JoystickPOVDownRight;

    /** Joystick POV hat center (not pressed) trigger. */
    public static Trigger JoystickPOVCenter;

    // ======================== RUMBLE MANAGEMENT ========================

    /** Executor for scheduled rumble shutoff. */
    private static ScheduledExecutorService rumbleExecutor;

    /** Current rumble task for cancellation. */
    private static ScheduledFuture<?> currentRumbleTask;

    // ======================== INITIALIZATION ========================

    /**
     * Initializes the Buttons class with specified controller types.
     *
     * <p>This method must be called exactly once from Robot.robotInit() before
     * any triggers are used. Subsequent calls will be ignored with a warning.
     *
     * <p><strong>Common Configurations:</strong>
     * <pre>{@code
     * // Driver on Xbox, operator on joystick
     * Buttons.init(0, 1, ControllerType.XBOX, ControllerType.EXTREME_3D_PRO);
     *
     * // Driver on joystick, operator on Xbox
     * Buttons.init(0, 1, ControllerType.EXTREME_3D_PRO, ControllerType.XBOX);
     *
     * // Both Xbox controllers
     * Buttons.init(0, 1, ControllerType.XBOX, ControllerType.XBOX);
     *
     * // Driver on PS5, operator on Xbox
     * Buttons.init(0, 1, ControllerType.PS5, ControllerType.XBOX);
     *
     * // Only driver controller
     * Buttons.init(0, -1, ControllerType.XBOX, ControllerType.NONE);
     * }</pre>
     *
     * @param driverPort USB port number for driver controller (typically 0-5)
     * @param operatorPort USB port number for operator controller (typically 0-5, or -1 for none)
     * @param driverControllerType Type of controller for driver
     * @param operatorControllerType Type of controller for operator
     */
    public static void init(int driverPort, int operatorPort,
                           ControllerType driverControllerType,
                           ControllerType operatorControllerType) {
        if (initialized) {
            edu.wpi.first.wpilibj.DriverStation.reportWarning(
                "Buttons.init() called multiple times. Ignoring subsequent calls.", false);
            return;
        }

        driverType = driverControllerType;
        operatorType = operatorControllerType;

        // Initialize driver controller
        driverController = createController(driverPort, driverControllerType);

        // Initialize operator controller
        if (operatorPort >= 0 && operatorControllerType != ControllerType.NONE) {
            operatorController = createController(operatorPort, operatorControllerType);
        }

        // Set initialized flag before trigger initialization so getJoystick() works
        initialized = true;

        // Initialize triggers based on controller types
        initializeXboxTriggers();
        initializePS5Triggers();
        initializeJoystickTriggers();

        // Initialize rumble executor
        rumbleExecutor = Executors.newSingleThreadScheduledExecutor(r -> {
            Thread t = new Thread(r, "RumbleThread");
            t.setDaemon(true);
            return t;
        });
    }

    /**
     * Creates a controller instance based on type.
     *
     * @param port USB port number
     * @param type Controller type
     * @return Controller instance (CommandXboxController, CommandPS5Controller, or CommandJoystick)
     */
    private static Object createController(int port, ControllerType type) {
        return switch (type) {
            case XBOX -> new CommandXboxController(port);
            case PS5 -> new CommandPS5Controller(port);
            case EXTREME_3D_PRO -> new CommandJoystick(port);
            case NONE -> null;
        };
    }

    /** A trigger that never activates - used as fallback when controller unavailable. */
    private static final Trigger NEVER_TRIGGER = new Trigger(() -> false);

    /**
     * Initializes Xbox controller triggers.
     * If no Xbox controller is configured, initializes to "never trigger" to prevent NPE.
     */
    private static void initializeXboxTriggers() {
        CommandXboxController xbox = getXboxController();

        if (xbox == null) {
            // Log warning for debugging - helps diagnose controller initialization issues
            if (driverType == ControllerType.XBOX || operatorType == ControllerType.XBOX) {
                edu.wpi.first.wpilibj.DriverStation.reportWarning(
                    "Buttons: Xbox controller type configured but controller is null. " +
                    "driverType=" + driverType + ", operatorType=" + operatorType +
                    ", driverController=" + driverController + ", initialized=" + initialized, false);
            }
            // Initialize all Xbox triggers to "never" to prevent NullPointerException
            XboxBackButton = NEVER_TRIGGER;
            XboxStartButton = NEVER_TRIGGER;
            XboxXButton = NEVER_TRIGGER;
            XboxYButton = NEVER_TRIGGER;
            XboxBButton = NEVER_TRIGGER;
            XboxAButton = NEVER_TRIGGER;
            XboxLeftBumper = NEVER_TRIGGER;
            XboxRightBumper = NEVER_TRIGGER;
            XboxLeftStickButton = NEVER_TRIGGER;
            XboxRightStickButton = NEVER_TRIGGER;
            XboxLeftTriggerButton = NEVER_TRIGGER;
            XboxRightTriggerButton = NEVER_TRIGGER;
            XboxRightStickUp = NEVER_TRIGGER;
            XboxRightStickDown = NEVER_TRIGGER;
            XboxLeftStickUp = NEVER_TRIGGER;
            XboxLeftStickDown = NEVER_TRIGGER;
            XboxDPadN = NEVER_TRIGGER;
            XboxDPadNE = NEVER_TRIGGER;
            XboxDPadE = NEVER_TRIGGER;
            XboxDPadSE = NEVER_TRIGGER;
            XboxDPadS = NEVER_TRIGGER;
            XboxDPadSW = NEVER_TRIGGER;
            XboxDPadW = NEVER_TRIGGER;
            XboxDPadNW = NEVER_TRIGGER;
            return;
        }

        // Buttons
        XboxBackButton = xbox.back();
        XboxStartButton = xbox.start();
        XboxXButton = xbox.x();
        XboxYButton = xbox.y();
        XboxBButton = xbox.b();
        XboxAButton = xbox.a();
        XboxLeftBumper = xbox.leftBumper();
        XboxRightBumper = xbox.rightBumper();
        XboxLeftStickButton = xbox.leftStick();
        XboxRightStickButton = xbox.rightStick();

        // Analog triggers as buttons
        XboxLeftTriggerButton = xbox.leftTrigger(DEFAULT_TRIGGER_THRESHOLD);
        XboxRightTriggerButton = xbox.rightTrigger(DEFAULT_TRIGGER_THRESHOLD);

        // Stick directions
        XboxRightStickUp = new Trigger(() -> xbox.getRightY() < -DEFAULT_STICK_THRESHOLD);
        XboxRightStickDown = new Trigger(() -> xbox.getRightY() > DEFAULT_STICK_THRESHOLD);
        XboxLeftStickUp = new Trigger(() -> xbox.getLeftY() < -DEFAULT_STICK_THRESHOLD);
        XboxLeftStickDown = new Trigger(() -> xbox.getLeftY() > DEFAULT_STICK_THRESHOLD);

        // D-Pad
        XboxDPadN = xbox.pov(0);
        XboxDPadNE = xbox.pov(45);
        XboxDPadE = xbox.pov(90);
        XboxDPadSE = xbox.pov(135);
        XboxDPadS = xbox.pov(180);
        XboxDPadSW = xbox.pov(225);
        XboxDPadW = xbox.pov(270);
        XboxDPadNW = xbox.pov(315);
    }

    /**
     * Initializes PS5 controller triggers if a PS5 controller is present.
     */
    private static void initializePS5Triggers() {
        CommandPS5Controller ps5 = getPS5Controller();
        if (ps5 == null) return;

        // Buttons
        PS5Create = ps5.create();
        PS5Options = ps5.options();
        PS5Square = ps5.square();
        PS5Triangle = ps5.triangle();
        PS5Circle = ps5.circle();
        PS5Cross = ps5.cross();
        PS5L1 = ps5.L1();
        PS5R1 = ps5.R1();
        PS5L3 = ps5.L3();
        PS5R3 = ps5.R3();
        PS5PS = ps5.PS();
        PS5Touchpad = ps5.touchpad();

        // Analog triggers as buttons (PS5 L2/R2 methods don't accept threshold, use axis)
        PS5L2Button = new Trigger(() -> ps5.getL2Axis() > DEFAULT_TRIGGER_THRESHOLD);
        PS5R2Button = new Trigger(() -> ps5.getR2Axis() > DEFAULT_TRIGGER_THRESHOLD);

        // Stick directions
        PS5RightStickUp = new Trigger(() -> ps5.getRightY() < -DEFAULT_STICK_THRESHOLD);
        PS5RightStickDown = new Trigger(() -> ps5.getRightY() > DEFAULT_STICK_THRESHOLD);
        PS5LeftStickUp = new Trigger(() -> ps5.getLeftY() < -DEFAULT_STICK_THRESHOLD);
        PS5LeftStickDown = new Trigger(() -> ps5.getLeftY() > DEFAULT_STICK_THRESHOLD);

        // D-Pad
        PS5DPadUp = ps5.povUp();
        PS5DPadDown = ps5.povDown();
        PS5DPadLeft = ps5.povLeft();
        PS5DPadRight = ps5.povRight();
    }

    /**
     * Initializes joystick triggers if a joystick is present.
     */
    private static void initializeJoystickTriggers() {
        CommandJoystick joystick = getJoystick();
        if (joystick == null) return;

        // Buttons
        JoystickButton1 = joystick.button(1);
        JoystickButton2 = joystick.button(2);
        JoystickButton3 = joystick.button(3);
        JoystickButton4 = joystick.button(4);
        JoystickButton5 = joystick.button(5);
        JoystickButton6 = joystick.button(6);
        JoystickButton7 = joystick.button(7);
        JoystickButton8 = joystick.button(8);
        JoystickButton9 = joystick.button(9);
        JoystickButton10 = joystick.button(10);
        JoystickButton11 = joystick.button(11);
        JoystickButton12 = joystick.button(12);

        // POV Hat
        JoystickPOVUp = joystick.povUp();
        JoystickPOVDown = joystick.povDown();
        JoystickPOVLeft = joystick.povLeft();
        JoystickPOVRight = joystick.povRight();
        JoystickPOVUpLeft = joystick.povUpLeft();
        JoystickPOVUpRight = joystick.povUpRight();
        JoystickPOVDownLeft = joystick.povDownLeft();
        JoystickPOVDownRight = joystick.povDownRight();
        JoystickPOVCenter = joystick.povCenter();
    }

    // ======================== CONTROLLER GETTERS ========================

    /**
     * Ensures that init() has been called before accessing controllers.
     *
     * @throws IllegalStateException if init() has not been called
     */
    private static void ensureInitialized() {
        if (!initialized) {
            throw new IllegalStateException("Buttons class not initialized. Call Buttons.init() first.");
        }
    }

    /**
     * Gets the Xbox controller instance if one is configured.
     *
     * @return CommandXboxController instance, or null if no Xbox controller configured
     */
    public static CommandXboxController getXboxController() {
        if (!initialized) return null;

        if (driverType == ControllerType.XBOX) {
            return (CommandXboxController) driverController;
        } else if (operatorType == ControllerType.XBOX) {
            return (CommandXboxController) operatorController;
        }
        return null;
    }

    /**
     * Gets the PS5 controller instance if one is configured.
     *
     * @return CommandPS5Controller instance, or null if no PS5 controller configured
     */
    public static CommandPS5Controller getPS5Controller() {
        if (!initialized) return null;

        if (driverType == ControllerType.PS5) {
            return (CommandPS5Controller) driverController;
        } else if (operatorType == ControllerType.PS5) {
            return (CommandPS5Controller) operatorController;
        }
        return null;
    }

    /**
     * Gets the joystick instance if one is configured.
     *
     * @return CommandJoystick instance, or null if no joystick configured
     */
    public static CommandJoystick getJoystick() {
        if (!initialized) return null;

        if (driverType == ControllerType.EXTREME_3D_PRO) {
            return (CommandJoystick) driverController;
        } else if (operatorType == ControllerType.EXTREME_3D_PRO) {
            return (CommandJoystick) operatorController;
        }
        return null;
    }

    /**
     * Gets the driver controller as generic Object.
     * Cast to appropriate type based on driverType.
     *
     * @return Driver controller instance
     */
    public static Object getDriverController() {
        ensureInitialized();
        return driverController;
    }

    /**
     * Gets the operator controller as generic Object.
     * Cast to appropriate type based on operatorType.
     *
     * @return Operator controller instance
     */
    public static Object getOperatorController() {
        ensureInitialized();
        return operatorController;
    }

    /**
     * Gets the driver controller type.
     *
     * @return Driver controller type
     */
    public static ControllerType getDriverType() {
        return driverType;
    }

    /**
     * Gets the operator controller type.
     *
     * @return Operator controller type
     */
    public static ControllerType getOperatorType() {
        return operatorType;
    }

    // ======================== INPUT PROCESSING UTILITIES ========================

    /**
     * Applies a deadzone to controller input, returning 0 if below threshold.
     *
     * <p>This prevents small unintentional joystick movements from being registered.
     *
     * @param input Raw joystick input value
     * @param deadzoneThreshold Minimum absolute value to register (e.g., 0.1)
     * @return Input value if above threshold, otherwise 0
     */
    public static double applyDeadzone(double input, double deadzoneThreshold) {
        return Math.abs(input) < deadzoneThreshold ? 0.0 : input;
    }

    /**
     * Applies a cubic curve for precise low-speed control.
     *
     * <p>This curve combines linear and cubic components for better low-speed
     * control while maintaining high-speed responsiveness. The tuning parameters
     * (0.1 linear + 0.9 cubic) provide precise control at low speeds.
     *
     * <p><strong>Formula:</strong> output = (0.1 * input) + (0.9 * input³)
     *
     * @param input Raw joystick input (-1.0 to 1.0)
     * @return Curved output value (-1.0 to 1.0)
     */
    public static double applyCubicCurve(double input) {
        return (CUBIC_LINEAR_COMPONENT * input) + (CUBIC_CUBIC_COMPONENT * Math.pow(input, 3));
    }

    /**
     * Applies a sigmoid curve for smooth gradual response.
     *
     * <p>This creates a gentle S-shaped response curve that provides fine control
     * near the center while still allowing full range. The steepness parameter
     * controls how quickly the curve transitions.
     *
     * <p><strong>Use Case:</strong> Best for precise positioning tasks where
     * gradual acceleration is desired. For most driving tasks, use cubic curve instead.
     *
     * @param input Raw joystick input (-1.0 to 1.0)
     * @return Smoothed output value (-1.0 to 1.0)
     */
    public static double applySigmoidCurve(double input) {
        double sigmoid = 1.0 / (1.0 + Math.exp(-SIGMOID_STEEPNESS * input));
        return 2.0 * sigmoid - 1.0;  // Map from (0,1) to (-1,1)
    }

    /**
     * Comprehensive input processing with deadzone and curve.
     *
     * <p>This method combines deadzone application and curve selection in one call.
     * Recommended for most use cases as it ensures consistent input processing.
     *
     * <p><strong>Example:</strong>
     * <pre>{@code
     * // Drive with cubic curve and 0.05 deadzone
     * double forward = Buttons.processInput(controller.getLeftY(), 0.05, InputCurve.CUBIC);
     * }</pre>
     *
     * @param rawInput Raw joystick input (-1.0 to 1.0)
     * @param deadzone Deadzone threshold (typically 0.05-0.15)
     * @param curve Input curve type to apply
     * @return Processed input value (-1.0 to 1.0)
     */
    public static double processInput(double rawInput, double deadzone, InputCurve curve) {
        // Apply deadzone first
        double afterDeadzone = applyDeadzone(rawInput, deadzone);
        if (afterDeadzone == 0.0) return 0.0;

        // Apply selected curve
        return switch (curve) {
            case LINEAR -> afterDeadzone;
            case CUBIC -> applyCubicCurve(afterDeadzone);
            case SIGMOID -> applySigmoidCurve(afterDeadzone);
        };
    }

    /**
     * Applies the default input curve (cubic) to raw joystick values.
     *
     * <p>In simulation mode, returns the raw input unmodified for testing.
     * For robot operation, applies cubic curve.
     *
     * @param rawInput Raw joystick input (-1.0 to 1.0)
     * @return Processed input value (-1.0 to 1.0)
     */
    public static double applyCurve(double rawInput) {
        if (RobotBase.isSimulation()) {
            return rawInput;
        }
        return applyCubicCurve(rawInput);
    }

    // ======================== DRIVE INPUT SUPPLIERS ========================

    /**
     * Creates a supplier for forward/backward drive input from the active controller.
     *
     * @param deadzone Deadzone threshold (typically 0.05-0.15)
     * @param curve Input curve type to apply
     * @return DoubleSupplier that provides processed Y-axis input
     */
    public static DoubleSupplier createForwardSupplier(double deadzone, InputCurve curve) {
        return () -> {
            ensureInitialized();
            double rawInput = 0.0;

            if (driverType == ControllerType.EXTREME_3D_PRO) {
                rawInput = ((CommandJoystick) driverController).getY();
            } else if (driverType == ControllerType.XBOX) {
                rawInput = ((CommandXboxController) driverController).getLeftY();
            } else if (driverType == ControllerType.PS5) {
                rawInput = ((CommandPS5Controller) driverController).getLeftY();
            }

            return processInput(rawInput, deadzone, curve);
        };
    }

    /**
     * Creates a supplier for left/right strafe input from the active controller.
     *
     * @param deadzone Deadzone threshold (typically 0.05-0.15)
     * @param curve Input curve type to apply
     * @return DoubleSupplier that provides processed X-axis input
     */
    public static DoubleSupplier createStrafeSupplier(double deadzone, InputCurve curve) {
        return () -> {
            ensureInitialized();
            double rawInput = 0.0;

            if (driverType == ControllerType.EXTREME_3D_PRO) {
                rawInput = ((CommandJoystick) driverController).getX();
            } else if (driverType == ControllerType.XBOX) {
                rawInput = ((CommandXboxController) driverController).getLeftX();
            } else if (driverType == ControllerType.PS5) {
                rawInput = ((CommandPS5Controller) driverController).getLeftX();
            }

            return processInput(rawInput, deadzone, curve);
        };
    }

    /**
     * Creates a supplier for rotational input from the active controller.
     *
     * @param deadzone Deadzone threshold (typically 0.05-0.15)
     * @param curve Input curve type to apply
     * @return DoubleSupplier that provides processed rotation input
     */
    public static DoubleSupplier createRotationSupplier(double deadzone, InputCurve curve) {
        return () -> {
            ensureInitialized();
            double rawInput = 0.0;

            if (driverType == ControllerType.EXTREME_3D_PRO) {
                rawInput = ((CommandJoystick) driverController).getZ();  // Twist axis
            } else if (driverType == ControllerType.XBOX) {
                rawInput = ((CommandXboxController) driverController).getRightX();
            } else if (driverType == ControllerType.PS5) {
                rawInput = ((CommandPS5Controller) driverController).getRightX();
            }

            return processInput(rawInput, deadzone, curve);
        };
    }

    // ======================== RUMBLE/HAPTIC FEEDBACK ========================

    /**
     * Rumbles an Xbox controller for a specified duration and intensity.
     *
     * <p>This method runs asynchronously to avoid blocking the main robot thread.
     * The rumble will automatically stop after the specified time elapses.
     * Any existing rumble will be cancelled when a new rumble starts.
     *
     * <p><strong>Example:</strong>
     * <pre>{@code
     * // Rumble at 70% intensity for 500ms when scoring
     * Buttons.rumbleXbox(Buttons.getXboxController(), 500, 0.7);
     *
     * // Light rumble for 200ms when game piece detected
     * Buttons.rumbleXbox(Buttons.getXboxController(), 200, 0.3);
     * }</pre>
     *
     * @param controller The Xbox controller to rumble
     * @param durationMs Duration of rumble in milliseconds (clamped to 0-5000ms)
     * @param intensity Rumble intensity from 0.0-1.0 (0% to 100%)
     */
    public static void rumbleXbox(CommandXboxController controller, int durationMs, double intensity) {
        ensureInitialized();

        // Cancel any existing rumble
        if (currentRumbleTask != null && !currentRumbleTask.isDone()) {
            currentRumbleTask.cancel(false);
        }

        var hid = controller.getHID();
        intensity = MathUtil.clamp(intensity, 0.0, 1.0);
        int duration = MathUtil.clamp(durationMs, 0, MAX_RUMBLE_DURATION_MS);

        // Start rumble
        hid.setRumble(RumbleType.kBothRumble, intensity);

        // Schedule automatic stop
        currentRumbleTask = rumbleExecutor.schedule(
            () -> hid.setRumble(RumbleType.kBothRumble, 0.0),
            duration,
            TimeUnit.MILLISECONDS
        );
    }

    /**
     * Rumbles a PS5 controller for a specified duration and intensity.
     *
     * <p>Uses the PS5 DualSense haptic feedback system.
     *
     * @param controller The PS5 controller to rumble
     * @param durationMs Duration of rumble in milliseconds (clamped to 0-5000ms)
     * @param intensity Rumble intensity from 0.0-1.0 (0% to 100%)
     */
    public static void rumblePS5(CommandPS5Controller controller, int durationMs, double intensity) {
        ensureInitialized();

        // Cancel any existing rumble
        if (currentRumbleTask != null && !currentRumbleTask.isDone()) {
            currentRumbleTask.cancel(false);
        }

        var hid = controller.getHID();
        intensity = MathUtil.clamp(intensity, 0.0, 1.0);
        int duration = MathUtil.clamp(durationMs, 0, MAX_RUMBLE_DURATION_MS);

        // Start rumble
        hid.setRumble(RumbleType.kBothRumble, intensity);

        // Schedule automatic stop
        currentRumbleTask = rumbleExecutor.schedule(
            () -> hid.setRumble(RumbleType.kBothRumble, 0.0),
            duration,
            TimeUnit.MILLISECONDS
        );
    }

    /**
     * Immediately stops rumble on Xbox controller.
     *
     * @param controller The Xbox controller to stop rumbling
     */
    public static void stopRumbleXbox(CommandXboxController controller) {
        if (currentRumbleTask != null) {
            currentRumbleTask.cancel(false);
        }
        controller.getHID().setRumble(RumbleType.kBothRumble, 0.0);
    }

    /**
     * Immediately stops rumble on PS5 controller.
     *
     * @param controller The PS5 controller to stop rumbling
     */
    public static void stopRumblePS5(CommandPS5Controller controller) {
        if (currentRumbleTask != null) {
            currentRumbleTask.cancel(false);
        }
        controller.getHID().setRumble(RumbleType.kBothRumble, 0.0);
    }

    // ======================== ADVANCED TRIGGER PATTERNS ========================

    /**
     * Creates a trigger binding that continuously re-schedules a command while held.
     *
     * <p>Unlike the standard {@code whileTrue()}, which only schedules the command once,
     * this method re-schedules the command on every loop cycle while the trigger is active.
     * This is useful for commands that need to restart immediately after completing.
     *
     * <p>Adapted from FRC Team 6328 Mechanical Advantage.
     * Licensed under MIT License - Copyright (c) 2025-2026 Littleton Robotics.
     *
     * @see <a href="https://github.com/Mechanical-Advantage/RobotCode2026Public">Original Source</a>
     *
     * <p><strong>Example:</strong>
     * <pre>{@code
     * // Continuously reschedule intake command while button is held
     * Buttons.whileTrueContinuous(Buttons.XboxAButton, new IntakeCommand(intake));
     *
     * // Useful for commands that complete quickly and need immediate restart
     * Buttons.whileTrueContinuous(shootTrigger, new ShootOnceCommand(shooter));
     * }</pre>
     *
     * @param trigger The trigger to monitor
     * @param command The command to continuously schedule while trigger is active
     */
    public static void whileTrueContinuous(Trigger trigger, Command command) {
        trigger.whileTrue(command.repeatedly()).onFalse(command.finallyDo(command::cancel));
    }

    /**
     * Creates a trigger that activates only on double-press within 0.4 seconds.
     *
     * <p>The returned trigger activates when the base trigger is pressed twice
     * in quick succession (within 400ms). Useful for confirming destructive
     * actions or activating special modes.
     *
     * <p>Adapted from FRC Team 6328 Mechanical Advantage.
     * Licensed under MIT License - Copyright (c) 2025-2026 Littleton Robotics.
     *
     * @see <a href="https://github.com/Mechanical-Advantage/RobotCode2026Public">Original Source</a>
     *
     * <p><strong>Example:</strong>
     * <pre>{@code
     * // Require double-press to reset gyro (prevents accidental resets)
     * Trigger resetGyroTrigger = Buttons.doublePress(Buttons.XboxStartButton);
     * resetGyroTrigger.onTrue(Commands.runOnce(() -> gyro.reset()));
     *
     * // Double-press A button to enable "turbo mode"
     * Buttons.doublePress(Buttons.XboxAButton).onTrue(enableTurboMode());
     * }</pre>
     *
     * @param baseTrigger The trigger to monitor for double-press
     * @return A new Trigger that activates only on double-press
     */
    public static Trigger doublePress(Trigger baseTrigger) {
        return doublePress(baseTrigger, 0.4);
    }

    /**
     * Creates a trigger that activates only on double-press within the specified time.
     *
     * <p>Adapted from FRC Team 6328 Mechanical Advantage.
     * Licensed under MIT License - Copyright (c) 2025-2026 Littleton Robotics.
     *
     * @see <a href="https://github.com/Mechanical-Advantage/RobotCode2026Public">Original Source</a>
     *
     * @param baseTrigger The trigger to monitor for double-press
     * @param maxTimeSecs Maximum time between presses to count as double-press
     * @return A new Trigger that activates only on double-press
     */
    public static Trigger doublePress(Trigger baseTrigger, double maxTimeSecs) {
        DoublePressTracker tracker = new DoublePressTracker(baseTrigger, maxTimeSecs);
        return new Trigger(tracker::get);
    }

    /**
     * Tracks double-press detection for a trigger using a state machine.
     *
     * <p>Adapted from FRC Team 6328 Mechanical Advantage.
     * Licensed under MIT License - Copyright (c) 2025-2026 Littleton Robotics.
     *
     * @see <a href="https://github.com/Mechanical-Advantage/RobotCode2026Public">Original Source</a>
     */
    private static class DoublePressTracker {
        /** State machine states for double-press detection. */
        private enum State {
            /** Waiting for first press. */
            IDLE,
            /** Button held after first press. */
            FIRST_PRESS,
            /** Button released, awaiting second press. */
            FIRST_RELEASE,
            /** Second press detected - trigger should activate. */
            SECOND_PRESS
        }

        private final Trigger baseTrigger;
        private final double maxTimeSecs;
        private final Timer timer = new Timer();
        private State state = State.IDLE;

        /**
         * Creates a new DoublePressTracker.
         *
         * @param baseTrigger The trigger to monitor
         * @param maxTimeSecs Maximum time between presses
         */
        DoublePressTracker(Trigger baseTrigger, double maxTimeSecs) {
            this.baseTrigger = baseTrigger;
            this.maxTimeSecs = maxTimeSecs;
        }

        /**
         * Returns whether a double-press is currently detected.
         *
         * <p>Must be called each loop cycle to update the state machine.
         *
         * @return {@code true} if currently in double-press state
         */
        boolean get() {
            boolean pressed = baseTrigger.getAsBoolean();

            switch (state) {
                case IDLE:
                    if (pressed) {
                        state = State.FIRST_PRESS;
                        timer.restart();
                    }
                    break;

                case FIRST_PRESS:
                    if (!pressed) {
                        state = State.FIRST_RELEASE;
                    } else if (timer.hasElapsed(maxTimeSecs)) {
                        // Held too long without release - reset
                        state = State.IDLE;
                    }
                    break;

                case FIRST_RELEASE:
                    if (pressed) {
                        if (!timer.hasElapsed(maxTimeSecs)) {
                            state = State.SECOND_PRESS;
                        } else {
                            // Too slow - treat as new first press
                            state = State.FIRST_PRESS;
                            timer.restart();
                        }
                    } else if (timer.hasElapsed(maxTimeSecs)) {
                        // Timed out waiting for second press
                        state = State.IDLE;
                    }
                    break;

                case SECOND_PRESS:
                    if (!pressed) {
                        state = State.IDLE;
                    }
                    break;
            }

            return state == State.SECOND_PRESS;
        }
    }
}
