package com.adambots.lib.utils;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * All Game Controller Button Mappings
 * <p>
 * This class must be initialized by calling {@code Buttons.init()}
 * from your Robot class's {@code robotInit()} method.
 */
public class Buttons {

        // Declare controllers and triggers as static, but NOT final initially.
        // They will be initialized in the static init() method.
        public static CommandXboxController xBoxController;
        public static CommandJoystick ex3dPro;

        private static boolean initialized = false;

        // All Trigger fields will also be static (non-final)
        public static Trigger XboxBackButton;
        public static Trigger XboxStartButton;
        public static Trigger XboxXButton;
        public static Trigger XboxYButton;
        public static Trigger XboxBButton;
        public static Trigger XboxAButton;
        public static Trigger XboxLeftBumper;
        public static Trigger XboxRightBumper;
        public static Trigger XboxLeftStickButton;
        public static Trigger XboxRightStickButton;

        public static Trigger XboxLeftTriggerButton;
        public static Trigger XboxRightTriggerButton;

        public static Trigger rightStickUp;
        public static Trigger rightStickDown;
        public static Trigger leftStickUp;
        public static Trigger leftStickDown;

        public static Trigger XboxDPadN;
        public static Trigger XboxDPadNE;
        public static Trigger XboxDPadE;
        public static Trigger XboxDPadSE;
        public static Trigger XboxDPadS;
        public static Trigger XboxDPadSW;
        public static Trigger XboxDPadW;
        public static Trigger XboxDPadNW;

        public static Trigger JoystickButton1;
        public static Trigger JoystickButton2;
        public static Trigger JoystickButton3;
        public static Trigger JoystickButton4;
        public static Trigger JoystickButton5;
        public static Trigger JoystickButton6;
        public static Trigger JoystickButton7;
        public static Trigger JoystickButton8;
        public static Trigger JoystickButton9;
        public static Trigger JoystickButton10;
        public static Trigger JoystickButton11;
        public static Trigger JoystickButton12;
        public static Trigger JoystickButton13;
        public static Trigger JoystickButton14;
        public static Trigger JoystickButton15;
        public static Trigger JoystickButton16;

        public static Trigger JoystickThumbUp;
        public static Trigger JoystickThumbDown;
        public static Trigger JoystickThumbUpLeft;
        public static Trigger JoystickThumbUpRight;
        public static Trigger JoystickThumbDownLeft;
        public static Trigger JoystickThumbDownRight;
        public static Trigger JoystickThumbLeft;
        public static Trigger JoystickThumbRight;
        public static Trigger JoystickThumbCenter;

        /**
         * Initializes the Buttons class with the specified controller ports.
         * This method must be called exactly once, typically in Robot.robotInit().
         *
         * @param xboxControllerPort     The USB port for the Xbox Controller.
         * @param joystickControllerPort The USB port for the EX3D Pro Joystick.
         */
        public static void init(int xboxControllerPort, int joystickControllerPort) {
                if (initialized) {
                        System.err.println("Buttons.init() called multiple times. Ignoring subsequent calls.");
                        return;
                }

                // Initialize controllers
                xBoxController = new CommandXboxController(xboxControllerPort);
                ex3dPro = new CommandJoystick(joystickControllerPort);

                // Initialize all Xbox Controller Buttons
                XboxBackButton = xBoxController.back();
                XboxStartButton = xBoxController.start();
                XboxXButton = xBoxController.x();
                XboxYButton = xBoxController.y();
                XboxBButton = xBoxController.b();
                XboxAButton = xBoxController.a();
                XboxLeftBumper = xBoxController.leftBumper();
                XboxRightBumper = xBoxController.rightBumper();
                XboxLeftStickButton = xBoxController.leftStick();
                XboxRightStickButton = xBoxController.rightStick();

                // Initialize Xbox Throttle triggers
                XboxLeftTriggerButton = xBoxController.leftTrigger(0.3);
                XboxRightTriggerButton = xBoxController.rightTrigger(0.3);

                // Initialize Xbox Stick Throttles
                rightStickUp = new Trigger(() -> xBoxController.getRightY() < -0.8);
                rightStickDown = new Trigger(() -> xBoxController.getRightY() > 0.8);

                leftStickUp = new Trigger(() -> xBoxController.getLeftY() < -0.8);
                leftStickDown = new Trigger(() -> xBoxController.getLeftY() > 0.8);

                // Initialize Xbox DPad
                XboxDPadN = xBoxController.pov(0);
                XboxDPadNE = xBoxController.pov(45);
                XboxDPadE = xBoxController.pov(90);
                XboxDPadSE = xBoxController.pov(135);
                XboxDPadS = xBoxController.pov(180);
                XboxDPadSW = xBoxController.pov(225);
                XboxDPadW = xBoxController.pov(270);
                XboxDPadNW = xBoxController.pov(315);

                // Initialize Joystick Buttons
                JoystickButton1 = ex3dPro.button(1);
                JoystickButton2 = ex3dPro.button(2);
                JoystickButton3 = ex3dPro.button(3);
                JoystickButton4 = ex3dPro.button(4);
                JoystickButton5 = ex3dPro.button(5);
                JoystickButton6 = ex3dPro.button(6);
                JoystickButton7 = ex3dPro.button(7);
                JoystickButton8 = ex3dPro.button(8);
                JoystickButton9 = ex3dPro.button(9);
                JoystickButton10 = ex3dPro.button(10);
                JoystickButton11 = ex3dPro.button(11);
                JoystickButton12 = ex3dPro.button(12);
                JoystickButton13 = ex3dPro.button(13);
                JoystickButton14 = ex3dPro.button(14);
                JoystickButton15 = ex3dPro.button(15);
                JoystickButton16 = ex3dPro.button(16);

                // Initialize Joystick Thumb Pad
                JoystickThumbUp = ex3dPro.povUp();
                JoystickThumbDown = ex3dPro.povDown();
                JoystickThumbUpLeft = ex3dPro.povUpLeft();
                JoystickThumbUpRight = ex3dPro.povUpRight();
                JoystickThumbDownLeft = ex3dPro.povDownLeft();
                JoystickThumbDownRight = ex3dPro.povDownRight();
                JoystickThumbLeft = ex3dPro.povLeft();
                JoystickThumbRight = ex3dPro.povRight();
                JoystickThumbCenter = ex3dPro.povCenter();

                initialized = true;
        }

        // --- Utility methods to ensure controllers are initialized before access ---
        private static void ensureInitialized() {
                if (!initialized) {
                        throw new IllegalStateException("Buttons class not initialized. Call Buttons.init() first.");
                }
        }

        // Add public static getters for the controllers themselves
        public static CommandXboxController getxBoxController() {
                ensureInitialized();
                return xBoxController;
        }

        public static CommandJoystick getEx3dPro() {
                ensureInitialized();
                return ex3dPro;
        }

        // --- Existing static methods remain largely unchanged ---

        /**
         * Return a value only if it is greater than a threshold, otherwise return 0
         * <p>
         * DO NOT USE ON TOP OF applyCurve, ADJUST THE CURVE ITSELF TO HAVE DESIRED
         * DEADZONE
         */
        public static double deaden(double input, double deadenThreshold) {
                if (Math.abs(input) < deadenThreshold) {
                        return 0;
                } else {
                        return input;
                }
        }

        // Sigmoid Curve to smooth input values. Use this if you want a smooth curve.
        // Mr. B - DO NOT REMOVE THIS FUNCTION EVEN IF IT IS NOT USED
        public static double smoothInput(double input) {
                // Adjust the parameter 'a' to control the steepness of the curve
                double a = 2.0;

                // Apply a sigmoid function to the input value
                double sigmoid = 1.0 / (1.0 + Math.exp(-a * input));

                // Map the output range from (0,1) to (minOutput, maxOutput)
                double output = 2 * sigmoid - 1;

                return output;
        }

        // Cubic Curve - An alternative to Sigmoid curve. Use this if you want a more
        // aggressive curve.
        // Mr. B - DO NOT REMOVE THIS FUNCTION EVEN IF IT IS NOT USED
        public static double cubic(double input) {
                double tuneA = 0.1; // try different values. However, tuneA and tuneB should add up to 1
                double tuneB = 0.9;

                return (tuneA * input) + (tuneB * Math.pow(input, 3));
        }

        public static double applyCurve(double rawInput) {
                if (RobotBase.isSimulation()) {
                        return rawInput;
                }

                // return smoothInput(rawInput);
                return cubic(rawInput);
        }

        // These suppliers now implicitly use the 'ex3dPro' field, which is
        // initialized in the init() method.
        public static DoubleSupplier forwardSupplier = () -> {
                ensureInitialized(); // Ensure controller is ready
                return applyCurve(ex3dPro.getY());
        };
        public static DoubleSupplier sidewaysSupplier = () -> {
                ensureInitialized(); // Ensure controller is ready
                return applyCurve(ex3dPro.getX());
        };
        public static DoubleSupplier rotateSupplier = () -> {
                ensureInitialized(); // Ensure controller is ready
                return applyCurve(ex3dPro.getZ());
        };

        /**
         * Rumble the XBox Controller
         *
         * @param controller    pass the Xbox controller to rumble
         * @param timeInMillis  how many milliseconds to rumble the controller - max
         *                      value is 5000
         * @param intensity0to1 how intense should the rumble be
         */
        public static void rumble(CommandXboxController controller, int timeInMillis, int intensity0to1) {
                ensureInitialized(); // Ensure controller is ready
                var joy = controller.getHID();
                final int time = MathUtil.clamp(timeInMillis, 0, 5000);

                // Perform an async operation to avoid scheduler overruns
                Thread rumbleThread = new Thread(() -> {

                        long rumbleStartTime = System.currentTimeMillis();

                        while (System.currentTimeMillis() - rumbleStartTime <= time) {
                                joy.setRumble(RumbleType.kBothRumble, intensity0to1); // Rumble both sides of the
                                                                                      // controller
                        }

                        joy.setRumble(RumbleType.kBothRumble, 0);
                });

                rumbleThread.start();
        }
}