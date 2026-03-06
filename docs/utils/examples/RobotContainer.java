package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import com.adambots.lib.utils.Buttons;
import com.adambots.lib.utils.Buttons.InputCurve;
import frc.robot.subsystems.*;
import frc.robot.Constants.FieldPositions;

/**
 * Example RobotContainer showing comprehensive Buttons usage.
 *
 * <p>This example demonstrates:
 * <ul>
 *   <li>Default drive command with input curves and deadzones</li>
 *   <li>Button bindings for Xbox and PS5 controllers</li>
 *   <li>Rumble feedback for important events</li>
 *   <li>Trigger combinations for safety-critical operations</li>
 *   <li>Organized binding methods by subsystem</li>
 *   <li>Mixed controller type support</li>
 * </ul>
 *
 * <p><strong>Prerequisites:</strong> Buttons.init() must be called in Robot.robotInit()
 * BEFORE creating RobotContainer.
 */
public class RobotContainer {
    // ========================================================================
    // Subsystems
    // ========================================================================
    private final SwerveSubsystem swerve = new SwerveSubsystem();
    private final IntakeSubsystem intake = new IntakeSubsystem();
    private final ShooterSubsystem shooter = new ShooterSubsystem();
    private final ClimberSubsystem climber = new ClimberSubsystem();
    private final LEDSubsystem leds = new LEDSubsystem();

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        configureDefaultCommands();
        configureDriverBindings();
        configureOperatorBindings();
        configureComboBindings();
    }

    // ========================================================================
    // Default Commands
    // ========================================================================
    /**
     * Configure default commands for subsystems.
     *
     * <p>Default commands run whenever no other command is using the subsystem.
     * This is where we set up the swerve drive command that runs during teleop.
     */
    private void configureDefaultCommands() {
        // ====================================================================
        // Swerve Drive with Input Processing
        // ====================================================================
        // Create drive suppliers with:
        //   - 0.05 (5%) deadzone for translation (prevents drift)
        //   - 0.1 (10%) deadzone for rotation (larger for stability)
        //   - CUBIC curve for translation (smooth low-speed control)
        //   - SIGMOID curve for rotation (prevents over-rotation)
        //
        // The suppliers automatically adapt to the controller type:
        //   - Xbox: Left stick Y/X for forward/strafe, Right stick X for rotation
        //   - PS5: Left stick Y/X for forward/strafe, Right stick X for rotation
        //   - Joystick: Y/X for forward/strafe, Twist for rotation
        //
        swerve.setDefaultCommand(
            swerve.driveCommand(
                Buttons.createForwardSupplier(0.05, InputCurve.CUBIC),
                Buttons.createStrafeSupplier(0.05, InputCurve.CUBIC),
                Buttons.createRotationSupplier(0.1, InputCurve.SIGMOID),
                0.8
            )
        );

        // Alternative: Linear curves for experienced drivers
        // swerve.setDefaultCommand(
        //     swerve.driveCommand(
        //         Buttons.createForwardSupplier(0.05, InputCurve.LINEAR),
        //         Buttons.createStrafeSupplier(0.05, InputCurve.LINEAR),
        //         Buttons.createRotationSupplier(0.1, InputCurve.LINEAR),
        //         0.8
        //     )
        // );

        // Alternative: Custom input processing
        // swerve.setDefaultCommand(
        //     swerve.driveCommand(
        //         () -> {
        //             double raw = Buttons.getXboxController().getLeftY();
        //             return Buttons.processInput(raw, 0.08, InputCurve.SIGMOID);
        //         },
        //         () -> {
        //             double raw = Buttons.getXboxController().getLeftX();
        //             return Buttons.processInput(raw, 0.08, InputCurve.SIGMOID);
        //         },
        //         () -> {
        //             double raw = Buttons.getXboxController().getRightX();
        //             return Buttons.processInput(raw, 0.15, InputCurve.CUBIC);
        //         },
        //         0.8
        //     )
        // );
    }

    // ========================================================================
    // Driver Bindings (Xbox Controller)
    // ========================================================================
    /**
     * Configure driver controller bindings.
     *
     * <p>Driver controls:
     * <ul>
     *   <li>A/B - Intake deploy/retract</li>
     *   <li>X/Y - Intake/Eject game pieces</li>
     *   <li>Left/Right Bumper - Speed control</li>
     *   <li>Left/Right Trigger - Auto-aim features</li>
     *   <li>Back/Start - System functions</li>
     *   <li>D-Pad - Preset positions</li>
     * </ul>
     */
    private void configureDriverBindings() {
        // ====================================================================
        // Face Buttons - Intake Control
        // ====================================================================

        // A - Deploy intake with rumble feedback
        Buttons.XboxA.onTrue(
            intake.deployCommand()
                .andThen(Commands.runOnce(() ->
                    Buttons.rumbleXbox(Buttons.getXboxController(), 200, 0.5)
                ))
        );

        // B - Retract intake
        Buttons.XboxB.onTrue(intake.retractCommand());

        // X - Intake game piece (hold button)
        Buttons.XboxX.whileTrue(
            intake.intakeCommand()
                // When game piece acquired, rumble and stop
                .until(() -> intake.hasGamePiece())
                .andThen(Commands.runOnce(() ->
                    Buttons.rumbleXbox(Buttons.getXboxController(), 300, 0.7)
                ))
        );

        // Y - Eject game piece
        Buttons.XboxY.whileTrue(intake.ejectCommand());

        // ====================================================================
        // Bumpers - Speed Control
        // ====================================================================

        // Left Bumper - Slow mode (30% speed)
        Buttons.XboxLeftBumper.whileTrue(swerve.setMaxSpeedCommand(0.3));

        // Right Bumper - Turbo mode (100% speed)
        Buttons.XboxRightBumper.whileTrue(swerve.setMaxSpeedCommand(1.0));

        // ====================================================================
        // Triggers - Auto-Aim Features
        // ====================================================================

        // Left Trigger - Auto-align to AprilTag 4 (Speaker)
        Buttons.XboxLeftTriggerButton.whileTrue(
            swerve.aimAtAprilTagCommand(4, 2.0)  // Tag 4, 2 degree tolerance
        );

        // Right Trigger - Auto-aim at vision target
        Buttons.XboxRightTriggerButton.whileTrue(
            swerve.aimAtTargetCommand(Cameras.SHOOTER_CAM)
        );

        // ====================================================================
        // Back/Start - System Functions
        // ====================================================================

        // Back - Reset gyro to 0 degrees
        Buttons.XboxBack.onTrue(swerve.resetGyroCommand());

        // Start - Lock wheels (X-pattern for defense)
        Buttons.XboxStart.whileTrue(swerve.lockWheelsCommand());

        // ====================================================================
        // D-Pad - Preset Positions
        // ====================================================================

        // D-Pad Up - Drive to Amp
        Buttons.XboxPOVUp.onTrue(
            swerve.driveToPoseCommand(FieldPositions.AMP)
        );

        // D-Pad Right - Drive to Source
        Buttons.XboxPOVRight.onTrue(
            swerve.driveToPoseCommand(FieldPositions.SOURCE)
        );

        // D-Pad Down - Drive to Speaker
        Buttons.XboxPOVDown.onTrue(
            swerve.driveToPoseCommand(FieldPositions.SPEAKER)
        );

        // D-Pad Left - Drive to Climb Position
        Buttons.XboxPOVLeft.onTrue(
            swerve.driveToPoseCommand(FieldPositions.CLIMB)
        );

        // ====================================================================
        // Stick Presses - Less Common Functions
        // ====================================================================

        // Left Stick Press - Toggle field-oriented drive
        Buttons.XboxLeftStick.onTrue(swerve.toggleFieldOrientedCommand());

        // Right Stick Press - Center modules (for calibration)
        Buttons.XboxRightStick.onTrue(swerve.centerModulesCommand());
    }

    // ========================================================================
    // Operator Bindings (PS5 Controller)
    // ========================================================================
    /**
     * Configure operator controller bindings.
     *
     * <p>Operator controls:
     * <ul>
     *   <li>Square/Circle/Triangle - Shooter control</li>
     *   <li>Cross - Feed to shooter</li>
     *   <li>L1/R1 - Climber control</li>
     *   <li>L2/R2 - Manual climber adjustment</li>
     *   <li>D-Pad - Shooter presets</li>
     *   <li>Options - Emergency stop</li>
     * </ul>
     */
    private void configureOperatorBindings() {
        // ====================================================================
        // Face Buttons - Shooter Control
        // ====================================================================

        // Square - Spin up shooter wheels
        Buttons.PS5Square.whileTrue(
            shooter.spinUpCommand()
                // When at speed, rumble operator
                .alongWith(
                    Commands.waitUntil(() -> shooter.isAtSpeed())
                        .andThen(Commands.runOnce(() ->
                            Buttons.rumblePS5(Buttons.getPS5Controller(), 200, 0.5)
                        ))
                )
        );

        // Circle - Shoot (requires shooter at speed)
        Buttons.PS5Circle.onTrue(
            Commands.either(
                // If at speed, shoot
                Commands.sequence(
                    shooter.shootCommand(),
                    Commands.runOnce(() ->
                        Buttons.rumblePS5(Buttons.getPS5Controller(), 300, 0.7)
                    )
                ),
                // If not at speed, rumble as warning
                Commands.runOnce(() ->
                    Buttons.rumblePS5(Buttons.getPS5Controller(), 100, 1.0)
                ),
                shooter::isAtSpeed
            )
        );

        // Triangle - Stop shooter
        Buttons.PS5Triangle.onTrue(shooter.stopCommand());

        // Cross - Feed game piece to shooter
        Buttons.PS5Cross.whileTrue(intake.feedToShooterCommand());

        // ====================================================================
        // Bumpers - Climber Control
        // ====================================================================

        // L1 - Extend climber
        Buttons.PS5L1.whileTrue(climber.extendCommand());

        // R1 - Retract climber
        Buttons.PS5R1.whileTrue(climber.retractCommand());

        // ====================================================================
        // Triggers - Manual Climber Adjustment
        // ====================================================================

        // L2 - Manual left climber control
        Buttons.PS5L2Button.whileTrue(
            climber.manualLeftCommand(
                () -> Buttons.getPS5Controller().getL2Axis()
            )
        );

        // R2 - Manual right climber control
        Buttons.PS5R2Button.whileTrue(
            climber.manualRightCommand(
                () -> Buttons.getPS5Controller().getR2Axis()
            )
        );

        // ====================================================================
        // D-Pad - Shooter Presets
        // ====================================================================

        // D-Pad Up - Speaker shot
        Buttons.PS5DPadUp.onTrue(shooter.setShotCommand(ShooterPresets.SPEAKER));

        // D-Pad Down - Amp shot
        Buttons.PS5DPadDown.onTrue(shooter.setShotCommand(ShooterPresets.AMP));

        // D-Pad Left - Pass shot
        Buttons.PS5DPadLeft.onTrue(shooter.setShotCommand(ShooterPresets.PASS));

        // D-Pad Right - Lob shot
        Buttons.PS5DPadRight.onTrue(shooter.setShotCommand(ShooterPresets.LOB));

        // ====================================================================
        // System Buttons
        // ====================================================================

        // Options - Emergency stop all mechanisms
        Buttons.PS5Options.onTrue(
            Commands.parallel(
                shooter.stopCommand(),
                intake.stopCommand(),
                climber.stopCommand()
            ).withTimeout(0.1)
        );

        // Create - Toggle LED animations
        Buttons.PS5Create.onTrue(leds.toggleAnimationsCommand());

        // PS Button - Reserved for system use
        // Touchpad - Reserved for future use
    }

    // ========================================================================
    // Combination Bindings
    // ========================================================================
    /**
     * Configure combination button bindings.
     *
     * <p>These require pressing multiple buttons simultaneously for
     * safety-critical or advanced operations.
     */
    private void configureComboBindings() {
        // ====================================================================
        // Driver + Operator Combos
        // ====================================================================

        // Driver Start + Operator Triangle - Deploy climber sequence
        // (Requires both drivers to agree before deploying climber)
        Buttons.XboxStart.and(Buttons.PS5Triangle).onTrue(
            Commands.sequence(
                Commands.print("=== DEPLOYING CLIMBER ==="),
                climber.deploySequenceCommand(),
                Commands.runOnce(() -> {
                    Buttons.rumbleXbox(Buttons.getXboxController(), 500, 0.5);
                    Buttons.rumblePS5(Buttons.getPS5Controller(), 500, 0.5);
                })
            )
        );

        // Driver Back + Operator Circle - Retract climber sequence
        Buttons.XboxBack.and(Buttons.PS5Circle).onTrue(
            Commands.sequence(
                Commands.print("=== RETRACTING CLIMBER ==="),
                climber.retractSequenceCommand(),
                Commands.runOnce(() -> {
                    Buttons.rumbleXbox(Buttons.getXboxController(), 500, 0.5);
                    Buttons.rumblePS5(Buttons.getPS5Controller(), 500, 0.5);
                })
            )
        );

        // ====================================================================
        // Driver Combos
        // ====================================================================

        // Left Bumper + A - Advanced intake mode
        Buttons.XboxLeftBumper.and(Buttons.XboxA).onTrue(
            intake.advancedIntakeCommand()
        );

        // Right Bumper + X - Reverse intake at high speed
        Buttons.XboxRightBumper.and(Buttons.XboxX).whileTrue(
            intake.reverseHighSpeedCommand()
        );

        // ====================================================================
        // Operator Combos
        // ====================================================================

        // L1 + R1 - Reset climber encoders
        Buttons.PS5L1.and(Buttons.PS5R1).onTrue(
            climber.resetEncodersCommand()
        );

        // Square + Circle - Auto-shoot sequence
        Buttons.PS5Square.and(Buttons.PS5Circle).onTrue(
            Commands.sequence(
                shooter.spinUpCommand().withTimeout(1.5),
                Commands.waitUntil(() -> shooter.isAtSpeed()),
                shooter.shootCommand(),
                Commands.runOnce(() ->
                    Buttons.rumblePS5(Buttons.getPS5Controller(), 300, 0.7)
                )
            )
        );
    }

    // ========================================================================
    // Autonomous
    // ========================================================================
    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // Example: Get autonomous command from chooser
        // return autoChooser.getSelected();

        // Example: Fixed autonomous path
        return swerve.getAutonomousCommand("4PieceAuto");

        // Example: Conditional autonomous based on alliance
        // if (DriverStation.getAlliance().isPresent() &&
        //     DriverStation.getAlliance().get() == Alliance.Red) {
        //     return swerve.getAutonomousCommand("RedAuto");
        // } else {
        //     return swerve.getAutonomousCommand("BlueAuto");
        // }
    }
}
