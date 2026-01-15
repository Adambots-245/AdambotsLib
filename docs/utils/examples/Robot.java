package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import com.adambots.lib.utils.Buttons;
import com.adambots.lib.utils.Buttons.ControllerType;

/**
 * Example Robot.java showing proper Buttons initialization.
 *
 * <p>This example demonstrates:
 * <ul>
 *   <li>Initializing Buttons in robotInit() BEFORE RobotContainer</li>
 *   <li>Configuring controller types for driver and operator</li>
 *   <li>Proper command scheduler integration</li>
 *   <li>Standard autonomous and teleop initialization</li>
 * </ul>
 *
 * <p><strong>CRITICAL:</strong> Buttons.init() MUST be called before creating
 * RobotContainer because RobotContainer uses Buttons triggers for command binding.
 */
public class Robot extends TimedRobot {
    private Command m_autonomousCommand;
    private RobotContainer m_robotContainer;

    /**
     * This function is run when the robot is first started up and should be used
     * for any initialization code.
     */
    @Override
    public void robotInit() {
        // ========================================================================
        // STEP 1: Initialize Buttons FIRST
        // ========================================================================
        // This MUST happen before creating RobotContainer because RobotContainer
        // uses Buttons triggers (e.g., Buttons.XboxA.onTrue(...))
        //
        // Parameters:
        //   - Driver port: 0 (USB port on driver station)
        //   - Operator port: 1 (USB port on driver station)
        //   - Driver controller type: XBOX
        //   - Operator controller type: PS5
        //
        // Common configurations:
        //   - Both Xbox: Buttons.init(0, 1, ControllerType.XBOX, ControllerType.XBOX)
        //   - Both PS5: Buttons.init(0, 1, ControllerType.PS5, ControllerType.PS5)
        //   - Xbox + Joystick: Buttons.init(0, 1, ControllerType.XBOX, ControllerType.EXTREME_3D_PRO)
        //   - Driver only: Buttons.init(0, -1, ControllerType.XBOX, ControllerType.NONE)
        //
        Buttons.init(0, 1, ControllerType.XBOX, ControllerType.PS5);

        // ========================================================================
        // STEP 2: Create RobotContainer
        // ========================================================================
        // Now that Buttons is initialized, we can create RobotContainer which
        // will bind commands to Buttons triggers
        m_robotContainer = new RobotContainer();
    }

    /**
     * This function is called every 20 ms, no matter the mode. Use this for items
     * like diagnostics that you want run during disabled, autonomous, teleoperated
     * and test modes.
     *
     * <p>This runs after the mode specific periodic functions, but before LiveWindow
     * and SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
        // Runs the Scheduler. This is responsible for polling buttons, adding
        // newly-scheduled commands, running already-scheduled commands, removing
        // finished or interrupted commands, and running subsystem periodic() methods.
        // This must be called from the robot's periodic block in order for anything
        // in the Command-based framework to work.
        CommandScheduler.getInstance().run();
    }

    /**
     * This function is called once each time the robot enters Disabled mode.
     */
    @Override
    public void disabledInit() {
        // Optional: Stop all rumble when disabled
        if (Buttons.getDriverType() == ControllerType.XBOX) {
            Buttons.stopRumbleXbox(Buttons.getXboxController());
        } else if (Buttons.getDriverType() == ControllerType.PS5) {
            Buttons.stopRumblePS5(Buttons.getPS5Controller());
        }
    }

    @Override
    public void disabledPeriodic() {}

    /**
     * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
     */
    @Override
    public void autonomousInit() {
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        // Schedule the autonomous command
        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void teleopInit() {
        // This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to
        // continue until interrupted by another command, remove
        // this line or comment it out.
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }

        // Optional: Give feedback when teleop starts
        if (Buttons.getDriverType() == ControllerType.XBOX) {
            Buttons.rumbleXbox(Buttons.getXboxController(), 200, 0.3);
        } else if (Buttons.getDriverType() == ControllerType.PS5) {
            Buttons.rumblePS5(Buttons.getPS5Controller(), 200, 0.3);
        }
    }

    @Override
    public void teleopPeriodic() {}

    @Override
    public void testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void simulationInit() {}

    @Override
    public void simulationPeriodic() {}
}
