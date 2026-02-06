// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.lib.subsystems;

import java.io.File;
import java.util.List;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import static edu.wpi.first.units.Units.*;
import edu.wpi.first.units.measure.*;

import java.util.Objects;

import com.adambots.lib.Constants.DriveConstants;
import com.adambots.lib.Constants.ModuleConstants;
import com.adambots.lib.targets.GameTarget;
import com.adambots.lib.targets.GameTargetConfig;
import com.adambots.lib.targets.GameTargetLoader;
import com.adambots.lib.utils.Utils;
import com.adambots.lib.vision.VisionCameraInterface;
import com.adambots.lib.vision.VisionResult;
import com.adambots.lib.vision.VisionSystem;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import swervelib.SwerveDrive;
import swervelib.SwerveDriveTest;
import swervelib.imu.SwerveIMU;
import swervelib.math.SwerveMath;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveControllerConfiguration;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

/**
 * YAGSL-based swerve drive subsystem for holonomic drivetrain control.
 *
 * <p>This subsystem uses YAGSL (Yet Another Generic Swerve Library) to manage
 * all swerve module hardware and kinematics. Swerve modules are configured via
 * JSON files in /deploy/swerve/kraken/ - NO custom SwerveModule class is needed.
 *
 * <p><strong>Features:</strong>
 * <ul>
 *   <li>Vision system integration for vision-corrected odometry</li>
 *   <li>PathPlanner integration for autonomous path following</li>
 *   <li>Command factory methods for all drive operations</li>
 *   <li>Trigger methods for state-based command composition</li>
 * </ul>
 *
 * <p><strong>Configuration:</strong>
 * <ul>
 *   <li>Swerve modules: /deploy/swerve/kraken/modules/*.json</li>
 *   <li>Drive config: /deploy/swerve/kraken/swervedrive.json</li>
 *   <li>PID properties: /deploy/swerve/kraken/modules/pidfproperties.json</li>
 *   <li>PathPlanner PID and behavior: {@link SwerveConfig} (passed to constructor)</li>
 * </ul>
 *
 * <p><strong>Command Factories:</strong>
 * This subsystem provides command factory methods grouped by category:
 * <ul>
 *   <li><strong>Vision-based commands:</strong> aimAtTargetCommand, alignAndStrafeCommand,
 *       driveToNearestPoseWithVisionCommand, etc.</li>
 *   <li><strong>Vision control commands:</strong> enableVisionCommand, disableVisionCommand</li>
 *   <li><strong>PathPlanner commands:</strong> driveToPoseCommand, getAutonomousCommand</li>
 *   <li><strong>Manual drive commands:</strong> driveCommand (multiple overloads),
 *       driveFieldOrientedCommand, centerModulesCommand</li>
 *   <li><strong>Distance-based commands:</strong> driveToDistanceCommand,
 *       driveToDistanceFieldOrientedCommand, driveForwardDistanceCommand</li>
 *   <li><strong>Simple movement commands:</strong> turnToAngleCommand, strafeCommand,
 *       creepForwardCommand, creepCommand, driveToPositionWithHeadingCommand</li>
 *   <li><strong>SysId commands:</strong> sysIdDriveMotorCommand, sysIdAngleMotorCommand</li>
 * </ul>
 *
 * <p><strong>Triggers:</strong>
 * State exposure triggers for command composition:
 * <ul>
 *   <li>atPoseTrigger() - Check proximity to target pose</li>
 *   <li>inRegionTrigger() - Check if robot is within field region</li>
 *   <li>isStationaryTrigger() - Check if robot velocity is below threshold</li>
 *   <li>isMovingFastTrigger() - Check if robot velocity exceeds threshold</li>
 *   <li>isAlignedWithTagTrigger() - Check alignment with AprilTag</li>
 *   <li>hasVisionTargetTrigger() - Check if any AprilTags visible</li>
 *   <li>isInRangeOfTagTrigger() - Check distance range to AprilTag</li>
 *   <li>isFacingHeadingTrigger() - Check if robot is facing target heading</li>
 * </ul>
 *
 * <p><strong>Example Usage:</strong>
 * <pre>{@code
 * // Manual drive with controller
 * swerve.setDefaultCommand(
 *   swerve.driveCommand(
 *     () -> -controller.getLeftY(),
 *     () -> -controller.getLeftX(),
 *     () -> -controller.getRightX()
 *   )
 * );
 *
 * // Autonomous path following
 * Command autoCommand = swerve.getAutonomousCommand("MyAutoPath");
 *
 * // Vision-based alignment
 * Command aimCommand = swerve.aimAtAprilTagCommand(4, 2.0);
 *
 * // State-based command binding
 * swerve.hasVisionTargetTrigger()
 *   .whileTrue(swerve.aimAtAprilTagCommand(7, 2.0));
 * }</pre>
 *
 * @see <a href="https://docs.yagsl.com/">YAGSL Documentation</a>
 * @see <a href="https://github.com/BroncBotz3481/YAGSL">YAGSL GitHub Repository</a>
 * @see VisionSystem
 * @see com.pathplanner.lib.auto.AutoBuilder
 */
public class SwerveSubsystem extends SubsystemBase {

  private final SwerveDrive swerveDrive;
  private final AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

  /**
   * Vision system for pose estimation and target tracking.
   * Nullable until setupVision() is called.
   */
  private VisionSystem vision;

  // Configuration
  private final SwerveConfig swerveConfig;

  // Game target configuration
  private GameTargetConfig gameTargetConfig;

  // State tracking for commands
  private Pose2d startPose;

  /**
   * Creates a new SwerveSubsystem with default configuration.
   *
   * <p>Uses default PID values and drive behavior settings. For custom configuration,
   * use {@link #SwerveSubsystem(File, SwerveConfig)} instead.
   *
   * @param directory The directory containing YAGSL swerve configuration JSON files
   *
   * @see #SwerveSubsystem(File, SwerveConfig)
   */
  public SwerveSubsystem(File directory) {
    this(directory, new SwerveConfig());
  }

  /**
   * Creates a new SwerveSubsystem with custom configuration.
   *
   * <p>Allows customization of PathPlanner PID values and drive behavior settings
   * without modifying AdambotsLib source code.
   *
   * <p><strong>Usage Example:</strong>
   * <pre>{@code
   * // Define config in Constants
   * public static final SwerveConfig SWERVE_CONFIG = new SwerveConfig()
   *     .withTranslationPID(5.0, 0.0, 0.0)
   *     .withRotationPID(3.0, 0.0, 0.0)
   *     .withHeadingCorrection(false)
   *     .withCosineCompensation(true);
   *
   * // Create subsystem with config
   * SwerveSubsystem swerve = new SwerveSubsystem(
   *     new File(Filesystem.getDeployDirectory(), "swerve/kraken"),
   *     Constants.SWERVE_CONFIG
   * );
   * }</pre>
   *
   * @param directory The directory containing YAGSL swerve configuration JSON files
   * @param config Configuration object with PID values and behavior settings
   *
   * @see SwerveConfig
   */
  public SwerveSubsystem(File directory, SwerveConfig config) {
    this.swerveConfig = config;
    // The 2 value below will be defined in the JSON configuration file. However,
    // alternatively, we can do it here.
    // Use the same values when defining the JSON file

    // The angleConversionFactor is used to convert the steering encoder readings
    // to actual wheel angles. This is necessary for accurate control of the swerve
    // drive system, ensuring that the wheels are oriented correctly based on the
    // desired direction and rotation.
    // Angle conversion factor is 360 / (GEAR RATIO * ENCODER RESOLUTION)
    // In this case the gear ratio is 12.8 motor revolutions per wheel rotation.
    // The encoder resolution per motor revolution is 1 per motor revolution.
    double angleConversionFactor = SwerveMath.calculateDegreesPerSteeringRotation(ModuleConstants.kSteeringGearRatio);

    // Calculates the drive conversion factor in meters per rotation for the swerve
    // module.
    // This is required to convert the drive motor encoder readings to actual wheel
    // speeds.
    // Motor conversion factor is (PI * WHEEL DIAMETER IN METERS) / (GEAR RATIO *
    // ENCODER RESOLUTION).
    // In this case the wheel diameter is 4 inches, which must be converted to
    // meters to get meters/second.
    // The gear ratio is 6.75 motor revolutions per wheel rotation.
    // The encoder resolution per motor revolution is 1 per motor revolution.
    double driveConversionFactor = SwerveMath.calculateMetersPerRotation(ModuleConstants.kWheelRadius.in(Meters),
        1 / ModuleConstants.kSwerveModuleFinalGearRatio);

    // Configure the Telemetry before creating the SwerveDrive to avoid unnecessary
    // objects being created.
    // Adjust this value to control the amount of telemetry data that is printed to
    // the console. Turn it off or to low or pose for competition.
    SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;

    try {
      // Loads the conversion factors via JSON files
      swerveDrive = new SwerveParser(directory).createSwerveDrive(DriveConstants.kMaxSpeed.in(MetersPerSecond),
          new Pose2d(new Translation2d(Meter.of(1),
              Meter.of(4)),
              Rotation2d.fromDegrees(0)));
      // Alternative method if you don't want to supply the conversion factor via JSON
      // files.
      // swerveDrive = new SwerveParser(directory).createSwerveDrive(maximumSpeed,
      // angleConversionFactor, driveConversionFactor);
    } catch (Exception e) {
      // Code is broken - stop the robot
      throw new RuntimeException(e);
    }

    // Heading correction uses the heading PID from controllerproperties.json and
    // the current yaw to calculate an omega (angular velocity) turning speed using
    // SwerveController.headingCalculate
    // Calculating Omega (ω): The SwerveController.headingCalculate function
    // computes the necessary angular velocity (ω) to correct any deviation from the
    // desired heading. This calculation determines how quickly and in which
    // direction the robot should rotate to achieve the target orientation.
    // Applying the Correction: The computed ω is then used to adjust the swerve
    // modules' wheel angles and speeds, facilitating the desired rotational
    // movement.
    swerveDrive.setHeadingCorrection(swerveConfig.isHeadingCorrectionEnabled());

    // Cosine compensation is a technique used in swerve drive systems to enhance
    // control and efficiency by adjusting the speed of each wheel based on its
    // alignment with the desired movement direction.
    // In a swerve drive, each wheel can rotate independently to achieve precise
    // movement. However, when a wheel's orientation doesn't perfectly match the
    // intended direction, it can introduce inefficiencies or unintended forces.
    // To address this, cosine compensation scales the wheel's speed by the cosine
    // of the angle difference between its current orientation and the desired
    // direction.
    // This may cause unintended consequences. Hence, test it before fully enabling
    // it.
    // Will not work in simulation
    swerveDrive.setCosineCompensator(swerveConfig.isCosineCompensationEnabled());

    // Angular Velocity Compensation is a feature designed to mitigate the skewing
    // effect that can occur when a swerve-drive robot moves linearly while
    // simultaneously rotating.
    // This skewing results from the combined translational and rotational
    // movements, causing the robot to deviate from its intended path.
    // The primary goal of angular velocity compensation is to adjust the wheel
    // velocities in real-time to counteract the undesired lateral movements induced
    // by rotation.
    // By doing so, the robot maintains a straighter trajectory during combined
    // translational and rotational motions, enhancing overall maneuverability and
    // control.
    swerveDrive.setAngularVelocityCompensation(
        swerveConfig.isAngularVelocityCompensationEnabled(),
        swerveConfig.isAngularVelocityCompensationEnabled(),
        swerveConfig.getAngularVelocityCoeff());

    // Enable if you want to resynchronize your absolute encoders and motor encoders
    // periodically when they are not moving.
    // YAGSL will automatically synchronize the absolute encoders and internal
    // encoders of the angle/steering/azimuth motor controllers everytime the module
    // is at rest for half-a-second if the delta between the internal encoder and
    // absolute encoder is greater than the deadband.
    // Enable this after initial setup and tests; set the deadband in degrees as the
    // second parameter.
    swerveDrive.setModuleEncoderAutoSynchronize(false, 1.0);

    // Offset Offloading is where the absolute encoder offset is stored on the
    // absolute encoder (or Motor Controller if the absolute encoder is attached to
    // the motor controller) instead of the roboRIO. This normally results in a
    // faster loop cycle on the roboRIO however it can add some instability to the
    // Swerve Drive if anything breaks during a match, like a CAN bus or a low
    // brown-out.
    // swerveDrive.pushOffsetsToEncoders();

    if (swerveConfig.useManualOdometry()) {
      // Note: Vision must now be configured externally using setupVision(VisionSystem)
      // Stop the odometry thread if we are using vision that way we can synchronize
      // updates better. Disable this in simulation (maple-sim) for proper odometry updates.
      swerveDrive.stopOdometryThread();
    }

    setupPathPlanner();
  }

  /**
   * Construct the swerve drive with explicit configuration objects.
   *
   * <p>This constructor is for advanced use cases where you need direct control
   * over the SwerveDriveConfiguration and SwerveControllerConfiguration.
   *
   * @param driveCfg      SwerveDriveConfiguration for the swerve.
   * @param controllerCfg Swerve Controller.
   */
  public SwerveSubsystem(SwerveDriveConfiguration driveCfg, SwerveControllerConfiguration controllerCfg) {
    this(driveCfg, controllerCfg, new SwerveConfig());
  }

  /**
   * Construct the swerve drive with explicit configuration objects and SwerveConfig.
   *
   * @param driveCfg      SwerveDriveConfiguration for the swerve.
   * @param controllerCfg Swerve Controller.
   * @param config        SwerveConfig with PID and behavior settings.
   */
  public SwerveSubsystem(SwerveDriveConfiguration driveCfg, SwerveControllerConfiguration controllerCfg,
                         SwerveConfig config) {
    this.swerveConfig = config;
    swerveDrive = new SwerveDrive(driveCfg,
        controllerCfg,
        DriveConstants.kMaxSpeed.in(MetersPerSecond),
        new Pose2d(new Translation2d(Meters.of(2), Meters.of(0)),
            Rotation2d.fromDegrees(0)));
    setupPathPlanner();
  }

  /**
   * Setup the vision system with a VisionSystem implementation.
   *
   * <p>This method allows using any vision system that implements the {@link VisionSystem}
   * interface, such as PhotonVision, Limelight, or custom implementations.
   *
   * <p><strong>Usage Example:</strong>
   * <pre>{@code
   * // In RobotContainer - handling circular dependency
   * // 1. Create swerve first (can run basic odometry without vision)
   * SwerveSubsystem swerve = new SwerveSubsystem(directory, config);
   *
   * // 2. Create vision with reference to swerve
   * VisionSystem vision = createVisionSystem(visionConfig, swerve::getPose, swerve.getField());
   *
   * // 3. Pass vision back to swerve
   * swerve.setupVision(vision);
   * }</pre>
   *
   * @param visionSystem The vision system to use for pose estimation
   * @throws NullPointerException if visionSystem is null
   */
  public void setupVision(VisionSystem visionSystem) {
    this.vision = Objects.requireNonNull(visionSystem, "Vision system cannot be null");
  }

  /**
   * Checks if vision is configured and available.
   *
   * @return true if vision system has been set up
   */
  public boolean isVisionAvailable() {
    return vision != null;
  }

  private void setupPathPlanner() {
    // Load the RobotConfig from the GUI settings. You should probably
    // store this in your Constants file
    RobotConfig config;
    try {
      config = RobotConfig.fromGUISettings();

      final boolean enableFeedforward = swerveConfig.isFeedforwardEnabled();
      // Configure AutoBuilder last
      AutoBuilder.configure(
          this::getPose,
          // Robot pose supplier
          this::resetOdometry,
          // Method to reset odometry (will be called if your auto has a starting pose)
          this::getRobotVelocity,
          // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
          (speedsRobotRelative, moduleFeedForwards) -> {
            if (enableFeedforward) {
              swerveDrive.drive(
                  speedsRobotRelative,
                  swerveDrive.kinematics.toSwerveModuleStates(speedsRobotRelative),
                  moduleFeedForwards.linearForces());
            } else {
              swerveDrive.setChassisSpeeds(speedsRobotRelative);
            }
          },
          // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also
          // optionally outputs individual module feedforwards
          new PPHolonomicDriveController(
              // PPHolonomicController is the built in path following controller for holonomic
              // drive trains
              swerveConfig.getTranslationPID(), // Translation PID constants from SwerveConfig
              swerveConfig.getRotationPID() // Rotation PID constants from SwerveConfig
          ),
          config,
          // The robot configuration
          () -> {
            // Boolean supplier that controls when the path will be mirrored for the red
            // alliance
            // This will flip the path being followed to the red side of the field.
            // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

            return Utils.isOnRedAlliance();
          },
          this
      // Reference to this subsystem to set requirements
      );

    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
    }

    // Preload PathPlanner Path finding
    // IF USING CUSTOM PATHFINDER ADD BEFORE THIS LINE
    CommandScheduler.getInstance().schedule(PathfindingCommand.warmupCommand());
  }

  /**
   * Gets the current pose (position and rotation) of the robot, as reported by
   * odometry.
   *
   * @return The robot's pose
   */
  public Pose2d getPose() {
    return swerveDrive.getPose();
  }

  /**
   * Gets the current pitch angle of the robot, as reported by the imu.
   *
   * @return The heading as a {@link Rotation2d} angle
   */
  public Rotation2d getPitch() {
    return swerveDrive.getPitch();
  }

  /**
   * Resets odometry to the given pose. Gyro angle and module positions do not
   * need to be reset when calling this
   * method. However, if either gyro angle or module position is reset, this must
   * be called in order for odometry to
   * keep working.
   *
   * @param initialHolonomicPose The pose to set the odometry to
   */
  public void resetOdometry(Pose2d initialHolonomicPose) {
    swerveDrive.resetOdometry(initialHolonomicPose);
  }

  /**
   * Gets the current field-relative velocity (x, y and omega) of the robot
   *
   * @return A ChassisSpeeds object of the current field-relative velocity
   */
  public ChassisSpeeds getFieldVelocity() {
    return swerveDrive.getFieldVelocity();
  }

  public SwerveIMU getGyro() {
    return swerveDrive.getGyro();
  }

  /**
   * Gets the current velocity (x, y and omega) of the robot
   *
   * @return A {@link ChassisSpeeds} object of the current velocity
   */
  public ChassisSpeeds getRobotVelocity() {
    return swerveDrive.getRobotVelocity();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // Always update odometry - required for pose to update in both teleop and auto
    swerveDrive.updateOdometry();

    // Vision pose estimation only when manual odometry mode AND vision is configured
    // (vision requires synchronized updates which is why we stop the odometry thread)
    if (swerveConfig.useManualOdometry() && vision != null) {
      vision.updatePoseEstimation(swerveDrive);
    }
  }

  @Override
  public void simulationPeriodic() {
    // YAGSL handles simulation updates internally via its physics simulation thread.
    // No additional calls needed here for maple-sim compatibility.
  }

  /**
   * Get the distance to a specific AprilTag.
   *
   * @param tagID The ID of the AprilTag to get the distance to.
   * @return Distance to a specific AprilTag in meters.
   */
  public double getDistanceToAprilTag(int tagID) {
    Pose3d tagPose = aprilTagFieldLayout.getTagPose(tagID).get();
    return getPose().getTranslation().getDistance(tagPose.toPose2d().getTranslation());
  }

  /**
   * Get the yaw to aim at an AprilTag.
   *
   * @param tagID The ID of the AprilTag to aim at.
   * @return {@link Rotation2d} of which you need to achieve.
   */
  public Rotation2d getAprilTagYaw(int tagID) {
    Pose3d tagPose = aprilTagFieldLayout.getTagPose(tagID).get();
    Translation2d relativeTrl = tagPose.toPose2d().relativeTo(getPose()).getTranslation();
    return new Rotation2d(relativeTrl.getX(), relativeTrl.getY()).plus(swerveDrive.getOdometryHeading());
  }

  /**
   * Retrieves the SwerveDrive instance from the subsystem. Use cautiously.
   *
   * @return the SwerveDrive instance
   */
  public SwerveDrive getSwerveDrive() {
    return swerveDrive;
  }

  /**
   * Get the {@link SwerveDriveConfiguration} object.
   *
   * @return The {@link SwerveDriveConfiguration} fpr the current drive.
   */
  public SwerveDriveConfiguration getSwerveDriveConfiguration() {
    return swerveDrive.swerveDriveConfiguration;
  }

  /**
   * Get the {@link SwerveConfig} object used to configure this subsystem.
   *
   * @return The SwerveConfig with PID and behavior settings
   */
  public SwerveConfig getSwerveConfig() {
    return swerveConfig;
  }

  /**
   * Gets the current yaw angle of the robot, as reported by the swerve pose
   * estimator in the underlying drivebase.
   * Note, this is not the raw gyro reading, this may be corrected from calls to
   * resetOdometry().
   *
   * @return The yaw angle
   */
  public Rotation2d getHeading() {
    return getPose().getRotation();
  }

  /**
   * The primary method for controlling the drivebase. Takes a
   * {@link Translation2d} and a rotation rate, and
   * calculates and commands module states accordingly. Can use either open-loop
   * or closed-loop velocity control for
   * the wheel velocities. Also has field- and robot-relative modes, which affect
   * how the translation vector is used.
   *
   * @param translation   {@link Translation2d} that is the commanded linear
   *                      velocity of the robot, in meters per
   *                      second. In robot-relative mode, positive x is torwards
   *                      the bow (front) and positive y is
   *                      torwards port (left). In field-relative mode, positive x
   *                      is away from the alliance wall
   *                      (field North) and positive y is torwards the left wall
   *                      when looking through the driver station
   *                      glass (field West).
   * @param rotation      Robot angular rate, in radians per second. CCW positive.
   *                      Unaffected by field/robot
   *                      relativity.
   * @param fieldRelative Drive mode. True for field-relative, false for
   *                      robot-relative.
   */
  public void drive(Translation2d translation, double rotation, boolean fieldRelative) {
    swerveDrive.drive(translation,
        rotation,
        fieldRelative,
        false); // Open loop is disabled since it shouldn't be used most of the time.
  }

  /**
   * Drive the robot given a chassis field oriented velocity.
   *
   * @param velocity Velocity according to the field.
   */
  public void driveFieldOriented(ChassisSpeeds velocity) {
    swerveDrive.driveFieldOriented(velocity);
  }

  /**
   * Get the chassis speeds based on controller input of 1 joystick and one angle.
   * Control the robot at an offset of
   * 90deg.
   *
   * @param xInput X joystick input for the robot to move in the X direction.
   * @param yInput Y joystick input for the robot to move in the Y direction.
   * @param angle  The angle in as a {@link Rotation2d}.
   * @return {@link ChassisSpeeds} which can be sent to the Swerve Drive.
   */
  public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, Rotation2d angle) {
    Translation2d scaledInputs = SwerveMath.cubeTranslation(new Translation2d(xInput, yInput));

    return swerveDrive.swerveController.getTargetSpeeds(scaledInputs.getX(),
        scaledInputs.getY(),
        angle.getRadians(),
        getHeading().getRadians(),
        DriveConstants.kMaxSpeed.in(MetersPerSecond));
  }

  /**
   * Drive according to the chassis robot oriented velocity.
   *
   * @param velocity Robot oriented {@link ChassisSpeeds}
   */
  public void drive(ChassisSpeeds velocity) {
    swerveDrive.drive(velocity);
  }

  /**
   * Lock the swerve drive to prevent it from moving.
   */
  public void lock() {
    swerveDrive.lockPose();
  }

  /**
   * Get the swerve drive kinematics object.
   *
   * @return {@link SwerveDriveKinematics} of the swerve drive.
   */
  public SwerveDriveKinematics getKinematics() {
    return swerveDrive.kinematics;
  }

  /**
   * Set chassis speeds with closed-loop velocity control.
   *
   * @param chassisSpeeds Chassis Speeds to set.
   */
  public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
    swerveDrive.setChassisSpeeds(chassisSpeeds);
  }

  /**
   * Post the trajectory to the field.
   *
   * @param trajectory The trajectory to post.
   */
  public void postTrajectory(Trajectory trajectory) {
    swerveDrive.postTrajectory(trajectory);
  }

  /**
   * Resets the gyro angle to zero and resets odometry to the same position, but
   * facing toward 0.
   */
  public void zeroGyro() {
    swerveDrive.zeroGyro();
  }

  /**
   * This will zero (calibrate) the robot to assume the current position is facing
   * forward
   * <p>
   * If red alliance rotate the robot 180 after the drviebase zero command
   */
  public void zeroGyroWithAlliance() {
    if (Utils.isOnRedAlliance()) {
      zeroGyro();
      // Set the pose 180 degrees
      resetOdometry(new Pose2d(getPose().getTranslation(), Rotation2d.fromDegrees(180)));
    } else {
      zeroGyro();
    }
  }

  /**
   * Sets the drive motors to brake/coast mode.
   *
   * @param brake True to set motors to brake mode, false for coast.
   */
  public void setMotorBrake(boolean brake) {
    swerveDrive.setMotorIdleMode(brake);
  }

  /**
   * Checks if manual odometry mode is enabled.
   *
   * <p>When true, YAGSL's odometry thread is stopped and odometry is updated
   * manually in periodic() for synchronized vision updates.
   *
   * @return true if manual odometry mode is enabled, false otherwise.
   */
  public boolean isManualOdometryEnabled() {
    return swerveConfig.useManualOdometry();
  }

  /**
   * Get the vision system instance.
   *
   * @return The VisionSystem instance, or null if not configured.
   */
  public VisionSystem getVision() {
    return vision;
  }

  /**
   * Add a fake vision reading for testing purposes.
   */
  public void addFakeVisionReading() {
    swerveDrive.addVisionMeasurement(new Pose2d(3, 3, Rotation2d.fromDegrees(65)), Timer.getFPGATimestamp());
  }

  // ==================== TRIGGER METHODS ====================

  /**
   * Trigger that is true when robot is within tolerance of target pose.
   *
   * @param targetPose Target pose to check against
   * @param positionToleranceMeters Position tolerance in meters
   * @param rotationToleranceDegrees Rotation tolerance in degrees
   * @return Trigger for pose proximity
   */
  public Trigger atPoseTrigger(Pose2d targetPose, double positionToleranceMeters, double rotationToleranceDegrees) {
    return new Trigger(() -> {
      Pose2d current = getPose();
      double distance = current.getTranslation().getDistance(targetPose.getTranslation());
      double rotationError = Math.abs(
          current.getRotation().minus(targetPose.getRotation()).getDegrees()
      );
      return distance <= positionToleranceMeters && rotationError <= rotationToleranceDegrees;
    });
  }

  /**
   * Trigger that is true when robot is within rectangular field region.
   *
   * @param minX Minimum X coordinate (meters)
   * @param maxX Maximum X coordinate (meters)
   * @param minY Minimum Y coordinate (meters)
   * @param maxY Maximum Y coordinate (meters)
   * @return Trigger for region containment
   */
  public Trigger inRegionTrigger(double minX, double maxX, double minY, double maxY) {
    return new Trigger(() -> {
      Pose2d pose = getPose();
      double x = pose.getX();
      double y = pose.getY();
      return x >= minX && x <= maxX && y >= minY && y <= maxY;
    });
  }

  /**
   * Trigger that is true when robot is stationary (linear velocity below threshold).
   *
   * @param velocityThreshold Velocity threshold in m/s
   * @return Trigger for stationary state
   */
  public Trigger isStationaryTrigger(double velocityThreshold) {
    return new Trigger(() -> {
      ChassisSpeeds speeds = getRobotVelocity();
      double velocity = Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
      return velocity < velocityThreshold;
    });
  }

  /**
   * Convenience trigger that is true when robot is stopped.
   *
   * <p>Uses default thresholds: 0.05 m/s linear, 0.1 rad/s angular.
   *
   * @return Trigger for stopped state
   */
  public Trigger isStoppedTrigger() {
    return isFullyStoppedTrigger(0.05, 0.1);
  }

  /**
   * Trigger that is true when robot is moving (linear velocity above threshold).
   *
   * @param velocityThreshold Velocity threshold in m/s
   * @return Trigger for moving state
   */
  public Trigger isMovingTrigger(double velocityThreshold) {
    return new Trigger(() -> {
      ChassisSpeeds speeds = getRobotVelocity();
      double velocity = Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
      return velocity >= velocityThreshold;
    });
  }

  /**
   * Convenience trigger that is true when robot is moving.
   *
   * <p>Uses default threshold of 0.1 m/s.
   *
   * @return Trigger for moving state
   */
  public Trigger isMovingTrigger() {
    return isMovingTrigger(0.1);
  }

  /**
   * Trigger that is true when robot exceeds velocity threshold.
   *
   * @param velocityThreshold Velocity threshold in m/s
   * @return Trigger for high-speed state
   * @deprecated Use {@link #isMovingTrigger(double)} instead
   */
  @Deprecated
  public Trigger isMovingFastTrigger(double velocityThreshold) {
    return isMovingTrigger(velocityThreshold);
  }

  /**
   * Trigger that is true when aligned with AprilTag within tolerance.
   *
   * @param tagID AprilTag ID to align with
   * @param tolerance Angular tolerance
   * @return Trigger for tag alignment
   */
  public Trigger isAlignedWithTagTrigger(int tagID, Angle tolerance) {
    return new Trigger(() -> {
      if (vision == null || !vision.hasTarget()) return false;
      double error = Math.abs(getAprilTagYaw(tagID).minus(getHeading()).getDegrees());
      return error <= tolerance.in(Degrees);
    });
  }

  /**
   * Trigger that is true when aligned with AprilTag within tolerance.
   *
   * @param tagID AprilTag ID to align with
   * @param toleranceDegrees Angular tolerance in degrees
   * @return Trigger for tag alignment
   * @deprecated Use {@link #isAlignedWithTagTrigger(int, Angle)} instead
   */
  @Deprecated
  public Trigger isAlignedWithTagTrigger(int tagID, double toleranceDegrees) {
    return isAlignedWithTagTrigger(tagID, Degrees.of(toleranceDegrees));
  }

  /**
   * Trigger that is true when any AprilTag is visible.
   *
   * @return Trigger for vision target detection
   */
  public Trigger hasVisionTargetTrigger() {
    return new Trigger(() -> vision != null && vision.hasTarget());
  }

  /**
   * Trigger that is true when within distance range of AprilTag.
   *
   * @param tagID AprilTag ID to check
   * @param minDistance Minimum distance in meters
   * @param maxDistance Maximum distance in meters
   * @return Trigger for distance range
   */
  public Trigger isInRangeOfTagTrigger(int tagID, double minDistance, double maxDistance) {
    return new Trigger(() -> {
      if (vision == null) return false;
      double distance = vision.getDistanceFromAprilTag(tagID);
      return distance >= minDistance && distance <= maxDistance;
    });
  }

  /**
   * Trigger that is true when robot is facing target heading.
   *
   * @param targetHeading Target heading as Rotation2d
   * @param toleranceDegrees Tolerance in degrees
   * @return Trigger for heading alignment
   */
  public Trigger isFacingHeadingTrigger(Rotation2d targetHeading, double toleranceDegrees) {
    return new Trigger(() -> {
      double error = Math.abs(getHeading().minus(targetHeading).getDegrees());
      return error <= toleranceDegrees;
    });
  }

  /**
   * Trigger that is true when robot angular velocity exceeds threshold.
   *
   * <p>Useful for detecting when the robot is actively rotating, which may
   * affect vision accuracy or require different control strategies.
   *
   * @param thresholdRadPerSec Angular velocity threshold in radians per second
   * @return Trigger for rotation detection
   */
  public Trigger isRotatingTrigger(double thresholdRadPerSec) {
    return new Trigger(() -> {
      ChassisSpeeds speeds = getRobotVelocity();
      return Math.abs(speeds.omegaRadiansPerSecond) > thresholdRadPerSec;
    });
  }

  /**
   * Trigger that is true when robot angular velocity is below threshold.
   *
   * <p>Useful for detecting when the robot has settled after a rotation,
   * which may be needed before taking vision measurements or scoring.
   *
   * @param thresholdRadPerSec Angular velocity threshold in radians per second
   * @return Trigger for rotation settled detection
   */
  public Trigger isNotRotatingTrigger(double thresholdRadPerSec) {
    return new Trigger(() -> {
      ChassisSpeeds speeds = getRobotVelocity();
      return Math.abs(speeds.omegaRadiansPerSecond) <= thresholdRadPerSec;
    });
  }

  /**
   * Trigger that is true when robot is moving in specified field direction.
   *
   * <p>Useful for detecting movement direction for game piece handling,
   * such as detecting when moving toward a scoring location.
   *
   * <p><strong>Usage Example:</strong>
   * <pre>{@code
   * // Trigger when moving toward blue alliance wall (+X direction)
   * swerve.isMovingInDirectionTrigger(Rotation2d.fromDegrees(0), 45, 0.1)
   *     .onTrue(Commands.print("Moving forward"));
   *
   * // Trigger when moving left (field +Y direction)
   * swerve.isMovingInDirectionTrigger(Rotation2d.fromDegrees(90), 30, 0.2)
   *     .whileTrue(intake.runCommand());
   * }</pre>
   *
   * @param direction Target field direction as Rotation2d (0° = +X toward red alliance)
   * @param toleranceDegrees Angular tolerance in degrees
   * @param minVelocity Minimum velocity in m/s to consider as "moving"
   * @return Trigger for directional movement detection
   */
  public Trigger isMovingInDirectionTrigger(Rotation2d direction, double toleranceDegrees, double minVelocity) {
    return new Trigger(() -> {
      ChassisSpeeds fieldSpeeds = getFieldVelocity();
      double velocity = Math.hypot(fieldSpeeds.vxMetersPerSecond, fieldSpeeds.vyMetersPerSecond);

      if (velocity < minVelocity) {
        return false;
      }

      // Calculate actual movement direction
      Rotation2d movementDirection = new Rotation2d(fieldSpeeds.vxMetersPerSecond, fieldSpeeds.vyMetersPerSecond);
      double error = Math.abs(movementDirection.minus(direction).getDegrees());

      return error <= toleranceDegrees;
    });
  }

  /**
   * Convenience trigger for detecting forward movement (field +X direction).
   *
   * <p>Uses 45° tolerance and 0.1 m/s minimum velocity.
   *
   * @return Trigger for forward movement detection
   */
  public Trigger isMovingForwardTrigger() {
    return isMovingInDirectionTrigger(Rotation2d.fromDegrees(0), 45, 0.1);
  }

  /**
   * Trigger that is true when robot is strafing (moving perpendicular to heading).
   *
   * <p>Useful for detecting lateral movement during alignment maneuvers.
   *
   * @param minVelocity Minimum velocity in m/s to consider as "moving"
   * @return Trigger for strafe detection
   */
  public Trigger isStrafingTrigger(double minVelocity) {
    return new Trigger(() -> {
      ChassisSpeeds fieldSpeeds = getFieldVelocity();
      double velocity = Math.hypot(fieldSpeeds.vxMetersPerSecond, fieldSpeeds.vyMetersPerSecond);

      if (velocity < minVelocity) {
        return false;
      }

      // Calculate movement direction relative to robot heading
      Rotation2d movementDirection = new Rotation2d(fieldSpeeds.vxMetersPerSecond, fieldSpeeds.vyMetersPerSecond);
      Rotation2d heading = getHeading();
      double angleFromHeading = Math.abs(movementDirection.minus(heading).getDegrees());

      // Strafing is when movement is roughly perpendicular to heading (60-120 degrees)
      return angleFromHeading >= 60 && angleFromHeading <= 120;
    });
  }

  /**
   * Trigger that is true when robot is within distance range of a field point.
   *
   * <p>Useful for region-based triggers around scoring locations, pickup zones,
   * or defensive positions.
   *
   * <p><strong>Usage Example:</strong>
   * <pre>{@code
   * // Trigger when within 1-3 meters of a scoring location
   * Translation2d scoringSpot = new Translation2d(2.0, 5.5);
   * swerve.distanceFromPointTrigger(scoringSpot, 1.0, 3.0)
   *     .whileTrue(swerve.aimAtAprilTagCommand(7, 2.0));
   * }</pre>
   *
   * @param point Field point to measure distance from
   * @param minDistance Minimum distance in meters (inclusive)
   * @param maxDistance Maximum distance in meters (inclusive)
   * @return Trigger for distance range detection
   */
  public Trigger distanceFromPointTrigger(Translation2d point, double minDistance, double maxDistance) {
    return new Trigger(() -> {
      double distance = getPose().getTranslation().getDistance(point);
      return distance >= minDistance && distance <= maxDistance;
    });
  }

  /**
   * Trigger that is true when robot pitch exceeds threshold.
   *
   * <p>Useful for detecting when the robot is on a ramp, incline, or climbing.
   * This can be used to adjust driving behavior or trigger climbing sequences.
   *
   * @param thresholdDegrees Pitch threshold in degrees (absolute value)
   * @return Trigger for pitch detection
   */
  public Trigger isPitchedTrigger(double thresholdDegrees) {
    return new Trigger(() -> {
      double pitch = Math.abs(getPitch().getDegrees());
      return pitch > thresholdDegrees;
    });
  }

  /**
   * Trigger that is true when both linear AND angular velocity are below thresholds.
   *
   * <p>Useful for detecting when the robot has fully stopped, which is
   * important for vision measurements, scoring, or state transitions.
   *
   * <p>This is stricter than isStationaryTrigger, which only checks linear velocity.
   *
   * @param linearThreshold Linear velocity threshold in m/s
   * @param angularThreshold Angular velocity threshold in rad/s
   * @return Trigger for fully stopped detection
   */
  public Trigger isFullyStoppedTrigger(double linearThreshold, double angularThreshold) {
    return new Trigger(() -> {
      ChassisSpeeds speeds = getRobotVelocity();
      double linearVelocity = Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
      double angularVelocity = Math.abs(speeds.omegaRadiansPerSecond);
      return linearVelocity < linearThreshold && angularVelocity < angularThreshold;
    });
  }

  // ==================== GAME TARGET CONFIGURATION ====================

  /**
   * Sets up game targets from a JSON file in the deploy directory.
   *
   * <p>The JSON file should define game-specific targets like scoring locations,
   * pickup zones, etc. Target positions are calculated by combining AprilTag
   * positions from the field layout with offsets defined in the JSON.
   *
   * <p><strong>Usage Example:</strong>
   * <pre>{@code
   * // In RobotContainer constructor
   * swerve.setupGameTargets("gametargets-2026-rebuilt.json");
   *
   * // Then use targets
   * swerve.driveToTargetCommand("tower-blue").schedule();
   * }</pre>
   *
   * @param jsonFilename Name of JSON file in deploy directory
   * @see GameTargetLoader
   * @see GameTargetConfig
   */
  public void setupGameTargets(String jsonFilename) {
    gameTargetConfig = GameTargetLoader.load(jsonFilename);
  }

  /**
   * Sets up game targets using a programmatically built configuration.
   *
   * <p>Use this method when you prefer to define targets in code rather than JSON.
   *
   * <p><strong>Usage Example:</strong>
   * <pre>{@code
   * GameTargetConfig config = GameTargetConfigBuilder.create()
   *     .gameYear(2026)
   *     .gameName("REBUILT")
   *     .addTarget("tower-blue").tagIds(15, 16).offsetMeters(-0.5, 0, 180).done()
   *     .addTarget("tower-red").tagIds(31, 32).offsetMeters(-0.5, 0, 0).done()
   *     .build();
   *
   * swerve.setupGameTargets(config);
   * }</pre>
   *
   * @param config GameTargetConfig built using GameTargetConfigBuilder
   * @see GameTargetConfigBuilder
   */
  public void setupGameTargets(GameTargetConfig config) {
    gameTargetConfig = config;
  }

  /**
   * Gets the current game target configuration.
   *
   * @return GameTargetConfig or null if not configured
   */
  public GameTargetConfig getGameTargetConfig() {
    return gameTargetConfig;
  }

  /**
   * Gets the pose for a named game target.
   *
   * @param targetName Target name (e.g., "tower-blue")
   * @return Target pose, or empty if target not found or tag not in layout
   */
  public Optional<Pose2d> getTargetPose(String targetName) {
    if (gameTargetConfig == null) {
      return Optional.empty();
    }
    return gameTargetConfig.getTargetPose(aprilTagFieldLayout, targetName);
  }

  /**
   * Gets the alliance-specific target pose.
   *
   * <p>Automatically appends "-blue" or "-red" to the base name based on
   * the current alliance color.
   *
   * <p><strong>Usage Example:</strong>
   * <pre>{@code
   * // If on blue alliance, gets "tower-blue"
   * // If on red alliance, gets "tower-red"
   * Optional<Pose2d> towerPose = swerve.getAllianceTargetPose("tower");
   * }</pre>
   *
   * @param baseName Base target name (without alliance suffix)
   * @return Target pose for current alliance, or empty if not found
   */
  public Optional<Pose2d> getAllianceTargetPose(String baseName) {
    if (gameTargetConfig == null) {
      return Optional.empty();
    }
    return gameTargetConfig.getAllianceTargetPose(aprilTagFieldLayout, baseName, Utils.isOnRedAlliance());
  }

  // ==================== GAME TARGET TRIGGERS ====================

  /**
   * Trigger that is true when robot is at the specified game target.
   *
   * <p>Uses the tolerance values defined in the target configuration.
   *
   * <p><strong>Usage Example:</strong>
   * <pre>{@code
   * // Trigger when at the tower scoring position
   * swerve.isAtTargetTrigger("tower-blue")
   *     .onTrue(shooter.shootCommand());
   * }</pre>
   *
   * @param targetName Name of the target to check
   * @return Trigger for target proximity, always false if target not configured
   */
  public Trigger isAtTargetTrigger(String targetName) {
    return new Trigger(() -> {
      if (gameTargetConfig == null) {
        return false;
      }
      Optional<GameTarget> target = gameTargetConfig.getTarget(targetName);
      return target.map(t -> t.isAtTarget(aprilTagFieldLayout, getPose())).orElse(false);
    });
  }

  /**
   * Trigger that is true when robot is at the alliance-specific target.
   *
   * @param baseName Base target name (without alliance suffix)
   * @return Trigger for alliance target proximity
   */
  public Trigger isAtAllianceTargetTrigger(String baseName) {
    return new Trigger(() -> {
      if (gameTargetConfig == null) {
        return false;
      }
      Optional<GameTarget> target = gameTargetConfig.getAllianceTarget(baseName, Utils.isOnRedAlliance());
      return target.map(t -> t.isAtTarget(aprilTagFieldLayout, getPose())).orElse(false);
    });
  }

  /**
   * Trigger that is true when robot is within distance range of a game target.
   *
   * <p><strong>Usage Example:</strong>
   * <pre>{@code
   * // Start aiming when within 1-3 meters of tower
   * swerve.isInRangeOfTargetTrigger("tower-blue", 1.0, 3.0)
   *     .whileTrue(swerve.aimAtTargetCommand("tower-blue", 2.0));
   * }</pre>
   *
   * @param targetName Name of the target
   * @param minDistance Minimum distance in meters
   * @param maxDistance Maximum distance in meters
   * @return Trigger for distance range, always false if target not configured
   */
  public Trigger isInRangeOfTargetTrigger(String targetName, double minDistance, double maxDistance) {
    return new Trigger(() -> {
      if (gameTargetConfig == null) {
        return false;
      }
      Optional<GameTarget> target = gameTargetConfig.getTarget(targetName);
      if (target.isEmpty()) {
        return false;
      }
      double distance = target.get().getDistance(aprilTagFieldLayout, getPose());
      return distance >= 0 && distance >= minDistance && distance <= maxDistance;
    });
  }

  /**
   * Trigger that is true when robot heading is aligned with target rotation.
   *
   * <p>Checks if the robot is facing the direction specified in the target's offset rotation.
   *
   * @param targetName Name of the target
   * @param tolerance Angular tolerance
   * @return Trigger for rotational alignment, always false if target not configured
   */
  public Trigger isAlignedWithTargetTrigger(String targetName, Angle tolerance) {
    return new Trigger(() -> {
      if (gameTargetConfig == null) {
        return false;
      }
      Optional<Pose2d> targetPose = getTargetPose(targetName);
      if (targetPose.isEmpty()) {
        return false;
      }
      double error = Math.abs(getHeading().minus(targetPose.get().getRotation()).getDegrees());
      return error <= tolerance.in(Degrees);
    });
  }

  // ==================== COMMAND FACTORIES ====================

  // ==================== VISION-BASED COMMANDS ====================

  /**
   * Command to rotate the robot to aim at a specific AprilTag.
   *
   * <p><strong>How It Works:</strong> Uses the heading PID controller to calculate
   * the angular velocity needed to align the robot's heading with the AprilTag.
   * The command completes when the angular error is within the specified tolerance.
   *
   * <p><strong>Note:</strong> This command only rotates the robot. It does not
   * move toward or away from the tag. Combine with translation for full alignment.
   *
   * <p><strong>Usage Example:</strong>
   * <pre>{@code
   * // Aim at target tag when button is held
   * Buttons.XboxLeftTriggerButton.whileTrue(swerve.aimAtAprilTagCommand(7, 2.0));
   *
   * // Aim then score sequence
   * Commands.sequence(
   *     swerve.aimAtAprilTagCommand(7, 1.0),  // Aim within 1 degree
   *     scorer.scoreCommand()
   * );
   *
   * // Continuous aiming while driving (combine with drive command)
   * Buttons.XboxRightTriggerButton.whileTrue(
   *     Commands.parallel(
   *         swerve.aimAtAprilTagCommand(7, 2.0),
   *         mechanism.prepareCommand()
   *     )
   * );
   * }</pre>
   *
   * @param tagId AprilTag fiducial ID to aim at (see field layout for IDs)
   * @param tolerance Angular tolerance in degrees (command ends when within this)
   * @return Command that rotates robot to face the specified AprilTag
   *
   * @see #aimAtTargetCommand(VisionCameraInterface)
   * @see #alignAndStrafeCommand(int, double, double, double)
   */
  public Command aimAtAprilTagCommand(int tagId, double tolerance) {
    return Commands.run(() -> {
      ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(0, 0,
          swerveDrive.swerveController.headingCalculate(
              getHeading().getRadians(),
              getAprilTagYaw(tagId).getRadians()),
          getHeading());
      drive(speeds);
    }, this).until(() ->
        Math.abs(getAprilTagYaw(tagId).minus(getHeading()).getDegrees()) < tolerance
    ).withName("AimAtTag(" + tagId + ")");
  }

  /**
   * Command to log the distance from a specific AprilTag.
   *
   * @param tagID AprilTag ID to measure distance to
   * @return Command that logs distance
   */
  public Command getDistanceFromAprilTagCommand(int tagID) {
    return Commands.runOnce(() -> {
      if (vision == null) return;
      edu.wpi.first.wpilibj.DataLogManager.log("AprilTag " + tagID + " distance - X: " +
          vision.getTransformToAprilTag(tagID).getX() +
          " Y: " + vision.getTransformToAprilTag(tagID).getY());
    }).withName("GetDistanceFromTag(" + tagID + ")");
  }

  /**
   * Command to align with an AprilTag then strafe sideways.
   *
   * <p><strong>What This Does:</strong> A two-phase command that:
   * <ol>
   *   <li>Rotates the robot to face the specified AprilTag</li>
   *   <li>Strafes left or right by the specified distance</li>
   * </ol>
   *
   * <p><strong>Use Cases:</strong>
   * <ul>
   *   <li>Lining up with a scoring position offset from a tag</li>
   *   <li>Moving to pickup positions relative to field elements</li>
   *   <li>Automated alignment sequences</li>
   * </ul>
   *
   * <p><strong>Strafe Direction:</strong>
   * <ul>
   *   <li>Positive strafeDistance → Move left (field +Y direction)</li>
   *   <li>Negative strafeDistance → Move right (field -Y direction)</li>
   * </ul>
   *
   * <p><strong>Usage Example:</strong>
   * <pre>{@code
   * // Align with target tag, then strafe 0.5m left to score position
   * Buttons.XboxAButton.onTrue(
   *     swerve.alignAndStrafeCommand(7, 0.5, 1.0, 2.0)
   * );
   *
   * // Align with tag, strafe right to scoring position
   * Commands.sequence(
   *     swerve.alignAndStrafeCommand(5, -0.3, 0.5, 1.0),
   *     scorer.scoreCommand()
   * );
   * }</pre>
   *
   * @param tagId              The AprilTag fiducial ID to align with
   * @param strafeDistance     Distance to strafe in meters (positive = left)
   * @param strafeSpeed        Speed to strafe at in meters per second
   * @param alignmentTolerance Angular tolerance in degrees for initial alignment
   * @return Command sequence that aligns then strafes
   *
   * @see #aimAtAprilTagCommand(int, double)
   */
  public Command alignAndStrafeCommand(int tagId, double strafeDistance, double strafeSpeed,
                                       double alignmentTolerance) {
    return Commands.sequence(
        // First align with the AprilTag
        aimAtAprilTagCommand(tagId, alignmentTolerance),

        // Then strafe the specified distance
        Commands.sequence(
            // Record start position
            Commands.runOnce(() -> {
              startPose = swerveDrive.getPose();
            }),

            // Drive sideways
            Commands.run(() -> {
              // Sign of strafeDistance determines direction (positive = left)
              double speedY = Math.copySign(strafeSpeed, strafeDistance);
              drive(
                  new Translation2d(0, speedY), // Only Y movement
                  0, // No rotation
                  true // Field relative
              );
            }, this)
                .until(() -> {
                  // Calculate distance traveled sideways
                  double distanceTraveled = Math.abs(
                      swerveDrive.getPose().getTranslation().getY() -
                          startPose.getTranslation().getY());
                  return distanceTraveled >= Math.abs(strafeDistance);
                }),

            // Stop moving
            Commands.runOnce(() -> drive(new ChassisSpeeds(0, 0, 0)))
        )
    ).withName("AlignAndStrafe(" + tagId + ")");
  }

  /**
   * Command to aim robot at the best target from a specific camera.
   *
   * @param camera VisionCameraInterface to use
   * @return Command that aims at best target from camera
   */
  public Command aimAtTargetCommand(VisionCameraInterface camera) {
    return Commands.run(() -> {
      Optional<? extends VisionResult> resultO = camera.getBestResult();
      if (resultO.isPresent()) {
        var result = resultO.get();
        if (result.hasTargets()) {
          drive(getTargetSpeeds(0, 0,
              Rotation2d.fromDegrees(result.getBestTarget().getYaw())));
        }
      }
    }, this).withName("AimAtTarget(" + camera.getName() + ")");
  }

  /**
   * Command to aim robot at the best target from a camera by name.
   *
   * @param cameraName Name of the camera to use
   * @return Command that aims at best target from camera
   */
  public Command aimAtTargetCommand(String cameraName) {
    return Commands.run(() -> {
      if (vision == null) return;
      VisionCameraInterface camera = vision.getCamera(cameraName);
      if (camera == null) return;

      Optional<? extends VisionResult> resultO = camera.getBestResult();
      if (resultO.isPresent()) {
        var result = resultO.get();
        if (result.hasTargets()) {
          drive(getTargetSpeeds(0, 0,
              Rotation2d.fromDegrees(result.getBestTarget().getYaw())));
        }
      }
    }, this).withName("AimAtTarget(" + cameraName + ")");
  }

  /**
   * Command to drive to the nearest pose from a list, continuously updating with vision.
   *
   * @param targetPoses List of poses to potentially drive to
   * @return Command that drives to nearest pose with vision updates
   */
  public Command driveToNearestPoseWithVisionCommand(List<Pose2d> targetPoses) {
    return Commands.run(() -> {
      if (vision == null) return;

      // Get current robot pose
      Pose2d currentPose = swerveDrive.getPose();

      // Find nearest target pose
      Pose2d nearestPose = findNearestPose(targetPoses);

      // Get visible AprilTags and their poses
      boolean hasVisibleTags = false;
      for (VisionCameraInterface camera : vision.getCameras()) {
        if (!camera.isEnabled()) continue;
        var result = camera.getLatestResult();
        if (result.isPresent() && result.get().hasTargets()) {
          hasVisibleTags = true;
          // Vision updates are handled by periodic() in SwerveSubsystem

          // If pose changed significantly, recalculate path
          if (poseChangedSignificantly(currentPose, swerveDrive.getPose())) {
            CommandScheduler.getInstance().schedule(driveToPoseCommand(nearestPose));
          }
        }
      }

      // If no tags visible, continue with last known path
      if (!hasVisibleTags) {
        CommandScheduler.getInstance().schedule(driveToPoseCommand(nearestPose));
      }
    }, this)
        .until(() -> hasReachedPose(findNearestPose(targetPoses), 0.05, 5.0))
        .withName("DriveToNearestPoseWithVision");
  }

  // ==================== VISION CONTROL COMMANDS ====================

  /**
   * Command to enable vision-based pose estimation.
   *
   * <p><strong>What This Does:</strong> Enables all configured vision cameras
   * to contribute pose estimates to the robot's odometry. When enabled, AprilTag
   * detections will correct accumulated odometry drift.
   *
   * <p><strong>When to Enable Vision:</strong>
   * <ul>
   *   <li>During autonomous for accurate positioning</li>
   *   <li>When approaching scoring positions</li>
   *   <li>After any maneuver that might cause odometry drift</li>
   * </ul>
   *
   * <p><strong>When to Disable Vision:</strong>
   * <ul>
   *   <li>During fast spinning maneuvers (vision can be noisy)</li>
   *   <li>When cameras are blocked or pointed at the ground</li>
   *   <li>If experiencing vision-related pose jumps</li>
   * </ul>
   *
   * <p><strong>Usage Example:</strong>
   * <pre>{@code
   * // Enable vision when entering scoring zone
   * swerve.inRegionTrigger(0, 5, 0, 8).onTrue(swerve.enableVisionCommand());
   *
   * // Toggle vision with a button
   * Buttons.XboxBackButton.onTrue(swerve.enableVisionCommand());
   * Buttons.XboxStartButton.onTrue(swerve.disableVisionCommand());
   * }</pre>
   *
   * @return Instant command that enables all vision cameras
   *
   * @see #disableVisionCommand()
   * @see #setupVision(VisionSystem)
   */
  public Command enableVisionCommand() {
    return Commands.runOnce(() -> {
      if (vision != null) vision.enableAllCameras();
    }, this).withName("EnableVision");
  }

  /**
   * Command to disable vision-based pose estimation (odometry only).
   *
   * <p><strong>What This Does:</strong> Disables all vision cameras from
   * contributing to pose estimation. The robot will rely solely on wheel odometry
   * and gyro for position tracking.
   *
   * <p><strong>Why Disable Vision:</strong>
   * <ul>
   *   <li>During fast rotations where vision can cause pose jumps</li>
   *   <li>When cameras have obstructed views</li>
   *   <li>If vision is causing erratic behavior</li>
   *   <li>When you need predictable, smooth odometry</li>
   * </ul>
   *
   * <p><strong>Note:</strong> Odometry will drift over time without vision
   * corrections. Re-enable vision when accuracy is needed.
   *
   * <p><strong>Usage Example:</strong>
   * <pre>{@code
   * // Disable vision during spin move
   * Commands.sequence(
   *     swerve.disableVisionCommand(),
   *     spinMoveCommand,
   *     swerve.enableVisionCommand()
   * );
   * }</pre>
   *
   * @return Instant command that disables all vision cameras
   *
   * @see #enableVisionCommand()
   */
  public Command disableVisionCommand() {
    return Commands.runOnce(() -> {
      if (vision != null) vision.disableAllCameras();
    }, this).withName("DisableVision");
  }

  /**
   * Command to add fake vision reading for testing.
   *
   * @return Command that adds test vision pose
   */
  public Command addFakeVisionReadingCommand() {
    return Commands.runOnce(() -> addFakeVisionReading(), this)
        .withName("AddFakeVisionReading");
  }

  // ==================== PATHPLANNER COMMANDS ====================

  /**
   * Command to drive to a specific pose using PathPlanner's on-the-fly pathfinding.
   *
   * <p><strong>How Pathfinding Works:</strong> PathPlanner generates an optimal path
   * from the robot's current position to the target pose, avoiding obstacles defined
   * in your PathPlanner project. The path is generated in real-time, not pre-planned.
   *
   * <p><strong>Constraints Applied:</strong>
   * <ul>
   *   <li>Max velocity: Robot's maximum chassis velocity</li>
   *   <li>Max acceleration: 4.0 m/s²</li>
   *   <li>Max angular velocity: Robot's maximum angular velocity</li>
   *   <li>Max angular acceleration: 720°/s²</li>
   * </ul>
   *
   * <p><strong>Alliance Flipping:</strong> If the robot is on the red alliance,
   * the target pose is automatically mirrored to the red side of the field
   * (configured in setupPathPlanner).
   *
   * <p><strong>Usage Example:</strong>
   * <pre>{@code
   * // Drive to a specific field position
   * Pose2d scoringPosition = new Pose2d(2.0, 5.5, Rotation2d.fromDegrees(180));
   * Buttons.XboxAButton.onTrue(swerve.driveToPoseCommand(scoringPosition));
   *
   * // Drive to position relative to AprilTag
   * Pose2d tagPose = aprilTagFieldLayout.getTagPose(7).get().toPose2d();
   * Pose2d offsetPose = tagPose.transformBy(new Transform2d(-1.0, 0, new Rotation2d()));
   * swerve.driveToPoseCommand(offsetPose).schedule();
   *
   * // Chain with other commands
   * Commands.sequence(
   *     swerve.driveToPoseCommand(pickupPose),
   *     intake.grabCommand(),
   *     swerve.driveToPoseCommand(scorePose),
   *     shooter.shootCommand()
   * );
   * }</pre>
   *
   * @param pose Target pose (x, y in meters, rotation as Rotation2d)
   * @return PathPlanner pathfinding command that completes when pose is reached
   *
   * @see #getAutonomousCommand(String)
   * @see com.pathplanner.lib.auto.AutoBuilder
   */
  public Command driveToPoseCommand(Pose2d pose) {
    return AutoBuilder.pathfindToPose(
        pose,
        new com.pathplanner.lib.path.PathConstraints(
            swerveDrive.getMaximumChassisVelocity(),
            4.0,
            swerveDrive.getMaximumChassisAngularVelocity(),
            edu.wpi.first.math.util.Units.degreesToRadians(720)
        )
    ).withName("DriveToPose");
  }

  /**
   * Command to run a pre-planned PathPlanner autonomous routine.
   *
   * <p><strong>What is an Auto?:</strong> A PathPlanner "auto" is a complete autonomous
   * routine created in the PathPlanner GUI. It can include:
   * <ul>
   *   <li>Multiple path segments with waypoints</li>
   *   <li>Event markers that trigger commands (intake, shoot, etc.)</li>
   *   <li>Named commands linked to your robot's subsystems</li>
   *   <li>Starting pose for odometry reset</li>
   * </ul>
   *
   * <p><strong>File Location:</strong> Auto files are stored in:
   * {@code src/main/deploy/pathplanner/autos/[pathName].auto}
   *
   * <p><strong>Alliance Flipping:</strong> Paths are automatically mirrored for
   * the red alliance based on the supplier configured in setupPathPlanner().
   *
   * <p><strong>Named Commands:</strong> Register named commands before calling this:
   * <pre>{@code
   * // In RobotContainer constructor
   * NamedCommands.registerCommand("intake", intake.runCommand());
   * NamedCommands.registerCommand("shoot", shooter.shootCommand());
   * }</pre>
   *
   * <p><strong>Usage Example:</strong>
   * <pre>{@code
   * // In RobotContainer
   * public Command getAutonomousCommand() {
   *     return swerve.getAutonomousCommand("4PieceAuto");
   * }
   *
   * // With SendableChooser for multiple autos
   * autoChooser.addOption("4 Piece", swerve.getAutonomousCommand("4PieceAuto"));
   * autoChooser.addOption("2 Piece + Balance", swerve.getAutonomousCommand("2PieceBalance"));
   * autoChooser.addOption("Just Leave", swerve.getAutonomousCommand("Mobility"));
   * }</pre>
   *
   * @param pathName Name of the auto file (without .auto extension)
   * @return Complete autonomous command sequence
   *
   * @see #driveToPoseCommand(Pose2d)
   * @see com.pathplanner.lib.commands.PathPlannerAuto
   */
  public Command getAutonomousCommand(String pathName) {
    return new PathPlannerAuto(pathName)
        .withName("Auto(" + pathName + ")");
  }

  // ==================== GAME TARGET COMMANDS ====================

  /**
   * Command to drive to a named game target using PathPlanner pathfinding.
   *
   * <p>The target pose is calculated by applying the target's offset to the
   * associated AprilTag position from the field layout.
   *
   * <p><strong>Usage Example:</strong>
   * <pre>{@code
   * // Drive to the tower scoring position
   * Buttons.XboxAButton.onTrue(swerve.driveToTargetCommand("tower-blue"));
   *
   * // Drive to alliance-specific target
   * Buttons.XboxBButton.onTrue(swerve.driveToAllianceTargetCommand("tower"));
   * }</pre>
   *
   * @param targetName Name of the game target
   * @return PathPlanner pathfinding command, or empty command if target not found
   */
  public Command driveToTargetCommand(String targetName) {
    return Commands.either(
        Commands.defer(() -> {
          Optional<Pose2d> targetPose = getTargetPose(targetName);
          return targetPose.map(this::driveToPoseCommand)
              .orElse(Commands.none());
        }, java.util.Set.of(this)),
        Commands.none(),
        () -> gameTargetConfig != null && gameTargetConfig.hasTarget(targetName)
    ).withName("DriveToTarget(" + targetName + ")");
  }

  /**
   * Command to drive to the alliance-specific game target.
   *
   * <p>Automatically selects the blue or red variant based on current alliance.
   *
   * @param baseName Base target name (without alliance suffix)
   * @return PathPlanner pathfinding command
   */
  public Command driveToAllianceTargetCommand(String baseName) {
    return Commands.defer(() -> {
      String fullName = baseName + (Utils.isOnRedAlliance() ? "-red" : "-blue");
      return driveToTargetCommand(fullName);
    }, java.util.Set.of(this)).withName("DriveToAllianceTarget(" + baseName + ")");
  }

  /**
   * Command to aim (rotate) the robot toward a game target's position.
   *
   * <p>This command rotates the robot to face the target but does not drive toward it.
   *
   * <p><strong>Usage Example:</strong>
   * <pre>{@code
   * // Aim at tower while driving
   * swerve.isInRangeOfTargetTrigger("tower-blue", 2.0, 5.0)
   *     .whileTrue(swerve.aimAtGameTargetCommand("tower-blue", 2.0));
   * }</pre>
   *
   * @param targetName Name of the game target
   * @param toleranceDegrees Angular tolerance in degrees
   * @return Aiming command, or empty command if target not found
   */
  public Command aimAtGameTargetCommand(String targetName, double toleranceDegrees) {
    return Commands.run(() -> {
      Optional<Pose2d> targetPose = getTargetPose(targetName);
      if (targetPose.isEmpty()) return;

      // Calculate yaw to target
      Translation2d robotPos = getPose().getTranslation();
      Translation2d targetPos = targetPose.get().getTranslation();
      Translation2d diff = targetPos.minus(robotPos);
      Rotation2d targetYaw = new Rotation2d(diff.getX(), diff.getY());

      // Use heading controller to rotate toward target
      ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(0, 0,
          swerveDrive.swerveController.headingCalculate(
              getHeading().getRadians(),
              targetYaw.getRadians()),
          getHeading());
      drive(speeds);
    }, this).until(() -> {
      Optional<Pose2d> targetPose = getTargetPose(targetName);
      if (targetPose.isEmpty()) return true;

      Translation2d robotPos = getPose().getTranslation();
      Translation2d targetPos = targetPose.get().getTranslation();
      Translation2d diff = targetPos.minus(robotPos);
      Rotation2d targetYaw = new Rotation2d(diff.getX(), diff.getY());

      return Math.abs(targetYaw.minus(getHeading()).getDegrees()) < toleranceDegrees;
    }).withName("AimAtGameTarget(" + targetName + ")");
  }

  /**
   * Command to aim at the alliance-specific game target.
   *
   * @param baseName Base target name (without alliance suffix)
   * @param toleranceDegrees Angular tolerance in degrees
   * @return Aiming command
   */
  public Command aimAtAllianceGameTargetCommand(String baseName, double toleranceDegrees) {
    return Commands.defer(() -> {
      String fullName = baseName + (Utils.isOnRedAlliance() ? "-red" : "-blue");
      return aimAtGameTargetCommand(fullName, toleranceDegrees);
    }, java.util.Set.of(this)).withName("AimAtAllianceGameTarget(" + baseName + ")");
  }

  // ==================== MANUAL DRIVE COMMANDS ====================

  /**
   * Command for manual teleop drive using joystick inputs (field-oriented).
   *
   * <p><strong>Field-Oriented Drive:</strong> The robot moves relative to the field,
   * not the robot's current heading. Pushing the joystick "up" always moves the robot
   * toward the opposite alliance wall, regardless of which way the robot is facing.
   * This is the most intuitive control mode for drivers.
   *
   * <p><strong>Input Processing:</strong>
   * <ul>
   *   <li>Translation inputs are scaled by 80% for finer control</li>
   *   <li>Rotation input is cubed (x³) for precise low-speed turning</li>
   *   <li>All inputs are multiplied by max chassis velocity</li>
   * </ul>
   *
   * <p><strong>Joystick Mapping (typical):</strong>
   * <ul>
   *   <li>translationX → Left stick Y axis (forward/backward)</li>
   *   <li>translationY → Left stick X axis (left/right strafe)</li>
   *   <li>angularRotationX → Right stick X axis (rotation)</li>
   * </ul>
   *
   * <p><strong>Usage Example:</strong>
   * <pre>{@code
   * // Basic setup with Xbox controller
   * swerve.setDefaultCommand(
   *     swerve.driveCommand(
   *         () -> -controller.getLeftY(),   // Forward (inverted for standard convention)
   *         () -> -controller.getLeftX(),   // Strafe left
   *         () -> -controller.getRightX()   // Rotate CCW
   *     )
   * );
   *
   * // With Buttons utility class
   * swerve.setDefaultCommand(
   *     swerve.driveCommand(
   *         Buttons.createForwardSupplier(0.05, InputCurve.CUBIC),
   *         Buttons.createStrafeSupplier(0.05, InputCurve.CUBIC),
   *         Buttons.createRotationSupplier(0.1, InputCurve.SIGMOID)
   *     )
   * );
   * }</pre>
   *
   * @param translationX X translation supplier (-1 to 1, forward positive)
   * @param translationY Y translation supplier (-1 to 1, left positive)
   * @param angularRotationX Angular velocity supplier (-1 to 1, CCW positive)
   * @return Teleop drive command (should be set as default command)
   *
   * @see #driveCommand(DoubleSupplier, DoubleSupplier, DoubleSupplier, DoubleSupplier)
   * @see #driveFieldOrientedCommand(Supplier)
   */
  public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY,
                               DoubleSupplier angularRotationX) {
    return Commands.run(() -> {
      swerveDrive.drive(SwerveMath.scaleTranslation(new Translation2d(
              translationX.getAsDouble() * swerveDrive.getMaximumChassisVelocity(),
              translationY.getAsDouble() * swerveDrive.getMaximumChassisVelocity()), 0.8),
          Math.pow(angularRotationX.getAsDouble(), 3) * swerveDrive.getMaximumChassisAngularVelocity(),
          true,
          false);
    }, this).withName("TeleopDrive");
  }

  /**
   * Command for manual drive with heading lock using a second joystick.
   *
   * <p><strong>Heading Lock Mode:</strong> Instead of directly controlling rotation speed,
   * the right joystick sets a target heading angle. The robot automatically rotates to
   * face that direction while you control translation. This is useful for:
   * <ul>
   *   <li>Keeping the robot facing a specific direction while strafing</li>
   *   <li>Quick 180° turns by flicking the stick</li>
   *   <li>Maintaining orientation during defense</li>
   * </ul>
   *
   * <p><strong>How Heading is Calculated:</strong>
   * The heading joystick (X, Y) forms a vector. The angle of this vector becomes the
   * target heading. For example:
   * <ul>
   *   <li>Stick pushed up (0, 1) → Face 90° (toward opponent alliance)</li>
   *   <li>Stick pushed right (1, 0) → Face 0° (toward right side)</li>
   *   <li>Stick pushed down-left (-1, -1) → Face 225°</li>
   * </ul>
   *
   * <p><strong>Usage Example:</strong>
   * <pre>{@code
   * // Use right stick to set heading direction
   * swerve.setDefaultCommand(
   *     swerve.driveCommand(
   *         () -> -controller.getLeftY(),   // Forward/back
   *         () -> -controller.getLeftX(),   // Strafe
   *         () -> -controller.getRightX(),  // Heading X
   *         () -> -controller.getRightY()   // Heading Y
   *     )
   * );
   * }</pre>
   *
   * @param translationX X translation supplier (-1 to 1, forward positive)
   * @param translationY Y translation supplier (-1 to 1, left positive)
   * @param headingX Heading joystick X component (-1 to 1)
   * @param headingY Heading joystick Y component (-1 to 1)
   * @return Teleop drive command with heading lock
   *
   * @see #driveCommand(DoubleSupplier, DoubleSupplier, DoubleSupplier)
   */
  public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY,
                               DoubleSupplier headingX, DoubleSupplier headingY) {
    return Commands.run(() -> {
      Translation2d scaledInputs = SwerveMath.scaleTranslation(
          new Translation2d(translationX.getAsDouble(), translationY.getAsDouble()), 0.8);

      driveFieldOriented(
          swerveDrive.swerveController.getTargetSpeeds(
              scaledInputs.getX(), scaledInputs.getY(),
              headingX.getAsDouble(),
              headingY.getAsDouble(),
              swerveDrive.getOdometryHeading().getRadians(),
              swerveDrive.getMaximumChassisVelocity()));
    }, this).withName("TeleopDriveHeading");
  }

  /**
   * Command to drive using field-oriented chassis speeds.
   *
   * <p><strong>Field-Oriented vs Robot-Oriented:</strong>
   * <ul>
   *   <li><strong>Field-Oriented:</strong> Velocities are relative to the field.
   *       +X always points toward the red alliance wall, +Y always points left
   *       (from blue driver station perspective). The robot's heading doesn't
   *       affect the direction of movement.</li>
   *   <li><strong>Robot-Oriented:</strong> Velocities are relative to the robot.
   *       +X is forward (robot's front), +Y is left (robot's left side).
   *       Direction changes as robot rotates.</li>
   * </ul>
   *
   * <p><strong>ChassisSpeeds Components:</strong>
   * <ul>
   *   <li>vxMetersPerSecond - Forward velocity (field +X direction)</li>
   *   <li>vyMetersPerSecond - Leftward velocity (field +Y direction)</li>
   *   <li>omegaRadiansPerSecond - Rotational velocity (CCW positive)</li>
   * </ul>
   *
   * <p><strong>Usage Example:</strong>
   * <pre>{@code
   * // Drive forward at 1 m/s while rotating at 0.5 rad/s
   * swerve.driveFieldOrientedCommand(() ->
   *     new ChassisSpeeds(1.0, 0.0, 0.5)
   * ).schedule();
   *
   * // Drive based on joystick with custom processing
   * swerve.setDefaultCommand(
   *     swerve.driveFieldOrientedCommand(() -> new ChassisSpeeds(
   *         -controller.getLeftY() * maxSpeed,
   *         -controller.getLeftX() * maxSpeed,
   *         -controller.getRightX() * maxAngularSpeed
   *     ))
   * );
   * }</pre>
   *
   * @param velocity Supplier of field-relative chassis speeds
   * @return Field-oriented drive command
   *
   * @see ChassisSpeeds
   * @see #driveCommand(DoubleSupplier, DoubleSupplier, DoubleSupplier)
   */
  public Command driveFieldOrientedCommand(Supplier<ChassisSpeeds> velocity) {
    return Commands.run(() -> swerveDrive.driveFieldOriented(velocity.get()), this)
        .withName("DriveFieldOriented");
  }

  /**
   * Command to center all swerve modules (point wheels straight ahead).
   *
   * <p><strong>What This Does:</strong> Sets all four swerve module steering angles
   * to 0 degrees, pointing the wheels straight forward. This is useful for:
   * <ul>
   *   <li>Initial alignment during robot setup</li>
   *   <li>Verifying encoder calibration</li>
   *   <li>Pre-match preparation</li>
   *   <li>Debugging steering issues</li>
   * </ul>
   *
   * <p><strong>Note:</strong> This command runs continuously. Bind it to a button
   * with {@code whileTrue()} or use {@code withTimeout()} to limit duration.
   *
   * <p><strong>Usage Example:</strong>
   * <pre>{@code
   * // Center modules while holding button
   * Buttons.XboxYButton.whileTrue(swerve.centerModulesCommand());
   *
   * // Center modules for 2 seconds at startup
   * Commands.sequence(
   *     swerve.centerModulesCommand().withTimeout(2.0),
   *     // ... rest of auto
   * );
   * }</pre>
   *
   * @return Command that continuously sets all module angles to 0°
   */
  public Command centerModulesCommand() {
    return Commands.run(() ->
        java.util.Arrays.asList(swerveDrive.getModules())
            .forEach(it -> it.setAngle(0.0))
    , this).withName("CenterModules");
  }

  // ==================== DISTANCE-BASED DRIVE COMMANDS ====================

  /**
   * Command to drive forward a specific distance (robot-relative).
   *
   * <p><strong>Robot-Relative Movement:</strong> The robot drives forward relative
   * to its current heading. If the robot is facing 45°, it will move in the 45°
   * direction. This is different from field-oriented movement.
   *
   * <p><strong>How Distance is Measured:</strong> Uses odometry to track the
   * Euclidean distance traveled from the starting position. The command ends
   * when this distance exceeds the target.
   *
   * <p><strong>Use Cases:</strong>
   * <ul>
   *   <li>Simple autonomous movements</li>
   *   <li>Backing away from game pieces</li>
   *   <li>Approaching a target a known distance</li>
   * </ul>
   *
   * <p><strong>Usage Example:</strong>
   * <pre>{@code
   * // Drive forward 2 meters at 1.5 m/s
   * swerve.driveToDistanceCommand(2.0, 1.5).schedule();
   *
   * // Drive backward 0.5 meters at 0.5 m/s
   * swerve.driveToDistanceCommand(0.5, -0.5).schedule();
   *
   * // In autonomous sequence
   * Commands.sequence(
   *     swerve.driveToDistanceCommand(1.0, 2.0),  // Drive forward 1m
   *     intake.grabCommand(),
   *     swerve.driveToDistanceCommand(1.0, -2.0)  // Back up 1m
   * );
   * }</pre>
   *
   * @param distanceInMeters Distance to drive in meters (always positive)
   * @param speedInMetersPerSecond Speed in m/s (positive = forward, negative = backward)
   * @return Command that drives the specified distance then stops
   *
   * @see #driveToDistanceFieldOrientedCommand(double, double)
   * @see #driveForwardDistanceCommand(double, double)
   */
  public Command driveToDistanceCommand(double distanceInMeters, double speedInMetersPerSecond) {
    return Commands.sequence(
        Commands.runOnce(() -> startPose = swerveDrive.getPose()),
        Commands.run(() -> drive(new ChassisSpeeds(speedInMetersPerSecond, 0, 0)), this)
            .until(() -> swerveDrive.getPose().getTranslation()
                .getDistance(startPose.getTranslation()) > distanceInMeters),
        Commands.runOnce(() -> drive(new ChassisSpeeds(0, 0, 0)))
    ).withName("DriveDistance(" + distanceInMeters + "m)");
  }

  /**
   * Command to drive a specific distance while maintaining current heading (field-oriented).
   *
   * <p><strong>Field-Oriented Distance Drive:</strong> The robot moves in the direction
   * it's currently facing, but uses field-oriented control. This means:
   * <ul>
   *   <li>The robot maintains its heading throughout the movement</li>
   *   <li>External disturbances won't cause the robot to veer off course</li>
   *   <li>The gyro is used to calculate the correct X/Y velocities</li>
   * </ul>
   *
   * <p><strong>When to Use This vs driveToDistanceCommand:</strong>
   * <ul>
   *   <li>Use this for more accurate straight-line movement</li>
   *   <li>Use driveToDistanceCommand for simple robot-relative movement</li>
   * </ul>
   *
   * <p><strong>Usage Example:</strong>
   * <pre>{@code
   * // Drive 3 meters in current heading direction at 2 m/s
   * swerve.driveToDistanceFieldOrientedCommand(3.0, 2.0).schedule();
   *
   * // Approach scoring position
   * Commands.sequence(
   *     swerve.aimAtAprilTagCommand(7, 2.0),  // Aim at target
   *     swerve.driveToDistanceFieldOrientedCommand(1.5, 1.0)  // Approach
   * );
   * }</pre>
   *
   * @param distanceInMeters Distance to drive in meters
   * @param speedInMetersPerSecond Speed in m/s (positive = forward, negative = backward)
   * @return Command that drives field-oriented for the specified distance
   *
   * @see #driveToDistanceCommand(double, double)
   */
  public Command driveToDistanceFieldOrientedCommand(double distanceInMeters, double speedInMetersPerSecond) {
    return Commands.sequence(
        Commands.runOnce(() -> startPose = swerveDrive.getPose()),

        Commands.run(() -> {
          Rotation2d heading = getHeading();
          double speed = Math.abs(speedInMetersPerSecond);
          int direction = (speedInMetersPerSecond >= 0) ? 1 : -1;

          double xVel = direction * speed * Math.cos(heading.getRadians());
          double yVel = direction * speed * Math.sin(heading.getRadians());

          driveFieldOriented(new ChassisSpeeds(xVel, yVel, 0));
        }, this)
            .until(() -> swerveDrive.getPose().getTranslation()
                .getDistance(startPose.getTranslation()) >= distanceInMeters),

        Commands.runOnce(() -> drive(new Translation2d(), 0, true))
    ).withName("DriveDistanceFieldOriented(" + distanceInMeters + "m)");
  }

  /**
   * Command to drive forward a specific distance (alternative implementation).
   *
   * @param distanceInMeters Distance to drive in meters
   * @param speedInMetersPerSecond Speed in meters per second
   * @return Command that drives forward
   */
  public Command driveForwardDistanceCommand(double distanceInMeters, double speedInMetersPerSecond) {
    return Commands.sequence(
        Commands.runOnce(() -> startPose = swerveDrive.getPose()),
        Commands.run(() -> drive(
            ChassisSpeeds.fromFieldRelativeSpeeds(
                new ChassisSpeeds(speedInMetersPerSecond, 0, 0),
                swerveDrive.getOdometryHeading())
        ), this)
            .until(() -> swerveDrive.getPose().getTranslation()
                .getDistance(startPose.getTranslation()) > distanceInMeters),
        Commands.runOnce(() -> drive(new ChassisSpeeds(0, 0, 0)))
    ).withName("DriveForward(" + distanceInMeters + "m)");
  }

  // Helper Methods (private)

  /**
   * Find the nearest pose from a list of target poses.
   *
   * @param targetPoses List of poses to search
   * @return Nearest pose to current robot position
   */
  private Pose2d findNearestPose(List<Pose2d> targetPoses) {
    Pose2d currentPose = getPose();
    return targetPoses.stream()
        .min((p1, p2) -> Double.compare(
            currentPose.getTranslation().getDistance(p1.getTranslation()),
            currentPose.getTranslation().getDistance(p2.getTranslation())))
        .orElse(targetPoses.get(0));
  }

  /**
   * Check if pose has changed enough to warrant path recalculation.
   *
   * @param oldPose Previous pose
   * @param newPose Current pose
   * @return true if pose changed significantly
   */
  private boolean poseChangedSignificantly(Pose2d oldPose, Pose2d newPose) {
    double poseDifference = oldPose.getTranslation()
        .getDistance(newPose.getTranslation());
    double rotationDifference = Math.abs(
        oldPose.getRotation().minus(newPose.getRotation()).getDegrees());

    return poseDifference > 0.1 || rotationDifference > 5.0; // Adjust these thresholds
  }

  /**
   * Check if robot has reached target pose within tolerance.
   *
   * @param targetPose Target pose
   * @param positionTolerance Position tolerance in meters
   * @param rotationTolerance Rotation tolerance in degrees
   * @return true if within tolerance
   */
  private boolean hasReachedPose(Pose2d targetPose, double positionTolerance, double rotationTolerance) {
    Pose2d currentPose = getPose();

    double poseError = currentPose.getTranslation()
        .getDistance(targetPose.getTranslation());
    double rotationError = Math.abs(
        currentPose.getRotation().minus(targetPose.getRotation()).getDegrees());

    return poseError < positionTolerance && rotationError < rotationTolerance;
  }

  // ==================== SIMPLE MOVEMENT COMMANDS ====================

  /**
   * Command to rotate the robot to face a specific field-relative angle.
   *
   * <p><strong>How It Works:</strong> Uses the YAGSL heading controller to smoothly
   * rotate the robot to the target angle. The command ends when the robot is within
   * the specified tolerance of the target heading.
   *
   * <p><strong>Use Cases:</strong>
   * <ul>
   *   <li>Pre-positioning for scoring (face target direction)</li>
   *   <li>Autonomous alignment to field-relative angles</li>
   *   <li>Resetting robot orientation during teleop</li>
   * </ul>
   *
   * <p><strong>Usage Example:</strong>
   * <pre>{@code
   * // Turn to face 0 degrees (toward red alliance wall)
   * swerve.turnToAngleCommand(Rotation2d.fromDegrees(0), 2.0).schedule();
   *
   * // Turn to face target before scoring
   * Commands.sequence(
   *     swerve.turnToAngleCommand(Rotation2d.fromDegrees(180), 1.0),
   *     scorer.scoreCommand()
   * );
   *
   * // Compose turn-then-drive sequence
   * Commands.sequence(
   *     swerve.turnToAngleCommand(Rotation2d.fromDegrees(45), 2.0),
   *     swerve.driveToDistanceCommand(1.5, 2.0)
   * );
   * }</pre>
   *
   * @param targetAngle Field-relative target angle
   * @param toleranceDegrees Angular tolerance in degrees (command ends when within this)
   * @return Command that rotates robot to face the specified angle
   *
   * @see #aimAtAprilTagCommand(int, double)
   * @see #isFacingHeadingTrigger(Rotation2d, double)
   */
  public Command turnToAngleCommand(Rotation2d targetAngle, double toleranceDegrees) {
    return Commands.run(() -> {
      ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(0, 0,
          swerveDrive.swerveController.headingCalculate(
              getHeading().getRadians(),
              targetAngle.getRadians()),
          getHeading());
      drive(speeds);
    }, this).until(() ->
        Math.abs(targetAngle.minus(getHeading()).getDegrees()) < toleranceDegrees
    ).withName("TurnToAngle(" + targetAngle.getDegrees() + "°)");
  }

  /**
   * Command to strafe (move sideways) a specified distance while maintaining current heading.
   *
   * <p><strong>How It Works:</strong> Moves the robot perpendicular to its current
   * heading using field-oriented control. The robot maintains its heading throughout
   * the strafe movement.
   *
   * <p><strong>Strafe Direction:</strong>
   * <ul>
   *   <li>Positive distance → Strafe left (robot's left side)</li>
   *   <li>Negative distance → Strafe right (robot's right side)</li>
   * </ul>
   *
   * <p><strong>Use Cases:</strong>
   * <ul>
   *   <li>Manual positioning adjustments during autonomous</li>
   *   <li>Aligning with game pieces or scoring positions</li>
   *   <li>Side-stepping obstacles</li>
   * </ul>
   *
   * <p><strong>Usage Example:</strong>
   * <pre>{@code
   * // Strafe 0.5 meters left at 1.0 m/s
   * swerve.strafeCommand(0.5, 1.0).schedule();
   *
   * // Strafe 0.3 meters right at 0.5 m/s
   * swerve.strafeCommand(-0.3, 0.5).schedule();
   *
   * // Align then strafe sequence
   * Commands.sequence(
   *     swerve.turnToAngleCommand(Rotation2d.fromDegrees(90), 2.0),
   *     swerve.strafeCommand(0.4, 0.8)
   * );
   * }</pre>
   *
   * @param distanceMeters Distance to strafe in meters (positive = left, negative = right)
   * @param speedMetersPerSecond Speed in meters per second (always positive)
   * @return Command that strafes the specified distance then stops
   *
   * @see #alignAndStrafeCommand(int, double, double, double)
   * @see #driveToDistanceCommand(double, double)
   */
  public Command strafeCommand(double distanceMeters, double speedMetersPerSecond) {
    return Commands.sequence(
        Commands.runOnce(() -> startPose = swerveDrive.getPose()),

        Commands.run(() -> {
          Rotation2d heading = getHeading();
          double speed = Math.abs(speedMetersPerSecond);
          int direction = (distanceMeters >= 0) ? 1 : -1;

          // Strafe perpendicular to heading (90 degrees offset)
          double strafeAngle = heading.getRadians() + Math.PI / 2;
          double xVel = direction * speed * Math.cos(strafeAngle);
          double yVel = direction * speed * Math.sin(strafeAngle);

          driveFieldOriented(new ChassisSpeeds(xVel, yVel, 0));
        }, this)
            .until(() -> swerveDrive.getPose().getTranslation()
                .getDistance(startPose.getTranslation()) >= Math.abs(distanceMeters)),

        Commands.runOnce(() -> drive(new ChassisSpeeds(0, 0, 0)))
    ).withName("Strafe(" + distanceMeters + "m)");
  }

  /**
   * Command for slow, continuous forward movement (creeping).
   *
   * <p><strong>How It Works:</strong> Drives forward at a slow, preset speed
   * (0.3 m/s) continuously until the command is cancelled. This is useful for
   * fine positioning with visual feedback.
   *
   * <p><strong>Use Cases:</strong>
   * <ul>
   *   <li>Slow approach to game pieces for pickup</li>
   *   <li>Fine-tuning position before scoring</li>
   *   <li>Moving while operator aligns visually</li>
   * </ul>
   *
   * <p><strong>Note:</strong> This command runs indefinitely until cancelled.
   * Use with .until(), .withTimeout(), or .whileTrue() for controlled execution.
   *
   * <p><strong>Usage Example:</strong>
   * <pre>{@code
   * // Creep forward while button is held
   * Buttons.XboxAButton.whileTrue(swerve.creepForwardCommand());
   *
   * // Creep until sensor detects game piece
   * swerve.creepForwardCommand()
   *     .until(() -> intake.hasGamePiece())
   *     .schedule();
   * }</pre>
   *
   * @return Command that drives forward slowly (0.3 m/s) continuously
   *
   * @see #creepCommand(double)
   * @see #driveToDistanceCommand(double, double)
   */
  public Command creepForwardCommand() {
    return creepCommand(0.3);
  }

  /**
   * Command for slow, continuous movement at a configurable speed.
   *
   * <p><strong>How It Works:</strong> Drives forward (or backward if negative)
   * at the specified speed continuously until the command is cancelled.
   *
   * <p><strong>Speed Direction:</strong>
   * <ul>
   *   <li>Positive speed → Move forward</li>
   *   <li>Negative speed → Move backward</li>
   * </ul>
   *
   * <p><strong>Recommended Speed Range:</strong> 0.1 to 0.5 m/s for fine positioning.
   * Higher speeds defeat the purpose of creeping.
   *
   * <p><strong>Note:</strong> This command runs indefinitely until cancelled.
   * Use with .until(), .withTimeout(), or .whileTrue() for controlled execution.
   *
   * <p><strong>Usage Example:</strong>
   * <pre>{@code
   * // Creep forward at custom speed while button is held
   * Buttons.XboxLeftBumper.whileTrue(swerve.creepCommand(0.2));
   *
   * // Creep backward slowly
   * Buttons.XboxRightBumper.whileTrue(swerve.creepCommand(-0.2));
   *
   * // Creep until aligned with target
   * swerve.creepCommand(0.25)
   *     .until(() -> swerve.isAlignedWithTagTrigger(7, Degrees.of(2)).getAsBoolean())
   *     .schedule();
   * }</pre>
   *
   * @param speedMetersPerSecond Speed in meters per second (positive = forward, negative = backward)
   * @return Command that drives at the specified speed continuously
   *
   * @see #creepForwardCommand()
   */
  public Command creepCommand(double speedMetersPerSecond) {
    return Commands.run(() -> drive(new ChassisSpeeds(speedMetersPerSecond, 0, 0)), this)
        .withName("Creep(" + speedMetersPerSecond + "m/s)");
  }

  /**
   * Command to drive to a position while maintaining a specified heading.
   *
   * <p><strong>How It Works:</strong> Drives the robot to the target X,Y position
   * while independently controlling the robot's heading. This decouples translation
   * and rotation, unlike {@link #driveToPoseCommand(Pose2d)} which uses PathPlanner
   * and couples the rotation to the path.
   *
   * <p><strong>Key Difference from driveToPoseCommand:</strong>
   * <ul>
   *   <li>This command: Maintains specified heading throughout entire movement</li>
   *   <li>driveToPoseCommand: Rotates as part of the path trajectory</li>
   * </ul>
   *
   * <p><strong>Use Cases:</strong>
   * <ul>
   *   <li>Drive to pickup spot while facing the game piece</li>
   *   <li>Approach scoring position while facing the target</li>
   *   <li>Strafe to a position while maintaining forward orientation</li>
   * </ul>
   *
   * <p><strong>Usage Example:</strong>
   * <pre>{@code
   * // Drive to position (2, 5) while facing 90 degrees
   * swerve.driveToPositionWithHeadingCommand(
   *     new Translation2d(2.0, 5.0),
   *     Rotation2d.fromDegrees(90),
   *     0.1
   * ).schedule();
   *
   * // Approach target while facing it
   * swerve.driveToPositionWithHeadingCommand(
   *     new Translation2d(1.5, 5.5),
   *     Rotation2d.fromDegrees(180),
   *     0.05
   * ).schedule();
   * }</pre>
   *
   * @param targetPosition Target X,Y position on the field
   * @param heading Desired heading to maintain throughout the movement
   * @param positionToleranceMeters Position tolerance in meters (command ends when within this)
   * @return Command that drives to position while maintaining heading
   *
   * @see #driveToPoseCommand(Pose2d)
   * @see #turnToAngleCommand(Rotation2d, double)
   */
  public Command driveToPositionWithHeadingCommand(Translation2d targetPosition, Rotation2d heading,
                                                   double positionToleranceMeters) {
    return Commands.run(() -> {
      Translation2d currentPosition = getPose().getTranslation();
      Translation2d toTarget = targetPosition.minus(currentPosition);
      double distance = toTarget.getNorm();

      // Calculate translation speed (slow down as we approach)
      double maxSpeed = DriveConstants.kMaxSpeed.in(MetersPerSecond);
      double translationSpeed = Math.min(maxSpeed, Math.max(0.3, distance * 2.0));

      // Calculate field-relative velocities toward target
      double xVel = 0;
      double yVel = 0;
      if (distance > positionToleranceMeters) {
        Rotation2d directionToTarget = new Rotation2d(toTarget.getX(), toTarget.getY());
        xVel = translationSpeed * Math.cos(directionToTarget.getRadians());
        yVel = translationSpeed * Math.sin(directionToTarget.getRadians());
      }

      // Calculate rotation speed using heading controller
      double omega = swerveDrive.swerveController.headingCalculate(
          getHeading().getRadians(),
          heading.getRadians());

      driveFieldOriented(new ChassisSpeeds(xVel, yVel, omega));
    }, this).until(() -> {
      double distance = getPose().getTranslation().getDistance(targetPosition);
      return distance < positionToleranceMeters;
    }).withName("DriveToPositionWithHeading(" + targetPosition + ")");
  }

  // ==================== SYSID COMMANDS ====================

  /**
   * Command to characterize the drive motors using WPILib's SysId tool.
   *
   * <p><strong>What is SysId?:</strong> System Identification (SysId) is a tool that
   * measures your robot's physical characteristics by running controlled tests.
   * This data is used to calculate feedforward gains (kS, kV, kA) for more accurate
   * velocity control.
   *
   * <p><strong>How the Test Works:</strong>
   * <ol>
   *   <li>Quasistatic test: Slowly ramps voltage to measure kS (static friction)</li>
   *   <li>Dynamic test: Applies step voltage to measure kV and kA</li>
   *   <li>Both forward and reverse directions are tested</li>
   * </ol>
   *
   * <p><strong>Test Parameters:</strong>
   * <ul>
   *   <li>Quasistatic ramp: 3.0 seconds</li>
   *   <li>Dynamic step: 5.0 seconds</li>
   *   <li>Timeout: 3.0 seconds between tests</li>
   *   <li>Max voltage: 12V</li>
   * </ul>
   *
   * <p><strong>IMPORTANT:</strong> Run this on a flat surface with plenty of space!
   * The robot will drive in both directions during characterization.
   *
   * <p><strong>Usage Example:</strong>
   * <pre>{@code
   * // Bind to a button for testing (typically in test mode)
   * Buttons.XboxAButton.onTrue(swerve.sysIdDriveMotorCommand());
   *
   * // Or run directly in testInit()
   * @Override
   * public void testInit() {
   *     swerve.sysIdDriveMotorCommand().schedule();
   * }
   * }</pre>
   *
   * <p><strong>After Running:</strong> Use the SysId tool in WPILib to analyze
   * the logged data and calculate feedforward gains.
   *
   * @return SysId characterization command for drive motors
   *
   * @see #sysIdAngleMotorCommand()
   * @see <a href="https://docs.wpilib.org/en/stable/docs/software/advanced-controls/system-identification/">WPILib SysId Docs</a>
   */
  public Command sysIdDriveMotorCommand() {
    return SwerveDriveTest.generateSysIdCommand(
        SwerveDriveTest.setDriveSysIdRoutine(
            new Config(),
            this, swerveDrive, 12, true),
        3.0, 5.0, 3.0);
  }

  /**
   * Command to characterize the steering/angle motors using WPILib's SysId tool.
   *
   * <p><strong>What This Characterizes:</strong> The angle motors (also called
   * steering or azimuth motors) that rotate the swerve modules. Accurate
   * characterization improves module pointing accuracy and responsiveness.
   *
   * <p><strong>How It Differs from Drive SysId:</strong>
   * <ul>
   *   <li>Tests rotational movement of modules, not wheel spinning</li>
   *   <li>Modules will rotate back and forth during the test</li>
   *   <li>Robot should be elevated so wheels can spin freely</li>
   * </ul>
   *
   * <p><strong>Test Parameters:</strong>
   * <ul>
   *   <li>Quasistatic ramp: 3.0 seconds</li>
   *   <li>Dynamic step: 5.0 seconds</li>
   *   <li>Timeout: 3.0 seconds between tests</li>
   * </ul>
   *
   * <p><strong>IMPORTANT:</strong> Elevate the robot on blocks so the wheels
   * don't touch the ground! The modules need to rotate freely.
   *
   * <p><strong>Usage Example:</strong>
   * <pre>{@code
   * // Bind to a button for testing
   * Buttons.XboxBButton.onTrue(swerve.sysIdAngleMotorCommand());
   *
   * // Run in test mode
   * @Override
   * public void testInit() {
   *     // Make sure robot is elevated!
   *     swerve.sysIdAngleMotorCommand().schedule();
   * }
   * }</pre>
   *
   * @return SysId characterization command for angle/steering motors
   *
   * @see #sysIdDriveMotorCommand()
   */
  public Command sysIdAngleMotorCommand() {
    return SwerveDriveTest.generateSysIdCommand(
        SwerveDriveTest.setAngleSysIdRoutine(
            new Config(),
            this, swerveDrive),
        3.0, 5.0, 3.0);
  }
}
