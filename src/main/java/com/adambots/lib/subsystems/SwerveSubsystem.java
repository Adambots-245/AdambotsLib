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

import org.photonvision.targeting.PhotonPipelineResult;

import com.adambots.lib.Constants.DriveConstants;
import com.adambots.lib.Constants.ModuleConstants;
import com.adambots.lib.utils.Utils;
import com.adambots.lib.vision.PhotonVision;
import com.adambots.lib.vision.VisionCamera;
import com.adambots.lib.vision.config.VisionSystemConfig;
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
 *   <li>PhotonVision integration for vision-corrected odometry</li>
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
 * @see PhotonVision
 * @see com.pathplanner.lib.auto.AutoBuilder
 */
public class SwerveSubsystem extends SubsystemBase {

  private final SwerveDrive swerveDrive;
  private final AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
  private final boolean visionDriveTest = true;
  private PhotonVision vision;

  // Configuration
  private final SwerveConfig swerveConfig;

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

    if (visionDriveTest) {
      // Note: Vision must now be configured externally using setupPhotonVision(VisionSystemConfig)
      // Stop the odometry thread if we are using vision that way we can synchronize
      // updates better.
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
   * Setup the photon vision class with a custom configuration.
   *
   * <p>This method allows full customization of cameras, standard deviations,
   * and filtering without modifying AdambotsLib source code.
   *
   * <p><strong>Usage Example:</strong>
   * <pre>{@code
   * // In your robot project's Constants file
   * public static final int[] SCORING_TAGS = {6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22};
   * public static final int[] INTAKE_TAGS = {1, 2, 4, 5, 12, 13, 14, 15};
   *
   * public static final VisionSystemConfig VISION_CONFIG = VisionConfigBuilder.create()
   *     .addCamera("Left")
   *         .positionInches(15, 11.75, 8)
   *         .rotationDegrees(0, 0, -30)
   *         .purpose(CameraPurpose.ODOMETRY)
   *         .allowedTags(SCORING_TAGS)
   *         .done()
   *     .addCamera("Right")
   *         .positionInches(15, -11.75, 8)
   *         .rotationDegrees(0, 0, 30)
   *         .purpose(CameraPurpose.ODOMETRY)
   *         .allowedTags(SCORING_TAGS)
   *         .done()
   *     .ambiguityThreshold(0.25)
   *     .build();
   *
   * // In RobotContainer
   * swerve.setupPhotonVision(VisionConstants.VISION_CONFIG);
   * }</pre>
   *
   * @param config The vision system configuration
   */
  public void setupPhotonVision(VisionSystemConfig config) {
    vision = new PhotonVision(config, swerveDrive::getPose, swerveDrive.field);
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

    // Always update odometry
    swerveDrive.updateOdometry();

    // Vision pose estimation only when vision is configured
    if (visionDriveTest && vision != null) {
      vision.updatePoseEstimation(swerveDrive);
    }

  }

  @Override
  public void simulationPeriodic() {
  }

  /**
   * Get the distance to a specific AprilTag.
   *
   * @param tagID The ID of the AprilTag to get the distance to.
   * @return Distance to a specific AprilTag in meters.
   */
  public double getDistanceToAprilTag(int tagID) {

    // Taken from PhotonUtils.getDistanceToPose
    Pose3d speakerAprilTagPose = aprilTagFieldLayout.getTagPose(tagID).get();
    return getPose().getTranslation().getDistance(speakerAprilTagPose.toPose2d().getTranslation());
  }

  /**
   * Get the yaw to aim at an AprilTag.
   * 
   * @param tagID The ID of the AprilTag to aim at.
   * @return {@link Rotation2d} of which you need to achieve.
   */
  public Rotation2d getAprilTagYaw(int tagID) {
    // Taken from PhotonUtils.getYawToPose()
    Pose3d speakerAprilTagPose = aprilTagFieldLayout.getTagPose(tagID).get();
    Translation2d relativeTrl = speakerAprilTagPose.toPose2d().relativeTo(getPose()).getTranslation();
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
   * Checks if the vision drive test mode is enabled.
   *
   * @return true if vision drive test mode is enabled, false otherwise.
   */
  public boolean isVisionDriveTest() {
    return visionDriveTest;
  }

  /**
   * Get the PhotonVision instance.
   *
   * @return The PhotonVision instance.
   */
  public PhotonVision getVision() {
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
   * Trigger that is true when robot is stationary.
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
   * Trigger that is true when robot exceeds velocity threshold.
   *
   * @param velocityThreshold Velocity threshold in m/s
   * @return Trigger for high-speed state
   */
  public Trigger isMovingFastTrigger(double velocityThreshold) {
    return new Trigger(() -> {
      ChassisSpeeds speeds = getRobotVelocity();
      double velocity = Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
      return velocity >= velocityThreshold;
    });
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
      if (!vision.hasTarget()) return false;
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
    return new Trigger(() -> vision.hasTarget());
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

  // ==================== COMMAND FACTORIES ====================

  // ==================== VISION-BASED COMMANDS ====================

  /**
   * Command to rotate the robot to aim at a specific AprilTag.
   *
   * <p><strong>How It Works:</strong> Uses the heading PID controller to calculate
   * the angular velocity needed to align the robot's heading with the AprilTag.
   * The command completes when the angular error is within the specified tolerance.
   *
   * <p><strong>AprilTag IDs (2024 Crescendo):</strong>
   * <ul>
   *   <li>1-4: Red source tags</li>
   *   <li>5: Red amp</li>
   *   <li>6-7: Blue speaker</li>
   *   <li>8: Blue amp</li>
   *   <li>9-10: Red speaker</li>
   *   <li>11-16: Stage tags</li>
   * </ul>
   *
   * <p><strong>Note:</strong> This command only rotates the robot. It does not
   * move toward or away from the tag. Combine with translation for full alignment.
   *
   * <p><strong>Usage Example:</strong>
   * <pre>{@code
   * // Aim at speaker tag when button is held
   * Buttons.XboxLeftTriggerButton.whileTrue(swerve.aimAtAprilTagCommand(7, 2.0));
   *
   * // Aim then shoot sequence
   * Commands.sequence(
   *     swerve.aimAtAprilTagCommand(7, 1.0),  // Aim within 1 degree
   *     shooter.shootCommand()
   * );
   *
   * // Continuous aiming while driving (combine with drive command)
   * Buttons.XboxRightTriggerButton.whileTrue(
   *     Commands.parallel(
   *         swerve.aimAtAprilTagCommand(7, 2.0),
   *         shooter.spinUpCommand()
   *     )
   * );
   * }</pre>
   *
   * @param tagId AprilTag fiducial ID to aim at (see field layout for IDs)
   * @param tolerance Angular tolerance in degrees (command ends when within this)
   * @return Command that rotates robot to face the specified AprilTag
   *
   * @see #aimAtTargetCommand(VisionCamera)
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
    return Commands.runOnce(() ->
        edu.wpi.first.wpilibj.DataLogManager.log("AprilTag " + tagID + " distance - X: " +
            vision.getTransformToAprilTag(tagID).getX() +
            " Y: " + vision.getTransformToAprilTag(tagID).getY())
    ).withName("GetDistanceFromTag(" + tagID + ")");
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
   * // Align with speaker tag, then strafe 0.5m left to score position
   * Buttons.XboxAButton.onTrue(
   *     swerve.alignAndStrafeCommand(7, 0.5, 1.0, 2.0)
   * );
   *
   * // Align with amp tag, strafe right to scoring position
   * Commands.sequence(
   *     swerve.alignAndStrafeCommand(5, -0.3, 0.5, 1.0),
   *     scorer.scoreAmpCommand()
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
   * Command to aim robot at the best target from a specific VisionCamera.
   *
   * @param camera VisionCamera to use
   * @return Command that aims at best target from camera
   */
  public Command aimAtTargetCommand(VisionCamera camera) {
    return Commands.run(() -> {
      Optional<PhotonPipelineResult> resultO = camera.getBestResult();
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
      VisionCamera camera = vision.getCamera(cameraName);
      if (camera == null) return;

      Optional<PhotonPipelineResult> resultO = camera.getBestResult();
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
      // Get current robot pose
      Pose2d currentPose = swerveDrive.getPose();

      // Find nearest target pose
      Pose2d nearestPose = findNearestPose(targetPoses);

      // Get visible AprilTags and their poses
      boolean hasVisibleTags = false;
      for (VisionCamera camera : vision.getCameras()) {
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
   * <p><strong>What This Does:</strong> Enables all configured PhotonVision cameras
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
   * @see #setupPhotonVision(VisionSystemConfig)
   */
  public Command enableVisionCommand() {
    return Commands.runOnce(() -> vision.enableAllCameras(), this)
        .withName("EnableVision");
  }

  /**
   * Command to disable vision-based pose estimation (odometry only).
   *
   * <p><strong>What This Does:</strong> Disables all PhotonVision cameras from
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
    return Commands.runOnce(() -> vision.disableAllCameras(), this)
        .withName("DisableVision");
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
