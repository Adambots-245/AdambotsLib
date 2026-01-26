// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.lib.subsystems;

import java.io.File;
import java.util.List;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import static edu.wpi.first.units.Units.Meter;

import org.photonvision.targeting.PhotonPipelineResult;

import com.adambots.lib.Constants;
import com.adambots.lib.Constants.AutoConstants;
import com.adambots.lib.Constants.DriveConstants;
import com.adambots.lib.Constants.ModuleConstants;
import com.adambots.lib.utils.Utils;
import com.adambots.lib.vision.PhotonVision;
import com.adambots.lib.vision.VisionCamera;
import com.adambots.lib.vision.config.VisionSystemConfig;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.PIDConstants;
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

  // State tracking for commands
  private Pose2d startPose;

  /**
   * Creates a new SwerveSubsystem. Adapted from YAGSL-Example
   * Talk to Mr.B before making major changes to this file.
   */
  public SwerveSubsystem(File directory) {
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
    double driveConversionFactor = SwerveMath.calculateMetersPerRotation(ModuleConstants.kWheelRadiusMeters,
        1 / ModuleConstants.kSwerveModuleFinalGearRatio);

    // Configure the Telemetry before creating the SwerveDrive to avoid unnecessary
    // objects being created.
    // Adjust this value to control the amount of telemetry data that is printed to
    // the console. Turn it off or to low or pose for competition.
    SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;

    try {
      // Loads the conversion factors via JSON files
      swerveDrive = new SwerveParser(directory).createSwerveDrive(DriveConstants.kMaxSpeedMetersPerSecond,
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
    swerveDrive.setHeadingCorrection(false);

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
    swerveDrive.setCosineCompensator(true);

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
    swerveDrive.setAngularVelocityCompensation(true, true, 0.1);

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
   * Construct the swerve drive.
   *
   * @param driveCfg      SwerveDriveConfiguration for the swerve.
   * @param controllerCfg Swerve Controller.
   */
  public SwerveSubsystem(SwerveDriveConfiguration driveCfg, SwerveControllerConfiguration controllerCfg) {
    swerveDrive = new SwerveDrive(driveCfg,
        controllerCfg,
        DriveConstants.kMaxSpeedMetersPerSecond,
        new Pose2d(new Translation2d(Meter.of(2), Meter.of(0)),
            Rotation2d.fromDegrees(0)));
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
   * public static final int[] REEF_TAGS = {6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22};
   * public static final int[] HP_TAGS = {1, 2, 4, 5, 12, 13, 14, 15};
   *
   * public static final VisionSystemConfig VISION_CONFIG = VisionConfigBuilder.create()
   *     .addCamera("Left")
   *         .positionInches(15, 11.75, 8)
   *         .rotationDegrees(0, 0, -30)
   *         .purpose(CameraPurpose.ODOMETRY)
   *         .allowedTags(REEF_TAGS)
   *         .done()
   *     .addCamera("Right")
   *         .positionInches(15, -11.75, 8)
   *         .rotationDegrees(0, 0, 30)
   *         .purpose(CameraPurpose.ODOMETRY)
   *         .allowedTags(REEF_TAGS)
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

      final boolean enableFeedforward = true;
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
              new PIDConstants(AutoConstants.kPTranslationController, AutoConstants.kITranslationController,
                  AutoConstants.kDTranslationController), // Translation PID constants
              new PIDConstants(AutoConstants.kPThetaController, AutoConstants.kIThetaController,
                  AutoConstants.kDThetaController) // Rotation PID constants
          // Rotation PID constants
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

    // When vision is enabled we must manually update odometry in SwerveDrive
    if (visionDriveTest) {
      swerveDrive.updateOdometry();
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
        DriveConstants.kMaxSpeedMetersPerSecond);
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
   * @param toleranceDegrees Angular tolerance in degrees
   * @return Trigger for tag alignment
   */
  public Trigger isAlignedWithTagTrigger(int tagID, double toleranceDegrees) {
    return new Trigger(() -> {
      if (!vision.hasTarget()) return false;
      double error = Math.abs(getAprilTagYaw(tagID).minus(getHeading()).getDegrees());
      return error <= toleranceDegrees;
    });
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

  // Vision-Based Commands

  /**
   * Command to aim robot at specific AprilTag.
   * Uses heading PID to rotate until aligned within tolerance.
   *
   * @param tagId AprilTag ID to aim at
   * @param tolerance Angular tolerance in degrees
   * @return Command that aims at tag
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
   * Command to align with AprilTag then strafe sideways relative to tag.
   *
   * @param tagId              The AprilTag to align with
   * @param strafeDistance     Distance to strafe in meters (positive = left, negative = right)
   * @param strafeSpeed        Speed to strafe at in meters per second
   * @param alignmentTolerance Tolerance in degrees for alignment
   * @return Command that aligns then strafes
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

  // Vision Control Commands

  /**
   * Command to enable vision updates for pose estimation.
   *
   * @return Command that enables all cameras
   */
  public Command enableVisionCommand() {
    return Commands.runOnce(() -> vision.enableAllCameras(), this)
        .withName("EnableVision");
  }

  /**
   * Command to disable vision updates (odometry only).
   *
   * @return Command that disables all cameras
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

  // PathPlanner Commands

  /**
   * Command to drive to a specific pose using PathPlanner pathfinding.
   *
   * @param pose Target pose to drive to
   * @return PathPlanner pathfinding command
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
   * Command to run a PathPlanner autonomous path.
   *
   * @param pathName Name of the PathPlanner path
   * @return PathPlanner auto command
   */
  public Command getAutonomousCommand(String pathName) {
    return new PathPlannerAuto(pathName)
        .withName("Auto(" + pathName + ")");
  }

  // Manual Drive Commands

  /**
   * Command for manual teleop drive using joystick inputs for translation and rotation.
   *
   * @param translationX X translation supplier (cubed for smoother control)
   * @param translationY Y translation supplier (cubed for smoother control)
   * @param angularRotationX Angular velocity supplier (cubed for smoother control)
   * @return Teleop drive command
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
   * Command for manual drive using joystick inputs for translation and heading setpoint.
   *
   * @param translationX X translation supplier
   * @param translationY Y translation supplier
   * @param headingX Heading X joystick value
   * @param headingY Heading Y joystick value
   * @return Teleop drive command with heading control
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
   * @param velocity Supplier of field-relative chassis speeds
   * @return Field-oriented drive command
   */
  public Command driveFieldOrientedCommand(Supplier<ChassisSpeeds> velocity) {
    return Commands.run(() -> swerveDrive.driveFieldOriented(velocity.get()), this)
        .withName("DriveFieldOriented");
  }

  /**
   * Command to center all swerve modules (point straight ahead).
   *
   * @return Command that centers modules
   */
  public Command centerModulesCommand() {
    return Commands.run(() ->
        java.util.Arrays.asList(swerveDrive.getModules())
            .forEach(it -> it.setAngle(0.0))
    , this).withName("CenterModules");
  }

  // Distance-Based Drive Commands

  /**
   * Command to drive forward a specific distance at a given speed.
   * Uses robot-relative coordinates.
   *
   * @param distanceInMeters Distance to drive in meters
   * @param speedInMetersPerSecond Speed in meters per second
   * @return Command that drives the specified distance
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
   * Command to drive a specific distance using field-relative heading.
   *
   * @param distanceInMeters Distance to drive in meters
   * @param speedInMetersPerSecond Speed in meters per second
   * @return Command that drives field-oriented distance
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
   * Command to characterize the robot drive motors using SysId
   * 
   * @return SysId Drive Command
   */
  public Command sysIdDriveMotorCommand() {
    return SwerveDriveTest.generateSysIdCommand(
        SwerveDriveTest.setDriveSysIdRoutine(
            new Config(),
            this, swerveDrive, 12, true),
        3.0, 5.0, 3.0);
  }

  /**
   * Command to characterize the robot angle motors using SysId
   * 
   * @return SysId Angle Command
   */
  public Command sysIdAngleMotorCommand() {
    return SwerveDriveTest.generateSysIdCommand(
        SwerveDriveTest.setAngleSysIdRoutine(
            new Config(),
            this, swerveDrive),
        3.0, 5.0, 3.0);
  }
}
