// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.lib.commands.driveCommands;

import java.io.IOException;
import java.util.Arrays;
import java.util.List;
import java.util.Optional;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.json.simple.parser.ParseException;
import org.photonvision.targeting.PhotonPipelineResult;

import com.adambots.lib.subsystems.SwerveSubsystem;
import com.adambots.lib.vision.PhotonVision;
import com.adambots.lib.vision.PhotonVision.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import swervelib.SwerveController;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.swerve.SwerveSetpoint;
import com.pathplanner.lib.util.swerve.SwerveSetpointGenerator;

import swervelib.SwerveDrive;
import swervelib.SwerveDriveTest;
import swervelib.math.SwerveMath;

/** Provides a collection of utility commands for Driving */
public class DriveCommands {

    private final SwerveSubsystem subsystem;
    private final SwerveDrive swerveDrive;
    private SwerveController controller;
    private Pose2d startPose;

    public DriveCommands(SwerveSubsystem swerveDriveSubsystem) {
        this.subsystem = swerveDriveSubsystem;
        this.swerveDrive = swerveDriveSubsystem.getSwerveDrive();
        controller = swerveDrive.getSwerveController();
    }

    /**
     * Aim the robot at a specific Tag. Turn until it is aligned
     *
     * @param tolerance Tolerance in degrees within which the drive can stop
     * @return Command to turn the robot to the Tag.
     */
    public Command aimAtAprilTag(int tagId, double tolerance) {
        return Commands.run(
                () -> {
                    ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(0, 0,
                            controller.headingCalculate(subsystem.getHeading().getRadians(),
                                    subsystem.getAprilTagYaw(tagId).getRadians()),
                            subsystem.getHeading());
                    System.out.println("RObot Rotoation " + subsystem.getHeading());
                    System.out.println("Aptil Tag Rotoation " + subsystem.getAprilTagYaw(tagId).getRadians());
                    // System.out.println(speeds.omegaRadiansPerSecond);
                    subsystem.drive(speeds);

                }).until(() -> Math.abs(
                        subsystem.getAprilTagYaw(tagId).minus(subsystem.getHeading()).getDegrees()) < tolerance);
    }

    public Command getDistanceFromAprilTag(int tagID) {
        // return Commands.runOnce(()->
        // System.out.println(subsystem.getVision().getDistanceFromAprilTag(tagID)));
        return Commands
                .run(() -> System.out.println("X: " + subsystem.getVision().getDistanceFromAprilTagX(tagID).getX()
                        + " Y: " + subsystem.getVision().getDistanceFromAprilTagX(tagID).getY()));
    }

    /**
     * Aligns with AprilTag and then strafes sideways relative to the tag
     * 
     * @param tagId              The AprilTag to align with
     * @param strafeDistance     Distance to strafe in meters (positive = left,
     *                           negative = right)
     * @param strafeSpeed        Speed to strafe at in meters per second
     * @param alignmentTolerance Tolerance in degrees for alignment
     */
    public Command alignAndStrafeCommand(int tagId, double strafeDistance, double strafeSpeed,
            double alignmentTolerance) {
        return Commands.sequence(
                // First align with the AprilTag
                aimAtAprilTag(tagId, alignmentTolerance),

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
                            subsystem.drive(
                                    new Translation2d(0, speedY), // Only Y movement
                                    0, // No rotation
                                    true // Field relative
                            );
                        })
                                .until(() -> {
                                    // Calculate distance traveled sideways
                                    double distanceTraveled = Math.abs(
                                            swerveDrive.getPose().getTranslation().getY() -
                                                    startPose.getTranslation().getY());
                                    return distanceTraveled >= Math.abs(strafeDistance);
                                }),

                        // Stop moving
                        Commands.runOnce(() -> subsystem.drive(new ChassisSpeeds(0, 0, 0)))));
    }

    /**
     * Aim the robot at the target returned by PhotonVision.
     *
     * @return A {@link Command} which will run the alignment.
     */
    public Command aimAtTarget(Cameras camera) {

        return Commands.run(() -> {
            Optional<PhotonPipelineResult> resultO = camera.getBestResult();
            if (resultO.isPresent()) {
                var result = resultO.get();
                if (result.hasTargets()) {
                    subsystem.drive(subsystem.getTargetSpeeds(0, 0,
                            Rotation2d.fromDegrees(result.getBestTarget().getYaw())));
                }
            }
        });
    }

    /**
     * Use PathPlanner Path finding to go to a point on the field.
     *
     * @param pose Target {@link Pose2d} to go to.
     * @return PathFinding command
     */
    public Command driveToPose(Pose2d pose) {
        // Create the constraints to use while pathfinding
        PathConstraints constraints = new PathConstraints(
                1, 1.0,
                swerveDrive.getMaximumChassisAngularVelocity(), Units.degreesToRadians(720));

        // Since AutoBuilder is configured, we can use it to build pathfinding commands
        return AutoBuilder.pathfindToPose(
                pose,
                constraints,
                edu.wpi.first.units.Units.MetersPerSecond.of(0) // Goal end velocity in meters/sec
        );
    }

    // List of potential target poses
    List<Pose2d> targetPoses = Arrays.asList(
            new Pose2d(new Translation2d(1, 1), Rotation2d.fromDegrees(90)),
            new Pose2d(new Translation2d(2, 2), Rotation2d.fromDegrees(180)));

    /**
     * Drives to the nearest specified pose while continuously updating based on
     * AprilTag vision
     * 
     * @param targetPoses List of poses to potentially drive to
     * @return Command that continuously drives to nearest pose with vision updates
     */
    public Command driveToNearestPoseWithVision(List<Pose2d> targetPoses) {
        return Commands.run(() -> {
            // Get current robot pose
            Pose2d currentPose = swerveDrive.getPose();

            // Find nearest target pose
            Pose2d nearestPose = findNearestPose(currentPose, targetPoses);

            // Get visible AprilTags and their poses
            boolean hasVisibleTags = false;
            for (Cameras camera : Cameras.values()) {
                var result = camera.getLatestResult();
                if (result.isPresent() && result.get().hasTargets()) {
                    hasVisibleTags = true;
                    // Vision updates are handled by periodic() in SwerveSubsystem

                    // If pose changed significantly, recalculate path
                    if (poseChangedSignificantly(currentPose, swerveDrive.getPose())) {
                        driveToPose(nearestPose).schedule();
                    }
                }
            }

            // If no tags visible, continue with last known path
            if (!hasVisibleTags) {
                driveToPose(nearestPose).schedule();
            }
        })
                .until(() -> hasReachedPose(swerveDrive.getPose(),
                        findNearestPose(swerveDrive.getPose(), targetPoses)));
    }

    /**
     * Find the nearest pose from a list of target poses
     */
    private Pose2d findNearestPose(Pose2d currentPose, List<Pose2d> targetPoses) {
        return targetPoses.stream()
                .min((p1, p2) -> Double.compare(
                        currentPose.getTranslation().getDistance(p1.getTranslation()),
                        currentPose.getTranslation().getDistance(p2.getTranslation())))
                .orElse(targetPoses.get(0));
    }

    /**
     * Check if pose has changed enough to warrant path recalculation
     */
    private boolean poseChangedSignificantly(Pose2d oldPose, Pose2d newPose) {
        double poseDifference = oldPose.getTranslation()
                .getDistance(newPose.getTranslation());
        double rotationDifference = Math.abs(
                oldPose.getRotation().minus(newPose.getRotation()).getDegrees());

        return poseDifference > 0.1 || rotationDifference > 5.0; // Adjust these thresholds
    }

    /**
     * Check if robot has reached target pose within tolerance
     */
    private boolean hasReachedPose(Pose2d currentPose, Pose2d targetPose) {
        double poseTolerance = 0.05; // meters
        double rotationTolerance = 5.0; // degrees

        double poseError = currentPose.getTranslation()
                .getDistance(targetPose.getTranslation());
        double rotationError = Math.abs(
                currentPose.getRotation().minus(targetPose.getRotation()).getDegrees());

        return poseError < poseTolerance && rotationError < rotationTolerance;
    }

    /**
     * Drive with {@link SwerveSetpointGenerator} from 254, implemented by
     * PathPlanner.
     *
     * @param robotRelativeChassisSpeed Robot relative {@link ChassisSpeeds} to
     *                                  achieve.
     * @return {@link Command} to run.
     * @throws IOException    If the PathPlanner GUI settings is invalid
     * @throws ParseException If PathPlanner GUI settings is nonexistent.
     */
    private Command driveWithSetpointGenerator(Supplier<ChassisSpeeds> robotRelativeChassisSpeed)
            throws IOException, ParseException {
        SwerveSetpointGenerator setpointGenerator = new SwerveSetpointGenerator(RobotConfig.fromGUISettings(),
                swerveDrive.getMaximumChassisAngularVelocity());
        AtomicReference<SwerveSetpoint> prevSetpoint = new AtomicReference<>(
                new SwerveSetpoint(swerveDrive.getRobotVelocity(),
                        swerveDrive.getStates(),
                        DriveFeedforwards.zeros(swerveDrive.getModules().length)));
        AtomicReference<Double> previousTime = new AtomicReference<>();

        return Commands.startRun(() -> previousTime.set(Timer.getFPGATimestamp()),
                () -> {
                    double newTime = Timer.getFPGATimestamp();
                    SwerveSetpoint newSetpoint = setpointGenerator.generateSetpoint(prevSetpoint.get(),
                            robotRelativeChassisSpeed.get(),
                            newTime - previousTime.get());
                    swerveDrive.drive(newSetpoint.robotRelativeSpeeds(),
                            newSetpoint.moduleStates(),
                            newSetpoint.feedforwards().linearForces());
                    prevSetpoint.set(newSetpoint);
                    previousTime.set(newTime);

                });
    }

    /**
     * Drive with 254's Setpoint generator; port written by PathPlanner.
     *
     * @param fieldRelativeSpeeds Field-Relative {@link ChassisSpeeds}
     * @return Command to drive the robot using the setpoint generator.
     */
    public Command driveWithSetpointGeneratorFieldRelative(Supplier<ChassisSpeeds> fieldRelativeSpeeds) {
        try {
            return driveWithSetpointGenerator(() -> {
                return ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds.get(), subsystem.getHeading());

            });
        } catch (Exception e) {
            DriverStation.reportError(e.toString(), true);
        }
        return Commands.none();
    }

    /**
     * Command to characterize the robot drive motors using SysId
     *
     * @return SysId Drive Command
     */
    public Command sysIdDriveMotorCommand() {
        return SwerveDriveTest.generateSysIdCommand(
                SwerveDriveTest.setDriveSysIdRoutine(
                        new Config(),
                        subsystem, swerveDrive, 12.0, true),
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
                        subsystem, swerveDrive),
                3.0, 5.0, 3.0);
    }

    /**
     * Returns a Command that centers the modules of the SwerveDrive subsystem.
     *
     * @return a Command that centers the modules of the SwerveDrive subsystem
     */
    public Command centerModulesCommand() {
        return Commands.run(() -> Arrays.asList(swerveDrive.getModules())
                .forEach(it -> it.setAngle(0.0)));
    }

    /**
     * Returns a Command that drives the swerve drive to a specific distance at a
     * given speed.
     *
     * @param distanceInMeters       the distance to drive in meters
     * @param speedInMetersPerSecond the speed at which to drive in meters per
     *                               second
     * @return a Command that drives the swerve drive to a specific distance at a
     *         given speed
     */
    public Command driveToDistanceCommand(double distanceInMeters, double speedInMetersPerSecond) {
        return Commands.run(() -> subsystem.drive(new ChassisSpeeds(speedInMetersPerSecond, 0, 0)))
                .until(() -> swerveDrive.getPose().getTranslation()
                        .getDistance(new Translation2d(0, 0)) > distanceInMeters);
    }

    /**
     * Drives the robot a specified distance using field relative heading
     * 
     * @param distanceInMeters       Positive distance to drive in meters
     * @param speedInMetersPerSecond Speed to drive in meters per second (positive
     *                               for forward, negative for reverse)
     * @return Command to drive the specified distance
     */
    public Command driveToDistanceFieldOriented(double distanceInMeters, double speedInMetersPerSecond) {
        return Commands.sequence(
                // Capture starting pose
                Commands.runOnce(() -> startPose = swerveDrive.getPose()),

                // Drive relative to current heading
                Commands.run(() -> {
                    // Get current heading
                    Rotation2d heading = subsystem.getHeading();

                    // Calculate velocity vector in field coordinates
                    // Direction is based on sign of speedInMetersPerSecond
                    double speed = Math.abs(speedInMetersPerSecond);
                    int direction = (speedInMetersPerSecond >= 0) ? 1 : -1;

                    // Use direction to determine whether to move in heading direction or opposite
                    double xVel = direction * speed * Math.cos(heading.getRadians());
                    double yVel = direction * speed * Math.sin(heading.getRadians());

                    // Drive using field-oriented control
                    subsystem.driveFieldOriented(new ChassisSpeeds(xVel, yVel, 0));
                })
                .until(() -> {
                    // Calculate distance traveled from start
                    double distanceTraveled = swerveDrive.getPose().getTranslation()
                            .getDistance(startPose.getTranslation());
                    System.out.println("Distance: " + distanceTraveled);
                    return distanceTraveled >= distanceInMeters;
                }),
                // Stop when done
                Commands.runOnce(() -> subsystem.drive(new Translation2d(), 0, true)));
    }

    // fixed
    public Command driveToDistanceCommandFixed(double distanceInMeters, double speedInMetersPerSecond) {
        // Translation2d startPos = swerveDrive.getPose().getTranslation();
        // System.out.println("STARTING POSE X: " + startPos.getX() + "Y: "
        // +startPos.getY());

        return Commands.sequence(
                Commands.runOnce(() -> {
                    startPose = swerveDrive.getPose();
                }),
                Commands.run(() -> subsystem.drive(new ChassisSpeeds(speedInMetersPerSecond, 0, 0)))
                        .until(() -> {
                            // Translation2d startPos = swerveDrive.getPose().getTranslation();
                            // System.out.println("STARTING POSE X: " + startPos.getX() + "Y: "
                            // +startPos.getY());
                            return (swerveDrive.getPose().getTranslation()
                                    .getDistance(startPose.getTranslation()) > distanceInMeters);
                        }),
                Commands.runOnce(() -> subsystem.drive(new ChassisSpeeds(0, 0, 0))));
    }

    // fixed
    public Command driveToDistanceCommandFieldOrientated(double distanceInMeters, double speedInMetersPerSecond) {
        // Translation2d startPos = swerveDrive.getPose().getTranslation();
        // System.out.println("STARTING POSE X: " + startPos.getX() + "Y: "
        // +startPos.getY());

                return Commands.sequence(
                    Commands.runOnce(() -> {
                        startPose = swerveDrive.getPose();
                    }),
                    Commands.run(() -> subsystem.drive(ChassisSpeeds.fromFieldRelativeSpeeds(new ChassisSpeeds(speedInMetersPerSecond, 0,0), subsystem.getSwerveDrive().getOdometryHeading())))
                            .until(() -> {
                                // Translation2d startPos = swerveDrive.getPose().getTranslation();
                                // System.out.println("STARTING POSE X: " + startPos.getX() + "Y: "
                                // +startPos.getY());
                                return (swerveDrive.getPose().getTranslation()
                                        .getDistance(startPose.getTranslation()) > distanceInMeters);
                            }),
                    Commands.runOnce(() -> subsystem.drive(new ChassisSpeeds(0, 0, 0))));
    }

    /**
     * Command to drive the robot using translative values and heading as angular
     * velocity.
     *
     * @param translationX     Translation in the X direction. Cubed for smoother
     *                         controls.
     * @param translationY     Translation in the Y direction. Cubed for smoother
     *                         controls.
     * @param angularRotationX Angular velocity of the robot to set. Cubed for
     *                         smoother controls.
     * @return Drive command.
     */
    public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY,
            DoubleSupplier angularRotationX) {
        return Commands.run(() -> {
            // Make the robot move
            swerveDrive.drive(SwerveMath.scaleTranslation(new Translation2d(
                    translationX.getAsDouble() * swerveDrive.getMaximumChassisVelocity(),
                    translationY.getAsDouble() * swerveDrive.getMaximumChassisVelocity()), 0.8),
                    Math.pow(angularRotationX.getAsDouble(), 3) * swerveDrive.getMaximumChassisAngularVelocity(),
                    true,
                    false);
        });
    }

    // public Command driveUntilCANrangeCommand(double rangeDistanceinInches, double speedInMetersPerSecond) {
    //     return Commands.run(() -> {
    //         subsystem.drive(new ChassisSpeeds(speedInMetersPerSecond, 0, 0));
    //     }).until(() -> {
    //         double currentDistance = RobotMap.HPSrange.getDistanceInInches();
    //         return currentDistance <= rangeDistanceinInches;
    //     }).andThen(
    //         Commands.runOnce(() -> subsystem.drive(new ChassisSpeeds(0, 0, 0)))
    //     );
    // }

    /**
     * Command to drive the robot using translative values and heading as a
     * setpoint.
     *
     * @param translationX Translation in the X direction. Cubed for smoother
     *                     controls.
     * @param translationY Translation in the Y direction. Cubed for smoother
     *                     controls.
     * @param headingX     Heading X to calculate angle of the joystick.
     * @param headingY     Heading Y to calculate angle of the joystick.
     * @return Drive command.
     */
    public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier headingX,
            DoubleSupplier headingY) {
        // swerveDrive.setHeadingCorrection(true); // Normally you would want heading
        // correction for this kind of control.
        return Commands.run(() -> {

            Translation2d scaledInputs = SwerveMath.scaleTranslation(new Translation2d(translationX.getAsDouble(),
                    translationY.getAsDouble()), 0.8);

            // Make the robot move
            subsystem.driveFieldOriented(
                    swerveDrive.swerveController.getTargetSpeeds(scaledInputs.getX(), scaledInputs.getY(),
                            headingX.getAsDouble(),
                            headingY.getAsDouble(),
                            swerveDrive.getOdometryHeading().getRadians(),
                            swerveDrive.getMaximumChassisVelocity()));
        });
    }

    /**
     * Drive the robot given a chassis field oriented velocity.
     *
     * @param velocity Velocity according to the field.
     */
    public Command driveFieldOriented(Supplier<ChassisSpeeds> velocity) {
        return Commands.run(() -> {
            swerveDrive.driveFieldOriented(velocity.get());
        }, subsystem);
    }

    /**
     * Get the path follower with events.
     *
     * @param pathName PathPlanner path name.
     * @return {@link AutoBuilder#followPath(PathPlannerPath)} path command.
     */
    public Command getAutonomousCommand(String pathName) {
        // Create a path following command using AutoBuilder. This will also trigger
        // event markers.
        return new PathPlannerAuto(pathName);
    }

    public SwerveDrive getSwerveDrive() {
        return swerveDrive;
    }

    public Command disableBackCam(){
        return Commands.runOnce(
            ()->{
                PhotonVision.Cameras.CENTER_CAM.disableCamera();
            }
        );
    }

    public Command enableBackCam(){
        return Commands.runOnce(
            () -> {
                PhotonVision.Cameras.CENTER_CAM.enableCamera();
            }
        );
    }

    public Command disableFrontCams(){
        return Commands.runOnce(
            () -> {
                subsystem.getVision().disableFrontCameras();
                // PhotonVision.Cameras.LEFT_CAM.disableCamera();
                // PhotonVision.Cameras.RIGHT_CAM.disableCamera();
            }
        );
    }

    public Command enableFrontCams(){
        return Commands.runOnce(
            () -> {
                subsystem.getVision().enableFrontCameras();

                // PhotonVision.Cameras.LEFT_CAM.enableCamera();
                // PhotonVision.Cameras.RIGHT_CAM.enableCamera();
            }
        );
    }
}
