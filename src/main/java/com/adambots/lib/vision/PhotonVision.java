// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.lib.vision;

import static edu.wpi.first.units.Units.Microseconds;
import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.Seconds;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.PhotonUtils;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTablesJNI;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import swervelib.SwerveDrive;
import swervelib.telemetry.SwerveDriveTelemetry;

/**
 * PhotonVision integration for AprilTag-based vision pose estimation.
 *
 * <p>This class provides comprehensive vision functionality including:
 * <ul>
 *   <li>Multi-camera support with independent pose estimation</li>
 *   <li>AprilTag detection and tracking</li>
 *   <li>Vision-corrected odometry updates</li>
 *   <li>Simulation support with PhotonVision sim</li>
 *   <li>Camera filtering for human player vs reef tags</li>
 * </ul>
 *
 * <p><strong>Camera Management:</strong>
 * Cameras can be filtered to only use specific AprilTags:
 * <ul>
 *   <li>Front cameras (LEFT_CAM, RIGHT_CAM) - typically for reef scoring</li>
 *   <li>Back camera (CENTER_CAM) - typically for human player station</li>
 * </ul>
 *
 * <p><strong>Usage Example:</strong>
 * <pre>{@code
 * // In SwerveSubsystem constructor
 * vision = new PhotonVision(this::getPose, field);
 *
 * // In periodic()
 * vision.updatePoseEstimation(swerveDrive);
 * vision.updateVisionField();
 *
 * // Switch to human player cameras only
 * vision.useHumanPlayerCamerasOnly();
 *
 * // Re-enable all cameras
 * vision.useAllCameras();
 * }</pre>
 *
 * <p><strong>Based on:</strong> Modified from Ironclad 2024's Vision class.
 * @see <a href="https://gitlab.com/ironclad_code/ironclad-2024/-/blob/master/src/main/java/frc/robot/vision/Vision.java">Ironclad Vision</a>
 * @see <a href="https://docs.photonvision.org/">PhotonVision Documentation</a>
 */
public class PhotonVision {

  /**
   * April Tag Field Layout of the year.
   */
  public static final AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
  /**
   * Ambiguity defined as a value between (0,1). Used for filtering pose estimates.
   * Higher ambiguity = less confident pose estimate.
   */
  private final double maximumAmbiguity = 0.25;
  /**
   * Photon Vision Simulation
   */
  public VisionSystemSim visionSim;
  /**
   * Count of times that odometry estimates the robot is more than 10 meters away from
   * the AprilTag. Used to filter out incorrect pose jumps.
   */
  private double longDistancePoseEstimationCount = 0;
  /**
   * Current pose from the pose estimator using wheel odometry.
   */
  private Supplier<Pose2d> currentPose;
  /**
   * Field from {@link swervelib.SwerveDrive#field}
   */
  private Field2d field2d;

  /**
   * When true, only the CENTER_CAM (back camera) is used for pose estimation.
   * Front cameras (LEFT_CAM, RIGHT_CAM) are disabled.
   * This is useful when at the human player station where front cameras may see incorrect tags.
   */
  private static boolean useHumanPlayerCamerasOnly = false;

  /**
   * When true, all cameras are disabled and no vision measurements are added to odometry.
   * Use this when vision is unreliable or causing issues.
   */
  private static boolean areAllCamerasDisabled = false;


  /**
   * Constructor for the Vision class.
   *
   * @param currentPose Current pose supplier, should reference
   *                    {@link SwerveDrive#getPose()}
   * @param field       Current field, should be {@link SwerveDrive#field}
   */
  public PhotonVision(Supplier<Pose2d> currentPose, Field2d field) {
    this.currentPose = currentPose;
    this.field2d = field;

    if (RobotBase.isSimulation()) {
      visionSim = new VisionSystemSim("Vision");
      visionSim.addAprilTags(fieldLayout);

      for (Cameras c : Cameras.values()) {
        c.addToVisionSim(visionSim);
      }

      openSimCameraViews();
    }
  }

  /**
   * Calculates a target pose relative to an AprilTag on the field.
   *
   * @param aprilTag    The ID of the AprilTag.
   * @param robotOffset The offset {@link Transform2d} of the robot to apply to
   *                    the pose for the robot to position
   *                    itself correctly.
   * @return The target pose of the AprilTag.
   */
  public static Pose2d getAprilTagPose(int aprilTag, Transform2d robotOffset) {
    Optional<Pose3d> aprilTagPose3d = fieldLayout.getTagPose(aprilTag);
    if (aprilTagPose3d.isPresent()) {
      return aprilTagPose3d.get().toPose2d().transformBy(robotOffset);
    } else {
      throw new RuntimeException("Cannot get AprilTag " + aprilTag + " from field " + fieldLayout.toString());
    }

  }

  /**
   * Update the pose estimation inside of {@link SwerveDrive} with all of the
   * given poses.
   *
   * @param swerveDrive {@link SwerveDrive} instance.
   */
  public void updatePoseEstimation(SwerveDrive swerveDrive) {
    if (SwerveDriveTelemetry.isSimulation && swerveDrive.getSimulationDriveTrainPose().isPresent()) {
      /*
       * In the maple-sim, odometry is simulated using encoder values, accounting for
       * factors like skidding and drifting.
       * As a result, the odometry may not always be 100% accurate.
       * However, the vision system should be able to provide a reasonably accurate
       * pose estimation, even when odometry is incorrect.
       * (This is why teams implement vision system to correct odometry.)
       * Therefore, we must ensure that the actual robot pose is provided in the
       * simulator when updating the vision simulation during the simulation.
       */
      visionSim.update(swerveDrive.getSimulationDriveTrainPose().get());
    }
    for (Cameras camera : Cameras.values()) {
      Cameras.updatedCache = false;
      Optional<EstimatedRobotPose> poseEst = getEstimatedGlobalPose(camera);
      if (poseEst != null && poseEst.isPresent()) {
        var pose = poseEst.get();

        // Skip if all cameras are disabled
        if (areAllCamerasDisabled) {
          continue;
        }

        // If using human player cameras only, only use CENTER_CAM
        if (useHumanPlayerCamerasOnly && camera == Cameras.CENTER_CAM) {
          swerveDrive.addVisionMeasurement(pose.estimatedPose.toPose2d(),
              pose.timestampSeconds,
              camera.curStdDevs);
        }
        // If using all cameras, use front cameras (not CENTER_CAM)
        else if (!useHumanPlayerCamerasOnly && camera != Cameras.CENTER_CAM) {
          swerveDrive.addVisionMeasurement(pose.estimatedPose.toPose2d(),
              pose.timestampSeconds,
              camera.curStdDevs);
        }
      }
    }

  }

  /**
   * Generates the estimated robot pose. Returns empty if:
   * <ul>
   * <li>No Pose Estimates could be generated</li>
   * <li>The generated pose estimate was considered not accurate</li>
   * </ul>
   *
   * @return an {@link EstimatedRobotPose} with an estimated pose, timestamp, and
   *         targets used to create the estimate
   */
  public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Cameras camera) {
    Optional<EstimatedRobotPose> poseEst = camera.getEstimatedGlobalPose();

    if (RobotBase.isSimulation()) {
      Field2d debugField = visionSim.getDebugField();
      // Update simulation debug field with vision estimation
      poseEst.ifPresentOrElse(
          est -> debugField
              .getObject("VisionEstimation")
              .setPose(est.estimatedPose.toPose2d()),
          () -> {
            debugField.getObject("VisionEstimation").setPoses();
          });
    }
    return poseEst;
  }


  /**
   * Get distance of the robot from the AprilTag pose.
   *
   * @param id AprilTag ID
   * @return Distance in meters, or -1.0 if tag doesn't exist
   */
  public double getDistanceFromAprilTag(int id) {
    Optional<Pose3d> tag = fieldLayout.getTagPose(id);
    return tag.map(pose3d -> PhotonUtils.getDistanceToPose(currentPose.get(), pose3d.toPose2d())).orElse(-1.0);
  }

  /**
   * Get the transform (distance and angle) from the robot to the AprilTag.
   *
   * @param id AprilTag ID
   * @return Transform from robot to tag
   */
  public Transform2d getTransformToAprilTag(int id) {
    Pose2d aprilTagPose = getAprilTagPose(id, new Transform2d());
    return aprilTagPose.minus(currentPose.get());
  }

  /**
   * Get the yaw angle (left/right) from the robot to the AprilTag.
   * Positive = tag is to the left, negative = tag is to the right.
   *
   * @param id AprilTag ID
   * @return Rotation2d representing yaw to the tag, or null if tag doesn't exist
   */
  public Rotation2d getYawToAprilTag(int id) {
    Optional<Pose3d> tag = fieldLayout.getTagPose(id);
    if (tag.isEmpty()) {
      return null;
    }

    Pose2d tagPose = tag.get().toPose2d();
    Pose2d robotPose = currentPose.get();

    // Calculate angle from robot to tag
    double dx = tagPose.getX() - robotPose.getX();
    double dy = tagPose.getY() - robotPose.getY();
    Rotation2d angleToTag = new Rotation2d(dx, dy);

    // Return relative angle (difference between robot heading and angle to tag)
    return angleToTag.minus(robotPose.getRotation());
  }

  /**
   * Get the closest visible AprilTag from all cameras.
   *
   * @return The ID of the closest visible tag, or -1 if no tags are visible
   */
  public int getClosestVisibleTag() {
    int closestTagID = -1;
    double closestDistance = Double.MAX_VALUE;

    for (Cameras camera : Cameras.values()) {
      for (PhotonPipelineResult result : camera.resultsList) {
        if (result.hasTargets()) {
          for (PhotonTrackedTarget target : result.getTargets()) {
            int tagID = target.getFiducialId();
            double distance = getDistanceFromAprilTag(tagID);
            if (distance > 0 && distance < closestDistance) {
              closestDistance = distance;
              closestTagID = tagID;
            }
          }
        }
      }
    }

    return closestTagID;
  }

  /**
   * Get the distance to the closest visible AprilTag.
   *
   * @return Distance in meters to the closest tag, or -1.0 if no tags are visible
   */
  public double getDistanceToClosestTag() {
    int closestTag = getClosestVisibleTag();
    if (closestTag == -1) {
      return -1.0;
    }
    return getDistanceFromAprilTag(closestTag);
  }

  /**
   * Check if a specific AprilTag is currently visible in any camera.
   *
   * @param tagID AprilTag ID to check
   * @return true if the tag is visible in any camera
   */
  public boolean isTagVisible(int tagID) {
    for (Cameras camera : Cameras.values()) {
      for (PhotonPipelineResult result : camera.resultsList) {
        if (result.hasTargets()) {
          for (PhotonTrackedTarget target : result.getTargets()) {
            if (target.getFiducialId() == tagID) {
              return true;
            }
          }
        }
      }
    }
    return false;
  }

  /**
   * Check if a specific AprilTag is visible in a specific camera.
   *
   * @param tagID  AprilTag ID to check
   * @param camera Camera to check
   * @return true if the tag is visible in the specified camera
   */
  public boolean isTagVisibleInCamera(int tagID, Cameras camera) {
    for (PhotonPipelineResult result : camera.resultsList) {
      if (result.hasTargets()) {
        for (PhotonTrackedTarget target : result.getTargets()) {
          if (target.getFiducialId() == tagID) {
            return true;
          }
        }
      }
    }
    return false;
  }

  /**
   * Get the best (lowest ambiguity) target from a specific camera.
   *
   * @param camera Camera to check
   * @return PhotonTrackedTarget with lowest ambiguity, or null if no targets
   */
  public PhotonTrackedTarget getBestTargetFromCamera(Cameras camera) {
    PhotonTrackedTarget bestTarget = null;
    double bestAmbiguity = Double.MAX_VALUE;

    for (PhotonPipelineResult result : camera.resultsList) {
      if (result.hasTargets()) {
        for (PhotonTrackedTarget target : result.getTargets()) {
          double ambiguity = target.getPoseAmbiguity();
          if (ambiguity >= 0 && ambiguity < bestAmbiguity) {
            bestAmbiguity = ambiguity;
            bestTarget = target;
          }
        }
      }
    }

    return bestTarget;
  }

  /**
   * Get tracked target from a camera of AprilTagID
   *
   * @param id     AprilTag ID
   * @param camera Camera to check.
   * @return Tracked target.
   */
  public PhotonTrackedTarget getTargetFromId(int id, Cameras camera) {
    PhotonTrackedTarget target = null;
    for (PhotonPipelineResult result : camera.resultsList) {
      if (result.hasTargets()) {
        for (PhotonTrackedTarget i : result.getTargets()) {
          if (i.getFiducialId() == id) {
            return i;
          }
        }
      }
    }
    return target;

  }

  /**
   * Check if any of the specified AprilTag IDs are currently visible in any camera.
   *
   * @param tagIDs Array of AprilTag IDs to check
   * @return The first matching tag ID found, or -1 if none are visible
   */
  public int hasID(int[] tagIDs) {
    for (Cameras camera : Cameras.values()) {
      for (PhotonPipelineResult result : camera.resultsList) {
        if (result.hasTargets()) {
          for (PhotonTrackedTarget target : result.getTargets()) {
            for (int id : tagIDs) {
              if (target.getFiducialId() == id) {
                return target.getFiducialId();
              }
            }
          }
        }
      }
    }
    return -1;
  }

  public static List<Integer> getAllDetectedTags() {
    List<Integer> detectedTagIDs = new ArrayList<>();

    for (Cameras camera : Cameras.values()) {
      for (PhotonPipelineResult result : camera.resultsList) {
        if (result.hasTargets()) {
          for (PhotonTrackedTarget target : result.getTargets()) {
            detectedTagIDs.add(target.getFiducialId());
          }
        }
      }
    }

    return detectedTagIDs;
  }

  /**
   * Vision simulation.
   *
   * @return Vision Simulation
   */
  public VisionSystemSim getVisionSim() {
    return visionSim;
  }

  /**
   * Open up the photon vision camera streams on the localhost, assumes running
   * photon vision on localhost.
   */
  private void openSimCameraViews() {
    // Implementation removed - can be re-added if needed for debugging
  }

  public boolean hasTarget() {
    return Cameras.updatedCache;
  }

  /**
   * Update the {@link Field2d} to include tracked targets/
   */
  public void updateVisionField() {

    List<PhotonTrackedTarget> targets = new ArrayList<PhotonTrackedTarget>();
    for (Cameras c : Cameras.values()) {
      if (!c.resultsList.isEmpty()) {
        PhotonPipelineResult latest = c.resultsList.get(0);
        if (latest.hasTargets()) {
          targets.addAll(latest.targets);
        }
      }
    }

    List<Pose2d> poses = new ArrayList<>();
    for (PhotonTrackedTarget target : targets) {
      if (fieldLayout.getTagPose(target.getFiducialId()).isPresent()) {
        Pose2d targetPose = fieldLayout.getTagPose(target.getFiducialId()).get().toPose2d();
        poses.add(targetPose);
      }
    }

    field2d.getObject("tracked targets").setPoses(poses);
  }

  /**
   * Camera Enum to select each camera
   * Google Search for WPILib Coorinate System for more information -
   * https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html
   * 
   * Rotation 3D:
   * Roll: Side-to-side tilt (rarely used, usually 0)
   * Pitch: Up/down angle
   * 
   * Use a digital angle finder or phone app
   * Measure angle between horizontal and camera's forward view
   * Positive = camera tilted up
   * 
   * 
   * Yaw: Left/right angle from robot's forward direction
   * 
   * Positive = camera rotated left
   * Negative = camera rotated right
   * 
   * Translation 3D:
   * Define robot center (typically center of rotation between wheels)
   * Measure with a tape measure/ruler:
   * 
   * X: Distance forward(+) or back(-) from robot center to camera lens
   * Y: Distance left(+) or right(-) from robot center to camera lens
   * Z: Height from floor to camera lens
   * 
   * VecBuilder:
   * Represents the standard deviations (uncertainty) for vision measurements.
   * First number: X position uncertainty in meters
   * Second number: Y position uncertainty in meters
   * Third number: rotation uncertainty in radians
   * 
   * Higher values = less trust in vision
   * Lower values = more trust in vision
   */
  public enum Cameras {
    /**
     * Center Camera
     * //
     */
    LEFT_CAM("Left",
        new Rotation3d(0, 0, Units.degreesToRadians(-30)),
        new Translation3d(Units.inchesToMeters(15),
            Units.inchesToMeters(11.75),
            Units.inchesToMeters(8)),
        VecBuilder.fill(0.5, 0.5, 0.5), VecBuilder.fill(0.5, 0.5, 1), getReefTagIDs()),
    RIGHT_CAM("Right",
        new Rotation3d(0, Units.degreesToRadians(0), Units.degreesToRadians(30)),
        new Translation3d(Units.inchesToMeters(15),
            Units.inchesToMeters(-11.75),
            Units.inchesToMeters(8)),
        VecBuilder.fill(0.5, 0.5, 0.5), VecBuilder.fill(0.5, 0.5, 1), getReefTagIDs()),
    CENTER_CAM("Middle",
        new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(-43), Units.degreesToRadians(177)),
        new Translation3d(Units.inchesToMeters(8),
            Units.inchesToMeters(0),
            Units.inchesToMeters(41)),
        VecBuilder.fill(0.5, 0.5, 0.5), VecBuilder.fill(0.5, 0.5, 1), getHumanPlayerTagIDs());

    /**
     * Latency alert to use when high latency is detected.
     */
    public final Alert latencyAlert;
    /**
     * Camera instance for comms.
     */
    public final PhotonCamera camera;
    /**
     * Pose estimator for camera.
     */
    public final PhotonPoseEstimator poseEstimator;
    /**
     * Standard Deviation for single tag readings for pose estimation.
     */
    private final Matrix<N3, N1> singleTagStdDevs;
    /**
     * Standard deviation for multi-tag readings for pose estimation.
     */
    private final Matrix<N3, N1> multiTagStdDevs;
    /**
     * Transform of the camera rotation and translation relative to the center of
     * the robot
     */
    private final Transform3d robotToCamTransform;
    /**
     * Current standard deviations used.
     */
    public Matrix<N3, N1> curStdDevs;
    /**
     * Estimated robot pose.
     */
    public Optional<EstimatedRobotPose> estimatedRobotPose;
    /**
     * Simulated camera instance which only exists during simulations.
     */
    public PhotonCameraSim cameraSim;
    /**
     * Results list to be updated periodically and cached to avoid unnecessary
     * queries.
     */
    public List<PhotonPipelineResult> resultsList = new ArrayList<>();
    /**
     * Last read from the camera timestamp to prevent lag due to slow data fetches.
     */
    private double lastReadTimestamp = Microseconds.of(NetworkTablesJNI.now()).in(Seconds);

    private static boolean updatedCache = false;

    private int[] allowedTagIDs;

    /**
     * Construct a Photon Camera class with help. Standard deviations are fake
     * values, experiment and determine
     * estimation noise on an actual robot.
     *
     * @param name                  Name of the PhotonVision camera found in the PV
     *                              UI.
     * @param robotToCamRotation    {@link Rotation3d} of the camera.
     * @param robotToCamTranslation {@link Translation3d} relative to the center of
     *                              the robot.
     * @param singleTagStdDevs      Single AprilTag standard deviations of estimated
     *                              poses from the camera.
     * @param multiTagStdDevsMatrix Multi AprilTag standard deviations of estimated
     *                              poses from the camera.
     */
    Cameras(String name, Rotation3d robotToCamRotation, Translation3d robotToCamTranslation,
        Matrix<N3, N1> singleTagStdDevs, Matrix<N3, N1> multiTagStdDevsMatrix, int[] allowedTagIDs) {
      latencyAlert = new Alert("'" + name + "' Camera is experiencing high latency.", AlertType.kWarning);

      camera = new PhotonCamera(name);

      // https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html
      robotToCamTransform = new Transform3d(robotToCamTranslation, robotToCamRotation);

      poseEstimator = new PhotonPoseEstimator(PhotonVision.fieldLayout,
          PoseStrategy.AVERAGE_BEST_TARGETS,
          robotToCamTransform);
      poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

      this.singleTagStdDevs = singleTagStdDevs;
      this.multiTagStdDevs = multiTagStdDevsMatrix;
      this.allowedTagIDs = allowedTagIDs;

      if (RobotBase.isSimulation()) {
        SimCameraProperties cameraProp = new SimCameraProperties();
        // A 640 x 480 camera with a 100 degree diagonal FOV.
        cameraProp.setCalibration(960, 720, Rotation2d.fromDegrees(100));
        // Approximate detection noise with average and standard deviation error in
        // pixels.
        cameraProp.setCalibError(0.25, 0.08);
        // Set the camera image capture framerate (Note: this is limited by robot loop
        // rate).
        cameraProp.setFPS(30);
        // The average and standard deviation in milliseconds of image data latency.
        cameraProp.setAvgLatencyMs(35);
        cameraProp.setLatencyStdDevMs(5);

        cameraSim = new PhotonCameraSim(camera, cameraProp);
        cameraSim.enableDrawWireframe(true);
      }
    }

    /**
     * Add camera to {@link VisionSystemSim} for simulated photon vision.
     *
     * @param systemSim {@link VisionSystemSim} to use.
     */
    public void addToVisionSim(VisionSystemSim systemSim) {
      if (RobotBase.isSimulation()) {
        systemSim.addCamera(cameraSim, robotToCamTransform);
      }
    }

    public void clearCache() {
      resultsList.clear();
    }

    /**
     * Get the result with the least ambiguity from the best tracked target within
     * the cache. This may not be the most recent result!
     *
     * @return The result in the cache with the least ambiguous best tracked target.
     *         This is not the most recent result!
     */
    public Optional<PhotonPipelineResult> getBestResult() {
      if (resultsList.isEmpty()) {
        return Optional.empty();
      }

      PhotonPipelineResult bestResult = resultsList.get(0);
      if (!bestResult.hasTargets()) {
        return Optional.empty();
      }

      double bestAmbiguity = bestResult.getBestTarget().getPoseAmbiguity();

      for (PhotonPipelineResult result : resultsList) {
        if (!result.hasTargets()) {
          continue;
        }
        double currentAmbiguity = result.getBestTarget().getPoseAmbiguity();
        if (currentAmbiguity < bestAmbiguity && currentAmbiguity > 0) {
          bestResult = result;
          bestAmbiguity = currentAmbiguity;
        }
      }
      return Optional.of(bestResult);
    }

    /**
     * Get the latest result from the current cache.
     *
     * @return Empty optional if nothing is found. Latest result if something is
     *         there.
     */
    public Optional<PhotonPipelineResult> getLatestResult() {
      return resultsList.isEmpty() ? Optional.empty() : Optional.of(resultsList.get(0));
    }

    /**
     * Get the estimated robot pose. Updates the current robot pose estimation,
     * standard deviations, and flushes the
     * cache of results.
     *
     * @return Estimated pose.
     */
    public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
      // System.err.println("Checkpoint 5");
      updateUnreadResults();
      return estimatedRobotPose;
    }

    /**
     * Update the latest results, cached with a maximum refresh rate of 1req/15ms.
     * Sorts the list by timestamp.
     */
    private void updateUnreadResults() {
      double mostRecentTimestamp = resultsList.isEmpty() ? 0.0 : resultsList.get(0).getTimestampSeconds();
      double currentTimestamp = Microseconds.of(NetworkTablesJNI.now()).in(Seconds);
      double debounceTime = Milliseconds.of(15).in(Seconds);

      for (PhotonPipelineResult result : resultsList) {
        mostRecentTimestamp = Math.max(mostRecentTimestamp, result.getTimestampSeconds());
      }

      // Update results if debounce time has passed
      if ((resultsList.isEmpty() || (currentTimestamp - mostRecentTimestamp >= debounceTime)) &&
          (currentTimestamp - lastReadTimestamp) >= debounceTime) {
        resultsList.clear();
        resultsList = RobotBase.isReal() ? camera.getAllUnreadResults() : cameraSim.getCamera().getAllUnreadResults();
        lastReadTimestamp = currentTimestamp;
        resultsList.sort((PhotonPipelineResult a, PhotonPipelineResult b) -> {
          return a.getTimestampSeconds() >= b.getTimestampSeconds() ? 1 : -1;
        });

        if (!resultsList.isEmpty()) {
          if (resultsList.get(0).targets.size() > 0) {
            updatedCache = true;
          }
          updateEstimatedGlobalPose();
        }
      }
    }

    /**
     * The latest estimated robot pose on the field from vision data. This may be
     * empty. This should only be called once
     * per loop.
     *
     * <p>
     * Also includes updates for the standard deviations, which can (optionally) be
     * retrieved with
     * {@link Cameras#updateEstimationStdDevs}
     *
     * @return An {@link EstimatedRobotPose} with an estimated pose, estimate
     *         timestamp, and targets used for
     *         estimation.
     */
    private void updateEstimatedGlobalPose() {
      Optional<EstimatedRobotPose> visionEst = Optional.empty();

      for (var result : resultsList) {
        // Skip this result if there are no targets
        if (!result.hasTargets()) {
          continue;
        }

        // Check if this camera has tag filtering
        if (allowedTagIDs != null && allowedTagIDs.length > 0) {
          // Check if any of the targets match our allowed tag IDs
          boolean hasAllowedTag = false;
          for (PhotonTrackedTarget target : result.getTargets()) {
            for (int id : allowedTagIDs) {
              if (target.getFiducialId() == id) {
                hasAllowedTag = true;
                break;
              }
            }
            if (hasAllowedTag)
              break;
          }

          // Skip this result if it doesn't have any allowed tags
          if (!hasAllowedTag) {
            continue;
          }
        }

        // Update with the result
        visionEst = poseEstimator.update(result);

        // After getting the pose estimate, verify it used allowed tags if filtering is
        // enabled
        if (visionEst.isPresent() && allowedTagIDs != null && allowedTagIDs.length > 0) {
          boolean usedAllowedTag = false;
          for (PhotonTrackedTarget usedTarget : visionEst.get().targetsUsed) {
            for (int id : allowedTagIDs) {
              if (usedTarget.getFiducialId() == id) {
                usedAllowedTag = true;
                break;
              }
            }
            if (usedAllowedTag)
              break;
          }

          // If the pose didn't use any allowed tags, discard it
          if (!usedAllowedTag) {
            visionEst = Optional.empty();
            continue;
          }
        }

        updateEstimationStdDevs(visionEst, result.getTargets());
      }
      estimatedRobotPose = visionEst;
    }

    /**
     * Calculates new standard deviations This algorithm is a heuristic that creates
     * dynamic standard deviations based
     * on number of tags, estimation strategy, and distance from the tags.
     *
     * @param estimatedPose The estimated pose to guess standard deviations for.
     * @param targets       All targets in this camera frame
     */
    private void updateEstimationStdDevs(
        Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets) {
      if (estimatedPose.isEmpty()) {
        // No pose input. Default to single-tag std devs
        curStdDevs = singleTagStdDevs;

      } else {
        // Pose present. Start running Heuristic
        var estStdDevs = singleTagStdDevs;
        int numTags = 0;
        double avgDist = 0;

        // Precalculation - see how many tags we found, and calculate an
        // average-distance metric
        for (var tgt : targets) {
          var tagPose = poseEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
          if (tagPose.isEmpty()) {
            continue;
          }
          numTags++;
          avgDist += tagPose
              .get()
              .toPose2d()
              .getTranslation()
              .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
        }

        if (numTags == 0) {
          // No tags visible. Default to single-tag std devs
          curStdDevs = singleTagStdDevs;
        } else {
          // One or more tags visible, run the full heuristic.
          avgDist /= numTags;
          // Decrease std devs if multiple targets are visible
          if (numTags > 1) {
            estStdDevs = multiTagStdDevs;
          }
          // Increase std devs based on (average) distance
          if (numTags == 1 && avgDist > 4) {
            estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
          } else {
            estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
          }
          curStdDevs = estStdDevs;
        }
      }
    }
  }

  /**
   * Gets a list of tag IDs that are on the human player station (both alliances).
   *
   * <p>For 2025 Reefscape field:
   * <ul>
   *   <li>Tags 1, 2, 4, 5 - Blue alliance human player station</li>
   *   <li>Tags 12, 13, 14, 15 - Red alliance human player station</li>
   * </ul>
   *
   * <p><strong>Note:</strong> Modify these values based on the actual game field layout.
   *
   * @return Array of human player station tag IDs
   */
  public static int[] getHumanPlayerTagIDs() {
    return new int[] { 1, 2, 4, 5, 12, 13, 14, 15 };
  }

  /**
   * Gets a list of tag IDs that are on the reefs (both alliances).
   *
   * <p>For 2025 Reefscape field:
   * <ul>
   *   <li>Tags 6-11 - Red alliance reef</li>
   *   <li>Tags 17-22 - Blue alliance reef</li>
   * </ul>
   *
   * <p><strong>Note:</strong> Modify these values based on the actual game field layout.
   *
   * @return Array of reef tag IDs
   */
  public static int[] getReefTagIDs() {
    return new int[] { 6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22 };
  }

  /**
   * Switch to using only the human player camera (CENTER_CAM).
   * This disables front cameras (LEFT_CAM, RIGHT_CAM) from contributing to pose estimation.
   *
   * <p><strong>Use this when:</strong>
   * <ul>
   *   <li>Robot is at human player station</li>
   *   <li>Front cameras would see incorrect/conflicting tags</li>
   *   <li>You only want back camera for pose estimation</li>
   * </ul>
   */
  public void useHumanPlayerCamerasOnly() {
    useHumanPlayerCamerasOnly = true;
  }

  /**
   * Switch to using all cameras for pose estimation.
   * This enables front cameras (LEFT_CAM, RIGHT_CAM) and disables back camera (CENTER_CAM).
   *
   * <p><strong>Use this when:</strong>
   * <ul>
   *   <li>Robot is on the field away from human player station</li>
   *   <li>Front cameras see reef or other game element tags</li>
   *   <li>You want normal vision-corrected odometry</li>
   * </ul>
   */
  public void useAllCameras() {
    useHumanPlayerCamerasOnly = false;
  }

  /**
   * Disable all cameras from contributing to pose estimation.
   * Vision measurements will not be added to odometry.
   *
   * <p><strong>Use this when:</strong>
   * <ul>
   *   <li>Vision is unreliable or causing problems</li>
   *   <li>You want pure wheel odometry</li>
   *   <li>Debugging odometry issues</li>
   * </ul>
   */
  public void disableAllCameras() {
    areAllCamerasDisabled = true;
  }

  /**
   * Enable all cameras to contribute to pose estimation.
   * Respects the current camera filtering mode (all cameras vs human player cameras only).
   */
  public void enableAllCameras() {
    areAllCamerasDisabled = false;
  }

  /**
   * Check if vision is currently disabled.
   *
   * @return true if all cameras are disabled
   */
  public boolean areAllCamerasDisabled() {
    return areAllCamerasDisabled;
  }

  /**
   * Check if only human player cameras are active.
   *
   * @return true if only CENTER_CAM is used for pose estimation
   */
  public boolean isUsingHumanPlayerCamerasOnly() {
    return useHumanPlayerCamerasOnly;
  }
}
