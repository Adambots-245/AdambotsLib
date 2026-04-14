// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.lib.vision;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.Set;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.photonvision.EstimatedRobotPose;

import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.adambots.lib.vision.adapters.PhotonVisionTargetAdapter;
import com.adambots.lib.vision.config.VisionCameraConfig;
import com.adambots.lib.vision.config.VisionCameraConfig.CameraPurpose;
import com.adambots.lib.vision.config.VisionSystemConfig;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

/**
 * PhotonVision integration for AprilTag-based vision pose estimation.
 *
 * <p>This class provides comprehensive vision functionality including:
 * <ul>
 *   <li>Multi-camera support with independent pose estimation</li>
 *   <li>AprilTag detection and tracking</li>
 *   <li>Vision-corrected odometry updates</li>
 *   <li>Simulation support with PhotonVision sim</li>
 *   <li>Camera filtering by purpose and tag IDs</li>
 * </ul>
 *
 * <p><strong>Usage Example:</strong>
 * <pre>{@code
 * // Define configuration in Constants (import static edu.wpi.first.units.Units.*)
 * VisionSystemConfig config = VisionConfigBuilder.create()
 *     .addCamera("Left")
 *         .position(Inches.of(15), Inches.of(11.75), Inches.of(8))
 *         .rotation(Degrees.of(0), Degrees.of(0), Degrees.of(-30))
 *         .purpose(CameraPurpose.ODOMETRY)
 *         .allowedTags(6, 7, 8, 9, 10, 11)
 *         .done()
 *     .ambiguityThreshold(0.25)
 *     .build();
 *
 * // Initialize vision
 * PhotonVision vision = new PhotonVision(config, swerve::getPose, swerve.field);
 *
 * // In periodic()
 * vision.updatePoseEstimation(
 *     swerveDrive::addVisionMeasurement,
 *     swerveDrive::getSimulationDriveTrainPose
 * );
 * vision.updateVisionField();
 * }</pre>
 *
 * <p><strong>Based on:</strong> Modified from Ironclad 2024's Vision class.
 * @see <a href="https://gitlab.com/ironclad_code/ironclad-2024/-/blob/master/src/main/java/frc/robot/vision/Vision.java">Ironclad Vision</a>
 * @see <a href="https://docs.photonvision.org/">PhotonVision Documentation</a>
 * @see VisionSystem
 */
public class PhotonVision implements VisionSystem {

  /**
   * April Tag Field Layout of the year.
   */
  public static final AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

  /**
   * Ambiguity defined as a value between (0,1). Used for filtering pose estimates.
   * Higher ambiguity = less confident pose estimate.
   */
  private final double maximumAmbiguity;

  /**
   * Photon Vision Simulation
   */
  public VisionSystemSim visionSim;

  /**
   * Current pose from the pose estimator using wheel odometry.
   */
  private Supplier<Pose2d> currentPose;

  /**
   * Field2d for displaying vision targets on the dashboard.
   */
  private Field2d field2d;

  /**
   * When true, all cameras are disabled and no vision measurements are added to odometry.
   * Use this when vision is unreliable or causing issues.
   */
  private boolean allCamerasDisabled = false;

  /**
   * Dynamic scaler for vision std devs. Applied to all cameras uniformly.
   * Default returns 1.0 (no scaling).
   */
  private DoubleSupplier stdDevScaler = () -> 1.0;

  /**
   * List of configurable vision cameras.
   */
  private final List<VisionCamera> cameras = new ArrayList<>();

  /**
   * Map of camera names to VisionCamera instances for quick lookup.
   */
  private final Map<String, VisionCamera> camerasByName = new HashMap<>();

  /**
   * Constructor for the Vision class using configurable cameras.
   *
   * <p>This constructor allows full customization of cameras, standard deviations,
   * and filtering without modifying AdambotsLib source code.
   *
   * <p><strong>Usage Example:</strong>
   * <pre>{@code
   * // import static edu.wpi.first.units.Units.*
   * VisionSystemConfig config = VisionConfigBuilder.create()
   *     .addCamera("Left")
   *         .position(Inches.of(15), Inches.of(11.75), Inches.of(8))
   *         .rotation(Degrees.of(0), Degrees.of(0), Degrees.of(-30))
   *         .purpose(CameraPurpose.ODOMETRY)
   *         .allowedTags(6, 7, 8, 9, 10, 11)
   *         .done()
   *     .ambiguityThreshold(0.25)
   *     .build();
   *
   * PhotonVision vision = new PhotonVision(config, swerve::getPose, swerve.field);
   * }</pre>
   *
   * @param config      The vision system configuration
   * @param currentPose Current pose supplier, typically a method reference to the drivetrain's getPose()
   * @param field       Current field for displaying vision targets
   */
  public PhotonVision(VisionSystemConfig config, Supplier<Pose2d> currentPose, Field2d field) {
    this.currentPose = currentPose;
    this.field2d = field;
    this.maximumAmbiguity = config.ambiguityThreshold();

    // Initialize configurable cameras
    for (VisionCameraConfig cameraConfig : config.cameras()) {
      VisionCamera camera = new VisionCamera(cameraConfig, fieldLayout);
      cameras.add(camera);
      camerasByName.put(cameraConfig.name(), camera);
    }

    if (RobotBase.isSimulation()) {
      visionSim = new VisionSystemSim("Vision");
      visionSim.addAprilTags(fieldLayout);

      for (VisionCamera camera : cameras) {
        camera.addToVisionSim(visionSim);
      }
    }
  }

  /**
   * Calculates a target pose relative to an AprilTag on the field.
   *
   * @param aprilTag    The ID of the AprilTag.
   * @param robotOffset The offset {@link Transform2d} of the robot to apply to
   *                    the pose for the robot to position itself correctly.
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
   * Update the pose estimation by processing camera data and feeding measurements
   * to the provided consumer.
   *
   * @param visionConsumer Accepts (Pose2d pose, double timestampSeconds, Matrix stdDevs)
   * @param simPose Supplies the simulated drivetrain pose (for sim only), empty if not in sim
   */
  @Override
  public void updatePoseEstimation(
          VisionMeasurementConsumer visionConsumer,
          Supplier<Optional<Pose2d>> simPose) {

    if (RobotBase.isSimulation() && simPose.get().isPresent()) {
      visionSim.update(simPose.get().get());
    }

    if (allCamerasDisabled) {
      return;
    }

    for (VisionCamera camera : cameras) {
      if (!camera.isEnabled() || !camera.isForOdometry()) {
        continue;
      }

      Optional<EstimatedRobotPose> poseEst = camera.getEstimatedGlobalPose();
      if (poseEst.isPresent()) {
        var pose = poseEst.get();
        Pose2d estimatedPose2d = pose.estimatedPose.toPose2d();

        // Reject stale measurements — high latency means the robot has moved
        // significantly since the image was captured
        double latency = Timer.getFPGATimestamp() - pose.timestampSeconds;
        if (latency > 0.2) {
          continue;
        }

        if (RobotBase.isSimulation()) {
          visionSim.getDebugField().getObject("VisionEstimation")
              .setPose(estimatedPose2d);
        }

        visionConsumer.accept(
            estimatedPose2d,
            pose.timestampSeconds,
            camera.getCurrentStdDevs().times(Math.max(stdDevScaler.getAsDouble(), 0.01))
        );
      }
    }
  }

  /**
   * Sets a dynamic scaler for vision standard deviations.
   *
   * <p>The scaler is evaluated each cycle and multiplied into the computed std devs
   * before passing to the pose estimator. Use this to reduce vision trust at high
   * robot speeds (motion blur, latency).
   *
   * @param scaler Supplier returning the scale factor (1.0 = no change). Null resets to 1.0.
   */
  @Override
  public void setStdDevScaler(DoubleSupplier scaler) {
    this.stdDevScaler = scaler != null ? scaler : () -> 1.0;
  }

  /**
   * Sets a runtime tag filter restricting which AprilTags are used for pose estimation.
   *
   * <p>Applies to all cameras. Pass {@code null} or an empty array to restore the
   * config-time tag filter for each camera.
   *
   * @param allowedTagIds Array of allowed tag IDs, or null/empty to restore defaults
   */
  @Override
  public void setTagFilter(int[] allowedTagIds) {
    for (VisionCamera camera : cameras) {
      camera.setTagFilter(allowedTagIds);
    }
  }

  /**
   * Get distance of the robot from the AprilTag pose.
   *
   * @param id AprilTag ID
   * @return Distance in meters, or -1.0 if tag doesn't exist
   */
  @Override
  public double getDistanceFromAprilTag(int id) {
    Optional<Pose3d> tag = fieldLayout.getTagPose(id);
    if (tag.isEmpty()) {
      return -1.0;
    }
    return getDistanceToPoint(tag.get().toPose2d().getTranslation());
  }

  /**
   * Get the transform (distance and angle) from the robot to the AprilTag.
   *
   * @param id AprilTag ID
   * @return Transform from robot to tag
   */
  @Override
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
  @Override
  public Rotation2d getYawToAprilTag(int id) {
    Optional<Pose3d> tag = fieldLayout.getTagPose(id);
    if (tag.isEmpty()) {
      return null;
    }
    return getYawToPoint(tag.get().toPose2d().getTranslation());
  }

  @Override
  public Translation2d getTagGroupCenter(int[] tagIds) {
    double x = 0, y = 0;
    int count = 0;
    for (int id : tagIds) {
      Optional<Pose3d> pose = fieldLayout.getTagPose(id);
      if (pose.isPresent()) {
        x += pose.get().getX();
        y += pose.get().getY();
        count++;
      }
    }
    if (count == 0) return new Translation2d();
    return new Translation2d(x / count, y / count);
  }

  @Override
  public double getDistanceToPoint(Translation2d target) {
    return currentPose.get().getTranslation().getDistance(target);
  }

  @Override
  public Rotation2d getYawToPoint(Translation2d target) {
    Pose2d robotPose = currentPose.get();
    double dx = target.getX() - robotPose.getX();
    double dy = target.getY() - robotPose.getY();
    Rotation2d angleToTarget = new Rotation2d(dx, dy);
    return angleToTarget.minus(robotPose.getRotation());
  }

  @Override
  public int getVisibleTagCount(int[] filterIds, double maxAmbiguity) {
    Set<Integer> seen = new HashSet<>();
    for (VisionCamera camera : cameras) {
      if (!camera.isEnabled()) continue;
      for (PhotonPipelineResult result : camera.getResultsList()) {
        for (PhotonTrackedTarget target : result.getTargets()) {
          if (target.getPoseAmbiguity() > maxAmbiguity) continue;
          int fid = target.getFiducialId();
          for (int id : filterIds) {
            if (fid == id) {
              seen.add(fid);
              break;
            }
          }
        }
      }
    }
    return seen.size();
  }

  /**
   * Get the closest visible AprilTag from all cameras.
   *
   * @return The ID of the closest visible tag, or -1 if no tags are visible
   */
  @Override
  public int getClosestVisibleTag() {
    int closestTagID = -1;
    double closestDistance = Double.MAX_VALUE;

    for (VisionCamera camera : cameras) {
      if (!camera.isEnabled()) continue;
      for (PhotonPipelineResult result : camera.getResultsList()) {
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
  @Override
  public boolean isTagVisible(int tagID) {
    for (VisionCamera camera : cameras) {
      if (!camera.isEnabled()) continue;
      for (PhotonPipelineResult result : camera.getResultsList()) {
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
   * @param tagID      AprilTag ID to check
   * @param cameraName Name of the camera to check
   * @return true if the tag is visible in the specified camera
   */
  public boolean isTagVisibleInCamera(int tagID, String cameraName) {
    VisionCamera camera = camerasByName.get(cameraName);
    if (camera == null || !camera.isEnabled()) {
      return false;
    }

    for (PhotonPipelineResult result : camera.getResultsList()) {
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
   * <p>This is the interface-compliant version that returns a {@link VisionTarget}.
   * For direct PhotonVision access, use {@link #getBestPhotonTargetFromCamera(String)}.
   *
   * @param cameraName Name of the camera to check
   * @return VisionTarget with lowest ambiguity, or empty if no targets
   */
  @Override
  public Optional<? extends VisionTarget> getBestTargetFromCamera(String cameraName) {
    PhotonTrackedTarget target = getBestPhotonTargetFromCamera(cameraName);
    return target != null ? Optional.of(new PhotonVisionTargetAdapter(target)) : Optional.empty();
  }

  /**
   * Get the best (lowest ambiguity) target from a specific camera.
   *
   * <p>This returns the raw PhotonVision type for direct access.
   *
   * @param cameraName Name of the camera to check
   * @return PhotonTrackedTarget with lowest ambiguity, or null if no targets
   */
  public PhotonTrackedTarget getBestPhotonTargetFromCamera(String cameraName) {
    VisionCamera camera = camerasByName.get(cameraName);
    if (camera == null || !camera.isEnabled()) {
      return null;
    }

    PhotonTrackedTarget bestTarget = null;
    double bestAmbiguity = Double.MAX_VALUE;

    for (PhotonPipelineResult result : camera.getResultsList()) {
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
   * Get tracked target from a camera by AprilTag ID.
   *
   * @param id         AprilTag ID
   * @param cameraName Name of the camera to check
   * @return Tracked target, or null if not found
   */
  public PhotonTrackedTarget getTargetFromId(int id, String cameraName) {
    VisionCamera camera = camerasByName.get(cameraName);
    if (camera == null || !camera.isEnabled()) {
      return null;
    }

    for (PhotonPipelineResult result : camera.getResultsList()) {
      if (result.hasTargets()) {
        for (PhotonTrackedTarget target : result.getTargets()) {
          if (target.getFiducialId() == id) {
            return target;
          }
        }
      }
    }
    return null;
  }

  /**
   * Check if any of the specified AprilTag IDs are currently visible in any camera.
   *
   * @param tagIDs Array of AprilTag IDs to check
   * @return The first matching tag ID found, or -1 if none are visible
   */
  public int hasID(int[] tagIDs) {
    for (VisionCamera camera : cameras) {
      if (!camera.isEnabled()) continue;
      for (PhotonPipelineResult result : camera.getResultsList()) {
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

  /**
   * Get all detected AprilTag IDs across all enabled cameras.
   *
   * @return List of all detected tag IDs
   */
  public List<Integer> getAllDetectedTagIds() {
    Set<Integer> detectedTagIDs = new HashSet<>();
    for (VisionCamera camera : cameras) {
      if (!camera.isEnabled()) continue;
      for (PhotonPipelineResult result : camera.getResultsList()) {
        if (result.hasTargets()) {
          for (PhotonTrackedTarget target : result.getTargets()) {
            detectedTagIDs.add(target.getFiducialId());
          }
        }
      }
    }
    return new ArrayList<>(detectedTagIDs);
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
   * Check if any AprilTag is currently visible.
   *
   * @return true if any tag is visible in any enabled camera
   */
  @Override
  public boolean hasTarget() {
    for (VisionCamera camera : cameras) {
      if (camera.isEnabled() && camera.hasTarget()) {
        return true;
      }
    }
    return false;
  }

  /**
   * Update the {@link Field2d} to include tracked targets.
   */
  public void updateVisionField() {
    List<PhotonTrackedTarget> targets = new ArrayList<>();

    for (VisionCamera camera : cameras) {
      if (!camera.isEnabled()) continue;
      var resultsList = camera.getResultsList();
      if (!resultsList.isEmpty()) {
        PhotonPipelineResult latest = resultsList.get(0);
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

  // ==================== CAMERA MANAGEMENT METHODS ====================

  /**
   * Gets all cameras.
   *
   * @return List of VisionCameraInterface instances
   */
  @Override
  public List<? extends VisionCameraInterface> getCameras() {
    return List.copyOf(cameras);
  }

  /**
   * Gets a specific camera by name.
   *
   * @param name The camera name
   * @return The VisionCameraInterface, or null if not found
   */
  @Override
  public VisionCameraInterface getCamera(String name) {
    return camerasByName.get(name);
  }

  /**
   * Gets a specific VisionCamera by name (PhotonVision-specific).
   *
   * <p>Use this when you need access to PhotonVision-specific features.
   *
   * @param name The camera name
   * @return The VisionCamera, or null if not found
   */
  public VisionCamera getVisionCamera(String name) {
    return camerasByName.get(name);
  }

  /**
   * Gets all VisionCamera instances (PhotonVision-specific).
   *
   * <p>Use this when you need access to PhotonVision-specific features.
   *
   * @return List of VisionCamera instances
   */
  public List<VisionCamera> getVisionCameras() {
    return List.copyOf(cameras);
  }

  /**
   * Gets all cameras with a specific purpose.
   *
   * @param purpose The purpose to filter by (ODOMETRY, ALIGNMENT, or BOTH)
   * @return List of cameras matching the purpose
   */
  public List<VisionCamera> getCamerasWithPurpose(CameraPurpose purpose) {
    return cameras.stream()
        .filter(c -> c.getPurpose() == purpose || c.getPurpose() == CameraPurpose.BOTH)
        .toList();
  }

  /**
   * Enables all cameras with the specified purpose.
   *
   * @param purpose The purpose to enable
   */
  public void enableCamerasWithPurpose(CameraPurpose purpose) {
    for (VisionCamera camera : cameras) {
      if (camera.getPurpose() == purpose || camera.getPurpose() == CameraPurpose.BOTH) {
        camera.enable();
      }
    }
  }

  /**
   * Disables all cameras with the specified purpose.
   *
   * @param purpose The purpose to disable
   */
  public void disableCamerasWithPurpose(CameraPurpose purpose) {
    for (VisionCamera camera : cameras) {
      if (camera.getPurpose() == purpose || camera.getPurpose() == CameraPurpose.BOTH) {
        camera.disable();
      }
    }
  }

  /**
   * Enables a specific camera by name.
   *
   * @param name The camera name
   */
  public void enableCamera(String name) {
    VisionCamera camera = camerasByName.get(name);
    if (camera != null) {
      camera.enable();
    }
  }

  /**
   * Disables a specific camera by name.
   *
   * @param name The camera name
   */
  public void disableCamera(String name) {
    VisionCamera camera = camerasByName.get(name);
    if (camera != null) {
      camera.disable();
    }
  }

  /**
   * Disable all cameras from contributing to pose estimation.
   * Vision measurements will not be added to odometry.
   */
  @Override
  public void disableAllCameras() {
    allCamerasDisabled = true;
  }

  /**
   * Enable all cameras to contribute to pose estimation.
   */
  @Override
  public void enableAllCameras() {
    allCamerasDisabled = false;
  }

  /**
   * Check if vision is currently disabled.
   *
   * @return true if all cameras are disabled
   */
  public boolean areAllCamerasDisabled() {
    return allCamerasDisabled;
  }

  /**
   * Checks if a camera exists with the given name.
   *
   * @param name The camera name
   * @return true if the camera exists
   */
  public boolean hasCamera(String name) {
    return camerasByName.containsKey(name);
  }

  /**
   * Gets the number of configured cameras.
   *
   * @return Number of configured cameras
   */
  public int getCameraCount() {
    return cameras.size();
  }

  /**
   * Gets the ambiguity threshold used for filtering pose estimates.
   *
   * @return The ambiguity threshold (0-1)
   */
  public double getAmbiguityThreshold() {
    return maximumAmbiguity;
  }

}
