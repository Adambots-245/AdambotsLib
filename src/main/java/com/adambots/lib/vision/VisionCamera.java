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

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.adambots.lib.vision.adapters.PhotonVisionResultAdapter;
import com.adambots.lib.vision.config.VisionCameraConfig;
import com.adambots.lib.vision.config.VisionCameraConfig.CameraPurpose;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTablesJNI;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * Configurable vision camera for PhotonVision-based pose estimation.
 *
 * <p>This class encapsulates all functionality for a single PhotonVision camera,
 * including pose estimation, target tracking, and simulation support. It is
 * configured via {@link VisionCameraConfig} for year-to-year reusability.
 *
 * <p><strong>Features:</strong>
 * <ul>
 *   <li>Independent pose estimation per camera</li>
 *   <li>Configurable standard deviations</li>
 *   <li>AprilTag filtering by ID</li>
 *   <li>Purpose-based filtering (ODOMETRY, ALIGNMENT, BOTH)</li>
 *   <li>Full simulation support</li>
 *   <li>Enable/disable per camera</li>
 * </ul>
 *
 * <p><strong>Usage Example:</strong>
 * <pre>{@code
 * VisionCameraConfig config = new VisionCameraConfig(
 *     "LeftCam",
 *     CameraPurpose.ODOMETRY,
 *     new Translation3d(0.381, 0.298, 0.203),
 *     new Rotation3d(0, 0, Math.toRadians(-30)),
 *     VisionStdDevs.DEFAULT_SINGLE_TAG,
 *     VisionStdDevs.DEFAULT_MULTI_TAG,
 *     new int[]{6, 7, 8, 9, 10, 11},
 *     4.0,  // Max tag distance in meters
 *     PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR
 * );
 *
 * VisionCamera camera = new VisionCamera(config, fieldLayout);
 *
 * // In periodic loop
 * Optional<EstimatedRobotPose> pose = camera.getEstimatedGlobalPose();
 * }</pre>
 *
 * @see VisionCameraConfig
 * @see PhotonVision
 * @see VisionCameraInterface
 */
public class VisionCamera implements VisionCameraInterface {

    private final VisionCameraConfig config;
    private final AprilTagFieldLayout fieldLayout;

    /**
     * Latency alert to use when high latency is detected.
     */
    private final Alert latencyAlert;

    /**
     * Camera instance for communications.
     */
    private final PhotonCamera camera;

    /**
     * Pose estimator for this camera.
     */
    private final PhotonPoseEstimator poseEstimator;

    /**
     * Transform of the camera rotation and translation relative to robot center.
     */
    private final Transform3d robotToCamTransform;

    /**
     * Current standard deviations being used.
     */
    private Matrix<N3, N1> curStdDevs;

    /**
     * Estimated robot pose from this camera.
     */
    private Optional<EstimatedRobotPose> estimatedRobotPose = Optional.empty();

    /**
     * Simulated camera instance (only exists during simulations).
     */
    private PhotonCameraSim cameraSim;

    /**
     * Results list to be updated periodically and cached.
     */
    private List<PhotonPipelineResult> resultsList = new ArrayList<>();

    /**
     * Last read timestamp to prevent lag due to slow data fetches.
     */
    private double lastReadTimestamp = Microseconds.of(NetworkTablesJNI.now()).in(Seconds);

    /**
     * Runtime tag filter. When non-null, overrides the config-time allowedTagIDs.
     * Null means use the config-time filter (or allow all tags if none configured).
     */
    private int[] runtimeAllowedTagIds = null;

    /**
     * Whether this camera is currently enabled.
     */
    private boolean enabled = true;

    /**
     * Whether any target was found in the latest update.
     */
    private boolean hasTarget = false;

    /**
     * Creates a new VisionCamera with the given configuration.
     *
     * @param config The camera configuration
     * @param fieldLayout The AprilTag field layout for pose estimation
     */
    public VisionCamera(VisionCameraConfig config, AprilTagFieldLayout fieldLayout) {
        this.config = config;
        this.fieldLayout = fieldLayout;

        latencyAlert = new Alert("'" + config.name() + "' Camera is experiencing high latency.", AlertType.kWarning);

        camera = new PhotonCamera(config.name());

        robotToCamTransform = new Transform3d(
            config.robotToCamTranslation(),
            config.robotToCamRotation()
        );

        poseEstimator = new PhotonPoseEstimator(
            fieldLayout,
            config.poseStrategy(),
            robotToCamTransform
        );
        poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

        curStdDevs = config.singleTagStdDevs().toMatrix();

        if (RobotBase.isSimulation()) {
            initializeSimulation();
        }
    }

    /**
     * Initializes simulation components with default properties.
     */
    private void initializeSimulation() {
        SimCameraProperties cameraProp = new SimCameraProperties();
        // A 960 x 720 camera with a 100 degree diagonal FOV
        cameraProp.setCalibration(960, 720, Rotation2d.fromDegrees(100));
        // Approximate detection noise with average and standard deviation error in pixels
        cameraProp.setCalibError(0.25, 0.08);
        // Set the camera image capture framerate
        cameraProp.setFPS(30);
        // The average and standard deviation in milliseconds of image data latency
        cameraProp.setAvgLatencyMs(35);
        cameraProp.setLatencyStdDevMs(5);

        cameraSim = new PhotonCameraSim(camera, cameraProp);
        cameraSim.enableDrawWireframe(true);
    }

    /**
     * Initializes simulation components with custom properties.
     *
     * @param resWidth Camera resolution width in pixels
     * @param resHeight Camera resolution height in pixels
     * @param fovDegrees Field of view in degrees (diagonal)
     * @param fps Frames per second
     * @param avgLatencyMs Average latency in milliseconds
     * @param latencyStdDevMs Latency standard deviation in milliseconds
     */
    public void initializeSimulation(int resWidth, int resHeight, double fovDegrees,
                                     int fps, double avgLatencyMs, double latencyStdDevMs) {
        if (!RobotBase.isSimulation()) {
            return;
        }

        SimCameraProperties cameraProp = new SimCameraProperties();
        cameraProp.setCalibration(resWidth, resHeight, Rotation2d.fromDegrees(fovDegrees));
        cameraProp.setCalibError(0.25, 0.08);
        cameraProp.setFPS(fps);
        cameraProp.setAvgLatencyMs(avgLatencyMs);
        cameraProp.setLatencyStdDevMs(latencyStdDevMs);

        cameraSim = new PhotonCameraSim(camera, cameraProp);
        cameraSim.enableDrawWireframe(true);
    }

    /**
     * Adds this camera to a vision system simulation.
     *
     * @param systemSim The VisionSystemSim to add to
     */
    public void addToVisionSim(VisionSystemSim systemSim) {
        if (RobotBase.isSimulation() && cameraSim != null) {
            systemSim.addCamera(cameraSim, robotToCamTransform);
        }
    }

    /**
     * Clears the cached results list.
     */
    public void clearCache() {
        resultsList.clear();
        hasTarget = false;
    }

    /**
     * Gets the configuration for this camera.
     *
     * @return The VisionCameraConfig
     */
    public VisionCameraConfig getConfig() {
        return config;
    }

    /**
     * Gets the camera name.
     *
     * @return The camera name
     */
    public String getName() {
        return config.name();
    }

    /**
     * Gets the camera purpose.
     *
     * @return The CameraPurpose
     */
    public CameraPurpose getPurpose() {
        return config.purpose();
    }

    /**
     * Checks if this camera is for odometry/pose estimation.
     *
     * @return true if the camera is for odometry
     */
    public boolean isForOdometry() {
        return config.isForOdometry();
    }

    /**
     * Checks if this camera is for alignment/targeting.
     *
     * @return true if the camera is for alignment
     */
    public boolean isForAlignment() {
        return config.isForAlignment();
    }

    /**
     * Checks if this camera is currently enabled.
     *
     * @return true if the camera is enabled
     */
    public boolean isEnabled() {
        return enabled;
    }

    /**
     * Enables this camera.
     */
    public void enable() {
        this.enabled = true;
    }

    /**
     * Disables this camera.
     */
    public void disable() {
        this.enabled = false;
    }

    /**
     * Sets whether this camera is enabled.
     *
     * @param enabled true to enable, false to disable
     */
    public void setEnabled(boolean enabled) {
        this.enabled = enabled;
    }

    /**
     * Sets a runtime tag filter restricting which AprilTags are used for pose estimation.
     *
     * <p>Pass {@code null} or an empty array to restore the config-time tag filter.
     *
     * @param allowedTagIds Array of allowed tag IDs, or null/empty to restore defaults
     */
    @Override
    public void setTagFilter(int[] allowedTagIds) {
        this.runtimeAllowedTagIds = (allowedTagIds != null && allowedTagIds.length > 0)
            ? allowedTagIds.clone() : null;
    }

    /**
     * Checks whether any tag filter is active (either runtime or config-time).
     */
    private boolean hasActiveTagFilter() {
        return runtimeAllowedTagIds != null || config.hasTagFilter();
    }

    /**
     * Checks whether a specific tag ID is currently allowed by the active filter.
     */
    private boolean isTagCurrentlyAllowed(int tagId) {
        int[] activeFilter = runtimeAllowedTagIds;
        if (activeFilter != null) {
            for (int id : activeFilter) {
                if (id == tagId) return true;
            }
            return false;
        }
        // Fall back to config-time filter
        return config.hasTagFilter() ? config.isTagAllowed(tagId) : true;
    }

    /**
     * Checks if this camera has a valid target in the current frame.
     *
     * @return true if a target was found
     */
    public boolean hasTarget() {
        return hasTarget;
    }

    /**
     * Gets the current standard deviations being used.
     *
     * @return The current standard deviation matrix
     */
    public Matrix<N3, N1> getCurrentStdDevs() {
        return curStdDevs;
    }

    /**
     * Gets the estimated robot pose. Updates the current robot pose estimation,
     * standard deviations, and flushes the cache of results.
     *
     * @return Estimated pose, or empty if not available
     */
    public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
        if (!enabled) {
            return Optional.empty();
        }
        updateUnreadResults();
        return estimatedRobotPose;
    }

    /**
     * Gets the result with the least ambiguity from the best tracked target
     * within the cache. This may not be the most recent result.
     *
     * <p>This is the interface-compliant version that returns a {@link VisionResult}.
     * For direct PhotonVision access, use {@link #getBestPhotonResult()}.
     *
     * @return The result in the cache with the least ambiguous best tracked target
     */
    @Override
    public Optional<? extends VisionResult> getBestResult() {
        return getBestPhotonResult().map(PhotonVisionResultAdapter::new);
    }

    /**
     * Gets the result with the least ambiguity from the best tracked target
     * within the cache. This may not be the most recent result.
     *
     * <p>This returns the raw PhotonVision type for direct access.
     *
     * @return The result in the cache with the least ambiguous best tracked target
     */
    public Optional<PhotonPipelineResult> getBestPhotonResult() {
        if (resultsList.isEmpty()) {
            return Optional.empty();
        }

        PhotonPipelineResult bestResult = null;
        double bestAmbiguity = Double.MAX_VALUE;

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
        return Optional.ofNullable(bestResult);
    }

    /**
     * Gets the latest result from the current cache.
     *
     * <p>This is the interface-compliant version that returns a {@link VisionResult}.
     * For direct PhotonVision access, use {@link #getLatestPhotonResult()}.
     *
     * @return Empty optional if nothing is found, latest result if something is there
     */
    @Override
    public Optional<? extends VisionResult> getLatestResult() {
        return getLatestPhotonResult().map(PhotonVisionResultAdapter::new);
    }

    /**
     * Gets the latest result from the current cache.
     *
     * <p>This returns the raw PhotonVision type for direct access.
     *
     * @return Empty optional if nothing is found, latest result if something is there
     */
    public Optional<PhotonPipelineResult> getLatestPhotonResult() {
        return resultsList.isEmpty() ? Optional.empty() : Optional.of(resultsList.get(resultsList.size() - 1));
    }

    /**
     * Gets the cached results list.
     *
     * @return The list of cached PhotonPipelineResults
     */
    public List<PhotonPipelineResult> getResultsList() {
        return resultsList;
    }

    /**
     * Gets the underlying PhotonCamera.
     *
     * @return The PhotonCamera instance
     */
    public PhotonCamera getPhotonCamera() {
        return camera;
    }

    /**
     * Gets the PhotonPoseEstimator for this camera.
     *
     * @return The PhotonPoseEstimator
     */
    public PhotonPoseEstimator getPoseEstimator() {
        return poseEstimator;
    }

    /**
     * Gets the simulated camera (only valid during simulation).
     *
     * @return The PhotonCameraSim, or null if not in simulation
     */
    public PhotonCameraSim getCameraSim() {
        return cameraSim;
    }

    /**
     * Gets the robot-to-camera transform.
     *
     * @return The Transform3d from robot center to camera
     */
    public Transform3d getRobotToCamTransform() {
        return robotToCamTransform;
    }

    /**
     * Updates the latest results, cached with a maximum refresh rate of 1req/15ms.
     * Sorts the list by timestamp.
     */
    private void updateUnreadResults() {
        double currentTimestamp = Microseconds.of(NetworkTablesJNI.now()).in(Seconds);
        double debounceTime = Milliseconds.of(15).in(Seconds);

        // Always clear stale results so callers don't see old frames
        resultsList.clear();
        hasTarget = false;
        estimatedRobotPose = Optional.empty();

        // Only fetch new results if debounce time has passed
        if ((currentTimestamp - lastReadTimestamp) >= debounceTime) {
            resultsList = RobotBase.isReal()
                ? camera.getAllUnreadResults()
                : cameraSim.getCamera().getAllUnreadResults();
            lastReadTimestamp = currentTimestamp;

            resultsList.sort((PhotonPipelineResult a, PhotonPipelineResult b) -> {
                return a.getTimestampSeconds() >= b.getTimestampSeconds() ? 1 : -1;
            });

            if (!resultsList.isEmpty()) {
                if (resultsList.get(resultsList.size() - 1).targets.size() > 0) {
                    hasTarget = true;
                }
                updateEstimatedGlobalPose();
            }
        }
    }

    /**
     * Updates the estimated global pose based on current results.
     */
    private void updateEstimatedGlobalPose() {
        Optional<EstimatedRobotPose> visionEst = Optional.empty();

        for (var result : resultsList) {
            // Skip if there are no targets
            if (!result.hasTargets()) {
                continue;
            }

            // Check tag filtering (runtime filter takes priority over config-time filter)
            if (hasActiveTagFilter()) {
                boolean hasAllowedTag = false;
                for (PhotonTrackedTarget target : result.getTargets()) {
                    if (isTagCurrentlyAllowed(target.getFiducialId())) {
                        hasAllowedTag = true;
                        break;
                    }
                }

                // Skip if no allowed tags
                if (!hasAllowedTag) {
                    continue;
                }
            }

            // Update with the result
            visionEst = poseEstimator.update(result);

            // Verify the pose used allowed tags
            if (visionEst.isPresent() && hasActiveTagFilter()) {
                boolean usedAllowedTag = false;
                for (PhotonTrackedTarget usedTarget : visionEst.get().targetsUsed) {
                    if (isTagCurrentlyAllowed(usedTarget.getFiducialId())) {
                        usedAllowedTag = true;
                        break;
                    }
                }

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
     * Calculates new standard deviations based on number of tags and distance.
     *
     * @param estimatedPose The estimated pose to calculate std devs for
     * @param targets All targets in this camera frame
     */
    private void updateEstimationStdDevs(
            Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets) {

        if (estimatedPose.isEmpty()) {
            curStdDevs = config.singleTagStdDevs().toMatrix();
            return;
        }

        var estStdDevs = config.singleTagStdDevs().toMatrix();
        int numTags = 0;
        double avgDist = 0;

        // Calculate number of tags and average distance
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
            curStdDevs = config.singleTagStdDevs().toMatrix();
        } else {
            avgDist /= numTags;

            // Use multi-tag std devs if multiple targets visible
            if (numTags > 1) {
                estStdDevs = config.multiTagStdDevs().toMatrix();
            }

            // Increase std devs based on distance
            // Reject single-tag poses beyond the configured max distance
            if (numTags == 1 && avgDist > config.maxTagDistanceMeters()) {
                estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
            } else {
                estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
            }

            curStdDevs = estStdDevs;
        }
    }
}
