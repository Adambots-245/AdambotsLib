// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.lib.subsystems;

import com.pathplanner.lib.config.PIDConstants;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

/**
 * Configuration object for SwerveSubsystem settings.
 *
 * <p>This class provides a builder-style API for configuring swerve drive parameters
 * that may need to be customized per-robot. Using this configuration object allows
 * projects to override default values without modifying AdambotsLib source code.
 *
 * <p><strong>Configurable Settings:</strong>
 * <ul>
 *   <li>PathPlanner translation PID (controls X/Y position accuracy during auto)</li>
 *   <li>PathPlanner rotation PID (controls heading accuracy during auto)</li>
 *   <li>Heading correction enable/disable</li>
 *   <li>Cosine compensation enable/disable</li>
 * </ul>
 *
 * <p><strong>Usage Example:</strong>
 * <pre>{@code
 * // In your robot project's Constants file
 * public static final SwerveConfig SWERVE_CONFIG = new SwerveConfig()
 *     .withTranslationPID(5.0, 0.0, 0.0)
 *     .withRotationPID(3.0, 0.0, 0.0)
 *     .withHeadingCorrection(true)
 *     .withCosineCompensation(true);
 *
 * // In RobotContainer
 * SwerveSubsystem swerve = new SwerveSubsystem(
 *     new File(Filesystem.getDeployDirectory(), "swerve/kraken"),
 *     Constants.SWERVE_CONFIG
 * );
 * }</pre>
 *
 * <p><strong>Default Values:</strong>
 * <ul>
 *   <li>Translation PID: P=5.0, I=0.0, D=0.0</li>
 *   <li>Rotation PID: P=5.0, I=0.0, D=0.0</li>
 *   <li>Heading Correction: disabled</li>
 *   <li>Cosine Compensation: enabled</li>
 *   <li>Telemetry Verbosity: HIGH</li>
 * </ul>
 *
 * @see SwerveSubsystem#SwerveSubsystem(java.io.File, SwerveConfig)
 */
public class SwerveConfig {

    // PathPlanner PID defaults
    private double translationP = 5.0;
    private double translationI = 0.0;
    private double translationD = 0.0;

    private double rotationP = 5.0;
    private double rotationI = 0.0;
    private double rotationD = 0.0;

    // Drive behavior defaults
    private boolean headingCorrectionEnabled = false;
    private boolean cosineCompensationEnabled = true;
    private boolean angularVelocityCompensationEnabled = true;
    private double angularVelocityCoeff = 0.1;

    // PathPlanner feedforward
    private boolean feedforwardEnabled = true;

    // Telemetry
    private TelemetryVerbosity telemetryVerbosity = TelemetryVerbosity.HIGH;

    // Encoder auto-sync
    private boolean encoderAutoSyncEnabled = false;
    private double encoderAutoSyncDeadbandDeg = 1.0;

    // Vision/Odometry control
    private boolean useManualOdometry = true;

    /**
     * Creates a new SwerveConfig with default values.
     */
    public SwerveConfig() {
    }

    /**
     * Sets the PathPlanner translation PID constants.
     *
     * <p>These values control how accurately the robot follows the X/Y position
     * of autonomous paths. Higher P values make the robot respond more aggressively
     * to position errors.
     *
     * <p><strong>Tuning Tips:</strong>
     * <ul>
     *   <li>Start with P=5.0, I=0, D=0</li>
     *   <li>Increase P if robot is sluggish following paths</li>
     *   <li>Decrease P if robot oscillates around the path</li>
     *   <li>Add small D (0.1-0.5) to reduce overshoot</li>
     * </ul>
     *
     * @param p Proportional gain
     * @param i Integral gain
     * @param d Derivative gain
     * @return This config for chaining
     */
    public SwerveConfig withTranslationPID(double p, double i, double d) {
        this.translationP = p;
        this.translationI = i;
        this.translationD = d;
        return this;
    }

    /**
     * Sets the PathPlanner rotation PID constants.
     *
     * <p>These values control how accurately the robot maintains heading during
     * autonomous paths. Higher P values make the robot correct heading errors
     * more aggressively.
     *
     * <p><strong>Tuning Tips:</strong>
     * <ul>
     *   <li>Start with P=5.0, I=0, D=0</li>
     *   <li>Increase P if robot doesn't turn sharply enough</li>
     *   <li>Decrease P if robot oscillates when turning</li>
     *   <li>Add small D (0.1-0.5) to reduce heading overshoot</li>
     * </ul>
     *
     * @param p Proportional gain
     * @param i Integral gain
     * @param d Derivative gain
     * @return This config for chaining
     */
    public SwerveConfig withRotationPID(double p, double i, double d) {
        this.rotationP = p;
        this.rotationI = i;
        this.rotationD = d;
        return this;
    }

    /**
     * Sets both translation and rotation PID using PIDConstants objects.
     *
     * <p><strong>Usage Example:</strong>
     * <pre>{@code
     * new SwerveConfig().withPIDConstants(
     *     new PIDConstants(5.0, 0.0, 0.0),
     *     new PIDConstants(3.0, 0.0, 0.0)
     * );
     * }</pre>
     *
     * @param translation Translation PID constants
     * @param rotation Rotation PID constants
     * @return This config for chaining
     */
    public SwerveConfig withPIDConstants(PIDConstants translation, PIDConstants rotation) {
        this.translationP = translation.kP;
        this.translationI = translation.kI;
        this.translationD = translation.kD;
        this.rotationP = rotation.kP;
        this.rotationI = rotation.kI;
        this.rotationD = rotation.kD;
        return this;
    }

    /**
     * Enables or disables heading correction.
     *
     * <p>Heading correction uses the heading PID to automatically maintain
     * the robot's heading during teleop driving. When enabled, the robot
     * will resist rotation caused by external forces.
     *
     * <p><strong>When to Enable:</strong>
     * <ul>
     *   <li>Robot tends to drift rotationally during straight-line driving</li>
     *   <li>You want the robot to maintain heading while strafing</li>
     * </ul>
     *
     * <p><strong>When to Disable:</strong>
     * <ul>
     *   <li>Driver prefers full manual control of rotation</li>
     *   <li>Heading correction causes jerky behavior</li>
     * </ul>
     *
     * @param enabled true to enable heading correction
     * @return This config for chaining
     */
    public SwerveConfig withHeadingCorrection(boolean enabled) {
        this.headingCorrectionEnabled = enabled;
        return this;
    }

    /**
     * Enables or disables cosine compensation.
     *
     * <p>Cosine compensation adjusts wheel speeds based on their alignment
     * with the desired movement direction. This improves efficiency when
     * wheels aren't perfectly aligned.
     *
     * <p><strong>Note:</strong> Does not work in simulation.
     *
     * @param enabled true to enable cosine compensation
     * @return This config for chaining
     */
    public SwerveConfig withCosineCompensation(boolean enabled) {
        this.cosineCompensationEnabled = enabled;
        return this;
    }

    /**
     * Enables or disables angular velocity compensation.
     *
     * <p>Angular velocity compensation counteracts the skewing effect that
     * occurs when the robot moves linearly while rotating. This helps
     * maintain straighter trajectories during combined movements.
     *
     * @param enabled true to enable angular velocity compensation
     * @param coefficient Compensation coefficient (default 0.1)
     * @return This config for chaining
     */
    public SwerveConfig withAngularVelocityCompensation(boolean enabled, double coefficient) {
        this.angularVelocityCompensationEnabled = enabled;
        this.angularVelocityCoeff = coefficient;
        return this;
    }

    /**
     * Enables or disables feedforward for PathPlanner autonomous driving.
     *
     * <p>When enabled (default), PathPlanner uses a feedforward-based drive method:
     * <pre>{@code
     * swerveDrive.drive(speedsRobotRelative, swerveModuleStates, moduleFeedForwards.linearForces());
     * }</pre>
     *
     * <p>When disabled, it uses the simpler method:
     * <pre>{@code
     * swerveDrive.setChassisSpeeds(speedsRobotRelative);
     * }</pre>
     *
     * <p><strong>When to Disable:</strong>
     * <ul>
     *   <li>Running in simulation with maple-sim (feedforward doesn't work properly)</li>
     *   <li>Debugging autonomous path following issues</li>
     *   <li>If feedforward causes erratic behavior on your robot</li>
     * </ul>
     *
     * <p><strong>Usage Example:</strong>
     * <pre>{@code
     * // Disable feedforward for simulation
     * SwerveConfig config = new SwerveConfig()
     *     .withFeedforward(false);
     *
     * // Or conditionally based on simulation
     * SwerveConfig config = new SwerveConfig()
     *     .withFeedforward(!RobotBase.isSimulation());
     * }</pre>
     *
     * @param enabled true to enable feedforward (default), false to disable
     * @return This config for chaining
     */
    public SwerveConfig withFeedforward(boolean enabled) {
        this.feedforwardEnabled = enabled;
        return this;
    }

    /**
     * Enables or disables steering encoder auto-synchronization.
     *
     * <p>When enabled, YAGSL will automatically synchronize the steering relative
     * encoder with the absolute encoder when the robot is at rest and the difference
     * exceeds the deadband.
     *
     * <p><strong>When to Enable:</strong>
     * <ul>
     *   <li>After initial robot setup and testing are complete</li>
     *   <li>When you want to correct steering encoder drift automatically</li>
     * </ul>
     *
     * <p><strong>When to Disable (default):</strong>
     * <ul>
     *   <li>During initial setup before encoder offsets are calibrated</li>
     *   <li>If auto-sync causes unexpected wheel snapping</li>
     * </ul>
     *
     * @param enabled true to enable auto-sync
     * @param deadbandDegrees Minimum difference in degrees before sync occurs (default 1.0)
     * @return This config for chaining
     */
    public SwerveConfig withEncoderAutoSync(boolean enabled, double deadbandDegrees) {
        this.encoderAutoSyncEnabled = enabled;
        this.encoderAutoSyncDeadbandDeg = deadbandDegrees;
        return this;
    }

    /**
     * Controls whether to stop YAGSL's odometry thread and enable vision pose estimation.
     *
     * <p><strong>Note:</strong> Odometry is always updated in periodic() regardless of this
     * setting. This setting only controls:
     * <ul>
     *   <li>Whether to stop YAGSL's internal odometry thread (to avoid duplicate updates)</li>
     *   <li>Whether to enable vision pose estimation (requires synchronized updates)</li>
     * </ul>
     *
     * <p>When enabled (default), the SwerveSubsystem:
     * <ul>
     *   <li>Stops YAGSL's internal odometry thread</li>
     *   <li>Enables vision pose estimation if vision is configured</li>
     * </ul>
     *
     * <p>When disabled:
     * <ul>
     *   <li>YAGSL's internal odometry thread continues running (may cause duplicate updates)</li>
     *   <li>Vision pose estimation is disabled</li>
     * </ul>
     *
     * <p><strong>When to Disable:</strong>
     * <ul>
     *   <li>Not using vision pose estimation</li>
     *   <li>Testing without vision cameras</li>
     * </ul>
     *
     * <p><strong>Usage Example:</strong>
     * <pre>{@code
     * // Disable vision integration (still updates odometry)
     * SwerveConfig config = new SwerveConfig()
     *     .withManualOdometry(false);
     * }</pre>
     *
     * @param enabled true to stop YAGSL thread and enable vision (default), false otherwise
     * @return This config for chaining
     */
    /**
     * Sets the YAGSL telemetry verbosity level.
     *
     * <p>Controls how much swerve telemetry data is published to NetworkTables.
     * Higher verbosity is useful for debugging but adds CAN/NT overhead.
     *
     * <p><strong>Levels:</strong>
     * <ul>
     *   <li>{@code NONE} — No telemetry</li>
     *   <li>{@code LOW} — Minimal (module states only)</li>
     *   <li>{@code INFO} — Moderate (adds chassis speeds)</li>
     *   <li>{@code POSE} — Adds pose data</li>
     *   <li>{@code HIGH} — Full telemetry (default)</li>
     *   <li>{@code MACHINE} — Machine-readable format</li>
     * </ul>
     *
     * <p><strong>Recommendation:</strong> Use {@code HIGH} during development,
     * {@code LOW} or {@code NONE} for competition to reduce overhead.
     *
     * <p><strong>Usage Example:</strong>
     * <pre>{@code
     * SwerveConfig config = new SwerveConfig()
     *     .withTelemetryVerbosity(TelemetryVerbosity.LOW);
     * }</pre>
     *
     * @param verbosity The desired telemetry verbosity level
     * @return This config for chaining
     */
    public SwerveConfig withTelemetryVerbosity(TelemetryVerbosity verbosity) {
        this.telemetryVerbosity = verbosity;
        return this;
    }

    public SwerveConfig withManualOdometry(boolean enabled) {
        this.useManualOdometry = enabled;
        return this;
    }

    // ==================== GETTERS ====================

    /**
     * Gets the translation PID constants for PathPlanner.
     *
     * @return PIDConstants for translation control
     */
    public PIDConstants getTranslationPID() {
        return new PIDConstants(translationP, translationI, translationD);
    }

    /**
     * Gets the rotation PID constants for PathPlanner.
     *
     * @return PIDConstants for rotation control
     */
    public PIDConstants getRotationPID() {
        return new PIDConstants(rotationP, rotationI, rotationD);
    }

    /**
     * @return true if heading correction is enabled
     */
    public boolean isHeadingCorrectionEnabled() {
        return headingCorrectionEnabled;
    }

    /**
     * @return true if cosine compensation is enabled
     */
    public boolean isCosineCompensationEnabled() {
        return cosineCompensationEnabled;
    }

    /**
     * @return true if angular velocity compensation is enabled
     */
    public boolean isAngularVelocityCompensationEnabled() {
        return angularVelocityCompensationEnabled;
    }

    /**
     * @return Angular velocity compensation coefficient
     */
    public double getAngularVelocityCoeff() {
        return angularVelocityCoeff;
    }

    /**
     * @return true if feedforward is enabled for PathPlanner
     */
    public boolean isFeedforwardEnabled() {
        return feedforwardEnabled;
    }

    /**
     * @return true if encoder auto-sync is enabled
     */
    public boolean isEncoderAutoSyncEnabled() {
        return encoderAutoSyncEnabled;
    }

    /**
     * @return Encoder auto-sync deadband in degrees
     */
    public double getEncoderAutoSyncDeadbandDeg() {
        return encoderAutoSyncDeadbandDeg;
    }

    /**
     * @return true if manual odometry updates should be used (stops YAGSL odometry thread)
     */
    public boolean useManualOdometry() {
        return useManualOdometry;
    }

    /**
     * @return The configured telemetry verbosity level
     */
    public TelemetryVerbosity getTelemetryVerbosity() {
        return telemetryVerbosity;
    }
}
