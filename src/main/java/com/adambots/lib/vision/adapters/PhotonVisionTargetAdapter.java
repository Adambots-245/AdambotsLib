// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.lib.vision.adapters;

import org.photonvision.targeting.PhotonTrackedTarget;

import com.adambots.lib.vision.VisionTarget;

import edu.wpi.first.math.geometry.Transform3d;

/**
 * Adapter that wraps a PhotonVision {@link PhotonTrackedTarget} to implement
 * the {@link VisionTarget} interface.
 *
 * <p>This adapter enables the abstraction layer to work with PhotonVision targets
 * without exposing PhotonVision-specific types to consumers of the vision system.
 *
 * @see VisionTarget
 * @see PhotonTrackedTarget
 */
public class PhotonVisionTargetAdapter implements VisionTarget {

    private final PhotonTrackedTarget target;

    /**
     * Creates a new adapter wrapping the given PhotonTrackedTarget.
     *
     * @param target The PhotonTrackedTarget to wrap
     */
    public PhotonVisionTargetAdapter(PhotonTrackedTarget target) {
        this.target = target;
    }

    /**
     * Gets the underlying PhotonTrackedTarget.
     *
     * <p>Use this when you need access to PhotonVision-specific features
     * not exposed through the VisionTarget interface.
     *
     * @return The wrapped PhotonTrackedTarget
     */
    public PhotonTrackedTarget getPhotonTarget() {
        return target;
    }

    @Override
    public int getFiducialId() {
        return target.getFiducialId();
    }

    @Override
    public double getYaw() {
        return target.getYaw();
    }

    @Override
    public double getPitch() {
        return target.getPitch();
    }

    @Override
    public double getArea() {
        return target.getArea();
    }

    @Override
    public double getPoseAmbiguity() {
        return target.getPoseAmbiguity();
    }

    @Override
    public Transform3d getBestCameraToTarget() {
        return target.getBestCameraToTarget();
    }
}
