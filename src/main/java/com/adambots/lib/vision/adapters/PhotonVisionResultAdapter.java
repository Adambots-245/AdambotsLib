// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.lib.vision.adapters;

import java.util.List;
import java.util.stream.Collectors;

import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.adambots.lib.vision.VisionResult;
import com.adambots.lib.vision.VisionTarget;

/**
 * Adapter that wraps a PhotonVision {@link PhotonPipelineResult} to implement
 * the {@link VisionResult} interface.
 *
 * <p>This adapter enables the abstraction layer to work with PhotonVision results
 * without exposing PhotonVision-specific types to consumers of the vision system.
 *
 * @see VisionResult
 * @see PhotonPipelineResult
 */
public class PhotonVisionResultAdapter implements VisionResult {

    private final PhotonPipelineResult result;
    private List<PhotonVisionTargetAdapter> adaptedTargets;

    /**
     * Creates a new adapter wrapping the given PhotonPipelineResult.
     *
     * @param result The PhotonPipelineResult to wrap
     */
    public PhotonVisionResultAdapter(PhotonPipelineResult result) {
        this.result = result;
    }

    /**
     * Gets the underlying PhotonPipelineResult.
     *
     * <p>Use this when you need access to PhotonVision-specific features
     * not exposed through the VisionResult interface.
     *
     * @return The wrapped PhotonPipelineResult
     */
    public PhotonPipelineResult getPhotonResult() {
        return result;
    }

    @Override
    public boolean hasTargets() {
        return result.hasTargets();
    }

    @Override
    public VisionTarget getBestTarget() {
        if (!result.hasTargets()) {
            return null;
        }
        PhotonTrackedTarget best = result.getBestTarget();
        return best != null ? new PhotonVisionTargetAdapter(best) : null;
    }

    @Override
    public List<? extends VisionTarget> getTargets() {
        if (adaptedTargets == null) {
            adaptedTargets = result.getTargets().stream()
                .map(PhotonVisionTargetAdapter::new)
                .collect(Collectors.toList());
        }
        return adaptedTargets;
    }

    @Override
    public double getTimestampSeconds() {
        return result.getTimestampSeconds();
    }
}
