// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.lib.vision.adapters;

import java.util.Optional;

import com.adambots.lib.vision.VisionCamera;
import com.adambots.lib.vision.VisionCameraInterface;
import com.adambots.lib.vision.VisionResult;

/**
 * Adapter that wraps a {@link VisionCamera} to implement the {@link VisionCameraInterface}.
 *
 * <p>Note: This adapter is provided for completeness, but since {@link VisionCamera}
 * directly implements {@link VisionCameraInterface}, you typically don't need to use
 * this adapter. It's useful if you have a VisionCamera and need to ensure all results
 * are returned as the abstract types.
 *
 * @see VisionCameraInterface
 * @see VisionCamera
 */
public class PhotonVisionCameraAdapter implements VisionCameraInterface {

    private final VisionCamera camera;

    /**
     * Creates a new adapter wrapping the given VisionCamera.
     *
     * @param camera The VisionCamera to wrap
     */
    public PhotonVisionCameraAdapter(VisionCamera camera) {
        this.camera = camera;
    }

    /**
     * Gets the underlying VisionCamera.
     *
     * @return The wrapped VisionCamera
     */
    public VisionCamera getVisionCamera() {
        return camera;
    }

    @Override
    public String getName() {
        return camera.getName();
    }

    @Override
    public boolean isEnabled() {
        return camera.isEnabled();
    }

    @Override
    public void enable() {
        camera.enable();
    }

    @Override
    public void disable() {
        camera.disable();
    }

    @Override
    public boolean hasTarget() {
        return camera.hasTarget();
    }

    @Override
    public Optional<? extends VisionResult> getBestResult() {
        // VisionCamera already implements VisionCameraInterface and returns adapted results
        return camera.getBestResult();
    }

    @Override
    public Optional<? extends VisionResult> getLatestResult() {
        // VisionCamera already implements VisionCameraInterface and returns adapted results
        return camera.getLatestResult();
    }
}
