// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.lib.targets;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Filesystem;

import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

/**
 * Loads GameTargetConfig from JSON files in the deploy directory.
 *
 * <p>JSON files should be placed in the deploy folder (src/main/deploy/)
 * and can be loaded by filename.
 *
 * <p><strong>Expected JSON Format:</strong>
 * <pre>{@code
 * {
 *   "gameYear": 2026,
 *   "gameName": "REBUILT",
 *   "targets": [
 *     {
 *       "name": "tower-blue",
 *       "tagIds": [15, 16],
 *       "offset": { "x": -0.5, "y": 0.0, "rotation": 180 },
 *       "tolerance": { "position": 0.05, "rotation": 2.0 }
 *     },
 *     {
 *       "name": "tower-red",
 *       "tagIds": [31, 32],
 *       "offset": { "x": -0.5, "y": 0.0, "rotation": 0 }
 *     }
 *   ]
 * }
 * }</pre>
 *
 * <p><strong>Notes:</strong>
 * <ul>
 *   <li>Offset x/y are in meters</li>
 *   <li>Offset rotation is in degrees</li>
 *   <li>Tolerance is optional (defaults to 0.1m position, 5° rotation)</li>
 *   <li>tagIds array must have at least one ID</li>
 * </ul>
 *
 * <p><strong>Usage Example:</strong>
 * <pre>{@code
 * // Load from deploy folder
 * GameTargetConfig config = GameTargetLoader.load("gametargets-2026-rebuilt.json");
 *
 * // Load from absolute path
 * GameTargetConfig config = GameTargetLoader.loadFromPath("/path/to/config.json");
 * }</pre>
 *
 * @see GameTargetConfig
 * @see GameTarget
 */
public class GameTargetLoader {

    private static final ObjectMapper objectMapper = new ObjectMapper();

    private GameTargetLoader() {
        // Utility class, no instantiation
    }

    /**
     * Loads a GameTargetConfig from a JSON file in the deploy directory.
     *
     * @param filename JSON filename (e.g., "gametargets.json")
     * @return Loaded GameTargetConfig
     * @throws RuntimeException if file cannot be read or parsed
     */
    public static GameTargetConfig load(String filename) {
        File deployDir = Filesystem.getDeployDirectory();
        File file = new File(deployDir, filename);
        return loadFromPath(file.getAbsolutePath());
    }

    /**
     * Loads a GameTargetConfig from an absolute file path.
     *
     * @param path Absolute path to JSON file
     * @return Loaded GameTargetConfig
     * @throws RuntimeException if file cannot be read or parsed
     */
    public static GameTargetConfig loadFromPath(String path) {
        try {
            File file = new File(path);
            JsonNode root = objectMapper.readTree(file);
            return parseConfig(root);
        } catch (IOException e) {
            throw new RuntimeException("Failed to load game targets from: " + path, e);
        }
    }

    /**
     * Loads a GameTargetConfig from a JSON string.
     *
     * @param json JSON string
     * @return Loaded GameTargetConfig
     * @throws RuntimeException if JSON cannot be parsed
     */
    public static GameTargetConfig loadFromString(String json) {
        try {
            JsonNode root = objectMapper.readTree(json);
            return parseConfig(root);
        } catch (IOException e) {
            throw new RuntimeException("Failed to parse game targets JSON", e);
        }
    }

    private static GameTargetConfig parseConfig(JsonNode root) {
        int gameYear = root.has("gameYear") ? root.get("gameYear").asInt() : 2026;
        String gameName = root.has("gameName") ? root.get("gameName").asText() : "Unknown";

        List<GameTarget> targets = new ArrayList<>();
        JsonNode targetsNode = root.get("targets");

        if (targetsNode != null && targetsNode.isArray()) {
            for (JsonNode targetNode : targetsNode) {
                targets.add(parseTarget(targetNode));
            }
        }

        return new GameTargetConfig(gameYear, gameName, targets);
    }

    private static GameTarget parseTarget(JsonNode node) {
        String name = node.get("name").asText();

        // Parse tag IDs
        JsonNode tagIdsNode = node.get("tagIds");
        int[] tagIds;
        if (tagIdsNode.isArray()) {
            tagIds = new int[tagIdsNode.size()];
            for (int i = 0; i < tagIdsNode.size(); i++) {
                tagIds[i] = tagIdsNode.get(i).asInt();
            }
        } else {
            tagIds = new int[]{tagIdsNode.asInt()};
        }

        // Parse offset
        Transform2d offset = new Transform2d();
        if (node.has("offset")) {
            JsonNode offsetNode = node.get("offset");
            double x = offsetNode.has("x") ? offsetNode.get("x").asDouble() : 0;
            double y = offsetNode.has("y") ? offsetNode.get("y").asDouble() : 0;
            double rotation = offsetNode.has("rotation") ? offsetNode.get("rotation").asDouble() : 0;
            offset = new Transform2d(
                new Translation2d(x, y),
                Rotation2d.fromDegrees(rotation)
            );
        }

        // Parse tolerance (optional)
        GameTarget.Tolerance tolerance = null;
        if (node.has("tolerance")) {
            JsonNode tolNode = node.get("tolerance");
            double position = tolNode.has("position") ? tolNode.get("position").asDouble() : 0.1;
            double rotation = tolNode.has("rotation") ? tolNode.get("rotation").asDouble() : 5.0;
            tolerance = new GameTarget.Tolerance(position, rotation);
        }

        return new GameTarget(name, tagIds, offset, tolerance);
    }
}
