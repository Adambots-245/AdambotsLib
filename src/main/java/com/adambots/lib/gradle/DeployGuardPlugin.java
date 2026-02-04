package com.adambots.lib.gradle;

import org.gradle.api.GradleException;
import org.gradle.api.Plugin;
import org.gradle.api.Project;
import org.gradle.api.Task;

import java.io.BufferedReader;
import java.io.InputStreamReader;

/**
 * Gradle plugin that prevents deploying robot code from unauthorized git branches.
 *
 * <p>This plugin automatically hooks into GradleRIO's deploy task and validates
 * that the current git branch is authorized before deployment proceeds.
 *
 * <p><strong>Allowed branches:</strong>
 * <ul>
 *   <li>{@code main} - exact match</li>
 *   <li>{@code comp/*} - any branch starting with "comp/" (e.g., comp/kettering-week-2)</li>
 * </ul>
 *
 * <p><strong>Usage:</strong>
 * <pre>{@code
 * plugins {
 *     id "edu.wpi.first.GradleRIO" version "2026.x.x"
 *     id "com.adambots.deploy-guard"
 * }
 * }</pre>
 *
 * @since 2026.2.2
 */
public class DeployGuardPlugin implements Plugin<Project> {

    /** Branch name allowed for deployment (exact match). */
    private static final String ALLOWED_BRANCH_EXACT = "main";

    /** Branch prefix allowed for deployment (e.g., comp/kettering-week-2). */
    private static final String ALLOWED_BRANCH_PREFIX = "comp/";

    @Override
    public void apply(Project project) {
        // Wait until project is fully evaluated to find the deploy task
        project.afterEvaluate(proj -> {
            // Find all deploy-related tasks from GradleRIO
            proj.getTasks().matching(task ->
                task.getName().equals("deploy") ||
                task.getName().startsWith("deploy")
            ).all(task -> {
                // Add our validation as the first action
                task.doFirst(t -> validateBranch(proj));
            });

            // Also hook into the main deploy task if it exists
            Task deployTask = proj.getTasks().findByName("deploy");
            if (deployTask != null) {
                proj.getLogger().info("DeployGuard: Hooked into deploy task");
            } else {
                proj.getLogger().warn("DeployGuard: No deploy task found. " +
                        "Make sure GradleRIO plugin is applied before deploy-guard.");
            }
        });
    }

    /**
     * Validates that the current git branch is allowed for deployment.
     *
     * @param project The Gradle project
     * @throws GradleException if the branch is not allowed
     */
    private void validateBranch(Project project) {
        // Check for hidden override (mentor emergency use only)
        String override = System.getenv("ADAMBOTS_DEPLOY_OVERRIDE");
        if ("true".equalsIgnoreCase(override)) {
            project.getLogger().warn("DeployGuard: Override enabled - skipping branch check");
            return;
        }

        String currentBranch = getCurrentBranch(project);

        if (currentBranch == null) {
            // Not in a git repository or git not available - allow deployment
            project.getLogger().warn("DeployGuard: Could not determine git branch - allowing deployment");
            return;
        }

        if (isBranchAllowed(currentBranch)) {
            // Branch is allowed
            printAllowedMessage(project, currentBranch);
            return;
        }

        // Branch is not allowed - block deployment
        printBlockedMessage(project);
        throw new GradleException("Deploy blocked: Current branch is not authorized for deployment.");
    }

    /**
     * Checks if the given branch is allowed for deployment.
     *
     * @param branch The branch name to check
     * @return true if the branch is allowed, false otherwise
     */
    private boolean isBranchAllowed(String branch) {
        // Exact match for 'main'
        if (ALLOWED_BRANCH_EXACT.equals(branch)) {
            return true;
        }
        // Prefix match for 'comp/'
        if (branch.startsWith(ALLOWED_BRANCH_PREFIX)) {
            return true;
        }
        return false;
    }

    /**
     * Gets the current git branch name.
     *
     * @param project The Gradle project
     * @return The branch name, or null if not in a git repo
     */
    private String getCurrentBranch(Project project) {
        try {
            ProcessBuilder pb = new ProcessBuilder("git", "rev-parse", "--abbrev-ref", "HEAD");
            pb.directory(project.getProjectDir());
            pb.redirectErrorStream(true);

            Process process = pb.start();
            try (BufferedReader reader = new BufferedReader(new InputStreamReader(process.getInputStream()))) {
                String branch = reader.readLine();
                int exitCode = process.waitFor();

                if (exitCode == 0 && branch != null && !branch.isEmpty()) {
                    return branch.trim();
                }
            }
        } catch (Exception e) {
            project.getLogger().debug("DeployGuard: Failed to get git branch: " + e.getMessage());
        }
        return null;
    }

    /**
     * Prints a confirmation message when deployment is allowed.
     */
    private void printAllowedMessage(Project project, String branch) {
        String message = String.format(
                "\n" +
                "  ============================================\n" +
                "  DeployGuard: Branch '%s' is authorized\n" +
                "  Proceeding with deployment...\n" +
                "  ============================================\n",
                branch
        );
        project.getLogger().lifecycle(message);
    }

    /**
     * Prints a prominent error message when deployment is blocked.
     */
    private void printBlockedMessage(Project project) {
        String message =
                "\n" +
                "  ╔══════════════════════════════════════════════════════════════════╗\n" +
                "  ║                                                                  ║\n" +
                "  ║   ██████╗ ███████╗██████╗ ██╗      ██████╗ ██╗   ██╗             ║\n" +
                "  ║   ██╔══██╗██╔════╝██╔══██╗██║     ██╔═══██╗╚██╗ ██╔╝             ║\n" +
                "  ║   ██║  ██║█████╗  ██████╔╝██║     ██║   ██║ ╚████╔╝              ║\n" +
                "  ║   ██║  ██║██╔══╝  ██╔═══╝ ██║     ██║   ██║  ╚██╔╝               ║\n" +
                "  ║   ██████╔╝███████╗██║     ███████╗╚██████╔╝   ██║                ║\n" +
                "  ║   ╚═════╝ ╚══════╝╚═╝     ╚══════╝ ╚═════╝    ╚═╝                ║\n" +
                "  ║                                                                  ║\n" +
                "  ║   ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗██████╗       ║\n" +
                "  ║   ██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝██╔══██╗      ║\n" +
                "  ║   ██████╔╝██║     ██║   ██║██║     █████╔╝ █████╗  ██║  ██║      ║\n" +
                "  ║   ██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ██╔══╝  ██║  ██║      ║\n" +
                "  ║   ██████╔╝███████╗╚██████╔╝╚██████╗██║  ██╗███████╗██████╔╝      ║\n" +
                "  ║   ╚═════╝ ╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝╚═════╝       ║\n" +
                "  ║                                                                  ║\n" +
                "  ╠══════════════════════════════════════════════════════════════════╣\n" +
                "  ║                                                                  ║\n" +
                "  ║   You cannot deploy from this branch!                            ║\n" +
                "  ║                                                                  ║\n" +
                "  ║   To deploy robot code:                                          ║\n" +
                "  ║     1. Commit your changes                                       ║\n" +
                "  ║     2. Push to GitHub                                            ║\n" +
                "  ║     3. Create a Pull Request to 'main'                           ║\n" +
                "  ║     4. Get your code reviewed and merged                         ║\n" +
                "  ║     5. Switch to 'main' and pull latest                          ║\n" +
                "  ║     6. Run deploy again                                          ║\n" +
                "  ║                                                                  ║\n" +
                "  ╚══════════════════════════════════════════════════════════════════╝\n";
        project.getLogger().error(message);
    }
}
