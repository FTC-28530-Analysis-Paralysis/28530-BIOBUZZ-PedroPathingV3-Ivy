package org.firstinspires.ftc.teamcode.paths;

import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

/**
 * Interface for standalone visualizer path exports.
 * Allows autonomous routines to dynamically load starting pose and path segments.
 */
public interface PathContainer {

    /**
     * @return The starting pose for the robot on the field.
     */
    Pose getStartPose();

    /**
     * Returns a named path segment by identifier or enum.
     */
    Path getPath(String pathName);
}
