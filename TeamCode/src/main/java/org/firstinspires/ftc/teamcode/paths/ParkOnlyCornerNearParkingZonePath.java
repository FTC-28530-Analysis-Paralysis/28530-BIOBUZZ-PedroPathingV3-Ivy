package org.firstinspires.ftc.teamcode.paths;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

import org.firstinspires.ftc.teamcode.config.GameState;

/**
 * Dedicated visualizer export path class for Park Only routine.
 * Pure path definitions separated from OpMode execution logic.
 * Automatically handles alliance mirroring via PoseFactory.
 */
public class ParkOnlyCornerNearParkingZonePath implements PathContainer {

    private final PoseFactory poseFactory;

    private final Pose start;
    private final Pose park;

    public ParkOnlyCornerNearParkingZonePath() {
        this(GameState.alliance);
    }

    public ParkOnlyCornerNearParkingZonePath(GameState.Alliance alliance) {
        this.poseFactory = (alliance == GameState.Alliance.BLUE)
                ? PoseFactory.degrees().rotateAround(70.75, 70.75, 180)
                : PoseFactory.degrees();

        start = poseFactory.of(8.5, 133, 270);
        park = poseFactory.of(10, 120, 270);
    }

    public Path startToPark() {
        return Paths.line(start,park).constant(start);
    }

    @Override
    public Path getPath(String pathName) {
        if ("startToPark".equals(pathName) || "park".equals(pathName)) {
            return startToPark();
        }
        return null;
    }

    @Override
    public Pose getStartPose() {
        return start;
    }
}
