package org.firstinspires.ftc.teamcode.paths;

import static com.pedropathing.api.Paths.*;
import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

/**
 * Visualizer exported path class implementing PathContainer.
 * Pure path definitions separated from OpMode execution logic.
 */
public class AutoPath1 implements PathContainer {

    private final PoseFactory poseFactory = PoseFactory.degrees();

    public final Pose start = poseFactory.of(57, 8.5, 90);
    public final Pose startToScore = poseFactory.of(57, 14, 90);
    public final Pose scoreToGardenApproach = poseFactory.of(24, 9.5, -179.4926);
    public final Pose scoreToGardenApproachControl1 = poseFactory.of(57, 21, 0);
    public final Pose scoreToGardenApproachControl2 = poseFactory.of(36.8, 9.5, 0);
    public final Pose gardenPickup = poseFactory.of(9, 9, 180);
    public final Pose gardenToScoreStart = poseFactory.of(9, 9, 180);
    public final Pose gardenToScore = poseFactory.of(57, 14, 90);
    public final Pose gardenToScoreControl1 = poseFactory.of(19.9708, 15.4878, 0);
    public final Pose scoreToGarden = poseFactory.of(11.8275, 42, 179.4941);
    public final Pose scoreToGardenControl1 = poseFactory.of(57, 30, 0);
    public final Pose scoreToGardenControl2 = poseFactory.of(25.2052, 42, 0);
    public final Pose flowerToScore = poseFactory.of(57, 14, 91.1173);
    public final Pose flowerToScoreControl1 = poseFactory.of(25.4883, 42.2115, 0);
    public final Pose flowerToScoreControl2 = poseFactory.of(57.0049, 29.8635, 0);
    public final Pose scoreToPark = poseFactory.of(10, 90, 90.5391);
    public final Pose scoreToParkControl1 = poseFactory.of(57, 28, 0);
    public final Pose scoreToParkControl2 = poseFactory.of(10, 40, 0);

    @Override
    public Pose getStartPose() {
        return start;
    }

    public Path startToScore() {
        return Paths.line(start, startToScore).linear(start, startToScore);
    }

    public Path scoreToGardenApproach() {
        return Paths.curve(startToScore, scoreToGardenApproachControl1, scoreToGardenApproachControl2, scoreToGardenApproach).tangent();
    }

    public Path gardenPickup() {
        return Paths.line(scoreToGardenApproach, gardenPickup).constant(gardenPickup);
    }

    public Path gardenToScore() {
        return Paths.curve(gardenToScoreStart, gardenToScoreControl1, gardenToScore).linear(gardenToScoreStart, gardenToScore);
    }

    public Path scoreToGarden() {
        return Paths.curve(gardenToScore, scoreToGardenControl1, scoreToGardenControl2, scoreToGarden).tangent();
    }

    public Path flowerToScore() {
        return Paths.curve(scoreToGarden, flowerToScoreControl1, flowerToScoreControl2, flowerToScore).reverseTangent();
    }

    public Path scoreToPark() {
        return Paths.curve(flowerToScore, scoreToParkControl1, scoreToParkControl2, scoreToPark).tangent();
    }

    @Override
    public Path getPath(String pathName) {
        switch (pathName) {
            case "startToScore": return startToScore();
            case "scoreToGardenApproach": return scoreToGardenApproach();
            case "gardenPickup": return gardenPickup();
            case "gardenToScore": return gardenToScore();
            case "scoreToGarden": return scoreToGarden();
            case "flowerToScore": return flowerToScore();
            case "scoreToPark": return scoreToPark();
            default: return null;
        }
    }
}
