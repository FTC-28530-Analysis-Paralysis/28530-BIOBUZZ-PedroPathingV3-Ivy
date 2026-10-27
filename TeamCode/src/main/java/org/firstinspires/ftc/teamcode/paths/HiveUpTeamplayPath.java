package org.firstinspires.ftc.teamcode.paths;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

/**
 * Visualizer exported path class implementing PathContainer.
 * Pure path definitions separated from OpMode execution logic.
 */
public class HiveUpTeamplayPath implements PathContainer {

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(58, 8.5, 90);
    private final Pose score = poseFactory.of(58, 14, 90);
    private final Pose gardenApproach = poseFactory.of(24, 9.5, 180);
    private final Pose scoreToGardenApproachControl1 = poseFactory.of(57, 21, 0);
    private final Pose scoreToGardenApproachControl2 = poseFactory.of(37, 9.5, 0);
    private final Pose gardenPickup = poseFactory.of(9, 9, 180);
    private final Pose gardenToScoreControl1 = poseFactory.of(20.0, 15.5, 0);
    private final Pose flower = poseFactory.of(12, 42, 180);
    private final Pose scoreToFlowerControl1 = poseFactory.of(58, 30.0, 0);
    private final Pose scoreToFlowerControl2 = poseFactory.of(25, 42.0, 0);
    private final Pose flowerToScoreControl1 = poseFactory.of(25.5, 42, 0);
    private final Pose flowerToScoreControl2 = poseFactory.of(57, 30, 0);
    private final Pose park = poseFactory.of(10, 90, 90);
    private final Pose scoreToParkControl1 = poseFactory.of(58, 28, 0);
    private final Pose scoreToParkControl2 = poseFactory.of(10, 40, 0);

    public Path startToScore() {
        return Paths.line(start, score).linear(start, score);
    }

    public Path scoreToGardenApproach() {
        return Paths.curve(score, scoreToGardenApproachControl1, scoreToGardenApproachControl2, gardenApproach).tangent();
    }

    public Path gardenPickup() {
        return Paths.line(gardenApproach, gardenPickup).constant(gardenPickup);
    }

    public Path gardenToScore() {
        return Paths.curve(gardenPickup, gardenToScoreControl1, score).linear(gardenPickup, score);
    }

    public Path scoreToFlower() {
        return Paths.curve(score, scoreToFlowerControl1, scoreToFlowerControl2, flower).tangent();
    }

    public Path flowerToScore() {
        return Paths.curve(flower, flowerToScoreControl1, flowerToScoreControl2, score).reverseTangent();
    }

    public Path scoreToPark() {
        return Paths.curve(score, scoreToParkControl1, scoreToParkControl2, park).tangent();
    }

    @Override
    public Path getPath(String pathName) {
        switch (pathName) {
            case "score": return startToScore();
            case "gardenApproach": return scoreToGardenApproach();
            case "gardenPickup": return gardenPickup();
            case "gardenToScore": return gardenToScore();
            case "flower": return scoreToFlower();
            case "flowerToScore": return flowerToScore();
            case "scoreToPark": return scoreToPark();
            default: return null;
        }
    }

    @Override
    public Pose getStartPose() {
        return start;
    }
}
