package org.firstinspires.ftc.teamcode.paths;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

/**
 * Visualizer exported path class implementing PathContainer.
 * Pure path definitions separated from OpMode execution logic.
 */
public class HiveDownTeamplayPath implements PathContainer {

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(59, 133, 270);
    private final Pose scoreC6 = poseFactory.of(59, 127, 270);
    private final Pose flowerBC6 = poseFactory.of(49, 127, 141.6198);
    private final Pose scoreToFlowerBC6Control1 = poseFactory.of(59, 123, 0);
    private final Pose scoreToFlowerBC6Control2 = poseFactory.of(54, 123, 0);
    private final Pose flowerBC6ToScore = poseFactory.of(58, 125, 270);
    private final Pose sweepUpBalls1 = poseFactory.of(27, 108, 59);
    private final Pose sweepUpBalls1Control1 = poseFactory.of(77, 117, 0);
    private final Pose sweepUpBalls1Control2 = poseFactory.of(20, 97, 0);
    private final Pose sweepUpBalls2 = poseFactory.of(48, 124, 180);
    private final Pose sweepUpBalls2Control1 = poseFactory.of(29.5, 113, 0);
    private final Pose sweepUpBalls2Control2 = poseFactory.of(68, 124.5, 0);
    private final Pose sweepTowardPark = poseFactory.of(18, 124, 180);
    private final Pose parkA56 = poseFactory.of(11, 124, 0);

    public Path scoreC6() {
        return Paths.line(start, scoreC6).linear(start, scoreC6);
    }

    public Path scoreToFlowerBC6() {
        return Paths.curve(scoreC6, scoreToFlowerBC6Control1, scoreToFlowerBC6Control2, flowerBC6).tangent();
    }

    public Path flowerBC6ToScore() {
        return Paths.line(flowerBC6, flowerBC6ToScore).linear(flowerBC6, flowerBC6ToScore);
    }

    public Path sweepUpBalls1() {
        return Paths.curve(flowerBC6ToScore, sweepUpBalls1Control1, sweepUpBalls1Control2, sweepUpBalls1).tangent();
    }

    public Path sweepUpBalls2() {
        return Paths.curve(sweepUpBalls1, sweepUpBalls2Control1, sweepUpBalls2Control2, sweepUpBalls2).tangent();
    }

    public Path sweepTowardPark() {
        return Paths.line(sweepUpBalls2, sweepTowardPark).tangent();
    }

    public Path parkA56() {
        return Paths.line(sweepTowardPark, parkA56).reverseTangent();
    }

    @Override
    public Path getPath(String pathName) {
        switch (pathName) {
            case "scoreC6": return scoreC6();
            case "flowerBC6": return scoreToFlowerBC6();
            case "flowerBC6ToScore": return flowerBC6ToScore();
            case "sweepUpBalls1": return sweepUpBalls1();
            case "sweepUpBalls2": return sweepUpBalls2();
            case "sweepTowardPark": return sweepTowardPark();
            case "parkA56": return parkA56();
            default: return null;
        }
    }

    @Override
    public Pose getStartPose() {
        return start;
    }
}
