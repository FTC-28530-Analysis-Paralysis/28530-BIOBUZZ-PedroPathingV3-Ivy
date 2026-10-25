package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.api.Paths;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.config.FieldLocations;

@Autonomous(name = "PP C1 Hive Up AutoPath", group = "Autonomous")
public class PP_C1_HiveUp_AutoPath extends LinearOpMode {

    private Follower follower;

    private final Pose start = FieldLocations.START;
    private final Pose startToScore = FieldLocations.SCORE;
    private final Pose scoreToGardenApproach = FieldLocations.GARDEN_APPROACH;
    private final Pose scoreToGardenApproachControl1 = FieldLocations.SCORE_TO_GARDEN_CONTROL_1;
    private final Pose scoreToGardenApproachControl2 = FieldLocations.SCORE_TO_GARDEN_CONTROL_2;
    private final Pose gardenPickup = FieldLocations.GARDEN_PICKUP;
    private final Pose gardenToScoreStart = FieldLocations.GARDEN_TO_SCORE_START;
    private final Pose gardenToScore = FieldLocations.GARDEN_TO_SCORE;
    private final Pose gardenToScoreControl1 = FieldLocations.GARDEN_TO_SCORE_CONTROL_1;
    private final Pose scoreToFlower = FieldLocations.SCORE_TO_FLOWER;
    private final Pose scoreToFlowerControl1 = FieldLocations.SCORE_TO_FLOWER_CONTROL_1;
    private final Pose scoreToFlowerControl2 = FieldLocations.SCORE_TO_FLOWER_CONTROL_2;
    private final Pose flowerToScore = FieldLocations.FLOWER_TO_SCORE;
    private final Pose flowerToScoreControl1 = FieldLocations.FLOWER_TO_SCORE_CONTROL_1;
    private final Pose flowerToScoreControl2 = FieldLocations.FLOWER_TO_SCORE_CONTROL_2;
    private final Pose scoreToPark = FieldLocations.PARK;
    private final Pose scoreToParkControl1 = FieldLocations.SCORE_TO_PARK_CONTROL_1;
    private final Pose scoreToParkControl2 = FieldLocations.SCORE_TO_PARK_CONTROL_2;

    // Autonomous routine
    public Command autoRoutine() {
        return sequential(
                follow(follower, startToScore()),
                follow(follower, scoreToGardenApproach()),
                follow(follower, gardenPickup()),
                follow(follower, gardenToScore()),
                follow(follower, scoreToFlower()),
                follow(follower, flowerToScore()),
                follow(follower, scoreToPark())
        );
    }

    @Override
    public void runOpMode() {
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        follower.setPose(start);
        follower.update();

        waitForStart();
        schedule(autoRoutine());

        while (opModeIsActive()) {
            follower.update();
            Scheduler.execute();

            telemetry.addData("x", follower.pose().x());
            telemetry.addData("y", follower.pose().y());
            telemetry.addData("heading", follower.pose().heading());

            if (follower.currentPath() != null) {
                telemetry.addData("Current path distance remaining", follower.distanceToEndpoint());
                telemetry.addData("Path number", follower.pathIndex());
            }

            telemetry.update();
        }
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

    public Path scoreToFlower() {
        return Paths.curve(gardenToScore, scoreToFlowerControl1, scoreToFlowerControl2, scoreToFlower).tangent();
    }

    public Path flowerToScore() {
        return Paths.curve(scoreToFlower, flowerToScoreControl1, flowerToScoreControl2, flowerToScore).reverseTangent();
    }

    public Path scoreToPark() {
        return Paths.curve(flowerToScore, scoreToParkControl1, scoreToParkControl2, scoreToPark).tangent();
    }
}
