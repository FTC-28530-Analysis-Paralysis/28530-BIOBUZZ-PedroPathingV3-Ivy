package org.firstinspires.ftc.teamcode.opmodes;

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
import org.firstinspires.ftc.teamcode.pedro.Constants;

@Autonomous(name = "PP C1 Hive Up AutoPath", group = "Autonomous")
public class PP_C1_HiveUp_AutoPath extends LinearOpMode {

    private Follower follower;

    private final Pose start = FieldLocations.START_C1;
    private final Pose score = FieldLocations.SCORE_C1;
    private final Pose scoreToGardenApproach = FieldLocations.RED_GARDEN_APPROACH;
    private final Pose scoreToGardenApproachControl1 = FieldLocations.SCORE_TO_RED_GARDEN_CONTROL_1;
    private final Pose scoreToGardenApproachControl2 = FieldLocations.SCORE_TO_RED_GARDEN_CONTROL_2;
    private final Pose gardenPickup = FieldLocations.RED_GARDEN_PICKUP;
    private final Pose gardenToScoreControl1 = FieldLocations.RED_GARDEN_TO_SCORE_CONTROL_1;
    private final Pose flower = FieldLocations.FLOWER_A2_3;
    private final Pose scoreToFlowerControl1 = FieldLocations.SCORE_TO_FLOWER_A23_CONTROL_1;
    private final Pose scoreToFlowerControl2 = FieldLocations.SCORE_TO_FLOWER_A23_CONTROL_2;
    private final Pose flowerToScoreControl1 = FieldLocations.FLOWER_A23_TO_SCORE_CONTROL_1;
    private final Pose flowerToScoreControl2 = FieldLocations.FLOWER_A23_TO_SCORE_CONTROL_2;
    private final Pose scoreToPark = FieldLocations.RED_PARK_A4;
    private final Pose scoreToParkControl1 = FieldLocations.SCORE_C1_TO_RED_PARK_CONTROL_1;
    private final Pose scoreToParkControl2 = FieldLocations.SCORE_C1_TO_RED_PARK_CONTROL_2;

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
                telemetry.addData("Path name", follower.currentPath().toString());
            }

            telemetry.update();
        }
    }

    public Path startToScore() {
        return Paths.line(start, score).linear(start, score);
    }

    public Path scoreToGardenApproach() {
        return Paths.curve(score, scoreToGardenApproachControl1, scoreToGardenApproachControl2, scoreToGardenApproach).tangent();
    }

    public Path gardenPickup() {
        return Paths.line(scoreToGardenApproach, gardenPickup).constant(gardenPickup);
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
        return Paths.curve(score, scoreToParkControl1, scoreToParkControl2, scoreToPark).tangent();
    }
}
