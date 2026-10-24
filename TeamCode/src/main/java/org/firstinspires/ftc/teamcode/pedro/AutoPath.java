package org.firstinspires.ftc.teamcode.pedro;

import static com.pedropathing.api.Paths.*;
import com.pedropathing.api.Paths;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.*;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "AutoPath", group = "Autonomous")
public class AutoPath extends LinearOpMode {

    private Follower follower;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(57, 8.5, 90);
    private final Pose startToScore = poseFactory.of(57, 14, 90);
    private final Pose scoreToGardenApproach = poseFactory.of(24, 9.5, -179.4926);
    private final Pose scoreToGardenApproachControl1 = poseFactory.of(57, 21, 0);
    private final Pose scoreToGardenApproachControl2 = poseFactory.of(36.8, 9.5, 0);
    private final Pose gardenPickup = poseFactory.of(9, 9, 180);
    private final Pose gardenToScoreStart = poseFactory.of(9, 9, 180);
    private final Pose gardenToScore = poseFactory.of(57, 14, 90);
    private final Pose gardenToScoreControl1 = poseFactory.of(19.9708, 15.4878, 0);
    private final Pose scoreToGarden = poseFactory.of(11.8275, 42, 179.4941);
    private final Pose scoreToGardenControl1 = poseFactory.of(57, 30, 0);
    private final Pose scoreToGardenControl2 = poseFactory.of(25.2052, 42, 0);
    private final Pose flowerToScore = poseFactory.of(57, 14, 91.1173);
    private final Pose flowerToScoreControl1 = poseFactory.of(25.4883, 42.2115, 0);
    private final Pose flowerToScoreControl2 = poseFactory.of(57.0049, 29.8635, 0);
    private final Pose scoreToPark = poseFactory.of(10, 90, 90.5391);
    private final Pose scoreToParkControl1 = poseFactory.of(57, 28, 0);
    private final Pose scoreToParkControl2 = poseFactory.of(10, 40, 0);

    // Autonomous routine
    public Command autoRoutine() {
        return sequential(
                follow(follower, startToScore()),
                follow(follower, scoreToGardenApproach()),
                follow(follower, gardenPickup()),
                follow(follower, gardenToScore()),
                follow(follower, scoreToGarden()),
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

    public Path scoreToGarden() {
        return Paths.curve(gardenToScore, scoreToGardenControl1, scoreToGardenControl2, scoreToGarden).tangent();
    }

    public Path flowerToScore() {
        return Paths.curve(scoreToGarden, flowerToScoreControl1, flowerToScoreControl2, flowerToScore).reverseTangent();
    }

    public Path scoreToPark() {
        return Paths.curve(flowerToScore, scoreToParkControl1, scoreToParkControl2, scoreToPark).tangent();
    }
}
