package org.firstinspires.ftc.teamcode.routines;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import static com.pedropathing.ivy.commands.Commands.*;
import static com.pedropathing.ivy.groups.Groups.*;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.paths.HiveDownTeamplayPath;
import org.firstinspires.ftc.teamcode.subsystems.RobotContainer;

public class HiveDownTeamplayRoutine implements AutoRoutine {

    private final HiveDownTeamplayPath path = new HiveDownTeamplayPath();

    @Override
    public String getName() {
        return "Hive Down Teamplay";
    }

    @Override
    public Pose getStartPose() {
        return path.getStartPose();
    }

    @Override
    public Command buildCommand(Follower follower, RobotContainer robot, boolean visionEnabled, double startDelaySec) {
        Command delayCommand = startDelaySec > 0 ? waitMs(startDelaySec * 1000.0) : instant(() -> {});

        return sequential(
                delayCommand,
                follow(follower, path.scoreC6()),
                follow(follower, path.scoreToFlowerBC6()),
                follow(follower, path.flowerBC6ToScore()),
                follow(follower, path.sweepUpBalls1()),
                follow(follower, path.sweepUpBalls2()),
                follow(follower, path.sweepTowardPark()),
                follow(follower, path.parkA56())
        );
    }
}
