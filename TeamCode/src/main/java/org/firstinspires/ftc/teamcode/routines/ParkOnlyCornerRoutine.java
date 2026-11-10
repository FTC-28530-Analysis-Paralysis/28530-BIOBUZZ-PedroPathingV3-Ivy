package org.firstinspires.ftc.teamcode.routines;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import static com.pedropathing.ivy.commands.Commands.*;
import static com.pedropathing.ivy.groups.Groups.*;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.config.GameState;
import org.firstinspires.ftc.teamcode.paths.ParkOnlyCornerPath;
import org.firstinspires.ftc.teamcode.subsystems.RobotContainer;

public class ParkOnlyCornerRoutine implements AutoRoutine {

    @Override
    public String getName() {
        return "Park Only (" + GameState.alliance.getName() + ")";
    }

    @Override
    public Pose getStartPose() {
        return new ParkOnlyCornerPath(GameState.alliance).getStartPose();
    }

    @Override
    public Command buildCommand(Follower follower, RobotContainer robot, boolean visionEnabled, double startDelaySec) {
        ParkOnlyCornerPath path = new ParkOnlyCornerPath(GameState.alliance);
        Command delayCommand = startDelaySec > 0 ? waitMs(startDelaySec * 1000.0) : instant(() -> {});

        return sequential(
                delayCommand,
                follow(follower, path.startToPark())
        );
    }
}
