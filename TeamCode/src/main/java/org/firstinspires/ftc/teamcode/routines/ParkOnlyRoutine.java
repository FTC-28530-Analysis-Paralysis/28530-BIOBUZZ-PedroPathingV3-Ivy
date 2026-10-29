package org.firstinspires.ftc.teamcode.routines;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import static com.pedropathing.ivy.commands.Commands.*;
import static com.pedropathing.ivy.groups.Groups.*;
import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.config.FieldLocations;
import org.firstinspires.ftc.teamcode.subsystems.RobotContainer;

public class ParkOnlyRoutine implements AutoRoutine {

    @Override
    public String getName() {
        return "Park Only";
    }

    @Override
    public Pose getStartPose() {
        return FieldLocations.START_C1;
    }

    @Override
    public Command buildCommand(Follower follower, RobotContainer robot, boolean visionEnabled, double startDelaySec) {
        Command delayCommand = startDelaySec > 0 ? waitMs(startDelaySec * 1000.0) : instant(() -> {});

        return sequential(
                delayCommand,
                instant(() -> {})
        );
    }
}
