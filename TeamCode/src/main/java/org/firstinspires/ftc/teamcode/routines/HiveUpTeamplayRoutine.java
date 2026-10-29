package org.firstinspires.ftc.teamcode.routines;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import static com.pedropathing.ivy.commands.Commands.*;
import static com.pedropathing.ivy.groups.Groups.*;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.paths.HiveUpTeamplayPath;
import org.firstinspires.ftc.teamcode.subsystems.RobotContainer;

public class HiveUpTeamplayRoutine implements AutoRoutine {

    private final HiveUpTeamplayPath path = new HiveUpTeamplayPath();

    @Override
    public String getName() {
        return "Hive Up Teamplay";
    }

    @Override
    public Pose getStartPose() {
        return path.getStartPose();
    }

    @Override
    public Command buildCommand(Follower follower, RobotContainer robot, boolean visionEnabled, double startDelaySec) {
        Command delayCommand = startDelaySec > 0 ? waitMs(startDelaySec * 1000.0) : instant(() -> {});

        if (visionEnabled) {
            // Reactive Routine using Limelight triggers & fallbacks
            return sequential(
                    delayCommand,
                    // 1. Prepare swivel & look for AprilTag on Hive
                    robot.limelight.prepareForHiveDetectionCommand(robot.swivel),

                    // 2. Drive to score position while spinning up flywheel launcher in parallel
                    parallel(
                            follow(follower, path.startToScore()),
                            robot.launcher.spinUpCommand()
                    ),

                    // 3. Check for hive tipped & launch
                    race(
                            robot.limelight.waitForHiveTippedCommand(),
                            waitMs(1500) // Timeout fallback
                    ),
                    robot.launcher.launchSequenceCommand(robot.transfer, 1.0),

                    // 4. Drive to garden approach while preparing Limelight for color blob detection
                    parallel(
                            follow(follower, path.scoreToGardenApproach()),
                            robot.limelight.prepareForBlobDetectionCommand(robot.swivel)
                    ),

                    // 5. Race approach to garden against Limelight detecting pollen/nectar
                    race(
                            follow(follower, path.gardenPickup()),
                            robot.limelight.waitForBallDetectedCommand()
                    ),

                    // 6. Branch depending on whether ball was spotted
                    conditional(
                            robot.limelight::seesBallBlob,
                            sequential(
                                    robot.intake.startCommand(),
                                    waitMs(1500),
                                    robot.intake.stopCommand()
                            ),
                            follow(follower, path.gardenToScore())
                    ),

                    // 7. Complete score to park
                    follow(follower, path.scoreToPark())
            );
        } else {
            // Pure Pathing Routine (Direct Visualizer Path Execution)
            return sequential(
                    delayCommand,
                    follow(follower, path.startToScore()),
                    follow(follower, path.scoreToGardenApproach()),
                    follow(follower, path.gardenPickup()),
                    follow(follower, path.gardenToScore()),
                    follow(follower, path.scoreToFlower()),
                    follow(follower, path.flowerToScore()),
                    follow(follower, path.scoreToPark())
            );
        }
    }
}
