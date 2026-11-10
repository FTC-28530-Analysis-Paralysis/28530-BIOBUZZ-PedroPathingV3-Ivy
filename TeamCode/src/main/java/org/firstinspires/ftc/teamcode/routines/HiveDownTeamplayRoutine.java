package org.firstinspires.ftc.teamcode.routines;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import static com.pedropathing.ivy.commands.Commands.*;
import static com.pedropathing.ivy.groups.Groups.*;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.config.GameState;
import org.firstinspires.ftc.teamcode.paths.HiveDownTeamplayPath;
import org.firstinspires.ftc.teamcode.subsystems.RobotContainer;

public class HiveDownTeamplayRoutine implements AutoRoutine {

    @Override
    public String getName() {
        return "Hive Down Teamplay (" + GameState.alliance.getName() + ")";
    }

    @Override
    public Pose getStartPose() {
        return new HiveDownTeamplayPath(GameState.alliance).getStartPose();
    }

    @Override
    public Command buildCommand(Follower follower, RobotContainer robot, boolean visionEnabled, double startDelaySec) {
        HiveDownTeamplayPath path = new HiveDownTeamplayPath(GameState.alliance);
        Command delayCommand = startDelaySec > 0 ? waitMs(startDelaySec * 1000.0) : instant(() -> {});

        // Safely enable vision features ONLY IF manually enabled AND Limelight hardware initialized successfully
        boolean useVision = visionEnabled && robot.limelight.isAvailable();

        Command mainAutoSequence;

        if (useVision) {
            // Reactive Vision Routine using Limelight triggers & fallbacks
            mainAutoSequence = sequential(
                    delayCommand,

                    // 1. Initial Drive to Score & Launch immediately (facing score on start)
                    parallel(
                            follow(follower, path.scoreC6()),
                            robot.launcher.spinUpCommand()
                    ),
                    robot.launchSequenceCommand(),

                    // 2. Drive to Flower BC6 & Intake
                    parallel(
                            follow(follower, path.scoreToFlowerBC6()),
                            robot.intake.startCommand()
                    ),
                    waitMs(1500.0),
                    robot.intake.stopCommand(),

                    // 3. Return to Score, Prepare Limelight for Hive tip detection & Launch
                    parallel(
                            follow(follower, path.flowerBC6ToScore()),
                            robot.limelight.prepareForHiveDetectionCommand(robot.swivel)
                    ),
                    race(
                            robot.limelight.waitForHiveTippedCommand(),
                            waitMs(4000.0) // Timeout fallback if hive doesn't tip
                    ),
                    parallel(robot.launcher.spinUpCommand(), instant(() -> {})),
                    robot.launchSequenceCommand(),

                    // 4. Sweep up field elements with intake
                    parallel(
                            follow(follower, path.sweepUpBalls1()),
                            robot.intake.startCommand()
                    ),
                    follow(follower, path.sweepUpBalls2()),
                    robot.intake.stopCommand(),

                    // 5. Move toward park position & launch remaining elements
                    parallel(
                            follow(follower, path.sweepTowardPark()),
                            robot.launcher.spinUpCommand()
                    ),
                    robot.launchSequenceCommand()
            );
        } else {
            // Non-Limelight / Pure Pathing Fallback (Pure Time & Position Based Execution)
            mainAutoSequence = sequential(
                    delayCommand,

                    // 1. Initial Drive to Score & Launch
                    parallel(
                            follow(follower, path.scoreC6()),
                            robot.launcher.spinUpCommand()
                    ),
                    robot.launchSequenceCommand(),

                    // 2. Move to Flower BC6 & Intake
                    parallel(
                            follow(follower, path.scoreToFlowerBC6()),
                            robot.intake.startCommand()
                    ),
                    waitMs(1500.0),
                    robot.intake.stopCommand(),

                    // 3. Return to Score & Launch
                    parallel(
                            follow(follower, path.flowerBC6ToScore()),
                            robot.launcher.spinUpCommand()
                    ),
                    robot.launchSequenceCommand(),

                    // 4. Sweep up field elements with intake
                    parallel(
                            follow(follower, path.sweepUpBalls1()),
                            robot.intake.startCommand()
                    ),
                    follow(follower, path.sweepUpBalls2()),

                    // 5. Move toward park position
                    follow(follower, path.sweepTowardPark()),
                    robot.intake.stopCommand()
                    );
        }

        // Wrap main sequence in a 25-second race limit; interrupt whatever step is running at 25s and Park!
        return sequential(
                race(
                        mainAutoSequence,
                        waitMs(25000.0) // Hard 25-second limit
                ),
                // Final Parking Sequence
                follow(follower, path.parkA56())
        );
    }
}
