package org.firstinspires.ftc.teamcode.routines;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import static com.pedropathing.ivy.commands.Commands.*;
import static com.pedropathing.ivy.groups.Groups.*;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.config.GameState;
import org.firstinspires.ftc.teamcode.paths.HiveUpTeamplayPath;
import org.firstinspires.ftc.teamcode.subsystems.RobotContainer;

public class HiveUpTeamplayRoutine implements AutoRoutine {

    @Override
    public String getName() {
        return "Hive Up Teamplay (" + GameState.alliance.getName() + ")";
    }

    @Override
    public Pose getStartPose() {
        return new HiveUpTeamplayPath(GameState.alliance).getStartPose();
    }

    @Override
    public Command buildCommand(Follower follower, RobotContainer robot, boolean visionEnabled, double startDelaySec) {
        HiveUpTeamplayPath path = new HiveUpTeamplayPath(GameState.alliance);
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
                            follow(follower, path.startToScore()),
                            robot.launcher.spinUpCommand()
                    ),
                    robot.launcher.launchSequenceCommand(robot.transfer, 1.0),

                    // 2. Drive to Garden Approach on Alliance side & search for Pollen/Nectar (Max 8 seconds)
                    parallel(
                            follow(follower, path.scoreToGardenApproach()),
                            robot.limelight.prepareForBlobDetectionCommand(robot.swivel)
                    ),
                    race(
                            sequential(
                                    follow(follower, path.gardenPickup()),
                                    conditional(
                                            robot.limelight::seesBallBlob,
                                            sequential(
                                                    robot.intake.startCommand(),
                                                    waitMs(1500),
                                                    robot.intake.stopCommand()
                                            ),
                                            instant(() -> {}) // No element spotted
                                    )
                            ),
                            waitMs(8000.0) // 8-second search limit on alliance side
                    ),

                    // 3. Move back to score position & wait for Hive to tip up before launching
                    follow(follower, path.gardenToScore()),
                    robot.limelight.prepareForHiveDetectionCommand(robot.swivel),
                    race(
                            robot.limelight.waitForHiveTippedCommand(),
                            waitMs(4000.0) // Timeout fallback if hive doesn't tip
                    ),
                    parallel(robot.launcher.spinUpCommand(), instant(() -> {})),
                    robot.launcher.launchSequenceCommand(robot.transfer, 1.0),

                    // 4. Head to Flower, Intake element, Return to Score & Launch
                    follow(follower, path.scoreToFlower()),
                    sequential(
                            robot.intake.startCommand(),
                            waitMs(1500),
                            robot.intake.stopCommand()
                    ),
                    follow(follower, path.flowerToScore()),
                    robot.launcher.launchSequenceCommand(robot.transfer, 1.0)
            );
        } else {
            // Non-Limelight / Pure Pathing Fallback (Pure Time & Position Based Execution)
            mainAutoSequence = sequential(
                    delayCommand,

                    // 1. Start to Score & Launch
                    parallel(
                            follow(follower, path.startToScore()),
                            robot.launcher.spinUpCommand()
                    ),
                    robot.launcher.launchSequenceCommand(robot.transfer, 1.0),

                    // 2. Garden Approach & Pickup
                    follow(follower, path.scoreToGardenApproach()),
                    follow(follower, path.gardenPickup()),

                    // 3. Return to Score & Launch
                    parallel(
                            follow(follower, path.gardenToScore()),
                            robot.launcher.spinUpCommand()
                    ),
                    robot.launcher.launchSequenceCommand(robot.transfer, 1.0),

                    // 4. Head to Flower, Intake, Return to Score & Launch
                    follow(follower, path.scoreToFlower()),
                    sequential(
                            robot.intake.startCommand(),
                            waitMs(1500),
                            robot.intake.stopCommand()
                    ),
                    parallel(
                            follow(follower, path.flowerToScore()),
                            robot.launcher.spinUpCommand()
                    ),
                    robot.launcher.launchSequenceCommand(robot.transfer, 1.0)
            );
        }

        // Wrap main sequence in a 25-second race limit; interrupt whatever step is running at 25s and Park!
        return sequential(
                race(
                        mainAutoSequence,
                        waitMs(25000.0) // Hard 25-second limit
                ),
                // Final Parking Sequence
                follow(follower, path.scoreToPark())
        );
    }
}
