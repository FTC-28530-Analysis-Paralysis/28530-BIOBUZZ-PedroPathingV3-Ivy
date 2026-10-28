package org.firstinspires.ftc.teamcode.opmodes;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.*;
import static com.pedropathing.ivy.groups.Groups.parallel;
import static com.pedropathing.ivy.groups.Groups.race;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.paths.HiveUpTeamplayPath;
import org.firstinspires.ftc.teamcode.paths.PathContainer;
import org.firstinspires.ftc.teamcode.subsystems.RobotContainer;

/**
 * Main Autonomous OpMode featuring:
 * 1. An interactive initialization menu on gamepad1 to select path routines, start delays, and vision modes.
 * 2. Separation of path definitions (PathContainer classes) from execution logic.
 * 3. Modular Ivy Command routines stitching PedroPathing follower paths with mechanism subsystems.
 * 4. Extends FTC's iterative OpMode structure (init, init_loop, start, loop).
 */
@Autonomous(name = "Configurable Auto (PP3 + Ivy)", group = "01 Autonomous")
public class ConfigurableAutoOpMode extends OpMode {

    private Follower follower;
    private RobotContainer robot;

    // Selection Options
    public enum SelectedRoutine {
        HIVE_UP_TEAMPLAY("Hive Up Teamplay"),
        PARK_ONLY("Park Only");

        private final String name;
        SelectedRoutine(String name) { this.name = name; }
        public String getName() { return name; }
    }

    private SelectedRoutine selectedRoutine = SelectedRoutine.HIVE_UP_TEAMPLAY;
    private double startDelaySeconds = 0.0;
    private boolean visionInterruptionEnabled = true;

    @Override
    public void init() {
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        robot = new RobotContainer(hardwareMap, telemetry);

        PathContainer currentPathContainer = getSelectedPathContainer();
        follower.setPose(currentPathContainer.getStartPose());
        follower.update();
    }

    @Override
    public void init_loop() {
        handleMenuInput();

        telemetry.addLine("=== 28530 BIOBUZZ AUTO CONFIGURATION ===");
        telemetry.addData("[D-Pad Up/Down] Selected Routine", selectedRoutine.getName());
        telemetry.addData("[D-Pad Left/Right] Start Delay", "%.1f sec", startDelaySeconds);
        telemetry.addData("[Y Button] Vision Interruption", visionInterruptionEnabled ? "ENABLED (Limelight Interrupts)" : "DISABLED (Pure Pathing)");
        telemetry.addLine("---------------------------------------");
        telemetry.addLine("Press START when driver station is ready.");
    }

    @Override
    public void start() {
        // Instantiate chosen PathContainer and sync initial follower pose
        PathContainer currentPathContainer = getSelectedPathContainer();
        follower.setPose(currentPathContainer.getStartPose());
        follower.update();

        // Build and schedule Ivy Command Routine
        Command autoRoutine = buildAutoRoutine(currentPathContainer);
        schedule(autoRoutine);
    }

    @Override
    public void loop() {
        follower.update();
        Scheduler.execute();

        telemetry.addData("X", follower.pose().x());
        telemetry.addData("Y", follower.pose().y());
        telemetry.addData("Heading (deg)", Math.toDegrees(follower.pose().heading()));

        if (follower.currentPath() != null) {
            telemetry.addData("Path Remaining Distance", follower.distanceToEndpoint());
            telemetry.addData("Path Index", follower.pathIndex());
        }
    }

    private PathContainer getSelectedPathContainer() {
        switch (selectedRoutine) {
            case HIVE_UP_TEAMPLAY:
            default:
                return new HiveUpTeamplayPath();
        }
    }

    /**
     * Stitches together PedroPathing follower commands and mechanism subsystem commands using Ivy.
     */
    private Command buildAutoRoutine(PathContainer pathContainer) {
        Command delayCommand = startDelaySeconds > 0 ? waitMs(startDelaySeconds * 1000.0) : instant(() -> {});

        if (pathContainer instanceof HiveUpTeamplayPath) {
            HiveUpTeamplayPath path = (HiveUpTeamplayPath) pathContainer;

            if (visionInterruptionEnabled) {
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

        // Default Fallback
        return sequential(
                delayCommand,
                follow(follower, pathContainer.getPath("startToScore"))
        );
    }

    private void handleMenuInput() {
        if (gamepad1.dpadUpWasPressed()) {
            SelectedRoutine[] routines = SelectedRoutine.values();
            int nextIndex = (selectedRoutine.ordinal() + 1) % routines.length;
            selectedRoutine = routines[nextIndex];
        } else if (gamepad1.dpadDownWasPressed()) {
            SelectedRoutine[] routines = SelectedRoutine.values();
            int prevIndex = (selectedRoutine.ordinal() - 1 + routines.length) % routines.length;
            selectedRoutine = routines[prevIndex];
        } else if (gamepad1.dpadRightWasPressed()) {
            startDelaySeconds = Math.min(10.0, startDelaySeconds + 1.0);
        } else if (gamepad1.dpadLeftWasPressed()) {
            startDelaySeconds = Math.max(0.0, startDelaySeconds - 1.0);
        } else if (gamepad1.yWasPressed()) {
            visionInterruptionEnabled = !visionInterruptionEnabled;
        }
    }
}
