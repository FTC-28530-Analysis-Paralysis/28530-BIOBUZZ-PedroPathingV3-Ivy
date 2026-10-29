package org.firstinspires.ftc.teamcode.opmodes;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import static com.pedropathing.ivy.Scheduler.schedule;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.config.GameState;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.routines.AutoRoutine;
import org.firstinspires.ftc.teamcode.routines.HiveDownTeamplayRoutine;
import org.firstinspires.ftc.teamcode.routines.HiveUpTeamplayRoutine;
import org.firstinspires.ftc.teamcode.routines.ParkOnlyRoutine;
import org.firstinspires.ftc.teamcode.subsystems.RobotContainer;

/**
 * Main Autonomous OpMode featuring:
 * 1. An interactive initialization menu on gamepad1 to select path routines, start delays, and vision modes.
 * 2. Separation of pure PedroPathing visualizer exports (paths/) and Ivy mechanism commands (routines/).
 * 3. Extends FTC's iterative OpMode structure (init, init_loop, start, loop).
 */
@Autonomous(name = "Configurable Auto (PP3 + Ivy)", group = "01 Autonomous")
public class ConfigurableAutoOpMode extends OpMode {

    private Follower follower;
    private RobotContainer robot;

    // Selection Options linked to AutoRoutine implementations
    public enum SelectedRoutine {
        HIVE_UP_TEAMPLAY(new HiveUpTeamplayRoutine()),
        HIVE_DOWN_TEAMPLAY(new HiveDownTeamplayRoutine()),
        PARK_ONLY(new ParkOnlyRoutine());

        private final AutoRoutine routine;
        SelectedRoutine(AutoRoutine routine) { this.routine = routine; }
        public AutoRoutine getRoutine() { return routine; }
        public String getName() { return routine.getName(); }
    }

    private SelectedRoutine selectedRoutine = SelectedRoutine.HIVE_UP_TEAMPLAY;
    private double startDelaySeconds = 0.0;
    private boolean visionInterruptionEnabled = false;

    @Override
    public void init() {
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        robot = new RobotContainer(hardwareMap, telemetry);

        AutoRoutine routine = selectedRoutine.getRoutine();
        follower.setPose(routine.getStartPose());
        follower.update();
    }

    @Override
    public void init_loop() {
        handleMenuInput();

        AutoRoutine routine = selectedRoutine.getRoutine();
        follower.setPose(routine.getStartPose());
        follower.update();

        telemetry.addLine("=== 28530 BIOBUZZ AUTO CONFIGURATION ===");
        telemetry.addData("[D-Pad Up/Down] Selected Routine", selectedRoutine.getName());
        telemetry.addData("[X Button] Selected Alliance", GameState.alliance.getName());
        telemetry.addData("[D-Pad Left/Right] Start Delay", "%.1f sec", startDelaySeconds);
        telemetry.addData("[Y Button] Vision Interruption", visionInterruptionEnabled ? "ENABLED (Limelight Interrupts)" : "DISABLED (Pure Pathing)");
        telemetry.addLine("---------------------------------------");
        telemetry.addData("Start Pose X", "%.1f", routine.getStartPose().x());
        telemetry.addData("Start Pose Y", "%.1f", routine.getStartPose().y());
        telemetry.addData("Start Heading", "%.1f deg", Math.toDegrees(routine.getStartPose().heading()));
        telemetry.addLine("---------------------------------------");
        telemetry.addLine("Press START when driver station is ready.");
    }

    @Override
    public void start() {
        AutoRoutine routine = selectedRoutine.getRoutine();
        follower.setPose(routine.getStartPose());
        follower.update();

        // Build and schedule Ivy Command Routine
        Command autoCommand = routine.buildCommand(follower, robot, visionInterruptionEnabled, startDelaySeconds);
        schedule(autoCommand);
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

    @Override
    public void stop(){
        GameState.autonomousEndPose = follower.pose();
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
        } else if (gamepad1.xWasPressed()) {
            GameState.alliance = (GameState.alliance == GameState.Alliance.RED)
                    ? GameState.Alliance.BLUE
                    : GameState.Alliance.RED;
        } else if (gamepad1.dpadRightWasPressed()) {
            startDelaySeconds = Math.min(10.0, startDelaySeconds + 1.0);
        } else if (gamepad1.dpadLeftWasPressed()) {
            startDelaySeconds = Math.max(0.0, startDelaySeconds - 1.0);
        } else if (gamepad1.yWasPressed()) {
            visionInterruptionEnabled = !visionInterruptionEnabled;
        }
    }
}
