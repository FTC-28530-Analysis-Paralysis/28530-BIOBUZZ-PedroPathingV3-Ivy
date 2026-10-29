package org.firstinspires.ftc.teamcode.opmodes;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.DriverAssist;
import org.firstinspires.ftc.teamcode.subsystems.RobotContainer;

/**
 * Main TeleOp OpMode featuring:
 * 1. PedroPathing mecanum drive with DriverAssist (Field-Centric, Robot-Centric, and Hive Target Lock modes).
 * 2. Field-centric heading reset calibration button.
 * 3. Manual toggle for intake (ON / REVERSE / OFF).
 * 4. Asynchronous timed launching sequence using Ivy Command Scheduler.
 * 5. Swivel servo position control for field intake vs hive aiming.
 */
@TeleOp(name = "Morris TeleOp (PP3 + Ivy)", group = "01 Teleop")
public class MorrisPP3IvyTeleop extends OpMode {

    private Follower follower;
    private RobotContainer robot;
    private DriverAssist driverAssist;
    private final PoseFactory poseFactory = PoseFactory.degrees();

    public final Pose start = poseFactory.of(57, 8.5, 90);
    private boolean isIntakeRunning = false;

    @Override
    public void init() {
        Scheduler.reset();

        follower = Constants.create(hardwareMap);
        follower.setPose(start);
        follower.update();
        robot = new RobotContainer(hardwareMap, telemetry);
        driverAssist = new DriverAssist(follower);


        // Default swivel to field intake position
        robot.swivel.lookAtField();

        telemetry.addLine("=== 28530 BIOBUZZ TELEOP INITIALIZED ===");
        telemetry.addLine("Drive Mode Default: FIELD-CENTRIC");
        telemetry.addLine("Ready. Press START.");
    }

    @Override
    public void start() {
        Scheduler.reset();
    }

    @Override
    public void loop() {
        handleDriveControls();
        handleMechanismControls();

        // Update PedroPathing follower and execute scheduled Ivy Commands asynchronously
        follower.update();
        Scheduler.execute();

        // Telemetry
        telemetry.addLine("=== 28530 BIOBUZZ TELEOP ===");
        telemetry.addData("Drive Mode [Start Button]", driverAssist.getMode().getName());
        telemetry.addData("Heading (deg)", Math.toDegrees(follower.pose().heading()));
        telemetry.addData("Target Hive Pose", "X: %.1f | Y: %.1f",
                driverAssist.getNearestHivePose().x(), driverAssist.getNearestHivePose().y());
        telemetry.addLine("[Back/Select Button] Reset Heading (0°)");
        telemetry.addData("Robot Pose", "X: %.1f | Y: %.1f", follower.pose().x(), follower.pose().y());
        telemetry.addLine("--------------------------------");
        telemetry.addData("Intake Status [A / B]", isIntakeRunning ? "RUNNING" : "STOPPED");
        telemetry.addLine("[Right Bumper] Timed Launch Sequence");
        telemetry.addLine("[Left Bumper] Swivel Position (Hive / Field)");
    }

    private void handleDriveControls() {
        // Cycle Drive Modes: FIELD_CENTRIC -> ROBOT_CENTRIC -> FC_TARGET_LOCK -> FIELD_CENTRIC
        if (gamepad1.startWasPressed()) {
            DriverAssist.DriveMode[] modes = {
                    DriverAssist.DriveMode.FIELD_CENTRIC,
                    DriverAssist.DriveMode.ROBOT_CENTRIC,
                    DriverAssist.DriveMode.FC_TARGET_LOCK
            };
            int nextIdx = (driverAssist.getMode().ordinal() + 1) % modes.length;
            driverAssist.setMode(modes[nextIdx]);
        }

        // Reset field-centric heading calibration
        if (gamepad1.backWasPressed()) {
            follower.setPose(new Pose(follower.pose().x(), follower.pose().y(), 0));
        }

        // Update DriverAssist with joystick inputs (-Y is forward on gamepad sticks)
        driverAssist.update(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);
    }

    private void handleMechanismControls() {
        // Toggle Intake ON / OFF
        if (gamepad1.aWasPressed()) {
            isIntakeRunning = !isIntakeRunning;
            if (isIntakeRunning) {
                robot.intake.start();
            } else {
                robot.intake.stop();
            }
        }

        // Reverse Intake
        if (gamepad1.bWasPressed()) {
            isIntakeRunning = false;
            robot.intake.reverse();
        }

        // Swivel servo toggle
        if (gamepad1.leftBumperWasPressed()) {
            robot.swivel.lookAtHive();
        } else if (gamepad1.leftTriggerWasPressed()) {
            robot.swivel.lookAtField();
        }

        // Trigger timed launch sequence (spins up flywheel, feeds pollen, then stops)
        if (gamepad1.rightBumperWasPressed()) {
            isIntakeRunning = false;
            robot.intake.stop();
            Scheduler.schedule(robot.launcher.launchSequenceCommand(robot.transfer, 1.5));
        }
    }
}
