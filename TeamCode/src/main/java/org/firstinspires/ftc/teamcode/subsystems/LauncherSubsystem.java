package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.ivy.Command;
import static com.pedropathing.ivy.commands.Commands.*;
import static com.pedropathing.ivy.groups.Groups.sequential;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Subsystem for flywheel launcher mechanism with Ivy Commands using velocity control.
 */
public class LauncherSubsystem {
    private DcMotorEx leftFlywheel = null;
    private DcMotorEx rightFlywheel = null;
    public static double STARTING_SPEED = 3000; // starting velocity (RPM / ticks per second)

    public static double SPEED_INCREMENT = 50; // how much to adjust flywheel velocity
    public double currentSpeed = STARTING_SPEED;
    private boolean isSpinningUp = false;

    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        try {
            leftFlywheel = hardwareMap.get(DcMotorEx.class, "left_flywheel");
            rightFlywheel = hardwareMap.get(DcMotorEx.class, "right_flywheel");
            leftFlywheel.setDirection(DcMotor.Direction.REVERSE);
            rightFlywheel.setDirection(DcMotor.Direction.FORWARD);
            leftFlywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFlywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        } catch (Exception e) {
            // Hardware unconfigured
            telemetry.addLine("Launcher Hardware not configured");
        }
    }

    public void setVelocity(double velocity) {
        if (leftFlywheel != null) leftFlywheel.setVelocity(velocity);
        if (rightFlywheel != null) rightFlywheel.setVelocity(velocity);
    }

    public void spinUp() {
        isSpinningUp = true;
        setVelocity(currentSpeed);
    }

    public void stop() {
        isSpinningUp = false;
        setVelocity(0.0);
    }

    public void increaseSpeed() {
        currentSpeed += SPEED_INCREMENT;
        if (isSpinningUp) {
            spinUp();
        }
    }

    public void decreaseSpeed() {
        currentSpeed = Math.max(0, currentSpeed - SPEED_INCREMENT);
        if (isSpinningUp) {
            spinUp();
        }
    }

    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Launcher Target Velocity", currentSpeed);
        if (leftFlywheel != null && rightFlywheel != null) {
            telemetry.addData("Launcher Actual Velocity (L/R)", "%.1f / %.1f",
                    leftFlywheel.getVelocity(), rightFlywheel.getVelocity());
        }
    }

    // Ivy Commands
    public Command spinUpCommand() {
        return instant(this::spinUp);
    }

    public Command stopCommand() {
        return instant(this::stop);
    }

    /**
     * Spins up launcher, waits for flywheel spin-up time, then feeds pollen (leaving flywheel running).
     */
    public Command launchSequenceCommand(TransferSubsystem transfer, double feedDurationSeconds) {
        return sequential(
                instant(this::spinUp),
                waitMs(800), // Spin-up delay
                transfer.feedCommand(),
                waitMs(feedDurationSeconds * 1000.0),
                transfer.stopCommand()
        );
    }
}
