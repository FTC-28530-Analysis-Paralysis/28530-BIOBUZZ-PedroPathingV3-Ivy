package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.ivy.Command;
import static com.pedropathing.ivy.commands.Commands.*;
import static com.pedropathing.ivy.groups.Groups.sequential;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Subsystem for flywheel launcher mechanism with Ivy Commands.
 */
public class LauncherSubsystem {
    private DcMotor leftFlywheel = null;
    private DcMotor rightFlywheel = null;

    public void init(HardwareMap hardwareMap) {
        try {
            leftFlywheel = hardwareMap.get(DcMotor.class, "left_flywheel");
            rightFlywheel = hardwareMap.get(DcMotor.class, "right_flywheel");
            leftFlywheel.setDirection(DcMotor.Direction.REVERSE);
            rightFlywheel.setDirection(DcMotor.Direction.FORWARD);
            leftFlywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightFlywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        } catch (Exception e) {
            // Hardware unconfigured
        }
    }

    public void setPower(double power) {
        if (leftFlywheel != null) leftFlywheel.setPower(power);
        if (rightFlywheel != null) rightFlywheel.setPower(power);
    }

    public void spinUp() { setPower(1.0); }
    public void stop() { setPower(0.0); }

    // Ivy Commands
    public Command spinUpCommand() {
        return instant(this::spinUp);
    }

    public Command stopCommand() {
        return instant(this::stop);
    }

    /**
     * Spins up launcher, waits for flywheel spin-up time, feeds pollen, then stops.
     */
    public Command launchSequenceCommand(TransferSubsystem transfer, double feedDurationSeconds) {
        return sequential(
                instant(this::spinUp),
                waitMs(800), // Spin-up delay
                transfer.feedCommand(),
                waitMs(feedDurationSeconds * 1000.0),
                transfer.stopCommand(),
                instant(this::stop)
        );
    }
}
