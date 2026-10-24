package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.ivy.Command;
import static com.pedropathing.ivy.commands.Commands.*;
import static com.pedropathing.ivy.groups.Groups.sequential;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Subsystem for collecting pollen/nectar wrapped with Ivy Commands.
 */
public class IntakeSubsystem {
    private DcMotor intakeMotor = null;

    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        try {
            intakeMotor = hardwareMap.get(DcMotor.class, "intake");
            intakeMotor.setDirection(DcMotor.Direction.FORWARD);
            intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        } catch (Exception e) {
            // Hardware unconfigured
            telemetry.addLine("Intake hardware not configured.");
        }
    }

    public void setPower(double power) {
        if (intakeMotor != null) {
            intakeMotor.setPower(power);
        }
    }

    public void start() { setPower(1.0); }
    public void stop() { setPower(0.0); }
    public void reverse() { setPower(-1.0); }

    // Ivy Commands
    public Command startCommand() {
        return instant(this::start);
    }

    public Command stopCommand() {
        return instant(this::stop);
    }

    public Command intakeForDurationCommand(double seconds) {
        return sequential(
                instant(this::start),
                waitMs(seconds * 1000.0),
                instant(this::stop)
        );
    }
}
