package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.ivy.Command;
import static com.pedropathing.ivy.commands.Commands.*;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Subsystem for transfer / feed continuous rotation servo with Ivy Commands.
 */
public class TransferSubsystem {
    private CRServo transferServo = null;

    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        try {
            transferServo = hardwareMap.get(CRServo.class, "transfer");
        } catch (Exception e) {
            // Hardware unconfigured
            telemetry.addLine("Transfer Hardware unconfigured");
        }
    }

    public void setPower(double power) {
        if (transferServo != null) {
            transferServo.setPower(power);
        }
    }

    public void feed() { setPower(0.8); }
    public void stop() { setPower(0.0); }
    public void reverse() { setPower(-0.8); }

    // Ivy Commands
    public Command feedCommand() { return instant(this::feed); }
    public Command stopCommand() { return instant(this::stop); }
    public Command reverseCommand() { return instant(this::reverse); }
}
