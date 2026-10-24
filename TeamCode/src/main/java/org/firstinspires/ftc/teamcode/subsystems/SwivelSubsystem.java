package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.ivy.Command;
import static com.pedropathing.ivy.commands.Commands.*;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Subsystem for Limelight tilting servo (tilting up at hive vs down at field).
 */
public class SwivelSubsystem {
    private Servo swivelServo = null;

    public static final double POSITION_UP_HIVE = 0.85;
    public static final double POSITION_DOWN_FIELD = 0.15;

    public void init(HardwareMap hardwareMap) {
        try {
            swivelServo = hardwareMap.get(Servo.class, "swivel");
        } catch (Exception e) {
            // Hardware unconfigured
        }
    }

    public void setPosition(double position) {
        if (swivelServo != null) {
            swivelServo.setPosition(position);
        }
    }

    public void lookAtHive() { setPosition(POSITION_UP_HIVE); }
    public void lookAtField() { setPosition(POSITION_DOWN_FIELD); }

    // Ivy Commands
    public Command lookAtHiveCommand() { return instant(this::lookAtHive); }
    public Command lookAtFieldCommand() { return instant(this::lookAtField); }
}
