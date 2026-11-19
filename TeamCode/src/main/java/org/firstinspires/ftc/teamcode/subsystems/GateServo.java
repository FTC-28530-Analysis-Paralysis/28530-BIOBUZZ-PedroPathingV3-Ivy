package org.firstinspires.ftc.teamcode.subsystems;

import static com.pedropathing.ivy.commands.Commands.instant;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class GateServo {
    private Servo gateServo;

    // Define the servo positions for each track. These are placeholders and must be tuned.
    public static final double OPEN_POSITION = 0.0;
    public static final double CLOSED_POSITION = 1.0;

    public enum GatePosition {
        Open,  // It opens the gate
        Closed  // It closes the gate
    }

    GatePosition gatePosition = GatePosition.Open;

    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        try {
            gateServo = hardwareMap.get(Servo.class, "gate");
        } catch (Exception e) {
            // Hardware unconfigured
            if (telemetry != null) {
                telemetry.addLine("Gate Servo Hardware not configured");
            }
        }
    }

    /**
     * Sets the diverter gate to the specified position.
     * @param position The target position (Open or Closed).
     */
    private void setPosition(GatePosition position) {
        if (gateServo != null) {
            if (position == GatePosition.Open) {
                gateServo.setPosition(OPEN_POSITION);
            } else {
                gateServo.setPosition(CLOSED_POSITION);
            }
        }
    }

    public Command Open() {
        return instant(() -> setPosition(GatePosition.Open));
    }

    public Command Close() {
        return instant(() -> setPosition(GatePosition.Closed));
    }
}
