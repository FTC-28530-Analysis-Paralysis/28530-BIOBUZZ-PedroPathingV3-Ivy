package org.firstinspires.ftc.teamcode.subsystems;

import static com.pedropathing.ivy.commands.Commands.instant;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class GateServo {
    private Servo GateServo;

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
            GateServo = hardwareMap.get(Servo.class, "GateServo");
        } catch (Exception e) {
            // Hardware unconfigured
            if (telemetry != null) {
                telemetry.addLine("Gate Servo Hardware not configured");
            }
        }
    }

    /**
     * Sets the diverter gate to the specified position.
     * @param position The target position (Open or GREEN).
     */
    private void setPosition(GatePosition position) {
        GateServo.setPosition(position);
    }


    public Command Open() {
        return instant(this::setPosition(OPEN_POSITION));
    }

    private void setPosition(GatePosition position) {
    }


    public Command Close() {
        return
    }
}