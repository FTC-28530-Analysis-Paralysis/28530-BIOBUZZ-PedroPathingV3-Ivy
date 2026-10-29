package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.hardware.ServoControllerEx;

import java.util.Set;

/*
 * This is a program inspired by MotorPortTest to increment through and set servo positions one at a time
 * on each Control Hub and Expansion Hub servo port.
 *
 * When a robot has complex wiring, this program helps identify which physical servo is plugged into which port.
 * It doesn't need to know the configured name of the servo on each port—it accesses the servo controller
 * by hub name and commands the port directly.
 *
 * Controls:
 * - D-Pad Left / Right: Cycle through 12 total servo ports (0-5 Control Hub, 0-5 Expansion Hub).
 * - Left Stick Y: Smoothly adjust servo target position (0.0 to 1.0).
 * - A Button: Set position to 0.0 (Min).
 * - B Button: Set position to 0.5 (Mid).
 * - Y Button: Set position to 1.0 (Max).
 * - X Button: Toggle PWM enable/disable status for the active hub.
 */
@TeleOp(name = "Servo Port Test", group = "Test")
public class ServoPortTest extends LinearOpMode {
    private int servoPortNumber = 0;
    private int selectionIndex = 0;
    private ServoControllerEx servoController;
    private final String[] hubOptions = {"Control Hub", "Expansion Hub 2"};
    private String selectedHub = null;
    private boolean dpadRightPressed = false;
    private boolean dpadLeftPressed = false;
    private boolean xButtonPressed = false;
    private double position = 0.5;

    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();

        while (opModeIsActive()) {
            boolean exceptionCaught = false;

            // Ensure selectionIndex wraps properly across 12 total servo ports (6 per hub x 2 hubs)
            selectionIndex = ((selectionIndex % 12) + 12) % 12;
            servoPortNumber = selectionIndex % 6; // 6 servo ports on each REV hub (0 to 5)

            // Ports 0-5 are on Control Hub (index / 6 == 0), Ports 6-11 are on Expansion Hub
            if (selectionIndex / 6 == 0) {
                selectedHub = hubOptions[0];
            } else {
                selectedHub = hubOptions[1];
            }

            try {
                servoController = hardwareMap.get(ServoControllerEx.class, selectedHub);
            } catch (IllegalArgumentException e) {
                telemetry.addData("Error", "Servo controller not found on " + selectedHub + " port " + servoPortNumber);
                exceptionCaught = true;
            }

            if (!exceptionCaught && servoController != null) {
                // Preset positions
                if (gamepad1.a) {
                    position = 0.0;
                } else if (gamepad1.b) {
                    position = 0.5;
                } else if (gamepad1.y) {
                    position = 1.0;
                } else if (Math.abs(gamepad1.left_stick_y) > 0.05) {
                    // Map left_stick_y (-1.0 to 1.0, where pushing stick UP is -1.0) to position 0.0 to 1.0
                    position = Math.max(0.0, Math.min(1.0, (1.0 - gamepad1.left_stick_y) / 2.0));
                }

                // Toggle PWM state with X button
                if (gamepad1.x && !xButtonPressed) {
                    if (servoController.getPwmStatus() == ServoControllerEx.PwmStatus.ENABLED) {
                        servoController.pwmDisable();
                    } else {
                        servoController.pwmEnable();
                    }
                    xButtonPressed = true;
                } else if (!gamepad1.x) {
                    xButtonPressed = false;
                }

                try {
                    servoController.setServoPosition(servoPortNumber, position);
                } catch (Exception e) {
                    telemetry.addData("Servo Error", e.getMessage());
                }
            }

            // D-Pad Left/Right navigation
            if (gamepad1.dpad_right && !dpadRightPressed) {
                selectionIndex++;
                dpadRightPressed = true;
            } else if (!gamepad1.dpad_right) {
                dpadRightPressed = false;
            }

            if (gamepad1.dpad_left && !dpadLeftPressed) {
                selectionIndex--;
                dpadLeftPressed = true;
            } else if (!gamepad1.dpad_left) {
                dpadLeftPressed = false;
            }

            // Calculate equivalent CR servo power (-1.0 to +1.0)
            double crPower = (position - 0.5) * 2.0;

            // Retrieve configured name from HardwareMap if set
            String configuredName = (servoController != null)
                    ? getConfiguredServoName(servoController, servoPortNumber)
                    : "<Unavailable>";

            // Telemetry
            telemetry.addData("Selected Hub", selectedHub);
            telemetry.addData("Selection Index", selectionIndex + " (0-11)");
            telemetry.addData("Servo Port", servoPortNumber + " (0-5)");
            telemetry.addData("Configured Name", configuredName);
            telemetry.addData("Target Position (Positional)", "%.3f", position);
            telemetry.addData("Equivalent Power (CR Servo)", "%.3f", crPower);
            if (servoController != null) {
                telemetry.addData("PWM Status", servoController.getPwmStatus());
            }
            telemetry.addLine("------------------------------------");
            telemetry.addLine("[D-Pad L/R] Cycle Servo Ports");
            telemetry.addLine("[Left Stick Y] Smooth Position / CR Speed Control");
            telemetry.addLine("[A/B/Y] Positional: 0.0 (A) | 0.5 (B) | 1.0 (Y)");
            telemetry.addLine("[A/B/Y] CR Servo: Rev (A) | Stop (B) | Fwd (Y)");
            telemetry.addLine("[X Button] Toggle PWM Enable/Disable");
            telemetry.update();
        }
    }

    private String getConfiguredServoName(ServoController targetController, int port) {
        for (Servo servo : hardwareMap.getAll(Servo.class)) {
            if (servo.getController().equals(targetController) && servo.getPortNumber() == port) {
                Set<String> names = hardwareMap.getNamesOf(servo);
                if (!names.isEmpty()) {
                    return names.iterator().next() + " [Servo]";
                }
            }
        }
        for (CRServo crServo : hardwareMap.getAll(CRServo.class)) {
            if (crServo.getController().equals(targetController) && crServo.getPortNumber() == port) {
                Set<String> names = hardwareMap.getNamesOf(crServo);
                if (!names.isEmpty()) {
                    return names.iterator().next() + " [CR Servo]";
                }
            }
        }
        return "<Unconfigured / None>";
    }
}
