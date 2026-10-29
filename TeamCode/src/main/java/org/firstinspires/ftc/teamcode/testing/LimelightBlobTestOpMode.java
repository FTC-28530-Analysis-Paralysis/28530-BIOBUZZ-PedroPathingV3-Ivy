package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.config.GameState;
import org.firstinspires.ftc.teamcode.subsystems.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SwivelSubsystem;

/**
 * Diagnostic OpMode to test Limelight 3A Color Blob detection for Pollen (Yellow)
 * and Nectar (Red / Blue) across Red and Blue Alliances using FTC SDK rising-edge button detection.
 */
@TeleOp(name = "Limelight Blob & Nectar Test", group = "02 Testing")
public class LimelightBlobTestOpMode extends OpMode {

    private LimelightSubsystem limelight;
    private SwivelSubsystem swivel;

    @Override
    public void init() {
        limelight = new LimelightSubsystem();
        limelight.init(hardwareMap, telemetry);

        swivel = new SwivelSubsystem();
        swivel.init(hardwareMap, telemetry);

        // Default to looking down at field for blob pickup
        swivel.lookAtField();
        limelight.setPipelineForTarget(GameState.targetElement);

        telemetry.addLine("=== LIMELIGHT BLOB TEST INITIALIZED ===");
        telemetry.addLine("Ready. Press START.");
    }

    @Override
    public void loop() {
        handleControls();

        telemetry.addLine("=== 28530 BIOBUZZ LIMELIGHT BLOB TEST ===");
        telemetry.addData("Selected Alliance [D-Pad L/R]", GameState.alliance.getName());
        telemetry.addData("Target Element [D-Pad Up/Dn]", GameState.targetElement.getDescription());
        telemetry.addData("Effective Target Color", GameState.getEffectiveTargetElement().name());
        telemetry.addLine("-----------------------------------------");
        telemetry.addLine("[A/B] Swivel (Hive / Field) | [X] Sync Pipeline");
        telemetry.addLine("-----------------------------------------");

        limelight.printBlobTelemetry(telemetry);
    }

    private void handleControls() {
        // Toggle Alliance (Red / Blue) using rising-edge detection
        if (gamepad1.dpadLeftWasPressed() || gamepad1.dpadRightWasPressed()) {
            GameState.alliance = (GameState.alliance == GameState.Alliance.BLUE) ? 
                    GameState.Alliance.RED : GameState.Alliance.BLUE;
            limelight.setPipelineForTarget(GameState.targetElement);
        }

        // Cycle Target Element (Yellow Pollen, Alliance Nectar, Red Nectar, Blue Nectar)
        if (gamepad1.dpadUpWasPressed()) {
            GameState.TargetElement[] elements = GameState.TargetElement.values();
            int nextIdx = (GameState.targetElement.ordinal() + 1) % elements.length;
            GameState.targetElement = elements[nextIdx];
            limelight.setPipelineForTarget(GameState.targetElement);
        } else if (gamepad1.dpadDownWasPressed()) {
            GameState.TargetElement[] elements = GameState.TargetElement.values();
            int prevIdx = (GameState.targetElement.ordinal() - 1 + elements.length) % elements.length;
            GameState.targetElement = elements[prevIdx];
            limelight.setPipelineForTarget(GameState.targetElement);
        }

        // Swivel servo positions
        if (gamepad1.aWasPressed()) {
            swivel.lookAtHive();
        } else if (gamepad1.bWasPressed()) {
            swivel.lookAtField();
        }

        // Force pipeline sync
        if (gamepad1.xWasPressed()) {
            limelight.setPipelineForTarget(GameState.targetElement);
        }
    }
}
