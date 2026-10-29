package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SwivelSubsystem;

/**
 * Diagnostic OpMode to test Limelight 3A AprilTag reading on Hive structures
 * and calibrate Hive Tipped Up vs Tipped Down detection thresholds using rising-edge button detection.
 */
@TeleOp(name = "Limelight Hive AprilTag Test", group = "02 Testing")
public class LimelightHiveTestOpMode extends OpMode {

    private LimelightSubsystem limelight;
    private SwivelSubsystem swivel;

    private boolean isAprilTagPipeline = true;

    @Override
    public void init() {
        limelight = new LimelightSubsystem();
        limelight.init(hardwareMap, telemetry);

        swivel = new SwivelSubsystem();
        swivel.init(hardwareMap, telemetry);

        telemetry.addLine("=== LIMELIGHT HIVE TEST INITIALIZED ===");
        telemetry.addLine("Ready. Press START.");
    }

    @Override
    public void loop() {
        handleControls();

        telemetry.addLine("=== LIMELIGHT HIVE APRILTAG TEST ===");
        telemetry.addLine("[D-Pad Up/Down] Change Tip Angle Threshold");
        telemetry.addLine("[A Button] Swivel Up (Hive) | [B Button] Swivel Down (Field)");
        telemetry.addLine("[Y Button] Toggle Pipeline (AprilTag / Blob)");
        telemetry.addLine("------------------------------------");

        limelight.printTelemetry(telemetry);
    }

    private void handleControls() {
        // Adjust vertical tip angle threshold
        if (gamepad1.dpadUpWasPressed()) {
            double currentThreshold = limelight.getTyTipThresholdDegrees();
            limelight.setTyTipThresholdDegrees(currentThreshold + 0.5);
        } else if (gamepad1.dpadDownWasPressed()) {
            double currentThreshold = limelight.getTyTipThresholdDegrees();
            limelight.setTyTipThresholdDegrees(Math.max(0.5, currentThreshold - 0.5));
        }

        // Swivel positions
        if (gamepad1.aWasPressed()) {
            swivel.lookAtHive();
        } else if (gamepad1.bWasPressed()) {
            swivel.lookAtField();
        }

        // Toggle pipeline
        if (gamepad1.yWasPressed()) {
            isAprilTagPipeline = !isAprilTagPipeline;
            limelight.setPipeline(isAprilTagPipeline ? 
                    LimelightSubsystem.Pipeline.APRIL_TAG_HIVE : 
                    LimelightSubsystem.Pipeline.BLOB_DETECTION);
        }
    }
}
