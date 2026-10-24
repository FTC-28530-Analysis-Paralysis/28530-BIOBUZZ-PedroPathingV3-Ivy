package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SwivelSubsystem;

/**
 * Diagnostic OpMode to test Limelight 3A AprilTag reading on Hive structures
 * and calibrate Hive Tipped Up vs Tipped Down detection thresholds.
 */
@TeleOp(name = "Limelight Hive AprilTag Test", group = "Testing")
public class LimelightHiveTestOpMode extends OpMode {

    private LimelightSubsystem limelight;
    private SwivelSubsystem swivel;

    private final ElapsedTime debounceTimer = new ElapsedTime();
    private boolean isAprilTagPipeline = true;

    @Override
    public void init() {
        limelight = new LimelightSubsystem();
        limelight.init(hardwareMap, telemetry);

        swivel = new SwivelSubsystem();
        swivel.init(hardwareMap, telemetry);

        telemetry.addLine("=== LIMELIGHT HIVE TEST INITIALIZED ===");
        telemetry.addLine("Ready. Press START.");
        telemetry.update();
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
        telemetry.update();
    }

    private void handleControls() {
        if (debounceTimer.milliseconds() < 200) return;

        // Adjust vertical tip angle threshold
        if (gamepad1.dpad_up) {
            double currentThreshold = limelight.getTyTipThresholdDegrees();
            limelight.setTyTipThresholdDegrees(currentThreshold + 0.5);
            debounceTimer.reset();
        } else if (gamepad1.dpad_down) {
            double currentThreshold = limelight.getTyTipThresholdDegrees();
            limelight.setTyTipThresholdDegrees(Math.max(0.5, currentThreshold - 0.5));
            debounceTimer.reset();
        }

        // Swivel positions
        if (gamepad1.a) {
            swivel.lookAtHive();
            debounceTimer.reset();
        } else if (gamepad1.b) {
            swivel.lookAtField();
            debounceTimer.reset();
        }

        // Toggle pipeline
        if (gamepad1.y) {
            isAprilTagPipeline = !isAprilTagPipeline;
            limelight.setPipeline(isAprilTagPipeline ? 
                    LimelightSubsystem.Pipeline.APRIL_TAG_HIVE : 
                    LimelightSubsystem.Pipeline.BLOB_DETECTION);
            debounceTimer.reset();
        }
    }
}
