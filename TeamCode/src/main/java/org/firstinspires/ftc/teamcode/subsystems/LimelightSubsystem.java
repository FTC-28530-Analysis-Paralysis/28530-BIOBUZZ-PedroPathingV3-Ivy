package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.ivy.Command;
import static com.pedropathing.ivy.commands.Commands.*;
import static com.pedropathing.ivy.groups.Groups.sequential;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Subsystem for Limelight 3A sensor with Ivy Commands and Condition Suppliers.
 */
public class LimelightSubsystem {
    private Limelight3A limelight = null;

    public enum Pipeline {
        APRIL_TAG_HIVE(0),
        BLOB_DETECTION(1);

        private final int id;
        Pipeline(int id) { this.id = id; }
        public int getId() { return id; }
    }

    public void init(HardwareMap hardwareMap) {
        try {
            limelight = hardwareMap.get(Limelight3A.class, "limelight");
            limelight.pipelineSwitch(Pipeline.APRIL_TAG_HIVE.getId());
            limelight.start();
        } catch (Exception e) {
            // Hardware unconfigured
        }
    }

    public void setPipeline(Pipeline pipeline) {
        if (limelight != null) {
            limelight.pipelineSwitch(pipeline.getId());
        }
    }

    public boolean isHiveTipped() {
        if (limelight != null) {
            LLResult result = limelight.getLatestResult();
            return result != null && result.isValid();
        }
        return false;
    }

    public boolean seesBallBlob() {
        if (limelight != null) {
            LLResult result = limelight.getLatestResult();
            return result != null && result.isValid();
        }
        return false;
    }

    // Ivy Commands & Preparation
    public Command prepareForHiveDetectionCommand(SwivelSubsystem swivel) {
        return sequential(
                swivel.lookAtHiveCommand(),
                instant(() -> setPipeline(Pipeline.APRIL_TAG_HIVE))
        );
    }

    public Command prepareForBlobDetectionCommand(SwivelSubsystem swivel) {
        return sequential(
                swivel.lookAtFieldCommand(),
                instant(() -> setPipeline(Pipeline.BLOB_DETECTION))
        );
    }

    public Command waitForHiveTippedCommand() {
        return waitUntil(this::isHiveTipped);
    }

    public Command waitForBallDetectedCommand() {
        return waitUntil(this::seesBallBlob);
    }
}
