package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.ivy.Command;
import static com.pedropathing.ivy.commands.Commands.*;
import static com.pedropathing.ivy.groups.Groups.sequential;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.List;

/**
 * Subsystem for Limelight 3A sensor with Ivy Commands, AprilTag tracking, and Hive Tip detection.
 */
public class LimelightSubsystem {
    private Limelight3A limelight = null;

    // Configurable vertical angle threshold (degrees) to determine tip status
    private double tyTipThresholdDegrees = 4.0;

    public enum Pipeline {
        APRIL_TAG_HIVE(0),
        BLOB_DETECTION(1);

        private final int id;
        Pipeline(int id) { this.id = id; }
        public int getId() { return id; }
    }

    public enum HiveTipState {
        NO_TAGS_VISIBLE("No AprilTags Detected"),
        UPRIGHT("Upright / Neutral"),
        TIPPED_UP("Tipped UP"),
        TIPPED_DOWN("Tipped DOWN");

        private final String description;
        HiveTipState(String description) { this.description = description; }
        public String getDescription() { return description; }
    }

    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        try {
            limelight = hardwareMap.get(Limelight3A.class, "limelight");
            limelight.pipelineSwitch(Pipeline.APRIL_TAG_HIVE.getId());
            limelight.start();
        } catch (Exception e) {
            // Hardware unconfigured
            telemetry.addLine("Limelight Hardware Unconfigured");
        }
    }

    public void setPipeline(Pipeline pipeline) {
        if (limelight != null) {
            limelight.pipelineSwitch(pipeline.getId());
        }
    }

    public LLResult getLatestResult() {
        if (limelight != null) {
            return limelight.getLatestResult();
        }
        return null;
    }

    public LLStatus getStatus() {
        if (limelight != null) {
            return limelight.getStatus();
        }
        return null;
    }

    /**
     * Returns list of currently visible AprilTag fiducial results.
     */
    public List<LLResultTypes.FiducialResult> getFiducialResults() {
        LLResult result = getLatestResult();
        if (result != null && result.isValid()) {
            return result.getFiducialResults();
        }
        return new ArrayList<>();
    }

    /**
     * Calculates the estimated Hive Tip state based on average vertical angle (ty) of visible AprilTags.
     */
    public HiveTipState getHiveTipState() {
        List<LLResultTypes.FiducialResult> fiducials = getFiducialResults();
        if (fiducials.isEmpty()) {
            return HiveTipState.NO_TAGS_VISIBLE;
        }

        double sumTy = 0.0;
        for (LLResultTypes.FiducialResult fr : fiducials) {
            sumTy += fr.getTargetYDegrees();
        }
        double avgTy = sumTy / fiducials.size();

        if (avgTy > tyTipThresholdDegrees) {
            return HiveTipState.TIPPED_UP;
        } else if (avgTy < -tyTipThresholdDegrees) {
            return HiveTipState.TIPPED_DOWN;
        } else {
            return HiveTipState.UPRIGHT;
        }
    }

    public boolean isHiveTipped() {
        HiveTipState state = getHiveTipState();
        return state == HiveTipState.TIPPED_UP || state == HiveTipState.TIPPED_DOWN;
    }

    public boolean seesBallBlob() {
        LLResult result = getLatestResult();
        return result != null && result.isValid();
    }

    public double getTyTipThresholdDegrees() {
        return tyTipThresholdDegrees;
    }

    public void setTyTipThresholdDegrees(double threshold) {
        this.tyTipThresholdDegrees = threshold;
    }

    /**
     * Outputs rich telemetry for debugging Limelight AprilTag readings and Hive tip status.
     */
    public void printTelemetry(Telemetry telemetry) {
        if (limelight == null) {
            telemetry.addData("Limelight", "NOT CONFIGURED / NULL");
            return;
        }

        LLStatus status = getStatus();
        if (status != null) {
            telemetry.addData("Limelight Status", "Temp: %.1fC | FPS: %d | Pipeline: %d",
                    status.getTemp(), (int) status.getFps(), status.getPipelineIndex());
        }

        LLResult result = getLatestResult();
        if (result != null && result.isValid()) {
            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
            telemetry.addData("Visible Tags Count", fiducials.size());
            telemetry.addData("Hive Tip Status", getHiveTipState().getDescription());
            telemetry.addData("Tip Threshold Angle", "±%.1f deg", tyTipThresholdDegrees);

            StringBuilder tagIds = new StringBuilder();
            for (LLResultTypes.FiducialResult fr : fiducials) {
                tagIds.append("#").append(fr.getFiducialId()).append(" ");
            }
            telemetry.addData("Detected Tag IDs", tagIds.toString().trim());

            for (LLResultTypes.FiducialResult fr : fiducials) {
                org.firstinspires.ftc.robotcore.external.navigation.Pose3D pose = fr.getCameraPoseTargetSpace();
                double pitch = (pose != null && pose.getOrientation() != null) ? 
                        pose.getOrientation().getPitch(org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES) : 0.0;
                telemetry.addLine(String.format(java.util.Locale.US, " -> Tag #%d | tx: %.1f° | ty: %.1f° | 3D Pitch: %.1f° | Area: %.2f%%",
                        fr.getFiducialId(), fr.getTargetXDegrees(), fr.getTargetYDegrees(), pitch, fr.getTargetArea()));
            }
        } else {
            telemetry.addData("Limelight Target", "No valid target/tags in view");
        }
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
