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

import org.firstinspires.ftc.teamcode.config.GameState;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

/**
 * Subsystem for Limelight 3A Vision Sensor with Ivy Commands, AprilTag tracking, and Hive Tip detection.
 *
 * <p>========================================================================================</p>
 * <h2>1. LIMELIGHT 3A WEB PORTAL CONFIGURATION GUIDE</h2>
 * <p>To set up the Limelight 3A sensor via its web interface:</p>
 * <ul>
 *   <li><b>Access Web UI:</b> Connect your computer to the Control Hub / Limelight network and navigate
 *       to <code>http://limelight.local:5801</code> (or <code>http://192.168.43.1:5801</code>).</li>
 *   <li><b>Pipeline 0 (AprilTag Hive):</b>
 *     <ul>
 *       <li>Set <i>Pipeline Type</i> to <b>AprilTag</b>.</li>
 *       <li>In <i>Input / Camera Controls</i>, lower camera exposure so AprilTag black/white edges are crisp
 *           and not washed out by bright arena lighting.</li>
 *       <li>In <i>3D / AprilTag Settings</i>, select family <b>36h11</b> (or official FTC tag family)
 *           and set target size to exact tag width (e.g. 108 mm / 4.25 inches).</li>
 *     </ul>
 *   </li>
 *   <li><b>Pipeline 1 (Yellow Pollen Detection):</b>
 *     <ul>
 *       <li>Set <i>Pipeline Type</i> to <b>Color HSV Threshold</b> or Neural Detector tuned for Yellow Pollen.</li>
 *     </ul>
 *   </li>
 *   <li><b>Pipeline 2 (Red Nectar Detection):</b>
 *     <ul>
 *       <li>Set <i>Pipeline Type</i> to <b>Color HSV Threshold</b> tuned for Red Nectar.</li>
 *     </ul>
 *   </li>
 *   <li><b>Pipeline 3 (Blue Nectar Detection):</b>
 *     <ul>
 *       <li>Set <i>Pipeline Type</i> to <b>Color HSV Threshold</b> tuned for Blue Nectar.</li>
 *     </ul>
 *   </li>
 * </ul>
 *
 * <p>========================================================================================</p>
 * <h2>2. SWIVEL SERVO GEOMETRY & CAMERA MOUNTING OFFSETS</h2>
 * <p>When the Limelight is mounted on a tilting servo mechanism (managed by {@link SwivelSubsystem}):</p>
 * <ul>
 *   <li><b>Dynamic Pitch Angle:</b> Because the servo actively changes the camera's physical pitch
 *       (e.g., looking UP at the Hive vs. looking DOWN at the field), static camera mounting offsets in the
 *       Limelight Web UI should remain set to <code>0</code>.</li>
 *   <li><b>Repeatable Aiming:</b> Before reading Hive AprilTags, the autonomous or teleop routine commands
 *       <code>swivel.lookAtHive()</code>. Holding the servo at a fixed, repeatable angle ensures that any
 *       measured change in vertical angle corresponds directly to the Hive tipping up or down.</li>
 * </ul>
 *
 * <p>========================================================================================</p>
 * <h2>3. 2D ELEVATION (ty) vs. 3D PNP TARGET SPACE PITCH</h2>
 * <ul>
 *   <li><b>2D Vertical Offset (<code>ty</code>):</b> The angle (in degrees) from the camera lens optical center
 *       vertically to the center of the AprilTag. When aimed from a known scoring position, an upright Hive
 *       yields a consistent <code>ty</code> baseline; if the Hive tips toward the robot (TIPPED UP),
 *       <code>ty</code> increases ($+ty$), and if it tips away (TIPPED DOWN), <code>ty</code> decreases ($-ty$).</li>
 *   <li><b>3D Target Space Pose Pitch ({@link LLResultTypes.FiducialResult#getCameraPoseTargetSpace()}):</b>
 *       Limelight utilizes Perspective-n-Point (PnP) geometry solving across the 4 corners of the rigid square
 *       AprilTag to compute the tag's true 3D pose $(X, Y, Z, \text{Roll}, \text{Pitch}, \text{Yaw})$.
 *       The 3D Pitch measures the physical 3D surface tilt angle of the tag, providing an invariant tilt
 *       measurement independent of robot distance or camera view angle as the robot drives around.</li>
 * </ul>
 * <p>========================================================================================</p>
 */
public class LimelightSubsystem {
    private Limelight3A limelight = null;

    // Configurable vertical angle threshold (degrees) to determine tip status
    private double tyTipThresholdDegrees = 4.0;

    public enum Pipeline {
        APRIL_TAG_HIVE(0),
        YELLOW_POLLEN(1),
        RED_NECTAR(2),
        BLUE_NECTAR(3),
        BLOB_DETECTION(1); // Alias for default blob

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
            // Hardware unconfigured or missing
            limelight = null;
            if (telemetry != null) {
                telemetry.addLine("Limelight Hardware Unconfigured / Missing");
            }
        }
    }

    /**
     * Returns true if the Limelight 3A camera hardware was successfully initialized.
     */
    public boolean isAvailable() {
        return limelight != null;
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
     * Returns a list of currently visible AprilTag fiducial results from the Limelight camera.
     *
     * @return List of {@link LLResultTypes.FiducialResult} objects, or an empty list if none detected.
     */
    public List<LLResultTypes.FiducialResult> getFiducialResults() {
        LLResult result = getLatestResult();
        if (result != null && result.isValid()) {
            return result.getFiducialResults();
        }
        return new ArrayList<>();
    }

    /**
     * Calculates the estimated Hive Tip state based on the average vertical angle (ty)
     * of all currently visible AprilTags.
     *
     * @return Current {@link HiveTipState} (NO_TAGS_VISIBLE, UPRIGHT, TIPPED_UP, or TIPPED_DOWN).
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

    /**
     * Returns true if the Hive is detected to be tipped either UP or DOWN.
     */
    public boolean isHiveTipped() {
        HiveTipState state = getHiveTipState();
        return state == HiveTipState.TIPPED_UP || state == HiveTipState.TIPPED_DOWN;
    }

    /**
     * Returns a list of currently detected color blob results.
     */
    public List<LLResultTypes.ColorResult> getColorResults() {
        LLResult result = getLatestResult();
        if (result != null && result.isValid()) {
            return result.getColorResults();
        }
        return new ArrayList<>();
    }

    /**
     * Automatically sets the Limelight pipeline according to the current target element and alliance.
     */
    public void setPipelineForTarget(GameState.TargetElement element) {
        GameState.TargetElement effective = (element == GameState.TargetElement.ALLIANCE_NECTAR) ?
                GameState.getEffectiveTargetElement() : element;

        switch (effective) {
            case RED_NECTAR:
                setPipeline(Pipeline.RED_NECTAR);
                break;
            case BLUE_NECTAR:
                setPipeline(Pipeline.BLUE_NECTAR);
                break;
            case YELLOW_POLLEN:
            default:
                setPipeline(Pipeline.YELLOW_POLLEN);
                break;
        }
    }

    /**
     * Outputs diagnostic telemetry for color blob / pollen & nectar detection.
     */
    public void printBlobTelemetry(Telemetry telemetry) {
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
            List<LLResultTypes.ColorResult> colorResults = result.getColorResults();
            telemetry.addData("Blob Target Acquired", seesBallBlob() ? "YES" : "NO");
            telemetry.addData("Detected Color Blobs", colorResults.size());
            telemetry.addData("Main Target Offsets", "tx: %.2f° | ty: %.2f° | Area: %.2f%%",
                    result.getTx(), result.getTy(), result.getTa());

            int count = 1;
            for (LLResultTypes.ColorResult cr : colorResults) {
                telemetry.addLine(String.format(Locale.US, " -> Blob #%d | tx: %.1f° | ty: %.1f° | Area: %.2f%%",
                        count++, cr.getTargetXDegrees(), cr.getTargetYDegrees(), cr.getTargetArea()));
            }
        } else {
            telemetry.addData("Limelight Target", "No color blob detected in view");
        }
    }

    /**
     * Returns true if a valid target or ball blob is currently in the camera's view.
     */
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
     * Outputs rich diagnostic telemetry for debugging Limelight AprilTag readings, 3D target pitch,
     * and Hive tip status on the FTC Driver Station.
     *
     * @param telemetry Telemetry instance from OpMode.
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
                telemetry.addLine(String.format(Locale.US, " -> Tag #%d | tx: %.1f° | ty: %.1f° | 3D Pitch: %.1f° | Area: %.2f%%",
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
