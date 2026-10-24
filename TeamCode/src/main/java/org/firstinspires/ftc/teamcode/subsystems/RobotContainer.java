package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * RobotContainer holds all mechanism subsystems.
 */
public class RobotContainer {

    public final IntakeSubsystem intake = new IntakeSubsystem();
    public final LauncherSubsystem launcher = new LauncherSubsystem();
    public final TransferSubsystem transfer = new TransferSubsystem();
    public final SwivelSubsystem swivel = new SwivelSubsystem();
    public final LimelightSubsystem limelight = new LimelightSubsystem();

    public RobotContainer(HardwareMap hardwareMap, Telemetry telemetry) {
        intake.init(hardwareMap, telemetry);
        launcher.init(hardwareMap, telemetry);
        transfer.init(hardwareMap, telemetry);
        swivel.init(hardwareMap, telemetry);
        limelight.init(hardwareMap, telemetry);
    }
}
