package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * RobotContainer holds all mechanism subsystems.
 */
public class RobotContainer {

    public final IntakeSubsystem intake = new IntakeSubsystem();
    public final LauncherSubsystem launcher = new LauncherSubsystem();
    public final TransferSubsystem transfer = new TransferSubsystem();
    public final SwivelSubsystem swivel = new SwivelSubsystem();
    public final LimelightSubsystem limelight = new LimelightSubsystem();

    public RobotContainer(HardwareMap hardwareMap) {
        intake.init(hardwareMap);
        launcher.init(hardwareMap);
        transfer.init(hardwareMap);
        swivel.init(hardwareMap);
        limelight.init(hardwareMap);
    }
}
