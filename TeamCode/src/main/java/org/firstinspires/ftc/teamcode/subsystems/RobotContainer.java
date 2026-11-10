package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.ivy.Command;
import static com.pedropathing.ivy.commands.Commands.*;
import static com.pedropathing.ivy.groups.Groups.*;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * RobotContainer holds all mechanism subsystems and orchestrates multi-subsystem commands.
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

    /**
     * Complete multi-subsystem launch sequence combining Launcher, Transfer, and Intake.
     * Uses global default feed duration defined in LauncherSubsystem.
     */
    public Command launchSequenceCommand() {
        return launchSequenceCommand(LauncherSubsystem.DEFAULT_FEED_DURATION);
    }

    /**
     * Complete multi-subsystem launch sequence combining Launcher, Transfer, and Intake.
     * Uses pulsed/burst feeding (0.5s pulses) to verify flywheel speed recovery between bursts.
     *
     * @param totalDurationSeconds Total duration for the launching sequence.
     */
    public Command launchSequenceCommand(double totalDurationSeconds) {
        double pulseDuration = 0.5; // Duration of each burst pulse in seconds
        int pulseCount = Math.max(1, (int) Math.ceil(totalDurationSeconds / pulseDuration));

        Command[] pulses = new Command[pulseCount];
        for (int i = 0; i < pulseCount; i++) {
            pulses[i] = sequential(
                    waitUntil(launcher::isAtTargetSpeed),
                    parallel(
                            transfer.feedCommand(),
                            intake.startCommand()
                    ),
                    waitMs(pulseDuration * 1000.0),
                    transfer.stopCommand(),
                    intake.stopCommand()
            );
        }

        return sequential(
                launcher.spinUpCommand(),
                sequential(pulses),
                transfer.stopCommand(),
                intake.stopCommand()
        );
    }
}
