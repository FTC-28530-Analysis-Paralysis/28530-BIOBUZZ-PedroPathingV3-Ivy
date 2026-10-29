package org.firstinspires.ftc.teamcode.routines;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.subsystems.RobotContainer;

/**
 * Interface for autonomous routines combining PedroPathing follower paths
 * with Ivy mechanism subsystem commands.
 */
public interface AutoRoutine {

    /**
     * User-friendly display name of the routine.
     */
    String getName();

    /**
     * Starting Pose of the robot on the field.
     */
    Pose getStartPose();

    /**
     * Constructs the full Ivy Command tree for execution.
     *
     * @param follower          PedroPathing Follower instance.
     * @param robot             RobotContainer containing mechanism subsystems.
     * @param visionEnabled     Whether Limelight vision interrupts are enabled.
     * @param startDelaySec     Start delay in seconds.
     * @return Fully built Ivy Command sequence.
     */
    Command buildCommand(Follower follower, RobotContainer robot, boolean visionEnabled, double startDelaySec);
}
