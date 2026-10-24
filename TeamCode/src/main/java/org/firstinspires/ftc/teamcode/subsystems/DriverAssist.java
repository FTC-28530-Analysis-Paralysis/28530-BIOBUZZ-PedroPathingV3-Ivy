package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.utils.Angle;

import org.firstinspires.ftc.teamcode.config.GameState;

import java.util.ArrayList;
import java.util.List;

/**
 * DriverAssist provides teleop drive utilities for PedroPathing 3, including:
 * 1. Non-linear joystick input response curves (squareInputWithSign) for precision control.
 * 2. Alliance-aware auto-targeting framework to automatically lock heading onto the nearest Hive.
 * 3. Support for Field-Centric and Robot-Centric drive modes with heading error correction.
 */
public class DriverAssist {

    public enum DriveMode {
        ROBOT_CENTRIC("Robot-Centric"),
        FIELD_CENTRIC("Field-Centric"),
        FC_TARGET_LOCK("Field-Centric (Hive Lock)"),
        RC_TARGET_LOCK("Robot-Centric (Hive Lock)");

        private final String name;
        DriveMode(String name) { this.name = name; }
        public String getName() { return name; }
    }

    private final Follower follower;
    private DriveMode currentMode = DriveMode.FIELD_CENTRIC;

    // Proportional gain for target locking. Tune for responsive but stable heading alignment.
    private double headingKp = 1.2;

    // Turning multiplier to scale manual joystick turning speed for finer control.
    private double turningMultiplier = 0.8;

    // Default Hive Target Poses (X, Y in inches, Heading in radians)
    private static final Pose RED_HIVE_1  = new Pose(57.0, 14.0, Math.toRadians(90.0));
    private static final Pose RED_HIVE_2  = new Pose(57.0, 130.0, Math.toRadians(270.0));
    private static final Pose BLUE_HIVE_1 = new Pose(87.0, 14.0, Math.toRadians(90.0));
    private static final Pose BLUE_HIVE_2 = new Pose(87.0, 130.0, Math.toRadians(270.0));

    private final List<Pose> customRedHives = new ArrayList<>();
    private final List<Pose> customBlueHives = new ArrayList<>();

    public DriverAssist(Follower follower) {
        this.follower = follower;
        
        // Add default hives
        customRedHives.add(RED_HIVE_1);
        customRedHives.add(RED_HIVE_2);
        customBlueHives.add(BLUE_HIVE_1);
        customBlueHives.add(BLUE_HIVE_2);
    }

    public void setMode(DriveMode mode) {
        this.currentMode = mode;
    }

    public DriveMode getMode() {
        return currentMode;
    }

    public void setHeadingKp(double headingKp) {
        this.headingKp = headingKp;
    }

    public double getHeadingKp() {
        return headingKp;
    }

    public void setTurningMultiplier(double turningMultiplier) {
        this.turningMultiplier = turningMultiplier;
    }

    public double getTurningMultiplier() {
        return turningMultiplier;
    }

    /**
     * Calculates the nearest Hive pose for the current Alliance selected in {@link GameState}.
     *
     * @return {@link Pose} of the closest Hive to the robot's current position.
     */
    public Pose getNearestHivePose() {
        Pose robotPose = follower.pose();
        List<Pose> candidateHives = (GameState.alliance == GameState.Alliance.RED) ? customRedHives : customBlueHives;

        Pose nearest = null;
        double minDistance = Double.MAX_VALUE;

        for (Pose hive : candidateHives) {
            double dx = hive.x() - robotPose.x();
            double dy = hive.y() - robotPose.y();
            double distance = Math.hypot(dx, dy);

            if (distance < minDistance) {
                minDistance = distance;
                nearest = hive;
            }
        }

        return nearest != null ? nearest : candidateHives.get(0);
    }

    /**
     * Calculates the target heading (in radians) required for the robot to face or align with
     * the specified Hive pose.
     *
     * @param targetHive Hive pose to target.
     * @return Target angle in radians.
     */
    public double getTargetHeading(Pose targetHive) {
        Pose robotPose = follower.pose();
        double dx = targetHive.x() - robotPose.x();
        double dy = targetHive.y() - robotPose.y();

        // Point directly at target Hive position if at a distance, or lock to target Hive heading
        if (Math.hypot(dx, dy) > 24.0) {
            return Math.atan2(dy, dx);
        } else {
            return targetHive.heading();
        }
    }

    /**
     * Updates the robot's drive powers using PedroPathing 3 according to joystick inputs and
     * active {@link DriveMode}.
     *
     * @param joyY    Forward/backward joystick input (-1.0 to +1.0).
     * @param joyX    Strafe left/right joystick input (-1.0 to +1.0).
     * @param joyTurn Turn joystick input (-1.0 to +1.0).
     */
    public void update(double joyY, double joyX, double joyTurn) {
        Pose robotPose = follower.pose();

        // Apply non-linear response curve for fine joystick control
        double forward = squareInputWithSign(joyY);
        double strafe  = squareInputWithSign(joyX);
        double turn    = squareInputWithSign(joyTurn) * turningMultiplier;

        boolean isFieldCentric = (currentMode == DriveMode.FIELD_CENTRIC || currentMode == DriveMode.FC_TARGET_LOCK);
        boolean isTargetLock   = (currentMode == DriveMode.FC_TARGET_LOCK || currentMode == DriveMode.RC_TARGET_LOCK);

        if (isTargetLock) {
            Pose targetHive = getNearestHivePose();
            double targetHeading = getTargetHeading(targetHive);
            double angleError = Angle.error(robotPose.heading(), targetHeading);

            // Proportional heading correction power
            turn = Math.max(-1.0, Math.min(1.0, angleError * headingKp));
        }

        // Drive PedroPathing 3 drivetrain
        follower.drivetrain.drive(new DrivePowers(forward, strafe, turn), isFieldCentric);
    }

    /**
     * Squares a joystick input while preserving its sign to provide fine control at lower
     * deflection angles while retaining maximum speed at full deflection.
     *
     * @param input Joystick input between -1.0 and +1.0.
     * @return Squared input with sign preserved.
     */
    public double squareInputWithSign(double input) {
        return Math.copySign(input * input, input);
    }
}
