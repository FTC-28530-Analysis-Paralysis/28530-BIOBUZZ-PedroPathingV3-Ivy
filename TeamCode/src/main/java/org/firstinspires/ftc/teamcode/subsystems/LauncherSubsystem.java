package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.ivy.Command;
import static com.pedropathing.ivy.commands.Commands.*;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Subsystem for flywheel launcher mechanism with Ivy Commands using velocity control.
 */
public class LauncherSubsystem {
    private DcMotorEx leftFlywheel = null;
    private DcMotorEx rightFlywheel = null;

    public static double STARTING_SPEED = 1040; // starting velocity (RPM / ticks per second)
    public static double SPEED_INCREMENT = 50;  // speed adjustment step
    public static double DEFAULT_FEED_DURATION = 3.0; // global default launch feed duration in seconds
    public static final double RPM_TOLERANCE = 25.0; // velocity error tolerance

    public double targetSpeed = STARTING_SPEED;
    private boolean isSpinningUp = false;

    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        try {
            leftFlywheel = hardwareMap.get(DcMotorEx.class, "left_flywheel");
            rightFlywheel = hardwareMap.get(DcMotorEx.class, "right_flywheel");
            leftFlywheel.setDirection(DcMotor.Direction.REVERSE);
            rightFlywheel.setDirection(DcMotor.Direction.FORWARD);
            leftFlywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFlywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        } catch (Exception e) {
            // Hardware unconfigured
            if (telemetry != null) {
                telemetry.addLine("Launcher Hardware not configured");
            }
        }
    }

    public void setVelocity(double velocity) {
        if (leftFlywheel != null) leftFlywheel.setVelocity(velocity);
        if (rightFlywheel != null) rightFlywheel.setVelocity(velocity);
    }

    public void spinUp() {
        isSpinningUp = true;
        setVelocity(targetSpeed);
    }

    public void stop() {
        isSpinningUp = false;
        setVelocity(0.0);
    }

    public void increaseSpeed() {
        targetSpeed += SPEED_INCREMENT;
        if (isSpinningUp) {
            spinUp();
        }
    }

    public void decreaseSpeed() {
        targetSpeed = Math.max(0, targetSpeed - SPEED_INCREMENT);
        if (isSpinningUp) {
            spinUp();
        }
    }

    /**
     * Checks whether both flywheels are within RPM_TOLERANCE of targetSpeed.
     */
    public boolean isAtTargetSpeed() {
        if (leftFlywheel == null || rightFlywheel == null) return true;
        double leftVel = Math.abs(leftFlywheel.getVelocity());
        double rightVel = Math.abs(rightFlywheel.getVelocity());
        double target = Math.abs(targetSpeed);
        return Math.abs(leftVel - target) <= RPM_TOLERANCE && Math.abs(rightVel - target) <= RPM_TOLERANCE;
    }

    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Launcher Target Velocity", targetSpeed);
        if (leftFlywheel != null && rightFlywheel != null) {
            telemetry.addData("Launcher Actual Velocity (L/R)", "%.1f / %.1f",
                    leftFlywheel.getVelocity(), rightFlywheel.getVelocity());
            telemetry.addData("Flywheel Target Ready", isAtTargetSpeed() ? "YES" : "NO");
        }
    }

    // Ivy Commands
    public Command spinUpCommand() {
        return instant(this::spinUp);
    }

    public Command stopCommand() {
        return instant(this::stop);
    }
}
