package org.firstinspires.ftc.teamcode.teleops.subsystems;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class Launcher {

    public static class PARAMS {
        private final double FEED_TIME_SECONDS = 0.20;

        private final double STOP_SPEED = 0.0;

        private final double FULL_SPEED = 0.0;

        private final double LAUNCHER_TARGET_VELOCITY = 1125;

        private final double LAUNCHER_MIN_VELOCITY = 1075;

    }
    private PARAMS params = new PARAMS();

    private DcMotorEx launcher;

    private CRServo leftFeeder, rightFeeder;

    ElapsedTime feederTimer = new ElapsedTime();

    private enum LauncherState {
        IDLE,
        SPIN_UP,
        LAUNCH,
        LAUNCHING,
    }

    private LauncherState launcherState;

    public void init(HardwareMap hardwareMap) {
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        leftFeeder = hardwareMap.get(CRServo.class, "left_feeder");
        rightFeeder = hardwareMap.get(CRServo.class, "right_feeder");

        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(
                300, 0, 0, 10
        ));

        leftFeeder.setDirection(CRServo.Direction.REVERSE);

        launcherState = LauncherState.IDLE;
        stopFeeder();
        stopLauncher();

    }

    public void stopFeeder() {
        leftFeeder.setPower(params.STOP_SPEED);
        rightFeeder.setPower(params.STOP_SPEED);
    }

    public void updateLauncher() {
        switch (launcherState) {
            case IDLE:
                break;
            case SPIN_UP:
                launcher.setVelocity(params.LAUNCHER_TARGET_VELOCITY);
                if (launcher.getVelocity() >= params.LAUNCHER_MIN_VELOCITY) {
                    //transiton states
                    launcherState = LauncherState.LAUNCH;
                }

                break;
            case LAUNCH:
                leftFeeder.setPower(params.FULL_SPEED);
                rightFeeder.setPower(params.FULL_SPEED);
                feederTimer.reset();
                // transition
                launcherState = LauncherState.LAUNCHING;
                break;
            case LAUNCHING:
                if (feederTimer.seconds() > params.FEED_TIME_SECONDS) {
                    stopFeeder();
                    // transition event
                    launcherState = LauncherState.IDLE;
                }
                break;
        }
    }

    public void startLauncher() {
        if (launcherState == LauncherState.IDLE) {
            //transition sattes
            launcherState = LauncherState.SPIN_UP;
        }
    }

    public void stopLauncher() {
        stopFeeder();
        launcher.setVelocity(params.STOP_SPEED);
        launcherState = LauncherState.IDLE;
    }

    public String getState() {
        return launcherState.toString();
    }

    public double getVelocity() {
        return launcher.getVelocity();
    }

    public void displayTelemetry(Telemetry telemetry) {
        telemetry.addLine("Launcher Telemetry:");
        telemetry.addData("Motor Velocity", launcher.getVelocity());
        telemetry.addData("Launcher State", getState());
    }
}