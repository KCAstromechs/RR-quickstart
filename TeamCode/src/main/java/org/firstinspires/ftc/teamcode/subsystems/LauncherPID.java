package org.firstinspires.ftc.teamcode.subsystems;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@Config
public class LauncherPID {
    public static class PARAMS {
        public double P = 145.0;
        public double F = 12.3681;

//        public double highVel = 1000; // RPM
//        public double lowVel = 800; // RPM
        public double defaultVel = 800; // RPM
        public double toleranceVel = 20; // RPM

        public double targetVel = defaultVel;
    }

    // params
    public static PARAMS params = new PARAMS();

    // motors
    private DcMotorEx outtakeLeft = null;
    private DcMotorEx outtakeRight = null;

    // state
    public enum LauncherState {
        SPIN_UP,
        LAUNCHING,
        RESPIN_UP,
        OFF
    }
    private LauncherState launcherState;

    // other vars
    private double curLeftVel;
    private double curRightVel;
    private double distFromGoal;

    public void init(HardwareMap hardwareMap) {
        outtakeLeft = hardwareMap.get(DcMotorEx.class, "outtakeLeft"); // port 2 exp. hub
        outtakeRight = hardwareMap.get(DcMotorEx.class, "outtakeRight"); // port 1 exp. hub
        outtakeLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        outtakeRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        outtakeLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        outtakeRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        outtakeLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        outtakeRight.setDirection(DcMotorSimple.Direction.REVERSE);

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(params.P, 0, 0, params.F);
        outtakeLeft.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        outtakeRight.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
    }

    // No need for inc/dec defaultVel (FOR NOW)

    /**
     * Update Launcher with distance calculations aprilTagDetection isn't null
     * @param aprilTagDetection AprilTagDetection obj representing the aprilTag to get dist from
     */
    public void updateLauncher(AprilTagDetection aprilTagDetection) {
        // update other vars
        if (aprilTagDetection != null) {
            distFromGoal = aprilTagDetection.ftcPose.range;
            params.targetVel = calculateTargetVel(distFromGoal);
        } else {
            params.targetVel = params.defaultVel;
        }
        curLeftVel = outtakeLeft.getVelocity();
        curRightVel = outtakeRight.getVelocity();
        switch (launcherState) {
            case SPIN_UP:
                outtakeLeft.setVelocity(params.targetVel);
                outtakeRight.setVelocity(params.targetVel);
                // if curVel is within toleranceVel, switch to launching
                if (velWithinTolerance()) {
                    launcherState = LauncherState.LAUNCHING;
                }
                break;
            case LAUNCHING:
                outtakeLeft.setVelocity(params.targetVel);
                outtakeRight.setVelocity(params.targetVel);
                // if curVel is no long within toleranceVel, switch back to spin_up
                if (!velWithinTolerance()) {
                    launcherState = LauncherState.RESPIN_UP;
                }
                break;
            case RESPIN_UP:
                outtakeLeft.setVelocity(params.targetVel);
                outtakeRight.setVelocity(params.targetVel);
                // if curVel is within toleranceVel, switch to launching
                if (velWithinTolerance()) {
                    launcherState = LauncherState.LAUNCHING;
                }
                break;
            case OFF:
                outtakeLeft.setVelocity(0);
                outtakeRight.setVelocity(0);
                break;
            default:
                break;
        }
    }

    public void startLauncher() {
        if (launcherState == LauncherState.OFF) {
            launcherState = LauncherState.SPIN_UP;
        }
    }

    public void stopLauncher() {
        launcherState = LauncherState.OFF;
    }

    public LauncherState getLauncherState() {
        return launcherState;
    }

    public void displayTelemetry(Telemetry telemetry) {
        telemetry.addLine("Launcher Telemetry:");
        telemetry.addData("Target Velocity", params.targetVel);
        telemetry.addLine("--------Current Velocities--------");
        telemetry.addData("Current Left Vel", curLeftVel);
        telemetry.addData("Current Right Vel", curRightVel);
        telemetry.addData("Last AprilTag Range (dist from goal)", distFromGoal);
    }

    private double calculateTargetVel(double range) {
        // TODO
        double targetVel = 0;
        // do stuff
        return targetVel;
    }

    private boolean velWithinTolerance() {
        boolean rightWithinRange = (curRightVel > params.targetVel - params.toleranceVel && curRightVel < params.targetVel + params.toleranceVel);
        boolean leftWithinRange = (curLeftVel > params.targetVel - params.toleranceVel && curLeftVel < params.targetVel + params.toleranceVel);
        return rightWithinRange && leftWithinRange;
    }
}
