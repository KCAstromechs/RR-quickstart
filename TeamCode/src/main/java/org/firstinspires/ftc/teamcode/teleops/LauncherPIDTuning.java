package org.firstinspires.ftc.teamcode.teleops;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@Config
@TeleOp
public class LauncherPIDTuning extends OpMode {

    // for testing only
    private DcMotor intake = null;
    private DcMotor progression = null;
    // ----------

    private DcMotorEx motorLeft;
    private DcMotorEx motorRight;
    private double leftTicksPerRev;
    private double rightTicksPerRev; // google says 28 tpr

    public static class PARAMS {
        public double P = 145.0;
        public double F = 12.3681;

        public double highVel = 1000; // RPM
        public double lowVel = 800; // RPM

        public double targetVel = highVel;
    }

    public static PARAMS params = new PARAMS();

    double[] stepSizes = {10.0, 1.0, 0.1, 0.01, 0.001, 0.0001};

    int stepIndex = 1; // starts at 1.0 step size

    @Override
    public void init() {
        // init motors
        intake = hardwareMap.get(DcMotor.class, "intake"); // port 0 exp. hub
        progression = hardwareMap.get(DcMotor.class, "progression"); // port 3 exp. hub
        intake.setDirection(DcMotor.Direction.FORWARD);
        progression.setDirection(DcMotor.Direction.REVERSE);

        motorLeft = hardwareMap.get(DcMotorEx.class, "outtakeLeft");
        motorRight = hardwareMap.get(DcMotorEx.class, "outtakeRight");
        motorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // TODO reverse motors
        motorLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        motorRight.setDirection(DcMotorSimple.Direction.REVERSE);

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(params.P, 0, 0, params.F);
        motorLeft.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        motorRight.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        leftTicksPerRev = motorLeft.getMotorType().getTicksPerRev();
        rightTicksPerRev = motorRight.getMotorType().getTicksPerRev();

        telemetry.addLine("Init complete");
    }

    @Override
    public void loop() {
        // get all gamepad commands
        // set target vel
        // update telemetry

        if (gamepad1.yWasPressed()) {
            if (params.targetVel == params.highVel) {
                params.targetVel = params.lowVel;
            } else {
                params.targetVel = params.highVel;
            }
        }

        if (gamepad1.bWasPressed()) {
            stepIndex = (stepIndex + 1) % stepSizes.length;
        }

        if (gamepad1.dpadLeftWasPressed()) {
            params.F -= stepSizes[stepIndex];
        }

        if (gamepad1.dpadRightWasPressed()) {
            params.F += stepSizes[stepIndex];
        }

        if (gamepad1.dpadUpWasPressed()) {
            params.P += stepSizes[stepIndex];
        }

        if (gamepad1.dpadDownWasPressed()) {
            params.P -= stepSizes[stepIndex];
        }

        if (gamepad1.x || gamepad1.right_bumper) {
//            progression.setPower(1);
            intake.setPower(1);
            if (gamepad1.x) {
                progression.setPower(1);
            }
        } else {
            progression.setPower(0);
            intake.setPower(0);
        }

        // (re)set new PIDF coefficients
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(params.P, 0, 0, params.F);
        motorLeft.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        motorRight.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        // set velocities
        if (gamepad1.a) {
            motorLeft.setVelocity(params.targetVel);
            motorRight.setVelocity(params.targetVel);
        } else {
            motorLeft.setPower(0);
            motorRight.setPower(0);
        }
        double curLeftVel = motorLeft.getVelocity();
        double curRightVel = motorRight.getVelocity();
        double leftError = params.targetVel - curLeftVel;
        double rightError = params.targetVel - curRightVel;

        // telemetry
        telemetry.addData("Target Velocity", params.targetVel);
        telemetry.addLine("--------Current Velocities--------");
        telemetry.addData("Current Left Vel", curLeftVel);
        telemetry.addData("Current Right Vel", curRightVel);
        telemetry.addLine("--------------Errors--------------");
        telemetry.addData("Left Error", leftError);
        telemetry.addData("Right Error", rightError);
        telemetry.addLine("---------PIDF Coefficients--------");
        telemetry.addData("Tuning P", "%.4f (D-Pad U/D)", params.P);
        telemetry.addData("Tuning F", "%.4f (D-Pad L/R)", params.F);
        telemetry.addData("Step Size", "%.4f (B Button)", stepSizes[stepIndex]);
        telemetry.addLine("----------Other Controls----------");
        telemetry.addData("Change targetVel", "Press Y");
        telemetry.addData("Change Step Size", "Press B");
        telemetry.addData("Begin firing", "Press (and hold) A");
        telemetry.addData("Run Progression & Intake", "Press X");
        telemetry.addData("Run Intake Only", "Press Right Bumper");
        telemetry.addData("Left rps", (motorLeft.getVelocity() / leftTicksPerRev) * 60);
        telemetry.addData("Right rps", (motorRight.getVelocity() / rightTicksPerRev) * 60);
    }
}
