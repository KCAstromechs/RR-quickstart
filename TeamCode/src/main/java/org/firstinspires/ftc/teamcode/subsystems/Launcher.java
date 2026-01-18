package org.firstinspires.ftc.teamcode.subsystems;


import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad2;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class Launcher {

    public static class Params {
        public double closeAODTolerance = 1;
        public double farAODTolerance = 0.05;
        public double AODTolerance = closeAODTolerance;
        public double defaultShooterPercent = 20; // 100 = 100%
        public double defaultMinRPS = 30; // originally 95 RPM before 10/22/2025
        public double shooterPercent = defaultShooterPercent;
        public double minRPS = defaultMinRPS;
    }

    public static Params params = new Params();

    private DcMotorEx outtakeLeft = null;
    private DcMotorEx outtakeRight = null;

    private double leftTicksPerRev;
    private double rightTicksPerRev;
    private double leftRPS;
    private double rightRPS;
    private boolean shooting = false;
    private boolean canShoot = false;

    public void init(HardwareMap hardwareMap) {
        outtakeLeft = hardwareMap.get(DcMotorEx.class, "outtakeLeft"); // port 2 exp. hub
        outtakeRight = hardwareMap.get(DcMotorEx.class, "outtakeRight"); // port 1 exp. hub

        leftTicksPerRev = outtakeLeft.getMotorType().getTicksPerRev();
        rightTicksPerRev = outtakeRight.getMotorType().getTicksPerRev();
        outtakeLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        outtakeRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

//        // Speed stuff
        outtakeLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outtakeRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outtakeLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        outtakeRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

//        outtakeLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        outtakeRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftRPS = (outtakeLeft.getVelocity() / leftTicksPerRev) * 60;
        rightRPS = (outtakeRight.getVelocity() / rightTicksPerRev) * 60 * -1;
        shooting = gamepad2.right_trigger > 0.5;
        canShoot = (leftRPS > params.minRPS && rightRPS > params.minRPS);

        // shooter buttons
        if (gamepad2.dpadDownWasPressed()) {
            params.shooterPercent -= .01; // -5%
        } else if (gamepad2.dpadUpWasPressed()) {
            params.shooterPercent += .01; // +5%
        }
        // minRPM buttons
        if (gamepad2.dpadRightWasPressed()) {
            params.minRPS += 1;
        } else if (gamepad2.dpadLeftWasPressed()) {
            params.minRPS -= 1;
        }

        // outtake
        outtakeLeft.setPower(gamepad2.right_trigger * (params.shooterPercent * .01));
        outtakeRight.setPower(-gamepad2.right_trigger * (params.shooterPercent * .01));

        // Distance-based shooting power
        if (idRed == null && idBlue == null) {
            params.shooterPercent = params.defaultShooterPercent;
            params.minRPS = params.defaultMinRPS;
        } else if (idRed != null){
            // math
            if (idRed.ftcPose.range < 200) {
                params.shooterPercent = 19.10442 - (-0.003401434 / -0.01617827) * (1 - Math.pow(Math.E, 0.01617827 * (idRed.ftcPose.range))); //TODO: Replace with updated equation
                params.minRPS = params.shooterPercent + 10; //TODO: Give minRPS its own equation
            } else {
                params.shooterPercent = 30;
                params.minRPS = 40;
            }
        } else { // if blue
            if (idBlue.ftcPose.range < 200) {
                params.shooterPercent = 19.10442 - (-0.003401434 / -0.01617827) * (1 - Math.pow(Math.E, 0.01617827 * (idBlue.ftcPose.range))); //TODO: Replace with updated equation
                params.minRPS = params.shooterPercent + 10; //TODO: Give minRPS its own equation
            } else {
                params.shooterPercent = 30;
                params.minRPS = 40;
            }
        }

    }
}