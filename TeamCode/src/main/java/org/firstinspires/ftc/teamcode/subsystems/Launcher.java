package org.firstinspires.ftc.teamcode.subsystems;


import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad2;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

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
    }

    public void decreaseShooterPercent() {
        params.shooterPercent -= .01; // -1%
    }
    public void increaseShooterPercent() {
        params.shooterPercent += .01; // +1%
    }
    public void increaseMinRPS() {
        params.minRPS += 1;
    }
    public void decreaseMinRPS() {
        params.minRPS -= 1;
    }

    /**
     * shoots balls
     * @param shooterMagnitude the magnitude of power (scaled down by shooterPercent) --> value between [0.0, 1.0]
     */
    public void shoot(double shooterMagnitude, AprilTagDetection tagDetection) {

        leftRPS = (outtakeLeft.getVelocity() / leftTicksPerRev) * 60;
        rightRPS = (outtakeRight.getVelocity() / rightTicksPerRev) * 60 * -1;
        shooting = gamepad2.right_trigger > 0.5;
        canShoot = (leftRPS > params.minRPS && rightRPS > params.minRPS);

        // outtake
        outtakeLeft.setPower(gamepad2.right_trigger * (params.shooterPercent * .01));
        outtakeRight.setPower(-gamepad2.right_trigger * (params.shooterPercent * .01));

        // Distance-based shooting power
        if (tagDetection == null) {
            params.shooterPercent = params.defaultShooterPercent;
            params.minRPS = params.defaultMinRPS;
        } else {
            // math
            if (tagDetection.ftcPose.range < 200) {
                params.shooterPercent = 19.10442 - (-0.003401434 / -0.01617827) * (1 - Math.pow(Math.E, 0.01617827 * (tagDetection.ftcPose.range))); //TODO: Replace with updated equation
                params.minRPS = params.shooterPercent + 10; //TODO: Give minRPS its own equation
            } else {
                params.shooterPercent = 30;
                params.minRPS = 40;
            }
        }
    }
}