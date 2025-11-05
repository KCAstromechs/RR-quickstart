package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "ShooterTest", group = "Testing")
public class ShooterTest extends LinearOpMode {

    // Declare OpMode Members.
    private DcMotorEx shooter;

    private double ticksPerRev;
    private double shooterPercent = 1; // 1.0 = 100%
    private double minRPM;
    private double shooterRPM;
    private boolean shooting;
    private boolean canShoot;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");

        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        ticksPerRev = shooter.getMotorType().getTicksPerRev();

        shooter.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        shooter.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            shooter.setPower(gamepad1.right_trigger * shooterPercent);

            // shooter buttons
            if (gamepad1.dpadDownWasPressed()) {
                shooterPercent -= .05; // -5%
            } else if (gamepad1.dpadUpWasPressed()) {
                shooterPercent += .05; // +5%
            }

            telemetry.addData("Shooter %", shooterPercent * 100); // note: not rounded at all
        }
    }
}
