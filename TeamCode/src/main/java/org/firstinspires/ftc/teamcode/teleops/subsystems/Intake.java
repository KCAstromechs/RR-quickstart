package org.firstinspires.ftc.teamcode.teleops.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class Intake {

    public static class Speeds {
        public double IN = 1.0;
        public double OUT = -1.0; // currently half
        public double OFF = 0.0;

    }

    public static Speeds speeds = new Speeds();

    private DcMotor intake;

    public void init(HardwareMap hardwareMap) {
        intake = hardwareMap.get(DcMotor.class, "intake");

        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        intake.setDirection(DcMotor.Direction.FORWARD);

        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    public void intake() {
        intake.setPower(speeds.IN);
    }
    public void outtake() {
        intake.setPower(speeds.OUT);
    }
    public void stop() {
        intake.setPower(speeds.OFF);
    }
}
