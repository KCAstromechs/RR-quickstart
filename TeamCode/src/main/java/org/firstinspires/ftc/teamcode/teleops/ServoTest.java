package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "ServoTest", group = "Testing")
public class ServoTest extends OpMode {

    private CRServo servo;

    @Override
    public void init() {
        servo = hardwareMap.get(CRServo.class, "intake");
        servo.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    @Override
    public void loop() {
        if (gamepad1.a) {
            servo.setPower(1);
        } else {
            servo.setPower(0);
        }

        telemetry.addData("Servo/intake setSpeed", servo.getPower());
        telemetry.update();
    }
}
