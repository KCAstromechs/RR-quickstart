package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Lift {

    private final double DOWN_POSITION = 50;
    private final double UP_POSITION = 4000;
    private final double SPEED_LIMIT_DOWN = 0.5; // 50% power/speed
    private final double SPEED_LIMIT_UP = 1; // 100% power/speed
    private DcMotor motor;

    public Lift(HardwareMap hardwareMap) {
        // Init the lift
        motor = hardwareMap.get(DcMotor.class, "lift");
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    public Action lower_lift() {
        return new Action() {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    // RUN CODE HERE
                    motor.setPower(-SPEED_LIMIT_DOWN);
                    initialized = true;
                }

                double pos = motor.getCurrentPosition();
                packet.put("Lift Pos", pos);
                if (pos < DOWN_POSITION) {
                    motor.setPower(0);
                    return false;
                }
                else {
                    return true;
                }
            }
        };
    }

    public Action raise_lift() {
        return new Action() {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    // RUN CODE HERE
                    motor.setPower(SPEED_LIMIT_UP);
                    initialized = true;
                }

                double pos = motor.getCurrentPosition();
                packet.put("Lift Pos", pos);
                if (pos > UP_POSITION) {
                    motor.setPower(0);
                    return false;
                }
                else {
                    return true;
                }
            }
        };
    }
}
