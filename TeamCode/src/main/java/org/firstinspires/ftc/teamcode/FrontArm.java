package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class FrontArm {

    private final double DOWN_POSITION = 290;
    private final double UP_POSITION = 50;

    private final double SPEED_LIMIT = 0.5; // 50% power/speed
    private DcMotor motor;

    public FrontArm(HardwareMap hardwareMap) {
        // Init the lift
        motor = hardwareMap.get(DcMotor.class, "lift");
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    public Action lowerArm() {
        return new Action() {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    // RUN CODE HERE
                    motor.setPower(SPEED_LIMIT);
                    initialized = true;
                }

                double pos = motor.getCurrentPosition();
                packet.put("Lift Pos", pos);
                if (pos > DOWN_POSITION) {
                    motor.setPower(0);
                    return false;
                }
                else {
                    return true;
                }
            }
        };
    }

    public Action raiseArm() {
        return new Action() {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    // RUN CODE HERE
                    motor.setPower(-SPEED_LIMIT);
                    initialized = true;
                }

                double pos = motor.getCurrentPosition();
                packet.put("Lift Pos", pos);
                if (pos < UP_POSITION) {
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
