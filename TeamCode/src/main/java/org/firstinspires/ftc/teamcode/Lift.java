package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class Lift {

    public static class Params {

        public double upPosition = 4000;
        public double downPosition = 50;
        public double speedLimitDown = 0.5; // 50% power/speed
        public double speedLimitUp = 1; // 100% power/speed

    }

    public static Params PARAMS = new Params();
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
                    motor.setPower(-PARAMS.speedLimitDown);
                    initialized = true;
                }

                double pos = motor.getCurrentPosition();
                packet.put("Lift Pos", pos);
                if (pos < PARAMS.downPosition) {
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
                    motor.setPower(PARAMS.speedLimitUp);
                    initialized = true;
                }

                double pos = motor.getCurrentPosition();
                packet.put("Lift Pos", pos);
                if (pos > PARAMS.upPosition) {
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
