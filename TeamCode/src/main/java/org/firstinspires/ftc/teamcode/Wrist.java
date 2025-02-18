package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Wrist {

    private final double GROUND_POSITION = 0.75;
    private final double SUB_POSITION = 0.9;
    private final double BUCKET_POSITION = 0.55;
    private final Servo servo;

    public Wrist(HardwareMap hardwareMap) {
        servo = hardwareMap.get(Servo.class, "wrist");
    }

    public Action wrist_to_ground() {
        return new Action() {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                servo.setPosition(GROUND_POSITION);
                return false;
            }
        };
    }

    public Action wrist_to_sub() {
        return new Action() {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                servo.setPosition(SUB_POSITION);
                return false;
            }
        };
    }

    public Action wrist_to_bucket() {
        return new Action() {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                servo.setPosition(BUCKET_POSITION);
                return false;
            }
        };
    }
}
