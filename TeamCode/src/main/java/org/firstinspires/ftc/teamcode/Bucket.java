package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Bucket {

    private static final double FLIPPED_POSITION = 0;
    private static final double NORMAL_POSITION = 0.67;
    private final Servo servo;

    public Bucket(HardwareMap hardwareMap) {
        servo = hardwareMap.get(Servo.class, "bucket");
    }

    public Action flip_bucket() {
        return new Action() {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                servo.setPosition(FLIPPED_POSITION);
                return false;
            }
        };
    }

    public Action reset_bucket() {
        return new Action() {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                servo.setPosition(NORMAL_POSITION);
                return false;
            }
        };
    }
}
