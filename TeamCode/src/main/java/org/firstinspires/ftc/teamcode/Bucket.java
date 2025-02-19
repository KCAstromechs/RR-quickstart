package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Bucket {

    public static class Params {
        public double flippedPosition = 0;
        public double normalPosition = 0.67;

    }

    public static Params PARAMS = new Params();

    private final Servo servo;
    public Bucket(HardwareMap hardwareMap) {
        servo = hardwareMap.get(Servo.class, "bucket");
    }

    public Action flip_bucket() {
        return new Action() {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                servo.setPosition(PARAMS.flippedPosition);
                return false;
            }
        };
    }

    public Action reset_bucket() {
        return new Action() {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                servo.setPosition(PARAMS.normalPosition);
                return false;
            }
        };
    }
}
