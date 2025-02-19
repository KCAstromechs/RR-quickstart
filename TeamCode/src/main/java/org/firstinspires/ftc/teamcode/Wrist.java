package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Wrist {

    public static class Params {
        public double groundPosition = 0.75;
        public double subPosition = 0.9;
        public double bucketPosition = 0.55;
    }

    public static Params PARAMS = new Params();
    private final Servo servo;

    public Wrist(HardwareMap hardwareMap) {
        servo = hardwareMap.get(Servo.class, "wrist");
    }

    public Action wrist_to_ground() {
        return new Action() {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                servo.setPosition(PARAMS.groundPosition);
                return false;
            }
        };
    }

    public Action wrist_to_sub() {
        return new Action() {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                servo.setPosition(PARAMS.subPosition);
                return false;
            }
        };
    }

    public Action wrist_to_bucket() {
        return new Action() {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                servo.setPosition(PARAMS.bucketPosition);
                return false;
            }
        };
    }
}
