package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Bucket {
    private final Servo servo;

    public Bucket(HardwareMap hardwareMap) {
        servo = hardwareMap.get(Servo.class, "bucket");
    }

    public Action flip_bucket() {
        return new Action() {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    // RUN CODE HERE
                    servo.setPosition(0);
                    initialized = true;
                }

                double pos = servo.getPosition();
                packet.put("Bucket Pos", pos);
                return pos < 10_000.0;
            }
        };
    }

    public Action reset_bucket() {
        return new Action() {
            private boolean initialized = false;
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    servo.setPosition(0);
                    initialized = true;

                }

                double pos = servo.getPosition();
                packet.put("Bucket Pos", pos);
                return pos < 10_000.0;
            }
        };
    }
}
