package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Wrist {
    private final Servo servo;

    public Wrist(HardwareMap hardwareMap) {
        servo = hardwareMap.get(Servo.class, "wrist");
    }

    public Action wrist_to_ground() {
        return new Action() {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    // RUN CODE HERE
                    servo.setPosition(0.75);
                    initialized = true;
                }

                double pos = servo.getPosition();
                packet.put("Wrist Pos", pos);
                return pos < 10_000.0;
            }
        };
    }

    public Action wrist_to_sub() {
        return new Action() {
            private boolean initialized = false;
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    servo.setPosition(0.9);
                    initialized = true;

                }

                double pos = servo.getPosition();
                packet.put("Wrist Pos", pos);
                return pos < 10_000.0;
            }
        };
    }

    public Action wrist_to_bucket() {
        return new Action() {
            private boolean initialized = false;
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    servo.setPosition(0.55);
                    initialized = false;

                }

                double pos = servo.getPosition();
                packet.put("Wrist Pos", pos);
                return pos < 10_000.0;
            }
        };
    }
}
