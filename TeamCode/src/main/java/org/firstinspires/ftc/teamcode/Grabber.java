package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Grabber {
    private final Servo servo;

    public Grabber(HardwareMap hardwareMap) {
        servo = hardwareMap.get(Servo.class, "grabber");
    }

    public Action close_grabber() {
        return new Action() {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    // RUN CODE HERE
                    servo.setPosition(.1);
                    initialized = true;
                }

                double pos = servo.getPosition();
                packet.put("Grabber Pos", pos);
                return pos < 10_000.0;
            }
        };
    }

    public Action open_grabber() {
        return new Action() {
            private  boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    // RUN CODE HERE
                    servo.setPosition(0);
                    initialized = true;
                }

                double pos = servo.getPosition();
                packet.put("Grabber Pos", pos);
                return pos < 10_000.0;
            }
        };
    }
}
