package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Grabber {

    public final double CLOSED_POSITION = 0.9;
    public final double OPEN_POSITION = 0.65;
    private final Servo servo;

    public Grabber(HardwareMap hardwareMap) {
        servo = hardwareMap.get(Servo.class, "grabber");
    }

    public Action close_grabber() {
        return new Action() {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                servo.setPosition(CLOSED_POSITION);
                return false;
            }
        };
    }

    public Action open_grabber() {
        return new Action() {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                servo.setPosition(OPEN_POSITION);
                return false;
            }
        };
    }
}
