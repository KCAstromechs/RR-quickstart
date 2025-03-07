package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Grabber {

    public static class Params {
        public double closedPosition = 0.9;
        public double openPosition = 0.60;
    }

    public static Params PARAMS = new Params();
    private final Servo servo;

    public Grabber(HardwareMap hardwareMap) {
        servo = hardwareMap.get(Servo.class, "grabber");
    }

    public Action close_grabber() {
        return new Action() {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                servo.setPosition(PARAMS.closedPosition);
                return false;
            }
        };
    }

    public Action open_grabber() {
        return new Action() {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                servo.setPosition(PARAMS.openPosition);
                return false;
            }
        };
    }
}
