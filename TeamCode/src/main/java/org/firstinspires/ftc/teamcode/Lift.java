package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Lift {
    private DcMotor lift;

    public Lift(HardwareMap hardwareMap) {
        // Init the lift
        lift = hardwareMap.get(DcMotor.class, "lift");
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    public Action lower_lift() {
        return new Action() {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    // RUN CODE HERE
                    // TODO change 0 to 'low' lift position
                    if (lift.getCurrentPosition() > 0) {
                        while (lift.getCurrentPosition() > 0) {
                            lift.setPower(-0.5);
                        }
                        lift.setPower(0);
                    } else if (lift.getCurrentPosition() < 0) {
                        while (lift.getCurrentPosition() < 0) {
                            lift.setPower(0.5);
                        }
                        lift.setPower(0);
                    }
                    initialized = true;
                }

                double pos = lift.getCurrentPosition();
                packet.put("Lift Pos", pos);
                return pos < 10_000.0;
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
                    // TODO change 0 to 'raised' lift position
                    if (lift.getCurrentPosition() > 1350) {
                        while (lift.getCurrentPosition() > 1350) {
                            lift.setPower(-0.5);
                        }
                        lift.setPower(0);
                    } else if (lift.getCurrentPosition() < 1350) {
                        while (lift.getCurrentPosition() < 1350) {
                            lift.setPower(0.5);
                        }
                        lift.setPower(0);
                    }
                    initialized = true;
                }

                double pos = lift.getCurrentPosition();
                packet.put("Lift Pos", pos);
                return pos < 10_000.0;
            }
        };
    }
}
