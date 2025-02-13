package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class FrontArm {
    private DcMotor motor;

    public FrontArm(HardwareMap hardwareMap) {
        // Init the lift
        motor = hardwareMap.get(DcMotor.class, "lift");
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    public Action lower_arm() {
        return new Action() {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    // RUN CODE HERE
                    // TODO change 0 to 'low' frontArm position
                    if (motor.getCurrentPosition() > 0) {
                        while (motor.getCurrentPosition() > 0) {
                            motor.setPower(-0.5);
                        }
                        motor.setPower(0);
                    } else if (motor.getCurrentPosition() < 0) {
                        while (motor.getCurrentPosition() < 0) {
                            motor.setPower(0.5);
                        }
                        motor.setPower(0);
                    }
                    initialized = true;
                }

                double pos = motor.getCurrentPosition();
                packet.put("Lift Pos", pos);
                return pos < 10_000.0;
            }
        };
    }

    public Action raise_arm() {
        return new Action() {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    // RUN CODE HERE
                    // TODO change 0 to 'raised' frontArm position
                    if (motor.getCurrentPosition() > 1350) {
                        while (motor.getCurrentPosition() > 1350) {
                            motor.setPower(-0.5);
                        }
                        motor.setPower(0);
                    } else if (motor.getCurrentPosition() < 1350) {
                        while (motor.getCurrentPosition() < 1350) {
                            motor.setPower(0.5);
                        }
                        motor.setPower(0);
                    }
                    initialized = true;
                }

                double pos = motor.getCurrentPosition();
                packet.put("Lift Pos", pos);
                return pos < 10_000.0;
            }
        };
    }
}
