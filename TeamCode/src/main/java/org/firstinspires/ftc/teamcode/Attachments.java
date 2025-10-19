package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Attachments {
    // INSTANCE VARS
    // Shooters
    private DcMotorEx outtakeLeft;
    private double leftRPM;
    private double leftTPR;
    private DcMotorEx outtakeRight;
    private double rightRPM;
    private double rightTPR;
    private boolean canShoot;


    // Progression & intake
    private DcMotorEx progression;
    private DcMotorEx intake;

    // THE ONLY CONSTRUCTOR
    public Attachments(HardwareMap hardwareMap) {
        // initialize shooters
        outtakeLeft = hardwareMap.get(DcMotorEx.class, "outtakeLeft");
        outtakeLeft.setDirection(DcMotorEx.Direction.FORWARD); // TODO change direction if needed
        outtakeLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outtakeLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftTPR = outtakeLeft.getMotorType().getTicksPerRev();

        outtakeRight = hardwareMap.get(DcMotorEx.class, "outtakeRight");
        outtakeRight.setDirection(DcMotorEx.Direction.FORWARD); // TODO change direction if needed
        outtakeRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outtakeRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightTPR = outtakeRight.getMotorType().getTicksPerRev();

        // init progression and intake
        progression = hardwareMap.get(DcMotorEx.class, "progression");
        progression.setDirection(DcMotorEx.Direction.REVERSE);

        intake = hardwareMap.get(DcMotorEx.class, "intake");
        intake.setDirection(DcMotorEx.Direction.FORWARD);
    }

    /** Spins up shooter motors and ends once motors reach targetRPM
     *
     * @param targetRPM the desired RPM for shooters to spin up before firing(recommended for m1: 95 RPM)
     * @return
     */
    public Action spinUp(double targetRPM) {
        return new Action() {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    outtakeLeft.setPower(0.7);
                    outtakeRight.setPower(0.7);
                    // TODO change powers to 0.8?
                    initialized = true;
                }
                leftRPM = outtakeLeft.getVelocity() / leftTPR;
                rightRPM = outtakeRight.getVelocity() / rightTPR;
                canShoot = (leftRPM > targetRPM && rightRPM > targetRPM);

                packet.put("Left RPM", leftRPM);
                packet.put("Right RPM", rightRPM);
                return (!canShoot); // return false if canShoot
                // return true to continue action, false to end action
            }
        };
    }

    /** Fires one artifact for s seconds
     *
     * @param durationSeconds (double) number of seconds you want to continuously shoot
     * @return
     */
    public Action fireArtifact(double durationSeconds, double targetRPM) {
        return new Action() {
            private boolean initialized = false;
            private ElapsedTime timer = new ElapsedTime();

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    // continue outtaking
                    outtakeLeft.setPower(0.7);
                    outtakeRight.setPower(0.7);
                    // TODO change powers to 0.8?

                    // reset/start timer
                    timer.reset();
                    initialized = true;
                }

                // constantly update progression & intake
                canShoot = (leftRPM > targetRPM && rightRPM > targetRPM);
                if (canShoot) {
                    progression.setPower(1);
                    intake.setPower(1);
                } else {
                    progression.setPower(0);
                    intake.setPower(0);
                }

                double elapsed = timer.seconds();
                packet.put("Shooting timer", elapsed); // telemtry

                if (elapsed >= durationSeconds) {
                    // stop all motors
                    progression.setPower(0);
                    intake.setPower(0);

                    outtakeLeft.setPower(0);
                    outtakeRight.setPower(0);

                    return false; // action finished
                }

                return true; // keep running until timer reaches target time
            }
        };
    }

    // TODO add intake Action
}
