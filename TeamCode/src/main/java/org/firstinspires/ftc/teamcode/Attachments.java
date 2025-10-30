package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
public class Attachments {
    // INSTANCE VARS
    // Params
    public static class Params {
        private double defaultShooterSpeed = 0.3; // 1.0 = 100%
        private double defaultMinRPM = 30;
    }
    public static Params params = new Params();

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

    // timer
    private ElapsedTime timer = new ElapsedTime();

    // THE ONLY CONSTRUCTOR
    public Attachments(HardwareMap hardwareMap) {
        // initialize shooters
        outtakeLeft = hardwareMap.get(DcMotorEx.class, "outtakeLeft");
        outtakeLeft.setDirection(DcMotorEx.Direction.FORWARD); // direction done
        outtakeLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outtakeLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftTPR = outtakeLeft.getMotorType().getTicksPerRev();

        outtakeRight = hardwareMap.get(DcMotorEx.class, "outtakeRight");
        outtakeRight.setDirection(DcMotorEx.Direction.REVERSE); // direction done
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
     * @param targetRPM the desired RPM for shooters to spin up before firing (recommended for m1: 95 RPM)
     * @return returns false once both motors reach targetRPM
     */
    public Action spinUp(double targetRPM) {
        return new Action() {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    outtakeLeft.setPower(params.defaultShooterSpeed);
                    outtakeRight.setPower(params.defaultShooterSpeed);
                    initialized = true;
                }
                leftRPM = 60 * (outtakeLeft.getVelocity() / leftTPR);
                rightRPM = 60 * (outtakeRight.getVelocity() / rightTPR);
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
     * @return returns false after shooter, progression, and intake run for durationSeconds seconds
     */
    public Action fireArtifact(double durationSeconds, double targetRPM) {
        return new Action() {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    // continue outtaking
                    outtakeLeft.setPower(params.defaultShooterSpeed);
                    outtakeRight.setPower(params.defaultShooterSpeed);

                    // reset/start timer
                    timer.reset();
                    initialized = true;
                }

                // constantly update progression & intake
                leftRPM = 60 * (outtakeLeft.getVelocity() / leftTPR);
                rightRPM = 60 * (outtakeRight.getVelocity() / rightTPR);
                canShoot = (leftRPM > targetRPM && rightRPM > targetRPM);
                if (canShoot) {
                    progression.setPower(1);
                    intake.setPower(1);
                } else {
                    progression.setPower(0);
                    intake.setPower(0);
                }

                double elapsed = timer.seconds();
                packet.put("Shooting timer", elapsed); // telemetry

                if (elapsed >= durationSeconds) { // check if timer is done
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

    /** Fires one artifact for s seconds
     *
     * @param durationSeconds (double) number of seconds you want to continuously shoot
     * @return returns false after shooter, progression, and intake run for durationSeconds seconds
     */
    public Action fireArtifact(double durationSeconds, double targetRPM, double targetSpeed) {
        return new Action() {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    // continue outtaking
                    outtakeLeft.setPower(targetSpeed);
                    outtakeRight.setPower(targetSpeed);

                    // reset/start timer
                    timer.reset();
                    initialized = true;
                }

                // constantly update progression & intake
                leftRPM = 60 * (outtakeLeft.getVelocity() / leftTPR);
                rightRPM = 60 * (outtakeRight.getVelocity() / rightTPR);
                canShoot = (leftRPM > targetRPM && rightRPM > targetRPM);
                if (canShoot) {
                    progression.setPower(1);
                    intake.setPower(1);
                } else {
                    progression.setPower(0);
                    intake.setPower(0);
                }

                double elapsed = timer.seconds();
                packet.put("Shooting timer", elapsed); // telemetry

                if (elapsed >= durationSeconds) { // check if timer is done
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

    /**
     * @param durationSeconds The amount of seconds to run intake
     * @return returns false after intake runs for durationSeconds seconds
     */
    public Action intake(double durationSeconds) {
        return new Action() {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    intake.setPower(1); // turn on intake

                    timer.reset(); // reset/start timer
                    initialized = true;
                }

                double elapsed = timer.seconds();
                packet.put("Intaking", true);

                if (elapsed >= durationSeconds) { // check if timer is done
                    // stop all motors
                    progression.setPower(0);
                    intake.setPower(0);

                    outtakeLeft.setPower(0);
                    outtakeRight.setPower(0);

                    return false; // action finished
                }

                return true; // keep intaking until timer reaches target time
            }
        };
    }
}
